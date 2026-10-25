"""Étape 0 du plan : acquisition lidar fiable, publiée tour par tour.

Reprend la logique de lidar_polar.py (lecture dans un thread dédié avec sa boucle asyncio, détection
des tours, horodatage t_rx_ns de la bibliothèque) sous forme de classe réutilisable. Chaque tour
complet est déposé dans `rev_queue` ; un traitement (clusters, EKF) le consomme dans un autre thread,
sans jamais interrompre la lecture du port série.

Un tour est « complet » s'il commence près de 0°, finit près de 360° et contient assez de points. Les
autres sont rejetés et comptés : un tour tronqué donnerait des clusters faux. Le tout premier tour
(qui commence au milieu d'une rotation) n'est pas compté du tout : on se synchronise sur le premier 0°.
"""
import asyncio
import collections
import importlib
import logging
import queue
import threading
import time
from collections import Counter
from dataclasses import dataclass

import numpy as np

from .lidar_clusters import POINT_DTYPE
from .timebase import ClockAligner


@dataclass
class Revolution:
    """Conteneur des mesures valides regroupées pour un tour complet du lidar."""
    number: int
    points: np.ndarray      # POINT_DTYPE, mesures valides seulement (distance non nulle)
    n_raw: int              # nombre de points reçus, valides ou non
    t_start_ns: int         # instants (epoch) du premier et du dernier point du tour
    t_end_ns: int


class _ErrorCounter(logging.Handler):
    """Compte les WARNING/ERROR de la bibliothèque (trames décalées...) au lieu de les afficher."""

    def __init__(self):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        super().__init__(level=logging.WARNING)
        self.count = 0

    def emit(self, record):
        """Enregistre un événement de diagnostic dans le journal."""
        self.count += 1


def _import_rplidar():
    """Retourne la classe RPLidar. On essaie d'abord `lidar.rplidarc1` (la bibliothèque rangée dans lidar/),
    puis `rplidarc1`. Un nom qui s'importe mais ne contient pas RPLidar est ignoré : c'est typiquement un
    ancien dossier `rplidarc1/` sans __init__.py, que Python prend pour un « namespace package » vide
    (rplidarc1.__file__ vaut alors None)."""
    tried = []
    for name in ("lidar.rplidarc1", "rplidarc1"):
        try:
            mod = importlib.import_module(name)
        except ModuleNotFoundError as e:
            if e.name not in (name, name.split(".")[0]):   # un module manquant DANS la lib (pyserial...) : on le dit
                raise
            tried.append(f"{name} (introuvable)")
            continue
        if hasattr(mod, "RPLidar"):
            return mod.RPLidar
        tried.append(f"{name} (importé depuis {getattr(mod, '__file__', None) or list(getattr(mod, '__path__', []))} "
                     "mais sans classe RPLidar)")
    raise ImportError("Bibliothèque lidar introuvable. Essayé : " + " ; ".join(tried))


class LidarAcquisition:
    """Acquiert les mesures du lidar en arrière-plan et publie uniquement des tours complets."""
    def __init__(self, port="/dev/ttyUSB0", baud=460800, queue_size=20,
                 min_raw_points=300, max_first_angle_deg=10.0, min_last_angle_deg=350.0):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.port, self.baud = port, baud
        self.rev_queue = queue.Queue(maxsize=queue_size)
        self.clock = ClockAligner("lidar", unit="ns")     # t_rx_ns de la bibliothèque -> heure epoch
        self._min_raw = min_raw_points
        self._max_first = max_first_angle_deg
        self._min_last = min_last_angle_deg
        # compteurs cumulés (critères de l'étape 0)
        self.revs_seen = 0         # tours vus (hors tour de démarrage)
        self.revs_published = 0    # ... publiés dans rev_queue
        self.revs_rejected = 0     # ... rejetés (incomplets)
        self.revs_dropped = 0      # tours publiés mais perdus car la file était pleine (consommateur trop lent)
        self.points = 0
        self.raw_total = 0         # somme des points reçus par tour vu (sert à calculer le nombre moyen de points/tour)
        self.reject_reasons = Counter()   # pourquoi les tours sont rejetés : few_points, start_angle, end_angle, empty
        self.last_rev = None       # (points, premier angle, dernier angle) du dernier tour vu, rejeté ou non
        self.error = None          # exception éventuelle du thread
        self._errors = _ErrorCounter()
        self._lib_logger_name = None
        self._lidar = None
        self._loop = None
        self._thread = None          # thread de LECTURE (boucle asyncio de la bibliothèque + vidage de sa file)
        self._seg_thread = None      # thread de DÉCOUPAGE en tours (conversion des horloges, listes de points)
        self._seg_stop = threading.Event()
        self._raw = collections.deque()   # passage de relais lecture -> découpage (append/popleft : thread-safe)
        self._snap = (time.perf_counter(), 0, 0, 0)    # (instant, tours vus, points, points par tour cumulés)

    # ------------------------------------------------------------------ cycle de vie
    def start(self):
        """Démarre le traitement ou le thread associé à cet objet."""
        RPLidar = _import_rplidar()          # import ici : la bibliothèque n'est requise que pour acquérir

        self._lidar = RPLidar(self.port, self.baud)       # ouvre le port série + test de santé
        # Le compteur d'erreurs doit être accroché au logger du PACKAGE de la bibliothèque, dont le nom dépend
        # de l'endroit où elle est rangée : "rplidarc1" ou "lidar.rplidarc1" (déduit du nom du module de la
        # classe : "lidar.rplidarc1.scanner" -> "lidar.rplidarc1"). Les loggers fils (…protocol) remontent jusque-là.
        self._lib_logger_name = RPLidar.__module__.rsplit(".", 1)[0]
        lib_logger = logging.getLogger(self._lib_logger_name)
        lib_logger.setLevel(logging.WARNING)
        lib_logger.addHandler(self._errors)
        lib_logger.propagate = False
        self._thread = threading.Thread(target=self._thread_main, daemon=True, name="lidar-read")
        self._seg_thread = threading.Thread(target=self._seg_main, daemon=True, name="lidar-segment")
        self._seg_thread.start()
        self._thread.start()

    def stop(self):
        """Demande l’arrêt du traitement et attend sa terminaison si nécessaire."""
        if self._lidar is None:
            return
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._lidar.stop_event.set)   # asyncio.Event n'est pas thread-safe
        else:
            self._lidar.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        self._seg_stop.set()                 # le découpage finit de vider ce qui reste, puis s'arrête
        if self._seg_thread is not None:
            self._seg_thread.join(timeout=3)
        time.sleep(0.3)
        self._lidar.reset()                  # arrête aussi le moteur
        lib_logger = logging.getLogger(self._lib_logger_name)
        lib_logger.removeHandler(self._errors)
        lib_logger.propagate = True
        self._lidar = None

    @property
    def alive(self) -> bool:
        """Indique si le traitement associé est toujours actif."""
        return (self._thread is not None and self._thread.is_alive()
                and self._seg_thread is not None and self._seg_thread.is_alive())

    def snapshot(self) -> dict:
        """Compteurs cumulés + cadence mesurée depuis le dernier appel (tours/s et points/s)."""
        now = time.perf_counter()
        t0, seen0, pts0, raw0 = self._snap
        dt = max(now - t0, 1e-9)
        n_revs = self.revs_seen - seen0
        self._snap = (now, self.revs_seen, self.points, self.raw_total)
        return {
            "freq_hz": n_revs / dt,
            "pts_per_s": (self.points - pts0) / dt,
            # Points par tour : ~500 attendus avec un C1 à 10 Hz et 5000 points/s. Bien moins = des points
            # perdus entre le lidar et Python (c'est la cause à chercher quand tous les tours sont rejetés).
            "pts_per_rev": (self.raw_total - raw0) / n_revs if n_revs else 0.0,
            "reject_reasons": dict(self.reject_reasons),
            "last_rev": self.last_rev,
            "revs_seen": self.revs_seen,
            "revs_published": self.revs_published,
            "revs_rejected": self.revs_rejected,
            "revs_dropped": self.revs_dropped,
            "lib_errors": self._errors.count,
        }

    # ------------------------------------------------------------------ thread de lecture
    # Règle : la réception des trames série est SEULE dans son thread. Les ~500 points d'un tour arrivent en
    # rafale ; si ce même thread doit en plus convertir des horloges et construire des listes, il ne relit pas
    # le port à temps, le tampon USB déborde et des trames sont perdues (tours à moitié vides). Ici ce thread ne
    # fait que : lire (bibliothèque) + déplacer les points de la file asyncio vers une deque (quelques µs/point).
    def _thread_main(self):
        async def main():
            """Configure puis exécute le scénario principal du programme."""
            self._loop = asyncio.get_running_loop()
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self._drain())
                tg.create_task(self._lidar.simple_scan())     # lecture du port série (bibliothèque)

        try:
            asyncio.run(main())
        except BaseException as e:             # remontée au thread principal via .error
            self.error = e

    async def _drain(self):
        q, stop = self._lidar.output_queue, self._lidar.stop_event
        get, put = q.get_nowait, self._raw.append
        while not stop.is_set():
            try:
                while True:
                    put(get())
            except asyncio.QueueEmpty:
                pass
            await asyncio.sleep(0.002)         # cède la main à la lecture série

    # ------------------------------------------------------------------ thread de découpage en tours
    def _seg_main(self):
        try:
            self._segment()
        except BaseException as e:
            self.error = e

    def _segment(self):
        raw = self._raw
        cur = []                               # points valides du tour en cours
        n_raw = 0
        first_a = last_a = None
        t_first = t_last = 0
        prev_angle = None
        synced = False                         # False tant qu'on n'a pas vu le premier passage 360° -> 0°

        while True:
            if not raw:
                if self._seg_stop.is_set():
                    return
                time.sleep(0.005)
                continue
            while raw:
                p = raw.popleft()
                a, d = p["a_deg"], p["d_mm"]
                t = self.clock.epoch_ns(p["t_rx_ns"])
                self.points += 1
                # Nouveau tour = l'angle retombe de ~360° à ~0° (par ex. 359,6° -> 0,3° : écart de 359 > 180).
                # Un petit recul dû au bruit (359,6° -> 359,3°) ne déclenche rien.
                wrap = prev_angle is not None and prev_angle - a > 180
                prev_angle = a
                if wrap:                       # le point courant ouvre un nouveau tour
                    if synced:
                        self._finish(cur, n_raw, first_a, last_a, t_first, t_last)
                    synced, cur, n_raw, first_a = True, [], 0, None
                if not synced:
                    continue
                if first_a is None:
                    first_a, t_first = a, t
                n_raw += 1
                last_a, t_last = a, t
                if d is not None and d > 0:
                    cur.append((a, d, p["q"], t))

    def _finish(self, cur, n_raw, first_a, last_a, t_first, t_last):
        # Un tour normal du C1 : ~500 points, premier à ~0,3°, dernier à ~359,7°. Un tour qui commence à 120°
        # (octets perdus, démarrage en cours de rotation) ou qui a moins de 300 points est rejeté et compté :
        # il donnerait des clusters tronqués, donc de fausses associations.
        self.revs_seen += 1
        self.raw_total += n_raw
        self.last_rev = (n_raw, first_a, last_a)
        # On note TOUTES les raisons du rejet (un tour peut en cumuler plusieurs) : sans cela, « 219 rejetés »
        # ne dit pas s'il manque des points, si le tour démarre trop tard ou s'il finit trop tôt.
        reasons = []
        if n_raw < self._min_raw:
            reasons.append("few_points")
        if first_a > self._max_first:
            reasons.append("start_angle")
        if last_a < self._min_last:
            reasons.append("end_angle")
        if not cur:
            reasons.append("empty")
        if reasons:
            self.revs_rejected += 1
            self.reject_reasons.update(reasons)
            return
        rev = Revolution(self.revs_published, np.array(cur, dtype=POINT_DTYPE), n_raw, t_first, t_last)
        try:
            self.rev_queue.put_nowait(rev)
        except queue.Full:                     # consommateur trop lent : on garde le plus récent
            try:
                self.rev_queue.get_nowait()
            except queue.Empty:
                pass
            self.rev_queue.put_nowait(rev)
            self.revs_dropped += 1
        self.revs_published += 1

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
import logging
import queue
import threading
import time
from dataclasses import dataclass

import numpy as np

from .lidar_clusters import POINT_DTYPE
from .timebase import ClockAligner


@dataclass
class Revolution:
    number: int
    points: np.ndarray      # POINT_DTYPE, mesures valides seulement (distance non nulle)
    n_raw: int              # nombre de points reçus, valides ou non
    t_start_ns: int         # instants (epoch) du premier et du dernier point du tour
    t_end_ns: int


class _ErrorCounter(logging.Handler):
    """Compte les WARNING/ERROR de la bibliothèque (trames décalées...) au lieu de les afficher."""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.count = 0

    def emit(self, record):
        self.count += 1


class LidarAcquisition:
    def __init__(self, port="/dev/ttyUSB0", baud=460800, queue_size=20,
                 min_raw_points=300, max_first_angle_deg=10.0, min_last_angle_deg=350.0):
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
        self.error = None          # exception éventuelle du thread
        self._errors = _ErrorCounter()
        self._lidar = None
        self._loop = None
        self._thread = None
        self._snap = (time.perf_counter(), 0, 0)

    # ------------------------------------------------------------------ cycle de vie
    def start(self):
        from rplidarc1 import RPLidar        # import ici : la bibliothèque n'est requise que pour acquérir

        self._lidar = RPLidar(self.port, self.baud)       # ouvre le port série + test de santé
        lib_logger = logging.getLogger("rplidarc1")
        lib_logger.setLevel(logging.WARNING)
        lib_logger.addHandler(self._errors)
        lib_logger.propagate = False
        self._thread = threading.Thread(target=self._thread_main, daemon=True, name="lidar-acquisition")
        self._thread.start()

    def stop(self):
        if self._lidar is None:
            return
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._lidar.stop_event.set)   # asyncio.Event n'est pas thread-safe
        else:
            self._lidar.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        time.sleep(0.3)
        self._lidar.reset()                  # arrête aussi le moteur
        lib_logger = logging.getLogger("rplidarc1")
        lib_logger.removeHandler(self._errors)
        lib_logger.propagate = True
        self._lidar = None

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def snapshot(self) -> dict:
        """Compteurs cumulés + cadence mesurée depuis le dernier appel (tours/s et points/s)."""
        now = time.perf_counter()
        t0, seen0, pts0 = self._snap
        dt = max(now - t0, 1e-9)
        self._snap = (now, self.revs_seen, self.points)
        return {
            "freq_hz": (self.revs_seen - seen0) / dt,
            "pts_per_s": (self.points - pts0) / dt,
            "revs_seen": self.revs_seen,
            "revs_published": self.revs_published,
            "revs_rejected": self.revs_rejected,
            "revs_dropped": self.revs_dropped,
            "lib_errors": self._errors.count,
        }

    # ------------------------------------------------------------------ thread d'acquisition
    def _thread_main(self):
        async def main():
            self._loop = asyncio.get_running_loop()
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self._consume())
                tg.create_task(self._lidar.simple_scan())     # lecture du port série (bibliothèque)

        try:
            asyncio.run(main())
        except BaseException as e:             # remontée au thread principal via .error
            self.error = e

    async def _consume(self):
        q, stop = self._lidar.output_queue, self._lidar.stop_event
        cur = []                               # points valides du tour en cours
        n_raw = 0
        first_a = last_a = None
        t_first = t_last = 0
        prev_angle = None
        synced = False                         # False tant qu'on n'a pas vu le premier passage 360° -> 0°

        while not stop.is_set():
            while True:
                try:
                    p = q.get_nowait()
                except asyncio.QueueEmpty:
                    break
                a, d = p["a_deg"], p["d_mm"]
                t = self.clock.epoch_ns(p["t_rx_ns"])
                self.points += 1
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
            await asyncio.sleep(0.005)         # cède la main à la lecture série

    def _finish(self, cur, n_raw, first_a, last_a, t_first, t_last):
        self.revs_seen += 1
        if n_raw < self._min_raw or first_a > self._max_first or last_a < self._min_last or not cur:
            self.revs_rejected += 1
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

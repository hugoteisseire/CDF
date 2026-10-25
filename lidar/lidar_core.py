#!/usr/bin/env python3
"""
lidar_core.py - Acquisition RPLidar C1 et mise a disposition des donnees.

ARCHITECTURE (3 etages, chacun isole des autres)

  [thread "lidar-acq"]  lecture serie + decoupe en tours + horodatage
          |  RawScan  (un par tour, tableaux numpy en lecture seule)
          v
  [thread "lidar-proc"] capping + detection des clusters + ajustement du poteau
          |  ProcessedScan
          v
  Hub  --> Subscription  (un tampon borne PAR consommateur : EKF, trace, log...)
       --> latest()      (dernier tour, sans file d'attente : pour l'affichage)

Regles qui rendent l'ensemble robuste :
  * Le thread d'acquisition ne fait RIEN d'autre que lire, decouper et publier.
    Il ne calcule pas, ne dessine pas, ne peut jamais etre bloque par un consommateur.
  * Un consommateur lent perd ses PROPRES vieux tours (compteur `dropped`),
    il ne ralentit ni l'acquisition ni les autres consommateurs.
  * Les tours publies sont immuables (tableaux numpy non modifiables) :
    on les partage entre threads sans copie ni verrou.
  * Chaque point a son instant de mesure (perf_counter). Indispensable pour l'EKF :
    un tour dure 100 ms et le robot bouge pendant ce temps.

CONVENTIONS
  * Le lidar tourne dans le sens horaire, angle 0..360 deg (`angle_deg`, "repere lidar").
  * Repere robot : x devant, y a gauche, angle phi_rad positif dans le sens
    anti-horaire, en radians, distances en metres. phi = -angle (+ decalage de montage).
  * Le lidar est suppose au centre du repere robot. Un decalage de montage se
    traite dans le modele de mesure de l'EKF.
  * Les horodatages sont en time.perf_counter() (secondes, horloge monotone).
"""

from __future__ import annotations

import asyncio
import collections
import logging
import math
import threading
import time
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class LidarConfig:
    # --- repere robot
    """Paramètres de communication, de filtrage et de traitement du lidar."""
    clockwise: bool = True  # le RPLidar tourne dans le sens horaire
    mount_yaw_deg: float = 0.0  # direction du 0 deg lidar par rapport a l'avant du robot (anti-horaire +)

    # --- capping (rejet des points hors zone utile)
    rmin_mm: float = 100.0  # trop pres : structure du robot, bruit
    rmax_mm: float = 3000.0  # la diagonale du terrain fait 2830 mm
    blind_sectors_deg: tuple = ()  # ((a0, a1), ...) angles lidar ignores (mat, cable...). a0 > a1 : passe par 0

    # --- detection des clusters
    break_mm: float = 120.0  # ecart entre deux points voisins au-dela duquel on coupe le cluster
    max_gap_deg: float = 1.6  # trou angulaire max (2 echantillons = 1,44 deg a 0,72 deg de resolution)
    min_pts: int = 3  # un poteau a 2,8 m ne renvoie que ~3 points
    max_width_mm: float = 160.0  # corde max d'un poteau de 100 mm, avec marge pour le bruit
    pole_radius_mm: float = 50.0

    # --- acquisition
    latency_s: float = 0.0  # latence du lidar (serie + lib), a calibrer ; retranchee des horodatages
    poll_s: float = 0.002  # periode de vidage de la file de la lib
    min_samples_per_rev: int = 10  # echantillons recus (valides + sans mesure) sous lesquels un tour est rejete
    max_samples_per_rev: int = 3000
    period_range_s: tuple = (0.05, 0.3)  # un tour hors de cette plage est rejete (trames decalees, arret)

    # --- diffusion
    raw_queue: int = 16  # tampon entre acquisition et traitement


# --------------------------------------------------------------------------- #
# Structures de donnees (immuables)
# --------------------------------------------------------------------------- #
def _frozen(a: np.ndarray) -> np.ndarray:
    a.setflags(write=False)
    return a


@dataclass(frozen=True)
class RawScan:
    """Tour brut immuable contenant les points et leurs horodatages de mesure."""
    seq: int  # numero de tour, continu : un trou = tours perdus
    t_start: float  # debut du tour (perf_counter)
    t_end: float
    angle_deg: np.ndarray  # (N,) 0..360, repere lidar, croissant
    dist_mm: np.ndarray  # (N,)
    t: np.ndarray  # (N,) instant de chaque point (perf_counter)
    n_invalid: int  # points sans mesure ecartes a l'acquisition

    @property
    def period(self) -> float:
        """Retourne la durée estimée d’un tour de lidar."""
        return self.t_end - self.t_start


@dataclass(frozen=True)
class Cluster:
    """Représente un groupe de points lidar correspondant potentiellement à un objet."""
    phi_rad: float  # direction du CENTRE du poteau, repere robot (anti-horaire +)
    r_m: float  # distance lidar -> CENTRE du poteau (rayon deja pris en compte)
    a_deg: float  # meme direction, en angle lidar (pour l'affichage polaire)
    t: float  # instant moyen de mesure (perf_counter)
    n: int  # nombre de points
    width_m: float  # corde entre premier et dernier point
    r_front_m: float  # distance minimale mesuree (face avant)
    rms_m: float  # residu de l'ajustement de cercle


@dataclass(frozen=True)
class ProcessedScan:
    """Résultat immuable du traitement d’un tour brut."""
    raw: RawScan
    angle_deg: np.ndarray  # (M,) points conserves apres capping, repere lidar
    phi_rad: np.ndarray  # (M,) repere robot
    r_m: np.ndarray  # (M,)
    t: np.ndarray  # (M,)
    label: np.ndarray  # (M,) numero de cluster, -1 = hors cluster
    clusters: tuple  # tuple de Cluster, tries par angle lidar
    n_capped: int  # points ecartes par le capping
    proc_ms: float  # duree du traitement

    @property
    def seq(self) -> int:
        """Retourne l’identifiant séquentiel du tour."""
        return self.raw.seq


# --------------------------------------------------------------------------- #
# Capping
# --------------------------------------------------------------------------- #
def cap_mask(angle_deg: np.ndarray, dist_mm: np.ndarray, cfg: LidarConfig) -> np.ndarray:
    """Masque des points a CONSERVER : distance dans [rmin, rmax], hors secteurs aveugles.

    Les points hors plage sont ecartes, pas ramenes a rmax : un point force a rmax
    formerait un faux cluster sur le cercle de rayon rmax.
    """
    keep = (dist_mm >= cfg.rmin_mm) & (dist_mm <= cfg.rmax_mm)
    for a0, a1 in cfg.blind_sectors_deg:
        a0, a1 = a0 % 360.0, a1 % 360.0
        if a0 <= a1:
            blind = (angle_deg >= a0) & (angle_deg <= a1)
        else:
            blind = (angle_deg >= a0) | (angle_deg <= a1)
        keep &= ~blind
    return keep


# --------------------------------------------------------------------------- #
# Detection des clusters
# --------------------------------------------------------------------------- #
def _pole_range(D, d, radius):
    """Distance mesuree le long d'un faisceau d'ecart angulaire d (rad) par rapport a
    l'axe lidar -> centre d'un poteau de rayon `radius` situe a la distance D."""
    s = np.clip(D * np.sin(d), -radius, radius)
    return D * np.cos(d) - np.sqrt(radius * radius - s * s)


def fit_pole(phi, r, radius, iters: int = 6):
    """Direction et distance du CENTRE d'un poteau de rayon connu. phi en rad, r et radius en mm.

    Direction : moyenne des angles des points (ecart-type ~0,2 deg, soit 3 a 9 mm).
    Distance : moindres carres sur la distance mesuree le long de chaque faisceau.
    On n'ajuste PAS la distance geometrique point-cercle : avec un bruit (30 mm) du meme
    ordre que le rayon (50 mm), les points du bord ont toujours un residu positif et le
    centre est tire de ~16 mm vers le lidar. Verifie en simulation : biais < 3 mm ici.
    Retourne (psi_rad, D_mm, rms_mm).
    """
    dphi = np.remainder(phi - phi[0] + math.pi, 2.0 * math.pi) - math.pi  # sans souci de saut +-pi
    dpsi = float(np.mean(dphi))
    d = dphi - dpsi
    D = float(np.mean(r)) + radius
    h = 0.5
    for _ in range(iters):
        e = r - _pole_range(D, d, radius)
        j = (_pole_range(D + h, d, radius) - _pole_range(D - h, d, radius)) / (2.0 * h)
        jj = float(j @ j)
        if jj < 1e-9:
            break
        D += float(j @ e) / jj
    rms = float(np.sqrt(np.mean((r - _pole_range(D, d, radius)) ** 2)))
    psi = math.remainder(phi[0] + dpsi, 2.0 * math.pi)
    return psi, D, rms


def find_clusters(angle_deg, dist_mm, t, cfg: LidarConfig):
    """Detecte les poteaux dans un tour deja cappe (points tries par angle lidar croissant).

    Retourne (clusters, labels) ; labels[i] = numero de cluster du point i ou -1.
    Un cluster est une suite de points voisins (ecart < break_mm et trou angulaire
    < max_gap_deg) de 3 points ou plus, de corde <= max_width_mm. Un poteau a cheval
    sur la coupure 360 -> 0 est recolle.
    """
    n = len(angle_deg)
    labels = np.full(n, -1, dtype=int)
    if n < cfg.min_pts:
        return [], labels

    def euclid(r0, r1, da_deg):
        """Calcule la distance euclidienne entre deux mesures polaires."""
        return np.sqrt(np.maximum(r0 * r0 + r1 * r1 - 2.0 * r0 * r1 * np.cos(np.radians(da_deg)), 0.0))

    da = np.diff(angle_deg)
    cut = (euclid(dist_mm[:-1], dist_mm[1:], da) > cfg.break_mm) | (da > cfg.max_gap_deg)
    starts = np.concatenate(([0], np.nonzero(cut)[0] + 1))
    ends = np.concatenate((starts[1:], [n]))
    segs = [np.arange(s, e) for s, e in zip(starts, ends)]

    # raccord de la coupure 360 -> 0
    da_wrap = angle_deg[0] + 360.0 - angle_deg[-1]
    joined = not (euclid(dist_mm[-1], dist_mm[0], da_wrap) > cfg.break_mm or da_wrap > cfg.max_gap_deg)
    wrap_idx = None
    if joined and len(segs) >= 2:
        wrap_idx = len(segs) - 1
        segs[0] = np.concatenate((segs[-1], segs[0]))
        segs.pop()

    sign = -1.0 if cfg.clockwise else 1.0
    yaw = math.radians(cfg.mount_yaw_deg)
    clusters = []
    for k, idx in enumerate(segs):
        if len(idx) < cfg.min_pts:
            continue
        a = angle_deg[idx].astype(float)
        if k == 0 and wrap_idx is not None:  # points de fin de tour : angles ramenes a negatif
            a = np.where(a > 180.0, a - 360.0, a)
        r = dist_mm[idx].astype(float)
        phi = sign * np.radians(a) + yaw
        x, y = r * np.cos(phi), r * np.sin(phi)
        width = math.hypot(x[-1] - x[0], y[-1] - y[0])
        if width > cfg.max_width_mm:
            continue
        phi_c, d_c, rms = fit_pole(phi, r, cfg.pole_radius_mm)
        a_c = ((phi_c - yaw) / sign) * 180.0 / math.pi % 360.0
        labels[idx] = len(clusters)
        clusters.append(
            Cluster(
                phi_rad=phi_c,
                r_m=d_c / 1000.0,
                a_deg=a_c,
                t=float(np.mean(t[idx])),
                n=int(len(idx)),
                width_m=width / 1000.0,
                r_front_m=float(np.min(r)) / 1000.0,
                rms_m=rms / 1000.0,
            )
        )
    if not clusters:
        return [], labels
    order = sorted(range(len(clusters)), key=lambda i: clusters[i].a_deg)
    remap = np.full(len(clusters), -1, dtype=int)
    for new, old in enumerate(order):
        remap[old] = new
    labels = np.where(labels >= 0, remap[np.maximum(labels, 0)], -1)
    return [clusters[i] for i in order], labels


def process_scan(raw: RawScan, cfg: LidarConfig) -> ProcessedScan:
    """Filtre un tour brut et en extrait les clusters et les caractéristiques utiles."""
    t0 = time.perf_counter()
    keep = cap_mask(raw.angle_deg, raw.dist_mm, cfg)
    a, d, t = raw.angle_deg[keep], raw.dist_mm[keep], raw.t[keep]
    clusters, labels = find_clusters(a, d, t, cfg)
    sign = -1.0 if cfg.clockwise else 1.0
    phi = sign * np.radians(a) + math.radians(cfg.mount_yaw_deg)
    phi = (phi + math.pi) % (2.0 * math.pi) - math.pi
    return ProcessedScan(
        raw=raw,
        angle_deg=_frozen(a),
        phi_rad=_frozen(phi),
        r_m=_frozen(d / 1000.0),
        t=_frozen(t),
        label=_frozen(labels),
        clusters=tuple(clusters),
        n_capped=int(len(raw.angle_deg) - len(a)),
        proc_ms=1000.0 * (time.perf_counter() - t0),
    )


# --------------------------------------------------------------------------- #
# Diffusion : un tampon borne par consommateur, jamais bloquant pour l'emetteur
# --------------------------------------------------------------------------- #
class Closed(Exception):
    """Le flux est termine (arret ou erreur du thread producteur)."""


class Subscription:
    """File d’abonnement bornée qui isole un consommateur des autres consommateurs."""
    def __init__(self, maxlen: int):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self._dq = collections.deque()
        self._max = maxlen
        self._cv = threading.Condition()
        self._closed = False
        self.dropped = 0  # tours perdus parce que ce consommateur etait trop lent

    def _push(self, item) -> None:
        with self._cv:
            if len(self._dq) >= self._max:
                self._dq.popleft()
                self.dropped += 1
            self._dq.append(item)
            self._cv.notify()

    def _close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()

    def get(self, timeout: float | None = None):
        """Prochain tour dans l'ordre, ou None si rien n'est arrive avant `timeout`.
        Leve Closed quand le flux est termine et le tampon vide."""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cv:
            while not self._dq:
                if self._closed:
                    raise Closed
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._cv.wait(remaining)
            return self._dq.popleft()


class Hub:
    """Diffuse les résultats vers plusieurs abonnés sans imposer leur vitesse de traitement."""
    def __init__(self):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self._lock = threading.Lock()
        self._subs: list = []
        self._latest = None
        self._closed = False

    def subscribe(self, maxlen: int = 8) -> Subscription:
        """Crée un abonnement avec une file de taille limitée."""
        sub = Subscription(maxlen)
        with self._lock:
            self._subs.append(sub)
            if self._closed:
                sub._close()
        return sub

    def unsubscribe(self, sub: Subscription) -> None:
        """Retire un abonnement du diffuseur."""
        with self._lock:
            if sub in self._subs:
                self._subs.remove(sub)

    def publish(self, item) -> None:
        """Publie un résultat auprès des abonnés actifs."""
        with self._lock:
            self._latest = item
            subs = list(self._subs)
        for s in subs:
            s._push(item)

    def latest(self):
        """Retourne le résultat le plus récent sans attendre une file de tours."""
        return self._latest  # simple lecture d'une reference : atomique

    def close(self) -> None:
        """Ferme proprement la ressource ou la fenêtre associée."""
        with self._lock:
            self._closed = True
            subs = list(self._subs)
        for s in subs:
            s._close()


# --------------------------------------------------------------------------- #
# Etage 1 : acquisition (thread dedie, boucle asyncio dediee)
# --------------------------------------------------------------------------- #
class _ErrorCounter(logging.Handler):
    """Compte les WARNING/ERROR de la bibliotheque (trames mal alignees...)."""

    def __init__(self):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        super().__init__(level=logging.WARNING)
        self.count = 0

    def emit(self, record):
        """Enregistre un événement de diagnostic dans le journal."""
        self.count += 1


class Acquisition:
    """Lit le lidar dans un thread dédié, découpe les tours et horodate les mesures."""
    def __init__(self, lidar, cfg: LidarConfig, out: Hub):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.lidar, self.cfg, self.out = lidar, cfg, out
        self.errors = _ErrorCounter()
        self.error: BaseException | None = None
        self.bad_revs = 0
        self.n_invalid = 0
        self._rev_ends: collections.deque = collections.deque(maxlen=20)
        self._last_pts = 0
        self._loop = None
        self._thread = threading.Thread(target=self._run, name="lidar-acq", daemon=True)

    def start(self) -> None:
        """Démarre le traitement ou le thread associé à cet objet."""
        lib_logger = logging.getLogger("rplidarc1")
        lib_logger.setLevel(logging.WARNING)
        lib_logger.addHandler(self.errors)
        lib_logger.propagate = False
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Demande l’arrêt du traitement et attend sa terminaison si nécessaire."""
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self.lidar.stop_event.set)
        else:
            self.lidar.stop_event.set()
        self._thread.join(timeout)

    def alive(self) -> bool:
        """Indique si le traitement associé est toujours actif."""
        return self._thread.is_alive()

    def rate_hz(self) -> float:
        """Retourne la fréquence de tours mesurée en hertz."""
        ends = list(self._rev_ends)
        if len(ends) < 2 or ends[-1] <= ends[0]:
            return 0.0
        return (len(ends) - 1) / (ends[-1] - ends[0])

    def _run(self) -> None:
        async def main() -> None:
            """Configure puis exécute le scénario principal du programme."""
            self._loop = asyncio.get_running_loop()
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self._consume())
                tg.create_task(self.lidar.simple_scan())

        try:
            asyncio.run(main())
        except BaseException as e:  # remonte au thread principal via .error
            self.error = e
        finally:
            self.out.close()

    async def _consume(self) -> None:
        cfg = self.cfg
        queue, stop = self.lidar.output_queue, self.lidar.stop_event
        cur_a: list = []
        cur_d: list = []
        last_angle = None
        t_prev = None  # instant du dernier saut d'angle (debut du tour en cours)
        t_last_drain = time.perf_counter()
        seq = 0
        invalid = 0

        while not stop.is_set():
            t_now = time.perf_counter()
            # Les points vides ci-dessous sont arrives au hasard entre deux vidages :
            # en moyenne a mi-chemin. C'est l'instant attribue au saut d'angle.
            t_event = t_now - 0.5 * (t_now - t_last_drain) - cfg.latency_s
            t_last_drain = t_now
            while True:
                try:
                    p = queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
                angle, dist = p["a_deg"], p["d_mm"]

                # Nouveau tour : l'angle repasse de ~360 a ~0 deg
                if last_angle is not None and last_angle - angle > 180:
                    if t_prev is not None:
                        period = t_event - t_prev
                        ok = (
                            cfg.min_samples_per_rev <= len(cur_a) + invalid <= cfg.max_samples_per_rev
                            and cfg.period_range_s[0] <= period <= cfg.period_range_s[1]
                        )
                        if ok:
                            a = np.asarray(cur_a, dtype=float)
                            self._rev_ends.append(t_event)
                            self._last_pts = len(a)
                            self.n_invalid += invalid
                            self.out.publish(
                                RawScan(
                                    seq=seq,
                                    t_start=t_prev,
                                    t_end=t_event,
                                    angle_deg=_frozen(a),
                                    dist_mm=_frozen(np.asarray(cur_d, dtype=float)),
                                    t=_frozen(t_prev + a * (period / 360.0)),
                                    n_invalid=invalid,
                                )
                            )
                            seq += 1
                        else:
                            self.bad_revs += 1
                    t_prev = t_event
                    cur_a, cur_d, invalid = [], [], 0
                last_angle = angle

                if dist is not None and dist > 0:
                    if len(cur_a) < cfg.max_samples_per_rev:
                        cur_a.append(angle)
                        cur_d.append(dist)
                else:
                    invalid += 1

            await asyncio.sleep(cfg.poll_s)


# --------------------------------------------------------------------------- #
# Etage 2 : traitement (thread dedie)
# --------------------------------------------------------------------------- #
class Processor:
    """Traite les tours bruts dans un thread séparé de l’acquisition."""
    def __init__(self, cfg: LidarConfig, src: Hub, out: Hub):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.cfg, self.out = cfg, out
        self._sub = src.subscribe(cfg.raw_queue)
        self.skipped = 0  # tours bruts perdus avant traitement (ne doit jamais augmenter)
        self.last_ms = 0.0
        self.error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name="lidar-proc", daemon=True)

    def start(self) -> None:
        """Démarre le traitement ou le thread associé à cet objet."""
        self._thread.start()

    def join(self, timeout: float = 3.0) -> None:
        """Attend la fin du thread de traitement."""
        self._thread.join(timeout)

    def alive(self) -> bool:
        """Indique si le traitement associé est toujours actif."""
        return self._thread.is_alive()

    def _run(self) -> None:
        try:
            while True:
                try:
                    raw = self._sub.get(0.5)
                except Closed:
                    break
                if raw is None:
                    continue
                self.skipped = self._sub.dropped
                ps = process_scan(raw, self.cfg)
                self.last_ms = ps.proc_ms
                self.out.publish(ps)
        except BaseException as e:
            self.error = e
        finally:
            self.out.close()


# --------------------------------------------------------------------------- #
# Facade
# --------------------------------------------------------------------------- #
class LidarService:
    """Point d'entree. Exemple :

        with LidarService(RPLidar(port, 460800)) as lidar:
            sub = lidar.subscribe()           # EKF : recoit TOUS les tours, dans l'ordre
            while True:
                scan = sub.get(timeout=0.5)   # ProcessedScan ou None
                ...
            # affichage : lidar.latest() (dernier tour, sans file)
    """

    def __init__(self, lidar, cfg: LidarConfig | None = None):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.lidar = lidar
        self.cfg = cfg or LidarConfig()
        self._raw = Hub()
        self._scans = Hub()
        self._acq = Acquisition(lidar, self.cfg, self._raw)
        self._proc = Processor(self.cfg, self._raw, self._scans)

    def start(self) -> "LidarService":
        """Démarre le traitement ou le thread associé à cet objet."""
        self._proc.start()  # le traitement s'abonne avant que les donnees arrivent
        self._acq.start()
        return self

    def stop(self) -> None:
        """Demande l’arrêt du traitement et attend sa terminaison si nécessaire."""
        self._acq.stop()
        self._proc.join()
        time.sleep(0.2)
        try:
            self.lidar.reset()
        except Exception:
            pass

    def subscribe(self, maxlen: int = 8) -> Subscription:
        """Crée un abonnement avec une file de taille limitée."""
        return self._scans.subscribe(maxlen)

    def unsubscribe(self, sub: Subscription) -> None:
        """Retire un abonnement du diffuseur."""
        self._scans.unsubscribe(sub)

    def latest(self):
        """Retourne le résultat le plus récent sans attendre une file de tours."""
        return self._scans.latest()

    def alive(self) -> bool:
        """Indique si le traitement associé est toujours actif."""
        return self._acq.alive()

    @property
    def error(self):
        """Retourne la dernière erreur rencontrée, le cas échéant."""
        return self._acq.error or self._proc.error

    def stats(self) -> dict:
        """Retourne les compteurs et mesures de performance du service."""
        last = self._scans.latest()
        return {
            "rev_hz": self._acq.rate_hz(),
            "pts_per_rev": self._acq._last_pts,
            "lib_errors": self._acq.errors.count,
            "bad_revs": self._acq.bad_revs,
            "invalid_pts": self._acq.n_invalid,
            "proc_skipped": self._proc.skipped,
            "proc_ms": self._proc.last_ms,
            "n_clusters": len(last.clusters) if last else 0,
            "seq": last.seq if last else -1,
        }

    def __enter__(self):
        """Démarre le service à l’entrée du gestionnaire de contexte."""
        return self.start()

    def __exit__(self, *exc):
        """Arrête le service à la sortie du gestionnaire de contexte."""
        self.stop()

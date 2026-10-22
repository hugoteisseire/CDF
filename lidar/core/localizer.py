"""Localisation par fusion de capteurs (étapes 0 à 3 du plan).

Un thread unique, séparé de la lecture du lidar, fait deux choses en boucle :
  1. lit l'odométrie (HolonomicBase.pose) et fait avancer l'EKF (étape 2) ;
  2. pour chaque tour de lidar publié par LidarAcquisition : capping, clusters, association aux
     poteaux (étape 3). La correction de la pose (étape 4) n'est pas encore branchée : on produit un
     rapport par tour, avec les écarts mesure - attendu.

Les trois fonctions d'entrée sont volontairement simples (pose_source, rev_queue, init_pose) pour
pouvoir tester sans robot.
"""
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .ekf import EkfParams, PoseEKF
from .lidar_clusters import ClusterParams, cap_points, find_clusters
from .pole_association import AssocParams, AssocResult, associate_poles
from .timebase import ClockAligner


@dataclass
class Report:
    rev_number: int
    t_s: float                 # instant de référence du tour (epoch)
    n_points: int              # mesures valides du tour
    n_capped: int              # ... après capping
    n_clusters: int
    result: AssocResult
    pose: tuple                # (x mm, y mm, θ rad) prédite par l'odométrie
    std: tuple                 # (σx mm, σy mm, σθ °)
    proc_ms: float             # temps de traitement du tour (doit rester << 100 ms)


class Localizer:
    def __init__(self, pose_source: Callable, rev_queue: queue.Queue, init_pose: tuple,
                 params: Optional[EkfParams] = None, assoc: Optional[AssocParams] = None,
                 cluster_params: Optional[ClusterParams] = None, poll_s: float = 0.005,
                 on_report: Optional[Callable] = None):
        """
        pose_source : fonction sans argument -> (x mm, y mm, θ rad, instant) ; HolonomicBase.pose.
                      Instant en secondes, epoch ou monotone (recalé automatiquement).
        rev_queue   : file des tours publiés par LidarAcquisition.
        init_pose   : (x mm, y mm, θ rad) du robot dans le repère terrain au démarrage.
        """
        self.ekf = PoseEKF(params)
        self._pose_source = pose_source
        self._q = rev_queue
        self._init_pose = init_pose
        self._assoc = assoc or AssocParams()
        self._cp = cluster_params or ClusterParams()
        self._poll_s = poll_s
        self._on_report = on_report
        self.odo_clock = ClockAligner("odometrie", unit="s")
        self._last_odo_t = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.revs_processed = 0
        self.revs_skipped = 0      # tours arrivés avant le premier échantillon d'odométrie
        self.error = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="localizer")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def status(self) -> dict:
        with self._lock:
            return {"ready": self.ekf.ready, "pose": self.ekf.pose, "std": self.ekf.std,
                    "revs_processed": self.revs_processed, "revs_skipped": self.revs_skipped}

    # ------------------------------------------------------------------ boucle
    def _run(self):
        try:
            while not self._stop.is_set():
                self._poll_odometry()
                self._process_revs()
                self._stop.wait(self._poll_s)
        except BaseException as e:
            self.error = e

    def _poll_odometry(self):
        x, y, th, t = self._pose_source()
        if t is None or t == self._last_odo_t:
            return
        self._last_odo_t = t
        ts = self.odo_clock.epoch_s(t)
        with self._lock:
            if not self.ekf.ready:
                self.ekf.reset(*self._init_pose, ts)
            self.ekf.predict_odometry(x, y, th, ts)

    def _process_revs(self):
        while True:
            try:
                rev = self._q.get_nowait()
            except queue.Empty:
                return
            if not self.ekf.ready:
                self.revs_skipped += 1
                continue
            t0 = time.perf_counter()
            capped = cap_points(rev.points, self._cp)
            clusters = find_clusters(capped, self._cp)
            t_ref = (rev.t_start_ns + rev.t_end_ns) / 2e9
            res = associate_poles(self.ekf, clusters, t_ref, self._assoc)
            report = Report(rev.number, t_ref, len(rev.points), len(capped), len(clusters), res,
                            self.ekf.pose, self.ekf.std, (time.perf_counter() - t0) * 1e3)
            self.revs_processed += 1
            if self._on_report is not None:
                self._on_report(report)

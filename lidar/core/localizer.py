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
from dataclasses import dataclass, field
from typing import Callable, Optional

from .ekf import EkfParams, PoseEKF
from .lidar_clusters import ClusterParams, cap_points, find_clusters
from .pole_association import AssocParams, AssocResult, associate_poles
from .timebase import ClockAligner


@dataclass
class Report:
    """Résultat de traitement d’un tour, avec les associations et les écarts de mesure."""
    rev_number: int
    t_s: float                 # instant de référence du tour (epoch)
    n_points: int              # mesures valides du tour
    n_capped: int              # ... après capping
    n_clusters: int
    result: AssocResult
    pose: tuple                # (x mm, y mm, θ rad) prédite par l'odométrie
    std: tuple                 # (σx mm, σy mm, σθ °)
    proc_ms: float             # temps de traitement du tour (doit rester << 100 ms)
    clusters: list = field(default_factory=list)   # clusters du tour (diagnostic : où le lidar voit des objets)


class Localizer:
    """Coordonne le traitement des tours, le suivi de pose et l’association des poteaux connus."""
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
        """Démarre le traitement ou le thread associé à cet objet."""
        self._thread = threading.Thread(target=self._run, daemon=True, name="localizer")
        self._thread.start()

    def stop(self):
        """Demande l’arrêt du traitement et attend sa terminaison si nécessaire."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def status(self) -> dict:
        """Retourne un instantané de l’état courant du traitement."""
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
        # t est l'instant de lecture des moteurs (horloge de l'odométrie) ; on le ramène à l'heure système
        # pour qu'il soit comparable aux horodatages du lidar.
        ts = self.odo_clock.epoch_s(t)
        with self._lock:
            if not self.ekf.ready:
                # Le filtre démarre au premier échantillon d'odométrie, à la pose donnée par l'utilisateur
                # (repère terrain). Ensuite il n'utilise plus que les déplacements de l'odométrie.
                self.ekf.reset(*self._init_pose, ts)
            self.ekf.predict_odometry(x, y, th, ts)

    def _process_revs(self):
        while True:
            try:
                rev = self._q.get_nowait()
            except queue.Empty:
                return
            if not self.ekf.ready:
                # Pas encore d'odométrie : sans pose de départ, on ne peut pas dire où chercher les poteaux.
                self.revs_skipped += 1
                continue
            t0 = time.perf_counter()
            capped = cap_points(rev.points, self._cp)
            clusters = find_clusters(capped, self._cp)
            # Milieu du tour (secondes) : sert seulement d'instant de référence pour afficher la mesure attendue
            # des poteaux sans candidat. Les clusters, eux, sont datés point par point.
            t_ref = (rev.t_start_ns + rev.t_end_ns) / 2e9
            res = associate_poles(self.ekf, clusters, t_ref, self._assoc)
            report = Report(rev.number, t_ref, len(rev.points), len(capped), len(clusters), res,
                            self.ekf.pose, self.ekf.std, (time.perf_counter() - t0) * 1e3, clusters)
            self.revs_processed += 1
            if self._on_report is not None:
                self._on_report(report)

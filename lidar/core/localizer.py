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
from .lidar_clusters import ClusterParams, cap_points, filter_points_in_field, find_clusters
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
    clusters: list = field(default_factory=list)   # clusters du tour (diagnostic : où le lidar voit des objets)


class Localizer:
    def __init__(self, pose_source: Callable, rev_queue: queue.Queue, init_pose: tuple,
                 params: Optional[EkfParams] = None, assoc: Optional[AssocParams] = None,
                 cluster_params: Optional[ClusterParams] = None, poll_s: float = 0.005,
                 on_report: Optional[Callable] = None, on_pose: Optional[Callable] = None):
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
        self._on_pose = on_pose
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
        # t est l'instant de lecture des moteurs (horloge de l'odométrie) ; on le ramène à l'heure système
        # pour qu'il soit comparable aux horodatages du lidar.
        ts = self.odo_clock.epoch_s(t)
        with self._lock:
            if not self.ekf.ready:
                # Le filtre démarre au premier échantillon d'odométrie, à la pose donnée par l'utilisateur
                # (repère terrain). Ensuite il n'utilise plus que les déplacements de l'odométrie.
                self.ekf.reset(*self._init_pose, ts)
            self.ekf.predict_odometry(x, y, th, ts)
            pose = self.ekf.pose

        # Callback après chaque nouvel échantillon d’odométrie accepté.
        # Il est hors du verrou pour ne pas bloquer les lectures concurrentes de l’état EKF.
        if self._on_pose is not None:
            self._on_pose(ts, pose)

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
            # Filtrage géométrique dans le repère terrain : les points d'impact hors du terrain
            # élargi de 200 mm ne participent ni au clustering ni à l'association, et ne peuvent
            # donc pas apparaître dans les clusters transmis à la visualisation.
            in_field = filter_points_in_field(
                capped,
                self.ekf.state_at,
                field_size_mm=self.ekf.p.field_size_mm,
                margin_mm=self.ekf.p.field_margin_mm,
                lidar_offset_mm=self.ekf.p.lidar_offset_mm,
                lidar_zero_deg=self.ekf.p.lidar_zero_deg,
            )
            clusters = find_clusters(in_field, self._cp)
            # Milieu du tour (secondes) : sert seulement d'instant de référence pour afficher la mesure attendue
            # des poteaux sans candidat. Les clusters, eux, sont datés point par point.
            t_ref = (rev.t_start_ns + rev.t_end_ns) / 2e9
            res = associate_poles(self.ekf, clusters, t_ref, self._assoc)
            # Étape 4 : correction séquentielle. Chaque mesure validée par l'association est
            # également soumise au gating statistique (chi² 99 %, 2 ddl) avant de corriger l'EKF.
            # La distance mesurée est ramenée au centre du poteau (le lidar voit sa surface).
            accepted = rejected = 0
            for match in res.matches:
                if match.status != "matched" or match.cluster is None:
                    continue
                ok, _ = self.ekf.correct_pole(
                    match.pole,
                    match.cluster.dist_mm + self.ekf.p.pole_radius_mm,
                    match.cluster.angle_deg,
                )
                if ok:
                    accepted += 1
                else:
                    rejected += 1
            report = Report(rev.number, t_ref, len(rev.points), len(capped), len(clusters), res,
                            self.ekf.pose, self.ekf.std, (time.perf_counter() - t0) * 1e3, clusters)
            report.corrections_accepted = accepted
            report.corrections_rejected = rejected
            self.revs_processed += 1
            if self._on_report is not None:
                self._on_report(report)

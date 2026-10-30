"""Localisation par fusion de capteurs (étapes 0 à 6 du plan, journal de l'étape 7).

Un thread unique, séparé de la lecture du lidar, fait deux choses en boucle :
  1. lit l'odométrie (HolonomicBase.pose) et fait avancer l'EKF (étape 2) ;
  2. pour chaque tour de lidar publié par LidarAcquisition : capping, clusters, association aux
     poteaux (étape 3), correction séquentielle de la pose à l'instant de chaque cluster (étape 4),
     détection de perte et relocalisation par triangulation (étape 6).

La dérive d'odométrie (étape 6, point 3) n'est volontairement pas traitée : l'odométrie est considérée juste.

Les trois fonctions d'entrée sont volontairement simples (pose_source, rev_queue, init_pose) pour
pouvoir tester sans robot. `log_path` écrit un journal CSV (un tour par ligne) pour l'analyse après coup.
"""
import csv
import math
import queue
from collections import deque
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .ekf import EkfParams, PoseEKF
from .lidar_clusters import ClusterParams, cap_points, find_clusters
from .pole_association import AssocParams, AssocResult, associate_poles
from .relocalize import RelocParams, triangulate
from .timebase import ClockAligner

def _filter_points_in_field(points, ekf: PoseEKF):
    """Filtre les impacts hors terrain en projetant tous les points par lots NumPy.

    Seules les poses historiques sont nécessaires ici : appeler state_at() point par point
    calculait aussi les covariances interpolées, inutiles pour ce filtre géométrique.
    """
    import numpy as np

    if len(points) == 0:
        return points, np.empty((0, 2), dtype=float), 0

    p = ekf.p
    xmin, ymin, xmax, ymax = p.field_mm
    margin = p.field_filter_margin_mm
    xmin -= margin
    ymin -= margin
    xmax += margin
    ymax += margin
    ox, oy = p.lidar_offset_mm

    # Snapshot de l'historique. On n'interpole que x, y et theta, pas les covariances.
    if not ekf._ht:
        return points[:0], np.empty((0, 2), dtype=float), len(points)

    hist_t = np.asarray(ekf._ht, dtype=float)
    hist_x = np.asarray(ekf._hx, dtype=float)
    t_s = np.asarray(points["t_ns"], dtype=np.float64) / 1e9 - p.lidar_latency_s
    angles = np.asarray(points["angle"], dtype=float)
    distances = np.asarray(points["dist"], dtype=float)

    # Interpolation vectorisée équivalente à state_at() pour la pose. Les timestamps
    # en dehors de l'historique utilisent la pose la plus proche et sont comptés.
    idx1 = np.searchsorted(hist_t, t_s, side="right")
    before = idx1 == 0
    after = idx1 >= len(hist_t)
    clamped = before | after
    unverified = int(np.count_nonzero(
        (before & (t_s < hist_t[0] - p.clamp_tol_s))
        | (after & (t_s > hist_t[-1] + p.clamp_tol_s))
    ))

    idx0 = np.clip(idx1 - 1, 0, len(hist_t) - 1)
    idx1 = np.clip(idx1, 0, len(hist_t) - 1)
    t0, t1 = hist_t[idx0], hist_t[idx1]
    denom = t1 - t0
    w = np.zeros_like(t_s)
    valid_span = denom > 0
    w[valid_span] = (t_s[valid_span] - t0[valid_span]) / denom[valid_span]
    w = np.clip(w, 0.0, 1.0)
    poses = (1.0 - w[:, None]) * hist_x[idx0] + w[:, None] * hist_x[idx1]
    x, y, theta = poses[:, 0], poses[:, 1], poses[:, 2]

    c, s = np.cos(theta), np.sin(theta)
    lidar_x = x + c * ox - s * oy
    lidar_y = y + s * ox + c * oy
    world_angle = theta + np.radians(p.lidar_zero_deg - angles)
    hit_x = lidar_x + distances * np.cos(world_angle)
    hit_y = lidar_y + distances * np.sin(world_angle)

    keep = (hit_x >= xmin) & (hit_x <= xmax) & (hit_y >= ymin) & (hit_y <= ymax)
    world_xy = np.column_stack((hit_x[keep], hit_y[keep]))
    return points[keep], world_xy, unverified

LOG_COLUMNS = ("t_s", "rev", "x_mm", "y_mm", "theta_deg", "sigma_x_mm", "sigma_y_mm", "sigma_theta_deg",
               "n_clusters", "n_matched", "corr_accepted", "corr_rejected", "mahal_mean",
               "lost", "relocalized", "reliable", "queue_wait_ms", "ekf_predict_ms", "proc_ms",
               "cap_ms", "filter_ms", "clusters_ms", "association_ms", "correction_ms",
               "acq_to_correction_ms", "acq_to_correction_avg_5s_ms")


@dataclass
class Report:
    rev_number: int
    t_s: float                 # instant de référence du tour (epoch)
    n_points: int              # mesures valides du tour
    n_capped: int              # ... après capping
    n_clusters: int
    result: AssocResult
    pose: tuple                # (x mm, y mm, θ rad) après les corrections du tour
    std: tuple                 # (σx mm, σy mm, σθ °)
    proc_ms: float             # temps de calcul pur du tour (doit rester << 100 ms)
    acq_to_correction_ms: float = 0.0       # du premier point du tour à la fin des corrections
    acq_to_correction_avg_5s_ms: float = 0.0  # moyenne glissante sur les 5 dernières secondes
    queue_wait_ms: float = 0.0              # fin du tour -> début du traitement
    ekf_predict_ms: float = 0.0             # temps cumulé des prédictions odométriques depuis le tour précédent
    clusters: list = field(default_factory=list)   # clusters du tour (diagnostic : où le lidar voit des objets)
    corrections_accepted: int = 0
    corrections_rejected: int = 0                  # refusées par le gating statistique
    mahal: list = field(default_factory=list)      # écart normalisé de chaque correction acceptée (moyenne ~2 attendue)
    lost: bool = False                             # le filtre se considère perdu (aucune correction depuis > lost_after_s)
    relocalized: bool = False                      # la pose a été reprise par triangulation à ce tour
    reliable: bool = True                          # pas perdu et incertitude sous safe_std_mm
    odo_hz: float = 0.0                            # cadence réelle de l'odométrie vue par le filtre
    timings_ms: dict = field(default_factory=dict)  # cap_points, find_clusters, associate_poles, correct, relocalize, total
    points: object = None  # Points lidar conservés après filtrage géométrique
    points_world_mm: object = None  # Impacts filtrés déjà projetés dans le repère terrain
    points_unverified: int = 0  # Points rejetés faute d'un état EKF fiable à leur horodatage


class Localizer:
    def __init__(self, pose_source: Callable, rev_queue: queue.Queue, init_pose: tuple,
                 params: Optional[EkfParams] = None, assoc: Optional[AssocParams] = None,
                 cluster_params: Optional[ClusterParams] = None, poll_s: float = 0.005,
                 on_report: Optional[Callable] = None, on_pose: Optional[Callable] = None,
                 reloc: Optional[RelocParams] = None, log_path: Optional[str] = None):
        """
        pose_source : fonction sans argument -> (x mm, y mm, θ rad, instant) ; HolonomicBase.pose.
                      Instant en secondes, epoch ou monotone (recalé automatiquement).
        rev_queue   : file des tours publiés par LidarAcquisition.
        init_pose   : (x mm, y mm, θ rad) du robot dans le repère terrain au démarrage.
        log_path    : fichier CSV du journal (pose, incertitude, corrections, écart normalisé), ou None.
        """
        self.ekf = PoseEKF(params)
        self._pose_source = pose_source
        self._q = rev_queue
        self._init_pose = init_pose
        self._assoc = assoc or AssocParams()
        self._cp = cluster_params or ClusterParams()
        self._reloc = reloc or RelocParams()
        self._poll_s = poll_s
        self._on_report = on_report
        self._on_pose = on_pose
        self.odo_clock = ClockAligner("odometrie", unit="s")
        self._last_odo_t = None
        self._ekf_predict_ms = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.revs_processed = 0
        self.revs_skipped = 0      # tours ignorés (démarrage ou rattrapage de file)
        self.error = None
        self._last_timestamp_diag = 0.0
        self._timestamp_waits = 0
        # perte / relocalisation
        self.lost = False
        self.relocalizations = 0
        self._bad_since = None     # début de la série de tours « poteaux vus mais aucune correction acceptée »
        # cadence de l'odométrie
        self._odo_n = 0
        self._odo_win_t0 = time.perf_counter()
        self._odo_hz = 0.0
        # Latence de bout en bout : durée acquisition + attente en file + traitement/correction.
        # Fenêtre glissante indexée sur l'instant de fin de traitement (horloge monotone).
        self._e2e_samples = deque()
        self._e2e_last_ms = 0.0
        self._e2e_avg_5s_ms = 0.0
        # journal
        self._log_path = log_path
        self._log_f = None
        self._log_w = None
        self.log_error = None

    def start(self):
        self._open_log()
        self._thread = threading.Thread(target=self._run, daemon=True, name="localizer")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._log_f is not None:
            try:
                self._log_f.close()
            except OSError:
                pass
            self._log_f = self._log_w = None

    def status(self) -> dict:
        """Pose et incertitude cohérentes entre elles (lues sous verrou), pour la stratégie."""
        with self._lock:
            sx, sy, st = self.ekf.std
            ready = self.ekf.ready
            return {"ready": ready, "pose": self.ekf.pose, "std": (sx, sy, st),
                    "reliable": ready and not self.lost and math.hypot(sx, sy) < self.ekf.p.safe_std_mm,
                    "lost": self.lost, "relocalizations": self.relocalizations,
                    "revs_processed": self.revs_processed, "revs_skipped": self.revs_skipped,
                    "odo_hz": self._odo_hz, "acq_to_correction_ms": self._e2e_last_ms,
                    "acq_to_correction_avg_5s_ms": self._e2e_avg_5s_ms}

    # ------------------------------------------------------------------ journal
    def _open_log(self):
        if not self._log_path:
            return
        try:
            self._log_f = open(self._log_path, "w", newline="", encoding="utf-8")
            self._log_w = csv.writer(self._log_f)
            self._log_w.writerow(LOG_COLUMNS)
        except OSError as e:               # un journal impossible à écrire ne doit pas arrêter la fusion
            self.log_error = e
            self._log_f = self._log_w = None

    def _write_log(self, r: Report):
        if self._log_w is None:
            return
        n_matched = sum(1 for m in r.result.matches if m.status == "matched")
        mahal_mean = sum(r.mahal) / len(r.mahal) if r.mahal else ""
        x, y, th = r.pose
        try:
            self._log_w.writerow([f"{r.t_s:.6f}", r.rev_number, f"{x:.2f}", f"{y:.2f}", f"{math.degrees(th):.3f}",
                                  f"{r.std[0]:.2f}", f"{r.std[1]:.2f}", f"{r.std[2]:.3f}",
                                  r.n_clusters, n_matched, r.corrections_accepted, r.corrections_rejected,
                                  f"{mahal_mean:.3f}" if mahal_mean != "" else "",
                                  int(r.lost), int(r.relocalized), int(r.reliable),
                                  f"{r.queue_wait_ms:.2f}", f"{r.ekf_predict_ms:.2f}", f"{r.proc_ms:.2f}",
                                  f"{r.timings_ms.get('cap_points', 0.0):.2f}",
                                  f"{r.timings_ms.get('filter_points_in_field', 0.0):.2f}",
                                  f"{r.timings_ms.get('find_clusters', 0.0):.2f}",
                                  f"{r.timings_ms.get('associate_poles', 0.0):.2f}",
                                  f"{r.timings_ms.get('correct', 0.0):.2f}",
                                  f"{r.acq_to_correction_ms:.2f}", f"{r.acq_to_correction_avg_5s_ms:.2f}"])
            self._log_f.flush()
        except OSError as e:
            self.log_error = e
            self._log_w = None

    # ------------------------------------------------------------------ boucle
    def _run(self):
        try:
            while not self._stop.is_set():
                self._poll_odometry()
                self._process_revs()
                self._stop.wait(self._poll_s)
        except BaseException as e:
            self.error = e

    def _tick_odo_rate(self):
        self._odo_n += 1
        now = time.perf_counter()
        dt = now - self._odo_win_t0
        if dt >= 1.0:
            self._odo_hz = self._odo_n / dt
            self._odo_n = 0
            self._odo_win_t0 = now

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
            _ekf_t0 = time.perf_counter()
            self.ekf.predict_odometry(x, y, th, ts)
            self._ekf_predict_ms += (time.perf_counter() - _ekf_t0) * 1000.0
            pose = self.ekf.pose
        self._tick_odo_rate()

        # Callback après chaque nouvel échantillon d’odométrie accepté.
        # Il est hors du verrou pour ne pas bloquer les lectures concurrentes de l’état EKF.
        if self._on_pose is not None:
            self._on_pose(ts, pose)

    def _timestamp_diag(self, message: str):
        """Évite de saturer la console avec un diagnostic répété à chaque tour."""
        now = time.monotonic()
        if now - self._last_timestamp_diag >= 1.0:
            print(f"[LOCALIZER TIMESTAMP] {message}", flush=True)
            self._last_timestamp_diag = now

    def _process_revs(self):
        while True:
            # Garder l'historique EKF aussi récent que possible, même si plusieurs tours
            # lidar attendent dans la file.
            self._poll_odometry()
            try:
                rev = self._q.get_nowait()
            except queue.Empty:
                return

            if not self.ekf.ready:
                # Pas encore d'odométrie : sans pose de départ, on ne peut pas dire où chercher les poteaux.
                self.revs_skipped += 1
                continue

            # Si le tour est déjà plus ancien que l'historique EKF, le traiter ne peut
            # produire aucune correction utile. Vider alors la file et ne garder que le
            # tour le plus récent pour rattraper le retard au lieu de traiter des tours périmés.
            with self._lock:
                history_start = self.ekf._ht[0] if self.ekf._ht else None
                stale_limit_s = self.ekf.p.clamp_tol_s
            rev_end_s = rev.t_end_ns / 1e9
            if history_start is not None and rev_end_s < history_start - stale_limit_s:
                newest = rev
                dropped = 0
                while True:
                    try:
                        newest = self._q.get_nowait()
                        dropped += 1
                    except queue.Empty:
                        break
                if dropped:
                    self.revs_skipped += dropped
                    self._timestamp_diag(
                        f"MESURES TROP ANCIENNES : {dropped} tour(s) intermédiaire(s) écarté(s), "
                        f"conservation du plus récent (tour {newest.number})"
                    )
                    rev = newest
                else:
                    # Aucun tour plus récent à substituer : celui-ci est inutilisable.
                    self.revs_skipped += 1
                    delta_ms = (history_start - rev_end_s) * 1000.0
                    self._timestamp_diag(
                        f"MESURE TROP ANCIENNE : tour {rev.number}, début d'historique EKF "
                        f"{delta_ms:.1f} ms après la fin du tour ; tour ignoré"
                    )
                    continue

            # Ne pas traiter une mesure dont l'instant est encore dans le futur par rapport
            # à l'historique EKF : state_at() la bornerait au dernier état, et correct_pole()
            # finirait par refuser la correction. On la remet en tête logique de la file
            # (file FIFO non bornée attendue) et on laisse la boucle principale repoller l'odo.
            with self._lock:
                history_end = self.ekf._ht[-1] if self.ekf._ht else None
                tol_s = self.ekf.p.clamp_tol_s
                latency_s = self.ekf.p.lidar_latency_s
            # Le point le plus récent du tour est sa fin : utiliser le milieu laisserait
            # les derniers points hors historique alors que le tour semble acceptable.
            rev_latest_measure_s = rev.t_end_ns / 1e9 - latency_s
            if history_end is not None and rev_latest_measure_s > history_end + tol_s:
                try:
                    self._q.put_nowait(rev)
                except queue.Full:
                    # Ne pas perdre silencieusement le tour si un producteur utilise une
                    # file bornée : le prochain tour sera préféré au rattrapage.
                    self.revs_skipped += 1
                delta_ms = (rev_latest_measure_s - history_end) * 1000.0
                self._timestamp_waits += 1
                self._timestamp_diag(
                    f"MESURE TROP RÉCENTE : tour {rev.number}, EKF en retard de {delta_ms:.1f} ms "
                    f"(tolérance {tol_s * 1000:.0f} ms), attente de l'odométrie"
                )
                return

            self._process_one(rev)

    def _process_one(self, rev):
        p = self.ekf.p
        # Délai entre la fin de l'acquisition du tour et le début de son traitement.
        queue_wait_ms = max(0.0, (time.time_ns() - rev.t_end_ns) / 1e6)
        t0 = time.perf_counter()
        capped = cap_points(rev.points, self._cp)
        t1 = time.perf_counter()
        # Les impacts hors du terrain élargi de 200 mm sont retirés avant le clustering.
        # Les clusters transmis au rapport/visualiseur ne contiennent donc pas ces points.
        in_field, points_world_mm, points_unverified = _filter_points_in_field(capped, self.ekf)
        t_geom = time.perf_counter()
        clusters = find_clusters(in_field, self._cp)
        t2 = time.perf_counter()
        # Milieu du tour (secondes) : sert d'instant de référence pour afficher la mesure attendue des poteaux
        # sans candidat et pour dater une relocalisation. Les clusters, eux, sont datés point par point.
        t_ref = (rev.t_start_ns + rev.t_end_ns) / 2e9
        # Le clustering peut prendre plusieurs dizaines de ms : rafraîchir l'odométrie
        # avant l'association et les corrections afin que l'historique couvre les mesures.
        self._poll_odometry()
        res = associate_poles(self.ekf, clusters, t_ref, self._assoc)
        t3 = time.perf_counter()

        # Étape 4 : correction séquentielle. Chaque mesure validée par l'association est soumise au gating
        # statistique (seuil chi² à 99 % pour deux mesures), à la pose de l'instant du cluster.
        # La distance mesurée est ramenée au centre du poteau (le lidar voit sa surface).
        accepted = rejected = 0
        mahals = []
        # Nouvelle lecture juste avant la correction : l'association peut elle aussi prendre
        # du temps et ne doit pas laisser l'EKF derrière l'instant de mesure.
        self._poll_odometry()
        with self._lock:
            for match in res.matches:
                if match.status != "matched" or match.cluster is None:
                    continue
                ok, mahal = self.ekf.correct_pole(
                    match.pole,
                    match.cluster.dist_mm + p.pole_radius_mm,
                    match.cluster.angle_deg,
                    t_s=match.cluster.t_ns / 1e9,
                )
                if ok:
                    accepted += 1
                    mahals.append(mahal)
                else:
                    rejected += 1
        t4 = time.perf_counter()

        # Étape 6 : perdu = des poteaux de la bonne taille sont vus mais aucune correction n'est acceptée
        # pendant plus de lost_after_s. Un poteau simplement masqué (aucun cluster de sa taille) n'est pas
        # « perdu » : l'odométrie avance, P grandit et les fenêtres s'élargissent toutes seules.
        relocalized = False
        if accepted:
            self._bad_since = None
            self.lost = False
        elif res.n_size_ok > 0:
            if self._bad_since is None:
                self._bad_since = t_ref
            if t_ref - self._bad_since >= p.lost_after_s:
                self.lost = True
        if self.lost:
            found = None # triangulate(self.ekf, clusters, self._reloc)
            if found is not None:
                with self._lock:
                    self.ekf.relocalize(found.x_mm, found.y_mm, found.theta_rad, t_ref)
                    self.lost = False
                    self.relocalizations += 1
                self._bad_since = None
                relocalized = True
        t5 = time.perf_counter()

        # Latence complète : du premier timestamp du tour à la fin de la correction potentielle.
        # t_start_ns est en epoch ns; perf_counter sert uniquement à gérer la fenêtre glissante.
        e2e_ms = max(0.0, (time.time_ns() - rev.t_start_ns) / 1e6)
        window_now = time.perf_counter()
        with self._lock:
            self._e2e_samples.append((window_now, e2e_ms))
            cutoff = window_now - 5.0
            while self._e2e_samples and self._e2e_samples[0][0] < cutoff:
                self._e2e_samples.popleft()
            self._e2e_last_ms = e2e_ms
            self._e2e_avg_5s_ms = (
                sum(value for _, value in self._e2e_samples) / len(self._e2e_samples)
                if self._e2e_samples else 0.0
            )
            pose, std = self.ekf.pose, self.ekf.std
            lost = self.lost
            e2e_avg_5s_ms = self._e2e_avg_5s_ms
            self.revs_processed += 1
        # Inclut toutes les prédictions odométriques effectuées depuis le rapport précédent,
        # y compris celles déclenchées pendant le traitement de ce tour.
        ekf_predict_ms = self._ekf_predict_ms
        report = Report(
            rev.number, t_ref, len(rev.points), len(capped), len(clusters), res, pose, std,
            (t5 - t0) * 1e3, clusters=clusters,
            acq_to_correction_ms=e2e_ms, acq_to_correction_avg_5s_ms=e2e_avg_5s_ms,
            queue_wait_ms=queue_wait_ms, ekf_predict_ms=ekf_predict_ms,
            points=in_field,
            points_world_mm=points_world_mm, points_unverified=points_unverified,
            corrections_accepted=accepted, corrections_rejected=rejected, mahal=mahals,
            lost=lost, relocalized=relocalized,
            reliable=(not lost) and math.hypot(std[0], std[1]) < p.safe_std_mm,
            odo_hz=self._odo_hz,
            timings_ms={"cap_points": (t1 - t0) * 1e3, "filter_points_in_field": (t_geom - t1) * 1e3,
                        "find_clusters": (t2 - t_geom) * 1e3, "associate_poles": (t3 - t2) * 1e3,
                        "correct": (t4 - t3) * 1e3,
                        "ekf_total": ekf_predict_ms + (t4 - t3) * 1e3,
                        "relocalize": (t5 - t4) * 1e3,
                        "total": (t5 - t0) * 1e3},
        )
        self._ekf_predict_ms = 0.0
        self._write_log(report)
        if self._on_report is not None:
            self._on_report(report)

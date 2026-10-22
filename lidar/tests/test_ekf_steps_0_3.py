"""Tests sans robot des étapes 0 à 3 du plan d'intégration de l'EKF (lidar simulé).

    python -m pytest tests/test_ekf_steps_0_3.py -v

Le simulateur lance des rayons depuis le lidar sur des cercles (3 poteaux de 100 mm aux coins du
terrain 2 m x 2 m, un gros obstacle type robot adverse, un parasite de la taille d'un poteau),
avec un bruit de ±30 mm sur la distance et 500 points par tour, chaque point daté à son instant.
Les bords du terrain sont supposés sous le lidar (point ouvert du plan : s'ils sont à sa hauteur, un
poteau de coin fusionne avec le bord).
"""
import math
import queue
import sys
import time
import types

import numpy as np
import pytest

from core.ekf import EkfParams, PoseEKF, wrap_deg, wrap_pi
from core.lidar_clusters import POINT_DTYPE, ClusterParams, cap_points, find_clusters
from core.localizer import Localizer
from core.pole_association import AssocParams, associate_poles
from core.timebase import ClockAligner

T0 = 1000.0                                      # instant de départ des scénarios (s)
POLES = [(0.0, 0.0), (2000.0, 0.0), (0.0, 2000.0)]
POLE_CIRCLES = [(x, y, 50.0) for x, y in POLES]


# ---------------------------------------------------------------- outils de simulation
def make_scan(pose_fn, circles, t0=T0, n=500, rev_s=0.1, noise_mm=30.0, seed=0, lidar_zero_deg=0.0):
    """Un tour de lidar simulé. pose_fn(t) -> (x mm, y mm, θ rad) : la pose peut changer pendant le tour."""
    rng = np.random.default_rng(seed)
    pts = []
    for i in range(n):
        a = i * 360.0 / n                                  # angle lidar (sens horaire)
        t = t0 + i / n * rev_s
        x, y, th = pose_fn(t)
        phi = th + math.radians(lidar_zero_deg - a)        # direction du rayon dans le repère terrain
        ux, uy = math.cos(phi), math.sin(phi)
        best = None
        for cx, cy, r in circles:
            ox, oy = cx - x, cy - y
            b = ux * ox + uy * oy
            disc = b * b - (ox * ox + oy * oy - r * r)
            if disc < 0 or b <= 0:
                continue
            tt = b - math.sqrt(disc)
            if tt > 0 and (best is None or tt < best):
                best = tt
        if best is not None:
            pts.append((a, best + rng.uniform(-noise_mm, noise_mm), 47, int(round(t * 1e9))))
    return np.array(pts, dtype=POINT_DTYPE)


def static(x, y, th):
    return lambda t: (x, y, th)


def make_ekf(x, y, th, t=T0, hold_until=T0 + 0.3, pos_std=None, th_std_rad=None, params=None):
    """EKF initialisé à la pose donnée et maintenu immobile (historique rempli jusqu'à hold_until)."""
    ekf = PoseEKF(params)
    ekf.reset(x, y, th, t, pos_std_mm=pos_std, theta_std_rad=th_std_rad)
    k = 1
    while t + k * 0.005 <= hold_until:
        ekf.predict(0.0, 0.0, 0.0, t + k * 0.005)
        k += 1
    return ekf


def cluster_xy(c, x, y, th, radius=50.0):
    """Position terrain du centre d'un cluster, vue depuis la pose (x, y, θ) (lidar_zero = 0)."""
    phi = th + math.radians(-c.angle_deg)
    return x + (c.dist_mm + radius) * math.cos(phi), y + (c.dist_mm + radius) * math.sin(phi)


# ---------------------------------------------------------------- étape 1/2 : modèle et prédiction
@pytest.mark.parametrize("offset,zero", [((0.0, 0.0), 0.0), ((30.0, -20.0), 15.0)])
def test_jacobian_matches_finite_differences(offset, zero):
    ekf = PoseEKF(EkfParams(lidar_offset_mm=offset, lidar_zero_deg=zero))
    rng = np.random.default_rng(1)
    for _ in range(20):
        x = np.array([rng.uniform(300, 1700), rng.uniform(300, 1700), rng.uniform(-3, 3)])
        j = int(rng.integers(0, 3))
        r0, a0, H = ekf.measurement(x, j)
        for k, eps in enumerate((1e-3, 1e-3, 1e-6)):
            xp = x.copy()
            xp[k] += eps
            r1, a1, _ = ekf.measurement(xp, j)
            assert (r1 - r0) / eps == pytest.approx(H[0, k], rel=1e-4, abs=1e-6)
            da = wrap_pi(math.radians(a1) - math.radians(a0))
            assert da / eps == pytest.approx(H[1, k], rel=1e-4, abs=1e-8)


def test_straight_line_adds_20mm_per_meter():
    ekf = PoseEKF()
    ekf.reset(0, 0, 0, T0, pos_std_mm=0.0, theta_std_rad=0.0)
    for i in range(1, 201):                                  # 1 m en pas de 5 mm
        ekf.predict(5.0, 0.0, 0.0, T0 + i * 0.005)
    assert ekf.x[0] == pytest.approx(1000.0)
    assert math.sqrt(ekf.P[0, 0]) == pytest.approx(20.0, rel=0.01)    # 2 % de 1 m
    assert math.sqrt(ekf.P[1, 1]) == pytest.approx(20.0, rel=0.01)


def test_rotation_adds_about_1_percent():
    ekf = PoseEKF()
    ekf.reset(0, 0, 0, T0, pos_std_mm=0.0, theta_std_rad=0.0)
    for i in range(1, 201):                                  # 90° en 200 pas
        ekf.predict(0.0, 0.0, math.radians(90) / 200, T0 + i * 0.005)
    assert ekf.x[2] == pytest.approx(math.pi / 2)
    assert math.degrees(math.sqrt(ekf.P[2, 2])) == pytest.approx(0.72, abs=0.02)   # ~0,8 % de 90°


def test_odometry_frame_is_converted_to_terrain_frame():
    """Le robot est posé à (1000, 1000) cap 90° sur le terrain ; son odométrie démarre à (0, 0, 0).
    Rouler de 100 mm vers l'avant (x odométrie) doit faire monter le robot en y sur le terrain."""
    ekf = PoseEKF()
    ekf.reset(1000, 1000, math.pi / 2, T0)
    ekf.predict_odometry(0.0, 0.0, 0.0, T0)
    for i in range(1, 21):
        ekf.predict_odometry(5.0 * i, 0.0, 0.0, T0 + i * 0.005)
    x, y, th = ekf.pose
    assert (x, y) == pytest.approx((1000.0, 1100.0), abs=1e-6)
    assert th == pytest.approx(math.pi / 2)


def test_odometry_with_rotation_matches_holonomic_base_convention():
    """Même formule que HolonomicBase._odo_step (arc au cap moyen) : on la rejoue à l'envers."""
    ekf = PoseEKF()
    ekf.reset(0, 0, 0, T0)
    ex, ey, eth = 0.0, 0.0, 0.0                              # odométrie simulée, comme dans la base
    ekf.predict_odometry(ex, ey, eth, T0)
    for i in range(1, 101):
        dxl, dyl, dth = 4.0, 1.0, math.radians(0.9)
        thm = eth + dth / 2
        ex += math.cos(thm) * dxl - math.sin(thm) * dyl
        ey += math.sin(thm) * dxl + math.cos(thm) * dyl
        eth += dth
        ekf.predict_odometry(ex, ey, eth, T0 + i * 0.01)
    assert ekf.x[:2] == pytest.approx([ex, ey], abs=1e-6)      # repères confondus au départ
    assert ekf.x[2] == pytest.approx(eth)


def test_state_at_interpolates_and_flags_out_of_history():
    ekf = PoseEKF()
    ekf.reset(0, 0, 0, T0)
    for i in range(1, 21):                                   # 100 mm en 100 ms (1 m/s)
        ekf.predict(5.0, 0.0, 0.0, T0 + i * 0.005)
    mid = ekf.state_at(T0 + 0.0525)
    assert mid.x[0] == pytest.approx(52.5) and not mid.clamped
    assert ekf.state_at(T0 + 0.1 + 0.005).clamped is False   # quelques ms après la dernière entrée : normal
    assert ekf.state_at(T0 + 1.0).clamped is True            # horloges décalées : à signaler
    assert ekf.state_at(T0 - 1.0).clamped is True


def test_history_is_pruned():
    ekf = PoseEKF(EkfParams(history_s=0.5))
    ekf.reset(0, 0, 0, T0)
    for i in range(1, 401):
        ekf.predict(1.0, 0.0, 0.0, T0 + i * 0.005)           # 2 s
    assert len(ekf._ht) < 120 and ekf._ht[-1] - ekf._ht[0] < 0.52


def test_clock_aligner():
    epoch = ClockAligner("e")
    assert epoch.epoch_s(time.time()) == pytest.approx(time.time(), abs=0.01) and not epoch.recalibrated
    mono = ClockAligner("m")
    t1 = time.monotonic()
    assert mono.epoch_s(t1) == pytest.approx(time.time(), abs=0.01) and mono.recalibrated
    assert mono.epoch_s(t1 + 0.5) - mono.epoch_s(t1) == pytest.approx(0.5)      # les durées restent exactes
    ns = ClockAligner("n", unit="ns")
    assert ns.epoch_ns(time.time_ns()) == pytest.approx(time.time_ns(), abs=10_000_000)


# ---------------------------------------------------------------- étape 0/3 : clusters
def test_clusters_find_poles_and_obstacles():
    circles = POLE_CIRCLES + [(1400.0, 1300.0, 150.0), (1200.0, 1500.0, 50.0)]   # hors des lignes de visée des poteaux
    scan = make_scan(static(1000, 1000, 0.5), circles)
    clusters = find_clusters(cap_points(scan))
    ekf = PoseEKF()
    sized = [c for c in clusters if 15 <= c.width_mm <= 150]
    for j in range(3):                                       # un cluster de taille poteau à chaque poteau attendu
        d_exp, a_exp, _ = ekf.measurement(np.array([1000, 1000, 0.5]), j)
        near = [c for c in sized if abs(c.dist_mm + 50 - d_exp) < 40 and abs(wrap_deg(c.angle_deg - a_exp)) < 2]
        assert len(near) == 1
    assert any(c.width_mm > 150 for c in clusters)           # le gros obstacle est écarté par sa taille


def test_cluster_across_zero_degrees_is_merged():
    # cap 225° : le poteau (0, 0) est exactement à l'angle lidar 0°, donc coupé en deux par le début du tour
    scan = make_scan(static(1000, 1000, math.radians(225)), POLE_CIRCLES)
    clusters = find_clusters(scan)
    seam = [c for c in clusters if c.seam]
    assert len(seam) == 1 and seam[0].n_points >= 4
    assert abs(wrap_deg(seam[0].angle_deg)) < 2.0 and 15 <= seam[0].width_mm <= 150


def test_capping_removes_close_and_far_points():
    pts = np.zeros(4, dtype=POINT_DTYPE)
    pts["dist"] = [20.0, 100.0, 3000.0, 5000.0]
    assert list(cap_points(pts)["dist"]) == [100.0, 3000.0]


# ---------------------------------------------------------------- étape 3 : association
def test_association_static_random_poses_no_wrong_match():
    """Critère de passage de l'étape 3 : avec des obstacles parasites, chaque poteau associé est le bon."""
    rng = np.random.default_rng(42)
    wrong = matched = total = 0
    for k in range(60):
        x, y, th = rng.uniform(350, 1650), rng.uniform(350, 1650), rng.uniform(-math.pi, math.pi)
        circles = list(POLE_CIRCLES)
        for r in (150.0, 50.0):                              # gros obstacle puis parasite de taille poteau
            circles.append((rng.uniform(250, 1750), rng.uniform(250, 1750), r))
        scan = make_scan(static(x, y, th), circles, seed=k)
        # l'EKF se trompe un peu : ±20 mm et ±1° (son incertitude de départ)
        ex, ey, eth = x + rng.uniform(-20, 20), y + rng.uniform(-20, 20), th + math.radians(rng.uniform(-1, 1))
        ekf = make_ekf(ex, ey, eth)
        res = associate_poles(ekf, find_clusters(cap_points(scan)), T0 + 0.05)
        for m in res.matches:
            total += 1
            if m.status == "matched":
                matched += 1
                cx, cy = cluster_xy(m.cluster, x, y, th)
                if math.hypot(cx - POLES[m.pole][0], cy - POLES[m.pole][1]) > 150:
                    wrong += 1
    assert wrong == 0
    assert matched >= 0.85 * total          # le reste : poteaux masqués par un obstacle (absent), jamais faux


def test_cluster_outside_all_windows_is_ignored():
    # robot (1000, 1000), cap 0 ; le seul objet est un cylindre de poteau loin de tout poteau attendu
    scan = make_scan(static(1000, 1000, 0.0), [(1500.0, 1500.0, 50.0)])
    ekf = make_ekf(1000, 1000, 0.0)
    res = associate_poles(ekf, find_clusters(cap_points(scan)), T0 + 0.05)
    assert [m.status for m in res.matches] == ["absent"] * 3
    assert res.n_unused == res.n_clusters == 1


def _parasite_beside_pole(offset_deg):
    """Obstacle de taille poteau à la même distance que le poteau (2000, 0), décalé de offset_deg."""
    rx, ry = 1000.0, 1000.0
    d = math.hypot(2000 - rx, 0 - ry)
    ang = math.atan2(0 - ry, 2000 - rx) + math.radians(offset_deg)
    return (rx + d * math.cos(ang), ry + d * math.sin(ang), 50.0)


def test_two_candidates_means_skip_pole_and_window_follows_uncertainty():
    scan = make_scan(static(1000, 1000, 0.0), POLE_CIRCLES + [_parasite_beside_pole(6.0)])
    clusters = find_clusters(cap_points(scan))
    tight = associate_poles(make_ekf(1000, 1000, 0.0), clusters, T0 + 0.05)
    loose = associate_poles(make_ekf(1000, 1000, 0.0, pos_std=150.0, th_std_rad=math.radians(5)),
                            clusters, T0 + 0.05)
    # Pose précise : fenêtre ~±4°, le parasite à 6° est hors fenêtre -> le poteau est associé.
    assert tight.matches[1].status == "matched"
    # Pose incertaine : fenêtre élargie (~±15°), le parasite entre dedans -> deux candidats, on saute le poteau.
    assert loose.matches[1].status == "ambiguous" and loose.matches[1].n_candidates == 2
    assert loose.matches[1].win_angle_deg > 3 * tight.matches[1].win_angle_deg


def test_hidden_pole_is_absent_others_still_matched():
    circles = [POLE_CIRCLES[0], POLE_CIRCLES[2]]            # le poteau (2000, 0) est masqué
    scan = make_scan(static(1000, 1000, 0.0), circles)
    res = associate_poles(make_ekf(1000, 1000, 0.0), find_clusters(cap_points(scan)), T0 + 0.05)
    assert [m.status for m in res.matches] == ["matched", "absent", "matched"]


def test_association_while_moving_uses_pose_at_cluster_time():
    """À 1 m/s, le robot avance de 100 mm pendant un tour : l'association doit utiliser la pose à
    l'instant de chaque cluster, fournie par l'historique de l'EKF."""
    pose_fn = lambda t: (500.0 + 1000.0 * (t - T0), 1000.0, 0.3)
    ekf = PoseEKF()
    t_start = T0 - 0.3
    ekf.reset(*pose_fn(t_start), t_start)
    t = t_start
    while t < T0 + 0.13:                                    # odométrie parfaite, pas de 5 ms
        ekf.predict_odometry(*pose_fn(t), t)
        t += 0.005
    scan = make_scan(pose_fn, POLE_CIRCLES + [(1400.0, 1500.0, 150.0)])
    res = associate_poles(ekf, find_clusters(cap_points(scan)), T0 + 0.05)
    assert res.n_clamped == 0
    assert [m.status for m in res.matches] == ["matched"] * 3
    # Écarts proches du bruit du lidar (±30 mm). Avec la pose de fin de tour au lieu de celle de l'instant
    # du cluster, on mesure jusqu'à ~100 mm et ~4° d'écart à 1 m/s : ce test le détecterait.
    for m in res.matches:
        assert abs(m.d_dist_mm) < 40 and abs(m.d_angle_deg) < 0.5


# ---------------------------------------------------------------- étape 0 : acquisition (bibliothèque simulée)
def _fake_rplidar_module():
    import asyncio

    class FakeLidar:
        def __init__(self, port, baud):
            self.stop_event, self.output_queue = asyncio.Event(), asyncio.Queue()
            self.reset_called = False

        def simple_scan(self, make_return_dict=False):
            return self._gen()

        async def _gen(self):
            angle = 100.0                                    # démarre au milieu d'un tour : le premier tour est partiel
            while not self.stop_event.is_set():
                for _ in range(50):                          # 5000 points/s, 500 points par tour
                    angle = round((angle + 0.72) % 360, 2)
                    await self.output_queue.put({"q": 47, "a_deg": angle, "d_mm": 1000.0 if angle % 7 else None,
                                                 "t_rx_ns": time.time_ns()})
                await asyncio.sleep(0.01)

        def reset(self):
            self.reset_called = True

    mod = types.ModuleType("rplidarc1")
    mod.RPLidar = FakeLidar
    return mod


def test_acquisition_publishes_only_complete_revolutions(monkeypatch):
    monkeypatch.setitem(sys.modules, "rplidarc1", _fake_rplidar_module())
    from core.lidar_acquisition import LidarAcquisition

    acq = LidarAcquisition("fake")
    acq.start()
    time.sleep(1.8)
    acq.stop()
    assert acq.error is None
    revs = []
    while True:
        try:
            revs.append(acq.rev_queue.get_nowait())
        except queue.Empty:
            break
    snap = acq.snapshot()
    assert len(revs) >= 8
    assert snap["revs_seen"] == snap["revs_published"] == len(revs) + snap["revs_dropped"]
    assert snap["revs_rejected"] == 0                        # le tour partiel du démarrage n'est pas compté
    for r in revs:
        assert r.n_raw >= 300 and r.points["angle"][0] <= 10 and r.points["angle"][-1] >= 350
        assert r.t_end_ns > r.t_start_ns and np.all(np.diff(r.points["t_ns"]) >= 0)
        assert (r.points["dist"] > 0).all()                  # les mesures invalides (None) ne sont pas publiées


# ---------------------------------------------------------------- Localizer de bout en bout (sans robot)
def test_localizer_end_to_end_static():
    from core.lidar_acquisition import Revolution

    pose = (1000.0, 1000.0, 0.5)
    q = queue.Queue()
    reports = []
    loc = Localizer(lambda: (*pose, time.time()), q, pose, on_report=reports.append)
    loc.start()
    try:
        time.sleep(0.4)                                      # l'historique de l'EKF se remplit
        t0 = time.time() - 0.15                              # tour mesuré il y a 150 ms
        scan = make_scan(static(*pose), POLE_CIRCLES + [(1400.0, 1300.0, 150.0)], t0=t0)
        q.put(Revolution(0, scan, len(scan), int(scan["t_ns"][0]), int(scan["t_ns"][-1])))
        deadline = time.time() + 2
        while not reports and time.time() < deadline:
            time.sleep(0.02)
    finally:
        loc.stop()
    assert loc.error is None and reports
    r = reports[0]
    assert [m.status for m in r.result.matches] == ["matched"] * 3
    assert r.result.n_clamped == 0 and r.proc_ms < 50

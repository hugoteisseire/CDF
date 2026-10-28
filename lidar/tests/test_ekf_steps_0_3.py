"""Tests sans robot des étapes 0 à 3 du plan d'intégration de l'EKF (lidar simulé).

    python -m pytest tests/test_ekf_steps_0_3.py -v

Le simulateur lance des rayons depuis le lidar sur des cercles (3 poteaux de 100 mm aux coins du
terrain 2 m x 2 m, un gros obstacle type robot adverse, un parasite de la taille d'un poteau),
avec un bruit de ±30 mm sur la distance et 500 points par tour, chaque point daté à son instant.
Les bords du terrain sont supposés sous le lidar (point ouvert du plan : s'ils sont à sa hauteur, un
poteau de coin fusionne avec le bord).
"""
import math
import os
import queue
import sys
import threading
import time
import types

import numpy as np
import pytest

from core.ekf import EkfParams, PoseEKF, wrap_deg, wrap_pi
from core.lidar_clusters import POINT_DTYPE, ClusterParams, cap_points, find_clusters
from core.localizer import LOG_COLUMNS, Localizer
from core.pole_association import AssocParams, associate_poles
from core.relocalize import triangulate
from core.timebase import ClockAligner

T0 = 1000.0                                      # instant de départ des scénarios (s)
POLES = [(0.0, 0.0), (2000.0, 0.0), (0.0, 2000.0)]
POLE_CIRCLES = [(x, y, 50.0) for x, y in POLES]


def P(**kw):
    """EkfParams du terrain simulé (2 m x 2 m, poteaux aux coins). Les valeurs par défaut d'EkfParams sont
    celles de la table 2026 : sans ceci, l'EKF chercherait les poteaux à des endroits où le simulateur n'en met pas."""
    kw.setdefault("poles_mm", tuple(POLES))
    kw.setdefault("field_mm", (0.0, 0.0, 2000.0, 2000.0))
    return EkfParams(**kw)


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
    """Construit un état constant pour les tests de l’EKF."""
    return lambda t: (x, y, th)


def make_ekf(x, y, th, t=T0, hold_until=T0 + 0.3, pos_std=None, th_std_rad=None, params=None):
    """EKF initialisé à la pose donnée et maintenu immobile (historique rempli jusqu'à hold_until)."""
    ekf = PoseEKF(params or P())
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
    """Vérifie le comportement attendu pour le scénario « jacobian matches finite differences »."""
    ekf = PoseEKF(P(lidar_offset_mm=offset, lidar_zero_deg=zero))
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
    """Vérifie le comportement attendu pour le scénario « straight line adds 20mm per meter »."""
    ekf = PoseEKF()
    ekf.reset(0, 0, 0, T0, pos_std_mm=0.0, theta_std_rad=0.0)
    for i in range(1, 201):                                  # 1 m en pas de 5 mm
        ekf.predict(5.0, 0.0, 0.0, T0 + i * 0.005)
    assert ekf.x[0] == pytest.approx(1000.0)
    assert math.sqrt(ekf.P[0, 0]) == pytest.approx(20.0, rel=0.01)    # 2 % de 1 m
    assert math.sqrt(ekf.P[1, 1]) == pytest.approx(20.0, rel=0.01)


def test_rotation_adds_about_1_percent():
    """Vérifie le comportement attendu pour le scénario « rotation adds about 1 percent »."""
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
    """Vérifie le comportement attendu pour le scénario « state at interpolates and flags out of history »."""
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
    """Vérifie le comportement attendu pour le scénario « history is pruned »."""
    ekf = PoseEKF(EkfParams(history_s=0.5))
    ekf.reset(0, 0, 0, T0)
    for i in range(1, 401):
        ekf.predict(1.0, 0.0, 0.0, T0 + i * 0.005)           # 2 s
    assert len(ekf._ht) < 120 and ekf._ht[-1] - ekf._ht[0] < 0.52


def test_clock_aligner():
    """Vérifie le comportement attendu pour le scénario « clock aligner »."""
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
    """Vérifie le comportement attendu pour le scénario « clusters find poles and obstacles »."""
    circles = POLE_CIRCLES + [(1400.0, 1300.0, 150.0), (1200.0, 1500.0, 50.0)]   # hors des lignes de visée des poteaux
    scan = make_scan(static(1000, 1000, 0.5), circles)
    clusters = find_clusters(cap_points(scan))
    ekf = PoseEKF(P())
    sized =[c for c in clusters if 15 <= c.width_mm <= 150]
    for j in range(3):                                       # un cluster de taille poteau à chaque poteau attendu
        d_exp, a_exp, _ = ekf.measurement(np.array([1000, 1000, 0.5]), j)
        near = [c for c in sized if abs(c.dist_mm + 50 - d_exp) < 40 and abs(wrap_deg(c.angle_deg - a_exp)) < 2]
        assert len(near) == 1
    assert any(c.width_mm > 150 for c in clusters)           # le gros obstacle est écarté par sa taille


def test_cluster_across_zero_degrees_is_merged():
    # cap 225° : le poteau (0, 0) est exactement à l'angle lidar 0°, donc coupé en deux par le début du tour
    """Vérifie le comportement attendu pour le scénario « cluster across zero degrees is merged »."""
    scan = make_scan(static(1000, 1000, math.radians(225)), POLE_CIRCLES)
    clusters = find_clusters(scan)
    seam = [c for c in clusters if c.seam]
    assert len(seam) == 1 and seam[0].n_points >= 4
    assert abs(wrap_deg(seam[0].angle_deg)) < 2.0 and 15 <= seam[0].width_mm <= 150


def test_capping_removes_close_and_far_points():
    """Vérifie le comportement attendu pour le scénario « capping removes close and far points »."""
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
    """Vérifie le comportement attendu pour le scénario « cluster outside all windows is ignored »."""
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
    """Vérifie le comportement attendu pour le scénario « two candidates means skip pole and window follows uncertainty »."""
    # Le parasite est placé à 1,5 fois la demi-fenêtre d'angle d'une pose précise : hors de cette fenêtre, mais dans
    # celle d'une pose incertaine. Ainsi le test ne dépend pas de AssocParams.window_scale.
    clean = find_clusters(cap_points(make_scan(static(1000, 1000, 0.0), POLE_CIRCLES)))
    tight_win = associate_poles(make_ekf(1000, 1000, 0.0), clean, T0 + 0.05).matches[1].win_angle_deg
    scan = make_scan(static(1000, 1000, 0.0), POLE_CIRCLES + [_parasite_beside_pole(1.5 * tight_win)])
    clusters = find_clusters(cap_points(scan))
    tight = associate_poles(make_ekf(1000, 1000, 0.0), clusters, T0 + 0.05)
    loose = associate_poles(make_ekf(1000, 1000, 0.0, pos_std=150.0, th_std_rad=math.radians(5)),
                            clusters, T0 + 0.05)
    # Pose précise : le parasite est hors fenêtre -> le poteau est associé.
    assert tight.matches[1].status == "matched"
    # Pose incertaine : fenêtre élargie, le parasite entre dedans -> deux candidats, on saute le poteau.
    assert loose.matches[1].status == "ambiguous" and loose.matches[1].n_candidates == 2
    assert loose.matches[1].win_angle_deg > 3 * tight.matches[1].win_angle_deg


def test_hidden_pole_is_absent_others_still_matched():
    """Vérifie le comportement attendu pour le scénario « hidden pole is absent others still matched »."""
    circles = [POLE_CIRCLES[0], POLE_CIRCLES[2]]            # le poteau (2000, 0) est masqué
    scan = make_scan(static(1000, 1000, 0.0), circles)
    res = associate_poles(make_ekf(1000, 1000, 0.0), find_clusters(cap_points(scan)), T0 + 0.05)
    assert [m.status for m in res.matches] == ["matched", "absent", "matched"]


def test_association_while_moving_uses_pose_at_cluster_time():
    """À 1 m/s, le robot avance de 100 mm pendant un tour : l'association doit utiliser la pose à
    l'instant de chaque cluster, fournie par l'historique de l'EKF."""
    pose_fn = lambda t: (500.0 + 1000.0 * (t - T0), 1000.0, 0.3)
    ekf = PoseEKF(P())
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
        """Faux lidar utilisé pour tester le pipeline sans port série."""
        def __init__(self, port, baud):
            """Initialise l’objet et ses ressources à partir des paramètres fournis."""
            self.stop_event, self.output_queue = asyncio.Event(), asyncio.Queue()
            self.reset_called = False

        def simple_scan(self, make_return_dict=False):
            """Produit en continu les mesures simulées sous la même interface que le lidar réel."""
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
            """Réinitialise l’état de l’interface du lidar."""
            self.reset_called = True

    mod = types.ModuleType("rplidarc1")
    mod.RPLidar = FakeLidar
    return mod


def test_acquisition_publishes_only_complete_revolutions(monkeypatch):
    """Vérifie le comportement attendu pour le scénario « acquisition publishes only complete revolutions »."""
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
    """Vérifie le comportement attendu pour le scénario « localizer end to end static »."""
    from core.lidar_acquisition import Revolution

    pose = (1000.0, 1000.0, 0.5)
    q = queue.Queue()
    reports = []
    loc = Localizer(lambda: (*pose, time.time()), q, pose, params=P(), on_report=reports.append)
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


# ---------------------------------------------------------------- étape 4 : correction datée à l'instant du cluster
def _run_moving(v, timed, n_revs=25):
    """Robot qui avance et tourne à ~29 °/s, odométrie parfaite : le tour n'est traité qu'une fois fini.
    Retourne (erreur de position moyenne mm, erreur de cap moyenne °) une fois le filtre convergé."""
    def truth(t):
        s = t - T0
        return (500.0 + v * 0.8 * s, 1000.0 + v * 0.6 * s, 0.2 + 0.5 * s)

    ekf = PoseEKF(P())
    ekf.reset(*truth(T0), T0)
    errs = []
    for r in range(n_revs):
        t_a, t_b = T0 + r * 0.1, T0 + (r + 1) * 0.1
        scan = make_scan(truth, POLE_CIRCLES, t0=t_a, seed=r)
        k = 1
        while t_a + k * 0.005 <= t_b + 1e-9:
            p0, p1 = truth(t_a + (k - 1) * 0.005), truth(t_a + k * 0.005)
            thm = p0[2] + (p1[2] - p0[2]) / 2
            c, s = math.cos(thm), math.sin(thm)
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            ekf.predict(c * dx + s * dy, -s * dx + c * dy, p1[2] - p0[2], t_a + k * 0.005)
            k += 1
        res = associate_poles(ekf, find_clusters(cap_points(scan)), (t_a + t_b) / 2)
        for m in res.matches:
            if m.status == "matched":
                ekf.correct_pole(m.pole, m.cluster.dist_mm + 50.0, m.cluster.angle_deg,
                                 t_s=m.cluster.t_ns / 1e9 if timed else None)
        tx, ty, tth = truth(t_b)
        x, y, th = ekf.pose
        errs.append((math.hypot(x - tx, y - ty), abs(math.degrees(wrap_pi(th - tth)))))
    e = np.array(errs[5:])
    return float(e[:, 0].mean()), float(e[:, 1].mean())


def test_correction_at_cluster_time_beats_current_pose_while_moving():
    """Comparer la mesure d'un cluster à la pose COURANTE (fin de tour) au lieu de la pose de son instant biaise
    la correction dès que le robot bouge : plusieurs cm et ~1,5° avec une odométrie pourtant parfaite."""
    pos_t, th_t = _run_moving(300.0, timed=True)
    pos_c, th_c = _run_moving(300.0, timed=False)
    assert pos_t < 8.0 and th_t < 0.2
    assert pos_c > 15.0 and pos_c > 3 * pos_t


def test_correction_keeps_history_consistent_with_current_pose():
    """Après une correction, tout l'historique récent est décalé comme la pose courante : le pôle suivant du même
    tour compare sa mesure à une pose déjà corrigée, et les instants passés restent cohérents avec le présent."""
    ekf = make_ekf(1040.0, 980.0, 0.5, pos_std=50.0)           # l'EKF se trompe de 40 mm / 20 mm
    scan = make_scan(static(1000.0, 1000.0, 0.5), POLE_CIRCLES, t0=T0)
    res = associate_poles(ekf, find_clusters(cap_points(scan)), T0 + 0.05)
    n = 0
    for m in res.matches:
        assert m.status == "matched"
        ok, _ = ekf.correct_pole(m.pole, m.cluster.dist_mm + 50.0, m.cluster.angle_deg, t_s=m.cluster.t_ns / 1e9)
        n += ok
    assert n == 3
    x, y, _ = ekf.pose
    assert math.hypot(x - 1000.0, y - 1000.0) < 25.0           # erreur de départ ~45 mm ; mesuré : ~12 mm en moyenne
    assert ekf.state_at(T0).x == pytest.approx(ekf.x, abs=1e-9)    # robot immobile : tout l'historique a suivi le décalage
    assert ekf.state_at(T0 + 0.15).x == pytest.approx(ekf.x, abs=1e-9)


def test_heading_stays_continuous_across_pi_after_correction():
    ekf = make_ekf(1000.0, 1000.0, 3.12, th_std_rad=math.radians(3.0))
    scan = make_scan(static(1000.0, 1000.0, 3.16), POLE_CIRCLES, t0=T0)    # vrai cap : juste au-delà de π
    res = associate_poles(ekf, find_clusters(cap_points(scan)), T0 + 0.05)
    for m in res.matches:
        if m.status == "matched":
            ekf.correct_pole(m.pole, m.cluster.dist_mm + 50.0, m.cluster.angle_deg, t_s=m.cluster.t_ns / 1e9)
    assert ekf.x[2] > 3.0                                       # pas ramené à ~-3,1 : l'historique est interpolé
    assert max(abs(np.diff([h[2] for h in ekf._hx]))) < 1.0
    assert abs(wrap_pi(ekf.pose[2] - 3.16)) < 0.03


# ---------------------------------------------------------------- étape 6 : relocalisation
def test_triangulate_finds_pose_from_scratch():
    """Sans aucune pose de référence : jamais de pose fausse, et toujours une pose quand les 3 poteaux sont vus."""
    rng = np.random.default_rng(7)
    found_n = 0
    worst_pos = worst_th = 0.0
    for k in range(40):
        x, y, th = rng.uniform(400, 1600), rng.uniform(400, 1600), rng.uniform(-math.pi, math.pi)
        circles = list(POLE_CIRCLES) + [(rng.uniform(300, 1700), rng.uniform(300, 1700), 150.0)]
        clusters = find_clusters(cap_points(make_scan(static(x, y, th), circles, seed=k)))
        ref = PoseEKF(P())
        truth_pose = np.array([x, y, th])

        def seen(j):          # le poteau j est-il réellement vu ? (le bord d'un obstacle peut créer de faux clusters)
            d_exp, a_exp, _ = ref.measurement(truth_pose, j)
            return any(c.n_points >= 2 and 15 <= c.width_mm <= 150 and abs(c.dist_mm + 50 - d_exp) < 150
                       and abs(wrap_deg(c.angle_deg - a_exp)) < 3 for c in clusters)
        n_seen = sum(seen(j) for j in range(3))
        found = triangulate(PoseEKF(P()), clusters)
        if n_seen == 3:
            assert found is not None and found.n_inliers == 3, f"pose {k} : 3 poteaux vus mais rien trouvé"
        if found is not None:
            found_n += 1
            worst_pos = max(worst_pos, math.hypot(found.x_mm - x, found.y_mm - y))
            worst_th = max(worst_th, abs(math.degrees(wrap_pi(found.theta_rad - th))))
    assert found_n >= 20
    assert worst_pos < 50.0 and worst_th < 1.0                  # mesuré : ~14 mm et ~0,2° au pire


def test_triangulate_refuses_ambiguous_two_pole_case_unless_prior_is_clear():
    """Avec 2 poteaux symétriques, deux poses expliquent les mesures : on ne tranche que si la pose du filtre est
    proche d'une seule et loin de l'autre ; sinon on refuse plutôt que de deviner."""
    truth = (400.0, 1200.0, -1.0)
    scan = make_scan(static(*truth), POLE_CIRCLES[:2])           # le poteau (0, 2000) est masqué
    clusters = find_clusters(cap_points(scan))

    assert triangulate(PoseEKF(P()), clusters) is None           # aucune pose de référence

    near = PoseEKF(P())
    near.reset(440.0, 1180.0, -0.9, T0)                          # filtre à ~45 mm : tranche (l'autre solution est à 1,2 m)
    found = triangulate(near, clusters)
    assert found is not None and found.ambiguous and found.n_inliers == 2
    assert math.hypot(found.x_mm - truth[0], found.y_mm - truth[1]) < 100.0

    far = PoseEKF(P())
    far.reset(1500.0, 300.0, 0.0, T0)                            # filtre très loin des deux solutions
    assert triangulate(far, clusters) is None

    assert triangulate(PoseEKF(P()), find_clusters(cap_points(make_scan(static(*truth), POLE_CIRCLES[:1])))) is None


def test_triangulate_near_symmetry_axis_never_guesses():
    """Robot près de l'axe de symétrie : les deux solutions sont à ~150 mm, moins que l'erreur possible du filtre."""
    truth = (916.0, 1023.0, -2.0)
    clusters = find_clusters(cap_points(make_scan(static(*truth), POLE_CIRCLES[:2])))
    ekf = PoseEKF(P())
    ekf.reset(1010.0, 1139.0, -1.5, T0)                          # ce cas faisait choisir la mauvaise solution
    found = triangulate(ekf, clusters)
    assert found is None or math.hypot(found.x_mm - truth[0], found.y_mm - truth[1]) < 100.0


def _feed(loc, q, pose, circles, n, period=0.1):
    from core.lidar_acquisition import Revolution
    for k in range(n):
        scan = make_scan(static(*pose), circles, t0=time.time() - 0.12, seed=k)
        q.put(Revolution(k, scan, len(scan), int(scan["t_ns"][0]), int(scan["t_ns"][-1])))
        time.sleep(period)


def test_localizer_relocalizes_when_lost():
    truth = (1000.0, 1000.0, 0.5)
    wrong = (1350.0, 650.0, 1.1)                 # départ très faux : les fenêtres (±15°, ±400 mm) ne voient rien
    q, reports = queue.Queue(), []
    loc = Localizer(lambda: (0.0, 0.0, 0.0, time.time()), q, wrong, params=P(lost_after_s=0.3),
                    on_report=reports.append)
    loc.start()
    try:
        time.sleep(0.3)
        _feed(loc, q, truth, POLE_CIRCLES, 14)
        time.sleep(0.3)
    finally:
        loc.stop()
    assert loc.error is None and reports
    assert any(r.relocalized for r in reports) and loc.relocalizations >= 1
    s = loc.status()
    x, y, th = s["pose"]
    assert math.hypot(x - truth[0], y - truth[1]) < 40.0 and abs(math.degrees(wrap_pi(th - truth[2]))) < 2.0
    assert not s["lost"]


def test_localizer_is_not_lost_when_poles_are_only_masked():
    pose = (1000.0, 1000.0, 0.5)
    q = queue.Queue()
    loc = Localizer(lambda: (*pose, time.time()), q, pose, params=P(lost_after_s=0.3))
    loc.start()
    try:
        time.sleep(0.3)
        _feed(loc, q, pose, [(1400.0, 1300.0, 150.0)], 8)   # aucun cluster de la taille d'un poteau
    finally:
        loc.stop()
    assert loc.error is None and loc.revs_processed >= 6
    assert not loc.lost and loc.relocalizations == 0


def test_report_has_all_fields_and_journal_is_written(tmp_path):
    from core.lidar_acquisition import Revolution

    pose = (1000.0, 1000.0, 0.5)
    q, reports = queue.Queue(), []
    log = tmp_path / "journal.csv"
    loc = Localizer(lambda: (*pose, time.time()), q, pose, params=P(), on_report=reports.append, log_path=str(log))
    loc.start()
    try:
        time.sleep(0.4)
        scan = make_scan(static(*pose), POLE_CIRCLES, t0=time.time() - 0.15)
        q.put(Revolution(0, scan, len(scan), int(scan["t_ns"][0]), int(scan["t_ns"][-1])))
        deadline = time.time() + 2
        while not reports and time.time() < deadline:
            time.sleep(0.02)
    finally:
        loc.stop()
    assert loc.error is None and loc.log_error is None and reports
    r = reports[0]
    assert r.corrections_accepted == 3 and r.corrections_rejected == 0 and len(r.mahal) == 3
    assert r.odo_hz >= 0.0                                              # champ lu par le visualiseur de test_fusion
    assert {"cap_points", "find_clusters", "associate_poles", "correct", "relocalize", "total"} <= set(r.timings_ms)
    assert r.reliable and not r.lost and not r.relocalized
    lines = log.read_text(encoding="utf-8").splitlines()
    assert lines[0].split(",") == list(LOG_COLUMNS) and len(lines) == 2


def test_triangulate_never_returns_a_wrong_pose_with_parasites():
    """Garantie qui compte : avec des parasites de la taille d'un poteau (qui masquent aussi des poteaux), la
    triangulation peut REFUSER de répondre, mais ne doit jamais renvoyer une pose fausse. Elle doit aussi
    répondre dans la grande majorité des cas où il n'y a que 3 parasites (mesuré : 31 poses justes sur 40)."""
    rng = np.random.default_rng(11)
    good = wrong = refused = 0
    for k in range(40):
        x, y, th = rng.uniform(500, 1500), rng.uniform(500, 1500), rng.uniform(-math.pi, math.pi)
        circles = list(POLE_CIRCLES) + [(rng.uniform(250, 1750), rng.uniform(250, 1750), 50.0) for _ in range(3)]
        found = triangulate(PoseEKF(P()), find_clusters(cap_points(make_scan(static(x, y, th), circles, seed=k))))
        if found is None:
            refused += 1
        elif math.hypot(found.x_mm - x, found.y_mm - y) < 60.0 and abs(math.degrees(wrap_pi(found.theta_rad - th))) < 2.0:
            good += 1
        else:
            wrong += 1
    assert wrong == 0
    assert good >= 0.6 * 40


# ---------------------------------------------------------------- étape 0 : acquisition dans son propre processus
def _fake_lidar_factory(port, baud):
    """Importable par le processus fils : le faux lidar de _fake_rplidar_module, sans toucher à sys.modules."""
    return _fake_rplidar_module().RPLidar(port, baud)


def _failing_lidar_factory(port, baud):
    raise ConnectionError("port série introuvable (test)")


def _lidar_process(**kw):
    from core.lidar_process import LidarProcess
    return LidarProcess("fake", factory=f"{__name__}:_fake_lidar_factory", **kw)


def test_lidar_process_publishes_complete_revolutions_from_another_process():
    """Le lidar est lu dans un autre processus (son propre GIL) et les tours complets reviennent à l'identique."""
    acq = _lidar_process()
    acq.start()
    try:
        time.sleep(1.5)
        acq.snapshot()
        time.sleep(1.5)
        s = acq.snapshot()
        revs = []
        while True:
            try:
                revs.append(acq.rev_queue.get_nowait())
            except queue.Empty:
                break
        assert acq.alive
        proc = acq._proc
    finally:
        acq.stop()
    assert acq.error is None
    assert s["pid"] is not None and s["pid"] != os.getpid()
    assert proc.poll() is not None                          # arrêt propre : le fils est bien terminé
    assert len(revs) >= 8 and s["freq_hz"] > 7.0
    assert s["revs_rejected"] == 0 and s["revs_seen"] >= len(revs)
    assert s["lib_errors"] == 0 and s["max_gap_ms"] < 200.0
    for r in revs:
        assert r.n_raw >= 300 and r.points["angle"][0] <= 10 and r.points["angle"][-1] >= 350
        assert r.t_end_ns > r.t_start_ns and np.all(np.diff(r.points["t_ns"]) >= 0)
        assert abs(r.t_end_ns / 1e9 - time.time()) < 10.0     # horodatages en epoch, comme avec LidarAcquisition
    assert [r.number for r in revs] == sorted(r.number for r in revs)


def test_lidar_process_is_not_slowed_by_busy_threads_in_the_main_process():
    """Le défaut observé sur le robot : des threads Python occupés (affichage, filtre, odométrie) dans le MÊME
    processus que la lecture série la privent du GIL (mesuré sur un port virtuel : 10 tours/s seul, 0 avec un seul
    thread occupé). Avec l'acquisition dans son propre processus, cette charge ne doit plus compter."""
    acq = _lidar_process()
    acq.start()
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            x = 0
            for _ in range(20000):
                x += 1

    spinners = [threading.Thread(target=spin, daemon=True) for _ in range(3)]
    try:
        for t in spinners:
            t.start()
        time.sleep(1.0)
        acq.snapshot()
        time.sleep(3.0)
        s = acq.snapshot()
    finally:
        stop.set()
        acq.stop()
    assert acq.error is None
    assert s["freq_hz"] > 7.0 and s["revs_rejected"] == 0 and s["lib_errors"] == 0


def test_lidar_process_death_is_detected():
    acq = _lidar_process()
    acq.start()
    try:
        time.sleep(0.5)
        assert acq.alive and acq.error is None
        acq._proc.kill()
        deadline = time.time() + 3
        while (acq.alive or acq.error is None) and time.time() < deadline:
            time.sleep(0.05)
        assert not acq.alive and acq.error is not None
    finally:
        acq.stop()                                           # ne doit pas bloquer ni lever après la mort du fils


def test_lidar_process_start_reports_a_serial_port_error():
    from core.lidar_process import LidarProcess
    acq = LidarProcess("fake", factory=f"{__name__}:_failing_lidar_factory")
    with pytest.raises(ConnectionError, match="introuvable"):
        acq.start()
    assert acq._proc.wait(5) is not None                     # le fils ne reste pas orphelin


def test_error_counter_ignores_the_library_info_message():
    """« In waiting: N » est un message d'information (niveau WARNING) émis à chaque démarrage de scan : il ne doit
    pas compter comme erreur, sinon lib_errors vaut au moins 1 et le critère « aucune erreur » ne peut jamais passer."""
    import logging
    from core.lidar_acquisition import _ErrorCounter

    c = _ErrorCounter()
    c.emit(logging.LogRecord("x", logging.WARNING, __file__, 1, "In waiting: %d", (3,), None))
    assert c.count == 0
    c.emit(logging.LogRecord("x", logging.WARNING, __file__, 1, "Verification bytes not matching.", (), None))
    assert c.count == 1


# ---------------------------------------------------------------- fenêtre de recherche des poteaux : coefficient 3
def test_window_scale_widens_the_search_window_and_catches_an_overconfident_filter():
    """Le filtre se croit à ±20 mm / ±1° alors qu'il est à ~100 mm et 6° de la vérité (angle de montage, latence...) :
    avec la fenêtre du plan (coefficient 1) il ne voit plus ses poteaux ; avec le coefficient 3 (défaut) il les retrouve."""
    assert AssocParams().window_scale == 3.0
    x, y, th = 1000.0, 1000.0, 0.5
    clusters = find_clusters(cap_points(make_scan(static(x, y, th), POLE_CIRCLES)))
    ekf = make_ekf(x + 90.0, y - 40.0, th + math.radians(6.0))
    old = associate_poles(ekf, clusters, T0 + 0.05, AssocParams(window_scale=1.0))
    new = associate_poles(ekf, clusters, T0 + 0.05)
    assert sum(m.status == "matched" for m in old.matches) < 3
    assert [m.status for m in new.matches] == ["matched"] * 3
    for o, n in zip(old.matches, new.matches):               # exactement 3 fois, plafonds compris
        assert n.win_dist_mm == pytest.approx(3.0 * o.win_dist_mm)
        assert n.win_angle_deg == pytest.approx(3.0 * o.win_angle_deg)


def test_hard_correction_guard_rejects_large_pose_jump_even_when_chi2_passes():
    """Une covariance élevée peut rendre le chi² permissif : le garde-fou borne aussi le déplacement proposé."""
    ekf = make_ekf(1000.0, 1000.0, 0.0, pos_std=500.0, th_std_rad=math.radians(20.0))
    before = ekf.x.copy()
    # Mesure plausible au regard de la covariance, mais dont la correction EKF serait trop importante.
    expected_dist, expected_angle, _ = ekf.measurement(ekf.x, 0)
    ok, mahal = ekf.correct_pole(
        0, expected_dist + 100.0, expected_angle, t_s=T0 + 0.1
    )
    assert mahal < 9.21
    assert not ok
    assert ekf.x == pytest.approx(before)


def test_wider_window_may_pick_a_wrong_cluster_but_the_gate_refuses_to_correct_with_it():
    """Prix du coefficient 3 : si un poteau est masqué, un parasite de sa taille à ~250 mm peut entrer dans la
    fenêtre et être pris pour lui. La correction de pose, elle, reste protégée par le gating du chi² : la pose ne bouge pas."""
    circles = [POLE_CIRCLES[0], POLE_CIRCLES[2], (2180.0, 180.0, 50.0)]      # le poteau (2000, 0) est masqué
    clusters = find_clusters(cap_points(make_scan(static(1000.0, 1000.0, 0.0), circles)))
    ekf = make_ekf(1000.0, 1000.0, 0.0)
    assert associate_poles(ekf, clusters, T0 + 0.05, AssocParams(window_scale=1.0)).matches[1].status == "absent"
    m = associate_poles(ekf, clusters, T0 + 0.05).matches[1]
    assert m.status == "matched"                                              # le parasite est pris pour le poteau...
    before = ekf.x.copy()
    ok, mahal = ekf.correct_pole(1, m.cluster.dist_mm + 50.0, m.cluster.angle_deg, t_s=m.cluster.t_ns / 1e9)
    assert not ok and mahal > 9.21                                            # ... mais la correction est refusée
    assert ekf.x == pytest.approx(before)

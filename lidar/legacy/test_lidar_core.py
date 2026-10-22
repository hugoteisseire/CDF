#!/usr/bin/env python3
"""Tests sans materiel : python3 test_lidar_core.py (aucune dependance de test)."""

import math
import threading
import time

import numpy as np

from lidar_core import Closed, LidarConfig, LidarService, fit_pole
from lidar_sim import POLES, SimRPLidar, pole_truth


def collect(service, seconds, sub=None):
    sub = sub or service.subscribe(maxlen=64)
    out, t_end = [], time.monotonic() + seconds
    while time.monotonic() < t_end:
        s = sub.get(0.2)
        if s is not None:
            out.append(s)
    return out


def check(name, ok, detail=""):
    print(f"[{'OK' if ok else 'ECHEC'}] {name}  {detail}")
    return ok


def static_pose(x, y, th):
    return lambda t: (np.full_like(t, x), np.full_like(t, y), np.full_like(t, th))


def test_fit():
    rng = np.random.default_rng(0)
    ok = True
    for D in (1000.0, 2000.0, 2800.0):
        errs = []
        for _ in range(1000):
            ang = math.radians(30.0)
            c = D * np.array([math.cos(ang), math.sin(ang)])
            off = rng.uniform(-0.36, 0.36)
            ph, rr = [], []
            for p in ang + np.radians(np.arange(-3, 3.01, 0.72) + off):
                u = np.array([math.cos(p), math.sin(p)])
                b = u @ (-c)
                disc = b * b - (c @ c - 50.0**2)
                if disc > 0:
                    ph.append(p)
                    rr.append(-b - math.sqrt(disc))
            if len(ph) < 3:
                continue
            rr = np.array(rr) + rng.normal(0, 30.0, len(ph))  # bruit de 30 mm sur la distance
            psi, d, _ = fit_pole(np.array(ph), rr, 50.0)
            errs.append((d - D, math.degrees(psi - ang), len(ph)))
        e = np.array(errs)
        good = abs(e[:, 0].mean()) < 5 and e[:, 0].std() < 25 and e[:, 1].std() < 0.4
        ok &= check(f"ajustement du poteau a {D/1000:.1f} m ({e[:, 2].mean():.0f} points, bruit 30 mm)", good,
                    f"biais r {e[:, 0].mean():+.1f} mm, ecart-type r {e[:, 0].std():.1f} mm, "
                    f"ecart-type angle {e[:, 1].std():.2f} deg")
    return ok


def test_static():
    pose = (0.6, 0.8, 0.3)
    sim = SimRPLidar(pose_fn=static_pose(*pose))
    with LidarService(sim) as svc:
        scans = collect(svc, 5.0)
        st = svc.stats()
    ok = True
    seqs = [s.seq for s in scans]
    ok &= check("numeros de tour contigus", seqs == list(range(seqs[0], seqs[0] + len(seqs))), f"{len(seqs)} tours")
    ok &= check("frequence de tour ~10 Hz", abs(st["rev_hz"] - 10.0) < 0.3, f"{st['rev_hz']:.2f} Hz")
    per = np.array([s.raw.period for s in scans[1:]])
    ok &= check("duree de tour ~100 ms", abs(per.mean() - 0.1) < 0.002, f"moy {per.mean()*1000:.1f} ms, ecart-type {per.std()*1000:.2f} ms")
    ok &= check("horodatages croissants dans un tour", all(np.all(np.diff(s.raw.t) >= 0) for s in scans))
    n3 = sum(len(s.clusters) == 3 for s in scans)
    ok &= check("3 poteaux detectes a chaque tour", n3 >= 0.95 * len(scans), f"{n3}/{len(scans)} tours")
    truth = pole_truth(pose)
    errs = {i: [] for i in range(3)}
    for s in scans:
        for i, (pt, rt) in enumerate(truth):
            best = min(s.clusters, key=lambda c: abs(math.remainder(c.phi_rad - pt, 2 * math.pi)), default=None)
            if best and abs(math.remainder(best.phi_rad - pt, 2 * math.pi)) < math.radians(3):
                errs[i].append((best.r_m - rt, math.degrees(math.remainder(best.phi_rad - pt, 2 * math.pi)), best.n))
    for i, (pt, rt) in enumerate(truth):
        e = np.array(errs[i])
        print(f"      poteau {i} ({POLES[i]}), r={rt:.2f} m, {e[:, 2].mean():.1f} pts : "
              f"erreur r {1000*e[:, 0].mean():+.1f} mm (ecart-type {1000*e[:, 0].std():.1f}), "
              f"erreur angle {e[:, 1].mean():+.2f} deg (ecart-type {e[:, 1].std():.2f})")
    ok &= check("traitement < 5 ms par tour", st["proc_ms"] < 5.0, f"{st['proc_ms']:.2f} ms")
    ok &= check("aucun tour perdu avant traitement", st["proc_skipped"] == 0)
    return ok


def test_slow_consumer():
    sim = SimRPLidar(pose_fn=static_pose(0.6, 0.8, 0.3))
    with LidarService(sim) as svc:
        fast, slow = svc.subscribe(64), svc.subscribe(3)
        stop = threading.Event()

        def slow_loop():
            while not stop.is_set():
                try:
                    if slow.get(0.2) is not None:
                        time.sleep(0.35)  # consommateur tres lent (trace qui bloque)
                except Closed:
                    return

        th = threading.Thread(target=slow_loop)
        th.start()
        scans = collect(svc, 4.0, fast)
        st = svc.stats()
        stop.set()
        th.join()
    seqs = [s.seq for s in scans]
    ok = check("consommateur rapide : aucun tour perdu", fast.dropped == 0 and seqs == list(range(seqs[0], seqs[0] + len(seqs))))
    ok &= check("consommateur lent : perd ses propres tours", slow.dropped > 5, f"{slow.dropped} tours perdus")
    ok &= check("l'acquisition n'est pas ralentie par le consommateur lent", abs(st["rev_hz"] - 10.0) < 0.3, f"{st['rev_hz']:.2f} Hz")
    return ok


def test_wrap():
    # pose choisie pour que le poteau (0,0) soit pile a 0 deg lidar, sur la coupure du tour
    sim = SimRPLidar(pose_fn=static_pose(1.0, 1.0, math.radians(-135.0)))
    with LidarService(sim) as svc:
        scans = collect(svc, 3.0)
    n3 = sum(len(s.clusters) == 3 for s in scans)
    near0 = sum(any(min(c.a_deg, 360 - c.a_deg) < 3 for c in s.clusters) for s in scans)
    return check("poteau a cheval sur la coupure 360->0 recolle", n3 >= 0.9 * len(scans) and near0 >= 0.9 * len(scans),
                 f"3 clusters : {n3}/{len(scans)}, un cluster a 0 deg : {near0}/{len(scans)}")


def test_capping():
    sim = SimRPLidar(pose_fn=static_pose(0.6, 0.8, 0.3))
    cfg = LidarConfig(rmax_mm=1500.0, blind_sectors_deg=((350.0, 10.0),))
    with LidarService(sim, cfg) as svc:
        scans = collect(svc, 2.0)
    s = scans[-1]
    ok = check("capping : plus de point au-dela de rmax", s.r_m.max() <= 1.5)
    ok &= check("capping : secteur aveugle (350-10 deg) vide", not np.any((s.angle_deg >= 350) | (s.angle_deg <= 10)))
    ok &= check("capping : les points ecartes sont comptes", s.n_capped > 0, f"{s.n_capped} ecartes")
    return ok


def test_failure():
    class BrokenLidar(SimRPLidar):
        async def simple_scan(self):
            await asyncio.sleep(0.5)
            raise RuntimeError("liaison serie perdue")

    import asyncio
    svc = LidarService(BrokenLidar()).start()
    sub = svc.subscribe()
    closed = False
    t_end = time.monotonic() + 3
    while time.monotonic() < t_end:
        try:
            sub.get(0.2)
        except Closed:
            closed = True
            break
    ok = check("panne du lidar : les consommateurs sont prevenus (Closed)", closed)
    ok &= check("panne du lidar : l'erreur est remontee", isinstance(svc.error, BaseException), repr(svc.error))
    svc.stop()
    ok &= check("arret propre : threads termines", not svc._acq.alive() and not svc._proc.alive())
    return ok


if __name__ == "__main__":
    results = []
    for t in (test_fit, test_static, test_slow_consumer, test_wrap, test_capping, test_failure):
        print(f"\n== {t.__name__}")
        results.append(bool(t()))
    print(f"\n{sum(results)}/{len(results)} groupes de tests reussis")
    raise SystemExit(0 if all(results) else 1)

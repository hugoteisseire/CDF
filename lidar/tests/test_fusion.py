"""Fusion de capteurs sur le robot, étapes 0 à 3 du plan : acquisition lidar, prédiction de l'EKF par
l'odométrie, association des poteaux. Aucune correction de pose pour l'instant (étape 4).

    python -m tests.test_fusion --x0 1000 --y0 1000 --theta0-deg 0
    python -m tests.test_fusion --x0 1000 --y0 1000 --theta0-deg 0 --drive   # + aller-retour de test_odometry

(x0, y0, theta0) = pose du robot sur le terrain au démarrage, repère du plan : origine sur le poteau
(0, 0), poteaux en (2000, 0) et (0, 2000), θ en sens anti-horaire, 0 = axe x du terrain.
Le dossier rplidarc1/ (avec t_rx_ns) doit être importable depuis la racine du dépôt.

Ce que le test affiche, dans l'ordre du plan :
  - étape 0 : une ligne par seconde (tours/s, points/s, tours rejetés, erreurs de la lib) ;
  - étape 3 : une ligne par tour (poteau associé ou non, écart mesure - attendu, incertitude).
Robot immobile, les 3 poteaux doivent être "ok" à chaque tour, avec des écarts de quelques mm / dixièmes
de degré. Les tours rejetés doivent rester à 0 et le temps de traitement très inférieur à 100 ms.
"""
import argparse
import math
import time

import numpy as np

from core.lidar_acquisition import LidarAcquisition
from core.localizer import Localizer

TAGS = {"matched": "ok", "absent": "absent", "ambiguous": "ambigu"}


def build_base():
    """Même montage que tests/test_odometry.py."""
    import can
    from mks_servo_can import MksServo
    from core.holo_base import Robot
    from core.holonomic_base import HolonomicBase
    from core.motor_controller import Motor, MotorGroup

    gear_ratio = 48 / 15
    bus = can.interface.Bus(interface="socketcan", channel="can0")
    notifier = can.Notifier(bus, [])
    motors = [Motor(bus, i, MksServo(bus, notifier, i), gear_ratio=gear_ratio, notifier=notifier)
              for i in (1, 2, 3)]
    robot = Robot(pos=[0, 0], angle=0, radius=106.5,
                  wheel_angles=[np.radians(120), np.radians(240), np.radians(0)])
    group = MotorGroup(motors, robot)
    return bus, notifier, group, HolonomicBase(robot, group)


def format_report(r) -> str:
    parts = []
    for m in r.result.matches:
        if m.status == "matched":
            parts.append(f"P{m.pole + 1} ok d={m.d_dist_mm:+4.0f}mm a={m.d_angle_deg:+5.2f}deg")
        elif m.status == "ambiguous":
            parts.append(f"P{m.pole + 1} ambigu({m.n_candidates})")
        else:
            parts.append(f"P{m.pole + 1} absent")
    x, y, th = r.pose
    sx, sy, st = r.std
    return (f"[{time.strftime('%H:%M:%S', time.localtime(r.t_s))}] tour {r.rev_number:5d} | "
            f"{r.n_clusters:2d} clusters ({r.result.n_size_ok} taille poteau) | " + " | ".join(parts) +
            f" | pose ({x:6.0f},{y:6.0f},{math.degrees(th):6.1f}deg) sigma ({sx:3.0f}mm,{sy:3.0f}mm,{st:4.1f}deg)"
            f" | {r.proc_ms:4.1f} ms" + (f" | HORS HISTORIQUE x{r.result.n_clamped}" if r.result.n_clamped else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x0", type=float, required=True, help="x initial du robot sur le terrain (mm)")
    ap.add_argument("--y0", type=float, required=True, help="y initial (mm)")
    ap.add_argument("--theta0-deg", type=float, required=True, help="cap initial (deg, anti-horaire)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--duration", type=float, default=60.0, help="durée du test (s)")
    ap.add_argument("--every", type=int, default=5, help="affiche 1 tour sur N (défaut 5 = 2 lignes/s)")
    ap.add_argument("--odometry-hz", type=float, default=200.0)
    ap.add_argument("--drive", action="store_true", help="lance l'aller-retour de test_odometry.check_pose")
    args = ap.parse_args()

    bus, notifier, group, base = build_base()
    acq = LidarAcquisition(args.port)
    loc = None

    def on_report(r):
        if r.rev_number % args.every == 0:
            print(format_report(r))

    try:
        base.start_odometry(args.odometry_hz)
        time.sleep(0.3)
        acq.start()
        loc = Localizer(base.pose, acq.rev_queue,
                        (args.x0, args.y0, math.radians(args.theta0_deg)), on_report=on_report)
        loc.start()
        print(f"Pose initiale : ({args.x0:.0f} mm, {args.y0:.0f} mm, {args.theta0_deg:.1f} deg)")
        if args.drive:
            from tests.test_odometry import check_pose
            check_pose(base)
        t_end = time.time() + args.duration
        while time.time() < t_end and acq.alive and loc.error is None:
            time.sleep(1.0)
            s = acq.snapshot()
            print(f"  lidar : {s['freq_hz']:5.2f} tours/s | {s['pts_per_s']:5.0f} pts/s | vus {s['revs_seen']} "
                  f"publies {s['revs_published']} rejetes {s['revs_rejected']} perdus {s['revs_dropped']} | "
                  f"erreurs lib {s['lib_errors']}")
        if acq.error is not None:
            print(f"Erreur du thread lidar : {acq.error!r}")
        if loc.error is not None:
            print(f"Erreur du thread de fusion : {loc.error!r}")
        if acq.clock.recalibrated or loc.odo_clock.recalibrated:
            print("Note : horloge(s) non epoch recalée(s) sur l'heure système "
                  f"(lidar : {acq.clock.recalibrated}, odométrie : {loc.odo_clock.recalibrated}).")
    except KeyboardInterrupt:
        pass
    finally:
        if loc is not None:
            loc.stop()
        acq.stop()
        group.stop_all(255)
        base.stop_odometry()
        group.close()
        notifier.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()

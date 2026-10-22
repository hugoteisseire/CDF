#!/usr/bin/env python3
"""
Rangement de simu/base_holo : déplace dans legacy/ ce qui n'est plus utilisé, garde les exemples
concrets, réécrit tests/test_velocity_square.py (nouvelle API MotorGroup) et met le README à jour.

Usage (depuis simu/base_holo) :
    python tidy_base_holo.py            # simulation : affiche ce qui serait fait
    python tidy_base_holo.py --apply    # exécute (git mv si dépôt git, sinon déplacement simple)
"""
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
APPLY = "--apply" in sys.argv

# source -> destination (relatives à base_holo)
MOVES = {
    # Utilisent l'ancien mode asynchrone / méthodes supprimées, ou doublons de ce qui existe :
    "examples/example_motor_class.py":    "legacy/example_motor_class.py",
    "examples/example_robot_movements.py": "legacy/example_robot_movements.py",   # remplacé par MotorGroup.move_direction
    "examples/example_wheel_distances.py": "legacy/example_wheel_distances.py",   # doublon de tests/test_wheel_distances
    "tests/test_mot.py":                  "legacy/test_mot.py",                    # dépend de legacy.lib_moteur
    "tests/test_move_distance.py":        "legacy/test_move_distance.py",          # passe enable_async_control ; remplacé par test_odometry
    "docs/CHANGELOG_MOVE_DISTANCE.md":    "legacy/docs/CHANGELOG_MOVE_DISTANCE.md",
    "docs/README_MOTOR_ASYNC.md":         "legacy/docs/README_MOTOR_ASYNC.md",
}

LEGACY_README = """# legacy

Code **non maintenu**, conservé pour mémoire. Il peut ne plus fonctionner avec l'API actuelle
(mode asynchrone de `Motor`, `move_synchronized*` et `set_velocities` ont été retirés).
À consulter, pas à lancer.
"""

README = """# base_holo

Contrôle de la base holonome 3 roues (servos MKS sur bus CAN, socketcan 1 Mbit/s).

```
core/       holo_base.py       cinématique (Robot, distances / vitesses de roue)
            motor_controller.py  Motor (protocole MKS) et MotorGroup (mouvements, watchdog, poller)
            holonomic_base.py  HolonomicBase : façade au-dessus de Robot + MotorGroup, odométrie
examples/   teleop_zqsd.py     pilotage au clavier + affichage de la pose
tests/      test_odometry.py        validation / recalage des paramètres du robot (à relancer
                                    quand la mécanique change : signe, rayon, diamètre de roue)
            test_velocity_square.py carré en mode vitesse (MotorGroup.move_velocity_direction)
            test_wheel_distances.py calcul pur, sans matériel
tools/      calculate_motor_params.py, simulation_traj.py (simulation pygame)
legacy/     ancienne architecture et exemples abandonnés (non maintenus)
```

Lancer depuis ce dossier, en mode module :

```
python -m tests.test_wheel_distances      # sans matériel
python -m tests.test_odometry             # robot, bus CAN actif
python -m tests.test_velocity_square
python -m examples.teleop_zqsd
```

Après un changement mécanique : relancer `tests/test_odometry` (signe de position, rayon,
diamètre de roue) puis mettre à jour `wheel_diameter_mm`, `gear_ratio` et `radius` dans les scripts.
"""

VELOCITY_SQUARE = '''"""Carré en mode vitesse avec MotorGroup.move_velocity_direction (boucle ouverte, temporisé).

    python -m tests.test_velocity_square

Le robot suit 4 côtés (EST, NORD, OUEST, SUD) ; l'odométrie affiche la pose de départ et d'arrivée :
l'écart mesure la dérive sur un carré. Les consignes sont renvoyées toutes les 50 ms : le
watchdog freine tout seul si le programme s'arrête.
"""
import math
import time

import numpy as np

SQUARE_ANGLES = [("EST", 0.0), ("NORD", math.pi / 2), ("OUEST", math.pi), ("SUD", -math.pi / 2)]


def drive_leg(group, angle, speed_mm_s, duration_s, ramp_acc=220, refresh_s=0.05):
    """Roule dans la direction `angle` pendant `duration_s`, puis freine avec les mêmes rampes
    par roue qu'à l'accélération (décélération proportionnelle)."""
    t0 = time.perf_counter()
    cmds = None
    while time.perf_counter() - t0 < duration_s:
        cmds = group.move_velocity_direction(angle, speed_mm_s, ramp_acc=ramp_acc)
        time.sleep(refresh_s)
    for m, (_, _, acc) in zip(group.motors, cmds):
        m.move_velocity(0, acc, direction=0)
    group._wd_touch(False)      # freinage lancé en direct : désarme le watchdog pendant la rampe
    max_rpm = max(c[0] for c in cmds)
    time.sleep(max_rpm * (256 - ramp_acc) * 50e-6 + 0.4)     # durée de la rampe de freinage
    group.stop_all(255)                                     # remet l'état interne à zéro


def test_velocity_square(base, side_mm=300.0, speed_mm_s=80.0, ramp_acc=220):
    leg_s = side_mm / speed_mm_s
    x0, y0, th0, _ = base.pose()
    try:
        for name, angle in SQUARE_ANGLES:
            print(f"[{name}] {side_mm:.0f} mm à {speed_mm_s:.0f} mm/s ({leg_s:.1f} s)")
            drive_leg(base.group, angle, speed_mm_s, leg_s, ramp_acc)
    finally:
        base.group.stop_all(255)
    time.sleep(0.3)
    x1, y1, th1, _ = base.pose()
    print(f"Départ  : ({x0:.1f}, {y0:.1f}) mm, {math.degrees(th0):.2f}°")
    print(f"Arrivée : ({x1:.1f}, {y1:.1f}) mm, {math.degrees(th1):.2f}°  "
          f"(écart {math.hypot(x1 - x0, y1 - y0):.1f} mm, {math.degrees(th1 - th0):.2f}°)")


if __name__ == "__main__":
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
    base = HolonomicBase(robot, group)
    base.POSITION_SIGN = -1          # valeur validée par tests/test_odometry
    group.start_watchdog(0.3)
    base.start_odometry(100)
    time.sleep(0.3)
    try:
        test_velocity_square(base)
    finally:
        group.stop_all(255)
        base.stop_odometry()
        group.close()
        notifier.stop()
        bus.shutdown()
'''


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)


def is_git():
    r = run(["git", "rev-parse", "--is-inside-work-tree"])
    return r.returncode == 0 and r.stdout.strip() == "true"


def write(rel, text):
    p = ROOT / rel
    print(f"  écrire   {rel}")
    if APPLY:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")
        if is_git():
            run(["git", "add", rel])


def main():
    if not (ROOT / "core").is_dir():
        sys.exit("À lancer depuis simu/base_holo (dossier core/ introuvable).")
    git = is_git()
    print(("APPLIQUER" if APPLY else "SIMULATION") + f" — {'git mv' if git else 'déplacement simple'}\n")

    for src, dst in MOVES.items():
        s, d = ROOT / src, ROOT / dst
        if not s.exists():
            print(f"  (absent) {src}")
            continue
        print(f"  déplacer {src}  ->  {dst}")
        if APPLY:
            d.parent.mkdir(parents=True, exist_ok=True)
            if git:
                r = run(["git", "mv", src, dst])
                if r.returncode != 0:                       # fichier non suivi par git
                    shutil.move(str(s), str(d))
            else:
                shutil.move(str(s), str(d))

    write("legacy/README.md", LEGACY_README)
    write("tests/test_velocity_square.py", VELOCITY_SQUARE)
    write("README.md", README)

    docs = ROOT / "docs"
    if APPLY and docs.is_dir() and not any(docs.iterdir()):
        docs.rmdir()
        print("  supprimer dossier vide docs/")

    print("\nGardés : core/*, examples/teleop_zqsd.py, tests/test_odometry.py, "
          "tests/test_wheel_distances.py, tests/test_velocity_square.py (réécrit), tools/*")
    if not APPLY:
        print("\n(simulation) relancer avec --apply pour exécuter.")
    else:
        print("\nFait. Vérifier avec `git status`, puis commit.")


if __name__ == "__main__":
    main()

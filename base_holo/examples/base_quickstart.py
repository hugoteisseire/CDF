"""
Guide de démarrage de la base holonome : création de la structure + exemples d'appels.

    python -m examples.base_quickstart                # liste les exemples (ne bouge rien)
    python -m examples.base_quickstart pose           # un exemple précis
    python -m examples.base_quickstart velocity local direction rotate

Lancer depuis simu/base_holo, bus CAN actif (can0 à 1 Mbit/s). Robot posé au sol avec de la place
(ou roues en l'air pour un premier essai). Ctrl+C freine proprement.

REPÈRES ET CONVENTIONS (valables pour tous les appels ci-dessous)
-----------------------------------------------------------------
- Repère ROBOT : x+ vers la roue 3 (0°), y+ à 90° de x+ (vers la gauche du robot vu de dessus).
  Roues : 1 = 120°, 2 = 240°, 3 = 0°.
- θ positif = sens anti-horaire (vu de dessus). Vrai pour la direction ET pour la rotation.
- Repère GLOBAL (table) : défini par l'angle de départ `robot.angle` au moment de la création.
  Avec angle=0 les repères robot et global coïncident au démarrage. L'odométrie met ensuite
  `robot.pos` / `robot.angle` à jour : les directions globales suivent le cap estimé.
- Unités : mm, mm/s, radians, rad/s.
"""
import math
import sys
import time
from contextlib import contextmanager

import numpy as np

# ======================================================================
#   PARAMÈTRES DU ROBOT  (à mettre à jour si la mécanique change)
#   -> les valider avec tests/test_odometry.py
# ======================================================================
WHEEL_DIAMETER_MM = 55.8           # diamètre effectif de roue (calibré sur ligne droite)
GEAR_RATIO = 48 / 15               # tours moteur pour 1 tour de roue
ROBOT_RADIUS_MM = 106.5            # distance centre -> contact roue (calibrée en rotation)
WHEEL_ANGLES_DEG = (120, 240, 0)   # angle de chaque roue : moteurs 1, 2, 3
MOTOR_IDS = (1, 2, 3)              # identifiants CAN
START_ANGLE = 0.0                  # cap de départ dans le repère global (rad)
POSITION_SIGN = -1                 # sens de lecture des positions (validé par test_odometry)


# ======================================================================
#   1) CRÉATION DE LA STRUCTURE
# ======================================================================
@contextmanager
def open_base(bus_factory=None):
    """
    Construit toute la chaîne et la démarre ; la referme proprement (freinage, threads, bus) à la
    sortie du `with`, même en cas d'erreur ou de Ctrl+C.

        Bus CAN -> Notifier -> Motor x3 -> MotorGroup -> HolonomicBase
                                            (+ Robot : géométrie / pose)

        with open_base() as base:
            base.set_velocity(100, 0)
    """
    import can
    from mks_servo_can import MksServo
    from core.holo_base import Robot
    from core.holonomic_base import HolonomicBase
    from core.motor_controller import Motor, MotorGroup

    # Bus SocketCAN. Le débit se règle avec `ip link` (pas ici). Le Notifier est le SEUL
    # lecteur du bus : chaque Motor reçoit ses réponses via `notifier=` (jamais de bus.recv direct).
    bus = (bus_factory or (lambda: can.interface.Bus(interface="socketcan", channel="can0")))()
    notifier = can.Notifier(bus, [])

    # Un Motor par servo MKS : protocole CAN + conversions mm <-> pulses <-> RPM.
    motors = [Motor(bus, i, MksServo(bus, notifier, i),
                    wheel_diameter_mm=WHEEL_DIAMETER_MM, gear_ratio=GEAR_RATIO, notifier=notifier)
              for i in MOTOR_IDS]

    # Robot : géométrie (rayon, angles de roue) + pose courante (pos, angle).
    robot = Robot(pos=[0, 0], angle=START_ANGLE, radius=ROBOT_RADIUS_MM,
                  wheel_angles=[math.radians(a) for a in WHEEL_ANGLES_DEG])

    # MotorGroup : mouvements synchronisés, watchdog, lecture de position en continu.
    group = MotorGroup(motors, robot)

    # HolonomicBase : façade à utiliser partout au-dessus (stratégie, A*, teleop...).
    #   max_speed_mm_s / max_omega_rad_s : plafonds appliqués à TOUTE consigne
    #   watchdog_s : sans nouvelle consigne de vitesse pendant ce délai, les roues freinent seules
    #   odometry_hz : fréquence de lecture des positions
    base = HolonomicBase(robot, group, max_speed_mm_s=300.0, max_omega_rad_s=2.0,
                         watchdog_s=0.3, odometry_hz=100.0)
    base.POSITION_SIGN = POSITION_SIGN
    base.start()                      # lance watchdog + odométrie
    time.sleep(0.3)                   # laisse l'odométrie prendre sa première référence
    try:
        yield base
    finally:
        base.close()                  # freine, arrête les threads
        notifier.stop()
        bus.shutdown()


# ======================================================================
#   OUTILS
# ======================================================================
def hold_velocity(base, vx, vy, omega=0.0, duration_s=2.0, local=False,
                  ramp_acc=220, refresh_s=0.05, brake_s=0.4):
    """
    Maintient une vitesse pendant `duration_s`, puis freine en douceur.

    Une consigne de vitesse N'EST PAS un mouvement borné : le moteur garde sa dernière consigne.
    Il faut donc la renvoyer régulièrement (ici toutes les 50 ms ; le watchdog freine sinon).
    set_velocity ne renvoie une trame CAN que si le RPM d'une roue a changé : appeler souvent est
    gratuit.
    """
    send = base.set_velocity_local if local else base.set_velocity
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration_s:
        send(vx, vy, omega, ramp_acc=ramp_acc)
        time.sleep(refresh_s)
    # Freinage en douceur : on réduit la consigne par paliers (toutes les roues restent
    # proportionnelles), puis arrêt franc.
    steps = max(1, int(brake_s / refresh_s))
    for k in range(steps):
        f = 1.0 - (k + 1) / steps
        send(vx * f, vy * f, omega * f, ramp_acc=ramp_acc)
        time.sleep(refresh_s)
    base.stop()


def print_pose(base, label=""):
    x, y, th, _ = base.pose()
    dist, rot = base.odometry_stats()
    print(f"{label}pose = ({x:.1f}, {y:.1f}) mm, θ = {math.degrees(th):.1f}°  "
          f"| parcouru {dist:.0f} mm, tourné {rot:.0f}°")


# ======================================================================
#   2) EXEMPLES
# ======================================================================
def example_pose(base):
    """Lire / recaler la pose. L'odométrie part de robot.pos / robot.angle et dérive ensuite."""
    print_pose(base, "départ : ")
    # reset_pose recale la pose (ex. robot calé contre une bordure, ou position fournie par le lidar).
    base.reset_pose(1000.0, 500.0, math.pi / 2)
    print_pose(base, "après reset_pose : ")
    base.reset_pose(0.0, 0.0, START_ANGLE)
    base.reset_stats()                # remet à zéro les compteurs distance / rotation cumulées


def example_velocity(base):
    """set_velocity : vitesse dans le repère GLOBAL, en continu (mm/s, mm/s, rad/s)."""
    print("1) 100 mm/s vers x+ pendant 2 s")
    hold_velocity(base, 100, 0, 0, 2.0)
    print_pose(base)

    print("2) diagonale à 100 mm/s (direction 45°)")
    v = 100
    hold_velocity(base, v * math.cos(math.pi / 4), v * math.sin(math.pi / 4), 0, 2.0)
    print_pose(base)

    print("3) arc de cercle : 100 mm/s vers x+ en tournant à 0,5 rad/s")
    hold_velocity(base, 100, 0, 0.5, 3.0)
    print_pose(base)

    print("4) dépassement de la limite : 1000 mm/s demandés -> plafonné à max_speed_mm_s (300)")
    hold_velocity(base, 1000, 0, 0, 1.0)      # même direction, norme réduite
    print_pose(base)

    # Équivalent plus lisible pour "direction + vitesse" :
    #   base.move_velocity_direction(angle_rad, speed_mm_s)


def example_velocity_local(base):
    """set_velocity_local : même chose dans le repère du ROBOT (x+ = vers la roue 3)."""
    print("5) avancer de 100 mm/s dans l'axe x+ du robot, quel que soit son cap")
    hold_velocity(base, 100, 0, 0, 2.0, local=True)
    print("   puis translation latérale (y+ robot)")
    hold_velocity(base, 0, 100, 0, 2.0, local=True)
    print_pose(base)


def example_move_direction(base):
    """move_direction : déplacement BORNÉ (mode position) ; la fonction rend la main à la fin."""
    print("Carré de 300 mm en mode position (EST, NORD, OUEST, SUD)")
    for name, angle in (("EST", 0.0), ("NORD", math.pi / 2), ("OUEST", math.pi), ("SUD", -math.pi / 2)):
        # distance (mm), direction globale (rad), vitesse (mm/s). Bloque jusqu'à l'arrêt.
        ok = base.move_direction(300, angle, speed_mm_s=150, verbose=False)
        print(f"  {name}: {'ok' if ok else 'TIMEOUT'}")
    print_pose(base, "fin : ")

    # Translation + rotation simultanées : dtheta (rad) est ajouté au déplacement.
    print("Translation de 300 mm vers x+ en tournant de +90° en même temps")
    base.move_direction(300, 0.0, speed_mm_s=150, dtheta=math.pi / 2, verbose=False)
    print_pose(base)

    # wait=False : retourne tout de suite (le mouvement continue). À terminer avec
    # base.group.wait_all_stopped() ou base.stop().
    base.move_direction(200, 0.0, speed_mm_s=150, wait=False, verbose=False)
    base.group.wait_all_stopped(timeout=10)


def example_rotate(base):
    """rotate : rotation sur place (+ = anti-horaire)."""
    print("+90° à 1 rad/s")
    base.rotate(math.pi / 2, omega_rad_s=1.0, verbose=False)
    print("-90° (retour)")
    base.rotate(-math.pi / 2, omega_rad_s=1.0, verbose=False)
    print("tour complet à 2 rad/s (au-delà de max_omega_rad_s, il serait plafonné)")
    base.rotate(2 * math.pi, omega_rad_s=2.0, verbose=False)
    print_pose(base)
    # Pour calibrer ROBOT_RADIUS_MM : base.rotate(2*math.pi*10) puis comparer au cap réel.


def example_watchdog(base):
    """Le watchdog : sans nouvelle consigne de vitesse, le robot freine tout seul."""
    print("Une seule consigne, puis plus rien : arrêt automatique après watchdog_s (0,3 s)")
    base.set_velocity(100, 0, 0)
    time.sleep(2.0)                    # on ne renvoie rien -> le watchdog freine
    print_pose(base)


EXAMPLES = {
    "pose": example_pose,
    "velocity": example_velocity,
    "local": example_velocity_local,
    "direction": example_move_direction,
    "rotate": example_rotate,
    "watchdog": example_watchdog,
}


def main(argv):
    names = [a for a in argv if a in EXAMPLES]
    if not names:
        print(__doc__.split("REPÈRES")[0])
        print("Exemples disponibles :", ", ".join(EXAMPLES))
        return
    with open_base() as base:
        for n in names:
            print(f"\n=== {n} ===")
            EXAMPLES[n](base)
    # ↑ à la sortie du `with` : freinage + arrêt des threads, quoi qu'il arrive.


if __name__ == "__main__":
    main(sys.argv[1:])

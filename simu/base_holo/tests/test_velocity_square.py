"""Carré en mode vitesse avec MotorGroup.move_velocity_direction (boucle ouverte, temporisé).

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

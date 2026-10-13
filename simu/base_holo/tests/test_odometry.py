"""Valide l'odométrie sur le robot (roues en l'air pour l'étape 1, au sol pour l'étape 2).

    python -m tests.test_odometry
Étape 1 : une roue à la fois, move_distance(+100 mm) ; lit la position et dit quel POSITION_SIGN utiliser.
Étape 2 : move_direction 500 mm vers l'EST puis retour ; compare la pose odométrique à la consigne.
"""
import math
import time

import numpy as np


def check_sign(base, dist_mm=100.0):
    """Retourne le POSITION_SIGN à utiliser (+1 ou -1)."""
    votes = []
    for m in base.motors:
        p0 = m.read_position()
        m.move_distance(distance_mm=dist_mm, speed_rpm=60, acceleration_mks=240)
        m.wait_until_stopped()
        time.sleep(0.2)
        p1 = m.read_position()
        delta = p1 - p0
        sign = 1 if delta > 0 else -1
        print(f"Moteur {m.can_id} : move_distance(+{dist_mm:.0f} mm) -> delta lu = {delta} pulses "
              f"(attendu ≈ {m.distance_to_pulses(dist_mm)} en valeur absolue) => POSITION_SIGN = {sign}")
        votes.append(sign)
        m.move_distance(distance_mm=-dist_mm, speed_rpm=60, acceleration_mks=240)   # retour
        m.wait_until_stopped()
    if len(set(votes)) != 1:
        print("⚠️ Les 3 moteurs ne sont pas cohérents entre eux, vérifier le câblage/le sens !")
    return votes[0]


def check_pose(base, dist_mm=np.pi*2, speed_mm_s=500.0):
    x0, y0, th0, _ = base.pose()
    base.group.move_direction(500.0, 0, speed_mm_s,fastest_boost=2,ramp_acc=200)          # EST
    #base.group.rotate(math.radians(90), omega_rad_s=1.0,ramp_acc=200)  # SUD
    time.sleep(0.3)
    x1, y1, th1, _ = base.pose()
    print(f"Aller  : Δ = ({x1 - x0:.1f}, {y1 - y0:.1f}) mm, Δθ = {math.degrees(th1 - th0):.2f}° "
          f"(consigne : {dist_mm:.0f}, 0, 0)")
    time.sleep(1)
    base.group.move_direction(-500.0, 0, speed_mm_s,fastest_boost=2,ramp_acc=200)      # OUEST
    #base.group.rotate(-math.radians(90), omega_rad_s=1.0,ramp_acc=200)
    time.sleep(0.3)
    x2, y2, th2, _ = base.pose()
    print(f"Retour : position finale = ({x2 - x0:.1f}, {y2 - y0:.1f}) mm (consigne : 0, 0), "
          f"Δθ = {math.degrees(th2 - th0):.2f}°")


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
    try:
        sign = -1
        print(f"\n→ mettre HolonomicBase.POSITION_SIGN = {sign}\n")
        base.POSITION_SIGN = sign
        base.start_odometry(200)
        time.sleep(0.3)
        check_pose(base)
    finally:
        group.stop_all(255)
        base.stop_odometry()
        group.close()
        notifier.stop()
        bus.shutdown()

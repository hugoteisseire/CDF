"""Test de Motor.move_velocity sur les 3 moteurs : le robot fait un carré en boucle ouverte.

Lancer depuis ~/CDF/simu/base_holo :  python -m tests.test_velocity_square
"""
import math
import time

# Sens des 4 côtés (repère global) : EST, NORD, OUEST, SUD
SQUARE_ANGLES = [0.0, math.pi / 2, math.pi, -math.pi / 2]


def plan_velocity_leg(robot, motors, direction_angle, speed_mm_s,
                      reference_acc_mks=220):
    """Calcule (rpm, direction, acc) par roue pour une vitesse robot donnée.

    - vitesse de roue = projection cinématique (mm/s), via compute_wheel_distances_direction
      avec distance = speed_mm_s (donc "mm" = mm/s).
    - rampe de même durée pour toutes les roues : n_i = round(max_rpm*(256-ref)/rpm_i),
      acc_i = 256 - n_i  -> les vitesses restent proportionnelles pendant la rampe.
    """
    from core.holo_base import compute_wheel_distances_direction

    wheel_v = compute_wheel_distances_direction(robot, speed_mm_s, direction_angle)
    rpms = []
    for v, m in zip(wheel_v, motors):
        rpm = abs(v) * m.pulses_per_mm * 60 / m.TICKS_PER_REVOLUTION
        rpms.append(int(round(rpm)) if abs(v) > 1e-6 else 0)
    if any(r > 3000 for r in rpms):
        raise ValueError(f"RPM > 3000 : {rpms}, baisse speed_mm_s")

    max_rpm = max(rpms)
    ramp_steps = max_rpm * (256 - reference_acc_mks)
    plan = []
    for v, rpm in zip(wheel_v, rpms):
        if rpm == 0:
            plan.append((0, 0, reference_acc_mks))
            continue
        n = max(1, min(255, round(ramp_steps / rpm)))
        plan.append((rpm, 1 if v < 0 else 0, 256 - n))   # 0 = CCW (comme move_distance > 0)
    return plan


def stop_motors(motors, acc=220):
    """acc : un entier, ou une liste d'acc par roue (décélération proportionnelle)."""
    accs = acc if isinstance(acc, (list, tuple)) else [acc] * len(motors)
    for m, a in zip(motors, accs):
        m.move_velocity(0, a, direction=0)


def test_velocity_square(robot, motors, side_mm=300, speed_mm_s=80,
                         reference_acc_mks=220, pause_s=0.5, log=True):
    """Fait un carré de côté `side_mm` avec move_velocity (boucle ouverte, temporisé).

    Chaque côté dure side_mm / speed_mm_s (+ le temps de rampe est inclus dans la
    durée : la distance réelle est un peu plus courte -> compense via ramp_comp).
    """
    leg_s = side_mm / speed_mm_s
    try:
        for name, ang in zip(["EST", "NORD", "OUEST", "SUD"], SQUARE_ANGLES):
            plan = plan_velocity_leg(robot, motors, ang, speed_mm_s, reference_acc_mks)
            if log:
                print(f"[{name}] rpm/dir/acc = {plan}  durée {leg_s:.2f}s")
            t0 = time.perf_counter()
            for m, (rpm, d, acc) in zip(motors, plan):
                m.move_velocity(rpm, acc, direction=d)
            # Le côté se termine à l'instant où on lance la décélération ; la
            # rampe d'accélération et celle de freinage se compensent (même acc).
            while time.perf_counter() - t0 < leg_s:
                time.sleep(0.005)
            stop_motors(motors, [p[2] for p in plan])   # même acc par roue qu'à l'accélération
            # attendre l'arrêt effectif (durée de rampe max + marge)
            max_rpm = max(p[0] for p in plan)
            time.sleep(max_rpm * (256 - reference_acc_mks) * 50e-6 + pause_s)
    finally:
        stop_motors(motors, 255)   # arrêt de sécurité quoi qu'il arrive


if __name__ == "__main__":
    import can
    import numpy as np
    from core.holo_base import Robot
    from core.motor_controller import Motor, MotorGroup
    from mks_servo_can import MksServo
    gear_ratio = 48/15
    bus = can.interface.Bus(interface="socketcan", channel="can0")
    notifier = can.Notifier(bus, [])
    motors = [Motor(bus, i, MksServo(bus, notifier, i),gear_ratio=gear_ratio) for i in (1, 2, 3)]
    robot = Robot(pos=[0, 0], angle=np.pi / 2, radius=150,
                  wheel_angles=[np.radians(120), np.radians(240), np.radians(0)])
    try:
        test_velocity_square(robot, motors, side_mm=800, speed_mm_s=800)
    finally:
        notifier.stop()
        bus.shutdown()

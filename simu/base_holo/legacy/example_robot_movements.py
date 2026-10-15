"""
Exemple pratique : Déplacer le robot holonome avec compute_wheel_distances
Intègre les fonctions de calcul de distances avec la classe Motor
"""

import time
import numpy as np
import math
import can
from core.motor_controller import Motor, MotorGroup
from mks_servo_can import MksServo
from core.holo_base import Robot, compute_wheel_distances_global, compute_wheel_distances_direction

# =============================
#   CONFIGURATION CAN ET MOTEURS
# =============================

# Initialisation du bus CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
#bus.socket.setblocking(False)

# Création des servos MKS
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)  
servo3 = MksServo(bus, notifier, 3)

gear_ratio = 48/15  # Rapport de réduction mécanique (48 dents sur 15)
# Création des moteurs avec paramètres mécaniques
motor1 = Motor(
    bus=bus, can_id=1, mks_servo=servo1,
    wheel_diameter_mm=55.8,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=gear_ratio,                 # Pas de réducteur
    notifier=notifier
)

motor2 = Motor(
    bus=bus, can_id=2, mks_servo=servo2,
    wheel_diameter_mm=55.8,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=gear_ratio,                 # Pas de réducteur
    notifier=notifier
)

motor3 = Motor(
    bus=bus, can_id=3, mks_servo=servo3,
    wheel_diameter_mm=55.8,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=gear_ratio,                 # Pas de réducteur
    notifier=notifier
)

motors_list = [motor1, motor2, motor3]
motors = MotorGroup(motors_list)

# =============================
#   CONFIGURATION ROBOT
# =============================

robot = Robot(
    pos=np.array([0.0, 0.0]),
    angle=np.pi/4,  # Orienté vers le haut
    radius=150,     # Rayon en mm
    wheel_angles=[np.radians(120), np.radians(240), np.radians(0)]
)




def move_robot_direction(robot, motors_list, distance, direction_angle, speed_mm_s=50,
                         dtheta=0.0, reference_acc_mks=220, fastest_acc_boost=0):
    """Rampes de même durée (roue la plus rapide = référence), acc brut MKS (manuel p.69)."""
    if speed_mm_s <= 0:
        raise ValueError("speed_mm_s doit être strictement positif")
    if not 1 <= reference_acc_mks <= 255:
        raise ValueError(f"reference_acc_mks doit être dans [1, 255], reçu {reference_acc_mks}")

    wheel_distances = compute_wheel_distances_direction(
        robot, distance=distance, direction_angle=direction_angle, dtheta=dtheta)
    print(f"\n🎯 Déplacement : {distance:.0f} mm à {math.degrees(direction_angle):.0f}°")
    if dtheta != 0:
        print(f"   + Rotation : {math.degrees(dtheta):.0f}°")

    # Roues qui bougent réellement
    active = []
    for i, (motor, d) in enumerate(zip(motors_list, wheel_distances)):
        p = motor.distance_to_pulses(d)
        print(f"   Roue {i + 1} : {d:7.2f} mm ({p} pulses)")
        if p != 0:
            active.append((i, motor, d, p))
    if not active:
        print("   ℹ️ Aucun déplacement effectif à commander.")
        return
    max_p = max(abs(p) for _, _, _, p in active)

    # RPM cibles : même échelle que distance_to_pulses, arrondi au plus proche, minimum 1
    rpms = []
    for i, motor, d, p in active:
        v_mm_s = speed_mm_s * abs(p) / max_p
        rpm = round(v_mm_s * motor.pulses_per_mm * 60 / motor.TICKS_PER_REVOLUTION)
        rpms.append(max(1, min(3000, rpm)))
    max_rpm = max(rpms)

    # Durée de rampe commune, fixée par la roue la plus rapide : unités RPM x 50 µs
    ramp_steps = max_rpm * (256 - reference_acc_mks)

    # Un paramètre acc par roue, plafonné individuellement : une roue très lente prend acc=1
    # (rampe la plus douce possible) sans raccourcir la rampe des roues rapides.
    commands = []
    for (i, motor, d, p), rpm in zip(active, rpms):
        n_wanted = round(ramp_steps / rpm)
        n = max(1, min(255, n_wanted))
        acc = 256 - n
        if rpm == max_rpm:                        # toutes les roues à vitesse max, pas une seule
            acc = min(255, acc + fastest_acc_boost)
        n = 256 - acc
        note = " (plafonnée : rampe plus courte que la référence)" if n_wanted > 255 else ""
        print(f"   Roue {i + 1} : {rpm:4d} RPM | acc={acc:3d} | rampe≈{rpm * n * 0.05:.0f} ms{note}")
        commands.append((motor, d, rpm, acc, p, n))

    for motor, d, rpm, acc, _, _ in commands:
        motor.move_distance(distance_mm=d, speed_rpm=rpm, acceleration_mks=acc)

    # Attente : laisser démarrer, puis timeout = durée estimée + marge
    time.sleep(0.1)
    est = max(abs(p) / (rpm * motor.TICKS_PER_REVOLUTION / 60) + rpm * n * 50e-6
              for motor, _, rpm, _, p, n in commands)
    print(f"   ⏳ En cours (≈ {est:.1f} s)...")
    deadline = time.time() + est + 5.0
    while time.time() < deadline:
        if all(not motor.is_running() for motor, *_ in commands):
            print("   ✅ Mouvement terminé")
            return
        time.sleep(0.01)
    print("   ⚠️ Timeout")

#========================
#   PROGRAMME PRINCIPAL
# =============================

if __name__ == "__main__":
    try:
        print("\n" + "=" * 70)
        print("CONTRÔLE ROBOT HOLONOME - DÉPLACEMENTS PAR DISTANCE")
        print("=" * 70)
        print(f"\nConfiguration robot :")
        print(f"  - Rayon : {robot.radius} mm")
        print(f"  - Orientation : {math.degrees(robot.angle):.0f}°")
        print(f"  - Angles roues : {[math.degrees(a) for a in robot.wheel_angles]}°")
        print(f"\nConfiguration moteurs :")
        print(f"  - Diamètre roue : {motor1.wheel_diameter_mm} mm")
        print(f"  - Résolution : {motor1.pulses_per_mm:.2f} pulses/mm")
        
        # Décommenter l'exemple que vous voulez exécuter
        
        # example_cardinal_movements()
        # example_square_trajectory()
        # example_diagonal_movement()
        # example_translation_rotation()
        # example_xy_movement()
        # example_return_to_origin()
        
        # Ou un mouvement simple pour tester
        print("\n" + "=" * 70)
        print("TEST SIMPLE : Avancer de 100mm vers le NORD")
        print("=" * 70)
        move_robot_direction(robot, motors_list, distance=3000, direction_angle=0, speed_mm_s=600,reference_acc_mks=240, fastest_acc_boost=2)
        move_robot_direction(robot, motors_list, distance=3000, direction_angle=np.pi, speed_mm_s=600,reference_acc_mks=240, fastest_acc_boost=2)
        
    except KeyboardInterrupt:
        print("\n🛑 Interruption par l'utilisateur")
    
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Arrêt sécurisé de tous les moteurs
        print("\n🛑 Arrêt de tous les moteurs...")
        motors.stop_all(acceleration=150)
        time.sleep(0.5)
        print("✅ Arrêt complet")

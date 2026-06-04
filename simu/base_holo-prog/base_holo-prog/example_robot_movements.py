"""
Exemple pratique : Déplacer le robot holonome avec compute_wheel_distances
Intègre les fonctions de calcul de distances avec la classe Motor
"""

import time
import numpy as np
import math
import can
from motor_controller import Motor, MotorGroup
from mks_servo_can import MksServo
from holo_base import Robot, compute_wheel_distances_global, compute_wheel_distances_direction

# =============================
#   CONFIGURATION CAN ET MOTEURS
# =============================

# Initialisation du bus CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
bus.socket.setblocking(False)

# Création des servos MKS
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)

# Création des moteurs avec paramètres mécaniques
motor1 = Motor(
    bus=bus, can_id=1, mks_servo=servo1,
    wheel_diameter_mm=60.0,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=1.0
)

motor2 = Motor(
    bus=bus, can_id=2, mks_servo=servo2,
    wheel_diameter_mm=60.0,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=1.0
)

motor3 = Motor(
    bus=bus, can_id=3, mks_servo=servo3,
    wheel_diameter_mm=60.0,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=1.0
)

motors_list = [motor1, motor2, motor3]
motors = MotorGroup(motors_list)

# =============================
#   CONFIGURATION ROBOT
# =============================

robot = Robot(
    pos=np.array([0.0, 0.0]),
    angle=np.pi/2,  # Orienté vers le haut
    radius=150,     # Rayon en mm
    wheel_angles=[np.radians(120), np.radians(240), np.radians(0)]
)

# =============================
#   FONCTION UTILITAIRE
# =============================

def move_robot_direction(robot, motors_list, distance, direction_angle, speed_mm_s=50, dtheta=0.0):
    """
    Déplace le robot d'une distance dans une direction donnée.
    
    Args:
        robot: Instance Robot
        motors_list: Liste des 3 moteurs [motor1, motor2, motor3]
        distance: Distance à parcourir (mm)
        direction_angle: Angle de direction dans le repère global (radians)
        speed_mm_s: Vitesse linéaire (mm/s)
        dtheta: Rotation simultanée (radians)
    """
    # Calculer les distances pour chaque roue
    wheel_distances = compute_wheel_distances_direction(
        robot, 
        distance=distance, 
        direction_angle=direction_angle,
        dtheta=dtheta
    )
    
    print(f"\n🎯 Déplacement : {distance:.0f}mm à {math.degrees(direction_angle):.0f}°")
    if dtheta != 0:
        print(f"   + Rotation : {math.degrees(dtheta):.0f}°")
    
    # Afficher les distances calculées
    for i, dist in enumerate(wheel_distances):
        print(f"   Roue {i+1} : {dist:7.2f} mm")
    
    # Appliquer le mouvement à chaque moteur
    for motor, wheel_dist in zip(motors_list, wheel_distances):
        motor.move_distance(
            distance_mm=wheel_dist,
            speed_mm_s=speed_mm_s,
            acceleration=150
        )
    
    # Attendre que tous les moteurs s'arrêtent
    print("   ⏳ En cours...")
    all_stopped = False
    timeout = time.time() + 30.0
    
    while not all_stopped and time.time() < timeout:
        all_stopped = all(not m.is_running() for m in motors_list)
        time.sleep(0.01)
    
    if all_stopped:
        print("   ✅ Mouvement terminé")
    else:
        print("   ⚠️ Timeout")

def move_robot_global(robot, motors_list, dx, dy, speed_mm_s=50, dtheta=0.0):
    """
    Déplace le robot avec des composantes dx, dy dans le repère global.
    
    Args:
        robot: Instance Robot
        motors_list: Liste des 3 moteurs
        dx: Déplacement en X global (mm)
        dy: Déplacement en Y global (mm)
        speed_mm_s: Vitesse linéaire (mm/s)
        dtheta: Rotation simultanée (radians)
    """
    # Calculer les distances pour chaque roue
    wheel_distances = compute_wheel_distances_global(robot, dx, dy, dtheta)
    
    print(f"\n🎯 Déplacement : dx={dx:.0f}mm, dy={dy:.0f}mm")
    if dtheta != 0:
        print(f"   + Rotation : {math.degrees(dtheta):.0f}°")
    
    # Afficher les distances calculées
    for i, dist in enumerate(wheel_distances):
        print(f"   Roue {i+1} : {dist:7.2f} mm")
    
    # Appliquer le mouvement à chaque moteur
    for motor, wheel_dist in zip(motors_list, wheel_distances):
        motor.move_distance(
            distance_mm=wheel_dist,
            speed_mm_s=speed_mm_s,
            acceleration=150
        )
    
    # Attendre que tous les moteurs s'arrêtent
    print("   ⏳ En cours...")
    all_stopped = False
    timeout = time.time() + 30.0
    
    while not all_stopped and time.time() < timeout:
        all_stopped = all(not m.is_running() for m in motors_list)
        time.sleep(0.01)
    
    if all_stopped:
        print("   ✅ Mouvement terminé")
    else:
        print("   ⚠️ Timeout")

# =============================
#   EXEMPLE 1 : MOUVEMENTS CARDINAUX
# =============================

def example_cardinal_movements():
    """Test des mouvements dans les 4 directions cardinales."""
    print("\n" + "=" * 70)
    print("EXEMPLE 1 : Mouvements cardinaux (100mm)")
    print("=" * 70)
    
    distance = 100  # mm
    vitesse = 50    # mm/s
    
    directions = [
        (0, "EST"),
        (np.pi/2, "NORD"),
        (np.pi, "OUEST"),
        (-np.pi/2, "SUD")
    ]
    
    for angle, nom in directions:
        print(f"\n📍 Direction : {nom}")
        move_robot_direction(robot, motors_list, distance, angle, vitesse)
        time.sleep(1)
    
    print("\n✅ Mouvements cardinaux terminés")

# =============================
#   EXEMPLE 2 : TRAJECTOIRE CARRÉE
# =============================

def example_square_trajectory():
    """Trajectoire carrée de 200mm de côté."""
    print("\n" + "=" * 70)
    print("EXEMPLE 2 : Trajectoire carrée (200mm)")
    print("=" * 70)
    
    cote = 200  # mm
    vitesse = 80  # mm/s
    
    directions = [
        (0, "EST"),
        (np.pi/2, "NORD"),
        (np.pi, "OUEST"),
        (-np.pi/2, "SUD")
    ]
    
    for i, (angle, nom) in enumerate(directions):
        print(f"\n📍 Côté {i+1}/4 : {nom}")
        move_robot_direction(robot, motors_list, cote, angle, vitesse)
        time.sleep(0.5)
    
    print("\n✅ Trajectoire carrée terminée")

# =============================
#   EXEMPLE 3 : DÉPLACEMENT DIAGONAL
# =============================

def example_diagonal_movement():
    """Déplacement diagonal (Nord-Est)."""
    print("\n" + "=" * 70)
    print("EXEMPLE 3 : Déplacement diagonal (Nord-Est)")
    print("=" * 70)
    
    # 100mm à 45° (Nord-Est)
    distance = 100
    angle = np.pi/4
    vitesse = 50
    
    print(f"\nDéplacement de {distance}mm à {math.degrees(angle):.0f}°")
    move_robot_direction(robot, motors_list, distance, angle, vitesse)
    
    print("\n✅ Déplacement diagonal terminé")

# =============================
#   EXEMPLE 4 : TRANSLATION + ROTATION
# =============================

def example_translation_rotation():
    """Déplacement avec rotation simultanée."""
    print("\n" + "=" * 70)
    print("EXEMPLE 4 : Translation + Rotation")
    print("=" * 70)
    
    # Avancer de 150mm vers le nord tout en tournant de 90°
    distance = 150
    direction = np.pi/2  # Nord
    rotation = np.pi/2   # 90°
    vitesse = 40
    
    print(f"\nAvancer {distance}mm vers le NORD")
    print(f"tout en tournant de {math.degrees(rotation):.0f}°")
    
    move_robot_direction(robot, motors_list, distance, direction, vitesse, dtheta=rotation)
    
    print("\n✅ Mouvement combiné terminé")

# =============================
#   EXEMPLE 5 : DÉPLACEMENT EN X/Y
# =============================

def example_xy_movement():
    """Déplacement avec composantes dx, dy."""
    print("\n" + "=" * 70)
    print("EXEMPLE 5 : Déplacement dx/dy")
    print("=" * 70)
    
    # Déplacement de 100mm en X, 50mm en Y
    dx = 100
    dy = 50
    vitesse = 50
    
    print(f"\nDéplacement : dx={dx}mm, dy={dy}mm")
    move_robot_global(robot, motors_list, dx, dy, vitesse)
    
    print("\n✅ Déplacement dx/dy terminé")

# =============================
#   EXEMPLE 6 : RETOUR À L'ORIGINE
# =============================

def example_return_to_origin():
    """Retour à la position d'origine."""
    print("\n" + "=" * 70)
    print("EXEMPLE 6 : Retour à l'origine")
    print("=" * 70)
    
    print("\nLecture des positions actuelles...")
    positions = motors.read_all_positions()
    
    if all(p is not None for p in positions):
        print("Positions actuelles (pulses) :")
        for i, pos in enumerate(positions):
            print(f"  Roue {i+1} : {pos} pulses")
        
        # Convertir en distances
        distances_mm = [motor.pulses_to_distance(pos) for motor, pos in zip(motors_list, positions)]
        print("\nDistances parcourues (mm) :")
        for i, dist in enumerate(distances_mm):
            print(f"  Roue {i+1} : {dist:.2f} mm")
        
        # Revenir en arrière (distances négatives)
        print("\nRetour à l'origine...")
        for motor, dist in zip(motors_list, distances_mm):
            motor.move_distance(
                distance_mm=-dist,
                speed_mm_s=80,
                acceleration=200
            )
        
        # Attendre
        print("   ⏳ En cours...")
        time.sleep(0.1)
        all_stopped = False
        timeout = time.time() + 30.0
        
        while not all_stopped and time.time() < timeout:
            all_stopped = all(not m.is_running() for m in motors_list)
            time.sleep(0.01)
        
        if all_stopped:
            print("   ✅ Retour à l'origine terminé")
        else:
            print("   ⚠️ Timeout")
    else:
        print("⚠️ Impossible de lire les positions")

# =============================
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
        move_robot_direction(robot, motors_list, distance=100, direction_angle=np.pi/2, speed_mm_s=50)
        
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

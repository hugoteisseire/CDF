"""
Test de la fonction autonome compute_navigation_direction()
pour le robot réel (sans classe Simulator).

Ce fichier démontre comment utiliser la fonction de navigation
dans un contexte robot réel simplifié.
"""

import numpy as np
import time
import math
from holo_base import Robot, compute_wheel_speeds_global
from avoidance import compute_navigation_direction, reset_pathfinding_state


# ============================================================================
# CONFIGURATION
# ============================================================================

TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000
ROBOT_RADIUS_MM = 150
LOOP_FREQ_HZ = 60
MAX_WHEEL_SPEED_MPS = 0.3


# ============================================================================
# SIMULATION SIMPLIFIÉE DU ROBOT RÉEL
# ============================================================================

def main():
    """
    Boucle principale simulant le robot réel.
    Démontre l'utilisation de compute_navigation_direction().
    """
    
    print("=" * 70)
    print("TEST FONCTION AUTONOME DE NAVIGATION")
    print("=" * 70)
    
    # ===== INITIALISATION =====
    
    # Position initiale robot
    robot = Robot(
        pos=np.array([500.0, 500.0]),
        angle=0.0,
        radius=ROBOT_RADIUS_MM,
        wheel_angles=[math.radians(90), math.radians(210), math.radians(330)]
    )
    robot.wheel_speeds = [0.0, 0.0, 0.0]
    
    # Position cible
    target_pos = np.array([2500.0, 1500.0])
    
    # Position adversaire (fictive pour test)
    enemy_pos = np.array([1500.0, 1000.0])
    
    # Réinitialiser état pathfinding
    reset_pathfinding_state()
    
    print(f"\n📍 Position initiale robot: {robot.pos}")
    print(f"🎯 Position cible: {target_pos}")
    print(f"🔴 Position adversaire: {enemy_pos}")
    print(f"\n⏱️  Fréquence boucle: {LOOP_FREQ_HZ} Hz\n")
    
    # ===== BOUCLE PRINCIPALE =====
    
    frame_count = 0
    max_frames = 600  # 10 secondes à 60 Hz
    dt = 1.0 / LOOP_FREQ_HZ
    
    print("🚀 Démarrage navigation...\n")
    
    while frame_count < max_frames:
        start_time = time.time()
        
        # ===== 1. LOCALISATION (SIMULÉE) =====
        # Sur robot réel: robot.pos, robot.angle = localization.update(...)
        # Ici on simule juste un déplacement simple
        
        # ===== 2. PERCEPTION (SIMULÉE) =====
        # Sur robot réel: enemy_pos = lidar_handler.detect_enemy(...)
        # Ici position fixe pour test
        
        # ===== 3. NAVIGATION (FONCTION TESTÉE) =====
        result = compute_navigation_direction(
            robot=robot,
            target_pos=target_pos,
            enemy_pos=enemy_pos,
            table_width_mm=TABLE_WIDTH_MM,
            table_height_mm=TABLE_HEIGHT_MM,
            path_recompute_interval=15
        )
        
        direction = result['direction']
        code = result['code']
        reached = result['reached']
        error_message = result['error_message']
        should_pause = result['should_pause']
        
        # ===== 4. AFFICHAGE DEBUG =====
        if frame_count % 30 == 0:  # Toutes les 0.5s
            print(f"Frame {frame_count:4d} | Pos: ({robot.pos[0]:6.1f}, {robot.pos[1]:6.1f}) | "
                  f"Direction: ({direction[0]:5.2f}, {direction[1]:5.2f}) | "
                  f"Code: {code} | Reached: {reached}")
        
        # ===== 5. GESTION ERREURS =====
        if should_pause:
            print(f"\n⚠️  ERREUR: {error_message}")
            print("🛑 Arrêt robot\n")
            break
        
        if reached:
            print(f"\n✅ OBJECTIF ATTEINT !")
            print(f"📍 Position finale: ({robot.pos[0]:.1f}, {robot.pos[1]:.1f})")
            distance_finale = np.linalg.norm(robot.pos - target_pos)
            print(f"📏 Distance à la cible: {distance_finale:.1f} mm\n")
            break
        
        # ===== 6. CINÉMATIQUE =====
        # Calcul vitesses roues
        raw_speeds = compute_wheel_speeds_global(
            robot,
            direction[0],  # vx
            direction[1],  # vy
            0.0            # omega
        )
        
        # Conversion en mm/tick
        max_speed_mm_tick = (MAX_WHEEL_SPEED_MPS * 1000) / LOOP_FREQ_HZ
        robot.wheel_speeds = [s * max_speed_mm_tick for s in raw_speeds]
        
        # ===== 7. COMMANDES MOTEURS (SIMULÉES) =====
        # Sur robot réel:
        # motor_controller.set_wheel_speeds(robot.wheel_speeds)
        
        # Ici on simule le mouvement (pour test uniquement)
        # Déplacement simple dans la direction calculée
        speed_mm_per_frame = 3.0  # 3mm par frame (~180mm/s à 60Hz)
        robot.pos += direction * speed_mm_per_frame
        
        # Clamping dans la table
        robot.pos[0] = max(ROBOT_RADIUS_MM, min(TABLE_WIDTH_MM - ROBOT_RADIUS_MM, robot.pos[0]))
        robot.pos[1] = max(ROBOT_RADIUS_MM, min(TABLE_HEIGHT_MM - ROBOT_RADIUS_MM, robot.pos[1]))
        
        # ===== 8. TIMING BOUCLE =====
        elapsed = time.time() - start_time
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)
        
        frame_count += 1
    
    # ===== STATISTIQUES FINALES =====
    print("\n" + "=" * 70)
    print("STATISTIQUES")
    print("=" * 70)
    print(f"Frames exécutées: {frame_count}")
    print(f"Temps simulé: {frame_count / LOOP_FREQ_HZ:.2f} s")
    print(f"Position initiale: ({500.0:.1f}, {500.0:.1f})")
    print(f"Position finale: ({robot.pos[0]:.1f}, {robot.pos[1]:.1f})")
    
    distance_parcourue = np.linalg.norm(robot.pos - np.array([500.0, 500.0]))
    print(f"Distance parcourue: {distance_parcourue:.1f} mm")
    print("=" * 70 + "\n")


# ============================================================================
# EXEMPLE 2: USAGE MINIMAL
# ============================================================================

def exemple_minimal():
    """
    Exemple minimal d'utilisation de la fonction.
    Montre le strict minimum requis pour l'utiliser.
    """
    print("\n" + "=" * 70)
    print("EXEMPLE MINIMAL")
    print("=" * 70 + "\n")
    
    # 1. Créer robot
    robot = Robot(
        pos=np.array([1000.0, 1000.0]),
        angle=0.0,
        radius=150.0,
        wheel_angles=[math.radians(90), math.radians(210), math.radians(330)]
    )
    
    # 2. Définir cible et ennemi
    target = np.array([2500.0, 1500.0])
    enemy = np.array([1500.0, 1000.0])
    
    # 3. Appeler fonction
    result = compute_navigation_direction(robot, target, enemy)
    
    # 4. Utiliser résultat
    print(f"Direction calculée: {result['direction']}")
    print(f"Code d'état: {result['code']}")
    print(f"Erreur: {result['error_message']}")
    print(f"Doit s'arrêter: {result['should_pause']}")
    print(f"Objectif atteint: {result['reached']}")
    print(f"Nombre de cellules dans le chemin: {len(result['path'])}")
    
    print("\n" + "=" * 70 + "\n")


# ============================================================================
# EXEMPLE 3: INTÉGRATION ROBOT RÉEL
# ============================================================================

def exemple_integration_robot_reel():
    """
    Exemple de squelette pour intégration sur robot réel.
    Montre où placer les appels aux drivers.
    """
    print("\n" + "=" * 70)
    print("EXEMPLE INTÉGRATION ROBOT RÉEL (SQUELETTE)")
    print("=" * 70 + "\n")
    
    code_exemple = '''
# ===== PSEUDO-CODE POUR ROBOT RÉEL =====

from holo_base import Robot, compute_wheel_speeds_global
from avoidance import compute_navigation_direction, reset_pathfinding_state

# Imports vos drivers
# from motor_controller import MotorController
# from imu_handler import IMUHandler
# from lidar_handler import LidarHandler
# from localization import Localization

def boucle_robot_reel():
    # Initialisation hardware
    # motor_ctrl = MotorController(can_bus=init_can())
    # imu = IMUHandler()
    # lidar = LidarHandler(port='/dev/ttyUSB0')
    # localization = Localization(imu)
    
    # Robot virtuel (pour pathfinding)
    robot = Robot(pos=np.array([500.0, 500.0]), angle=0.0, radius=150.0, ...)
    target_pos = np.array([2500.0, 1500.0])
    
    reset_pathfinding_state()  # Important au démarrage !
    
    while True:
        # 1. LOCALISATION
        # robot.pos, robot.angle = localization.update()
        
        # 2. PERCEPTION
        # enemy_pos = lidar.detect_enemy(robot.pos, robot.angle)
        # obstacles_dynamic = lidar.get_obstacles_mm(robot.pos, robot.angle)
        
        # 3. NAVIGATION (votre fonction !)
        result = compute_navigation_direction(robot, target_pos, enemy_pos)
        
        if result['should_pause']:
            # motor_ctrl.emergency_stop()
            print(f"ARRÊT: {result['error_message']}")
            break
        
        if result['reached']:
            # target_pos = strategy.get_next_objective()
            print("Objectif atteint !")
        
        # 4. CINÉMATIQUE
        direction = result['direction']
        wheel_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)
        
        # 5. COMMANDES MOTEURS
        # motor_ctrl.set_wheel_speeds_normalized(wheel_speeds)
        
        # 6. TIMING (60 Hz)
        # time.sleep(1.0 / 60.0)
'''
    
    print(code_exemple)
    print("=" * 70 + "\n")


# ============================================================================
# POINT D'ENTRÉE
# ============================================================================

if __name__ == "__main__":
    # Exécuter les différents exemples
    
    # 1. Test complet avec simulation
    main()
    
    # 2. Exemple minimal
    exemple_minimal()
    
    # 3. Squelette intégration robot réel
    exemple_integration_robot_reel()
    
    print("✅ Tests terminés avec succès !\n")

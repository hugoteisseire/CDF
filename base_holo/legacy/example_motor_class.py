"""
Exemple d'utilisation de la classe Motor pour contrôler les moteurs holonomes.
Inclut des tests du mode asynchrone avec thread daemon.
"""

import time
import can
import numpy as np
from core.motor_controller import Motor, MotorGroup
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction, RunMotorResult
from mks_servo_can import MksServo
from core.holo_base import Robot, compute_wheel_speeds_global,compute_wheel_distances_global

# =============================
#   CONFIGURATION CAN
# =============================

# Initialisation du bus CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
bus.socket.setblocking(False)

# =============================
#   CRÉATION DES MOTEURS
# =============================

# Création des servos MKS (pour fonctionnalités avancées)
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)

# Création des instances Motor (UN SEUL objet par moteur)
# Le mode async peut être activé/désactivé à la volée
motor1 = Motor(
    bus=bus, can_id=1, mks_servo=servo1,
    wheel_diameter_mm=54.3,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=48.0/15.0
)

motor2 = Motor(
    bus=bus, can_id=2, mks_servo=servo2,
    wheel_diameter_mm=54.3,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=48.0/15.0
)

motor3 = Motor(
    bus=bus, can_id=3, mks_servo=servo3,
    wheel_diameter_mm=54.3,
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=48.0/15.0
)
# Groupe de moteurs pour contrôle synchronisé
motors = MotorGroup([motor1, motor2, motor3])

# =============================
#   CONFIGURATION ROBOT
# =============================

robot = Robot(
    pos=np.array([1000.0, 1000.0]),
    angle=np.pi/2,
    radius=150,
    wheel_angles=[np.radians(120), np.radians(240), np.radians(0)]
)

robot.wheel_speeds = [0.0, 0.0, 0.0]
robot.base_velocity = np.array([0.0, 0.0])
robot.base_velocity_global = np.array([0.0, 0.0])
# =============================
#   EXEMPLE 1 : MOUVEMENT SIMPLE
# =============================

def example_simple_movement():
    """Déplacement simple d'un seul moteur."""
    print("=== Exemple 1 : Mouvement simple ===")
    
    # Déplacement de 10000 pulses à 100 RPM avec accélération de 800 pulses/s²
    motor1.move_relative(
        pulses=10000,
        speed=100,
        acceleration=800  # En pulses/s² (sera converti automatiquement)
    )
    
    # Attendre la fin du mouvement
    motor1.wait_until_stopped()
    print("✅ Mouvement terminé")
    
    # Lire la position
    pos = motor1.read_position()
    print(f"Position actuelle : {pos} pulses")


# =============================
#   EXEMPLE 2 : MOUVEMENT SYNCHRONISÉ
# =============================

def example_synchronized_movement():
    """Trajectoire carrée avec mouvement synchronisé."""
    print("\n=== Exemple 2 : Trajectoire carrée synchronisée ===")
    
    # Directions pour un carré
    directions = [
        np.array([0, 1.0]),    # droite
        np.array([-1.0, 0]),   # haut
        np.array([0, -1.0]),   # gauche
        np.array([1.0, 0])     # bas
    ]
    
    # Paramètres
    max_speed =700  # RPM
    pulses_per_step = 30000
    base_accel = 100  # pulses/s²
    while True:
        for i, direction in enumerate(directions):
            print(f"\n📍 Étape {i+1}/4 - Direction: {direction}")
            
            # Calcul des vitesses de roues
            raw_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)
            
            # Déplacement synchronisé
            motors.move_synchronized(
                wheel_speeds=raw_speeds,
                max_speed=max_speed,
                pulses_per_step=pulses_per_step,
                base_acceleration=base_accel
            )
            
            # Attendre la fin
            motors.wait_all_stopped()
            print("✅ Tous les moteurs sont à l'arrêt")
    
    print("\n🎯 Trajectoire carrée terminée !")


# =============================
#   EXEMPLE 3 : CONTRÔLE EN VITESSE
# =============================

def example_velocity_control():
    """Contrôle en vitesse continue."""
    print("\n=== Exemple 3 : Contrôle en vitesse ===")
    
    # Démarrer à 50 RPM
    motor1.move_velocity(speed=50, acceleration=100)
    time.sleep(2)
    
    # Augmenter à 100 RPM
    motor1.move_velocity(speed=100, acceleration=100)
    time.sleep(2)
    
    # Arrêt progressif
    motor1.stop(acceleration=150)
    motor1.wait_until_stopped()
    
    print("✅ Cycle de vitesse terminé")


# =============================
#   EXEMPLE 4 : LECTURE D'ÉTAT
# =============================

def example_read_state():
    """Lecture de l'état des moteurs."""
    print("\n=== Exemple 4 : Lecture d'état ===")
    
    # Lire toutes les positions
    positions = motors.read_all_positions()
    print(f"Positions : {positions}")
    
    # Lire toutes les vitesses
    speeds = motors.read_all_speeds()
    print(f"Vitesses : {speeds}")
    
    # Afficher l'état d'un moteur
    print(f"\nÉtat moteur 1 : {motor1}")


# =============================
#   EXEMPLE 5 : CONVERSION ACCÉLÉRATION
# =============================

def example_acceleration_conversion():
    """Démonstration de la conversion d'accélération."""
    print("\n=== Exemple 5 : Conversion accélération ===")
    
    # Accélération en pulses/s²
    accel_linear = 800
    
    # Conversion vers paramètre MKS
    acc_param = Motor.linear_to_acc(accel_linear)
    print(f"{accel_linear} pulses/s² → acc_param = {acc_param}")
    
    # Conversion inverse
    accel_back = Motor.acc_to_linear(acc_param)
    print(f"acc_param {acc_param} → {accel_back:.1f} pulses/s²")
    
    # Table de correspondance
    print("\nTable de correspondance :")
    print("Acc MKS | Acc linéaire (pulses/s²)")
    print("--------|-------------------------")
    for acc_mks in [0, 50, 100, 150, 200, 250, 255]:
        linear = Motor.acc_to_linear(acc_mks)
        print(f"{acc_mks:7d} | {linear:15.0f}")


# =============================
#   EXEMPLES MODE ASYNCHRONE
# =============================

def example_async_speed_changes():
    """Test des changements de vitesse progressifs en mode asynchrone."""
    print("\n=== Exemple Async 1 : Changements de vitesse progressifs ===")
    
    # Activer le mode asynchrone sur les moteurs existants
    motor1.start_async_control()
    motor2.start_async_control()
    motor3.start_async_control()
    
    motors_list = [motor1, motor2, motor3]
    
    # Définir l'accélération par défaut
    for motor in motors_list:
        motor.state.acceleration = 250
    
    print("✅ Mode asynchrone activé sur tous les moteurs")
    
    # Changements de vitesse progressifs
    speeds = [50, -100, 150, 100, 200, -180]

    for speed in speeds:
        print(f"  → Consigne: {speed} RPM")
        for motor in motors_list:
            motor.set_target_velocity(speed)
        time.sleep(4)  # Le thread daemon applique le changement automatiquement
    
    print("✅ Test terminé")
    
    # Désactiver le mode asynchrone
    for motor in motors_list:
        motor.stop_async_control()
    
    print("✅ Mode asynchrone désactivé")

def example_async_square():
    print("\n=== Exemple Async 2 : Trajectoire carrée en mode asynchrone ===")
    # Activer le mode asynchrone sur les moteurs existants
    motor1.start_async_control()
    motor2.start_async_control()
    motor3.start_async_control()
        
    motors_list = [motor1, motor2, motor3]
    
    # Définir l'accélération par défaut
    for motor in motors_list:
        motor.state.acceleration = 250
    
    directions = [
    np.array([0, 1.0]),    # droite
    np.array([-1.0, 0]),   # haut
    np.array([0, -1.0]),  # gauche
    np.array([1.0, 0])    # bas
    ]
    vmax=100
    running = True
    try:
        step_duration = 3  # Durée de chaque segment (secondes)
        direction_index = 0
        t_start = time.time()
        sens=1.0
        # --- Exemple : trajectoire carrée (inatteignable à cause du 'continue' ci-dessus) ---
        while running:
            
            now = time.time()
            if now - t_start > step_duration:
                # Passage à la direction suivante
                direction_index = (direction_index + 1) % len(directions)
                t_start = now
                print("etape_suivante")
                direction = directions[direction_index]
                sens=sens*(-1.0)
                print(sens)
                raw_speeds = compute_wheel_speeds_global(robot, 0.0, 0.0,sens)
               
                motors.set_velocities(raw_speeds, max_speed=vmax)
            time.sleep(0.05)
    except Exception as e:
        print(f"Erreur dans la boucle principale : {e}")
    finally:
        pass

# exemple distance movement
def example_move_distance():
    
    directions = [
    np.array([0, -1.0,3000]),    # droite
    np.array([-1.0, 0,3000]),   # haut
    np.array([0, -1.0,3000]),  # gauche
    np.array([1.0, 0,3000])    # bas
    ]
    for direction in directions:
        distances=compute_wheel_distances_global(robot, direction[0]*direction[2], direction[1]*direction[2])
        print(f"distances to move: {distances}")
        motors.move_synchronized_distance(distances_mm=distances,base_acceleration=100, speed_rpm=200)  
        motors.wait_all_stopped()
        time.sleep(10)



# =============================
#   PROGRAMME PRINCIPAL
# =============================

if __name__ == "__main__":
    try:
        # Configuration initiale (décommenter si nécessaire)
        if False:
            print("⚙️ Configuration des moteurs...")
            for motor in [motor1, motor2, motor3]:
                motor.set_subdivisions(16)
                motor.set_work_mode(WorkMode.SrvFoc)
            
            time.sleep(1)
            
            # Calibration
            for motor in [motor1, motor2, motor3]:
                motor.calibrate()
            
            print("⏳ Calibration en cours (10s)...")
            time.sleep(10)
            
            # Reset zéro
            for motor in [motor1, motor2, motor3]:
                motor.reset_zero()
            
            time.sleep(2)
            print("✅ Configuration terminée\n")
            motor1.move_relative(direction=0, speed=1000, acceleration=250, pulses=100)
            motor2.move_relative(direction=0, speed=1000, acceleration=250, pulses=100)
            motor3.move_relative(direction=0, speed=1000, acceleration=250, pulses=100)
            time.sleep(0.2)     
            motor1.move_relative(direction=1, speed=1000, acceleration=250, pulses=100)
            motor2.move_relative(direction=1, speed=1000, acceleration=250, pulses=100)
            motor3.move_relative(direction=1, speed=1000, acceleration=250, pulses=100)
            time.sleep(2)    
        
        # Exécuter les exemples
        
        # --- Exemples mode synchrone ---
        # example_simple_movement()
        example_synchronized_movement()
        # example_velocity_control()
        # example_read_state()
        # example_acceleration_conversion()
        # example_move_distance()           # NOUVEAU: Déplacement par distance en mm
        # example_conversion_utilities()    # NOUVEAU: Utilitaires de conversion
        
        # --- Exemples mode asynchrone ---
        #example_async_square()
        # example_async_direction_changes()
        # example_async_square_trajectory()
        #example_move_distance()
        
    except KeyboardInterrupt:
        print("\n🛑 Interruption par l'utilisateur")
    
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Arrêt sécurisé de tous les moteurs
        print("\n🛑 Arrêt de tous les moteurs...")
    
        motors.stop_all(acceleration=255)
        time.sleep(0.5)
        print("✅ Arrêt complet")

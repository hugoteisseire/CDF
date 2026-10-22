# =============================
#      IMPORTS & CONSTANTS
# =============================
# sudo ip link set can0 up type can bitrate 1000000
# sudo ifconfig can0 txqueuelen 1000
import time
import threading
import can
import math
import numpy as np
from dataclasses import dataclass

# --- Custom library imports ---
from legacy.lib_moteur import (
    read_motor_speed, read_motor_position, move_relative, reset_zero,
    stop_pos_soft, move_velocity, stop_velocity_soft, calibrate
)
from core.holo_base import (
    Robot, compute_wheel_speeds_global, compute_base_velocity,
    compute_rotation_velocity, rotate_vector, shortest_angle_diff
)
# TODO: avoidance.py se trouve dans simu/simu_traj/ (pas dans ce dossier).
#       Cet import ne fonctionne pas tant qu'il n'est pas rendu accessible.
from avoidance import (
    compute_direction_champ, clamp_position, obstacles_rect,
    compute_direction_astar, cell_to_pos, compute_direction_gbfs,
    CELL_SIZE, normalize
)
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction, RunMotorResult




# =============================
#      CAN INITIALIZATION
# =============================

# CAN bus setup
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
bus.socket.setblocking(False)

# Servo motor objects
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)

# --- Optional: Initial configuration and calibration (set to False by default) ---
if True:
    print(servo1.set_subdivisions(16))
    print(servo3.set_subdivisions(16))
    print(servo2.set_subdivisions(16))
    servo1.set_work_mode(WorkMode.SrvFoc)
    servo2.set_work_mode(WorkMode.SrvFoc)
    servo3.set_work_mode(WorkMode.SrvFoc)
    time.sleep(1)
    calibrate(bus=bus, can_id=0x03)
    calibrate(bus=bus, can_id=0x01)
    calibrate(bus=bus, can_id=0x02)
    time.sleep(10)
    reset_zero(bus, can_id=0x02)
    reset_zero(bus, can_id=0x01)
    reset_zero(bus, can_id=0x03)
    time.sleep(2)     
move_relative(bus, can_id=0x01, direction=0, speed=1000, acceleration=250, pulses=100)
move_relative(bus, can_id=0x02, direction=0, speed=1000, acceleration=250, pulses=100)
move_relative(bus, can_id=0x03, direction=0, speed=1000, acceleration=250, pulses=100)
time.sleep(0.2)     
move_relative(bus, can_id=0x01, direction=1, speed=1000, acceleration=250, pulses=100)
move_relative(bus, can_id=0x02, direction=1, speed=1000, acceleration=250, pulses=100)
move_relative(bus, can_id=0x03, direction=1, speed=1000, acceleration=250, pulses=100)
time.sleep(2)    


# =============================
#      TABLE & ROBOT PARAMS
# =============================

# Table dimensions (mm)
TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000

# Robot parameters
ROBOT_RADIUS_MM = 150
ROBOT_WHEEL_RADIUS = 58 / 2
ROBOT_RADIUS_WHEEL_POS = 100  # Radius of the circle including the wheels
ROBOT_TICK_PER_TURN = 16 * 200

# =============================
#      ROBOT INSTANCE
# =============================

robot = Robot(
    pos=np.array([1000.0, 1000.0]),
    angle=3.1515/2,
    radius=ROBOT_RADIUS_MM,
    wheel_angles=[math.radians(120), math.radians(240), math.radians(0)]
)
robot.wheel_speeds = [0.0, 0.0, 0.0]
robot.base_velocity = np.array([0.0, 0.0])
robot.base_velocity_global = np.array([0.0, 0.0])

target_pos = np.array([2500.0, 1000.0])
running = True


# =============================
#      UTILITY FUNCTIONS
# =============================

# Cette fonction renvoie les vitesses pour chaque roue pour aller dans la direction "direction".
# TODO :
# - Ajouter à holo_base une fonction qui calcule le nb de tck de chaque roue pour aller à un point (en incluant une rotation cible comme compute_wheel_speeds_global)
# - Tester raw speed temporellement afin de faire un carré (4 directions à donner pendant un temps)
# - Faire un environnement propice au développement d'un déplacement du robot (plusieurs modes : vitesse, arrêt, déplacement en position)

def clear_can_buffer(bus):
    """
    Vide le buffer du bus CAN en lisant tous les messages disponibles.
    """
    while True:
        try:
            msg = bus.recv(timeout=0.01)  # Lit un message avec un timeout court
            if msg is None:
                break  # Plus de messages dans le buffer
        except Exception as e:
            print(f"Erreur lors de la lecture du buffer : {e}")
            break



def set_wheel_speeds(robot, speeds, base_acceleration=150, speed_multiplier=0):
    """
    Applique une vitesse à chaque roue, avec une accélération proportionnelle pour
    que toutes les roues atteignent leur cible simultanément.

    base_acceleration : valeur d'accélération pour la roue la plus rapide (max = 255)
    """
    speeds = [s * speed_multiplier for s in speeds]
    print(speeds)
    abs_speeds = [abs(s) for s in speeds]
    max_speed = max(abs_speeds)
    if max_speed == 0:
        return [0] * len(speeds)

    # Calcul de T_total (temps commun pour atteindre Vmax)
    # On choisit T_total = max_speed * (256 - base_acceleration)
    # Puis on ajuste pour que acc_i reste dans [0, 255]
    T_total = max_speed * (256 - base_acceleration)
    print(T_total)
    # Calcul des acc_i pour chaque moteur
    accelerations = []
    for s in abs_speeds:
        if s == 0:
            accelerations.append(0)
        else:
            # calcul de r
            r = max_speed / s
            acc = 256 - r * (256 - base_acceleration)
            print(acc)
            T_total = s * (256 - acc)
            print(T_total)
            acc = max(1, min(255, acc))  # On s'assure que acc est dans [0, 255]
            accelerations.append(acc)

    for i, (speed, acc) in enumerate(zip(speeds, accelerations)):
        direction = 0 if speed >= 0 else 1
        rpm = int(abs(speed))  # vitesses normalisées [0,1] → [0,3000 RPM]
        print(f"Moteur {i+1}: direction={direction}, rpm={rpm}, acc={acc}")
        move_velocity(bus, i + 1, direction, rpm, round(int(acc)))

    robot.wheel_speeds = speeds


def set_wheel_speeds2(robot, speeds, base_acceleration=150, speed_multiplier=0):
    """
    Met à jour la vitesse et la direction de chaque moteur dans la structure Motors (pour contrôle par thread).
    """
    speeds = [s * speed_multiplier for s in speeds]
    print(speeds)
    for i, s in enumerate(speeds):
        direction = 0 if s >= 0 else 1
        rpm = int(abs(s))
        Motors[i].speed = rpm
        Motors[i].direction = direction
    robot.wheel_speeds = speeds



# =============================
#      MOTOR STATE CLASS
# =============================

@dataclass
class Motor:
    speed: int
    lspeed: int
    accel: int
    direction: bool
    ldirection: bool
    state: int = 1  # 1: idle, 2: speed change, 3: direction change

# Liste d'objets Motor (un par roue)
Motors = [Motor(0, 0, 0, 0, 0, 1) for _ in range(3)]


def control_motor(motor_id, stop_event):
    """
    Boucle de contrôle d'un moteur (thread). Gère les changements de vitesse et de direction.
    """
    while not stop_event.is_set():
        # Attente d'un changement de vitesse ou direction
        if Motors[motor_id].state == 1:
            if Motors[motor_id].lspeed != Motors[motor_id].speed:
                Motors[motor_id].state = 2
            if Motors[motor_id].ldirection != Motors[motor_id].direction:
                Motors[motor_id].state = 3

        # Changement de vitesse
        elif Motors[motor_id].state == 2:
            if Motors[motor_id].ldirection != Motors[motor_id].direction:
                Motors[motor_id].state = 3
            else:
                move_velocity(bus, motor_id + 1, Motors[motor_id].direction, Motors[motor_id].speed, Motors[motor_id].accel)
                Motors[motor_id].lspeed = Motors[motor_id].speed
                Motors[motor_id].state = 1

        # Changement de direction : stop, attend, puis nouvelle direction
        elif Motors[motor_id].state == 3:
            move_velocity(bus, motor_id + 1, not Motors[motor_id].direction, 0, 250)
            # Attente de l'arrêt
            # on = True
            #while on:
            #    on = servo1.is_motor_running()
            time.sleep(0.1)  # Petite pause pour s'assurer de l'arrêt
            move_velocity(bus, motor_id + 1, Motors[motor_id].direction, Motors[motor_id].speed, Motors[motor_id].accel)
            Motors[motor_id].ldirection = Motors[motor_id].direction
            Motors[motor_id].lspeed = Motors[motor_id].speed
            Motors[motor_id].state = 1

        else:
            print("[ERREUR] État inconnu pour le moteur")



def stop_all_wheels(robot, base_acceleration=30):
    """
    Stoppe toutes les roues avec une accélération proportionnelle à leur vitesse actuelle.
    """
    stop_velocity_soft(bus,  1, 150)
    stop_velocity_soft(bus,  2, 150)
    stop_velocity_soft(bus,  3, 150)
    return 
    abs_speeds = [abs(s) for s in robot.wheel_speeds]
    max_speed = max(abs_speeds)
    if max_speed == 0:
        accelerations = [base_acceleration] * 3
    else:
        accelerations = [
            max(1, int(base_acceleration * (abs(s) / max_speed))) for s in abs_speeds
        ]
    for i, acc in enumerate(accelerations):
        stop_velocity_soft(bus, i + 1, 150)
        

def acc_to_linear(acc_param, ticks_per_rev=3200):
    """
    Convert MKS acceleration parameter to linear acceleration.
    
    Args:
        acc_param: MKS acceleration (0-255)
        ticks_per_rev: Ticks per motor revolution
    
    Returns:
        Acceleration in pulses/s²
    """
    if acc_param >= 256:
        return float('inf')
    rpm_per_s = 20_000 / (256 - acc_param)
    pulses_per_s2 = rpm_per_s * ticks_per_rev / 60
    return pulses_per_s2

def linear_to_acc(accel_pulses_s2):
    """
    Convert linear acceleration to MKS acceleration parameter.
    """
    
 
    if accel_pulses_s2 <= 0:
        return 255  # Maximum acceleration (instantaneous)
    acc = 256 - (20000 / accel_pulses_s2)
    return max(0, min(255, int(acc)))
# =============================
#      TEST TRAJECTORY (SQUARE)
# =============================

# Liste des directions à suivre (vecteurs unitaires)
directions = [
    np.array([0, 1.0]),    # droite
    np.array([-1.0, 0]),   # haut
    np.array([0, -1.0]),  # gauche
    np.array([1.0, 0])    # bas
]

# --- Boucle principale ---
try:
    clear_can_buffer(bus)

    step_duration = 4  # temps de chaque segment en secondes
    direction_index = 0
    t_start = time.time()
    on=True
    vmax=200
    accel=250
     # Threading pour le contrôle moteur
    stop_event = threading.Event()
    threads = []
    for i in range(3):
        t = threading.Thread(target=control_motor, args=(i, stop_event))
        t.start()
        threads.append(t)
    for i in range(3):
        Motors[i].accel = accel

    # =============================
    #      MAIN CONTROL LOOP
    # =============================

    try:
        step_duration = 3  # Durée de chaque segment (secondes)
        direction_index = 0
        t_start = time.time()
        vmax = 200
        accel = 250
        # --- Exemple : trajectoire carrée (inatteignable à cause du 'continue' ci-dessus) ---
        while running:
            now = time.time()
            if now - t_start > step_duration:
                # Passage à la direction suivante
                direction_index = (direction_index + 1) % len(directions)
                t_start = now
                print("etape_suivante")
                direction = directions[direction_index]
                raw_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)
                set_wheel_speeds2(robot, raw_speeds, base_acceleration=150, speed_multiplier=10)
            time.sleep(0.05)
            clear_can_buffer(bus)
    except Exception as e:
        print(f"Erreur dans la boucle principale : {e}")
    finally:
        pass


    # =============================
    #      MAIN CONTROL LOOP
    #  pos mod, synchronisé, fonctionnel
    # ============================
    try:
        step_duration = 5  # Durée de chaque segment (secondes)
        direction_index = 0
        t_start = time.time()
        speed = 80
        accel_general = 800
        pulses_per_step = 10000  # Nombre de pas pour chaque segment
        
        print("🔄 Début du test move_relative - Trajectoire carrée")
        
        while running:
            now = time.time()
            if True:
                # Passage à la direction suivante
                direction_index = (direction_index + 1) % len(directions)
                t_start = now
                print(f"📍 Étape {direction_index + 1}/4 - Direction: {directions[direction_index]}")
                
                # Calcul des vitesses de roues pour la direction actuelle
                direction = directions[direction_index]
                raw_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)
                
                # Envoi de move_relative pour chaque moteur
                # Synchronisation parfaite : vitesses ET accélérations proportionnelles aux distances
                max_abs_speed = max(abs(s) for s in raw_speeds)
                
                for i, wheel_speed in enumerate(raw_speeds):
                    motor_direction = 0 if wheel_speed >= 0 else 1
                    
                    # Vitesse proportionnelle à la distance (via wheel_speed normalisé)
                    speed_ratio = abs(wheel_speed) / max_abs_speed if max_abs_speed > 0 else 1.0
                    motor_speed = int(speed_ratio * speed)
                    pulses = int(speed_ratio * pulses_per_step)
                    
                    # Accélération proportionnelle : même ratio que la vitesse
                    # Si v1 = 0.5*v2, alors a1 = 0.5*a2 → temps pour atteindre Vmax identique
                    accel_pulses_s2 = speed_ratio * accel_general
                    acc_param = linear_to_acc(accel_pulses_s2)
                    
                    print(f"  Moteur {i+1}: ratio={speed_ratio:.2f}, speed={motor_speed} RPM, pulses={pulses}, acc={acc_param} ({accel_pulses_s2:.0f} pulses/s²)")
                    move_relative(bus, i + 1, direction=motor_direction, speed=motor_speed, acceleration=acc_param, pulses=pulses)
            time.sleep(0.5)
            # Attente que tous les moteurs soient à l'arrêt
            all_stopped = False
            while not all_stopped:
                motor1_running = servo1.is_motor_running()
                motor2_running = servo2.is_motor_running()
                motor3_running = servo3.is_motor_running()
                all_stopped = not (motor1_running or motor2_running or motor3_running)
                if not all_stopped:
                    time.sleep(0.01)
            print("✅ Tous les moteurs sont à l'arrêt")
                
        clear_can_buffer(bus)
            
    except Exception as e:  
        print(f"❌ Erreur dans la boucle principale : {e}")
    finally:
        print("🛑 Arrêt du test move_relative")
except KeyboardInterrupt:
    print("🛑 Interruption par l'utilisateur")
finally:
    stop_all_wheels(robot)
    stop_event.set()  # Signale aux threads de s'arrêter
    for t in threads:
        t.join()  # Attend la fin des threads
    print("✅ Robot arrêté proprement.")


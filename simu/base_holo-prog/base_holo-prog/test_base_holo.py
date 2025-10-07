import time
import can
from lib_moteur import read_motor_speed,read_motor_position,move_relative,reset_zero,stop_pos_soft,move_velocity,stop_velocity_soft,calibrate
import pygame
import sys
import math
import numpy as np
import random
import psutil
import os
from holo_base import (
    Robot,
    compute_wheel_speeds_global,
    compute_base_velocity,
    compute_rotation_velocity,
    rotate_vector,
    shortest_angle_diff
)
from avoidance import compute_direction_champ, clamp_position, obstacles_rect,compute_direction_astar,cell_to_pos,compute_direction_gbfs,CELL_SIZE,normalize
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction
from mks_servo_can.mks_enums import CalibrationResult, RunMotorResult

# Initialisation CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
bus.socket.setblocking(False)

servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)
if False:
    print(servo1.set_subdivisions(16))
    print(servo3.set_subdivisions(16))
    print(servo2.set_subdivisions(16))
    servo1.set_work_mode(WorkMode.SrvFoc)
    servo2.set_work_mode(WorkMode.SrvFoc)
    servo3.set_work_mode(WorkMode.SrvFoc)
    time.sleep(1)
    calibrate(bus=bus,can_id=0x03)
    calibrate(bus=bus,can_id=0x01)
    calibrate(bus=bus,can_id=0x02)
    time.sleep(10)
    reset_zero(bus, can_id=0x02)
    reset_zero(bus, can_id=0x01)
    reset_zero(bus, can_id=0x03)
    time.sleep(2)


# --- Dimensions réelles de la table (en mm) ---
TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000

# --- Paramètres divers ---
ROBOT_RADIUS_MM = 150
ROBOT_WHEEL_RADIUS =58/2
#rayont du cercle incluant les roue:
ROBOT_RADIUS_WHEEL_POS=100
ROBOT_TICK_PER_TURN=16*200

# --- Entities ---
robot = Robot(pos=np.array([1000.0, 1000.0]), angle=0.0, radius=ROBOT_RADIUS_MM, wheel_angles=[math.radians(90), math.radians(210), math.radians(330)])
robot.wheel_speeds = [0.0, 0.0, 0.0]
robot.base_velocity = np.array([0.0, 0.0])
robot.base_velocity_global = np.array([0.0, 0.0])

target_pos = np.array([2500.0, 1000.0])
running=True  

#//cette fonction renvoit les vitesse pour chaque roue pour aller dans la direction "direction"
# a faire :
# rajouter a holo base une fonction qui calcule le nb de tck de chaque roue pour aller a un point (en inclant une rotation cible comme compute_wheel_speeds_global(il inclut une rota constante lui) )
# tester raw speed temporelment afin de faire un carré ( 4 direction a lui donner pendant un temps)
#faire un environement propice au devellopement d'un deplacement du robot plusieur mode: vitesse dans une direction, arret du robot avec accel de chauque roue proportionel a sa vitesse, deplacement en possition 


def set_wheel_speeds(robot, speeds, base_acceleration=3,speed_multiplier=0):
    """
    Applique une vitesse à chaque roue, avec une accélération proportionnelle pour
    que toutes les roues atteignent leur cible simultanément.

    base_acceleration : valeur d'accélération pour la roue la plus rapide (max = 255)
    """
    speeds=[s* speed_multiplier for s in speeds]
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
            #calcul de r
            r=max_speed/s
            acc = 256 - r*(256-base_acceleration) 
            print(acc)
            T_total = s * (256 - acc)
            print(T_total)
            acc = max(1, min(255, acc))  # On s'assure que acc est dans [0, 255]
            accelerations.append(acc)


    for i, (speed, acc) in enumerate(zip(speeds, accelerations)):
        direction = 0 if speed >= 0 else 1
        rpm = int(abs(speed) )  # vitesses normalisées [0,1] → [0,3000 RPM]
        print(f"Moteur {i+1}: direction={direction}, rpm={rpm}, acc={acc}")
        move_velocity(bus, i + 1, direction, rpm, round(int(255)))

    robot.wheel_speeds = speeds


def stop_all_wheels(robot, base_acceleration=30):
    """
    Stoppe les roues avec des accélérations proportionnelles à leur vitesse actuelle.
    """
    abs_speeds = [abs(s) for s in robot.wheel_speeds]
    max_speed = max(abs_speeds)
    if max_speed == 0:
        accelerations = [base_acceleration] * 3
    else:
        accelerations = [
            max(1, int(base_acceleration * (abs(s) / max_speed))) for s in abs_speeds
        ]

    for i, acc in enumerate(accelerations):
        stop_velocity_soft(bus, i + 1, acc)

# Liste des directions à suivre (en X, Y), unitaires
directions = [
    np.array([1.0, 1]),   # droite
    np.array([-1, 1.0]),   # haut
    np.array([-1.0, -1.0]),  # gauche
    np.array([1.0, -1.0])   # bas
]


# --- Boucle principale ---
try:
    step_duration = 2  # temps de chaque segment en secondes
    direction_index = 0
    t_start = time.time()

    while running:
        now = time.time()

        if now - t_start > step_duration:
            # Passage à la direction suivante
            direction_index = (direction_index + 1) % len(directions)
            t_start = now
            print("etape_suivante")
            direction = directions[direction_index]
            raw_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)

            # Application des vitesses
            set_wheel_speeds(robot, raw_speeds, base_acceleration=230,speed_multiplier=500)
            

        # Direction actuelle
        

        time.sleep(0.05)

except KeyboardInterrupt:
    print("🛑 Interruption par l'utilisateur")
finally:
    stop_all_wheels(robot)
    print("✅ Robot arrêté proprement.")
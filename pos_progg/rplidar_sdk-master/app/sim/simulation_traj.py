import pygame
import sys
import math
import numpy as np
import random
import time
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
from avoidance import compute_direction_champ, clamp_position, obstacles_rect


# --- Dimensions réelles de la table (en mm) ---
TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000

# --- Dimensions de la fenêtre d'affichage (en pixels) ---
WIDTH, HEIGHT = 1000, 667
SCALE_X = WIDTH / TABLE_WIDTH_MM
SCALE_Y = HEIGHT / TABLE_HEIGHT_MM
SCALE = min(SCALE_X, SCALE_Y)

# --- Conversion logique vers affichage ---
def to_screen(pos_mm):
    return np.array([pos_mm[0] * SCALE, pos_mm[1] * SCALE])

def from_screen(pos_px):
    return np.array([pos_px[0] / SCALE, pos_px[1] / SCALE])

def mm_per_tick_to_mps(value_mm_per_tick):
    return value_mm_per_tick * TICKS_PER_SECOND / 1000.0  # mm/tick -> m/s

# --- Paramètres divers ---
ROBOT_RADIUS_MM = 150
TARGET_RADIUS_MM = 20
ENEMY_RADIUS_MM = 150
TICKS_PER_SECOND = 60
MAX_WHEEL_SPEED_MPS = 0.3  # vitesse max des roues

# --- Init Pygame ---
display = True
pygame.init()
if display:
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulateur Robot - Évitement d'obstacle")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

# --- Entities ---
robot = Robot(pos=np.array([1000.0, 1000.0]), angle=0.0, radius=ROBOT_RADIUS_MM, wheel_angles=[math.radians(90), math.radians(210), math.radians(330)])
robot.wheel_speeds = [0.0, 0.0, 0.0]
robot.base_velocity = np.array([0.0, 0.0])
robot.base_velocity_global = np.array([0.0, 0.0])

target_pos = np.array([2500.0, 1000.0])
enemy_pos = np.array([1500.0, 1000.0])

ENEMY_CAPTURE_THRESHOLD = 200
pause_robot = False
pause_start_time = None
PAUSE_DURATION = 3

enemy_move_timer = 0
enemy_move_interval = 2
enemy_velocity = np.array([0.0, 0.0])

def random_position(margin_mm=100):
    return np.array([
        random.uniform(margin_mm, TABLE_WIDTH_MM - margin_mm),
        random.uniform(margin_mm, TABLE_HEIGHT_MM - margin_mm)
    ])

# --- Affichage ---
def affichage_robot(robot, screen):
    pygame.draw.circle(screen, (0, 100, 255), to_screen(robot.pos).astype(int), int(robot.radius * SCALE))
    for i, theta in enumerate(robot.wheel_angles):
        wheel_dir = np.array([math.cos(theta), math.sin(theta)])
        tangent_dir = np.array([-math.sin(theta), math.cos(theta)])
        wheel_pos = robot.pos + rotate_vector(wheel_dir, robot.angle) * robot.radius
        tangent_rotated = rotate_vector(tangent_dir, robot.angle)
        tip = wheel_pos + tangent_rotated * robot.wheel_speeds[i] * 40
        pygame.draw.line(screen, (255, 0, 0), to_screen(wheel_pos), to_screen(tip), 3)
        pygame.draw.circle(screen, (0, 0, 0), to_screen(wheel_pos).astype(int), 5)
    pygame.draw.line(screen, (0, 100, 255), to_screen(robot.pos), to_screen(robot.pos + robot.base_velocity_global * 150), 4)

def affichage_vitesses(screen, robot, font):
    lines = []
    for i, s in enumerate(robot.wheel_speeds):
        s_mps = mm_per_tick_to_mps(s)
        lines.append(f"Roue {i+1} : {s_mps:.2f} m/s")
    vitesse = robot.base_velocity_global
    vx, vy = mm_per_tick_to_mps(vitesse[0]), mm_per_tick_to_mps(vitesse[1])
    lines.append(f"Vitesse réelle X : {vx:.2f} m/s")
    lines.append(f"Vitesse réelle Y : {vy:.2f} m/s")
    lines.append(f"Norme réelle : {math.hypot(vx, vy):.2f} m/s")
    x, y = WIDTH - 220, 10
    for line in lines:
        text_surface = font.render(line, True, (0, 0, 0))
        screen.blit(text_surface, (x, y))
        y += 20

def affichage_table(screen):
    screen.fill((255, 255, 255))
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, WIDTH, HEIGHT), 5)
    pygame.draw.rect(screen, (230, 230, 230), (int(50 * SCALE), int(50 * SCALE), WIDTH - int(100 * SCALE), HEIGHT - int(100 * SCALE)))
    pygame.draw.circle(screen, (255, 0, 0), to_screen(enemy_pos).astype(int), int(ENEMY_RADIUS_MM * SCALE))
    pygame.draw.circle(screen, (0, 0, 255), to_screen(target_pos).astype(int), int(TARGET_RADIUS_MM * SCALE))
    for rect in obstacles_rect:
        pygame.draw.rect(screen, (100, 100, 100), pygame.Rect(to_screen(np.array(rect[:2])), to_screen(np.array(rect[2:]))))

# --- Main loop ---
running = True
move_target = False
robot_moving = False

while running:
    start_time=time.perf_counter()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_t:
                move_target = not move_target
            if event.key == pygame.K_s:
                robot_moving = not robot_moving
            if event.key == pygame.K_h:
                robot.pos = from_screen(pygame.mouse.get_pos())

    if pygame.mouse.get_pressed()[0]:
        mouse_pos_mm = from_screen(pygame.mouse.get_pos())
        if move_target:
            target_pos = clamp_position(mouse_pos_mm, TARGET_RADIUS_MM, TABLE_WIDTH_MM, TABLE_HEIGHT_MM)
        else:
            enemy_pos = clamp_position(mouse_pos_mm, ENEMY_RADIUS_MM, TABLE_WIDTH_MM, TABLE_HEIGHT_MM)

    enemy_move_timer += 1 / TICKS_PER_SECOND
    if enemy_move_timer > enemy_move_interval:
        enemy_move_timer = 0
        enemy_velocity = (random_position() - enemy_pos)
        enemy_velocity /= np.linalg.norm(enemy_velocity) + 1e-5
        enemy_velocity *= 5  # mm par tick

    #enemy_pos += enemy_velocity
    enemy_pos = clamp_position(enemy_pos, ENEMY_RADIUS_MM, TABLE_WIDTH_MM, TABLE_HEIGHT_MM)

    distance_enemy_target = np.linalg.norm(enemy_pos - target_pos)
    distance_me_target = np.linalg.norm(robot.pos - target_pos)
    if distance_enemy_target < ENEMY_CAPTURE_THRESHOLD and distance_me_target < ENEMY_CAPTURE_THRESHOLD * 1.5:
        pause_robot = True
        pause_start_time = time.time()
    else:
        pause_robot = False

    direction = compute_direction_champ(robot, target_pos, enemy_pos, TABLE_WIDTH_MM, TABLE_HEIGHT_MM)

    if robot_moving:
        raw_speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0.0)
        max_speed_mm_tick = (MAX_WHEEL_SPEED_MPS * 1000) / TICKS_PER_SECOND
        if not pause_robot:
            robot.wheel_speeds = [s * max_speed_mm_tick for s in raw_speeds]
        else:
            robot.wheel_speeds = [0, 0, 0]
        robot.base_velocity = compute_base_velocity(robot, robot.wheel_speeds)
        robot.base_velocity_global = rotate_vector(robot.base_velocity, robot.angle)
        robot.pos += robot.base_velocity_global
        robot.angle += compute_rotation_velocity(robot, robot.wheel_speeds)

        if np.linalg.norm(robot.pos - target_pos) < TARGET_RADIUS_MM + robot.radius:
            target_pos = random_position()
    if display:
        affichage_table(screen)
        affichage_robot(robot, screen)
        affichage_vitesses(screen, robot, font)

    if pause_robot and display:
        pygame.draw.rect(screen, (255, 0, 0), (WIDTH//2 - 100, HEIGHT//2 - 30, 200, 60))
        text = font.render("PAUSE: ENNEMI SUR LA CIBLE", True, (255, 255, 255))
        screen.blit(text, (WIDTH//2 - text.get_width()//2, HEIGHT//2 - 10))

    if display:
        pygame.display.flip()
    clock.tick(TICKS_PER_SECOND)
    debug_perf=True
    if debug_perf :
        end_time = time.perf_counter()  # Fin du chrono
        elapsed = end_time - start_time
        print(f"Frame time: {elapsed*1000:.2f} ms")
        process = psutil.Process(os.getpid())
        mem_info = process.memory_info()
        print(f"Mémoire utilisée : {mem_info.rss / (1024 * 1024):.2f} Mo")


pygame.quit()
sys.exit()

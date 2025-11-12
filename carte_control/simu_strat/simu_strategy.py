#!/usr/bin/env python3
"""
Simulateur de programme de stratégie.
Ce programme sera lancé lors de l'initialisation.
Il lit les données LIDAR via le socket broadcast et les parse.
"""

import time
import sys
import signal
import os
import math
import pygame

# Ajouter le répertoire parent au path pour importer le module de lecture dédié
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lidar_reader import LidarReader

# Configuration
#LIDAR_DATA_SOCKET = '/tmp/robot.sock'
LIDAR_DATA_SOCKET = '/tmp/lidar_data.sock'
running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n🛑 Stratégie arrêtée proprement")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)




def main():
    print("🎯 Programme STRATÉGIE démarré")
    print("Visualisation LIDAR/IMU/Adversaire - vue table complète")

    # Démarrer le thread de lecture LIDAR
    lidar_reader = LidarReader(socket_path=LIDAR_DATA_SOCKET)
    lidar_reader.start()

    # ====== Config visualisation ======
    # Vue complète de la table (centrée dans la fenêtre) avec un peu de marge
    MARGIN_PX = 40         # marge autour de la table

    # Taille fenêtre en pixels (ratio 3:2 par défaut pour table 3000x2000)
    SCREEN_W, SCREEN_H = 1200, 800
    GRID_STEP_MM = 500  # pas de la grille

    # Table (optionnel), pour afficher une image de fond si fournie
    TABLE_W_MM = 3000
    TABLE_H_MM = 2000
    TABLE_BACKGROUND_IMAGE = None  # Exemple: r"c:/path/your_table.png" ou '/path/table.png'
    show_background = False if TABLE_BACKGROUND_IMAGE is None else True

    # Init pygame
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Stratégie - Vue LIDAR/IMU")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 20)

    bg_img = None
    if TABLE_BACKGROUND_IMAGE and os.path.isfile(TABLE_BACKGROUND_IMAGE):
        try:
            bg_img = pygame.image.load(TABLE_BACKGROUND_IMAGE).convert_alpha()
        except Exception as e:
            print(f"⚠️ Impossible de charger l'image de fond: {e}")
            bg_img = None
            show_background = False

    def compute_scale_full_table():
        # Échelle pour faire rentrer la table + marge dans la fenêtre
        return min((SCREEN_W - 2 * MARGIN_PX) / TABLE_W_MM,
                   (SCREEN_H - 2 * MARGIN_PX) / TABLE_H_MM)

    def world_to_screen_full(x_mm: float, y_mm: float, scale: float):
        # Table centrée dans la fenêtre (0,0) en bas-gauche en monde
        table_w_px = TABLE_W_MM * scale
        table_h_px = TABLE_H_MM * scale
        ox = (SCREEN_W - table_w_px) / 2.0
        oy = (SCREEN_H + table_h_px) / 2.0  # car Y écran est inversé
        sx = ox + x_mm * scale
        sy = oy - y_mm * scale
        return int(sx), int(sy)

    # Couleurs
    COLOR_BG = (10, 10, 10)
    GRID_COLOR = (35, 35, 35)
    COLOR_IMU = (50, 150, 255)
    COLOR_LIDAR = (50, 255, 100)
    COLOR_TRACKED_FOUND = (255, 80, 80)
    COLOR_TRACKED_MISSING = (140, 140, 140)
    COLOR_ENEMY = (250, 200, 0)

    R_IMU = 6
    R_LIDAR = 6
    R_TRACK = 4
    R_ENEMY = 7

    running_loop = True
    while running and running_loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running_loop = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running_loop = False
                elif event.key == pygame.K_b:
                    show_background = not show_background
                # (mode centré robot supprimé)

        # Dernières données LIDAR parsées
        lidar_data = lidar_reader.get_last_data()

        # Configurer la transformation monde->écran (vue table complète uniquement)
        scale = compute_scale_full_table()
        def w2s(x, y):
            return world_to_screen_full(x, y, scale)

        # Dessin de fond
        screen.fill(COLOR_BG)

        # Image de fond (si fournie): on la met à l'échelle mm->px et on la positionne
        if show_background and bg_img is not None:
            img_w = max(1, int(TABLE_W_MM * scale))
            img_h = max(1, int(TABLE_H_MM * scale))
            bg_scaled = pygame.transform.smoothscale(bg_img, (img_w, img_h))
            # Coin haut-gauche de l'image en full-table
            table_w_px = TABLE_W_MM * scale
            table_h_px = TABLE_H_MM * scale
            ox = (SCREEN_W - table_w_px) / 2.0
            oy = (SCREEN_H + table_h_px) / 2.0
            screen.blit(bg_scaled, (ox, oy - img_h))

        # Grille
        # Grille sur toute la table
        for gx in range(0, int(TABLE_W_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = w2s(gx, 0)
            x2, y2 = w2s(gx, TABLE_H_MM)
            pygame.draw.line(screen, GRID_COLOR, (x1, y1), (x2, y2), 1)

        for gy in range(0, int(TABLE_H_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = w2s(0, gy)
            x2, y2 = w2s(TABLE_W_MM, gy)
            pygame.draw.line(screen, GRID_COLOR, (x1, y1), (x2, y2), 1)

        # Dessins selon données disponibles
        if lidar_data:
            # IMU
            imu_sx, imu_sy = w2s(lidar_data.pos_imu.x, lidar_data.pos_imu.y)
            pygame.draw.circle(screen, COLOR_IMU, (imu_sx, imu_sy), R_IMU)
            # Orientation IMU
            imu_len = int(60 * scale)  # 60 mm
            ix2 = imu_sx + int(math.cos(lidar_data.pos_imu.angle) * imu_len)
            iy2 = imu_sy - int(math.sin(lidar_data.pos_imu.angle) * imu_len)
            pygame.draw.line(screen, COLOR_IMU, (imu_sx, imu_sy), (ix2, iy2), 2)

            # LIDAR
            lidar_sx, lidar_sy = w2s(lidar_data.pos_lidar.x, lidar_data.pos_lidar.y)
            pygame.draw.circle(screen, COLOR_LIDAR, (lidar_sx, lidar_sy), R_LIDAR)

            # Tracked pillars (polaires depuis IMU)
            for tp in (lidar_data.pillar_1, lidar_data.pillar_2, lidar_data.pillar_3):
                abs_angle = lidar_data.pos_imu.angle + tp.angle
                tx = lidar_data.pos_imu.x + tp.distance * math.cos(abs_angle)
                ty = lidar_data.pos_imu.y + tp.distance * math.sin(abs_angle)
                sx, sy = w2s(tx, ty)
                color = COLOR_TRACKED_FOUND if tp.found else COLOR_TRACKED_MISSING
                pygame.draw.circle(screen, color, (sx, sy), R_TRACK)
                # Ligne depuis IMU
                pygame.draw.line(screen, color, (imu_sx, imu_sy), (sx, sy), 1)

            # Robot adverse (si non nul)
            adv_x, adv_y = lidar_data.pos_adv
            if not (adv_x == 0.0 and adv_y == 0.0):
                ex, ey = w2s(adv_x, adv_y)
                pygame.draw.circle(screen, COLOR_ENEMY, (ex, ey), R_ENEMY)

            # Bandeau info
            t1 = font.render(
                f"État: {lidar_data.state} | IMU x={lidar_data.pos_imu.x:.0f} y={lidar_data.pos_imu.y:.0f} ang={lidar_data.pos_imu.angle_deg():.1f}°",
                True, (200, 200, 200)
            )
            screen.blit(t1, (10, 10))
        else:
            t_wait = font.render("En attente de données LIDAR... (B pour background)", True, (200, 200, 200))
            screen.blit(t_wait, (10, 10))

        pygame.display.flip()
        clock.tick(30)

    # Fin propre
    lidar_reader.stop()
    pygame.quit()

if __name__ == "__main__":
    main()

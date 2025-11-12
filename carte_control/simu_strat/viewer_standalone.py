#!/usr/bin/env python3
"""
Viewer standalone (Windows compatible) pour tester uniquement l'affichage de la table,
IMU, LIDAR, piliers et robot adverse sans aucune communication réseau.

Contrôles:
  ESC       : quitter
  B         : toggle image de fond (si définie)
  P         : pause animation
  A/Z       : augmenter/diminuer vitesse de rotation IMU
  E         : bascule adversaire visible / caché
  S         : screenshot (PNG)

Simulation:
  - IMU tourne lentement et se déplace sur une trajectoire en 8.
  - LIDAR est décalé légèrement par rapport à l'IMU.
  - Piliers: trois piliers simulés en coordonnées absolues.
  - Robot adverse: orbite autour du centre.
"""

import math
import os
import time
import pygame
from dataclasses import dataclass

# Dimensions de la table (mm)
TABLE_W_MM = 3000
TABLE_H_MM = 2000
MARGIN_PX = 40   # marge autour de la table
GRID_STEP_MM = 500

# Fenêtre (ratio adapté 3:2)
SCREEN_W, SCREEN_H = 1200, 800

# Image de fond optionnelle (mettre un chemin valide si souhaité)
TABLE_BACKGROUND_IMAGE = r"C:\Users\NBcra\Desktop\CDF\carte_control\simu_strat\table.png"  # ex: r"C:/temp/table.png"

# Couleurs
COLOR_BG = (10, 10, 10)
GRID_COLOR = (35, 35, 35)
COLOR_IMU = (50, 150, 255)
COLOR_LIDAR = (50, 255, 100)
COLOR_TRACKED_FOUND = (255, 80, 80)
COLOR_TRACKED_MISSING = (140, 140, 140)
COLOR_ENEMY = (250, 200, 0)
COLOR_TEXT = (210, 210, 210)

R_IMU = 6
R_LIDAR = 6
R_TRACK = 4
R_ENEMY = 7

@dataclass
class SimPillar:
    x: float
    y: float
    present: bool = True

# Piliers fixes (ex: trois positions sur la table)
PILLARS = [
    SimPillar(400, 400),
    SimPillar(2600, 400),
    SimPillar(1500, 1600)
]

class StandaloneViewer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("Standalone LIDAR/IMU Viewer")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 20)
        self.running = True
        self.show_background = False
        self.show_enemy = True
        self.angle_speed = 0.25  # rad/s
        self.t0 = time.time()
        self.paused = False

        # Charger image de fond éventuelle
        self.bg_img = None
        if TABLE_BACKGROUND_IMAGE and os.path.isfile(TABLE_BACKGROUND_IMAGE):
            try:
                self.bg_img = pygame.image.load(TABLE_BACKGROUND_IMAGE).convert_alpha()
                self.show_background = True
            except Exception as e:
                print(f"Impossible de charger l'image de fond: {e}")

    def scale(self):
        return min((SCREEN_W - 2 * MARGIN_PX) / TABLE_W_MM,
                   (SCREEN_H - 2 * MARGIN_PX) / TABLE_H_MM)

    def w2s(self, x_mm: float, y_mm: float):
        s = self.scale()
        table_w_px = TABLE_W_MM * s
        table_h_px = TABLE_H_MM * s
        ox = (SCREEN_W - table_w_px) / 2.0
        oy = (SCREEN_H + table_h_px) / 2.0
        sx = ox + x_mm * s
        sy = oy - y_mm * s
        return int(sx), int(sy)

    def simulate(self, t: float):
        # Trajectoire IMU: forme en "8" centrée table
        cx, cy = TABLE_W_MM / 2, TABLE_H_MM / 2
        r1 = 600
        r2 = 300
        imu_x = cx + r1 * math.sin(t * 0.4)
        imu_y = cy + r2 * math.sin(t * 0.8)
        imu_angle = (t * self.angle_speed) % (2 * math.pi)

        # LIDAR légèrement décalé
        lidar_offset = 60  # mm
        lidar_x = imu_x + lidar_offset * math.cos(imu_angle + 0.3)
        lidar_y = imu_y + lidar_offset * math.sin(imu_angle + 0.3)
        lidar_angle = imu_angle + 0.05

        # Ennemi en orbite
        enemy_r = 500
        enemy_x = cx + enemy_r * math.cos(t * 0.3)
        enemy_y = cy + enemy_r * math.sin(t * 0.3)

        return {
            "imu": (imu_x, imu_y, imu_angle),
            "lidar": (lidar_x, lidar_y, lidar_angle),
            "enemy": (enemy_x, enemy_y),
        }

    def draw_grid(self):
        for gx in range(0, int(TABLE_W_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = self.w2s(gx, 0)
            x2, y2 = self.w2s(gx, TABLE_H_MM)
            pygame.draw.line(self.screen, GRID_COLOR, (x1, y1), (x2, y2), 1)
        for gy in range(0, int(TABLE_H_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = self.w2s(0, gy)
            x2, y2 = self.w2s(TABLE_W_MM, gy)
            pygame.draw.line(self.screen, GRID_COLOR, (x1, y1), (x2, y2), 1)

    def draw_background(self):
        if not self.show_background or self.bg_img is None:
            return
        s = self.scale()
        img_w = max(1, int(TABLE_W_MM * s))
        img_h = max(1, int(TABLE_H_MM * s))
        bg_scaled = pygame.transform.smoothscale(self.bg_img, (img_w, img_h))
        table_w_px = TABLE_W_MM * s
        table_h_px = TABLE_H_MM * s
        ox = (SCREEN_W - table_w_px) / 2.0
        oy = (SCREEN_H + table_h_px) / 2.0
        self.screen.blit(bg_scaled, (ox, oy - img_h))

    def draw(self, sim):
        imu_x, imu_y, imu_angle = sim["imu"]
        lidar_x, lidar_y, lidar_angle = sim["lidar"]
        enemy_x, enemy_y = sim["enemy"]

        # IMU
        imu_sx, imu_sy = self.w2s(imu_x, imu_y)
        pygame.draw.circle(self.screen, COLOR_IMU, (imu_sx, imu_sy), R_IMU)
        imu_len = int(80 * self.scale())
        ix2 = imu_sx + int(math.cos(imu_angle) * imu_len)
        iy2 = imu_sy - int(math.sin(imu_angle) * imu_len)
        pygame.draw.line(self.screen, COLOR_IMU, (imu_sx, imu_sy), (ix2, iy2), 2)

        # LIDAR
        lidar_sx, lidar_sy = self.w2s(lidar_x, lidar_y)
        pygame.draw.circle(self.screen, COLOR_LIDAR, (lidar_sx, lidar_sy), R_LIDAR)

        # Piliers
        for p in PILLARS:
            sx, sy = self.w2s(p.x, p.y)
            color = COLOR_TRACKED_FOUND if p.present else COLOR_TRACKED_MISSING
            pygame.draw.circle(self.screen, color, (sx, sy), R_TRACK)

        # Ennemi
        if self.show_enemy:
            ex, ey = self.w2s(enemy_x, enemy_y)
            pygame.draw.circle(self.screen, COLOR_ENEMY, (ex, ey), R_ENEMY)

    def draw_hud(self, sim, fps):
        imu_x, imu_y, imu_angle = sim["imu"]
        text_lines = [
            f"IMU x={imu_x:.0f} y={imu_y:.0f} ang={math.degrees(imu_angle):.1f}°",
            f"FPS={fps:.0f} | angle_speed={self.angle_speed:.2f} rad/s",
            f"Fond={'ON' if self.show_background else 'OFF'} | Ennemi={'ON' if self.show_enemy else 'OFF'}",
            "B: fond  P: pause  A/Z: vitesse rotation  E: ennemi  S: screenshot  ESC: quit"
        ]
        y = 10
        for line in text_lines:
            surf = self.font.render(line, True, COLOR_TEXT)
            self.screen.blit(surf, (10, y))
            y += 18

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_b:
                    self.show_background = not self.show_background
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                elif event.key == pygame.K_a:
                    self.angle_speed *= 1.2
                elif event.key == pygame.K_z:
                    self.angle_speed /= 1.2
                elif event.key == pygame.K_e:
                    self.show_enemy = not self.show_enemy
                elif event.key == pygame.K_s:
                    ts = int(time.time())
                    path = f"screenshot_{ts}.png"
                    pygame.image.save(self.screen, path)
                    print(f"Screenshot sauvegardé: {path}")

    def run(self):
        while self.running:
            self.handle_events()
            t = time.time() - self.t0
            if self.paused:
                sim = self.simulate(self.last_t if hasattr(self, 'last_t') else 0.0)
            else:
                sim = self.simulate(t)
                self.last_t = t

            self.screen.fill(COLOR_BG)
            self.draw_background()
            self.draw_grid()
            self.draw(sim)
            fps = self.clock.get_fps()
            self.draw_hud(sim, fps)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    StandaloneViewer().run()

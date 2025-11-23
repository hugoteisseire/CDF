#!/usr/bin/env python3
"""
Viewer standalone pour tester des algorithmes de pathfinding.

Contrôles:
  ESC         : quitter
  B           : toggle image de fond
  P           : pause/reprendre animation
  R           : toggle mode placement Robot
  T           : toggle mode placement Target (cible)
  A           : toggle mode placement Adversaire
  O           : toggle mode placement Obstacles
  G           : toggle affichage grille
  F           : toggle affichage zones interdites
  S           : screenshot (PNG)
  Clic gauche : placer élément selon mode actif
  Clic droit  : supprimer obstacle (mode O uniquement)
  SPACE       : démarrer/arrêter le pathfinding
  +/-         : ajuster rayon robot
  [/]         : ajuster rayon adversaire

Modes de placement:
  - Mode R (Robot)      : placer notre robot
  - Mode T (Target)     : placer la cible
  - Mode A (Adversaire) : placer le robot adverse
  - Mode O (Obstacles)  : ajouter/supprimer obstacles rectangulaires
"""

import math
import os
import time
import pygame
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

# Import de l'algorithme de pathfinding
from potential_field import compute_potential_field

# Dimensions de la table (mm)
TABLE_W_MM = 3000
TABLE_H_MM = 2000
MARGIN_PX = 40
GRID_STEP_MM = 500
PLAY_AREA_MARGIN_MM = 50  # Zone interdite près des bords

# Fenêtre (ratio adapté 3:2)
SCREEN_W, SCREEN_H = 1200, 800

# Image de fond optionnelle
TABLE_BACKGROUND_IMAGE = r"C:\Users\NBcra\Desktop\CDF\carte_control\simu_strat\table.png"

# Couleurs
COLOR_BG = (10, 10, 10)
GRID_COLOR = (35, 35, 35)
COLOR_ROBOT = (50, 150, 255)
COLOR_TARGET = (50, 255, 100)
COLOR_ENEMY = (250, 200, 0)
COLOR_OBSTACLE = (180, 60, 60)
COLOR_FORBIDDEN = (80, 40, 40)
COLOR_PATH = (100, 255, 200)
COLOR_TEXT = (210, 210, 210)
COLOR_TEXT_HIGHLIGHT = (255, 255, 100)
COLOR_DRAG_PREVIEW = (150, 150, 150)

# Paramètres robots
DEFAULT_ROBOT_RADIUS_MM = 150
DEFAULT_ENEMY_RADIUS_MM = 150
MIN_RADIUS_MM = 50
MAX_RADIUS_MM = 300


@dataclass
class Obstacle:
    """Obstacle rectangulaire."""
    x: float  # coin supérieur gauche en mm
    y: float
    width: float
    height: float

@dataclass
class Robot:
    """Notre robot."""
    x: float
    y: float
    angle: float = 0.0

@dataclass
class Target:
    """Position cible."""
    x: float
    y: float

@dataclass
class Enemy:
    """Robot adverse."""
    x: float
    y: float

class PlacementMode:
    """Modes de placement."""
    ROBOT = "R"
    TARGET = "T"
    ENEMY = "A"
    OBSTACLE = "O"

# Obstacles de base (zones fixes sur la table de la Coupe de France de Robotique)
DEFAULT_OBSTACLES = [
    # Zone centrale haute
    Obstacle(600, 1550,  1800,450)
    # Zone centrale droite

]

class PathfindingViewer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("Pathfinding Testbed")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 20)
        self.font_large = pygame.font.SysFont(None, 28)
        self.running = True
        
        # Options d'affichage
        self.show_background = False
        self.show_grid = True
        self.show_forbidden = True
        self.paused = False
        
        # Entités
        self.robot = Robot(500, 500)
        self.target = Target(2500, 1500)
        self.enemy = Enemy(1500, 1000)
        
        # Obstacles (copie de la liste par défaut)
        self.obstacles: List[Obstacle] = [
            Obstacle(obs.x, obs.y, obs.width, obs.height) 
            for obs in DEFAULT_OBSTACLES
        ]
        
        # Rayons
        self.robot_radius = DEFAULT_ROBOT_RADIUS_MM
        self.enemy_radius = DEFAULT_ENEMY_RADIUS_MM
        
        # Mode de placement
        self.placement_mode = PlacementMode.ROBOT
        
        # Drag & drop pour obstacles
        self.dragging_obstacle = False
        self.drag_start_pos: Optional[Tuple[float, float]] = None
        self.drag_current_pos: Optional[Tuple[float, float]] = None
        
        # Pathfinding
        self.path_running = False
        self.path_points: List[Tuple[float, float]] = []  # Historique pour visualisation
        self.robot_speed = 5.0  # mm par frame (ajustable)
        self.current_direction = np.array([0.0, 0.0])  # Direction instantanée
        
        # Image de fond
        self.bg_img = None
        if TABLE_BACKGROUND_IMAGE and os.path.isfile(TABLE_BACKGROUND_IMAGE):
            try:
                self.bg_img = pygame.image.load(TABLE_BACKGROUND_IMAGE).convert_alpha()
                self.show_background = True
            except Exception as e:
                print(f"Impossible de charger l'image de fond: {e}")

    def scale(self) -> float:
        """Calcule l'échelle mm -> pixels."""
        return min((SCREEN_W - 2 * MARGIN_PX) / TABLE_W_MM,
                   (SCREEN_H - 2 * MARGIN_PX) / TABLE_H_MM)

    def w2s(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Convertit coordonnées monde (mm) -> écran (pixels)."""
        s = self.scale()
        table_w_px = TABLE_W_MM * s
        table_h_px = TABLE_H_MM * s
        ox = (SCREEN_W - table_w_px) / 2.0
        oy = (SCREEN_H + table_h_px) / 2.0
        sx = ox + x_mm * s
        sy = oy - y_mm * s
        return int(sx), int(sy)
    
    def s2w(self, sx: int, sy: int) -> Tuple[float, float]:
        """Convertit coordonnées écran (pixels) -> monde (mm)."""
        s = self.scale()
        table_w_px = TABLE_W_MM * s
        table_h_px = TABLE_H_MM * s
        ox = (SCREEN_W - table_w_px) / 2.0
        oy = (SCREEN_H + table_h_px) / 2.0
        x_mm = (sx - ox) / s
        y_mm = (oy - sy) / s
        return x_mm, y_mm
    
    def clamp_to_table(self, x: float, y: float, margin: float = 0) -> Tuple[float, float]:
        """Limite les coordonnées à la table avec marge."""
        x = max(margin, min(TABLE_W_MM - margin, x))
        y = max(margin, min(TABLE_H_MM - margin, y))
        return x, y

    
    def draw_grid(self):
        """Dessine la grille."""
        if not self.show_grid:
            return
        for gx in range(0, int(TABLE_W_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = self.w2s(gx, 0)
            x2, y2 = self.w2s(gx, TABLE_H_MM)
            pygame.draw.line(self.screen, GRID_COLOR, (x1, y1), (x2, y2), 1)
        for gy in range(0, int(TABLE_H_MM) + GRID_STEP_MM, GRID_STEP_MM):
            x1, y1 = self.w2s(0, gy)
            x2, y2 = self.w2s(TABLE_W_MM, gy)
            pygame.draw.line(self.screen, GRID_COLOR, (x1, y1), (x2, y2), 1)

    def draw_background(self):
        """Dessine l'image de fond si activée."""
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
    
    def draw_forbidden_zones(self):
        """Dessine les zones interdites (bords de table)."""
        if not self.show_forbidden:
            return
        s = self.scale()
        margin = PLAY_AREA_MARGIN_MM
        
        # Surface semi-transparente
        surf = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
        
        # Haut
        p1 = self.w2s(0, TABLE_H_MM)
        p2 = self.w2s(TABLE_W_MM, TABLE_H_MM - margin)
        rect = pygame.Rect(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1])
        pygame.draw.rect(surf, (*COLOR_FORBIDDEN, 80), rect)
        
        # Bas
        p1 = self.w2s(0, margin)
        p2 = self.w2s(TABLE_W_MM, 0)
        rect = pygame.Rect(p1[0], p2[1], p2[0] - p1[0], p1[1] - p2[1])
        pygame.draw.rect(surf, (*COLOR_FORBIDDEN, 80), rect)
        
        # Gauche
        p1 = self.w2s(0, TABLE_H_MM)
        p2 = self.w2s(margin, 0)
        rect = pygame.Rect(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1])
        pygame.draw.rect(surf, (*COLOR_FORBIDDEN, 80), rect)
        
        # Droite
        p1 = self.w2s(TABLE_W_MM - margin, TABLE_H_MM)
        p2 = self.w2s(TABLE_W_MM, 0)
        rect = pygame.Rect(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1])
        pygame.draw.rect(surf, (*COLOR_FORBIDDEN, 80), rect)
        
        self.screen.blit(surf, (0, 0))
    
    def draw_obstacles(self):
        """Dessine les obstacles."""
        for obs in self.obstacles:
            p1 = self.w2s(obs.x, obs.y + obs.height)
            p2 = self.w2s(obs.x + obs.width, obs.y)
            rect = pygame.Rect(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1])
            pygame.draw.rect(self.screen, COLOR_OBSTACLE, rect)
            pygame.draw.rect(self.screen, (255, 100, 100), rect, 2)
    
    def draw_drag_preview(self):
        """Dessine l'aperçu du drag en cours."""
        if not self.dragging_obstacle or not self.drag_start_pos or not self.drag_current_pos:
            return
        
        x1, y1 = self.drag_start_pos
        x2, y2 = self.drag_current_pos
        
        sx1, sy1 = self.w2s(x1, y1)
        sx2, sy2 = self.w2s(x2, y2)
        
        rect = pygame.Rect(
            min(sx1, sx2), min(sy1, sy2),
            abs(sx2 - sx1), abs(sy2 - sy1)
        )
        pygame.draw.rect(self.screen, COLOR_DRAG_PREVIEW, rect, 2)
    
    def draw_entities(self):
        """Dessine robot, cible et adversaire."""
        s = self.scale()
        
        # Notre robot (plein)
        rx, ry = self.w2s(self.robot.x, self.robot.y)
        pygame.draw.circle(self.screen, COLOR_ROBOT, (rx, ry), int(self.robot_radius * s))
        # Direction
        dx = rx + int(math.cos(self.robot.angle) * self.robot_radius * s)
        dy = ry - int(math.sin(self.robot.angle) * self.robot_radius * s)
        pygame.draw.line(self.screen, (255, 255, 255), (rx, ry), (dx, dy), 3)
        
        # Cible
        tx, ty = self.w2s(self.target.x, self.target.y)
        pygame.draw.circle(self.screen, COLOR_TARGET, (tx, ty), 8)
        pygame.draw.circle(self.screen, COLOR_TARGET, (tx, ty), 12, 2)
        
        # Adversaire (plein)
        ex, ey = self.w2s(self.enemy.x, self.enemy.y)
        pygame.draw.circle(self.screen, COLOR_ENEMY, (ex, ey), int(self.enemy_radius * s))
    
    def draw_path(self):
        """Dessine la trajectoire parcourue (historique)."""
        if len(self.path_points) < 2:
            return
        
        points_screen = [self.w2s(x, y) for x, y in self.path_points]
        
        # Tracer l'historique de la trajectoire
        pygame.draw.lines(self.screen, COLOR_PATH, False, points_screen, 2)
        
        # Dessiner une flèche pour la direction actuelle
        if self.path_running and np.linalg.norm(self.current_direction) > 0:
            robot_screen = self.w2s(self.robot.x, self.robot.y)
            s = self.scale()
            arrow_length = 100 * s  # Longueur de la flèche
            
            arrow_end = (
                int(robot_screen[0] + self.current_direction[0] * arrow_length),
                int(robot_screen[1] - self.current_direction[1] * arrow_length)
            )
            
            # Flèche principale
            pygame.draw.line(self.screen, (100, 255, 100), robot_screen, arrow_end, 4)
            
            # Pointe de flèche
            angle = math.atan2(-self.current_direction[1], self.current_direction[0])
            arrow_size = 15
            left_angle = angle + 2.5
            right_angle = angle - 2.5
            
            left_point = (
                int(arrow_end[0] - arrow_size * math.cos(left_angle)),
                int(arrow_end[1] - arrow_size * math.sin(left_angle))
            )
            right_point = (
                int(arrow_end[0] - arrow_size * math.cos(right_angle)),
                int(arrow_end[1] - arrow_size * math.sin(right_angle))
            )
            
            pygame.draw.polygon(self.screen, (100, 255, 100), [arrow_end, left_point, right_point])
    
    def draw_hud(self):
        """Dessine l'interface utilisateur."""
        lines = [
            f"Mode: {self.placement_mode} | Robot R={self.robot_radius:.0f}mm | Ennemi R={self.enemy_radius:.0f}mm | Vitesse: {self.robot_speed:.1f}mm/frame",
            f"Robot: ({self.robot.x:.0f}, {self.robot.y:.0f}) | Cible: ({self.target.x:.0f}, {self.target.y:.0f})",
            f"Obstacles: {len(self.obstacles)} | Pathfinding: {'ON' if self.path_running else 'OFF'} | Direction: ({self.current_direction[0]:.2f}, {self.current_direction[1]:.2f})",
            f"Grille: {'ON' if self.show_grid else 'OFF'} | Zones: {'ON' if self.show_forbidden else 'OFF'} | Fond: {'ON' if self.show_background else 'OFF'}",
        ]
        
        y = 10
        for i, line in enumerate(lines):
            color = COLOR_TEXT_HIGHLIGHT if i == 0 else COLOR_TEXT
            surf = self.font.render(line, True, color)
            self.screen.blit(surf, (10, y))
            y += 22
        
        # Aide
        help_text = "R:robot T:target A:ennemi O:obstacles SPACE:path ↑↓:vitesse C:reset X:clear G:grille F:zones S:screenshot"
        surf = self.font.render(help_text, True, COLOR_TEXT)
        self.screen.blit(surf, (10, SCREEN_H - 25))


    def handle_events(self):
        """Gestion des événements clavier et souris."""
        mouse_pos = pygame.mouse.get_pos()
        world_pos = self.s2w(*mouse_pos)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                
            elif event.type == pygame.KEYDOWN:
                # Quitter
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                
                # Modes de placement
                elif event.key == pygame.K_r:
                    self.placement_mode = PlacementMode.ROBOT
                elif event.key == pygame.K_t:
                    self.placement_mode = PlacementMode.TARGET
                elif event.key == pygame.K_a:
                    self.placement_mode = PlacementMode.ENEMY
                elif event.key == pygame.K_o:
                    self.placement_mode = PlacementMode.OBSTACLE
                
                # Toggles affichage
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif event.key == pygame.K_f:
                    self.show_forbidden = not self.show_forbidden
                elif event.key == pygame.K_b:
                    self.show_background = not self.show_background
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                
                # Pathfinding
                elif event.key == pygame.K_SPACE:
                    self.path_running = not self.path_running
                    if self.path_running:
                        print("Pathfinding démarré - champs de potentiels")
                        self.path_points = [(self.robot.x, self.robot.y)]
                    else:
                        print("Pathfinding arrêté")
                
                # Ajuster vitesse du robot
                elif event.key == pygame.K_UP:
                    self.robot_speed = min(20.0, self.robot_speed + 1.0)
                    print(f"Vitesse robot: {self.robot_speed:.1f} mm/frame")
                elif event.key == pygame.K_DOWN:
                    self.robot_speed = max(1.0, self.robot_speed - 1.0)
                    print(f"Vitesse robot: {self.robot_speed:.1f} mm/frame")
                
                # Reset obstacles
                elif event.key == pygame.K_c:
                    self.obstacles = [
                        Obstacle(obs.x, obs.y, obs.width, obs.height) 
                        for obs in DEFAULT_OBSTACLES
                    ]
                    print("Obstacles réinitialisés aux valeurs par défaut")
                
                # Clear obstacles
                elif event.key == pygame.K_x:
                    self.obstacles = []
                    print("Tous les obstacles supprimés")
                
                # Ajustements rayons
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.robot_radius = min(MAX_RADIUS_MM, self.robot_radius + 10)
                elif event.key == pygame.K_MINUS:
                    self.robot_radius = max(MIN_RADIUS_MM, self.robot_radius - 10)
                elif event.key == pygame.K_LEFTBRACKET:
                    self.enemy_radius = max(MIN_RADIUS_MM, self.enemy_radius - 10)
                elif event.key == pygame.K_RIGHTBRACKET:
                    self.enemy_radius = min(MAX_RADIUS_MM, self.enemy_radius + 10)
                
                # Screenshot
                elif event.key == pygame.K_s:
                    ts = int(time.time())
                    path = f"pathfinding_screenshot_{ts}.png"
                    pygame.image.save(self.screen, path)
                    print(f"Screenshot sauvegardé: {path}")
            
            # Clic gauche
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.placement_mode == PlacementMode.OBSTACLE:
                    self.dragging_obstacle = True
                    self.drag_start_pos = world_pos
                    self.drag_current_pos = world_pos
                else:
                    self.place_entity(world_pos)
            
            # Clic droit (supprimer obstacle)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                if self.placement_mode == PlacementMode.OBSTACLE:
                    self.remove_obstacle_at(world_pos)
            
            # Mouvement souris pendant drag
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging_obstacle:
                    self.drag_current_pos = world_pos
            
            # Relâcher clic gauche
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if self.dragging_obstacle and self.drag_start_pos and self.drag_current_pos:
                    self.create_obstacle_from_drag()
                    self.dragging_obstacle = False
                    self.drag_start_pos = None
                    self.drag_current_pos = None
    
    def place_entity(self, pos: Tuple[float, float]):
        """Place une entité selon le mode actif."""
        x, y = self.clamp_to_table(*pos, margin=self.robot_radius if self.placement_mode == PlacementMode.ROBOT else 0)
        
        if self.placement_mode == PlacementMode.ROBOT:
            self.robot.x = x
            self.robot.y = y
        elif self.placement_mode == PlacementMode.TARGET:
            self.target.x = x
            self.target.y = y
        elif self.placement_mode == PlacementMode.ENEMY:
            self.enemy.x = x
            self.enemy.y = y
    
    def create_obstacle_from_drag(self):
        """Crée un obstacle rectangulaire à partir du drag."""
        if not self.drag_start_pos or not self.drag_current_pos:
            return
        
        x1, y1 = self.drag_start_pos
        x2, y2 = self.drag_current_pos
        
        x = min(x1, x2)
        y = min(y1, y2)
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        
        # Ignorer les obstacles trop petits
        if width > 20 and height > 20:
            self.obstacles.append(Obstacle(x, y, width, height))
    
    def remove_obstacle_at(self, pos: Tuple[float, float]):
        """Supprime l'obstacle sous la position donnée."""
        x, y = pos
        for i, obs in enumerate(self.obstacles):
            if (obs.x <= x <= obs.x + obs.width and 
                obs.y <= y <= obs.y + obs.height):
                del self.obstacles[i]
                break
    
    def compute_path(self):
        """
        Calcule la direction instantanée avec champs de potentiels simple.
        """
        try:
            # Convertir les obstacles
            from potential_field import Obstacle as PFObstacle
            pf_obstacles = [
                PFObstacle(obs.x, obs.y, obs.width, obs.height)
                for obs in self.obstacles
            ]
            
            # Calculer la direction instantanée
            robot_pos = np.array([self.robot.x, self.robot.y])
            target_pos = np.array([self.target.x, self.target.y])
            enemy_pos = np.array([self.enemy.x, self.enemy.y])
            
            self.current_direction = compute_potential_field(
                robot_pos=robot_pos,
                target_pos=target_pos,
                enemy_pos=enemy_pos,
                obstacles=pf_obstacles,
                table_width=TABLE_W_MM,
                table_height=TABLE_H_MM,
                robot_radius=self.robot_radius,
                enemy_radius=self.enemy_radius
            )
            
        except Exception as e:
            print(f"Erreur lors du calcul de direction: {e}")
            import traceback
            traceback.print_exc()
            # Fallback: direction directe vers la cible
            to_target = np.array([self.target.x - self.robot.x, self.target.y - self.robot.y])
            norm = np.linalg.norm(to_target)
            if norm > 0:
                self.current_direction = to_target / norm
            else:
                self.current_direction = np.array([0.0, 0.0])
    
    def update(self):
        """Mise à jour logique."""
        if self.path_running:
            # Calcul de direction instantané à chaque frame
            self.compute_path()
            
            # Vérifier si la cible est atteinte
            dist_to_target = np.linalg.norm(
                np.array([self.target.x - self.robot.x, self.target.y - self.robot.y])
            )
            
            if dist_to_target < 100:  # Seuil d'arrivée (mm)
                print("Cible atteinte !")
                self.path_running = False
                return
            
            # Déplacer le robot selon la direction calculée
            move_dist = min(self.robot_speed, dist_to_target)
            
            self.robot.x += self.current_direction[0] * move_dist
            self.robot.y += self.current_direction[1] * move_dist
            
            # Mettre à jour l'angle du robot
            if np.linalg.norm(self.current_direction) > 0:
                self.robot.angle = math.atan2(self.current_direction[1], self.current_direction[0])
            
            # Ajouter la position à l'historique pour visualisation
            self.path_points.append((self.robot.x, self.robot.y))
            if len(self.path_points) > 200:
                self.path_points.pop(0)
    
    def render(self):
        """Rendu complet."""
        self.screen.fill(COLOR_BG)
        self.draw_background()
        self.draw_grid()
        self.draw_forbidden_zones()
        self.draw_obstacles()
        self.draw_path()
        self.draw_entities()
        self.draw_drag_preview()
        self.draw_hud()
        pygame.display.flip()

    def run(self):
        """Boucle principale."""
        while self.running:
            self.handle_events()
            if not self.paused:
                self.update()
            self.render()
            self.clock.tick(60)
        
        pygame.quit()

if __name__ == "__main__":
    PathfindingViewer().run()


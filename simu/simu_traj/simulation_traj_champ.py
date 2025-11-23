"""
Simulateur de robot holonome avec évitement par champs de potentiels.
Utilise compute_direction_champ pour la navigation.

Contrôles:
- Clic gauche: Déplacer cible (si mode T activé) ou ennemi
- T: Basculer mode déplacement cible/ennemi
- S: Démarrer/arrêter le robot
- H: Téléporter le robot à la position de la souris
- ESC/Fermeture: Quitter
"""

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
from avoidance import (
    compute_direction_champ,
    clamp_position,
    obstacles_rect,
    normalize
)


# ============================================================================
# CONFIGURATION
# ============================================================================

class Config:
    """Configuration centralisée de la simulation."""
    
    # Dimensions de la table (mm)
    TABLE_WIDTH_MM = 3000
    TABLE_HEIGHT_MM = 2000
    
    # Dimensions fenêtre (pixels)
    DISPLAY_WIDTH = 1000
    DISPLAY_HEIGHT = 667
    
    # Échelle
    SCALE_X = DISPLAY_WIDTH / TABLE_WIDTH_MM
    SCALE_Y = DISPLAY_HEIGHT / TABLE_HEIGHT_MM
    SCALE = min(SCALE_X, SCALE_Y)
    
    # Robot
    ROBOT_RADIUS_MM = 150
    MAX_WHEEL_SPEED_MPS = 0.3
    WHEEL_ANGLES = [math.radians(90), math.radians(210), math.radians(330)]
    
    # Simulation
    TICKS_PER_SECOND = 60
    TARGET_RADIUS_MM = 20
    ENEMY_RADIUS_MM = 150
    ENEMY_MOVE_INTERVAL = 2
    ENEMY_SPEED_MM_PER_TICK = 5
    RANDOM_MARGIN_MM = 100
    
    # Affichage
    DISPLAY_ENABLED = True
    DEBUG_PERF = True
    FONT_SIZE = 24
    
    # Couleurs
    class Colors:
        BACKGROUND = (255, 255, 255)
        BORDER = (0, 0, 0)
        TABLE = (230, 230, 230)
        ROBOT = (0, 100, 255)
        ENEMY = (255, 0, 0)
        TARGET = (0, 0, 255)
        OBSTACLE = (100, 100, 100)
        WHEEL = (255, 0, 0)
        WHEEL_CENTER = (0, 0, 0)
        DIRECTION_ARROW = (0, 255, 0)
        TEXT = (0, 0, 0)


# ============================================================================
# UTILITAIRES
# ============================================================================

class Utils:
    """Fonctions utilitaires de conversion."""
    
    @staticmethod
    def to_screen(pos_mm):
        """Convertit mm → pixels."""
        return np.array([pos_mm[0] * Config.SCALE, pos_mm[1] * Config.SCALE])
    
    @staticmethod
    def from_screen(pos_px):
        """Convertit pixels → mm."""
        return np.array([pos_px[0] / Config.SCALE, pos_px[1] / Config.SCALE])
    
    @staticmethod
    def mm_per_tick_to_mps(value_mm_per_tick):
        """Convertit mm/tick → m/s."""
        return value_mm_per_tick * Config.TICKS_PER_SECOND / 1000.0
    
    @staticmethod
    def random_position(margin_mm=Config.RANDOM_MARGIN_MM):
        """Génère une position aléatoire dans la table."""
        return np.array([
            random.uniform(margin_mm, Config.TABLE_WIDTH_MM - margin_mm),
            random.uniform(margin_mm, Config.TABLE_HEIGHT_MM - margin_mm)
        ])


# ============================================================================
# SYSTÈME D'AFFICHAGE
# ============================================================================

class Renderer:
    """Système d'affichage unifié."""
    
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font
        self.to_screen = Utils.to_screen
    
    def render_all(self, state):
        """Rendu complet de la scène."""
        self._render_table()
        self._render_entities(state.target_pos, state.enemy_pos)
        self._render_robot(state.robot, state.direction)
        self._render_ui(state)
    
    def _render_table(self):
        """Dessine la table et les obstacles."""
        # Fond
        self.screen.fill(Config.Colors.BACKGROUND)
        
        # Bordure
        pygame.draw.rect(
            self.screen,
            Config.Colors.BORDER,
            (0, 0, Config.DISPLAY_WIDTH, Config.DISPLAY_HEIGHT),
            5
        )
        
        # Surface de jeu
        margin = int(50 * Config.SCALE)
        pygame.draw.rect(
            self.screen,
            Config.Colors.TABLE,
            (margin, margin,
             Config.DISPLAY_WIDTH - 2 * margin,
             Config.DISPLAY_HEIGHT - 2 * margin)
        )
        
        # Obstacles
        for rect in obstacles_rect:
            obstacle_rect = pygame.Rect(
                self.to_screen(np.array([rect.left, rect.top])),
                self.to_screen(np.array([rect.width, rect.height]))
            )
            pygame.draw.rect(self.screen, Config.Colors.OBSTACLE, obstacle_rect)
    
    def _render_entities(self, target_pos, enemy_pos):
        """Dessine la cible et l'ennemi."""
        # Ennemi
        pygame.draw.circle(
            self.screen,
            Config.Colors.ENEMY,
            self.to_screen(enemy_pos).astype(int),
            int(Config.ENEMY_RADIUS_MM * Config.SCALE)
        )
        
        # Cible
        pygame.draw.circle(
            self.screen,
            Config.Colors.TARGET,
            self.to_screen(target_pos).astype(int),
            int(Config.TARGET_RADIUS_MM * Config.SCALE)
        )
    
    def _render_robot(self, robot, direction):
        """Dessine le robot complet (corps + roues + direction)."""
        # Corps
        pygame.draw.circle(
            self.screen,
            Config.Colors.ROBOT,
            self.to_screen(robot.pos).astype(int),
            int(robot.radius * Config.SCALE)
        )
        
        # Roues
        for i, theta in enumerate(robot.wheel_angles):
            wheel_dir = np.array([math.cos(theta), math.sin(theta)])
            tangent_dir = np.array([-math.sin(theta), math.cos(theta)])
            
            wheel_pos = robot.pos + rotate_vector(wheel_dir, robot.angle) * robot.radius
            tangent_rotated = rotate_vector(tangent_dir, robot.angle)
            tip = wheel_pos + tangent_rotated * robot.wheel_speeds[i] * 40
            
            pygame.draw.line(self.screen, Config.Colors.WHEEL,
                           self.to_screen(wheel_pos), self.to_screen(tip), 3)
            pygame.draw.circle(self.screen, Config.Colors.WHEEL_CENTER,
                             self.to_screen(wheel_pos).astype(int), 5)
        
        # Vecteur vitesse globale
        pygame.draw.line(
            self.screen,
            Config.Colors.ROBOT,
            self.to_screen(robot.pos),
            self.to_screen(robot.pos + robot.base_velocity_global * 150),
            4
        )
        
        # Flèche de direction du champ
        if np.linalg.norm(direction) > 0:
            arrow_length = 200
            pygame.draw.line(
                self.screen,
                Config.Colors.DIRECTION_ARROW,
                self.to_screen(robot.pos),
                self.to_screen(robot.pos + direction * arrow_length),
                5
            )
    
    def _render_ui(self, state):
        """Dessine l'interface utilisateur (vitesses)."""
        self._render_speeds(state.robot)
    
    def _render_speeds(self, robot):
        """Affiche les vitesses."""
        lines = []
        
        # Vitesses des roues
        for i, s in enumerate(robot.wheel_speeds):
            s_mps = Utils.mm_per_tick_to_mps(s)
            lines.append(f"Roue {i+1}: {s_mps:.2f} m/s")
        
        # Vitesse globale
        vx = Utils.mm_per_tick_to_mps(robot.base_velocity_global[0])
        vy = Utils.mm_per_tick_to_mps(robot.base_velocity_global[1])
        lines.append(f"Vitesse X: {vx:.2f} m/s")
        lines.append(f"Vitesse Y: {vy:.2f} m/s")
        lines.append(f"Norme: {math.hypot(vx, vy):.2f} m/s")
        
        # Affichage
        x, y = Config.DISPLAY_WIDTH - 220, 10
        for line in lines:
            text = self.font.render(line, True, Config.Colors.TEXT)
            self.screen.blit(text, (x, y))
            y += 20


# ============================================================================
# ÉTAT DE LA SIMULATION
# ============================================================================

class SimulationState:
    """Contient tout l'état de la simulation."""
    
    def __init__(self):
        # Robot
        self.robot = Robot(
            pos=np.array([1000.0, 1000.0]),
            angle=0.0,
            radius=Config.ROBOT_RADIUS_MM,
            wheel_angles=Config.WHEEL_ANGLES
        )
        self.robot.wheel_speeds = [0.0, 0.0, 0.0]
        self.robot.base_velocity = np.array([0.0, 0.0])
        self.robot.base_velocity_global = np.array([0.0, 0.0])
        
        # Entités
        self.target_pos = np.array([2500.0, 1000.0])
        self.enemy_pos = np.array([1500.0, 1000.0])
        
        # Ennemi
        self.enemy_move_timer = 0
        self.enemy_velocity = np.array([0.0, 0.0])
        
        # Contrôle
        self.running = True
        self.move_target = False
        self.robot_moving = False
        
        # Direction du champ
        self.direction = np.array([0.0, 0.0])


# ============================================================================
# CONTRÔLEUR PRINCIPAL
# ============================================================================

class Simulator:
    """Contrôleur principal de la simulation."""
    
    def __init__(self):
        pygame.init()
        
        # Pygame
        self.screen = pygame.display.set_mode((Config.DISPLAY_WIDTH, Config.DISPLAY_HEIGHT)) if Config.DISPLAY_ENABLED else None
        if Config.DISPLAY_ENABLED:
            pygame.display.set_caption("Simulateur Robot - Champs de potentiels")
        
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, Config.FONT_SIZE)
        
        # État
        self.state = SimulationState()
        
        # Renderer
        self.renderer = Renderer(self.screen, self.font) if Config.DISPLAY_ENABLED else None
    
    def run(self):
        """Boucle principale."""
        while self.state.running:
            start_time = time.perf_counter()
            
            self._handle_events()
            self._handle_mouse()
            self._update_enemy()
            self._update_robot()
            
            if Config.DISPLAY_ENABLED:
                self.renderer.render_all(self.state)
                pygame.display.flip()
            
            if Config.DEBUG_PERF:
                self._print_debug(time.perf_counter() - start_time)
            
            self.clock.tick(Config.TICKS_PER_SECOND)
        
        pygame.quit()
        sys.exit()
    
    def _handle_events(self):
        """Gestion des événements clavier."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.state.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    self.state.move_target = not self.state.move_target
                elif event.key == pygame.K_s:
                    self.state.robot_moving = not self.state.robot_moving
                elif event.key == pygame.K_h:
                    self.state.robot.pos = Utils.from_screen(pygame.mouse.get_pos())
                elif event.key == pygame.K_ESCAPE:
                    self.state.running = False
    
    def _handle_mouse(self):
        """Gestion de la souris."""
        if pygame.mouse.get_pressed()[0]:
            mouse_pos_mm = Utils.from_screen(pygame.mouse.get_pos())
            if self.state.move_target:
                self.state.target_pos = clamp_position(
                    mouse_pos_mm,
                    Config.TARGET_RADIUS_MM,
                    Config.TABLE_WIDTH_MM,
                    Config.TABLE_HEIGHT_MM
                )
            else:
                self.state.enemy_pos = clamp_position(
                    mouse_pos_mm,
                    Config.ENEMY_RADIUS_MM,
                    Config.TABLE_WIDTH_MM,
                    Config.TABLE_HEIGHT_MM
                )
    
    def _update_enemy(self):
        """Mise à jour de l'ennemi."""
        self.state.enemy_move_timer += 1 / Config.TICKS_PER_SECOND
        if self.state.enemy_move_timer > Config.ENEMY_MOVE_INTERVAL:
            self.state.enemy_move_timer = 0
            self.state.enemy_velocity = (Utils.random_position() - self.state.enemy_pos)
            self.state.enemy_velocity /= np.linalg.norm(self.state.enemy_velocity) + 1e-5
            self.state.enemy_velocity *= Config.ENEMY_SPEED_MM_PER_TICK
        
        # Mouvement commenté pour mode statique
        # self.state.enemy_pos += self.state.enemy_velocity
        self.state.enemy_pos = clamp_position(
            self.state.enemy_pos,
            Config.ENEMY_RADIUS_MM,
            Config.TABLE_WIDTH_MM,
            Config.TABLE_HEIGHT_MM
        )
    
    def _update_robot(self):
        """Mise à jour du robot."""
        if not self.state.robot_moving:
            return
        
        # Calcul de la direction via champs de potentiels
        self.state.direction = compute_direction_champ(
            self.state.robot,
            self.state.target_pos,
            self.state.enemy_pos,
            Config.TABLE_WIDTH_MM,
            Config.TABLE_HEIGHT_MM
        )
        
        # Vitesses
        raw_speeds = compute_wheel_speeds_global(
            self.state.robot,
            self.state.direction[0],
            self.state.direction[1],
            0.0
        )
        max_speed_mm_tick = (Config.MAX_WHEEL_SPEED_MPS * 1000) / Config.TICKS_PER_SECOND
        self.state.robot.wheel_speeds = [s * max_speed_mm_tick for s in raw_speeds]
        
        # Position
        self.state.robot.base_velocity = compute_base_velocity(
            self.state.robot,
            self.state.robot.wheel_speeds
        )
        self.state.robot.base_velocity_global = rotate_vector(
            self.state.robot.base_velocity,
            self.state.robot.angle
        )
        self.state.robot.pos += self.state.robot.base_velocity_global
        self.state.robot.angle += compute_rotation_velocity(
            self.state.robot,
            self.state.robot.wheel_speeds
        )
        
        # Clamping
        self.state.robot.pos = clamp_position(
            self.state.robot.pos,
            self.state.robot.radius,
            Config.TABLE_WIDTH_MM,
            Config.TABLE_HEIGHT_MM
        )
        
        # Cible atteinte
        dist = np.linalg.norm(self.state.robot.pos - self.state.target_pos)
        if dist < Config.TARGET_RADIUS_MM + self.state.robot.radius:
            self.state.target_pos = Utils.random_position()
    
    def _print_debug(self, elapsed):
        """Affiche les infos de debug."""
        print(f"Frame: {elapsed * 1000:.0f} ms | Direction: {self.state.direction}")


# ============================================================================
# POINT D'ENTRÉE
# ============================================================================

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()

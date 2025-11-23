"""
Simulateur de robot holonome avec évitement d'obstacles.
Utilise A* pour la planification de trajectoire.

Contrôles:
- Clic gauche: Déplacer cible (si mode T activé) ou ennemi
- T: Basculer mode déplacement cible/ennemi
- S: Démarrer/arrêter le robot
- H: Téléporter le robot à la position de la souris
- G: Activer/désactiver les lignes de grille A*
- O: Activer/désactiver l'affichage de la grille d'occupation
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
    compute_direction_astar,
    clamp_position,
    obstacles_rect,
    cell_to_pos,
    CELL_SIZE,
    normalize,
    is_position_in_obstacle
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
    PATH_RECOMPUTE_INTERVAL = 15  # Augmenté pour moins recalculer
    RANDOM_MARGIN_MM = 100
    
    # Affichage
    DISPLAY_ENABLED = True
    DEBUG_PERF = True
    FONT_SIZE = 24
    SHOW_GRID_LINES = False  # Désactiver les lignes de grille par défaut (gain de performance)
    SHOW_OCCUPANCY_GRID = True  # Afficher les cellules occupées
    
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
        PATH = (0, 255, 255)
        PATH_NODE = (0, 200, 255)
        PATH_END = (0, 255, 0)
        ERROR_BOX = (255, 0, 0)
        ERROR_TEXT = (255, 255, 255)
        GRID_LINE = (150, 150, 150)
        OCCUPANCY = (255, 0, 0)
        TEXT = (0, 0, 0)
    
    # Codes d'état
    PATH_CODE_OBSTACLE = 0
    PATH_CODE_ENEMY = 1
    PATH_CODE_CLEAR = 2
    PATH_CODE_UNREACHABLE = 4


# ============================================================================
# UTILITAIRES
# ============================================================================

class Utils:
    """Fonctions utilitaires de conversion."""
    
    @staticmethod
    def to_screen(pos_mm):
        """Convertit mm → pixels. (0,0) en bas à gauche, y+ vers le haut."""
        return np.array([pos_mm[0] * Config.SCALE, Config.DISPLAY_HEIGHT - pos_mm[1] * Config.SCALE])
    
    @staticmethod
    def from_screen(pos_px):
        """Convertit pixels → mm. (0,0) en bas à gauche, y+ vers le haut."""
        return np.array([pos_px[0] / Config.SCALE, (Config.DISPLAY_HEIGHT - pos_px[1]) / Config.SCALE])
    
    @staticmethod
    def mm_per_tick_to_mps(value_mm_per_tick):
        """Convertit mm/tick → m/s."""
        return value_mm_per_tick * Config.TICKS_PER_SECOND / 1000.0
    
    @staticmethod
    def random_position(margin_mm=Config.RANDOM_MARGIN_MM):
        """Génère une position aléatoire dans la table (évite les obstacles)."""
        max_attempts = 100
        for _ in range(max_attempts):
            pos = np.array([
                random.uniform(margin_mm, Config.TABLE_WIDTH_MM - margin_mm),
                random.uniform(margin_mm, Config.TABLE_HEIGHT_MM - margin_mm)
            ])
            # Vérifier que la position n'est pas dans un obstacle
            if not is_position_in_obstacle(pos, Config.TARGET_RADIUS_MM + 50, obstacles_rect):
                return pos
        # Si échec après max_attempts, retourner position centrale
        return np.array([Config.TABLE_WIDTH_MM / 2, Config.TABLE_HEIGHT_MM / 2])


# ============================================================================
# GESTION DES ÉTATS
# ============================================================================

class PathState:
    """Gestion de l'état du pathfinding."""
    
    @staticmethod
    def handle_code(code):
        """
        Gère l'état selon le code de pathfinding.
        
        Returns:
            tuple: (pause_robot, robot_moving, error_message, should_recompute)
        """
        if code == Config.PATH_CODE_OBSTACLE:
            return True, False, "Cible dans un obstacle", True
        
        elif code == Config.PATH_CODE_ENEMY:
            return False, True, None, True
        
        elif code == Config.PATH_CODE_CLEAR:
            return False, True, None, False
        
        elif code == Config.PATH_CODE_UNREACHABLE:
            return True, False, "Cible inaccessible (bloqué)", True
        
        else:
            return True, False, "Erreur inconnue", False


class Navigation:
    """Gestion de la navigation du robot."""
    
    @staticmethod
    def update_path_following(robot, path, current_index):
        """
        Met à jour la direction pour suivre le chemin.
        
        Returns:
            tuple: (direction, new_index, reached_end)
        """
        if not path or current_index >= len(path):
            return np.array([0.0, 0.0]), current_index, True
        
        target_cell = path[current_index]
        target_pos_mm = cell_to_pos(target_cell)
        
        # Vérifier si la cellule est atteinte
        dist_to_target = np.linalg.norm(robot.pos - target_pos_mm)
        if dist_to_target < robot.radius:
            current_index += 1
            if current_index >= len(path):
                return np.array([0.0, 0.0]), current_index, True
            target_cell = path[current_index]
            target_pos_mm = cell_to_pos(target_cell)
        
        direction = normalize(target_pos_mm - robot.pos)
        return direction, current_index, False


# ============================================================================
# SYSTÈME D'AFFICHAGE
# ============================================================================

class Renderer:
    """Système d'affichage unifié."""
    
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font
        self.to_screen = Utils.to_screen
        
        # Cache pour la grille d'occupation
        self.cached_occupancy_surface = None
        self.cached_grid_hash = None
    
    def render_all(self, state):
        """Rendu complet de la scène."""
        self._render_table()
        self._render_entities(state.target_pos, state.enemy_pos)
        self._render_robot(state.robot)
        self._render_path(state.path_astar)
        self._render_grid(state.grid)
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
            # Position bas-gauche de l'obstacle en mm
            bottom_left_mm = np.array([rect.left, rect.bottom])
            # Convertir en coordonnées écran
            bottom_left_screen = self.to_screen(bottom_left_mm)
            
            obstacle_rect = pygame.Rect(
                int(bottom_left_screen[0]),
                int(bottom_left_screen[1]),
                int(rect.width * Config.SCALE),
                int(rect.height * Config.SCALE)
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
    
    def _render_robot(self, robot):
        """Dessine le robot complet (corps + roues + vitesse)."""
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
    
    def _render_path(self, path):
        """Dessine le chemin calculé."""
        if not path or len(path) < 2:
            return
        
        # Segments
        for i in range(len(path) - 1):
            start_px = cell_to_pos(path[i])
            end_px = cell_to_pos(path[i + 1])
            pygame.draw.line(self.screen, Config.Colors.PATH,
                           self.to_screen(start_px), self.to_screen(end_px), 3)
            pygame.draw.circle(self.screen, Config.Colors.PATH_NODE,
                             self.to_screen(start_px).astype(int), 5)
        
        # Point final
        pygame.draw.circle(
            self.screen,
            Config.Colors.PATH_END,
            self.to_screen(cell_to_pos(path[-1])).astype(int),
            6
        )
    
    def _render_grid(self, grid):
        """Dessine la grille d'occupation avec cache."""
        if grid is None:
            return
        
        grid_width, grid_height = len(grid), len(grid[0])
        cell_size_px = int(CELL_SIZE * Config.SCALE)
        
        # Grille d'occupation (transparente) - AVEC CACHE
        if Config.SHOW_OCCUPANCY_GRID:
            # Calculer un hash simple de la grille
            grid_hash = hash(tuple(tuple(row) for row in grid))
            
            # Si la grille a changé, recréer la surface
            if self.cached_grid_hash != grid_hash or self.cached_occupancy_surface is None:
                self.cached_occupancy_surface = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
                self.cached_occupancy_surface.fill((0, 0, 0, 0))  # Transparent
                
                for x in range(grid_width):
                    for y in range(grid_height):
                        if grid[x][y] == 1:
                            # Position bas-gauche de la cellule en mm
                            cell_bottom_left_mm = np.array([x * CELL_SIZE, y * CELL_SIZE])
                            screen_pos = self.to_screen(cell_bottom_left_mm)
                            
                            rect_px = pygame.Rect(
                                int(screen_pos[0]),
                                int(screen_pos[1]),
                                cell_size_px,
                                cell_size_px
                            )
                            pygame.draw.rect(self.cached_occupancy_surface, (*Config.Colors.OCCUPANCY, 80), rect_px)
                
                self.cached_grid_hash = grid_hash
            
            # Utiliser la surface en cache (TRÈS rapide)
            self.screen.blit(self.cached_occupancy_surface, (0, 0))
        
        # Lignes de grille A* - VERSION OPTIMISÉE (optionnel, coûteux)
        if Config.SHOW_GRID_LINES:
            # Lignes verticales
            for x in range(grid_width + 1):
                start_mm = np.array([x * CELL_SIZE, 0])
                end_mm = np.array([x * CELL_SIZE, grid_height * CELL_SIZE])
                start_px = self.to_screen(start_mm)
                end_px = self.to_screen(end_mm)
                pygame.draw.line(self.screen, Config.Colors.GRID_LINE, start_px, end_px, 1)
            
            # Lignes horizontales
            for y in range(grid_height + 1):
                start_mm = np.array([0, y * CELL_SIZE])
                end_mm = np.array([grid_width * CELL_SIZE, y * CELL_SIZE])
                start_px = self.to_screen(start_mm)
                end_px = self.to_screen(end_mm)
                pygame.draw.line(self.screen, Config.Colors.GRID_LINE, start_px, end_px, 1)
    
    def _render_ui(self, state):
        """Dessine l'interface utilisateur (vitesses + erreurs)."""
        # Vitesses
        self._render_speeds(state.robot)
        
        # Message d'erreur
        if state.pause_robot and state.error_message:
            self._render_error(state.error_message)
    
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
    
    def _render_error(self, error_message):
        """Affiche un message d'erreur."""
        # Boîte
        box_width, box_height = 400, 60
        box_x = Config.DISPLAY_WIDTH // 2 - box_width // 2
        box_y = Config.DISPLAY_HEIGHT // 2 - box_height // 2
        pygame.draw.rect(self.screen, Config.Colors.ERROR_BOX,
                        (box_x, box_y, box_width, box_height))
        
        # Texte
        text = self.font.render(f"PAUSE: {error_message}", True, Config.Colors.ERROR_TEXT)
        text_x = Config.DISPLAY_WIDTH // 2 - text.get_width() // 2
        text_y = Config.DISPLAY_HEIGHT // 2 - 10
        self.screen.blit(text, (text_x, text_y))


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
        self.pause_robot = False
        
        # Pathfinding
        self.path_computed = False
        self.path_astar = []
        self.current_path_index = 1
        self.recompute_counter = 0
        self.grid = None
        self.code = Config.PATH_CODE_CLEAR
        self.error_message = None
        
        # Détection de blocage
        self.stuck_counter = 0
        self.last_position = np.array([1000.0, 1000.0])


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
            pygame.display.set_caption("Simulateur Robot - Évitement d'obstacle")
        
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, Config.FONT_SIZE)
        
        # État
        self.state = SimulationState()
        
        # Renderer
        self.renderer = Renderer(self.screen, self.font) if Config.DISPLAY_ENABLED else None
    
    def run(self):
        """Boucle principale."""
        while self.state.running:
            self._handle_events()
            self._handle_mouse()
            
            # Mesure le temps des algorithmes
            start_algo = time.perf_counter()
            self._update_enemy()
            self._update_pathfinding()
            self._update_robot()
            algo_time = time.perf_counter() - start_algo
            
            # Mesure le temps d'affichage
            render_time = 0.0
            if Config.DISPLAY_ENABLED:
                start_render = time.perf_counter()
                self.renderer.render_all(self.state)
                pygame.display.flip()
                render_time = time.perf_counter() - start_render
            
            if Config.DEBUG_PERF:
                self._print_debug(algo_time, render_time)
            
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
                elif event.key == pygame.K_g:
                    # Toggle affichage grille
                    Config.SHOW_GRID_LINES = not Config.SHOW_GRID_LINES
                elif event.key == pygame.K_o:
                    # Toggle affichage occupation
                    Config.SHOW_OCCUPANCY_GRID = not Config.SHOW_OCCUPANCY_GRID
    
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
        
        # Mouvement de l'ennemi
        self.state.enemy_pos += self.state.enemy_velocity
        self.state.enemy_pos = clamp_position(
            self.state.enemy_pos,
            Config.ENEMY_RADIUS_MM,
            Config.TABLE_WIDTH_MM,
            Config.TABLE_HEIGHT_MM
        )
    
    def _update_pathfinding(self):
        """Mise à jour du pathfinding."""
        # Calcul périodique
        if not self.state.path_computed:
            direction, path, code, grid = compute_direction_astar(
                self.state.robot,
                self.state.target_pos,
                obstacles_rect,
                Config.TABLE_WIDTH_MM,
                Config.TABLE_HEIGHT_MM,
                self.state.enemy_pos
            )
            self.state.path_astar = path
            self.state.code = code
            self.state.grid = grid
            self.state.path_computed = True
            self.state.recompute_counter = 0
            self.state.current_path_index = 1
        else:
            self.state.recompute_counter += 1
            if self.state.recompute_counter > Config.PATH_RECOMPUTE_INTERVAL:
                self.state.path_computed = False
        
        # Gestion de l'état
        pause, moving, error, recompute = PathState.handle_code(self.state.code)
        self.state.pause_robot = pause
        self.state.robot_moving = moving
        self.state.error_message = error
        
        if recompute:
            self.state.path_computed = False
    
    def _update_robot(self):
        """Mise à jour du robot."""
        if not self.state.robot_moving:
            # Arrêter toutes les roues si le robot ne bouge pas
            self.state.robot.wheel_speeds = [0.0, 0.0, 0.0]
            self.state.robot.base_velocity = np.array([0.0, 0.0])
            self.state.robot.base_velocity_global = np.array([0.0, 0.0])
            # Réinitialiser l'historique de lissage
            if hasattr(self.state.robot, 'wheel_speeds_history'):
                self.state.robot.wheel_speeds_history = [[], [], []]
            self.state.stuck_counter = 0
            return
        
        # Détection de blocage: vérifier si le robot bouge vraiment
        movement = np.linalg.norm(self.state.robot.pos - self.state.last_position)
        if movement < 1.0 and self.state.robot_moving:  # Moins de 1mm de mouvement
            self.state.stuck_counter += 1
        else:
            self.state.stuck_counter = 0
        self.state.last_position = self.state.robot.pos.copy()
        
        # Si bloqué depuis trop longtemps (0.5s = 30 frames à 60fps)
        if self.state.stuck_counter > 30:
            # Téléporter légèrement le robot dans une direction aléatoire
            escape_angle = random.uniform(0, 2 * math.pi)
            escape_distance = 100  # 100mm
            escape_offset = np.array([
                math.cos(escape_angle) * escape_distance,
                math.sin(escape_angle) * escape_distance
            ])
            self.state.robot.pos += escape_offset
            self.state.robot.pos = clamp_position(
                self.state.robot.pos,
                self.state.robot.radius,
                Config.TABLE_WIDTH_MM,
                Config.TABLE_HEIGHT_MM
            )
            self.state.stuck_counter = 0
            self.state.path_computed = False
            self.state.error_message = "Déblocage automatique"
            return
        
        # Vérifier la distance avec l'adversaire
        dist_to_enemy = np.linalg.norm(self.state.robot.pos - self.state.enemy_pos)
        safety_distance = self.state.robot.radius + Config.ENEMY_RADIUS_MM + 50  # Marge de sécurité de 50mm
        
        if dist_to_enemy < safety_distance:
            # Trop proche de l'adversaire, arrêter le robot
            self.state.robot.wheel_speeds = [0.0, 0.0, 0.0]
            self.state.robot.base_velocity = np.array([0.0, 0.0])
            self.state.robot.base_velocity_global = np.array([0.0, 0.0])
            self.state.pause_robot = True
            self.state.error_message = "Trop proche de l'adversaire"
            return
        
        # Direction
        direction, index, reached = Navigation.update_path_following(
            self.state.robot,
            self.state.path_astar,
            self.state.current_path_index
        )
        self.state.current_path_index = index
        
        if reached:
            self.state.robot_moving = False
            return
        
        # Vitesses
        raw_speeds = compute_wheel_speeds_global(
            self.state.robot,
            direction[0],
            direction[1],
            0.0
        )
        max_speed_mm_tick = (Config.MAX_WHEEL_SPEED_MPS * 1000) / Config.TICKS_PER_SECOND
        new_speeds = [s * max_speed_mm_tick for s in raw_speeds] if not self.state.pause_robot else [0, 0, 0]
        
        # Lissage des vitesses sur les 5 dernières valeurs
        if not hasattr(self.state.robot, 'wheel_speeds_history'):
            self.state.robot.wheel_speeds_history = [[], [], []]
        
        for i in range(3):
            self.state.robot.wheel_speeds_history[i].append(new_speeds[i])
            if len(self.state.robot.wheel_speeds_history[i]) > 5:
                self.state.robot.wheel_speeds_history[i].pop(0)
            # Moyenne des 5 dernières valeurs
            self.state.robot.wheel_speeds[i] = np.mean(self.state.robot.wheel_speeds_history[i])
        
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
        
        # Cible atteinte
        dist = np.linalg.norm(self.state.robot.pos - self.state.target_pos)
        if dist < Config.TARGET_RADIUS_MM + self.state.robot.radius:
            self.state.target_pos = Utils.random_position()
            self.state.path_computed = False
    
    def _print_debug(self, algo_time, render_time):
        """Affiche les infos de debug."""
        total = algo_time + render_time
        print(f"Algo: {algo_time * 1000:.2f} ms | Render: {render_time * 1000:.2f} ms | Total: {total * 1000:.2f} ms | Code: {self.state.code}")


# ============================================================================
# POINT D'ENTRÉE
# ============================================================================

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()

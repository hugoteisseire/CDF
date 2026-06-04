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
    is_position_in_obstacle,
    compute_navigation_direction,
    reset_pathfinding_state
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
        # SÉCURITÉ DÉSACTIVÉE: Cible peut être dans obstacle
        if code == Config.PATH_CODE_OBSTACLE:
            return False, True, "Cible dans un obstacle (tentative d'approche)", True
        
        if code == Config.PATH_CODE_ENEMY:
            return False, True, None, True
        
        elif code == Config.PATH_CODE_CLEAR:
            return False, True, None, False
        
        elif code == Config.PATH_CODE_UNREACHABLE:
            return False, True, "Cible inaccessible (tentative d'approche)", True
        
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
        
        # Pathfinding (simplifié - géré par compute_navigation_direction)
        self.path_astar = []
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

            # ===== PORTAGE ROBOT RÉEL =====
            
            # 1. SIMULATION: Clavier/souris → ROBOT RÉEL: Boutons physiques/stratégie préprogrammée
            self._handle_events()  # → Remplacer par gestion boutons (démarrage match, arrêt urgence)
            self._handle_mouse()   # → Supprimer (pas de souris sur robot)
            
            # Mesure le temps des algorithmes
            start_algo = time.perf_counter()
            
            # 2. LOCALISATION: SIMULATION: Position parfaite → ROBOT RÉEL: Odométrie + IMU + LiDAR
            #    AVANT _update_pathfinding(), ajouter:
            #    - robot.pos, robot.angle = localization.update(wheel_speeds, dt)
            #    - Fusion IMU (angle) + odométrie roues (position)
            
            # 3. PERCEPTION: SIMULATION: Ennemi fictif → ROBOT RÉEL: Détection LiDAR + clustering
            self._update_enemy()  # → Remplacer par: enemy_pos = lidar_handler.detect_enemy(robot.pos, robot.angle)
                                  #    + obstacles_dynamic = lidar_handler.get_obstacles_mm(...)
            
            # 4. PATHFINDING: ✅ GARDER TEL QUEL (core algorithme)
            self._update_pathfinding()  # ✅ Conserver ! Recalcule A* avec obstacles réels
                                        #    Modifier: passer obstacles_dynamic au lieu de obstacles_rect fixes
            
            # 5. CONTRÔLE: SIMULATION: Calcul position → ROBOT RÉEL: Commandes moteurs CAN
            self._update_robot()  # → Diviser en 2 parties:
                                  #    GARDER: Calcul direction + vitesses roues (cinématique)
                                  #    REMPLACER: robot.pos += velocity PAR motor_controller.set_wheel_speeds(speeds)
                                  #    SUPPRIMER: Simulation physique (position calculée par odométrie à l'étape 2)

            algo_time = time.perf_counter() - start_algo
            
            # 6. AFFICHAGE: SIMULATION: Pygame → ROBOT RÉEL: Optionnel (debug sur écran embarqué)
            render_time = 0.0
            if Config.DISPLAY_ENABLED:
                start_render = time.perf_counter()
                self.renderer.render_all(self.state)  # → Optionnel: garder pour debug sur Raspberry Pi
                pygame.display.flip()                 # → Optionnel: écran HDMI embarqué
                render_time = time.perf_counter() - start_render
            
            # 7. DEBUG: ✅ GARDER (utile pour tuning)
            if Config.DEBUG_PERF:
                self._print_debug(algo_time, render_time)  # → Optionnel: logger vers fichier ou console
            
            # 8. TIMING: ✅ GARDER ABSOLUMENT (boucle 60 Hz critique)
            self.clock.tick(Config.TICKS_PER_SECOND)  # ✅ ESSENTIEL ! Maintient 60 fps (16.67ms/loop)
        
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
        """Mise à jour du pathfinding avec la nouvelle fonction compute_navigation_direction."""
        # Appel de la fonction autonome de navigation
        nav_result = compute_navigation_direction(
            robot=self.state.robot,
            target_pos=self.state.target_pos,
            enemy_pos=self.state.enemy_pos,
            table_width_mm=Config.TABLE_WIDTH_MM,
            table_height_mm=Config.TABLE_HEIGHT_MM,
            path_recompute_interval=Config.PATH_RECOMPUTE_INTERVAL,
            force_recompute=False
        )
        
        # Mise à jour de l'état de la simulation avec les résultats
        self.state.path_astar = nav_result['path']
        self.state.code = nav_result['code']
        self.state.grid = nav_result['grid']
        self.state.pause_robot = nav_result['should_pause']
        self.state.error_message = nav_result['error_message']
        
        # Si la cible est atteinte, arrêter le mouvement
        if nav_result['reached']:
            self.state.robot_moving = False
    
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
        
        # Direction (calculée par compute_navigation_direction)
        # On récupère la direction directement depuis le pathfinding
        nav_result = compute_navigation_direction(
            robot=self.state.robot,
            target_pos=self.state.target_pos,
            enemy_pos=self.state.enemy_pos,
            table_width_mm=Config.TABLE_WIDTH_MM,
            table_height_mm=Config.TABLE_HEIGHT_MM,
            path_recompute_interval=Config.PATH_RECOMPUTE_INTERVAL,
            force_recompute=False
        )
        
        direction = nav_result['direction']
        
        if nav_result['reached']:
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
        
        # ===== SIMULATION PHYSIQUE (À SUPPRIMER SUR ROBOT RÉEL) =====
        # Sur robot réel: position vient de l'odométrie (étape 2 de run())
        # Ces lignes simulent ce que feraient les moteurs physiques
        
        # Calcul vitesse base à partir des vitesses roues (odométrie inverse)
        self.state.robot.base_velocity = compute_base_velocity(
            self.state.robot,
            self.state.robot.wheel_speeds
        )
        # Rotation vitesse locale → globale
        self.state.robot.base_velocity_global = rotate_vector(
            self.state.robot.base_velocity,
            self.state.robot.angle
        )
        # Intégration position (SIMULATION SEULEMENT)
        self.state.robot.pos += self.state.robot.base_velocity_global  # ← SUPPRIMER sur robot réel
        # Intégration angle (SIMULATION SEULEMENT)
        self.state.robot.angle += compute_rotation_velocity(           # ← SUPPRIMER sur robot réel
            self.state.robot,
            self.state.robot.wheel_speeds
        )
        
        # Cible atteinte
        dist = np.linalg.norm(self.state.robot.pos - self.state.target_pos)
        if dist < Config.TARGET_RADIUS_MM + self.state.robot.radius:
            self.state.target_pos = Utils.random_position()
            reset_pathfinding_state()  # Réinitialiser l'état du pathfinding
    
    def _update_robot_real(self):
        """
        Mise à jour du robot - VERSION ROBOT RÉEL.
        Cette fonction calcule les commandes moteurs et les envoie au contrôleur CAN.
        """
        # ===== 1. ARRÊT DU ROBOT =====
        if not self.state.robot_moving:
            # Arrêter toutes les roues
            self.state.robot.wheel_speeds = [0.0, 0.0, 0.0]
            self.state.robot.base_velocity = np.array([0.0, 0.0])
            self.state.robot.base_velocity_global = np.array([0.0, 0.0])
            
            # TODO: INTERFAÇAGE MOTEURS - Arrêt d'urgence
            # motor_controller.emergency_stop()
            # OU motor_controller.set_wheel_speeds([0, 0, 0])
            
            # Réinitialiser l'historique de lissage
            if hasattr(self.state.robot, 'wheel_speeds_history'):
                self.state.robot.wheel_speeds_history = [[], [], []]
            self.state.stuck_counter = 0
            return
        
        # ===== 2. DÉTECTION DE BLOCAGE =====
        # Vérifier si le robot bouge vraiment (détection deadzone)
        movement = np.linalg.norm(self.state.robot.pos - self.state.last_position)
        if movement < 1.0 and self.state.robot_moving:  # Moins de 1mm de mouvement
            self.state.stuck_counter += 1
        else:
            self.state.stuck_counter = 0
        self.state.last_position = self.state.robot.pos.copy()
        
        # Si bloqué trop longtemps (0.5s = 30 frames @ 60fps)
        if self.state.stuck_counter > 30:
            # TODO: INTERFAÇAGE RECALAGE - Déblocage automatique
            # Option 1: Mouvement arrière puis rotation
            # motor_controller.backward(distance_mm=100, duration_ms=500)
            # motor_controller.rotate(angle_deg=random.randint(-45, 45))
            
            # Option 2: Recalage position avec balises/LiDAR
            # robot.pos = localization.recalibrate_position()
            
            # Pour l'instant: marquer comme nécessitant recalcul
            self.state.stuck_counter = 0
            self.state.path_computed = False
            self.state.error_message = "Déblocage automatique"
            
            # TODO: INTERFAÇAGE MOTEURS - Arrêt temporaire
            # motor_controller.set_wheel_speeds([0, 0, 0])
            return
        
        # ===== 3. SÉCURITÉ DISTANCE ADVERSAIRE =====
        dist_to_enemy = np.linalg.norm(self.state.robot.pos - self.state.enemy_pos)
        safety_distance = self.state.robot.radius + Config.ENEMY_RADIUS_MM + 50  # 350mm
        
        if dist_to_enemy < safety_distance:
            # Trop proche de l'adversaire → ARRÊT IMMÉDIAT
            self.state.robot.wheel_speeds = [0.0, 0.0, 0.0]
            self.state.robot.base_velocity = np.array([0.0, 0.0])
            self.state.robot.base_velocity_global = np.array([0.0, 0.0])
            self.state.pause_robot = True
            self.state.error_message = "Trop proche de l'adversaire"
            
            # TODO: INTERFAÇAGE MOTEURS - Arrêt sécurité
            # motor_controller.emergency_stop()
            return
        
        # ===== 4. CALCUL DIRECTION (SUIVI DE CHEMIN A*) =====
        nav_result = compute_navigation_direction(
            robot=self.state.robot,
            target_pos=self.state.target_pos,
            enemy_pos=self.state.enemy_pos,
            table_width_mm=Config.TABLE_WIDTH_MM,
            table_height_mm=Config.TABLE_HEIGHT_MM,
            path_recompute_interval=Config.PATH_RECOMPUTE_INTERVAL,
            force_recompute=False
        )
        
        direction = nav_result['direction']
        
        if nav_result['reached']:
            # Chemin terminé, arrêter le robot
            self.state.robot_moving = False
            
            # TODO: INTERFAÇAGE MOTEURS - Arrêt fin de trajectoire
            # motor_controller.set_wheel_speeds([0, 0, 0])
            return
        
        # ===== 5. CINÉMATIQUE: DIRECTION → VITESSES ROUES =====
        # Calcul vitesses normalisées [-1, 1]
        raw_speeds = compute_wheel_speeds_global(
            self.state.robot,
            direction[0],  # vx
            direction[1],  # vy
            0.0            # omega (pas de rotation sur place)
        )
        
        # Conversion en mm/tick puis m/s
        max_speed_mm_tick = (Config.MAX_WHEEL_SPEED_MPS * 1000) / Config.TICKS_PER_SECOND
        new_speeds = [s * max_speed_mm_tick for s in raw_speeds] if not self.state.pause_robot else [0, 0, 0]
        
        # ===== 6. LISSAGE VITESSES (ANTI-OSCILLATIONS) =====
        if not hasattr(self.state.robot, 'wheel_speeds_history'):
            self.state.robot.wheel_speeds_history = [[], [], []]
        
        for i in range(3):
            self.state.robot.wheel_speeds_history[i].append(new_speeds[i])
            if len(self.state.robot.wheel_speeds_history[i]) > 5:
                self.state.robot.wheel_speeds_history[i].pop(0)
            # Moyenne mobile sur 5 frames (~83ms)
            self.state.robot.wheel_speeds[i] = np.mean(self.state.robot.wheel_speeds_history[i])
        
        # ===== 7. ENVOI COMMANDES MOTEURS =====
        # TODO: INTERFAÇAGE MOTEURS - Commandes CAN/UART
        # Conversion mm/tick → RPM ou autre unité moteur
        # 
        # Exemple conversion vers RPM:
        # wheel_radius_mm = 30  # Rayon de vos roues
        # for i in range(3):
        #     speed_mm_s = self.state.robot.wheel_speeds[i] * Config.TICKS_PER_SECOND
        #     speed_rad_s = speed_mm_s / wheel_radius_mm
        #     rpm = (speed_rad_s * 60) / (2 * math.pi)
        #     motor_controller.set_motor_rpm(motor_id=i, rpm=rpm)
        #
        # OU directement si votre contrôleur accepte des vitesses normalisées:
        # motor_controller.set_wheel_speeds_normalized([
        #     raw_speeds[0],  # Roue 1 (90°)
        #     raw_speeds[1],  # Roue 2 (210°)
        #     raw_speeds[2]   # Roue 3 (330°)
        # ])
        
        # ===== 8. VÉRIFICATION OBJECTIF ATTEINT =====
        dist_to_target = np.linalg.norm(self.state.robot.pos - self.state.target_pos)
        if dist_to_target < Config.TARGET_RADIUS_MM + self.state.robot.radius:
            # Objectif atteint
            
            # TODO: INTERFAÇAGE STRATÉGIE - Nouvel objectif
            # self.state.target_pos = strategy.get_next_objective()
            # OU pour test:
            # self.state.target_pos = Utils.random_position()
            
            reset_pathfinding_state()  # Réinitialiser l'état du pathfinding
    
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

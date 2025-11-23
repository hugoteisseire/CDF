"""
Algorithme de pathfinding hybride optimisé pour la Coupe de France de Robotique.

Architecture en 3 couches:
1. Planification globale (A*) - 1 Hz
2. Navigation locale (Champs de potentiels) - 20-100 Hz
3. Sécurité (Zones interdites) - 100+ Hz

Avec lissage avancé pour éviter les oscillations moteur.
"""

import numpy as np
import heapq
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import deque

@dataclass
class Obstacle:
    """Obstacle rectangulaire."""
    x: float
    y: float
    width: float
    height: float

# ============================================================================
# PARAMÈTRES CONFIGURABLES
# ============================================================================

# Champs de potentiels
K_ATTRACT = 0.8              # Force d'attraction vers waypoint
K_REPULSE_ROBOT = 200        # Répulsion de l'ennemi
K_REPULSE_OBSTACLE = 800     # Répulsion des obstacles
K_TANGENT = 2.5              # Force tangentielle (contournement)
REPULSE_RADIUS_ROBOT = 900   # Rayon d'influence ennemi (mm)
REPULSE_RADIUS_OBSTACLE = 200 # Rayon d'influence obstacle (mm)

# Grille A*
GRID_CELL_SIZE = 100  # mm (grille grossière pour vitesse)

# Lissage
DIRECTION_SMOOTHING_WINDOW = 7  # Fenêtre de lissage direction (frames)
SPEED_SMOOTHING_ALPHA = 0.3     # Facteur de lissage vitesse (0=lent, 1=réactif)
MAX_DIRECTION_CHANGE = 0.5      # Changement max de direction par frame (radians)

# Détection de blocage
STUCK_THRESHOLD = 50.0          # Distance parcourue minimale en 2s (mm)
STUCK_CHECK_WINDOW = 120        # Frames pour vérifier blocage (2s à 60 Hz)
OSCILLATION_THRESHOLD = 0.3     # Variance max pour détecter oscillation


# ============================================================================
# UTILITAIRES
# ============================================================================

def normalize(v: np.ndarray) -> np.ndarray:
    """Normalise un vecteur."""
    norm = np.linalg.norm(v)
    if norm < 1e-6:
        return v
    return v / norm


def closest_point_on_rect(pos: np.ndarray, obstacle: Obstacle) -> np.ndarray:
    """Trouve le point le plus proche sur un rectangle."""
    closest_x = max(obstacle.x, min(pos[0], obstacle.x + obstacle.width))
    closest_y = max(obstacle.y, min(pos[1], obstacle.y + obstacle.height))
    return np.array([closest_x, closest_y])


def angle_diff(a1: float, a2: float) -> float:
    """Calcule la différence entre deux angles (en radians)."""
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


# ============================================================================
# COUCHE 1: PLANIFICATION GLOBALE (A*)
# ============================================================================

class GlobalPlanner:
    """Planificateur global utilisant A* sur grille grossière."""
    
    def __init__(self, table_width: float, table_height: float, cell_size: float = GRID_CELL_SIZE):
        self.table_width = table_width
        self.table_height = table_height
        self.cell_size = cell_size
        self.grid_w = int(table_width / cell_size)
        self.grid_h = int(table_height / cell_size)
        self.grid = None
        
    def create_grid(self, obstacles: List[Obstacle], robot_radius: float, 
                    forbidden_margin: float = 50.0):
        """Crée la grille d'occupation."""
        self.grid = [[0 for _ in range(self.grid_h)] for _ in range(self.grid_w)]
        
        margin_cells = math.ceil(robot_radius / self.cell_size)
        
        # Marquer les obstacles
        for obs in obstacles:
            left = int((obs.x - robot_radius) / self.cell_size)
            right = int((obs.x + obs.width + robot_radius) / self.cell_size)
            top = int((obs.y - robot_radius) / self.cell_size)
            bottom = int((obs.y + obs.height + robot_radius) / self.cell_size)
            
            for x in range(max(0, left), min(self.grid_w, right + 1)):
                for y in range(max(0, top), min(self.grid_h, bottom + 1)):
                    self.grid[x][y] = 1
        
        # Marquer les bords interdits
        margin_cells_border = int(forbidden_margin / self.cell_size)
        for x in range(self.grid_w):
            for y in range(margin_cells_border):
                self.grid[x][y] = 1
                self.grid[x][self.grid_h - 1 - y] = 1
        for y in range(self.grid_h):
            for x in range(margin_cells_border):
                self.grid[x][y] = 1
                self.grid[self.grid_w - 1 - x][y] = 1
    
    def pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convertit position mm -> cellule grille."""
        cx = int(x / self.cell_size)
        cy = int(y / self.cell_size)
        return (max(0, min(self.grid_w - 1, cx)), max(0, min(self.grid_h - 1, cy)))
    
    def cell_to_pos(self, cx: int, cy: int) -> Tuple[float, float]:
        """Convertit cellule grille -> position mm (centre)."""
        return ((cx + 0.5) * self.cell_size, (cy + 0.5) * self.cell_size)
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heuristique distance euclidienne."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def astar(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """A* sur la grille."""
        if self.grid is None:
            return [start, goal]
        
        start_cell = self.pos_to_cell(*start)
        goal_cell = self.pos_to_cell(*goal)
        
        # Vérifier que départ et arrivée sont libres
        if self.grid[start_cell[0]][start_cell[1]] == 1 or \
           self.grid[goal_cell[0]][goal_cell[1]] == 1:
            return [start, goal]  # Fallback
        
        open_set = []
        heapq.heappush(open_set, (0, start_cell))
        came_from = {}
        g_score = {start_cell: 0}
        
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_cell:
                # Reconstruire le chemin
                path_cells = [current]
                while current in came_from:
                    current = came_from[current]
                    path_cells.append(current)
                path_cells.reverse()
                
                # Convertir en waypoints (simplifier le chemin)
                waypoints = [start]
                for i in range(1, len(path_cells) - 1, max(1, len(path_cells) // 5)):
                    waypoints.append(self.cell_to_pos(*path_cells[i]))
                waypoints.append(goal)
                return waypoints
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if 0 <= neighbor[0] < self.grid_w and 0 <= neighbor[1] < self.grid_h:
                    if self.grid[neighbor[0]][neighbor[1]] == 0:
                        move_cost = math.sqrt(dx*dx + dy*dy)
                        tentative_g = g_score[current] + move_cost
                        
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            g_score[neighbor] = tentative_g
                            f = tentative_g + self.heuristic(neighbor, goal_cell)
                            heapq.heappush(open_set, (f, neighbor))
                            came_from[neighbor] = current
        
        # Pas de chemin trouvé
        return [start, goal]


# ============================================================================
# COUCHE 2: NAVIGATION LOCALE (CHAMPS DE POTENTIELS)
# ============================================================================

def compute_local_direction(
    robot_pos: np.ndarray,
    waypoint: np.ndarray,
    enemy_pos: np.ndarray,
    obstacles: List[Obstacle],
    table_width: float,
    table_height: float,
    robot_radius: float,
    enemy_radius: float
) -> np.ndarray:
    """Calcule la direction locale vers le waypoint avec évitement."""
    
    direction = np.array([0.0, 0.0])
    
    # === 1. ATTRACTION VERS WAYPOINT ===
    to_waypoint = waypoint - robot_pos
    dist_waypoint = np.linalg.norm(to_waypoint)
    
    if dist_waypoint > 1e-6:
        direction += normalize(to_waypoint) * K_ATTRACT
    
    # === 2. RÉPULSION DE L'ENNEMI (FORTE) ===
    to_enemy = robot_pos - enemy_pos
    dist_enemy = np.linalg.norm(to_enemy)
    safe_distance = robot_radius + enemy_radius + REPULSE_RADIUS_ROBOT
    
    if dist_enemy < safe_distance and dist_enemy > 1e-6:
        # Répulsion très forte proche de l'ennemi
        repulse_strength = K_REPULSE_ROBOT / (dist_enemy ** 1.2)
        direction += normalize(to_enemy) * repulse_strength
    
    # === 3. RÉPULSION DES OBSTACLES ===
    for obs in obstacles:
        closest_point = closest_point_on_rect(robot_pos, obs)
        to_obstacle = robot_pos - closest_point
        dist = np.linalg.norm(to_obstacle)
        
        influence_dist = robot_radius + REPULSE_RADIUS_OBSTACLE
        
        if dist < influence_dist and dist > 1e-6:
            repulse_strength = K_REPULSE_OBSTACLE / (dist ** 1.5)
            direction += normalize(to_obstacle) * repulse_strength
            
            # === 4. FORCE TANGENTIELLE ===
            to_waypoint_norm = normalize(to_waypoint)
            to_closest = closest_point - robot_pos
            
            if np.linalg.norm(to_closest) > 1e-6:
                alignment = np.dot(normalize(to_closest), to_waypoint_norm)
                
                if alignment > 0.4:
                    tangent1 = normalize(np.array([-to_obstacle[1], to_obstacle[0]]))
                    tangent2 = -tangent1
                    
                    score1 = np.dot(tangent1, to_waypoint_norm)
                    score2 = np.dot(tangent2, to_waypoint_norm)
                    
                    chosen_tangent = tangent1 if score1 > score2 else tangent2
                    direction += chosen_tangent * K_TANGENT
    
    # === 5. RÉPULSION DES BORDS (TRÈS FORTE) ===
    margin = 50.0
    wall_force = 50000.0
    
    if robot_pos[0] < margin:
        direction[0] += (margin - robot_pos[0]) * wall_force
    elif robot_pos[0] > table_width - margin:
        direction[0] -= (robot_pos[0] - (table_width - margin)) * wall_force
    
    if robot_pos[1] < margin:
        direction[1] += (margin - robot_pos[1]) * wall_force
    elif robot_pos[1] > table_height - margin:
        direction[1] -= (robot_pos[1] - (table_height - margin)) * wall_force
    
    return normalize(direction)


# ============================================================================
# COUCHE 3: LISSAGE ET CONTRÔLE
# ============================================================================

class SmoothController:
    """Contrôleur avec lissage pour éviter les oscillations."""
    
    def __init__(self, smoothing_window: int = DIRECTION_SMOOTHING_WINDOW):
        self.direction_history = deque(maxlen=smoothing_window)
        self.velocity_history = deque(maxlen=20)
        self.position_history = deque(maxlen=STUCK_CHECK_WINDOW)
        
        self.current_speed = 0.0
        self.previous_angle = 0.0
        self.stuck_counter = 0
        self.is_stuck = False
    
    def smooth_direction(self, raw_direction: np.ndarray) -> np.ndarray:
        """Lisse la direction avec filtre médian + moyenne pondérée."""
        self.direction_history.append(raw_direction.copy())
        
        if len(self.direction_history) < 3:
            return raw_direction
        
        # Filtre médian sur les composantes
        history_array = np.array(list(self.direction_history))
        median_dir = np.median(history_array, axis=0)
        
        # Moyenne pondérée (plus de poids sur les directions récentes)
        weights = np.linspace(0.5, 1.0, len(self.direction_history))
        weights /= weights.sum()
        weighted_dir = np.average(history_array, axis=0, weights=weights)
        
        # Combiner médiane (50%) + moyenne pondérée (50%)
        smoothed = 0.5 * median_dir + 0.5 * weighted_dir
        
        return normalize(smoothed)
    
    def limit_direction_change(self, new_direction: np.ndarray) -> np.ndarray:
        """Limite le changement de direction pour éviter les à-coups."""
        if np.linalg.norm(new_direction) < 1e-6:
            return new_direction
        
        new_angle = math.atan2(new_direction[1], new_direction[0])
        angle_change = angle_diff(self.previous_angle, new_angle)
        
        # Limiter le changement d'angle
        if abs(angle_change) > MAX_DIRECTION_CHANGE:
            limited_angle = self.previous_angle + np.sign(angle_change) * MAX_DIRECTION_CHANGE
            limited_direction = np.array([math.cos(limited_angle), math.sin(limited_angle)])
            self.previous_angle = limited_angle
            return limited_direction
        
        self.previous_angle = new_angle
        return new_direction
    
    def smooth_speed(self, target_speed: float) -> float:
        """Lisse la vitesse avec filtre exponentiel."""
        self.current_speed = (SPEED_SMOOTHING_ALPHA * target_speed + 
                             (1 - SPEED_SMOOTHING_ALPHA) * self.current_speed)
        return self.current_speed
    
    def detect_stuck(self, robot_pos: np.ndarray) -> bool:
        """Détecte si le robot est bloqué (oscillation ou immobile)."""
        self.position_history.append(robot_pos.copy())
        
        if len(self.position_history) < STUCK_CHECK_WINDOW:
            return False
        
        # Calculer la distance totale parcourue
        positions = np.array(list(self.position_history))
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        total_distance = np.sum(distances)
        
        # Vérifier oscillation
        variance = np.var(positions, axis=0)
        is_oscillating = np.all(variance < OSCILLATION_THRESHOLD)
        
        # Bloqué si peu de mouvement ou oscillation
        if total_distance < STUCK_THRESHOLD or is_oscillating:
            self.stuck_counter += 1
            if self.stuck_counter > 30:  # 0.5s à 60 Hz
                self.is_stuck = True
                return True
        else:
            self.stuck_counter = 0
            self.is_stuck = False
        
        return False
    
    def get_unstuck_direction(self, robot_pos: np.ndarray, 
                             obstacles: List[Obstacle]) -> Optional[np.ndarray]:
        """Génère une direction pour se débloquer."""
        if not self.is_stuck:
            return None
        
        # Trouver l'obstacle le plus proche
        min_dist = float('inf')
        closest_obs = None
        
        for obs in obstacles:
            closest_point = closest_point_on_rect(robot_pos, obs)
            dist = np.linalg.norm(robot_pos - closest_point)
            if dist < min_dist:
                min_dist = dist
                closest_obs = obs
        
        if closest_obs is None:
            # Pas d'obstacle proche, aller dans une direction aléatoire
            angle = np.random.uniform(0, 2 * math.pi)
            return np.array([math.cos(angle), math.sin(angle)])
        
        # Longer l'obstacle (direction tangentielle)
        closest_point = closest_point_on_rect(robot_pos, closest_obs)
        to_obstacle = robot_pos - closest_point
        tangent = normalize(np.array([-to_obstacle[1], to_obstacle[0]]))
        
        return tangent
    
    def reset_stuck(self):
        """Réinitialise la détection de blocage."""
        self.stuck_counter = 0
        self.is_stuck = False


# ============================================================================
# SYSTÈME HYBRIDE COMPLET
# ============================================================================

class HybridPathfinder:
    """Système de pathfinding hybride complet."""
    
    def __init__(self, table_width: float, table_height: float):
        self.global_planner = GlobalPlanner(table_width, table_height)
        self.smooth_controller = SmoothController()
        
        self.waypoints = []
        self.current_waypoint_index = 0
        self.replan_counter = 0
        self.table_width = table_width
        self.table_height = table_height
    
    def initialize(self, obstacles: List[Obstacle], robot_radius: float):
        """Initialise la grille de planification."""
        self.global_planner.create_grid(obstacles, robot_radius)
    
    def compute_direction(
        self,
        robot_pos: Tuple[float, float],
        target_pos: Tuple[float, float],
        enemy_pos: Tuple[float, float],
        obstacles: List[Obstacle],
        robot_radius: float,
        enemy_radius: float,
        max_speed: float
    ) -> Tuple[np.ndarray, float]:
        """
        Calcule la direction et vitesse optimales.
        
        Returns:
            (direction_lissée, vitesse_lissée)
        """
        robot_np = np.array(robot_pos)
        target_np = np.array(target_pos)
        enemy_np = np.array(enemy_pos)
        
        # === REPLANIFICATION GLOBALE (1 Hz = toutes les 60 frames) ===
        self.replan_counter += 1
        if self.replan_counter >= 60 or len(self.waypoints) == 0:
            self.replan_counter = 0
            self.waypoints = self.global_planner.astar(robot_pos, target_pos)
            self.current_waypoint_index = 0
        
        # === SÉLECTION DU WAYPOINT ACTUEL ===
        if self.current_waypoint_index >= len(self.waypoints):
            current_waypoint = target_np
        else:
            current_waypoint = np.array(self.waypoints[self.current_waypoint_index])
            
            # Passer au waypoint suivant si proche
            dist_to_waypoint = np.linalg.norm(robot_np - current_waypoint)
            if dist_to_waypoint < 150:  # 150mm
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    current_waypoint = np.array(self.waypoints[self.current_waypoint_index])
                else:
                    current_waypoint = target_np
        
        # === DÉTECTION DE BLOCAGE ===
        is_stuck = self.smooth_controller.detect_stuck(robot_np)
        
        if is_stuck:
            # Mode déblocage
            unstuck_dir = self.smooth_controller.get_unstuck_direction(robot_np, obstacles)
            if unstuck_dir is not None:
                raw_direction = unstuck_dir
            else:
                raw_direction = compute_local_direction(
                    robot_np, current_waypoint, enemy_np, obstacles,
                    self.table_width, self.table_height, robot_radius, enemy_radius
                )
            
            # Vérifier si déblocage réussi
            if self.replan_counter % 30 == 0:
                self.smooth_controller.reset_stuck()
        else:
            # === CALCUL DE DIRECTION LOCALE ===
            raw_direction = compute_local_direction(
                robot_np, current_waypoint, enemy_np, obstacles,
                self.table_width, self.table_height, robot_radius, enemy_radius
            )
        
        # === LISSAGE DE LA DIRECTION ===
        smoothed_direction = self.smooth_controller.smooth_direction(raw_direction)
        limited_direction = self.smooth_controller.limit_direction_change(smoothed_direction)
        
        # === LISSAGE DE LA VITESSE ===
        dist_to_target = np.linalg.norm(robot_np - target_np)
        
        # Ralentir près de la cible
        if dist_to_target < 300:
            target_speed = max_speed * (dist_to_target / 300)
        else:
            target_speed = max_speed
        
        smoothed_speed = self.smooth_controller.smooth_speed(target_speed)
        
        return limited_direction, smoothed_speed
    
    def get_waypoints(self) -> List[Tuple[float, float]]:
        """Retourne les waypoints pour visualisation."""
        return self.waypoints

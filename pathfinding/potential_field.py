"""
Algorithme de pathfinding par champs de potentiels (Potential Fields).

Principe:
- Force d'attraction vers la cible
- Forces de répulsion depuis les obstacles et l'ennemi
- Forces tangentielles pour contourner les obstacles
- Répulsion des bords de la table
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class Obstacle:
    """Obstacle rectangulaire."""
    x: float
    y: float
    width: float
    height: float

# Paramètres des champs de force
K_ATTRACT = 0.5              # Force d'attraction vers la cible
K_REPULSE_ROBOT = 160        # Répulsion de l'ennemi
K_REPULSE_OBSTACLE = 600     # Répulsion des obstacles
K_TANGENT = 2.0              # Force tangentielle (contournement)
REPULSE_RADIUS_ROBOT = 800   # Rayon d'influence ennemi (mm)
REPULSE_RADIUS_OBSTACLE = 180 # Rayon d'influence obstacle (mm)
MARGIN_FROM_EDGE = 25        # Distance minimale des bords (mm)


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


def compute_potential_field(
    robot_pos: np.ndarray,
    target_pos: np.ndarray,
    enemy_pos: np.ndarray,
    obstacles: List[Obstacle],
    table_width: float,
    table_height: float,
    robot_radius: float = 150,
    enemy_radius: float = 150
) -> np.ndarray:
    """
    Calcule la direction du champ de potentiels.
    
    Args:
        robot_pos: Position actuelle du robot (x, y) en mm
        target_pos: Position cible (x, y) en mm
        enemy_pos: Position de l'ennemi (x, y) en mm
        obstacles: Liste des obstacles rectangulaires
        table_width: Largeur de la table en mm
        table_height: Hauteur de la table en mm
        robot_radius: Rayon du robot en mm
        enemy_radius: Rayon de l'ennemi en mm
    
    Returns:
        Vecteur direction normalisé (dx, dy)
    """
    
    direction = np.array([0.0, 0.0])
    
    # === 1. ATTRACTION VERS LA CIBLE ===
    to_target = target_pos - robot_pos
    direction += normalize(to_target) * K_ATTRACT
    
    # === 2. RÉPULSION DE L'ENNEMI ===
    to_enemy = robot_pos - enemy_pos
    dist_enemy = np.linalg.norm(to_enemy)
    
    # Zone de sécurité = rayons combinés
    safe_distance = robot_radius + enemy_radius + REPULSE_RADIUS_ROBOT
    
    if dist_enemy < safe_distance and dist_enemy > 1e-6:
        repulse_force = normalize(to_enemy) * (K_REPULSE_ROBOT / dist_enemy)
        direction += repulse_force
    
    # === 3. RÉPULSION DES OBSTACLES ===
    for obs in obstacles:
        # Trouver le point le plus proche sur l'obstacle
        closest_point = closest_point_on_rect(robot_pos, obs)
        to_obstacle = robot_pos - closest_point
        dist = np.linalg.norm(to_obstacle)
        
        # Zone d'influence
        influence_dist = robot_radius + REPULSE_RADIUS_OBSTACLE
        
        if dist < influence_dist and dist > 1e-6:
            # Force de répulsion inversement proportionnelle à la distance
            repulse_force = normalize(to_obstacle) * (K_REPULSE_OBSTACLE / (dist ** 1.5))
            direction += repulse_force
            
            # === 4. FORCE TANGENTIELLE (CONTOURNEMENT) ===
            # Si l'obstacle est entre le robot et la cible
            to_target_norm = normalize(to_target)
            to_closest = closest_point - robot_pos
            
            if np.linalg.norm(to_closest) > 1e-6:
                # L'obstacle bloque si aligné avec la direction cible
                alignment = np.dot(normalize(to_closest), to_target_norm)
                
                if alignment > 0.5:  # Obstacle devant
                    # Deux directions tangentielles perpendiculaires
                    tangent1 = normalize(np.array([-to_obstacle[1], to_obstacle[0]]))
                    tangent2 = -tangent1
                    
                    # Choisir la tangente la plus proche de la direction cible
                    score1 = np.dot(tangent1, to_target_norm)
                    score2 = np.dot(tangent2, to_target_norm)
                    
                    # Direction vers le centre de la table (pour éviter les bords)
                    center_vec = normalize(
                        np.array([table_width / 2, table_height / 2]) - robot_pos
                    )
                    dot1 = np.dot(tangent1, center_vec)
                    dot2 = np.dot(tangent2, center_vec)
                    
                    tangent_to_goal = tangent1 if score1 > score2 else tangent2
                    tangent_to_center = tangent1 if dot1 > dot2 else tangent2
                    
                    # Vérifier si tangent_to_goal mène trop près des bords
                    future_pos = robot_pos + tangent_to_goal * 600
                    too_close_to_border = (
                        future_pos[0] < MARGIN_FROM_EDGE or
                        future_pos[0] > table_width - MARGIN_FROM_EDGE or
                        future_pos[1] < MARGIN_FROM_EDGE or
                        future_pos[1] > table_height - MARGIN_FROM_EDGE
                    )
                    
                    chosen_tangent = tangent_to_center if too_close_to_border else tangent_to_goal
                    direction += chosen_tangent * K_TANGENT
    
    # === 5. RÉPULSION DES BORDS DE LA TABLE ===
    wall_repulse = np.array([0.0, 0.0])
    wall_force = 10000.0
    
    # Bord gauche
    if robot_pos[0] < MARGIN_FROM_EDGE:
        wall_repulse[0] += (MARGIN_FROM_EDGE - robot_pos[0]) * wall_force
    # Bord droit
    elif robot_pos[0] > table_width - MARGIN_FROM_EDGE:
        wall_repulse[0] -= (robot_pos[0] - (table_width - MARGIN_FROM_EDGE)) * wall_force
    
    # Bord bas
    if robot_pos[1] < MARGIN_FROM_EDGE:
        wall_repulse[1] += (MARGIN_FROM_EDGE - robot_pos[1]) * wall_force
    # Bord haut
    elif robot_pos[1] > table_height - MARGIN_FROM_EDGE:
        wall_repulse[1] -= (robot_pos[1] - (table_height - MARGIN_FROM_EDGE)) * wall_force
    
    direction += wall_repulse
    
    # === 6. NORMALISATION FINALE ===
    return normalize(direction)


def compute_path_with_potential_field(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Obstacle],
    enemy_pos: Tuple[float, float],
    table_width: float = 3000,
    table_height: float = 2000,
    robot_radius: float = 150,
    enemy_radius: float = 150,
    max_steps: int = 500,
    step_size: float = 50.0,
    goal_threshold: float = 100.0
) -> List[Tuple[float, float]]:
    """
    Génère un chemin complet en suivant le champ de potentiels.
    
    Args:
        start: Position de départ (x, y)
        goal: Position cible (x, y)
        obstacles: Liste des obstacles
        enemy_pos: Position de l'ennemi (x, y)
        table_width: Largeur de la table
        table_height: Hauteur de la table
        robot_radius: Rayon du robot
        enemy_radius: Rayon de l'ennemi
        max_steps: Nombre maximum d'itérations
        step_size: Taille du pas de simulation (mm)
        goal_threshold: Distance pour considérer la cible atteinte (mm)
    
    Returns:
        Liste de points (x, y) formant le chemin
    """
    
    path = [start]
    current_pos = np.array(start, dtype=float)
    goal_pos = np.array(goal, dtype=float)
    enemy_np = np.array(enemy_pos, dtype=float)
    
    for _ in range(max_steps):
        # Vérifier si la cible est atteinte
        dist_to_goal = np.linalg.norm(goal_pos - current_pos)
        if dist_to_goal < goal_threshold:
            path.append(tuple(goal_pos))
            break
        
        # Calculer la direction du champ
        direction = compute_potential_field(
            current_pos,
            goal_pos,
            enemy_np,
            obstacles,
            table_width,
            table_height,
            robot_radius,
            enemy_radius
        )
        
        # Déplacer dans cette direction
        current_pos = current_pos + direction * step_size
        
        # Clamper à la table
        current_pos[0] = np.clip(current_pos[0], robot_radius, table_width - robot_radius)
        current_pos[1] = np.clip(current_pos[1], robot_radius, table_height - robot_radius)
        
        path.append((float(current_pos[0]), float(current_pos[1])))
        
        # Détection de minimum local (oscillation)
        if len(path) > 10:
            recent_pos = np.array(path[-10:])
            variance = np.var(recent_pos, axis=0)
            if np.all(variance < 10):  # Bloqué dans un minimum local
                print("Minimum local détecté, arrêt du pathfinding")
                break
    
    return path


def smooth_path(path: List[Tuple[float, float]], window_size: int = 5) -> List[Tuple[float, float]]:
    """
    Lisse un chemin en moyennant les points voisins.
    
    Args:
        path: Liste de points (x, y)
        window_size: Taille de la fenêtre de lissage
    
    Returns:
        Chemin lissé
    """
    if len(path) < window_size:
        return path
    
    smoothed = [path[0]]  # Garder le point de départ
    
    for i in range(1, len(path) - 1):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(path), i + window_size // 2 + 1)
        
        window = path[start_idx:end_idx]
        avg_x = sum(p[0] for p in window) / len(window)
        avg_y = sum(p[1] for p in window) / len(window)
        
        smoothed.append((avg_x, avg_y))
    
    smoothed.append(path[-1])  # Garder le point d'arrivée
    
    return smoothed

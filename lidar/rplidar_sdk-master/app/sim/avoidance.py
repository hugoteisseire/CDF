import numpy as np
#from simulation_traj import	TABLE_WIDTH_MM ,TABLE_HEIGHT_MM
import heapq
# Paramètres de l’évitement
K_ATTRACT = 0.5
K_REPULSE_ROBOT = 160       # Ennemi
K_REPULSE_OB = 600          # Obstacle
TANGENT_REPULSE = 2      # Longement d'obstacle
REPULSE_RADIUS_ROBOT = 800
REPULSE_RADIUS_OB = 180
MARGIN_FROM_EDGE = 25
LOCAL_WINDOW_SIZE_MM = 1000  # carré de 1000 mm autour du robot

# --- Dimensions réelles de la table (en mm) ---
TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000

CELL_SIZE = 40  # mm
GRID_WIDTH = TABLE_WIDTH_MM // CELL_SIZE
GRID_HEIGHT = TABLE_HEIGHT_MM // CELL_SIZE

# Obstacles fixes (zones rectangulaires)
import pygame
obstacles_rect = [
    pygame.Rect(650, 0, 400, 200),#x,y
    pygame.Rect(1050, 0, 900, 450),
    pygame.Rect(1950, 0, 400, 200),
    pygame.Rect(0, 900, 450, 450),
    pygame.Rect(0, 1850, 450, 150),
    pygame.Rect(2000, 1850, 450, 150),
    pygame.Rect(1550, 1550, 450, 450)
  

]

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def compute_direction_champ(robot, target, enemy, width, height):
    to_target = target - robot.pos
    direction = normalize(to_target) * K_ATTRACT

    # --- Répulsion contre l'ennemi ---
    to_enemy = robot.pos - enemy
    dist_enemy = np.linalg.norm(to_enemy)
    if dist_enemy < REPULSE_RADIUS_ROBOT and dist_enemy != 0:
        direction += normalize(to_enemy) * (K_REPULSE_ROBOT / (dist_enemy))

    # --- Répulsion contre obstacles ---
    for rect in obstacles_rect:
        closest_x = max(rect.left, min(robot.pos[0], rect.right))
        closest_y = max(rect.top, min(robot.pos[1], rect.bottom))
        closest_point = np.array([closest_x, closest_y])
        to_obstacle = robot.pos - closest_point
        dist = np.linalg.norm(to_obstacle)

        if dist < REPULSE_RADIUS_OB and dist != 0:
            repulse_force = normalize(to_obstacle) * (K_REPULSE_OB / (dist ** 1.5))
            direction += repulse_force

            # --- Si l'obstacle est "entre" le robot et la cible ---
            to_target_norm = normalize(to_target)
            to_closest = closest_point - robot.pos
            if np.dot(normalize(to_closest), to_target_norm) > 0.5:
                # Vecteurs tangents possibles (deux directions)
                tangent1 = normalize(np.array([-to_obstacle[1], to_obstacle[0]]))
                tangent2 = -tangent1
                to_goal = normalize(target - robot.pos)
                score1 = np.dot(tangent1, to_goal)
                score2 = np.dot(tangent2, to_goal)                
                center_vec = normalize(np.array([1500, 1000]) - robot.pos)

                # Choisir le tangent le plus proche de la direction vers le centre
                dot1 = np.dot(tangent1, center_vec)
                dot2 = np.dot(tangent2, center_vec)

                tangent_to_goal = tangent1 if score1 > score2 else tangent2
                tangent_to_center = tangent1 if dot1 > dot2 else tangent2

                # Vérifie si tangent_to_goal mène trop près du bord
                future_pos = robot.pos + tangent_to_goal * 600
                too_close_to_border = (
                    future_pos[0] < MARGIN_FROM_EDGE or
                    future_pos[0] > width - MARGIN_FROM_EDGE or
                    future_pos[1] < MARGIN_FROM_EDGE or
                    future_pos[1] > height - MARGIN_FROM_EDGE                )

                chosen_tangent = tangent_to_center if too_close_to_border else tangent_to_goal
                direction += chosen_tangent * TANGENT_REPULSE  # Intensité ajustable




    # --- Répulsion contre murs ---
    wall_repulse = np.array([0.0, 0.0])
    if robot.pos[0] < MARGIN_FROM_EDGE:
        wall_repulse[0] += (MARGIN_FROM_EDGE - robot.pos[0]) * 10000
    elif robot.pos[0] > width - MARGIN_FROM_EDGE:
        wall_repulse[0] -= (robot.pos[0] - (width - MARGIN_FROM_EDGE)) * 10000
    if robot.pos[1] < MARGIN_FROM_EDGE:
        wall_repulse[1] += (MARGIN_FROM_EDGE - robot.pos[1]) * 10000
    elif robot.pos[1] > height - MARGIN_FROM_EDGE:
        wall_repulse[1] -= (robot.pos[1] - (height - MARGIN_FROM_EDGE)) * 10000

    direction += wall_repulse
    # Ajout de la direction brute à l'historique
    if not hasattr(robot, "direction_history"):
        robot.direction_history = []

    robot.direction_history.append(direction)
    if len(robot.direction_history) > 5:  # taille de la fenêtre médiane
        robot.direction_history.pop(0)

    # Appliquer la médiane sur x et y
    if len(robot.direction_history) >= 3:
        dir_array = np.array(robot.direction_history)
        median_direction = np.median(dir_array, axis=0)
        return normalize(median_direction)
    else:
        return normalize(direction)

def clamp_position(pos, radius, width, height):
    pos[0] = max(radius, min(width - radius, pos[0]))
    pos[1] = max(radius, min(height - radius, pos[1]))
    return pos

def pos_to_cell(pos):
    return int(pos[0] // CELL_SIZE), int(pos[1] // CELL_SIZE)

def cell_to_pos(cell):
    return np.array([(cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE])

def get_all_obstacles(obstacles_rect, enemy_pos, robot_radius_mm):
    enemy_rect = pygame.Rect(
        enemy_pos[0] - robot_radius_mm,
        enemy_pos[1] - robot_radius_mm,
        2 * robot_radius_mm,
        2 * robot_radius_mm
    )
    return obstacles_rect + [enemy_rect]

def create_occupancy_grid(obstacles_rect, cell_size, grid_width, grid_height, robot_radius_mm):
    grid = [[0 for _ in range(grid_height)] for _ in range(grid_width)]
    
    margin_cells = math.ceil(robot_radius_mm / cell_size)

    def mark_cell_as_occupied(cx, cy):
        if 0 <= cx < grid_width and 0 <= cy < grid_height:
            grid[cx][cy] = 1

    for rect in obstacles_rect:
        # Agrandir le rectangle de l'obstacle d'une marge égale au rayon du robot
        left = (rect.left - robot_radius_mm) // cell_size
        right = (rect.right + robot_radius_mm - 1) // cell_size
        top = (rect.top - robot_radius_mm) // cell_size
        bottom = (rect.bottom + robot_radius_mm - 1) // cell_size

        for x in range(int(left), int(right) + 1):
            for y in range(int(top), int(bottom) + 1):
                mark_cell_as_occupied(x, y)

    return grid


import heapq
import math

def astar(start, goal, grid):
    w, h = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)  # Heuristique diagonale admissible

    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),      # Cardinales
        (-1, -1), (-1, 1), (1, -1), (1, 1)     # Diagonales
    ]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            x, y = neighbor
            if 0 <= x < w and 0 <= y < h and grid[x][y] == 0:
                # Pondération diagonale
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current

    return []  # Aucun chemin trouvé


def compute_direction_astar(robot, target, obstacles_rect, table_width, table_height, enemy_pos):
    # Vérifie la validité de la cible
    status = check_target_validity(target, robot.radius, enemy_pos, obstacles_rect)
    all_obstacles = obstacles_rect.copy()
    
    enemy_rect = pygame.Rect(
        enemy_pos[0] - robot.radius,
        enemy_pos[1] - robot.radius,
        2 * robot.radius,
        2 * robot.radius
    )
    all_obstacles.append(enemy_rect)

    # Crée la grille en tenant compte du rayon du robot
    grid = create_occupancy_grid(all_obstacles, CELL_SIZE, GRID_WIDTH, GRID_HEIGHT, robot.radius)

    # Détermine le code de retour selon le statut
    if status == "obstacle":
        return [], [], 0 ,grid # Cible dans un décor interdit
    elif status == "enemy":
        code = 1  # Cible sur l'ennemi
    else:
        code = 2  # Cible libre

    # Construire la grille d'occupation avec ou sans l'ennemi
    #include_enemy = (code != 1)
    

    start_cell = pos_to_cell(robot.pos)
    goal_cell = pos_to_cell(target)

    # Si la cible est sur l'ennemi, on vise une cellule libre autour
    if code == 1:
        goal_cell = find_closest_free_cell_around(target, grid, robot.radius)

    # Calcul du chemin
    path = astar(start_cell, goal_cell, grid)

    if path is None or len(path) < 2:
        # Aucun chemin trouvé ou chemin vide
        code = 4
        direction = normalize(target - robot.pos)  # Fallback direction directe
        return direction, [], code,grid

    # Chemin trouvé
    next_cell = path[1]
    next_pos = cell_to_pos(next_cell)
    direction = normalize(next_pos - robot.pos)
    
    return direction, path, code, grid

def find_closest_free_cell_around(target_pos, grid, radius_mm):
    cx, cy = pos_to_cell(target_pos)
    max_radius_cells = int(radius_mm // CELL_SIZE) + 1

    for r in range(1, max_radius_cells + 3):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
                    if grid[nx][ny] == 0:
                        return (nx, ny)
    return (cx, cy)  # fallback (très rare)

def check_target_validity(target, robot_radius_mm, enemy_pos, obstacles_rect):
    """Vérifie dans quel cas se trouve la cible"""
    # Vérifie si cible est dans une zone interdite (décor)
    for rect in obstacles_rect:
        inflated_rect = rect.inflate(2 * robot_radius_mm, 2 * robot_radius_mm)
        if inflated_rect.collidepoint(target[0], target[1]):
            return "obstacle"

    # Vérifie si la cible est dans la zone ennemie (rayon de sécurité autour de lui)
    enemy_rect = pygame.Rect(
        enemy_pos[0] - robot_radius_mm,
        enemy_pos[1] - robot_radius_mm,
        2 * robot_radius_mm,
        2 * robot_radius_mm
    )
    if enemy_rect.collidepoint(target[0], target[1]):
        return "enemy"

    return "clear"



def compute_direction_gbfs(robot, target, obstacles_rect, table_width, table_height, enemy_pos):
    # Vérifie la validité de la cible
    status = check_target_validity(target, robot.radius, enemy_pos, obstacles_rect)

    # Détermine le code de retour selon le statut
    if status == "obstacle":
        return [], [], 0  # Cible dans un décor interdit
    elif status == "enemy":
        code = 1  # Cible sur l'ennemi
    else:
        code = 2  # Cible libre

    # Inclure l'ennemi comme obstacle
    all_obstacles = obstacles_rect.copy()
    enemy_rect = pygame.Rect(
        enemy_pos[0] - robot.radius,
        enemy_pos[1] - robot.radius,
        2 * robot.radius,
        2 * robot.radius
    )
    all_obstacles.append(enemy_rect)

    grid = create_occupancy_grid(all_obstacles, CELL_SIZE, GRID_WIDTH, GRID_HEIGHT, robot.radius)

    start_cell = pos_to_cell(robot.pos)
    goal_cell = pos_to_cell(target)

    if code == 1:
        goal_cell = find_closest_free_cell_around(target, grid, robot.radius)

    path = greedy_best_first_search(start_cell, goal_cell, grid)

    if path is None or len(path) < 2:
        code = 4
        direction = normalize(target - robot.pos)  # fallback
        return direction, [], code

    next_cell = path[1]
    next_pos = cell_to_pos(next_cell)
    direction = normalize(next_pos - robot.pos)

    return direction, path, code

def greedy_best_first_search(start, goal, grid):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), start))
    came_from = {start: None}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            # Reconstruire le chemin
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in get_neighbors(current, grid):
            if neighbor not in came_from:
                came_from[neighbor] = current
                heapq.heappush(frontier, (heuristic(neighbor, goal), neighbor))

    return None  # Aucun chemin trouvé


def heuristic(a, b):
    # Heuristique simple : distance de Manhattan
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(cell, grid):
    neighbors = []
    x, y = cell
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid):
            if not grid[ny][nx]:  # cell not occupied
                neighbors.append((nx, ny))
    return neighbors

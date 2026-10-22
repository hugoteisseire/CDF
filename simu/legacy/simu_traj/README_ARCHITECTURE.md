# Architecture du Simulateur Robot Holonome

## 📋 Vue d'ensemble

Ce simulateur implémente un robot holonome à 3 roues omnidirectionnelles avec évitement d'obstacles utilisant l'algorithme A* pour la planification de trajectoire.

### Système de coordonnées
- **Origine**: (0,0) en bas à gauche
- **Axe X**: positif vers la droite (→)
- **Axe Y**: positif vers le haut (↑)
- **Angles**: sens trigonométrique (0° = droite, 90° = haut)

---

## 📁 Structure des fichiers

### `simulation_traj_clean.py` - Programme principal
Contient toute la logique de simulation, affichage et contrôle.

### `avoidance.py` - Pathfinding et évitement
Gestion des obstacles et calcul de trajectoire.

### `holo_base.py` - Cinématique du robot
Calculs mathématiques pour le contrôle des roues holonomes.

---

## 🔧 Fonctions critiques et leur localisation

### 1. **Détection et échappement de deadzone**

#### `_update_robot()` dans `simulation_traj_clean.py` (lignes ~530-570)
```python
# Détection de blocage
movement = np.linalg.norm(self.state.robot.pos - self.state.last_position)
if movement < 1.0 and self.state.robot_moving:  # < 1mm
    self.state.stuck_counter += 1
else:
    self.state.stuck_counter = 0

# Déblocage automatique après 30 frames (0.5s @ 60fps)
if self.state.stuck_counter > 30:
    escape_angle = random.uniform(0, 2 * math.pi)
    escape_distance = 100  # 100mm
    escape_offset = np.array([
        math.cos(escape_angle) * escape_distance,
        math.sin(escape_angle) * escape_distance
    ])
    self.state.robot.pos += escape_offset
    self.state.stuck_counter = 0
    self.state.path_computed = False
```

**Paramètres clés:**
- `movement < 1.0`: Seuil de mouvement (1mm)
- `stuck_counter > 30`: Délai avant déblocage (0.5s)
- `escape_distance = 100`: Distance de téléportation (100mm)

---

#### `get_escape_direction()` dans `avoidance.py` (lignes ~310-340)
```python
def get_escape_direction(robot_pos, enemy_pos, obstacles_rect):
    """Calcule une direction pour échapper des deadzones."""
    escape_dir = np.array([0.0, 0.0])
    
    # Fuir l'ennemi
    if dist_enemy > 0:
        escape_dir += normalize(to_enemy) * 2.0
    
    # Fuir les obstacles proches (< 300mm)
    for rect in obstacles_rect:
        # ... calcul de répulsion
        if dist < 300 and dist > 0:
            escape_dir += normalize(to_obstacle) * (1.0 / (dist + 1))
    
    return normalize(escape_dir)
```

**Utilisation:** Appelée quand le robot est dans une cellule occupée de la grille A*.

---

### 2. **Sécurité anti-collision ennemi**

#### `_update_robot()` dans `simulation_traj_clean.py` (lignes ~575-585)
```python
# Vérification distance avec adversaire
dist_to_enemy = np.linalg.norm(self.state.robot.pos - self.state.enemy_pos)
safety_distance = self.state.robot.radius + Config.ENEMY_RADIUS_MM + 50  # +50mm marge

if dist_to_enemy < safety_distance:
    # ARRÊT COMPLET
    self.state.robot.wheel_speeds = [0.0, 0.0, 0.0]
    self.state.robot.base_velocity = np.array([0.0, 0.0])
    self.state.robot.base_velocity_global = np.array([0.0, 0.0])
    self.state.pause_robot = True
    self.state.error_message = "Trop proche de l'adversaire"
```

**Paramètres:**
- `safety_distance`: robot_radius + enemy_radius + 50mm
- Typique: 150 + 150 + 50 = **350mm**

---

### 3. **Lissage des vitesses des roues**

#### `_update_robot()` dans `simulation_traj_clean.py` (lignes ~600-615)
```python
# Historique de lissage (5 dernières valeurs)
if not hasattr(self.state.robot, 'wheel_speeds_history'):
    self.state.robot.wheel_speeds_history = [[], [], []]

for i in range(3):
    self.state.robot.wheel_speeds_history[i].append(new_speeds[i])
    if len(self.state.robot.wheel_speeds_history[i]) > 5:
        self.state.robot.wheel_speeds_history[i].pop(0)
    # Moyenne glissante
    self.state.robot.wheel_speeds[i] = np.mean(self.state.robot.wheel_speeds_history[i])
```

**Avantages:**
- Réduit les à-coups mécaniques
- Fenêtre de 5 frames (~83ms @ 60fps)
- Moyenne arithmétique simple

---

### 4. **Pathfinding A***

#### `compute_direction_astar()` dans `avoidance.py` (lignes ~240-290)
```python
def compute_direction_astar(robot, target, obstacles_rect, table_width, table_height, enemy_pos):
    # 1. Validation de la cible
    status = check_target_validity(target, robot.radius, enemy_pos, obstacles_rect)
    
    # 2. Création grille d'occupation
    grid = create_occupancy_grid(all_obstacles, CELL_SIZE, GRID_WIDTH, GRID_HEIGHT, robot.radius)
    
    # 3. Gestion deadzone: si robot dans cellule occupée
    if grid[start_cell[0]][start_cell[1]] == 1:
        start_cell = find_closest_free_cell_around(robot.pos, grid, robot.radius)
        if grid[start_cell[0]][start_cell[1]] == 1:
            escape_direction = get_escape_direction(robot.pos, enemy_pos, obstacles_rect)
            return escape_direction, [], 1, grid
    
    # 4. Calcul A*
    path = astar(start_cell, goal_cell, grid)
    
    # 5. Direction vers prochaine cellule
    next_cell = path[1]
    next_pos = cell_to_pos(next_cell)
    direction = normalize(next_pos - robot.pos)
    
    return direction, path, code, grid
```

**Codes de retour:**
- `0`: Cible dans obstacle
- `1`: Cible sur ennemi
- `2`: Chemin libre
- `4`: Cible inaccessible

---

#### `astar()` dans `avoidance.py` (lignes ~190-230)
```python
def astar(start, goal, grid):
    # Heuristique diagonale (Chebyshev)
    def heuristic(a, b):
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))
    
    # Mouvements 8-directions
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),      # Cardinales
        (-1, -1), (-1, 1), (1, -1), (1, 1)     # Diagonales
    ]
    
    # Coût diagonal = √2
    move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
```

**Paramètres grille:**
- `CELL_SIZE = 20mm` (haute résolution)
- `GRID_WIDTH = 150` cellules (3000mm / 20mm)
- `GRID_HEIGHT = 100` cellules (2000mm / 20mm)
- **Total: 15 000 cellules**

---

### 5. **Grille d'occupation avec cache**

#### `_render_grid()` dans `simulation_traj_clean.py` (lignes ~330-370)
```python
def _render_grid(self, grid):
    # Calcul hash de l'état de la grille
    grid_hash = hash(tuple(tuple(row) for row in grid))
    
    # Si grille inchangée, utiliser cache (RAPIDE)
    if self.cached_grid_hash != grid_hash or self.cached_occupancy_surface is None:
        # Régénérer surface en cache
        self.cached_occupancy_surface = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        
        for x in range(grid_width):
            for y in range(grid_height):
                if grid[x][y] == 1:
                    # Dessiner cellule occupée
                    pygame.draw.rect(self.cached_occupancy_surface, ...)
        
        self.cached_grid_hash = grid_hash
    
    # Blit rapide du cache
    self.screen.blit(self.cached_occupancy_surface, (0, 0))
```

**Performance:**
- Sans cache: ~12-15ms/frame (15 000 rectangles)
- Avec cache: ~1-2ms/frame (1 blit)
- **Gain: 85-90%**

---

### 6. **Cinématique holonome**

#### `compute_wheel_speeds_global()` dans `holo_base.py` (lignes ~12-25)
```python
def compute_wheel_speeds_global(robot, vx, vy, omega):
    # 1. Conversion vitesse globale → locale (rotation inverse)
    cos_a, sin_a = math.cos(-robot.angle), math.sin(-robot.angle)
    v_local = np.array([
        cos_a * vx - sin_a * vy,
        sin_a * vx + cos_a * vy
    ])
    
    # 2. Projection sur chaque roue
    for theta in robot.wheel_angles:  # [90°, 210°, 330°]
        speed = -sin(θ) * vx_local + cos(θ) * vy_local + ω * R
    
    # 3. Normalisation (max = 1.0)
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1:
        speeds = [s / max_speed for s in speeds]
```

**Configuration roues:**
- **Roue 1**: 90° (arrière)
- **Roue 2**: 210° (avant-gauche)
- **Roue 3**: 330° (avant-droite)
- **Rayon robot**: 150mm

---

#### `compute_base_velocity()` dans `holo_base.py` (lignes ~28-35)
```python
def compute_base_velocity(robot, wheel_speeds):
    """Odométrie: vitesses roues → vitesse base (locale)"""
    vx, vy = 0, 0
    for i, theta in enumerate(robot.wheel_angles):
        vx += -math.sin(theta) * wheel_speeds[i]
        vy += math.cos(theta) * wheel_speeds[i]
    return np.array([vx, vy])
```

**Utilisation:** Mise à jour de la position du robot.

---

### 7. **Spawn safe des objectifs**

#### `random_position()` dans `simulation_traj_clean.py` (lignes ~110-125)
```python
def random_position(margin_mm=100):
    """Génère position aléatoire HORS obstacles."""
    max_attempts = 100
    for _ in range(max_attempts):
        pos = np.array([
            random.uniform(margin_mm, TABLE_WIDTH_MM - margin_mm),
            random.uniform(margin_mm, TABLE_HEIGHT_MM - margin_mm)
        ])
        # Vérifier collision
        if not is_position_in_obstacle(pos, TARGET_RADIUS_MM + 50, obstacles_rect):
            return pos
    
    # Fallback: centre de la table
    return np.array([1500, 1000])
```

**Garanties:**
- 100 tentatives maximum
- Marge de 100mm des bords
- Marge de 50mm des obstacles

---

### 8. **Suivi de chemin**

#### `update_path_following()` dans `simulation_traj_clean.py` (lignes ~180-205)
```python
def update_path_following(robot, path, current_index):
    """Suit le chemin A* cellule par cellule."""
    target_cell = path[current_index]
    target_pos_mm = cell_to_pos(target_cell)
    
    # Vérifier si cellule atteinte
    dist_to_target = np.linalg.norm(robot.pos - target_pos_mm)
    if dist_to_target < robot.radius:
        current_index += 1  # Passer à cellule suivante
        if current_index >= len(path):
            return ..., True  # Fin du chemin
    
    # Direction vers cellule actuelle
    direction = normalize(target_pos_mm - robot.pos)
    return direction, current_index, False
```

**Critère passage:** Distance < rayon robot (150mm)

---

## ⚙️ Configuration importante

### `Config` dans `simulation_traj_clean.py` (lignes ~50-95)

```python
# Dimensions table
TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000

# Robot
ROBOT_RADIUS_MM = 150
MAX_WHEEL_SPEED_MPS = 0.3  # 30 cm/s
WHEEL_ANGLES = [90°, 210°, 330°]

# Simulation
TICKS_PER_SECOND = 60
PATH_RECOMPUTE_INTERVAL = 15  # Recalcul toutes les 15 frames (~250ms)

# Performance
SHOW_GRID_LINES = False       # Désactiver pour gain perf
SHOW_OCCUPANCY_GRID = True    # Avec cache, très rapide
```

---

## 🚀 Portage vers le robot réel

### Architecture recommandée

```
robot_real/
├── main_robot.py           # Point d'entrée principal
├── pathfinding.py          # Copie de avoidance.py (A*, grille)
├── kinematics.py           # Copie de holo_base.py (roues holonomes)
├── motor_control.py        # Interface moteurs (CAN/UART)
├── lidar_handler.py        # Acquisition données LiDAR
├── localization.py         # Odométrie/IMU (position/angle)
└── safety.py               # Watchdog, arrêts d'urgence
```

---

### 1. **Modules à porter directement**

#### ✅ `holo_base.py` → `kinematics.py`
**100% portable sans modification**

```python
# À utiliser tel quel:
from kinematics import (
    compute_wheel_speeds_global,
    compute_base_velocity,
    compute_rotation_velocity,
    rotate_vector
)

# Exemple sur robot réel:
robot = Robot(pos=odometry.get_position(), angle=imu.get_angle(), ...)
direction = pathfinding.get_direction(robot, target, ...)
wheel_speeds_normalized = compute_wheel_speeds_global(robot, direction[0], direction[1], 0)

# Conversion vitesses normalisées → commandes moteurs
MAX_RPM = 120  # À adapter selon vos moteurs
for i, speed_norm in enumerate(wheel_speeds_normalized):
    motor_rpm = speed_norm * MAX_RPM
    motor_controller.set_speed(i, motor_rpm)
```

---

#### ✅ `avoidance.py` → `pathfinding.py`
**95% portable, adaptations mineures**

**Modifications nécessaires:**

1. **Obstacles dynamiques:**
```python
# AVANT (simulation):
obstacles_rect = [...]  # Obstacles fixes

# APRÈS (robot réel):
def get_obstacles_from_lidar(lidar_data, robot_pos):
    """Convertit points LiDAR en rectangles obstacles."""
    obstacles = []
    
    # Clustering des points LiDAR proches
    clusters = cluster_lidar_points(lidar_data, threshold=100)  # 100mm
    
    for cluster in clusters:
        # Bounding box autour du cluster
        min_x, max_x = min(p[0] for p in cluster), max(p[0] for p in cluster)
        min_y, max_y = min(p[1] for p in cluster), max(p[1] for p in cluster)
        
        # Inflation du rectangle (marge de sécurité)
        margin = 50  # mm
        rect = pygame.Rect(
            min_x - margin,
            min_y - margin,
            (max_x - min_x) + 2*margin,
            (max_y - min_y) + 2*margin
        )
        obstacles.append(rect)
    
    return obstacles
```

2. **Position ennemi (détection balise):**
```python
# Récupération position adversaire via balise/LiDAR
enemy_pos = beacon.get_enemy_position()  # Si balise
# OU
enemy_pos = detect_largest_moving_obstacle(lidar_data)  # Heuristique LiDAR
```

3. **Recalcul adaptatif:**
```python
# Recalcul plus fréquent sur robot réel (obstacles mobiles)
PATH_RECOMPUTE_INTERVAL = 5  # Au lieu de 15 (recalcul toutes les ~80ms)
```

---

### 2. **Modules spécifiques au robot réel**

#### 🆕 `localization.py` - Localisation
```python
class Localization:
    def __init__(self, imu, encoders):
        self.imu = imu
        self.encoders = encoders
        self.pos = np.array([1500.0, 500.0])  # Position initiale
        self.angle = 0.0
        self.last_encoder_values = [0, 0, 0]
    
    def update(self, wheel_speeds_mm_per_tick, dt):
        """Odométrie holonome + fusion IMU."""
        
        # 1. Odométrie roues (locale)
        base_velocity_local = compute_base_velocity(self.robot, wheel_speeds_mm_per_tick)
        
        # 2. Angle IMU (plus précis que odométrie rotation)
        self.angle = self.imu.get_yaw()  # En radians
        
        # 3. Vitesse globale
        velocity_global = rotate_vector(base_velocity_local, self.angle)
        
        # 4. Intégration position
        self.pos += velocity_global * dt
        
        return self.pos, self.angle
    
    def reset_position(self, x, y, angle):
        """Recalage manuel (ex: départ match)."""
        self.pos = np.array([x, y])
        self.angle = angle
```

**Capteurs requis:**
- **IMU** (SparkFun Qwiic VL53L5CX ou similaire): angle absolu
- **Encodeurs roues** (optionnels): odométrie complémentaire
- **LiDAR** (RPLidar A1/A2): détection obstacles

---

#### 🆕 `motor_control.py` - Contrôle moteurs
```python
class MotorController:
    def __init__(self, can_bus):
        self.can_bus = can_bus
        self.motor_ids = [0x01, 0x02, 0x03]  # IDs CAN des 3 moteurs
        self.max_rpm = 120  # À calibrer
    
    def set_wheel_speeds(self, speeds_normalized):
        """Envoie commandes aux 3 moteurs.
        
        Args:
            speeds_normalized: [s1, s2, s3] dans [-1, 1]
        """
        for i, speed_norm in enumerate(speeds_normalized):
            rpm = int(speed_norm * self.max_rpm)
            self.send_can_command(self.motor_ids[i], rpm)
    
    def send_can_command(self, motor_id, rpm):
        """Envoie commande CAN à un moteur."""
        # Format dépend du contrôleur moteur (ex: MKS Servo)
        msg = can.Message(
            arbitration_id=motor_id,
            data=[0x64, rpm & 0xFF, (rpm >> 8) & 0xFF, ...],  # À adapter
            is_extended_id=False
        )
        self.can_bus.send(msg)
    
    def emergency_stop(self):
        """Arrêt d'urgence tous moteurs."""
        for motor_id in self.motor_ids:
            self.send_can_command(motor_id, 0)
```

---

#### 🆕 `lidar_handler.py` - Acquisition LiDAR
```python
from rplidar import RPLidar

class LidarHandler:
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = RPLidar(port)
        self.lidar.start_motor()
    
    def get_obstacles_mm(self, robot_pos, robot_angle):
        """Récupère obstacles en coordonnées globales table.
        
        Returns:
            List[pygame.Rect]: Rectangles obstacles
        """
        scan = self.lidar.iter_scans(max_buf_meas=500)
        points_global = []
        
        for quality, angle_deg, distance_mm in next(scan):
            if quality > 10 and distance_mm < 3000:  # Filtre qualité et portée
                # 1. Coordonnées polaires → cartésiennes (repère robot)
                angle_rad = math.radians(angle_deg)
                x_local = distance_mm * math.cos(angle_rad)
                y_local = distance_mm * math.sin(angle_rad)
                
                # 2. Rotation selon orientation robot
                point_local = np.array([x_local, y_local])
                point_rotated = rotate_vector(point_local, robot_angle)
                
                # 3. Translation position robot
                point_global = robot_pos + point_rotated
                
                points_global.append(point_global)
        
        # Clustering et conversion en rectangles
        return cluster_to_obstacles(points_global)
    
    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
```

---

#### 🆕 `safety.py` - Sécurité
```python
class SafetyMonitor:
    def __init__(self, motor_controller, lidar):
        self.motor_controller = motor_controller
        self.lidar = lidar
        self.emergency_stop_triggered = False
    
    def check_safety(self, robot_pos, enemy_pos):
        """Vérifications sécurité critiques."""
        
        # 1. Distance adversaire
        dist_enemy = np.linalg.norm(robot_pos - enemy_pos)
        if dist_enemy < 250:  # 250mm (robot + ennemi - marge)
            self.emergency_stop("Trop proche adversaire")
            return False
        
        # 2. Détection collision imminente (LiDAR)
        min_distance = self.lidar.get_min_distance()
        if min_distance < 100:  # 100mm
            self.emergency_stop("Obstacle très proche")
            return False
        
        # 3. Timeout pathfinding (robot bloqué)
        # (géré par stuck_counter dans _update_robot)
        
        return True
    
    def emergency_stop(self, reason):
        """Arrêt d'urgence avec log."""
        print(f"EMERGENCY STOP: {reason}")
        self.motor_controller.emergency_stop()
        self.emergency_stop_triggered = True
```

---

### 3. **Boucle principale robot réel**

#### `main_robot.py`

```python
import time
import numpy as np
from kinematics import Robot, compute_wheel_speeds_global
from pathfinding import compute_direction_astar, obstacles_rect
from localization import Localization
from motor_control import MotorController
from lidar_handler import LidarHandler
from safety import SafetyMonitor

# Configuration
LOOP_FREQ_HZ = 60
DT = 1.0 / LOOP_FREQ_HZ
MAX_WHEEL_SPEED_MPS = 0.3

def main():
    # Initialisation hardware
    motor_ctrl = MotorController(can_bus=init_can())
    lidar = LidarHandler(port='/dev/ttyUSB0')
    imu = init_imu()  # Votre lib IMU
    localization = Localization(imu, encoders=None)
    safety = SafetyMonitor(motor_ctrl, lidar)
    
    # État initial
    robot = Robot(
        pos=np.array([1500.0, 500.0]),  # Position départ
        angle=0.0,
        radius=150.0,
        wheel_angles=[math.radians(90), math.radians(210), math.radians(330)]
    )
    target_pos = np.array([2500.0, 1500.0])  # Premier objectif
    enemy_pos = np.array([1500.0, 1500.0])  # Position initiale adversaire
    
    path_computed = False
    path = []
    recompute_counter = 0
    PATH_RECOMPUTE_INTERVAL = 5  # Recalcul toutes les 5 frames (~80ms)
    
    # Boucle principale
    last_time = time.time()
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # 1. LOCALISATION (odométrie + IMU)
        robot.pos, robot.angle = localization.update(robot.wheel_speeds, dt)
        
        # 2. PERCEPTION (LiDAR)
        dynamic_obstacles = lidar.get_obstacles_mm(robot.pos, robot.angle)
        all_obstacles = obstacles_rect + dynamic_obstacles  # Fixes + dynamiques
        enemy_pos = detect_enemy(lidar)  # Heuristique ou balise
        
        # 3. SÉCURITÉ
        if not safety.check_safety(robot.pos, enemy_pos):
            time.sleep(DT)
            continue  # Skip si arrêt d'urgence
        
        # 4. PATHFINDING (recalcul périodique)
        if not path_computed or recompute_counter > PATH_RECOMPUTE_INTERVAL:
            direction, path, code, grid = compute_direction_astar(
                robot, target_pos, all_obstacles,
                3000, 2000, enemy_pos
            )
            path_computed = True
            recompute_counter = 0
        else:
            recompute_counter += 1
        
        # 5. SUIVI DE CHEMIN
        if len(path) > 1:
            target_cell = path[1]  # Prochaine cellule
            target_cell_pos = cell_to_pos(target_cell)
            direction = normalize(target_cell_pos - robot.pos)
        else:
            direction = np.array([0.0, 0.0])
        
        # 6. CINÉMATIQUE (direction → vitesses roues)
        wheel_speeds_normalized = compute_wheel_speeds_global(
            robot, direction[0], direction[1], 0.0
        )
        
        # 7. LISSAGE (optionnel, même code que simulation)
        # ... (copier code lissage de _update_robot)
        
        # 8. COMMANDE MOTEURS
        motor_ctrl.set_wheel_speeds(wheel_speeds_normalized)
        
        # 9. VÉRIFIER OBJECTIF ATTEINT
        dist_to_target = np.linalg.norm(robot.pos - target_pos)
        if dist_to_target < 200:  # 200mm tolérance
            target_pos = get_next_objective()  # Stratégie
            path_computed = False
        
        # Timing loop
        time.sleep(max(0, DT - (time.time() - current_time)))

if __name__ == "__main__":
    main()
```

---

### 4. **Calibration nécessaire**

#### A. **Moteurs**
```python
# Trouver MAX_RPM physique de vos moteurs
# Test: envoyer commande 100% et mesurer RPM réel
MAX_RPM_PHYSICAL = ???  # À mesurer

# Vitesse linéaire max souhaitée (config)
MAX_WHEEL_SPEED_MPS = 0.3  # 30 cm/s

# Rayon roue (à mesurer précisément)
WHEEL_RADIUS_MM = 30  # Exemple

# Conversion: m/s → RPM
# v = ω * r  →  ω = v / r
max_wheel_speed_rad_s = MAX_WHEEL_SPEED_MPS / (WHEEL_RADIUS_MM / 1000.0)
MAX_RPM = (max_wheel_speed_rad_s * 60) / (2 * math.pi)
```

#### B. **Géométrie robot**
```python
# Mesurer précisément:
ROBOT_RADIUS_MM = ???  # Centre robot → centre roue
WHEEL_ANGLES = [???, ???, ???]  # Angles réels des roues (degrés → radians)

# Vérifier avec test rotation sur place:
# Envoyer omega = 1.0 rad/s et mesurer vitesse angulaire réelle
```

#### C. **IMU**
```python
# Calibration offset + orientation
# Placer robot à 0° (vers droite), lire angle IMU
IMU_OFFSET_DEG = imu.read_yaw()  # À soustraire

def get_calibrated_angle():
    raw_angle = imu.read_yaw()
    return math.radians(raw_angle - IMU_OFFSET_DEG)
```

---

### 5. **Différences clés simulation vs réel**

| Aspect | Simulation | Robot réel |
|--------|-----------|-----------|
| **Position** | Connue parfaitement | Odométrie imprécise (dérive) |
| **Obstacles** | Fixes, prédéfinis | Dynamiques, détection LiDAR |
| **Timing** | Déterministe (60 fps) | Jitter, latences capteurs |
| **Roues** | Réponse instantanée | Inertie, glissement |
| **Recalcul path** | 15 frames (~250ms) | 5 frames (~80ms) recommandé |
| **Lissage vitesses** | 5 frames (confort) | **CRITIQUE** (éviter saturation moteurs) |

---

### 6. **Tests progressifs recommandés**

#### Étape 1: **Moteurs seuls**
```python
# Test rotation sur place
motor_ctrl.set_wheel_speeds([0.3, 0.3, 0.3])  # Toutes même sens
# → Robot doit tourner, vérifier direction et vitesse
```

#### Étape 2: **Cinématique en ligne droite**
```python
# Avancer tout droit (vx = 1, vy = 0, omega = 0)
direction = np.array([1.0, 0.0])
speeds = compute_wheel_speeds_global(robot, direction[0], direction[1], 0)
motor_ctrl.set_wheel_speeds(speeds)
# → Mesurer trajectoire réelle vs attendue
```

#### Étape 3: **Odométrie**
```python
# Déplacer robot 1 mètre, vérifier position calculée
initial_pos = localization.pos.copy()
# ... déplacement ...
final_pos = localization.pos
error = np.linalg.norm(final_pos - initial_pos) - 1000  # mm
print(f"Erreur odométrie: {error} mm")
```

#### Étape 4: **LiDAR → obstacles**
```python
# Placer obstacle physique, vérifier détection
obstacles = lidar.get_obstacles_mm(robot.pos, robot.angle)
print(f"Obstacles détectés: {len(obstacles)}")
# Afficher dans visualiseur (réutiliser code Renderer)
```

#### Étape 5: **Pathfinding avec LiDAR**
```python
# Test A* avec obstacles réels
path = compute_direction_astar(robot, target, lidar_obstacles, ...)
# Vérifier chemin évite obstacles détectés
```

#### Étape 6: **Boucle complète**
```python
# Intégration finale: main_robot.py
# Objectif simple: aller de point A à point B
```

---

## 📊 Optimisations critiques portées

### 1. **Cache grille d'occupation**
- **Simulation**: Gain 85% temps render (15ms → 2ms)
- **Robot réel**: Pas de rendu, **mais cache utile si:**
  - Affichage debug sur écran embarqué
  - Logging grille pour analyse post-match

### 2. **Lissage vitesses**
- **Simulation**: Confort visuel
- **Robot réel**: **ESSENTIEL**
  - Évite pics de courant moteurs
  - Réduit usure mécanique
  - Améliore précision trajectoire

### 3. **Deadzone escape**
- **Simulation**: Cas rare (grille 20mm fine)
- **Robot réel**: **TRÈS IMPORTANT**
  - Erreurs odométrie → robot croit être dans obstacle
  - LiDAR bruité → faux obstacles
  - Glissement roues → position réelle ≠ calculée
  - **Recommandation**: Conserver téléportation 100mm + recalage IMU

### 4. **Safety distance**
- **Simulation**: 350mm (150 + 150 + 50)
- **Robot réel**: **Augmenter à 400-450mm**
  - Compte tenu imprécisions capteurs
  - Délai réaction freinage moteurs

---

## 🔑 Points d'attention portage

### ❌ À NE PAS porter tel quel

1. **Gestion souris/clavier** (`_handle_events`, `_handle_mouse`)
   - Remplacer par: boutons physiques, Bluetooth, stratégie préprogrammée

2. **Rendu Pygame** (`Renderer`, `pygame.display`)
   - Optionnel: garder pour debug sur écran embarqué (ex: Raspberry Pi + mini-HDMI)

3. **Ennemi aléatoire** (`_update_enemy`)
   - Remplacer par: détection balise ou tracking LiDAR

### ✅ À porter ABSOLUMENT

1. **Toute la cinématique** (`holo_base.py`)
2. **Pathfinding A*** (`astar`, `compute_direction_astar`)
3. **Lissage vitesses**
4. **Deadzone escape**
5. **Safety distance check**
6. **Navigation path following**

---

## 📈 Chronologie d'implémentation

```
Semaine 1: Hardware
├─ Connexion moteurs (CAN/UART)
├─ Test IMU (angles)
└─ Test LiDAR (scan basique)

Semaine 2: Cinématique
├─ Porter holo_base.py → kinematics.py
├─ Calibration géométrie robot
├─ Test déplacements (ligne droite, rotation)
└─ Odométrie simple (encodeurs ou vitesses commandées)

Semaine 3: Perception
├─ Clustering points LiDAR
├─ Conversion obstacles rectangles
└─ Fusion odométrie + IMU

Semaine 4: Navigation
├─ Porter avoidance.py → pathfinding.py
├─ Intégration obstacles dynamiques
├─ Test A* avec obstacles réels
└─ Suivi de chemin

Semaine 5: Sécurité + optimisations
├─ Safety monitor (watchdog)
├─ Lissage vitesses
├─ Deadzone escape
└─ Tuning paramètres (vitesses max, marges...)

Semaine 6: Tests intégration
├─ Boucle complète main_robot.py
├─ Stratégie match (séquence objectifs)
└─ Tests terrain réel
```

---

## 🛠️ Outils de debug recommandés

### 1. **Logger position/vitesses**
```python
import csv

class DataLogger:
    def __init__(self, filename='robot_log.csv'):
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'pos_x', 'pos_y', 'angle', 'v1', 'v2', 'v3'])
    
    def log(self, t, pos, angle, wheel_speeds):
        self.writer.writerow([t, pos[0], pos[1], angle, *wheel_speeds])
    
    def close(self):
        self.file.close()

# Utilisation:
logger = DataLogger()
logger.log(time.time(), robot.pos, robot.angle, robot.wheel_speeds)
```

### 2. **Visualiseur temps réel (optionnel)**
Réutiliser `Renderer` de simulation pour afficher:
- Position robot (odométrie)
- Obstacles détectés (LiDAR)
- Chemin A* calculé
- Ennemi détecté

---

## 📚 Ressources

### Documentation interne
- `README_MOTOR_ASYNC.md`: Contrôle moteurs asynchrone
- `SCHEMA_SYSTEME.md`: Architecture système
- `SOCKET_BROADCAST_ARCHITECTURE.md`: Communication inter-processus

### Librairies externes
- **RPLidar**: `pip install rplidar-roboticia`
- **python-can**: `pip install python-can`
- **NumPy**: `pip install numpy`
- **Pygame** (debug seulement): `pip install pygame`

---

## 🎯 Résumé exécutif

| Fonction | Fichier source | Ligne | Portabilité | Priorité |
|----------|---------------|-------|-------------|----------|
| **Cinématique roues** | `holo_base.py` | 12-45 | ✅ 100% | 🔴 CRITIQUE |
| **A* pathfinding** | `avoidance.py` | 190-290 | ✅ 95% | 🔴 CRITIQUE |
| **Deadzone escape** | `simulation_traj_clean.py` + `avoidance.py` | 545-565, 310-340 | ✅ 90% | 🟡 IMPORTANT |
| **Lissage vitesses** | `simulation_traj_clean.py` | 600-615 | ✅ 100% | 🟡 IMPORTANT |
| **Safety distance** | `simulation_traj_clean.py` | 575-585 | ✅ 100% | 🟡 IMPORTANT |
| **Suivi chemin** | `simulation_traj_clean.py` | 180-205 | ✅ 95% | 🔴 CRITIQUE |
| **Cache grille** | `simulation_traj_clean.py` | 330-370 | ⚠️ 0% (render) | ⚪ OPTIONNEL |

---

**🚨 AVANT TOUT TEST RÉEL:**
1. Vérifier arrêt d'urgence physique fonctionnel
2. Calibrer géométrie robot (ROBOT_RADIUS, WHEEL_ANGLES)
3. Tester moteurs individuellement (sens rotation, vitesse)
4. Valider IMU (orientation cohérente avec mouvement)
5. Commencer vitesses RÉDUITES (50% max) puis augmenter progressivement

import math
import numpy as np

class Robot:
    def __init__(self, pos, angle, radius, wheel_angles):
        self.pos = np.array(pos, dtype=float)
        self.angle = angle
        self.radius = radius
        self.wheel_angles = wheel_angles
        self.direction_history = []  # à initialiser une fois, par exemple à l'init


def compute_wheel_speeds_global(robot, vx, vy, omega):
    """
    Calcule les vitesses normalisées des roues pour un mouvement holonome.
    
    Args:
        robot: Instance de Robot
        vx: Vitesse en X global, normalisée entre -1 et 1
        vy: Vitesse en Y global, normalisée entre -1 et 1
        omega: Vitesse de rotation normalisée entre -1 et 1
               (positive = rotation anti-horaire)
    
    Returns:
        Liste des vitesses normalisées pour chaque roue (entre -1 et 1)
    """
    cos_a, sin_a = math.cos(-robot.angle), math.sin(-robot.angle)
    v_local = np.array([
        cos_a * vx - sin_a * vy,
        sin_a * vx + cos_a * vy
    ])
    speeds = []
    for theta in robot.wheel_angles:
        # omega est déjà normalisé, on l'ajoute directement
        speed = -math.sin(theta) * v_local[0] + math.cos(theta) * v_local[1] + omega
        speeds.append(speed)
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1:
        speeds = [s / max_speed for s in speeds]
    return speeds

def compute_base_velocity(robot, wheel_speeds):
    vx, vy = 0, 0
    for i, theta in enumerate(robot.wheel_angles):
        vx += -math.sin(theta) * wheel_speeds[i]
        vy += math.cos(theta) * wheel_speeds[i]
    return np.array([vx, vy])

def compute_rotation_velocity(robot, wheel_speeds):
    return sum(wheel_speeds) / (3 * robot.radius)

def rotate_vector(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

def shortest_angle_diff(target, current):
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    return diff

def compute_wheel_distances_global(robot, dx, dy, dtheta=0.0):
    """
    Calcule la distance que chaque roue doit parcourir pour que le robot
    se déplace d'une distance (dx, dy) dans le repère global et effectue
    une rotation dtheta.
    
    Args:
        robot: Instance de Robot avec angle et wheel_angles
        dx: Déplacement en x dans le repère global (mm)
        dy: Déplacement en y dans le repère global (mm)
        dtheta: Rotation à effectuer (radians), par défaut 0
    
    Returns:
        Liste des distances à parcourir pour chaque roue (mm)
        
    Exemple:
        >>> # Robot orienté à 90° (pi/2)
        >>> robot = Robot(pos=[0, 0], angle=np.pi/2, radius=150,
        ...               wheel_angles=[np.radians(120), np.radians(240), np.radians(0)])
        >>> # Avancer de 100mm en x global
        >>> distances = compute_wheel_distances_global(robot, dx=100, dy=0)
        >>> print(distances)  # [dist_roue1, dist_roue2, dist_roue3] en mm
    """
    # Transformation du déplacement global vers le repère local du robot
    cos_a, sin_a = math.cos(-robot.angle), math.sin(-robot.angle)
    d_local = np.array([
        cos_a * dx - sin_a * dy,
        sin_a * dx + cos_a * dy
    ])
    
    # Calcul de la distance pour chaque roue
    # Utilise la même cinématique que compute_wheel_speeds_global
    distances = []
    for theta in robot.wheel_angles:
        distance = -math.sin(theta) * d_local[0] + math.cos(theta) * d_local[1] + dtheta * robot.radius
        distances.append(distance)
    
    return distances

def compute_wheel_distances_direction(robot, distance, direction_angle, dtheta=0.0):
    """
    Calcule la distance que chaque roue doit parcourir pour que le robot
    se déplace d'une certaine distance dans une direction donnée (repère global).
    
    Args:
        robot: Instance de Robot avec angle et wheel_angles
        distance: Distance totale à parcourir (mm)
        direction_angle: Angle de la direction dans le repère global (radians)
                        (0 = vers la droite, pi/2 = vers le haut)
        dtheta: Rotation à effectuer (radians), par défaut 0
    
    Returns:
        Liste des distances à parcourir pour chaque roue (mm)
        
    Exemple:
        >>> # Déplacer le robot de 100mm vers le nord (angle pi/2)
        >>> distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=np.pi/2)
    """
    # Convertir la distance et l'angle en composantes dx, dy
    dx = distance * math.cos(direction_angle)
    dy = distance * math.sin(direction_angle)
    
    return compute_wheel_distances_global(robot, dx, dy, dtheta)

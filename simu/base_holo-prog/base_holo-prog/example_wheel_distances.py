"""
Exemple d'utilisation des fonctions de calcul de distances de roues.
Démontre comment calculer les distances que chaque roue doit parcourir
pour déplacer le robot d'une distance donnée dans une direction spécifique.
"""

import numpy as np
import math
from holo_base import Robot, compute_wheel_distances_global, compute_wheel_distances_direction

# =============================
#   CONFIGURATION DU ROBOT
# =============================

robot = Robot(
    pos=np.array([0.0, 0.0]),
    angle=np.pi/2,  # Robot orienté vers le haut (90°)
    radius=150,     # Rayon du robot en mm
    wheel_angles=[np.radians(120), np.radians(240), np.radians(0)]
)

print("=" * 70)
print("CALCUL DES DISTANCES DE ROUES POUR DÉPLACEMENTS")
print("=" * 70)
print(f"\nConfiguration du robot :")
print(f"  - Position : {robot.pos}")
print(f"  - Orientation : {math.degrees(robot.angle):.1f}°")
print(f"  - Rayon : {robot.radius} mm")
print(f"  - Angles des roues : {[math.degrees(a) for a in robot.wheel_angles]}°")

# =============================
#   EXEMPLE 1 : DÉPLACEMENT EN X/Y GLOBAL
# =============================

print("\n" + "=" * 70)
print("EXEMPLE 1 : Déplacement avec dx, dy dans le repère global")
print("=" * 70)

# Avancer de 100mm en x global (vers la droite)
print("\n📍 Cas 1 : Avancer de 100mm en X (vers la droite)")
distances = compute_wheel_distances_global(robot, dx=100, dy=0)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")
print(f"  Roue 1 (120°) : {distances[0]:.2f} mm")
print(f"  Roue 2 (240°) : {distances[1]:.2f} mm")
print(f"  Roue 3 (0°)   : {distances[2]:.2f} mm")

# Avancer de 100mm en y global (vers le haut)
print("\n📍 Cas 2 : Avancer de 100mm en Y (vers le haut)")
distances = compute_wheel_distances_global(robot, dx=0, dy=100)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")
print(f"  Roue 1 (120°) : {distances[0]:.2f} mm")
print(f"  Roue 2 (240°) : {distances[1]:.2f} mm")
print(f"  Roue 3 (0°)   : {distances[2]:.2f} mm")

# Déplacement diagonal
print("\n📍 Cas 3 : Déplacement diagonal (100mm en X, 100mm en Y)")
distances = compute_wheel_distances_global(robot, dx=100, dy=100)
distance_totale = math.sqrt(100**2 + 100**2)
print(f"  Distance totale : {distance_totale:.2f} mm")
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")
print(f"  Roue 1 (120°) : {distances[0]:.2f} mm")
print(f"  Roue 2 (240°) : {distances[1]:.2f} mm")
print(f"  Roue 3 (0°)   : {distances[2]:.2f} mm")

# Rotation pure
print("\n📍 Cas 4 : Rotation pure de 90° (pi/2 radians)")
dtheta = np.pi/2
distances = compute_wheel_distances_global(robot, dx=0, dy=0, dtheta=dtheta)
print(f"  Rotation : {math.degrees(dtheta):.1f}°")
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")
print(f"  Roue 1 (120°) : {distances[0]:.2f} mm")
print(f"  Roue 2 (240°) : {distances[1]:.2f} mm")
print(f"  Roue 3 (0°)   : {distances[2]:.2f} mm")

# Translation + rotation
print("\n📍 Cas 5 : Translation (100mm en X) + Rotation (45°)")
dtheta = np.pi/4
distances = compute_wheel_distances_global(robot, dx=100, dy=0, dtheta=dtheta)
print(f"  Translation : 100mm en X")
print(f"  Rotation : {math.degrees(dtheta):.1f}°")
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")
print(f"  Roue 1 (120°) : {distances[0]:.2f} mm")
print(f"  Roue 2 (240°) : {distances[1]:.2f} mm")
print(f"  Roue 3 (0°)   : {distances[2]:.2f} mm")

# =============================
#   EXEMPLE 2 : DÉPLACEMENT PAR DIRECTION
# =============================

print("\n" + "=" * 70)
print("EXEMPLE 2 : Déplacement avec distance et angle de direction")
print("=" * 70)

# Nord (0°)
print("\n📍 Cas 1 : 100mm vers le NORD (90°)")
distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=np.pi/2)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# Est (0°)
print("\n📍 Cas 2 : 100mm vers l'EST (0°)")
distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=0)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# Sud (270°)
print("\n📍 Cas 3 : 100mm vers le SUD (270°)")
distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=-np.pi/2)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# Ouest (180°)
print("\n📍 Cas 4 : 100mm vers l'OUEST (180°)")
distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=np.pi)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# Nord-Est (45°)
print("\n📍 Cas 5 : 100mm vers le NORD-EST (45°)")
distances = compute_wheel_distances_direction(robot, distance=100, direction_angle=np.pi/4)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# Nord-Est avec rotation
print("\n📍 Cas 6 : 100mm vers le NORD-EST (45°) + Rotation de 30°")
distances = compute_wheel_distances_direction(
    robot, 
    distance=100, 
    direction_angle=np.pi/4, 
    dtheta=np.radians(30)
)
print(f"  Distances des roues : {[f'{d:.2f}' for d in distances]} mm")

# =============================
#   EXEMPLE 3 : TRAJECTOIRE CARRÉE
# =============================

print("\n" + "=" * 70)
print("EXEMPLE 3 : Trajectoire carrée de 200mm de côté")
print("=" * 70)

cote = 200  # mm
directions = [
    (0, "EST"),
    (np.pi/2, "NORD"),
    (np.pi, "OUEST"),
    (-np.pi/2, "SUD")
]

print(f"\nCôté du carré : {cote} mm\n")

for i, (angle, nom) in enumerate(directions):
    distances = compute_wheel_distances_direction(robot, distance=cote, direction_angle=angle)
    print(f"Côté {i+1} - Direction {nom} ({math.degrees(angle):.0f}°):")
    print(f"  Roue 1 : {distances[0]:7.2f} mm")
    print(f"  Roue 2 : {distances[1]:7.2f} mm")
    print(f"  Roue 3 : {distances[2]:7.2f} mm")
    print()

# =============================
#   EXEMPLE 4 : UTILISATION AVEC MOTOR
# =============================

print("=" * 70)
print("EXEMPLE 4 : Intégration avec Motor.move_distance()")
print("=" * 70)

print("""
Pour utiliser ces distances avec la classe Motor, voici un exemple :

```python
from motor_controller import Motor, MotorGroup

# Déplacement de 100mm vers le nord
distances = compute_wheel_distances_direction(
    robot, 
    distance=100, 
    direction_angle=np.pi/2
)

# Appliquer à chaque moteur (en supposant 3 moteurs)
for i, (motor, distance) in enumerate(zip([motor1, motor2, motor3], distances)):
    print(f"Moteur {i+1} : {distance:.2f} mm")
    motor.move_distance(
        distance_mm=distance,
        speed_mm_s=50,
        acceleration=150
    )

# Attendre que tous les moteurs s'arrêtent
for motor in [motor1, motor2, motor3]:
    motor.wait_until_stopped()
```

IMPORTANT : 
- Les distances peuvent être négatives (mouvement en arrière)
- Le signe de la distance gère automatiquement la direction du moteur
- Pour un mouvement synchronisé, tous les moteurs doivent démarrer en même temps
""")

# =============================
#   EXEMPLE 5 : VÉRIFICATION
# =============================

print("=" * 70)
print("EXEMPLE 5 : Vérification de cohérence")
print("=" * 70)

# Test : un déplacement de 100mm à 45° doit donner dx=dy≈70.7mm
distance = 100
angle = np.pi/4
dx_attendu = distance * math.cos(angle)
dy_attendu = distance * math.sin(angle)

distances_1 = compute_wheel_distances_direction(robot, distance=distance, direction_angle=angle)
distances_2 = compute_wheel_distances_global(robot, dx=dx_attendu, dy=dy_attendu)

print(f"\nTest : 100mm à 45°")
print(f"  dx attendu : {dx_attendu:.2f} mm")
print(f"  dy attendu : {dy_attendu:.2f} mm")
print(f"\nMéthode 1 (distance + angle) :")
print(f"  Roues : {[f'{d:.2f}' for d in distances_1]} mm")
print(f"\nMéthode 2 (dx + dy) :")
print(f"  Roues : {[f'{d:.2f}' for d in distances_2]} mm")
print(f"\nDifférence :")
diff = [abs(d1 - d2) for d1, d2 in zip(distances_1, distances_2)]
print(f"  {[f'{d:.6f}' for d in diff]} mm")
print(f"  ✅ Les deux méthodes donnent le même résultat" if max(diff) < 1e-10 else "  ❌ Erreur")

print("\n" + "=" * 70)
print("✅ EXEMPLES TERMINÉS")
print("=" * 70)

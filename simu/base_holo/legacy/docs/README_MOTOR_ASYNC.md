# Contrôle Asynchrone des Moteurs

## Vue d'ensemble

La classe `Motor` dispose désormais d'un mode de contrôle asynchrone qui utilise un **thread daemon** pour gérer automatiquement les changements de vitesse et de direction.

## Fonctionnalités

### 1. Thread Daemon Automatique

Le thread daemon surveille en continu l'état du moteur et applique automatiquement :
- Les changements de vitesse
- Les changements de direction (avec arrêt intermédiaire)

### 2. États de Contrôle

Le moteur peut être dans 3 états :
- **État 1 (Idle)** : Attente de changement
- **État 2 (Speed Change)** : Changement de vitesse en cours
- **État 3 (Direction Change)** : Changement de direction en cours

### 3. Gestion Intelligente des Transitions

- **Changement de vitesse** : Appliqué directement si la direction reste la même
- **Changement de direction** : Arrêt complet → Attente → Relance dans la nouvelle direction
- **Priorité** : Les changements de direction ont priorité sur les changements de vitesse

## Utilisation

### Mode Synchrone (ancien comportement)

```python
from motor_controller import Motor
import can

bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)

# Création en mode synchrone (par défaut)
motor = Motor(bus, can_id=1, enable_async_control=False)

# Contrôle manuel
motor.move_velocity(speed=1000, direction=0, acceleration=150)
motor.stop()
```

### Mode Asynchrone (nouveau)

```python
from motor_controller import Motor
from mks_servo_can import MksServo
import can

bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
servo = MksServo(bus, notifier, 1)

# Création en mode asynchrone
motor = Motor(bus, can_id=1, mks_servo=servo, enable_async_control=True)

# Définir l'accélération par défaut
motor.state.acceleration = 150

# Définir une vitesse cible (le thread daemon applique le changement)
motor.set_target_velocity(speed=1000, direction=0)

# Changer de vitesse (transition automatique)
motor.set_target_velocity(speed=1500, direction=0)

# Changer de direction (arrêt automatique puis relance)
motor.set_target_velocity(speed=1000, direction=1)

# Arrêt
motor.set_target_velocity(speed=0, direction=0)
```

## Exemple Complet : Contrôle de 3 Moteurs

```python
import time
from motor_controller import Motor
from mks_servo_can import MksServo
import can

# Initialisation CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])

# Création des servos
servos = [MksServo(bus, notifier, i) for i in range(1, 4)]

# Création des moteurs en mode asynchrone
motors = [
    Motor(bus, can_id=i, mks_servo=servos[i-1], enable_async_control=True)
    for i in range(1, 4)
]

# Configuration de l'accélération
for motor in motors:
    motor.state.acceleration = 150

# Trajectoire carrée
trajectories = [
    [800, 800, 800],    # Avant
    [800, -800, 0],     # Rotation droite
    [800, 800, 800],    # Avant
    [-800, 800, 0],     # Rotation gauche
]

try:
    for wheel_speeds in trajectories:
        for motor, speed in zip(motors, wheel_speeds):
            direction = 0 if speed >= 0 else 1
            motor.set_target_velocity(speed=abs(speed), direction=direction)
        time.sleep(3)  # Le thread daemon gère tout automatiquement
    
    # Arrêt
    for motor in motors:
        motor.set_target_velocity(speed=0, direction=0)
    time.sleep(1)

finally:
    # Nettoyage
    for motor in motors:
        motor.stop_async_control()
```

## Avantages du Mode Asynchrone

1. **Simplicité** : Plus besoin de gérer manuellement les états
2. **Robustesse** : Gestion automatique des transitions délicates (changement de direction)
3. **Performance** : Thread daemon ne bloque pas le programme principal
4. **Sécurité** : Arrêt automatique lors du changement de direction

## Structure Interne

### MotorState (amélioré)

```python
@dataclass
class MotorState:
    position: int = 0
    speed: int = 0                    # Vitesse actuelle
    target_speed: int = 0             # Vitesse cible (pour async)
    target_direction: int = 0         # Direction cible (pour async)
    direction: int = 0                # Direction actuelle
    acceleration: int = 100
    is_running: bool = False
    last_update: float = 0.0
    control_state: int = 1            # État de la machine d'états
```

### Méthodes Principales

- `start_async_control()` : Démarre le thread daemon
- `stop_async_control()` : Arrête le thread daemon
- `set_target_velocity(speed, direction, acceleration)` : Définit la consigne
- `_control_loop()` : Boucle principale du thread (interne)
- `_apply_velocity_change()` : Applique un changement de vitesse (interne)
- `_apply_direction_change()` : Applique un changement de direction (interne)

## Migration depuis l'Ancien Code

### Avant (test_base_holo.py)

```python
@dataclass
class Motor:
    speed: int
    lspeed: int
    accel: int
    direction: bool
    ldirection: bool
    state: int = 1

Motors = [Motor(0, 0, 0, 0, 0, 1) for _ in range(3)]

def control_motor(motor_id, stop_event):
    while not stop_event.is_set():
        # Logique de contrôle manuelle...
```

### Après (motor_controller.py)

```python
# Tout est intégré dans la classe Motor
motor = Motor(bus, can_id=1, enable_async_control=True)
motor.set_target_velocity(speed=1000, direction=0)
# Le thread daemon gère automatiquement les transitions
```

## Notes Importantes

1. **Thread Daemon** : Se termine automatiquement à la fin du programme
2. **MksServo Recommandé** : Pour `is_motor_running()` lors des changements de direction
3. **Accélération** : Définie via `motor.state.acceleration` (0-255)
4. **Nettoyage** : Appeler `motor.stop_async_control()` en fin de programme (optionnel)

## Compatibilité

- ✅ Compatible avec le mode synchrone existant
- ✅ Peut mélanger moteurs async et sync
- ✅ Pas de breaking changes pour le code existant

# Carte de Contrôle - Refactorisation

## Vue d'ensemble

Refactorisation de `init.py` avec une architecture modulaire facilitant l'ajout de threads et de sockets UNIX non-bloquants.

## Architecture

### Classes principales

#### `ControlBoard`
- **Rôle** : Encapsulation de l'accès matériel MCP23017
- **Fonctionnalités** :
  - Initialisation I2C avec gestion d'erreurs
  - Mode simulation (mocks) pour développement sans matériel
  - Helpers LED : `set_color_lidar()`, `set_color_state()`, `set_state_ok()`, etc.
  - Buzzer : `beep(duration_ms)`
  - Gestion état équipe avec feedback LED

#### `InputPoller` (Thread)
- **Rôle** : Polling des entrées GPIO avec debounce et callbacks
- **Fonctionnalités** :
  - Détection correcte rising/falling/change
  - Anti-rebond configurable (30ms par défaut)
  - Enregistrement de callbacks par entrée
  - Arrêt propre via `stop_event`
  - Gestion d'exceptions dans les callbacks

#### `UnixSocketThread` (Thread template)
- **Rôle** : Classe de base pour gérer des sockets UNIX
- **Fonctionnalités** :
  - Connexion non-bloquante avec `setblocking(False)`
  - Utilisation de `select()` pour polling efficace
  - Méthode `handle_data()` à surcharger
  - Logging détaillé et gestion d'erreurs

## Corrections apportées

### Bugs fixes
1. **Thread signature** : `read_callback` accepte maintenant `stop_event` correctement
2. **Edge detection** : 
   - Rising : `not last and current`
   - Falling : `last and not current`
3. **Callbacks** : Signature unifiée `(name: str, value: bool)`
4. **Polling loop** : Ajout de `time.sleep()` pour éviter 100% CPU
5. **State init** : `team` initialisé depuis `sw_team.value` au démarrage

### Améliorations
1. **Debounce** : Anti-rebond logiciel (30ms) pour éviter déclenchements multiples
2. **Logging** : `logging` avec horodatage au lieu de `print()`
3. **Exception handling** : Try/except autour de l'init I2C et dans les callbacks
4. **Mode simulation** : Mock pins pour développement sur PC
5. **Type hints** : Annotations pour meilleure maintenabilité

## Utilisation

### Exécution simple

```python
python init_refactored.py
```

### Ajout d'un callback personnalisé

```python
def on_mon_bouton(name: str, value: bool):
    logger.info(f"Bouton {name} -> {value}")
    board.beep(50)

poller.register_callback('bp_init', on_mon_bouton)
```

### Ajout d'un socket UNIX

```python
class RobotSocketThread(UnixSocketThread):
    def handle_data(self, data: bytes):
        # Traitement spécifique
        vals = struct.unpack('18f', data)
        logger.info(f"Position: x={vals[0]}, y={vals[1]}")

# Dans main()
robot_socket = RobotSocketThread("/tmp/robot.sock", stop_event)
robot_socket.start()
```

### Ajout d'un thread custom

```python
class MonThread(threading.Thread):
    def __init__(self, board: ControlBoard, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.board = board
        self.stop_event = stop_event
    
    def run(self):
        while not self.stop_event.is_set():
            # Logique du thread
            self.board.set_state_ok()
            time.sleep(1)

# Dans main()
mon_thread = MonThread(board, stop_event)
mon_thread.start()
```

## Configuration

### Constantes ajustables

```python
POLL_INTERVAL_MS = 10  # Polling des entrées (ms)
DEBOUNCE_MS = 30       # Anti-rebond (ms)
```

### Niveau de logging

```python
logging.basicConfig(level=logging.DEBUG)  # Pour debug détaillé
```

## Migration depuis `init.py`

| Ancien code | Nouveau code |
|-------------|--------------|
| `set_team_feed_back()` | `board.set_team_feedback()` |
| `set_color_lidar(r, g, b)` | `board.set_color_lidar(r, g, b)` |
| `set_color_state(r, g, b)` | `board.set_color_state(r, g, b)` |
| `print("message")` | `logger.info("message")` |
| Global `team` | `board.team` |

## Exemples avancés

### Multi-sockets

```python
# Dans main()
socket1 = UnixSocketThread("/tmp/lidar.sock", stop_event)
socket2 = UnixSocketThread("/tmp/imu.sock", stop_event)
socket3 = UnixSocketThread("/tmp/vision.sock", stop_event)

socket1.start()
socket2.start()
socket3.start()
```

### LED patterns

```python
def set_led_pattern_error():
    board.set_state_error()
    board.beep(200)
    time.sleep(0.3)
    board.beep(200)

def set_led_pattern_ready():
    board.set_color_lidar(False, True, False)
    board.set_color_state(False, True, False)
    board.beep(100)
```

## Tests

En mode simulation (sans Raspberry Pi) :

```bash
python init_refactored.py
```

Sur Raspberry Pi avec matériel :

```bash
sudo python init_refactored.py
```

## Prochaines étapes

1. Implémenter les threads spécifiques (LIDAR, IMU, vision)
2. Créer des sous-classes `UnixSocketThread` pour chaque protocole
3. Ajouter une machine à états pour gérer les phases du match
4. Implémenter le séquenceur de stratégie

## Références

- Architecture inspirée de `pipe_rec.py` pour sockets UNIX
- Pattern observer pour callbacks
- Threading avec arrêt propre via `Event`

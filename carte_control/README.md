# Carte de Contrôle - Architecture Modulaire

## 📁 Structure du projet

```
carte_control/
├── __init__.py              # Package principal
├── config.py                # Configuration globale et constantes
├── control_board.py         # Classe ControlBoard (gestion MCP23017)
├── board_actuation.py       # Thread d'actuation LED et monitoring I2C
├── input_poller.py          # Thread de polling GPIO avec debounce
├── process_manager.py       # Gestionnaire de processus externes
├── unix_socket_thread.py    # Thread socket UNIX non-bloquant
├── callbacks.py             # Callbacks applicatifs pour les événements
├── main.py                  # Point d'entrée principal ⭐
├── init_refactored.py       # Ancien fichier monolithique (déprécié)
└── README.md                # Cette documentation
```

## 🚀 Utilisation

### Lancement du programme

```bash
# Sur Raspberry Pi
sudo python3 main.py

# En mode simulation (sans matériel)
python3 main.py
```

### Import comme module

```python
from carte_control import ControlBoard, ProcessManager, InputPoller

board = ControlBoard()
process_manager = ProcessManager()
```

## 📦 Modules

### `config.py`
- **Rôle** : Configuration globale
- **Contenu** : Constantes, setup logging, détection hardware
- **Variables** :
  - `POLL_INTERVAL_MS` : Intervalle de polling (10ms)
  - `DEBOUNCE_MS` : Anti-rebond (30ms)
  - `STATE_ERROR`, `STATE_WARNING`, `STATE_OK`
  - `HARDWARE_AVAILABLE` : Détection auto du matériel

### `control_board.py`
- **Rôle** : Gestion de la carte MCP23017
- **Classe** : `ControlBoard`
- **Méthodes principales** :
  - `check_i2c()` : Vérifie l'état du bus I2C
  - `set_color_lidar(r, g, b)` : LED LIDAR
  - `set_color_state(r, g, b)` : LED état
  - `set_state_ok()`, `set_state_error()`, `set_state_warning()`
  - `beep(duration_ms)` : Buzzer
  - `read_switch_selections()` : Lecture switches
  - `get_switch_selections()` : Récupère valeur switches (0-7)

### `board_actuation.py`
- **Rôle** : Thread d'actualisation LED et monitoring I2C
- **Classe** : `BoardActuationThread`
- **Fréquence** : 100 Hz (10ms)
- **Fonctionnalités** :
  - Actualise les LED (team, state, lidar)
  - Monitore le bus I2C (10 Hz)
  - Détection par seuil d'erreurs consécutives
  - Set `stop_event` si I2C down

### `input_poller.py`
- **Rôle** : Thread de polling GPIO
- **Classe** : `InputPoller`
- **Fonctionnalités** :
  - Polling 100 Hz avec debounce 30ms
  - Détection rising/falling/change
  - Callbacks configurables
  - Gestion d'exceptions dans callbacks

### `process_manager.py`
- **Rôle** : Gestion des processus externes
- **Classe** : `ProcessManager`
- **Méthodes** :
  - `start_python(script, args, name)` : Lance script Python
  - `start_c_program(binary, args, name)` : Lance exécutable C
  - `stop_process(proc)` : Arrête un processus
  - `stop_by_name(name)` : Arrête par nom
  - `restart_process(proc)` : Redémarre
  - `list_processes()` : Liste tous les processus
  - `stop_all()` : Arrête tout (SIGTERM → SIGKILL)

### `unix_socket_thread.py`
- **Rôle** : Thread socket UNIX non-bloquant
- **Classe** : `UnixSocketThread`
- **Usage** : Classe de base à hériter
- **Méthode à surcharger** : `handle_data(data)`
- **Exemple** :
  ```python
  class LidarSocket(UnixSocketThread):
      def handle_data(self, data):
          # Traiter les données du LIDAR
          pass
  ```

### `callbacks.py`
- **Rôle** : Callbacks pour événements GPIO
- **Fonctions** :
  - `on_tirette_falling()` : Tirette retirée
  - `on_bp_rst_lidar_rising()` : Reset LIDAR
  - `on_bp_init_rising()` : Initialisation
  - `on_bp_rst_state_rising()` : Reset état
  - `on_sw_team_change(board)` : Changement équipe

### `main.py`
- **Rôle** : Point d'entrée principal
- **Fonctionnalités** :
  - Initialisation de tous les composants
  - Démarrage des threads
  - Gestion du cycle de vie
  - Arrêt propre (`clean_stop()`)
  - Redémarrage auto si I2C down

## 🔧 Exemples d'usage

### Démarrer des processus externes

```python
# Dans main.py, après création du process_manager

# Programme Python
lidar = process_manager.start_python(
    '/home/pi/lidar/main.py',
    name='LIDAR'
)

# Programme C
imu = process_manager.start_c_program(
    '/home/pi/imu/imu_server',
    args=['--port', '5000'],
    name='IMU'
)

# Arrêt individuel
process_manager.stop_by_name('LIDAR')

# Liste des processus
for proc in process_manager.list_processes():
    print(f"{proc['name']}: {'Running' if proc['running'] else 'Stopped'}")
```

### Ajouter un socket UNIX

```python
# Créer une classe spécialisée
class LidarSocket(UnixSocketThread):
    def handle_data(self, data: bytes):
        import struct
        vals = struct.unpack('18f', data)
        logger.info(f"LIDAR pos: x={vals[0]}, y={vals[1]}")

# Dans main()
lidar_socket = LidarSocket("/tmp/lidar.sock", stop_event)
lidar_socket.start()

# N'oubliez pas de join dans clean_stop()
# lidar_socket.join(timeout=2.0)
```

### Ajouter un callback personnalisé

```python
# Dans callbacks.py
def on_custom_button(name: str, value: bool):
    logger.info(f"Custom button pressed!")

# Dans main()
poller.register_callback('bp_spare_1', on_custom_button)

# Ajouter l'input dans input_poller.py
self.inputs = {
    ...
    'bp_spare_1': (self.board.bp_spare_1, 'rising'),
}
```

## 🎯 Avantages de la structure modulaire

✅ **Séparation des responsabilités** : Chaque classe a un rôle clair  
✅ **Réutilisabilité** : Import facile des composants  
✅ **Testabilité** : Chaque module peut être testé séparément  
✅ **Maintenabilité** : Modifications localisées  
✅ **Lisibilité** : Code organisé et documenté  
✅ **Extensibilité** : Facile d'ajouter de nouvelles fonctionnalités  

## 🔄 Migration depuis `init_refactored.py`

L'ancien fichier monolithique `init_refactored.py` reste présent pour référence mais est maintenant **déprécié**. Utilisez `main.py` à la place.

Les modifications nécessaires :
- Remplacer `python init_refactored.py` → `python main.py`
- Les imports sont maintenant explicites et modulaires
- Chaque classe est dans son propre fichier

## 📝 Notes de développement

- **Mode simulation** : Détecté automatiquement si pas de matériel
- **Logs** : Format avec timestamp, configurable dans `config.py`
- **I2C monitoring** : Check tous les 50ms (5 cycles × 10ms)
- **Arrêt propre** : SIGTERM avec fallback SIGKILL après timeout
- **Redémarrage auto** : En cas de perte I2C

## 🐛 Debugging

Pour activer les logs de debug :

```python
# Dans config.py
logger = setup_logging(level=logging.DEBUG)
```

## 📚 Dépendances

- Python 3.7+
- `adafruit-circuitpython-mcp230xx` (si matériel présent)
- `board`, `busio`, `digitalio` (CircuitPython)

## 🚀 Prochaines étapes

1. Implémenter les threads spécifiques (LIDAR, IMU, vision)
2. Créer des sous-classes `UnixSocketThread` pour chaque protocole
3. Ajouter une machine à états pour les phases du match
4. Implémenter le séquenceur de stratégie

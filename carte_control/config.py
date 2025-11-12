"""
Configuration globale pour la carte de contrôle.
"""

import logging

# ================================
# CONFIGURATION
# ================================

POLL_INTERVAL_MS = 10  # Intervalle de polling des entrées (ms)
DEBOUNCE_MS = 30       # Délai anti-rebond (ms)

# États LED
STATE_ERROR = 0
STATE_WARNING = 1
STATE_OK = 2

# ================================
# PROGRAMMES EXTERNES
# ================================

LIDAR_EXECUTABLE = '/home/raspi/Desktop/CDF/lidar/rplidar_sdk-master/rplidar_sdk-master/output/Linux/Release/ultra_simple'

# Programmes Python de stratégie (simulateur pour le moment)
STRATEGY_EXECUTABLE = '/home/raspi/Desktop/CDF/carte_control/simu_strat/simu_strategy.py'

# ================================
# SOCKETS UNIX
# ================================

# Socket pour l'état du LIDAR (LIDAR → Python)
LIDAR_STATE_SOCKET = '/tmp/robot.sock'

# Socket broadcast pour les données LIDAR (Python → STRATEGY, PROG_1, PROG_2, etc.)
LIDAR_DATA_SOCKET = '/tmp/lidar_data.sock'

# Programmes sélectionnables via sw_sel (0-7)
# Index correspond à la valeur de sw_sel (3 bits = 8 combinaisons)
SELECTABLE_PROGRAMS = {
    0: None,  # Aucun programme
    1: {'type': 'python', 'path': '/home/raspi/Desktop/CDF/carte_control/simu_prog/simuprog_sel_1.py', 'name': 'PROG_1'},
    2: {'type': 'python', 'path': '/home/raspi/Desktop/CDF/carte_control/simu_prog/simuprog_sel_2.py', 'name': 'PROG_2'},
    3: {'type': 'python', 'path': '/home/raspi/Desktop/CDF/carte_control/simu_prog/simuprog_sel_3.py', 'name': 'PROG_3'},
    4: {'type': 'python', 'path': '/home/raspi/Desktop/CDF/carte_control/simu_prog/simuprog_sel_4.py', 'name': 'PROG_4'},
    5: None,
    6: None,
    7: None,
}

# ================================
# HARDWARE AVAILABILITY
# ================================

try:
    import board
    import busio
    from digitalio import Direction, Pull
    from adafruit_mcp230xx.mcp23017 import MCP23017
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    Direction = None
    Pull = None
    MCP23017 = None
    logging.warning("Hardware libraries not available - running in simulation mode")

# ================================
# LOGGING
# ================================

def setup_logging(level=logging.INFO):
    """Configure le système de logging."""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)

logger = setup_logging()

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

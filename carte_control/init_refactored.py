# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Carte de contrôle avec MCP23017 - Architecture modulaire pour multi-threading et sockets UNIX.
"""

import time
import threading
import logging
import socket
import select
from typing import Callable, Optional, Dict
from dataclasses import dataclass

try:
    import board
    import busio
    from digitalio import Direction, Pull
    from adafruit_mcp230xx.mcp23017 import MCP23017
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    logging.warning("Hardware libraries not available - running in simulation mode")

# ================================
# CONFIGURATION
# ================================

POLL_INTERVAL_MS = 10  # Intervalle de polling des entrées (ms)
DEBOUNCE_MS = 30       # Délai anti-rebond (ms)

# ================================
# LOGGING
# ================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ================================
# CONTROL BOARD CLASS
# ================================

class ControlBoard:
    """Gère les entrées/sorties via MCP23017 et fournit des helpers pour LED, buzzer, etc."""
    
    def __init__(self, simulate: bool = False):
        self.simulate = simulate or not HARDWARE_AVAILABLE
        self.team = False
        self._init_hardware()
        
    def _init_hardware(self):
        """Initialise I2C et MCP23017."""
        if self.simulate:
            logger.info("Mode simulation - pas d'accès matériel")
            self._create_mock_pins()
            return
            
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.mcp1 = MCP23017(i2c, address=0x20)
            self.mcp2 = MCP23017(i2c, address=0x21)
            self._configure_pins()
            logger.info("MCP23017 initialisés avec succès")
        except Exception as e:
            logger.error(f"Erreur initialisation I2C/MCP23017: {e}")
            raise
    
    def _create_mock_pins(self):
        """Crée des pins simulés pour développement."""
        @dataclass
        class MockPin:
            value: bool = False
            direction: str = "INPUT"
            pull: str = "DOWN"
        
        # MCP1 outputs (Port A)
        self.team_rgb_b = MockPin()
        self.team_rgb_r = MockPin()
        self.lidar_rgb_g = MockPin()
        self.lidar_rgb_b = MockPin()
        self.lidar_rgb_r = MockPin()
        self.state_rgb_b = MockPin()
        self.state_rgb_g = MockPin()
        self.state_rgb_r = MockPin()
        
        # MCP1 inputs (Port B)
        self.sw_spare_1 = MockPin()
        self.sw_spare_2 = MockPin()
        self.tirette = MockPin()
        self.bp_rst_lidar = MockPin()
        self.bp_spare_1 = MockPin()
        self.bp_spare_2 = MockPin()
        self.bp_init = MockPin()
        self.bp_rst_state = MockPin()
        
        # MCP2
        self.sw_team = MockPin()
        self.buzzer = MockPin()
        self.sw_sel_1 = MockPin()
        self.sw_sel_2 = MockPin()
        self.sw_sel_3 = MockPin()
    
    def _configure_pins(self):
        """Configure les pins MCP23017 réels."""
        # MCP1 Port A : sorties RGB
        self.team_rgb_b = self.mcp1.get_pin(0)
        self.team_rgb_r = self.mcp1.get_pin(1)
        self.lidar_rgb_g = self.mcp1.get_pin(2)
        self.lidar_rgb_b = self.mcp1.get_pin(3)
        self.lidar_rgb_r = self.mcp1.get_pin(4)
        self.state_rgb_b = self.mcp1.get_pin(5)
        self.state_rgb_g = self.mcp1.get_pin(6)
        self.state_rgb_r = self.mcp1.get_pin(7)
        
        output_pins = [
            self.team_rgb_b, self.team_rgb_r,
            self.lidar_rgb_g, self.lidar_rgb_b, self.lidar_rgb_r,
            self.state_rgb_b, self.state_rgb_g, self.state_rgb_r
        ]
        for pin in output_pins:
            pin.direction = Direction.OUTPUT
        
        # MCP1 Port B : entrées
        self.sw_spare_1 = self.mcp1.get_pin(8)
        self.sw_spare_2 = self.mcp1.get_pin(9)
        self.tirette = self.mcp1.get_pin(10)
        self.bp_rst_lidar = self.mcp1.get_pin(11)
        self.bp_spare_1 = self.mcp1.get_pin(12)
        self.bp_spare_2 = self.mcp1.get_pin(13)
        self.bp_init = self.mcp1.get_pin(14)
        self.bp_rst_state = self.mcp1.get_pin(15)
        
        input_pins = [
            self.sw_spare_1, self.sw_spare_2, self.tirette, self.bp_rst_lidar,
            self.bp_spare_1, self.bp_spare_2, self.bp_init, self.bp_rst_state
        ]
        for pin in input_pins:
            pin.direction = Direction.INPUT
            pin.pull = Pull.DOWN
        
        # MCP2
        self.sw_team = self.mcp2.get_pin(0)
        self.buzzer = self.mcp2.get_pin(1)
        self.sw_sel_1 = self.mcp2.get_pin(2)
        self.sw_sel_2 = self.mcp2.get_pin(3)
        self.sw_sel_3 = self.mcp2.get_pin(4)
        
        self.buzzer.direction = Direction.OUTPUT
        for pin in [self.sw_team, self.sw_sel_1, self.sw_sel_2, self.sw_sel_3]:
            pin.direction = Direction.INPUT
            pin.pull = Pull.DOWN
    
    # ================================
    # LED HELPERS
    # ================================
    
    def set_team_feedback(self):
        """Met à jour les LED équipe selon l'état actuel."""
        if self.team:
            self.team_rgb_b.value = False
            self.team_rgb_r.value = True
        else:
            self.team_rgb_b.value = True
            self.team_rgb_r.value = False
    
    def set_color_lidar(self, red: bool, green: bool, blue: bool):
        """Configure la couleur LED lidar."""
        self.lidar_rgb_r.value = red
        self.lidar_rgb_g.value = green
        self.lidar_rgb_b.value = blue
    
    def set_color_state(self, red: bool, green: bool, blue: bool):
        """Configure la couleur LED état."""
        self.state_rgb_r.value = red
        self.state_rgb_g.value = green
        self.state_rgb_b.value = blue
    
    def set_state_ok(self):
        """LED état = vert."""
        self.set_color_state(False, True, False)
    
    def set_state_error(self):
        """LED état = rouge."""
        self.set_color_state(True, False, False)
    
    def set_state_warning(self):
        """LED état = orange."""
        self.set_color_state(True, True, False)
    
    def beep(self, duration_ms: int = 100):
        """Active le buzzer pendant duration_ms."""
        self.buzzer.value = True
        time.sleep(duration_ms / 1000.0)
        self.buzzer.value = False

# ================================
# INPUT POLLING THREAD
# ================================

class InputPoller(threading.Thread):
    """Thread qui poll les entrées GPIO avec debounce et callbacks."""
    
    def __init__(self, board: ControlBoard, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.board = board
        self.stop_event = stop_event
        self.callbacks: Dict[str, Callable] = {}
        self.last_state: Dict[str, bool] = {}
        self.last_change_ts: Dict[str, float] = {}
        
        # Inputs à surveiller (nom, pin, type: 'rising', 'falling', 'change')
        self.inputs = {
            'sw_team': (self.board.sw_team, 'change'),
            'bp_init': (self.board.bp_init, 'rising'),
            'bp_rst_state': (self.board.bp_rst_state, 'rising'),
            'bp_rst_lidar': (self.board.bp_rst_lidar, 'rising'),
            'tirette': (self.board.tirette, 'falling'),
        }
        
        # Init last state
        for name, (pin, _) in self.inputs.items():
            self.last_state[name] = pin.value
            self.last_change_ts[name] = 0.0
    
    def register_callback(self, input_name: str, callback: Callable):
        """Enregistre un callback pour une entrée donnée."""
        self.callbacks[input_name] = callback
    
    def run(self):
        """Boucle de polling."""
        logger.info("InputPoller démarré")
        poll_interval = POLL_INTERVAL_MS / 1000.0
        
        while not self.stop_event.is_set():
            now = time.time()
            
            for name, (pin, edge_type) in self.inputs.items():
                current = pin.value
                last = self.last_state[name]
                
                # Détection de changement
                if current != last:
                    # Debounce
                    if (now - self.last_change_ts[name]) < (DEBOUNCE_MS / 1000.0):
                        continue
                    
                    self.last_change_ts[name] = now
                    self.last_state[name] = current
                    
                    # Déclenchement callback selon le type
                    should_trigger = False
                    if edge_type == 'rising' and not last and current:
                        should_trigger = True
                    elif edge_type == 'falling' and last and not current:
                        should_trigger = True
                    elif edge_type == 'change':
                        should_trigger = True
                    
                    if should_trigger and name in self.callbacks:
                        try:
                            self.callbacks[name](name, current)
                        except Exception as e:
                            logger.error(f"Erreur callback {name}: {e}")
            
            time.sleep(poll_interval)
        
        logger.info("InputPoller arrêté")

# ================================
# SOCKET UNIX THREAD (TEMPLATE)
# ================================

class UnixSocketThread(threading.Thread):
    """Thread générique pour gérer un socket UNIX non-bloquant."""
    
    def __init__(self, socket_path: str, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.socket_path = socket_path
        self.stop_event = stop_event
        self.sock: Optional[socket.socket] = None
    
    def connect(self):
        """Établit la connexion au socket UNIX."""
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(self.socket_path)
            self.sock.setblocking(False)
            logger.info(f"Connecté au socket {self.socket_path}")
            return True
        except Exception as e:
            logger.error(f"Erreur connexion socket {self.socket_path}: {e}")
            return False
    
    def run(self):
        """Boucle principale du thread."""
        if not self.connect():
            return
        
        logger.info(f"UnixSocketThread démarré sur {self.socket_path}")
        
        while not self.stop_event.is_set():
            try:
                # Attente non-bloquante de données
                rlist, _, _ = select.select([self.sock], [], [], 0.05)
                
                if rlist:
                    data = self.sock.recv(4096)
                    if not data:
                        logger.warning("Socket fermé par le serveur")
                        break
                    self.handle_data(data)
                    
            except Exception as e:
                logger.error(f"Erreur dans boucle socket: {e}")
                break
        
        if self.sock:
            self.sock.close()
        logger.info(f"UnixSocketThread arrêté")
    
    def handle_data(self, data: bytes):
        """À surcharger pour traiter les données reçues."""
        logger.debug(f"Reçu {len(data)} octets")

# ================================
# CALLBACKS APPLICATIFS
# ================================

def on_tirette_falling(name: str, value: bool):
    logger.info("🚀 Tirette retirée - démarrage match!")

def on_bp_rst_lidar_rising(name: str, value: bool):
    logger.info("🔄 Reset LIDAR demandé")

def on_bp_init_rising(name: str, value: bool):
    logger.info("⚙️  Initialisation demandée")

def on_bp_rst_state_rising(name: str, value: bool):
    logger.info("🔄 Reset état demandé")

def on_sw_team_change(name: str, value: bool, board: ControlBoard):
    board.team = value
    board.set_team_feedback()
    logger.info(f"🔵🔴 Équipe changée: {'ROUGE' if value else 'BLEU'}")

# ================================
# MAIN
# ================================

def main():
    """Point d'entrée principal."""
    logger.info("🤖 Démarrage carte de contrôle")
    
    # Initialisation
    board = ControlBoard(simulate=not HARDWARE_AVAILABLE)
    stop_event = threading.Event()
    
    # Init état team
    board.team = board.sw_team.value
    board.set_team_feedback()
    
    # Démarrage du poller d'entrées
    poller = InputPoller(board, stop_event)
    poller.register_callback('tirette', on_tirette_falling)
    poller.register_callback('bp_rst_lidar', on_bp_rst_lidar_rising)
    poller.register_callback('bp_init', on_bp_init_rising)
    poller.register_callback('bp_rst_state', on_bp_rst_state_rising)
    poller.register_callback('sw_team', lambda n, v: on_sw_team_change(n, v, board))
    poller.start()
    
    # Exemple: démarrage d'un thread socket (à adapter selon besoin)
    # socket_thread = UnixSocketThread("/tmp/robot.sock", stop_event)
    # socket_thread.start()
    
    board.set_state_ok()
    logger.info("✅ Système prêt")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("🛑 Interruption par l'utilisateur")
    finally:
        stop_event.set()
        poller.join(timeout=2.0)
        # socket_thread.join(timeout=2.0)
        logger.info("🔴 Programme terminé")

if __name__ == "__main__":
    main()

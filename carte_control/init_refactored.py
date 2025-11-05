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
import subprocess
import os
from typing import Callable, Optional, Dict, List
from dataclasses import dataclass
import digitalio
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
STATE_ERROR=0
STATE_WARNING=1
STATE_OK=2


class ControlBoard:
    """Gère les entrées/sorties via MCP23017 et fournit des helpers pour LED, buzzer, etc."""
    
    def __init__(self, simulate: bool = False):
        self.simulate = simulate or not HARDWARE_AVAILABLE
        self.team = False
        self._init_hardware()
        self.state_lidar = STATE_ERROR
        self.state_board = STATE_ERROR
        self.sw_sel=0
    def _init_hardware(self):
        """Initialise I2C et MCP23017."""
        if self.simulate:
            logger.info("Mode simulation - pas d'accès matériel")
            self._create_mock_pins()
            return
            
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.mcp1 = MCP23017(i2c, address=0x20)
            self.mcp2 = MCP23017(i2c, address=0x25)
            self._configure_pins()
            logger.info("MCP23017 initialisés avec succès")
        except Exception as e:
            logger.error(f"Erreur initialisation I2C/MCP23017: {e}")
            time.sleep(1)
            restart_program()
            raise

    def check_i2c(self, log_errors: bool = True) -> bool:
        """
        Vérifie que le bus I2C est fonctionnel en testant la communication avec les MCP23017.
        Retourne True si OK, False si le bus est down.
        
        Args:
            log_errors: Si True, log les erreurs. Si False, détection silencieuse.
        """
        if self.simulate:
            return True  # Mode simulation, pas de test I2C
        
        try:
            # Test sur MCP1 : lecture du registre GPIO (plus fiable qu'une simple propriété)
            _ = self.mcp1.gpio
            # Test sur MCP2
            _ = self.mcp2.gpio
            return True
        except OSError as e:
            # Erreur typique I2C : [Errno 121] Remote I/O error, [Errno 5] Input/output error
            if log_errors:
                logger.error(f"⚠️ BUS I2C DOWN (OSError): {e}")
            return False
        except RuntimeError as e:
            # Autre erreur de communication
            if log_errors:
                logger.error(f"⚠️ BUS I2C DOWN (RuntimeError): {e}")
            return False
        except Exception as e:
            # Catch-all pour autres erreurs inattendues
            if log_errors:
                logger.error(f"⚠️ BUS I2C DOWN (Exception): {e}")
            return False
    
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
        self.lidar_rgb_r = self.mcp1.get_pin(2)
        self.lidar_rgb_b = self.mcp1.get_pin(3)
        self.lidar_rgb_g = self.mcp1.get_pin(4)
        self.state_rgb_b = self.mcp1.get_pin(5)
        self.state_rgb_r = self.mcp1.get_pin(6)
        self.state_rgb_g = self.mcp1.get_pin(7)
        
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
        
        # MCP2
        self.sw_team = self.mcp2.get_pin(8)
        self.buzzer = self.mcp2.get_pin(9)
        self.sw_sel_1 = self.mcp2.get_pin(10)
        self.sw_sel_2 = self.mcp2.get_pin(11)
        self.sw_sel_3 = self.mcp2.get_pin(12)

        self.buzzer.direction = Direction.OUTPUT
        for pin in [self.sw_team, self.sw_sel_1, self.sw_sel_2, self.sw_sel_3]:
            pin.direction = Direction.INPUT
    
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

    def set_state(self):
        """Met à jour l'état de la carte."""
        
        if self.state_board == STATE_OK:
            self.set_state_ok()
        elif self.state_board == STATE_WARNING:
            self.set_state_warning()
        elif self.state_board == STATE_ERROR:
            self.set_state_error()
    
    def set_state_lidar(self):
        """Met à jour l'état du LIDAR."""
        
        if self.state_lidar == STATE_OK:
            self.set_color_lidar(False, True, False)
        elif self.state_lidar == STATE_WARNING:
            self.set_color_lidar(True, True, False)
        elif self.state_lidar == STATE_ERROR:
            self.set_color_lidar(True, False, False)

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

    def read_switch_selections(self) :
        """Lit la position des switches de sélection (3 bits)."""
        sel1 = self.sw_sel_1.value
        sel2 = self.sw_sel_2.value
        sel3 = self.sw_sel_3.value
        self.sw_sel = (sel3 << 2) | (sel2 << 1) | sel1

    def get_switch_selections(self) -> int:
        """Retourne la position des switches de sélection (3 bits)."""
        return self.sw_sel
    
# ================================
# board actuation THREAD
# ================================

class board_actuation_thread(threading.Thread):
    """Thread qui poll les entrées GPIO avec debounce et callbacks."""
    
    def __init__(self, board: ControlBoard, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.board = board
        self.stop_event = stop_event
        self.actuation_interval_ms = 10  # Intervalle d'actualisation (ms)
        self.i2c_check_interval = 5  # Vérifier I2C tous les 10 cycles (= 100ms à 10ms/cycle)
        self.cycle_count = 0
        self.i2c_error_count = 0
        self.i2c_error_threshold = 1  # 3 erreurs consécutives avant de considérer le bus down

    def run(self):
        """Boucle d'actualisation."""
        logger.info("board_actuation_thread démarré")
        
        while not self.stop_event.is_set():
            now = time.time()
            
            # Check I2C périodiquement (tous les i2c_check_interval cycles)
            if self.cycle_count % self.i2c_check_interval == 0:
                i2c_ok = self.board.check_i2c(log_errors=False)
                if not i2c_ok:
                    self.i2c_error_count += 1
                    if self.i2c_error_count >= self.i2c_error_threshold:
                        logger.error(f"⚠️ BUS I2C DOWN détecté ({self.i2c_error_count} erreurs consécutives)")
                        self.stop_event.set()  # Déclenche l'arrêt
                else:
                    self.i2c_error_count = 0  # Reset compteur si OK
            
            # Actuations normales
            try:
                self.board.set_team_feedback()
                self.board.set_state()
                self.board.read_switch_selections()
                self.board.set_state_lidar()
            except Exception as e:
                logger.error(f"Erreur dans actuation: {e}")
                self.i2c_error_count += 1
                if self.i2c_error_count >= self.i2c_error_threshold:
                    self.stop_event.set()
            
            self.cycle_count += 1
            time.sleep(self.actuation_interval_ms / 1000.0)
        
        logger.info("board_actuation_thread arrêté")


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
# PROCESS MANAGER
# ================================

class ProcessManager:
    """Gestionnaire de processus externes (Python, C, etc.)"""
    
    def __init__(self):
        self.processes: List[Dict] = []
    
    def start_process(self, command: List[str], name: Optional[str] = None, shell: bool = False) -> Optional[subprocess.Popen]:
        """
        Lance un processus externe.
        
        Args:
            command: Liste ['programme', 'arg1', 'arg2'] ou string si shell=True
            name: Nom pour identification (optionnel)
            shell: Si True, exécute via shell
        
        Returns:
            subprocess.Popen object ou None si erreur
        """
        try:
            proc = subprocess.Popen(
                command,
                shell=shell,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                # Sur Linux/RPi: crée un nouveau process group pour faciliter le kill
                preexec_fn=os.setsid if hasattr(os, 'setsid') and not shell else None
            )
            proc_name = name or (command[0] if isinstance(command, list) else str(command))
            self.processes.append({
                'process': proc,
                'name': proc_name,
                'command': command
            })
            logger.info(f"✅ Processus '{proc_name}' démarré (PID: {proc.pid})")
            return proc
        except Exception as e:
            logger.error(f"❌ Erreur démarrage processus: {e}")
            return None
    
    def start_python(self, script_path: str, args: Optional[List[str]] = None, name: Optional[str] = None) -> Optional[subprocess.Popen]:
        """Lance un script Python."""
        cmd = ['python3', script_path] + (args or [])
        return self.start_process(cmd, name or f"py:{os.path.basename(script_path)}")
    
    def start_c_program(self, binary_path: str, args: Optional[List[str]] = None, name: Optional[str] = None) -> Optional[subprocess.Popen]:
        """Lance un exécutable C compilé."""
        cmd = [binary_path] + (args or [])
        return self.start_process(cmd, name or f"c:{os.path.basename(binary_path)}")
    
    def is_running(self, proc: subprocess.Popen) -> bool:
        """Vérifie si un processus est encore actif."""
        return proc.poll() is None
    
    def stop_process(self, proc: subprocess.Popen, timeout: int = 5) -> bool:
        """
        Arrête un processus spécifique.
        
        Args:
            proc: L'objet Popen du processus à arrêter
            timeout: Temps d'attente avant kill forcé (secondes)
        
        Returns:
            True si arrêté avec succès, False sinon
        """
        # Trouver les infos du processus
        proc_info = None
        for p in self.processes:
            if p['process'] == proc:
                proc_info = p
                break
        
        if proc_info is None:
            logger.warning("⚠️ Processus non trouvé dans la liste gérée")
            return False
        
        name = proc_info['name']
        
        if proc.poll() is not None:
            logger.info(f"'{name}' déjà arrêté")
            self.processes.remove(proc_info)
            return True
        
        try:
            # 1. Tentative d'arrêt propre (SIGTERM)
            logger.info(f"🛑 Arrêt de '{name}' (PID: {proc.pid})")
            proc.terminate()
            
            # Attendre l'arrêt
            try:
                proc.wait(timeout=timeout)
                logger.info(f"✅ '{name}' arrêté proprement")
                self.processes.remove(proc_info)
                return True
            except subprocess.TimeoutExpired:
                # 2. Si toujours actif, force kill (SIGKILL)
                logger.warning(f"⚠️ '{name}' ne répond pas, force kill...")
                proc.kill()
                proc.wait()
                logger.info(f"✅ '{name}' tué (SIGKILL)")
                self.processes.remove(proc_info)
                return True
                
        except Exception as e:
            logger.error(f"❌ Erreur arrêt '{name}': {e}")
            return False
    
    def stop_by_name(self, name: str, timeout: int = 5) -> bool:
        """
        Arrête un processus par son nom.
        
        Args:
            name: Nom du processus à arrêter
            timeout: Temps d'attente avant kill forcé (secondes)
        
        Returns:
            True si arrêté avec succès, False si non trouvé
        """
        for proc_info in self.processes:
            if proc_info['name'] == name:
                return self.stop_process(proc_info['process'], timeout)
        
        logger.warning(f"⚠️ Processus '{name}' non trouvé")
        return False
    
    def restart_process(self, proc: subprocess.Popen, timeout: int = 5) -> Optional[subprocess.Popen]:
        """
        Redémarre un processus spécifique.
        
        Args:
            proc: L'objet Popen du processus à redémarrer
            timeout: Temps d'attente avant kill forcé (secondes)
        
        Returns:
            Nouveau processus Popen ou None si erreur
        """
        # Trouver les infos du processus
        proc_info = None
        for p in self.processes:
            if p['process'] == proc:
                proc_info = p
                break
        
        if proc_info is None:
            logger.warning("⚠️ Processus non trouvé dans la liste gérée")
            return None
        
        name = proc_info['name']
        command = proc_info['command']
        
        # Arrêter le processus
        logger.info(f"🔄 Redémarrage de '{name}'...")
        self.stop_process(proc, timeout)
        
        # Relancer avec la même commande
        time.sleep(0.5)  # Petit délai pour libérer les ressources
        return self.start_process(command, name=name)
    
    def get_process_info(self, proc: subprocess.Popen) -> Optional[Dict]:
        """Retourne les infos d'un processus."""
        for proc_info in self.processes:
            if proc_info['process'] == proc:
                return {
                    'name': proc_info['name'],
                    'pid': proc.pid,
                    'running': self.is_running(proc),
                    'command': proc_info['command']
                }
        return None
    
    def list_processes(self) -> List[Dict]:
        """Liste tous les processus gérés avec leur statut."""
        result = []
        for proc_info in self.processes:
            proc = proc_info['process']
            result.append({
                'name': proc_info['name'],
                'pid': proc.pid,
                'running': self.is_running(proc),
                'command': proc_info['command']
            })
        return result
    
    def stop_all(self, timeout: int = 5):
        """Arrête proprement tous les processus gérés."""
        if not self.processes:
            logger.info("Aucun processus à arrêter")
            return
        
        logger.info(f"🛑 Arrêt de {len(self.processes)} processus...")
        
        for proc_info in self.processes:
            proc = proc_info['process']
            name = proc_info['name']
            
            if proc.poll() is None:  # Processus encore actif
                try:
                    # 1. Tentative d'arrêt propre (SIGTERM)
                    logger.info(f"Envoi SIGTERM à '{name}' (PID: {proc.pid})")
                    proc.terminate()
                    
                    # Attendre l'arrêt
                    try:
                        proc.wait(timeout=timeout)
                        logger.info(f"✅ '{name}' arrêté proprement")
                    except subprocess.TimeoutExpired:
                        # 2. Si toujours actif, force kill (SIGKILL)
                        logger.warning(f"⚠️ '{name}' ne répond pas, force kill...")
                        proc.kill()
                        proc.wait()
                        logger.info(f"✅ '{name}' tué (SIGKILL)")
                        
                except Exception as e:
                    logger.error(f"❌ Erreur arrêt '{name}': {e}")
            else:
                logger.info(f"'{name}' déjà arrêté")
        
        self.processes.clear()
        logger.info("✅ Tous les processus arrêtés")

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
def restart_program():
    import os, sys
    print("🔁 I2C KO → redémarrage du programme…")
    os.execv(sys.executable, ['python'] + sys.argv)

def main():
    """Point d'entrée principal."""
    logger.info("🤖 Démarrage carte de contrôle")
    
    # Initialisation
    board = ControlBoard(simulate=not HARDWARE_AVAILABLE)
    stop_event = threading.Event()
    process_manager = ProcessManager()
    
    # Init état team
    board.team = board.sw_team.value
    board.set_team_feedback()
    
    actuator = board_actuation_thread(board, stop_event)
    actuator.start()

    # Démarrage du poller d'entrées
    poller = InputPoller(board, stop_event)
    poller.register_callback('tirette', on_tirette_falling)
    poller.register_callback('bp_rst_lidar', on_bp_rst_lidar_rising)
    poller.register_callback('bp_init', on_bp_init_rising)
    poller.register_callback('bp_rst_state', on_bp_rst_state_rising)
    poller.register_callback('sw_team', lambda n, v: on_sw_team_change(n, v, board))
    poller.start()

    # ======================================
    # EXEMPLE: Démarrage de programmes externes
    # ======================================
    
    # Programmes Python
    # lidar_proc = process_manager.start_python('/home/pi/lidar/main.py', name='LIDAR')
    # vision_proc = process_manager.start_python('/home/pi/vision/detect.py', args=['--mode', 'auto'], name='Vision')
    # process_manager.stop_by_name('Motors')
    # Programmes C compilés
    # imu_proc = process_manager.start_c_program('/home/pi/imu/imu_server', name='IMU')
    # motor_proc = process_manager.start_c_program('/home/pi/motors/motor_control', args=['--can', 'can0'], name='Motors')
    
    # Socket threads (pour communiquer avec les programmes ci-dessus)
    # lidar_socket = UnixSocketThread("/tmp/lidar.sock", stop_event)
    # lidar_socket.start()
    
    board.set_state_ok()
    logger.info("✅ Système prêt")

    def clean_stop():
        """Arrêt propre du système."""
        logger.info("🧹 Nettoyage en cours...")
        stop_event.set()
        
        # Arrêt des processus externes
        process_manager.stop_all(timeout=5)
        
        # Arrêt des threads
        poller.join(timeout=2.0)
        actuator.join(timeout=2.0)
        # lidar_socket.join(timeout=2.0)
        
        logger.info("🔴 Arrêt terminé")
    
    try:
        while True:
            # Affichage des sélections (optionnel, peut être retiré en prod)
            print(f"Switch selections: {board.get_switch_selections()}")
            time.sleep(1)
            
            # Le check I2C est maintenant géré par board_actuation_thread
            # Si le bus est down, stop_event sera set automatiquement
            if stop_event.is_set():
                logger.error("🛑 Communication I2C perdue - redémarrage du programme")
                clean_stop()
                time.sleep(1)
                restart_program()
                
    except KeyboardInterrupt:
        logger.info("🛑 Interruption par l'utilisateur")
    finally:
        clean_stop()
       

if __name__ == "__main__":
    main()

"""
Thread de polling des entrées GPIO avec debounce et callbacks.
"""

import time
import threading
import logging
from typing import Callable, Dict

from config import POLL_INTERVAL_MS, DEBOUNCE_MS

logger = logging.getLogger(__name__)


class InputPoller(threading.Thread):
    """Thread qui poll les entrées GPIO avec debounce et callbacks."""
    
    def __init__(self, board, stop_event: threading.Event):
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

"""
Thread d'actuation de la carte - Gestion des LED et monitoring I2C.
"""

import time
import threading
import logging

logger = logging.getLogger(__name__)


class BoardActuationThread(threading.Thread):
    """Thread qui actualise les LED et monitore le bus I2C."""
    
    def __init__(self, board, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.board = board
        self.stop_event = stop_event
        self.actuation_interval_ms = 10  # Intervalle d'actualisation (ms)
        self.i2c_check_interval = 5  # Vérifier I2C tous les N cycles
        self.cycle_count = 0
        self.i2c_error_count = 0
        self.i2c_error_threshold = 1  # Nombre d'erreurs consécutives avant alarme

    def run(self):
        """Boucle d'actualisation."""
        logger.info("BoardActuationThread démarré")
        
        while not self.stop_event.is_set():
            # Check I2C périodiquement
            if self.cycle_count % self.i2c_check_interval == 0:
                i2c_ok = self.board.check_i2c(log_errors=False)
                if not i2c_ok:
                    self.i2c_error_count += 1
                    if self.i2c_error_count >= self.i2c_error_threshold:
                        logger.error(f"⚠️ BUS I2C DOWN détecté ({self.i2c_error_count} erreurs consécutives)")
                        self.stop_event.set()
                else:
                    self.i2c_error_count = 0
            
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
        
        logger.info("BoardActuationThread arrêté")

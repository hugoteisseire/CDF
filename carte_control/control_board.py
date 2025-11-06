"""
Classe ControlBoard - Gestion de la carte MCP23017.
"""

import time
import logging
from dataclasses import dataclass
from config import (
    HARDWARE_AVAILABLE, STATE_ERROR, STATE_WARNING, STATE_OK,
    Direction, Pull, MCP23017
)

try:
    import board
    import busio
except ImportError:
    board = None
    busio = None

logger = logging.getLogger(__name__)


class ControlBoard:
    """Gère les entrées/sorties via MCP23017 et fournit des helpers pour LED, buzzer, etc."""
    
    def __init__(self, simulate: bool = False):
        self.simulate = simulate or not HARDWARE_AVAILABLE
        self.team = False
        self.state_lidar = STATE_ERROR
        self.state_board = STATE_ERROR
        self.sw_sel = 0
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
            self.mcp2 = MCP23017(i2c, address=0x25)
            self._configure_pins()
            logger.info("MCP23017 initialisés avec succès")
        except Exception as e:
            logger.error(f"Erreur initialisation I2C/MCP23017: {e}")
            raise

    def check_i2c(self, log_errors: bool = True) -> bool:
        """
        Vérifie que le bus I2C est fonctionnel en testant la communication avec les MCP23017.
        
        Args:
            log_errors: Si True, log les erreurs. Si False, détection silencieuse.
        
        Returns:
            True si OK, False si le bus est down.
        """
        if self.simulate:
            return True
        
        try:
            _ = self.mcp1.gpio
            _ = self.mcp2.gpio
            return True
        except OSError as e:
            if log_errors:
                logger.error(f"⚠️ BUS I2C DOWN (OSError): {e}")
            return False
        except RuntimeError as e:
            if log_errors:
                logger.error(f"⚠️ BUS I2C DOWN (RuntimeError): {e}")
            return False
        except Exception as e:
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

    def read_switch_selections(self):
        """Lit la position des switches de sélection (3 bits)."""
        sel1 = self.sw_sel_1.value
        sel2 = self.sw_sel_2.value
        sel3 = self.sw_sel_3.value
        self.sw_sel = (sel3 << 2) | (sel2 << 1) | sel1

    def get_switch_selections(self) -> int:
        """Retourne la position des switches de sélection (3 bits)."""
        return self.sw_sel
    
    def get_switch_selection(self) -> int:
        """Alias pour get_switch_selections (retourne sw_sel 0-7)."""
        return self.sw_sel
    
    def change_lidar_state(self, state):
        self.state_lidar = state
    
    def get_team(self):
        return self.team    

#!/usr/bin/env python3
"""
Script de test pour vérifier l'import et l'initialisation des modules.
"""

import sys
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_imports():
    """Test que tous les modules peuvent être importés."""
    logger.info("🧪 Test des imports...")
    
    try:
        from config import HARDWARE_AVAILABLE, STATE_ERROR, STATE_OK, STATE_WARNING
        logger.info("✅ config.py importé")
        
        from control_board import ControlBoard
        logger.info("✅ control_board.py importé")
        
        from board_actuation import BoardActuationThread
        logger.info("✅ board_actuation.py importé")
        
        from input_poller import InputPoller
        logger.info("✅ input_poller.py importé")
        
        from process_manager import ProcessManager
        logger.info("✅ process_manager.py importé")
        
        from unix_socket_thread import UnixSocketThread
        logger.info("✅ unix_socket_thread.py importé")
        
        from callbacks import (
            on_tirette_falling,
            on_bp_rst_lidar_rising,
            on_bp_init_rising,
            on_bp_rst_state_rising,
            on_sw_team_change
        )
        logger.info("✅ callbacks.py importé")
        
        logger.info(f"📦 Hardware disponible: {HARDWARE_AVAILABLE}")
        
        return True
    except Exception as e:
        logger.error(f"❌ Erreur d'import: {e}")
        return False

def test_process_manager():
    """Test du ProcessManager."""
    logger.info("\n🧪 Test du ProcessManager...")
    
    try:
        from process_manager import ProcessManager
        import time
        
        pm = ProcessManager()
        logger.info("✅ ProcessManager créé")
        
        # Test démarrage d'un processus simple (sleep)
        proc = pm.start_process(['sleep', '2'], name='TestSleep')
        if proc:
            logger.info("✅ Processus de test démarré")
            
            # Vérifier qu'il tourne
            if pm.is_running(proc):
                logger.info("✅ Processus confirmé actif")
            
            # Lister les processus
            processes = pm.list_processes()
            logger.info(f"✅ Processus listés: {len(processes)}")
            
            # Arrêter le processus
            pm.stop_process(proc)
            logger.info("✅ Processus arrêté")
        
        return True
    except Exception as e:
        logger.error(f"❌ Erreur ProcessManager: {e}")
        return False

def test_control_board():
    """Test du ControlBoard en mode simulation."""
    logger.info("\n🧪 Test du ControlBoard (mode simulation)...")
    
    try:
        from control_board import ControlBoard
        
        board = ControlBoard(simulate=True)
        logger.info("✅ ControlBoard créé en mode simulation")
        
        # Test des méthodes LED
        board.set_state_ok()
        logger.info("✅ set_state_ok()")
        
        board.set_state_warning()
        logger.info("✅ set_state_warning()")
        
        board.set_state_error()
        logger.info("✅ set_state_error()")
        
        board.set_color_lidar(True, False, False)
        logger.info("✅ set_color_lidar()")
        
        board.set_team_feedback()
        logger.info("✅ set_team_feedback()")
        
        # Test switches
        board.read_switch_selections()
        sel = board.get_switch_selections()
        logger.info(f"✅ Switch selections: {sel}")
        
        return True
    except Exception as e:
        logger.error(f"❌ Erreur ControlBoard: {e}")
        return False

def main():
    """Exécute tous les tests."""
    logger.info("=" * 60)
    logger.info("🧪 Tests du package carte_control")
    logger.info("=" * 60)
    
    tests = [
        ("Imports", test_imports),
        ("ProcessManager", test_process_manager),
        ("ControlBoard", test_control_board),
    ]
    
    results = {}
    for name, test_func in tests:
        try:
            results[name] = test_func()
        except Exception as e:
            logger.error(f"❌ Exception dans test {name}: {e}")
            results[name] = False
    
    # Résumé
    logger.info("\n" + "=" * 60)
    logger.info("📊 Résumé des tests")
    logger.info("=" * 60)
    
    for name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        logger.info(f"{status} - {name}")
    
    total = len(results)
    passed = sum(results.values())
    logger.info(f"\n🎯 Résultat: {passed}/{total} tests réussis")
    
    return all(results.values())

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

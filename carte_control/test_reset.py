#!/usr/bin/env python3
"""
Test du système de reset d'état.
Simule l'init puis le reset pour vérifier l'arrêt des processus.
"""

import sys
import os
import time

# Ajouter le répertoire parent au path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from process_manager import ProcessManager
from control_board import ControlBoard
from callbacks import (
    set_process_manager, 
    set_board, 
    on_bp_init_rising,
    on_bp_rst_state_rising
)
from config import setup_logging

logger = setup_logging()

def test_reset():
    """Test de la séquence init → reset."""
    logger.info("="*70)
    logger.info("🧪 TEST DU SYSTÈME DE RESET D'ÉTAT")
    logger.info("="*70)
    
    # Créer les objets
    board = ControlBoard(simulate=True)
    pm = ProcessManager()
    
    # Injecter les dépendances
    set_process_manager(pm)
    set_board(board)
    
    # Configuration
    board.team = False
    board.sw_sel = 0  # Pas de programme sélectionnable
    
    # PHASE 1: INITIALISATION
    logger.info("")
    logger.info("📍 PHASE 1: Initialisation")
    logger.info("")
    on_bp_init_rising('bp_init', True)
    
    logger.info("")
    logger.info("⏳ Attente 3 secondes (processus en cours)...")
    time.sleep(3)
    
    # Vérifier les processus actifs
    processes = pm.list_processes()
    logger.info(f"\n📊 Processus actifs avant reset: {len(processes)}")
    for p in processes:
        logger.info(f"   🟢 {p['name']} (PID: {p['pid']})")
    
    # PHASE 2: RESET
    logger.info("")
    logger.info("📍 PHASE 2: Reset d'état")
    logger.info("")
    on_bp_rst_state_rising('bp_rst_state', True)
    
    # Vérifier que tout est arrêté
    logger.info("")
    logger.info("⏳ Vérification après reset...")
    time.sleep(1)
    
    processes_after = pm.list_processes()
    logger.info(f"\n📊 Processus actifs après reset: {len(processes_after)}")
    
    if len(processes_after) == 0:
        logger.info("✅ Tous les processus ont été arrêtés correctement")
    else:
        logger.warning(f"⚠️  Il reste {len(processes_after)} processus actifs:")
        for p in processes_after:
            logger.warning(f"   {p['name']} (PID: {p['pid']})")
    
    logger.info("")
    logger.info("="*70)
    logger.info("✅ TEST TERMINÉ")
    logger.info("="*70)

if __name__ == "__main__":
    try:
        test_reset()
    except KeyboardInterrupt:
        logger.info("\n🛑 Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"❌ Erreur durant le test: {e}")
        import traceback
        traceback.print_exc()

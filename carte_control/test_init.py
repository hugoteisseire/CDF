#!/usr/bin/env python3
"""
Test du système d'initialisation.
Simule l'appui sur BP_INIT pour vérifier le démarrage des programmes.
"""

import sys
import os
import time

# Ajouter le répertoire parent au path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from process_manager import ProcessManager
from control_board import ControlBoard
from callbacks import set_process_manager, set_board, on_bp_init_rising
from config import setup_logging

logger = setup_logging()

def test_init():
    """Test de la séquence d'initialisation."""
    logger.info("="*70)
    logger.info("🧪 TEST DU SYSTÈME D'INITIALISATION")
    logger.info("="*70)
    
    # Créer les objets
    board = ControlBoard(simulate=True)
    pm = ProcessManager()
    
    # Injecter les dépendances
    set_process_manager(pm)
    set_board(board)
    
    # Simuler les switches
    board.team = False  # Équipe BLEU
    board.sw_sel = 2    # Programme 2
    logger.info(f"📊 Configuration:")
    logger.info(f"   - Équipe: {'ROUGE' if board.team else 'BLEU'}")
    logger.info(f"   - Switch sélection: {board.sw_sel}")
    logger.info("")
    
    # Simuler l'appui sur BP_INIT
    logger.info("🔘 Simulation appui sur BP_INIT...")
    logger.info("")
    on_bp_init_rising('bp_init', True)
    
    logger.info("")
    logger.info("="*70)
    logger.info("⏳ Attente 5 secondes pour observer les processus...")
    logger.info("="*70)
    time.sleep(5)
    
    # Arrêter tous les processus
    logger.info("")
    logger.info("🛑 Arrêt de tous les processus...")
    pm.stop_all()
    
    logger.info("")
    logger.info("="*70)
    logger.info("✅ TEST TERMINÉ")
    logger.info("="*70)

if __name__ == "__main__":
    try:
        test_init()
    except KeyboardInterrupt:
        logger.info("\n🛑 Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"❌ Erreur durant le test: {e}")
        import traceback
        traceback.print_exc()

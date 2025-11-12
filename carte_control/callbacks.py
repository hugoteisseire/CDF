"""
Callbacks applicatifs pour les événements GPIO.
"""
from config import LIDAR_EXECUTABLE, STRATEGY_EXECUTABLE, SELECTABLE_PROGRAMS
import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)

# Référence ProcessManager injectée depuis main
_PM = None  # type: Optional[object]
# Référence Board pour accéder aux switches
_BOARD = None  # type: Optional[object]

def set_process_manager(pm):
    global _PM
    _PM = pm

def set_board(board):
    global _BOARD
    _BOARD = board


def on_tirette_falling(name: str, value: bool):
    """Callback déclenché quand la tirette est retirée."""
    logger.info("🚀 Tirette retirée - démarrage match!")


def on_bp_rst_lidar_rising(name: str, value: bool):
    """Callback déclenché quand le bouton reset LIDAR est pressé."""
    if _PM is None:
        logger.error("❌ ProcessManager non initialisé (appelez set_process_manager(pm))")
        return

    logger.info("🔄 Reset LIDAR demandé")
    # Arrêt par nom (bloquant, timeout=5s par défaut)
    if _PM.stop_by_name('LIDAR_SIM'):
        # Petit délai pour laisser le système libérer les ressources (socket, fichiers)
        time.sleep(0.1)
        _PM.start_c_program(LIDAR_EXECUTABLE, name='LIDAR_SIM', args=["1" if _BOARD.get_team() else "0"])
        logger.info("✅ LIDAR redémarré")
    else:
        logger.error("❌ Échec arrêt LIDAR - redémarrage annulé")


def on_bp_init_rising(name: str, value: bool):
    """Callback déclenché quand le bouton init est pressé."""
    if _PM is None:
        logger.error("❌ ProcessManager non initialisé")
        return
    if _BOARD is None:
        logger.error("❌ Board non initialisé")
        return
    
    logger.info("⚙️  🚀 INITIALISATION DEMANDÉE")
    logger.info("="*60)
    
    # 1. Démarrer le LIDAR
    logger.info("📡 Démarrage du LIDAR...")
    team_arg = "1" if _BOARD.get_team() else "0" 
    team_arg = "3"  # Mode simulation quel que soit le switch équipe
    lidar_proc = _PM.start_c_program(LIDAR_EXECUTABLE, args=[team_arg], name='LIDAR_SIM')
    if lidar_proc:
        logger.info("✅ LIDAR démarré")
    else:
        logger.error("❌ Échec démarrage LIDAR")
    
    time.sleep(0.2)
    
    # 2. Démarrer le programme sélectionné via sw_sel
    sw_sel = _BOARD.get_switch_selection()
    logger.info(f"🎛️  Switch de sélection: {sw_sel}")
    
    prog_config = SELECTABLE_PROGRAMS.get(sw_sel)
    if prog_config:
        logger.info(f"🔧 Démarrage du programme sélectionné: {prog_config['name']}")
        if prog_config['type'] == 'python':
            proc = _PM.start_python(prog_config['path'], name=prog_config['name'])
        elif prog_config['type'] == 'c':
            proc = _PM.start_c_program(prog_config['path'], name=prog_config['name'])
        else:
            logger.warning(f"⚠️  Type de programme inconnu: {prog_config['type']}")
            proc = None
        
        if proc:
            logger.info(f"✅ {prog_config['name']} démarré")
        else:
            logger.error(f"❌ Échec démarrage {prog_config['name']}")
    else:
        logger.info(f"ℹ️  Aucun programme configuré pour sw_sel={sw_sel}")
    
    time.sleep(0.2)
    
    # 3. Démarrer la stratégie
    logger.info("🎯 Démarrage de la STRATÉGIE...")
    strategy_proc = _PM.start_python(STRATEGY_EXECUTABLE, name='STRATEGY')
    if strategy_proc:
        logger.info("✅ STRATÉGIE démarrée")
    else:
        logger.error("❌ Échec démarrage STRATÉGIE")
    
    logger.info("="*60)
    logger.info("✅ INITIALISATION TERMINÉE")
    
    # Afficher résumé des processus
    processes = _PM.list_processes()
    logger.info(f"📊 {len(processes)} processus actifs:")
    for p in processes:
        status = "🟢" if p['running'] else "🔴"
        logger.info(f"   {status} {p['name']} (PID: {p['pid']})")


def on_bp_rst_state_rising(name: str, value: bool):
    """Callback déclenché quand le bouton reset état est pressé.
    Arrête tous les processus et réinitialise les états à ERROR."""
    if _PM is None:
        logger.error("❌ ProcessManager non initialisé")
        return
    if _BOARD is None:
        logger.error("❌ Board non initialisé")
        return
    
    logger.info("🔄 RESET ÉTAT DEMANDÉ")
    logger.info("="*60)
    
    # 1. Arrêter tous les processus
    logger.info("🛑 Arrêt de tous les processus en cours...")
    processes = _PM.list_processes()
    if processes:
        logger.info(f"   {len(processes)} processus à arrêter:")
        for p in processes:
            logger.info(f"   - {p['name']} (PID: {p['pid']})")
        
        _PM.stop_all(timeout=5)
        logger.info("✅ Tous les processus arrêtés")
    else:
        logger.info("ℹ️  Aucun processus en cours")
    
    # 2. Réinitialiser les états à ERROR
    from config import STATE_ERROR
    
    logger.info("⚠️  Réinitialisation des états à ERROR...")
    _BOARD.state_lidar = STATE_ERROR
    _BOARD.state_board = STATE_ERROR
    
    # Mise à jour visuelle des LEDs
    _BOARD.set_state_lidar()  # Met LED LIDAR en rouge
    _BOARD.set_state_error()  # Met LED état en rouge
    
    logger.info("="*60)
    logger.info("✅ RESET TERMINÉ")
    logger.info("💡 État LIDAR: ERROR (LED rouge)")
    logger.info("💡 État BOARD: ERROR (LED rouge)")
    logger.info("📊 Processus actifs: 0")


def on_sw_team_change(name: str, value: bool, board):
    """Callback déclenché quand le switch équipe change."""
    board.team = value
    board.set_team_feedback()
    logger.info(f"🔵🔴 Équipe changée: {'ROUGE' if value else 'BLEU'}")

"""
Callbacks applicatifs pour les événements GPIO.
"""

import logging

logger = logging.getLogger(__name__)


def on_tirette_falling(name: str, value: bool):
    """Callback déclenché quand la tirette est retirée."""
    logger.info("🚀 Tirette retirée - démarrage match!")


def on_bp_rst_lidar_rising(name: str, value: bool):
    """Callback déclenché quand le bouton reset LIDAR est pressé."""
    logger.info("🔄 Reset LIDAR demandé")


def on_bp_init_rising(name: str, value: bool):
    """Callback déclenché quand le bouton init est pressé."""
    logger.info("⚙️  Initialisation demandée")


def on_bp_rst_state_rising(name: str, value: bool):
    """Callback déclenché quand le bouton reset état est pressé."""
    logger.info("🔄 Reset état demandé")


def on_sw_team_change(name: str, value: bool, board):
    """Callback déclenché quand le switch équipe change."""
    board.team = value
    board.set_team_feedback()
    logger.info(f"🔵🔴 Équipe changée: {'ROUGE' if value else 'BLEU'}")

#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Carte de contrôle avec MCP23017 - Point d'entrée principal.
Architecture modulaire pour multi-threading et sockets UNIX.
"""

import time
import threading
import logging
import sys
import os

# Imports locaux
from carte_control.config import STATE_ERROR, STATE_OK, STATE_WARNING
from config import HARDWARE_AVAILABLE, setup_logging
from control_board import ControlBoard
from board_actuation import BoardActuationThread
from input_poller import InputPoller
from process_manager import ProcessManager
from unix_socket_thread import UnixSocketServerThread, LidarSocket
from callbacks import (
    on_tirette_falling,
    on_bp_rst_lidar_rising,
    on_bp_init_rising,
    on_bp_rst_state_rising,
    on_sw_team_change
)

logger = setup_logging()


def restart_program():
    """Redémarre le programme en cas de problème I2C."""
    logger.warning("🔁 I2C KO → redémarrage du programme…")
    os.execv(sys.executable, ['python3'] + sys.argv)


def main():
    """Point d'entrée principal."""
    logger.info("🤖 Démarrage carte de contrôle")
    
    # ================================
    # INITIALISATION
    # ================================
    
    board = ControlBoard(simulate=not HARDWARE_AVAILABLE)
    stop_event = threading.Event()
    process_manager = ProcessManager()
    
    # Init état équipe
    board.team = board.sw_team.value
    board.set_team_feedback()
    
    # ================================
    # DÉMARRAGE DES THREADS
    # ================================
    
    # Thread d'actuation (LED + monitoring I2C)
    actuator = BoardActuationThread(board, stop_event)
    actuator.start()

    # Thread de polling des entrées
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
    
    # Thread socket LIDAR (serveur Python, attend les clients)
    lidar_socket = LidarSocket("/tmp/robot.sock", stop_event)
    lidar_socket.start()
    logger.info("🎯 Serveur socket LIDAR démarré")
    
    # Laisser le temps au socket d'être créé
    time.sleep(0.5)
    
    # Programme LIDAR simulateur C (client, se connecte au socket Python)
    lidar_executable = '/home/raspi/Desktop/CDF/carte_control/simu_lidar/test_lidar_socket'
    lidar_proc = process_manager.start_c_program(lidar_executable, name='LIDAR_SIM')
    
    if not lidar_proc:
        logger.warning("⚠️  Programme LIDAR non démarré (normal en développement)")
    else:
        logger.info("✅ Simulateur LIDAR démarré")
    
    # Autres programmes (exemples commentés)
    # vision_proc = process_manager.start_python(
    #     '/home/pi/vision/detect.py',
    #     args=['--mode', 'auto'],
    #     name='Vision'
    # )

    
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
        lidar_socket.join(timeout=2.0)
        
        logger.info("🔴 Arrêt terminé")
    
    # ================================
    # BOUCLE PRINCIPALE
    # ================================
    
    try:
        while True:
            # Affichage de l'état LIDAR
            if lidar_socket.is_data_fresh():
                state = lidar_socket.get_state()
                
                if state== 'init':
                    board.change_lidar_state(STATE_WARNING)
                elif state== 'running':
                    board.change_lidar_state(STATE_OK)
                elif state== 'error':
                    board.change_lidar_state(STATE_ERROR)
                elif state== 'lost':
                    board.change_lidar_state(STATE_WARNING)
                
                print(f"\n{'='*50}")
                print(f" État LIDAR: {state.upper()}")
    
                print(f"🔄  Switches: {board.get_switch_selections()}")

            else:
                print("⏳ En attente des données LIDAR...")
            
            time.sleep(1)
            
            # Le check I2C est géré par BoardActuationThread
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


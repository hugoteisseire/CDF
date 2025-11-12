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
from config import STATE_ERROR, STATE_OK, STATE_WARNING, LIDAR_EXECUTABLE
from config import HARDWARE_AVAILABLE, setup_logging, LIDAR_STATE_SOCKET, LIDAR_DATA_SOCKET
from control_board import ControlBoard
from board_actuation import BoardActuationThread
from input_poller import InputPoller
from process_manager import ProcessManager
from unix_socket_thread import UnixSocketServerThread, LidarSocket, LidarDataBroadcastSocket
from callbacks import (
    set_process_manager,
    set_board,
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
    
    # Injecter les dépendances dans les callbacks
    set_process_manager(process_manager)
    set_board(board)
    
    # Init état équipe
    board.team = board.sw_team.value
    board.set_team_feedback()

    # ======================================
    # SOCKETS UNIX
    # ======================================
    
    # Socket 2: Broadcast données LIDAR (Python → STRATEGY/PROG)
    # Permet à plusieurs clients de lire les données LIDAR en parallèle
    lidar_broadcast = LidarDataBroadcastSocket(LIDAR_DATA_SOCKET, stop_event)
    lidar_broadcast.start()
    logger.info(f"📡 Socket broadcast LIDAR démarré ({LIDAR_DATA_SOCKET})")
    
    # Socket 1: État du LIDAR (LIDAR → Python → Broadcast)
    # Le LIDAR envoie ses données, Python les redistribue via broadcast
    lidar_socket = LidarSocket(LIDAR_STATE_SOCKET, stop_event, broadcast_socket=lidar_broadcast)
    lidar_socket.start()
    logger.info(f"🎯 Socket LIDAR démarré ({LIDAR_STATE_SOCKET})")
    
    # Enregistrer le socket broadcast dans le ProcessManager
    process_manager.set_broadcast_socket(lidar_broadcast)
    
    # Note: Le programme LIDAR sera démarré lors de l'appui sur BP_INIT
    # (voir callback on_bp_init_rising)
    
    # ================================
    # DÉMARRAGE DES THREADS
    # ================================
    

    # Thread d'actuation (LED + monitoring I2C)
    actuator = BoardActuationThread(board, stop_event, lidar_socket)
    actuator.start()

    # Thread de polling des entrées
    poller = InputPoller(board, stop_event)
    poller.register_callback('tirette', on_tirette_falling)
    poller.register_callback('bp_rst_lidar', on_bp_rst_lidar_rising)
    poller.register_callback('bp_init', on_bp_init_rising)
    poller.register_callback('bp_rst_state', on_bp_rst_state_rising)
    poller.register_callback('sw_team', lambda n, v: on_sw_team_change(n, v, board))
    poller.start()


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
        lidar_broadcast.join(timeout=2.0)
        
        logger.info("🔴 Arrêt terminé")
    
    # ================================
    # BOUCLE PRINCIPALE
    # ================================
    time.sleep(2)  # Attente initiale avant la boucle
    on_bp_init_rising(None, None)  # Démarrer LIDAR automatiquement
    try:
        while True:
            # Affichage de l'état LIDAR
            #print(f"🔄main :  Switches: {board.get_switch_selections()}")
            
            time.sleep(2)
            
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


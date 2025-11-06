#!/usr/bin/env python3
"""
Simulateur de programme de stratégie.
Ce programme sera lancé lors de l'initialisation.
Il lit les données LIDAR via le socket broadcast et les parse.
"""

import time
import sys
import signal
import socket
import select
import threading
import os

# Ajouter le répertoire parent au path pour importer lidar_data_parser
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lidar_data_parser import LidarDataParser

# Configuration
LIDAR_DATA_SOCKET = '/tmp/lidar_data.sock'
running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n🛑 Stratégie arrêtée proprement")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


class LidarReader(threading.Thread):
    """Thread pour lire les données LIDAR en arrière-plan."""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = None
        self.connected = False
        self.parser = LidarDataParser()
        self.last_data = None
        self.data_lock = threading.Lock()
    
    def connect(self):
        """Se connecte au socket broadcast LIDAR."""
        retry_count = 0
        max_retries = 10
        
        while retry_count < max_retries and running:
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(LIDAR_DATA_SOCKET)
                self.sock.setblocking(False)
                self.connected = True
                print(f"✅ STRATÉGIE connectée au LIDAR ({LIDAR_DATA_SOCKET})")
                return True
            except (FileNotFoundError, ConnectionRefusedError):
                retry_count += 1
                if retry_count == 1:
                    print(f"⏳ Attente socket LIDAR...")
                time.sleep(0.5)
            except Exception as e:
                print(f"❌ Erreur connexion LIDAR: {e}")
                return False
        
        if retry_count >= max_retries:
            print(f"⚠️ Socket LIDAR non disponible après {max_retries} tentatives")
        return False
    
    def run(self):
        """Boucle de lecture des données LIDAR."""
        if not self.connect():
            print("⚠️ STRATÉGIE fonctionne sans données LIDAR")
            return
        
        while running and self.connected:
            try:
                rlist, _, _ = select.select([self.sock], [], [], 0.1)
                
                if rlist:
                    data = self.sock.recv(4096)
                    if not data:
                        print("⚠️ Socket LIDAR déconnecté")
                        self.connected = False
                        break
                    
                    # Parser les données avec le LidarDataParser
                    lidar_data = self.parser.parse(data)
                    if lidar_data:
                        # Stocker les dernières données parsées
                        with self.data_lock:
                            self.last_data = lidar_data
                    
            except Exception as e:
                print(f"❌ Erreur lecture LIDAR: {e}")
                self.connected = False
                break
        
        if self.sock:
            self.sock.close()
    
    def get_last_data(self):
        """Récupère les dernières données LIDAR reçues."""
        with self.data_lock:
            return self.last_data


def main():
    print("🎯 Programme STRATÉGIE démarré")
    print("Simulation de la stratégie de match...")
    
    # Démarrer le thread de lecture LIDAR
    lidar_reader = LidarReader()
    lidar_reader.start()
    
    count = 0
    while running:
        count += 1
        
        # Récupérer les données LIDAR parsées
        lidar_data = lidar_reader.get_last_data()
        
        if count % 10 == 0:
            if lidar_data:
                print(f"🔄 STRATÉGIE | {lidar_data}")
                
                # === EXEMPLE D'UTILISATION DES DONNÉES ===
                # Position du robot
                robot_x = lidar_data.pos_imu.x
                robot_y = lidar_data.pos_imu.y
                robot_angle = lidar_data.pos_imu.angle_deg()
                
                # État du LIDAR
                if lidar_data.state == "lost":
                    print("  ⚠️ LIDAR perdu!")
                
            else:
                print("🔄 STRATÉGIE en cours (pas de données LIDAR)...")
        
        # === VOTRE LOGIQUE DE STRATÉGIE ICI ===
        # Utiliser lidar_data pour prendre des décisions
        # Par exemple: évitement d'obstacles, calcul de trajectoire, etc.
        
        time.sleep(1)

if __name__ == "__main__":
    main()

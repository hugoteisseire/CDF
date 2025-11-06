#!/usr/bin/env python3
"""
Programme sélectionnable #1
Lit les données LIDAR via le socket broadcast et les parse.
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
    print("\n🛑 PROG_1 arrêté proprement")
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
                print(f"✅ PROG_1 connecté au LIDAR ({LIDAR_DATA_SOCKET})")
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
            print("⚠️ PROG_1 fonctionne sans données LIDAR")
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
    print("🚀 Programme sélectionnable #1 démarré")
    
    # Démarrer le thread de lecture LIDAR
    lidar_reader = LidarReader()
    lidar_reader.start()
    
    count = 0
    while running:
        count += 1
        
        # Récupérer les données LIDAR parsées
        lidar_data = lidar_reader.get_last_data()
        
        if lidar_data:
            print(f"📍 PROG_1 | {lidar_data}")
            # === EXEMPLE D'UTILISATION ===
            # Accéder aux positions
            x = lidar_data.pos_imu.x
            y = lidar_data.pos_imu.y
            
            # Vérifier les piliers
            piliers_detectes = sum([
                lidar_data.pillar_1.found,
                lidar_data.pillar_2.found,
                lidar_data.pillar_3.found
            ])
        else:
            print("#programme sélectionné: 1 (pas de données LIDAR)")
        
        # === VOTRE LOGIQUE ICI ===
        # Utiliser lidar_data pour votre programme
        
        time.sleep(2)

if __name__ == "__main__":
    main()

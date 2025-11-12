#!/usr/bin/env python3
"""
Thread de lecture LIDAR dédié.
Se connecte au socket broadcast, lit les frames et expose la dernière donnée parsée.
"""

import os
import sys
import time
import socket
import select
import threading

# Import du parser depuis le répertoire parent (carte_control)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lidar_data_parser import LidarDataParser


class LidarReader(threading.Thread):
    """Thread pour lire les données LIDAR en arrière-plan."""

    def __init__(self, socket_path: str = '/tmp/lidar_data.sock'):
        super().__init__(daemon=True)
        self.socket_path = socket_path
        self.sock: socket.socket | None = None
        self.connected = False
        self.parser = LidarDataParser()
        self.last_data = None
        self.data_lock = threading.Lock()
        self._stop = False

    def connect(self) -> bool:
        """Se connecte au socket broadcast LIDAR avec quelques tentatives."""
        retry_count = 0
        max_retries = 10

        while retry_count < max_retries and not self._stop:
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(self.socket_path)
                self.sock.setblocking(False)
                self.connected = True
                print(f"✅ STRATÉGIE connectée au LIDAR ({self.socket_path})")
                return True
            except (FileNotFoundError, ConnectionRefusedError):
                retry_count += 1
                if retry_count == 1:
                    print("⏳ Attente socket LIDAR...")
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

        while not self._stop and self.connected:
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
            try:
                self.sock.close()
            except Exception:
                pass

    def get_last_data(self):
        """Récupère les dernières données LIDAR reçues (thread-safe)."""
        with self.data_lock:
            return self.last_data

    def stop(self):
        """Demande l'arrêt du thread et ferme le socket."""
        self._stop = True
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass

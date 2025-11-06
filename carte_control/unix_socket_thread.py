"""
Thread générique pour gérer un socket UNIX non-bloquant.
Python crée le serveur, les programmes C se connectent en clients.
"""

import socket
import select
import threading
import logging
import os
from typing import Optional, List

logger = logging.getLogger(__name__)


class UnixSocketServerThread(threading.Thread):
    """Thread serveur pour gérer un socket UNIX non-bloquant.
    Python crée le socket et attend les connexions des clients C."""
    
    def __init__(self, socket_path: str, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.socket_path = socket_path
        self.stop_event = stop_event
        self.server_sock: Optional[socket.socket] = None
        self.client_sock: Optional[socket.socket] = None
    
    def create_server(self):
        """Crée le serveur socket UNIX."""
        try:
            # Supprimer le socket s'il existe déjà
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
            
            # Créer le socket serveur
            self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.server_sock.bind(self.socket_path)
            self.server_sock.listen(1)
            self.server_sock.setblocking(False)
            logger.info(f"🎯 Serveur socket créé sur {self.socket_path}")
            return True
        except Exception as e:
            logger.error(f"Erreur création serveur socket {self.socket_path}: {e}")
            return False
    
    def accept_client(self):
        """Accepte une connexion cliente (non-bloquant)."""
        try:
            rlist, _, _ = select.select([self.server_sock], [], [], 0.05)
            if rlist:
                self.client_sock, _ = self.server_sock.accept()
                self.client_sock.setblocking(False)
                logger.info(f"✅ Client connecté sur {self.socket_path}")
                return True
        except Exception as e:
            logger.debug(f"En attente de client: {e}")
        return False
    
    def run(self):
        """Boucle principale du thread."""
        if not self.create_server():
            return
        
        logger.info(f"En attente de connexion sur {self.socket_path}...")
        
        while not self.stop_event.is_set():
            # Si pas de client, essayer d'accepter une connexion
            if not self.client_sock:
                self.accept_client()
                continue
            
            try:
                # Attente non-bloquante de données du client
                rlist, _, _ = select.select([self.client_sock], [], [], 0.05)
                
                if rlist:
                    data = self.client_sock.recv(4096)
                    if not data:
                        logger.warning("Client déconnecté")
                        self.client_sock.close()
                        self.client_sock = None
                        continue
                    self.handle_data(data)
                    
            except Exception as e:
                logger.error(f"Erreur lecture socket: {e}")
                if self.client_sock:
                    self.client_sock.close()
                    self.client_sock = None
        
        # Nettoyage
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        logger.info(f"Serveur socket arrêté")
    
    def handle_data(self, data: bytes):
        """À surcharger pour traiter les données reçues."""
        logger.debug(f"Reçu {len(data)} octets")


class LidarSocket(UnixSocketServerThread):
    """Thread pour recevoir l'état ET les données du LIDAR via socket UNIX."""
    
    # États possibles
    STATE_INIT = "init"
    STATE_READY = "ready"    
    STATE_LOST = "lost"
    STATE_ERROR = "error"
    
    def __init__(self, socket_path: str, stop_event: threading.Event, broadcast_socket=None):
        super().__init__(socket_path, stop_event)
        self.state = self.STATE_INIT
        self.last_update = None
        self.broadcast_socket = broadcast_socket  # Socket pour redistribuer les données
        # Buffer de réception pour extraire proprement la ligne d'état
        self.recv_buffer: bytes = b""
    
    def set_broadcast_socket(self, broadcast_socket):
        """Enregistre le socket broadcast pour redistribuer les données."""
        self.broadcast_socket = broadcast_socket
        logger.info("📡 Socket broadcast enregistré dans LidarSocket")
    
    def handle_data(self, data: bytes):
        """
        Traite les données reçues du programme LIDAR C.
        - Redistribue immédiatement les octets reçus (état + binaire) aux clients
        - Met à jour l'état en décodant UNIQUEMENT la ligne avant '\n'
        """
        import time

        # 1) Broadcast immédiat des octets bruts (préserve le flux pour les clients)
        if self.broadcast_socket:
            self.broadcast_socket.broadcast_data(data)

        # 2) Mise à jour de l'horodatage de fraîcheur
        self.last_update = time.time()

        # 3) Accumuler et tenter d'extraire une ligne d'état sans toucher au binaire
        #    La trame émise par le LIDAR est: "etat\n" puis 20 floats binaires (80 octets)
        self.recv_buffer += data

        # Chercher une fin de ligne (état)
        newline_idx = self.recv_buffer.find(b"\n")
        if newline_idx != -1:
            # Extraire uniquement la partie texte de l'état
            state_bytes = self.recv_buffer[:newline_idx]
            try:
                state_str = state_bytes.decode('utf-8', errors='strict').strip().lower()
            except UnicodeDecodeError:
                # Si jamais l'état est corrompu, ignorer ce chunk texte
                state_str = ""

            valid_states = [self.STATE_INIT, self.STATE_READY, self.STATE_LOST, self.STATE_ERROR]
            if state_str in valid_states:
                self.state = state_str

            # Purger le buffer après la ligne d'état pour éviter de conserver le binaire
            # (le binaire a déjà été diffusé aux clients)
            self.recv_buffer = b""
    
    def get_state(self):
        """Retourne l'état courant du LIDAR."""
        #seulement si le client est connecté
        if not self.client_sock:
            return None
        return self.state
    
    def is_data_fresh(self, max_age_sec=5.0):
        """Vérifie si les données sont récentes."""
        if not self.last_update:
            return False
        import time
        return (time.time() - self.last_update) < max_age_sec


class LidarDataBroadcastSocket(threading.Thread):
    """
    Socket serveur pour diffuser les données LIDAR à plusieurs clients.
    Le LIDAR publie des données, STRATEGY et programmes les lisent.
    Architecture: 1 serveur (Python) → N clients (STRATEGY, PROG_1, PROG_2, etc.)
    """
    
    def __init__(self, socket_path: str, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.socket_path = socket_path
        self.stop_event = stop_event
        self.server_sock: Optional[socket.socket] = None
        self.clients: List[socket.socket] = []  # Liste des clients connectés
        self.clients_lock = threading.Lock()
        self.data_to_broadcast: bytes = b""  # Dernières données à diffuser
        self.data_lock = threading.Lock()
    
    def create_server(self):
        """Crée le serveur socket UNIX."""
        try:
            # Supprimer le socket s'il existe déjà
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
            
            # Créer le socket serveur
            self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.server_sock.bind(self.socket_path)
            self.server_sock.listen(5)  # Accepter jusqu'à 5 connexions en attente
            self.server_sock.setblocking(False)
            logger.info(f"📡 Socket broadcast LIDAR créé sur {self.socket_path}")
            return True
        except Exception as e:
            logger.error(f"Erreur création socket broadcast {self.socket_path}: {e}")
            return False
    
    def accept_clients(self):
        """Accepte de nouvelles connexions clientes (non-bloquant)."""
        try:
            rlist, _, _ = select.select([self.server_sock], [], [], 0.05)
            if rlist:
                client_sock, _ = self.server_sock.accept()
                client_sock.setblocking(False)
                with self.clients_lock:
                    self.clients.append(client_sock)
                logger.info(f"✅ Nouveau client LIDAR connecté ({len(self.clients)} clients actifs)")
        except Exception as e:
            logger.debug(f"En attente de clients: {e}")
    
    def broadcast_data(self, data: bytes):
        """
        Envoie des données à tous les clients connectés.
        Cette méthode est appelée par le LIDAR pour publier ses données.
        """
        if not data:
            return
        
        with self.data_lock:
            self.data_to_broadcast = data
        
        disconnected = []
        
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.sendall(data)
                except (BrokenPipeError, ConnectionResetError):
                    logger.warning("Client déconnecté lors du broadcast")
                    disconnected.append(client)
                except Exception as e:
                    logger.error(f"Erreur envoi au client: {e}")
                    disconnected.append(client)
            
            # Nettoyer les clients déconnectés
            for client in disconnected:
                try:
                    client.close()
                except:
                    pass
                self.clients.remove(client)
            
            if disconnected:
                logger.info(f"📉 {len(disconnected)} client(s) déconnecté(s) ({len(self.clients)} clients actifs)")
    
    def run(self):
        """Boucle principale du thread."""
        if not self.create_server():
            return
        
        logger.info(f"En attente de clients sur {self.socket_path}...")
        
        while not self.stop_event.is_set():
            # Accepter de nouvelles connexions
            self.accept_clients()
            
            # Courte pause pour éviter de surcharger le CPU
            self.stop_event.wait(0.05)
        
        # Nettoyage
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except:
                    pass
            self.clients.clear()
        
        if self.server_sock:
            self.server_sock.close()
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        logger.info(f"Socket broadcast LIDAR arrêté")
    
    def get_client_count(self):
        """Retourne le nombre de clients connectés."""
        with self.clients_lock:
            return len(self.clients)
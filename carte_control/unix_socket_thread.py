"""
Thread générique pour gérer un socket UNIX non-bloquant.
Python crée le serveur, les programmes C se connectent en clients.
"""

import socket
import select
import threading
import logging
import os
from typing import Optional

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
    """Thread pour recevoir l'état du LIDAR via socket UNIX."""
    
    # États possibles
    STATE_INIT = "init"
    STATE_READY = "ready"    
    STATE_LOST = "lost"
    STATE_ERROR = "error"
    
    def __init__(self, socket_path: str, stop_event: threading.Event):
        super().__init__(socket_path, stop_event)
        self.state = self.STATE_INIT
        self.last_update = None
    
    def handle_data(self, data: bytes):
        """
        Traite les données reçues du programme LIDAR C.
        Format: chaîne de caractères terminée par '\n'
        États possibles: init, ready, lost, error
        """
        import time
        
        try:
            # Décoder le message en string
            message = data.decode('utf-8').strip()
            
            # Extraire l'état (premier mot si plusieurs)
            state = message.split()[0].lower() if message else ""
            
            # Valider l'état
            valid_states = [self.STATE_INIT, self.STATE_READY,self.STATE_LOST, self.STATE_ERROR]
            
            if state in valid_states:
                old_state = self.state
                self.state = state
                self.last_update = time.time()
                
                # Log uniquement si changement d'état
                if old_state != state:
                    logger.info(f"LIDAR état: {old_state} → {state}")
            else:
                logger.warning(f"État LIDAR inconnu: '{message}'")
                
        except UnicodeDecodeError as e:
            logger.error(f"Erreur décodage message LIDAR: {e}")
    
    def get_state(self):
        """Retourne l'état courant du LIDAR."""
        return self.state
    
    def is_data_fresh(self, max_age_sec=5.0):
        """Vérifie si les données sont récentes."""
        if not self.last_update:
            return False
        import time
        return (time.time() - self.last_update) < max_age_sec
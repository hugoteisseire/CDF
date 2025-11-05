"""
Thread générique pour gérer un socket UNIX non-bloquant.
"""

import socket
import select
import threading
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class UnixSocketThread(threading.Thread):
    """Thread générique pour gérer un socket UNIX non-bloquant."""
    
    def __init__(self, socket_path: str, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.socket_path = socket_path
        self.stop_event = stop_event
        self.sock: Optional[socket.socket] = None
    
    def connect(self):
        """Établit la connexion au socket UNIX."""
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(self.socket_path)
            self.sock.setblocking(False)
            logger.info(f"Connecté au socket {self.socket_path}")
            return True
        except Exception as e:
            logger.error(f"Erreur connexion socket {self.socket_path}: {e}")
            return False
    
    def run(self):
        """Boucle principale du thread."""
        if not self.connect():
            return
        
        logger.info(f"UnixSocketThread démarré sur {self.socket_path}")
        
        while not self.stop_event.is_set():
            try:
                # Attente non-bloquante de données
                rlist, _, _ = select.select([self.sock], [], [], 0.05)
                
                if rlist:
                    data = self.sock.recv(4096)
                    if not data:
                        logger.warning("Socket fermé par le serveur")
                        break
                    self.handle_data(data)
                    
            except Exception as e:
                logger.error(f"Erreur dans boucle socket: {e}")
                break
        
        if self.sock:
            self.sock.close()
        logger.info(f"UnixSocketThread arrêté")
    
    def handle_data(self, data: bytes):
        """À surcharger pour traiter les données reçues."""
        logger.debug(f"Reçu {len(data)} octets")

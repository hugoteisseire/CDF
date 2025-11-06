#!/usr/bin/env python3
"""
Parser pour les données LIDAR.
Décode la trame binaire envoyée par le programme LIDAR C/C++.

Format de la trame:
- État (string terminée par \n): "init\n", "ready\n", "lost\n", "error\n"
- Données (80 octets = 20 floats):
  [0-2]   posImu: x, y, angle (mm, mm, rad)
  [3-5]   poslidar: x, y, angle (mm, mm, rad)
  [6-7]   pos_adv: x, y (mm, mm)
  [8-10]  trackedPoints[0]: angle, distance, found (rad, mm, bool)
  [11-13] trackedPoints[1]: angle, distance, found
  [14-16] trackedPoints[2]: angle, distance, found
  [17-19] reserved
"""

import struct
import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class Position:
    """Position 2D avec orientation."""
    x: float  # mm
    y: float  # mm
    angle: float  # radians
    
    def angle_deg(self) -> float:
        """Retourne l'angle en degrés."""
        return self.angle * 180.0 / math.pi


@dataclass
class TrackedPoint:
    """Point tracké (pilier) par le LIDAR."""
    angle: float      # radians
    distance: float   # mm
    found: bool       # True si détecté, False si estimation


@dataclass
class LidarData:
    """Structure complète des données LIDAR."""
    state: str                      # "init", "ready", "lost", "error"
    pos_imu: Position               # Position IMU
    pos_lidar: Position             # Position LIDAR
    pos_adv: Tuple[float, float]    # Position adversaire (x, y)
    pillar_1: TrackedPoint          # Pilier 1
    pillar_2: TrackedPoint          # Pilier 2
    pillar_3: TrackedPoint          # Pilier 3
    
    def __str__(self):
        """Représentation textuelle."""
        return (f"État: {self.state} | "
                f"IMU: ({self.pos_imu.x:.0f}, {self.pos_imu.y:.0f}, {self.pos_imu.angle_deg():.1f}°) | "
                f"LIDAR: ({self.pos_lidar.x:.0f}, {self.pos_lidar.y:.0f}, {self.pos_lidar.angle_deg():.1f}°) | "
                f"Piliers: {int(self.pillar_1.found)}/{int(self.pillar_2.found)}/{int(self.pillar_3.found)} | "
                f"Adv: ({self.pos_adv[0]:.0f}, {self.pos_adv[1]:.0f})")


class LidarDataParser:
    """Parser pour les données LIDAR en format binaire."""
    
    # Taille des données binaires (20 floats)
    DATA_SIZE = 20 * 4  # 80 octets
    
    def __init__(self):
        self.buffer = b""
        self.last_state = "init"
    
    def parse(self, raw_data: bytes) -> Optional[LidarData]:
        """
        Parse les données brutes reçues du LIDAR.
        
        Args:
            raw_data: Données brutes (état + binaire)
        
        Returns:
            LidarData si parsing réussi, None sinon
        """
        # Ajouter au buffer
        self.buffer += raw_data
        
        # Chercher un état suivi de données binaires
        try:
            # Chercher la fin de l'état (\n)
            newline_idx = self.buffer.find(b'\n')
            if newline_idx == -1:
                # Pas encore d'état complet
                return None
            
            # Extraire l'état
            state_bytes = self.buffer[:newline_idx]
            state = state_bytes.decode('utf-8', errors='ignore').strip().lower()
            
            # Vérifier si on a assez de données après l'état
            data_start = newline_idx + 1
            if len(self.buffer) < data_start + self.DATA_SIZE:
                # Pas encore toutes les données binaires
                return None
            
            # Extraire les données binaires
            binary_data = self.buffer[data_start:data_start + self.DATA_SIZE]
            
            # Consommer les données du buffer
            self.buffer = self.buffer[data_start + self.DATA_SIZE:]
            
            # Décoder les 20 floats
            floats = struct.unpack('20f', binary_data)
            
            # Mémoriser l'état
            if state in ['init', 'ready', 'lost', 'error']:
                self.last_state = state
            
            # Construire la structure LidarData
            return LidarData(
                state=self.last_state,
                pos_imu=Position(floats[0], floats[1], floats[2]),
                pos_lidar=Position(floats[3], floats[4], floats[5]),
                pos_adv=(floats[6], floats[7]),
                pillar_1=TrackedPoint(floats[8], floats[9], floats[10] > 0.5),
                pillar_2=TrackedPoint(floats[11], floats[12], floats[13] > 0.5),
                pillar_3=TrackedPoint(floats[14], floats[15], floats[16] > 0.5)
            )
            
        except Exception as e:
            print(f"⚠️ Erreur parsing LIDAR: {e}")
            # Nettoyer le buffer en cas d'erreur
            self.buffer = b""
            return None
    
    def clear_buffer(self):
        """Vide le buffer de parsing."""
        self.buffer = b""


# Exemple d'utilisation
if __name__ == "__main__":
    import socket
    import select
    
    LIDAR_DATA_SOCKET = '/tmp/lidar_data.sock'
    
    print("🧪 Test du parser LIDAR")
    print(f"Connexion à {LIDAR_DATA_SOCKET}...")
    
    parser = LidarDataParser()
    
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(LIDAR_DATA_SOCKET)
        sock.setblocking(False)
        print("✅ Connecté\n")
        
        while True:
            rlist, _, _ = select.select([sock], [], [], 0.1)
            
            if rlist:
                data = sock.recv(4096)
                if not data:
                    print("Socket fermé")
                    break
                
                # Parser les données
                lidar_data = parser.parse(data)
                if lidar_data:
                    print(lidar_data)
                    
                    # Exemple d'utilisation des données
                    if lidar_data.pillar_1.found:
                        print(f"  → Pilier 1 détecté à {lidar_data.pillar_1.distance:.0f}mm, "
                              f"angle {lidar_data.pillar_1.angle * 180 / math.pi:.1f}°")
    
    except KeyboardInterrupt:
        print("\n🛑 Arrêt")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    finally:
        sock.close()

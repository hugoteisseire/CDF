# client.py
import socket
import struct
import time
import select
import math
import pygame
from dataclasses import dataclass

path = "/tmp/robot.sock"

# Reçoit exactement n octets (bloquant)


def recv_all(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


@dataclass
class Pos:
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0


@dataclass
class TrackedPoint:
    angle: float = 0.0
    distance: float = 0.0
    found: bool = False


# Paramètres du "monde" (dimensions réelles)
WORLD_W = 3000.0   # largeur monde (ex: mm)
WORLD_H = 2000.0   # hauteur monde (ex: mm)

# Facteur de rétrécissement (changer si besoin)
SHRINK = 2.0       # par ex. 2 -> fenêtre affichage 1500x1000
SCREEN_W = int(WORLD_W / SHRINK)
SCREEN_H = int(WORLD_H / SHRINK)

# Couleurs
COLOR_BG = (10, 10, 10)
COLOR_IMU = (50, 150, 255)
COLOR_LIDAR = (50, 255, 100)
COLOR_TRACKED_FOUND = (255, 60, 60)
COLOR_TRACKED_MISSING = (140, 140, 140)
GRID_COLOR = (30, 30, 30)

# Taille affichage des points
R_IMU = 6
R_LIDAR = 6
R_TRACK = 5

# Attendre un peu que le serveur C soit prêt
time.sleep(0.5)

# Création socket UNIX client
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(path)
# utilisation de select pour ne pas bloquer l'affichage
sock.setblocking(False)
print("Connecté au serveur C.")

# Le C envoie float data[18] -> 18 floats -> 72 octets
BYTES_EXPECTED = 18 * 4

# Initialisation pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Visualisation LIDAR / IMU")
clock = pygame.time.Clock()


def world_to_screen(x, y):
    """
    Convertit coordonnées monde (origine en bas-left, unité = mm) 
    en coordonnées écran (origine en top-left).
    """
    sx = int(x / SHRINK)
    # inversion Y pour mettre origine en bas-left
    sy = SCREEN_H - int(y / SHRINK)
    return sx, sy


# valeurs par défaut
pos_imu = Pos()
pos_lidar = Pos()
tracked = [TrackedPoint(), TrackedPoint(), TrackedPoint()]

running = True
while running:
    # gestion évènements pygame
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # attente non bloquante d'un nouveau paquet (timeout court)
    rlist, _, _ = select.select([sock], [], [], 0.05)
    if rlist:
        data = recv_all(sock, BYTES_EXPECTED)
        if not data:
            print("Connexion fermée par le serveur.")
            running = False
            break

        try:
            vals = struct.unpack('18f', data)
        except struct.error:
            print("Taille de paquet inattendue :", len(data))
            running = False
            break

        # Mappe les valeurs dans les structures
        pos_imu = Pos(x=vals[0], y=vals[1], angle=vals[2])
        pos_lidar = Pos(x=vals[3], y=vals[4], angle=vals[5])

        tracked = []
        # indices 6..14 contiennent 3 tracked points (3 champs chacun)
        for i in range(3):
            base = 6 + i * 3
            angle = vals[base]
            distance = vals[base + 1]
            # le champ 'found' peut être float(0/1) : convertir en bool
            found = bool(int(round(vals[base + 2])))
            tracked.append(TrackedPoint(
                angle=angle, distance=distance, found=found))

    # Dessin
    screen.fill(COLOR_BG)

    # grille simple tous les 500 mm
    step_mm = 500
    for gx in range(0, int(WORLD_W) + 1, step_mm):
        x1, y1 = world_to_screen(gx, 0)
        x2, y2 = world_to_screen(gx, WORLD_H)
        pygame.draw.line(screen, GRID_COLOR, (x1, y1), (x2, y2), 1)
    for gy in range(0, int(WORLD_H) + 1, step_mm):
        x1, y1 = world_to_screen(0, gy)
        x2, y2 = world_to_screen(WORLD_W, gy)
        pygame.draw.line(screen, GRID_COLOR, (x1, y1), (x2, y2), 1)

    # Dessine IMU
    imu_sx, imu_sy = world_to_screen(pos_imu.x, pos_imu.y)
    pygame.draw.circle(screen, COLOR_IMU, (imu_sx, imu_sy), R_IMU)
    # Dessine orientation IMU (petite ligne)
    imu_orient_len = int(40 / SHRINK)
    imu_dir_x = imu_sx + int(math.cos(pos_imu.angle) * imu_orient_len)
    imu_dir_y = imu_sy - int(math.sin(pos_imu.angle) * imu_orient_len)
    pygame.draw.line(screen, COLOR_IMU, (imu_sx, imu_sy),
                     (imu_dir_x, imu_dir_y), 2)

    # Dessine LIDAR
    lidar_sx, lidar_sy = world_to_screen(pos_lidar.x, pos_lidar.y)
    pygame.draw.circle(screen, COLOR_LIDAR, (lidar_sx, lidar_sy), R_LIDAR)

    # Dessine tracked points (coord polaires depuis LIDARs)
    for tp in tracked:
        # On suppose angle du tracked point relatif à l'orientation LIDAR.
        abs_angle = pos_lidar.angle + tp.angle
        tx = pos_lidar.x + tp.distance * math.cos(abs_angle)
        ty = pos_lidar.y + tp.distance * math.sin(abs_angle)
        sx, sy = world_to_screen(tx, ty)
        color = COLOR_TRACKED_FOUND if tp.found else COLOR_TRACKED_MISSING
        pygame.draw.circle(screen, color, (sx, sy), R_TRACK)
        # Ligne depuis IMU vers le point
        pygame.draw.line(screen, color, (imu_sx, imu_sy), (sx, sy), 1)

    # Affiche quelques textes synthétiques
    font = pygame.font.SysFont(None, 20)
    t1 = font.render(
        f"IMU x={pos_imu.x:.1f} y={pos_imu.y:.1f} ang={math.degrees(pos_imu.angle):.1f}°", True, COLOR_IMU)
    t2 = font.render(
        f"LIDAR x={pos_lidar.x:.1f} y={pos_lidar.y:.1f} ang={math.degrees(pos_lidar.angle):.1f}°", True, COLOR_LIDAR)
    screen.blit(t1, (10, 10))
    screen.blit(t2, (10, 30))

    pygame.display.flip()
    clock.tick(30)  # limite 30 FPS

sock.close()
pygame.quit()

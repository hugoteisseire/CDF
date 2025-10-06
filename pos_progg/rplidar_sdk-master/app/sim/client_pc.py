import pygame
import socket
import json
import numpy as np

# --- Paramètres d'affichage ---
WIDTH, HEIGHT = 1000, 667
SCALE_X = WIDTH / 3000
SCALE_Y = HEIGHT / 2000
SCALE = min(SCALE_X, SCALE_Y)

def to_screen(pos_mm):
    return np.array([pos_mm[0] * SCALE, pos_mm[1] * SCALE])

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Client Affichage Robot")
font = pygame.font.SysFont(None, 24)
clock = pygame.time.Clock()

# --- Connexion au Pi ---
sock = socket.socket()
sock.connect(("10.0.1.95", 12345))  # remplace par l'IP du Pi
sock_file = sock.makefile()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        line = sock_file.readline()
        if not line:
            break
        data = json.loads(line)
    except Exception as e:
        print("Erreur lecture socket :", e)
        break

    screen.fill((255, 255, 255))

    # Table
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, WIDTH, HEIGHT), 5)

    # Cible
    pygame.draw.circle(screen, (0, 0, 255), to_screen(data["target_pos"]).astype(int), int(20 * SCALE))

    # Ennemi
    pygame.draw.circle(screen, (255, 0, 0), to_screen(data["enemy_pos"]).astype(int), int(150 * SCALE))

    # Robot
    pos = to_screen(data["robot_pos"]).astype(int)
    pygame.draw.circle(screen, (0, 100, 255), pos, int(150 * SCALE))
    vel = np.array(data["base_velocity"]) * 150
    pygame.draw.line(screen, (0, 100, 255), pos, to_screen(np.array(data["robot_pos"]) + vel).astype(int), 4)

    # Pause info
    if data["pause_robot"]:
        pygame.draw.rect(screen, (255, 0, 0), (WIDTH//2 - 100, HEIGHT//2 - 30, 200, 60))
        text = font.render("PAUSE: ENNEMI SUR LA CIBLE", True, (255, 255, 255))
        screen.blit(text, (WIDTH//2 - text.get_width()//2, HEIGHT//2 - 10))
        # Commande à envoyer
    command = None
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_t:
                command = "toggle_target"
            elif event.key == pygame.K_s:
                command = "toggle_robot"
            elif event.key == pygame.K_h:
                command = "reset_pos"
        elif pygame.mouse.get_pressed()[0]:
            command = {
                "click": True,
                "mouse_pos": pygame.mouse.get_pos()
            }
    
    # Envoi de la commande
    if command is not None:
        try:
            sock.sendall((json.dumps({"command": command}) + "\n").encode())
        except Exception as e:
            print("Erreur d'envoi de commande :", e)
    pygame.display.flip()
    clock.tick(60)

pygame.quit()

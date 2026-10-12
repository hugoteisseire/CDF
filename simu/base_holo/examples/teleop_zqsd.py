"""Pilotage temps réel du robot holonome au clavier (ZQSD) avec Motor.move_velocity.

    Z / S : avancer / reculer (NORD / SUD du repère robot, comme dans le test du carré)
    Q / D : gauche / droite   (OUEST / EST)
    A / E : rotation anti-horaire / horaire
    ↑ / ↓ : vitesse max +/- 10 mm/s
    ESPACE: arrêt immédiat      ECHAP : quitter (arrêt avant sortie)

Lancer depuis ~/CDF/simu/base_holo (session graphique / VNC, la fenêtre pygame doit avoir le focus) :
    python -m examples.teleop_zqsd
Si la fenêtre perd le focus ou si le programme plante, les moteurs sont arrêtés.
"""
import math
import time

import numpy as np

from core.holo_base import Robot, compute_wheel_distances_global

LOOP_HZ = 50
MAX_RPM = 3000


class TeleopDriver:
    """Transforme une commande (ax, ay, aw) ∈ [-1,1] en consignes moteurs, avec rampe LOGICIELLE
    sur la vitesse du ROBOT (donc les 3 roues restent proportionnelles à tout instant).
    Les accélérations MKS sont mises au maximum : c'est la rampe logicielle qui décide.
    """

    def __init__(self, robot, motors, speed_mm_s=60.0, rot_rad_s=1.0,
                 accel_mm_s2=1000.0, rot_accel_rad_s2=4.0, mks_acc=255):
        self.robot, self.motors = robot, motors
        self.speed, self.rot = speed_mm_s, rot_rad_s
        self.accel, self.rot_accel = accel_mm_s2, rot_accel_rad_s2
        self.mks_acc = mks_acc
        self.v = np.zeros(3)            # vitesse courante (vx, vy mm/s ; omega rad/s)
        self._last_sent = [None] * len(motors)

    @staticmethod
    def _approach(cur, target, max_delta):
        return cur + max(-max_delta, min(max_delta, target - cur))

    def step(self, ax, ay, aw, dt):
        # normalise la commande diagonale pour ne pas aller plus vite à 45°
        n = math.hypot(ax, ay)
        if n > 1:
            ax, ay = ax / n, ay / n
        target = np.array([ax * self.speed, ay * self.speed, aw * self.rot])
        # rampe sur le vecteur vitesse robot (direction conservée pendant la rampe)
        dxy = target[:2] - self.v[:2]
        dist = float(np.hypot(*dxy))
        lim = self.accel * dt
        self.v[:2] = target[:2] if dist <= lim else self.v[:2] + dxy * (lim / dist)
        self.v[2] = self._approach(self.v[2], target[2], self.rot_accel * dt)
        return self.apply()

    def wheel_command(self):
        # "distances" par seconde = vitesses de roues en mm/s (dtheta = omega en rad/s)
        wheel_v = compute_wheel_distances_global(self.robot, self.v[0], self.v[1], self.v[2])
        cmds = []
        for v, m in zip(wheel_v, self.motors):
            rpm = int(round(abs(v) * m.pulses_per_mm * 60 / m.TICKS_PER_REVOLUTION))
            rpm = min(rpm, MAX_RPM)
            cmds.append((rpm, 1 if (v < 0 and rpm) else 0))   # 0 = CCW (= move_distance > 0)
        return cmds

    def apply(self):
        cmds = self.wheel_command()
        for i, (m, c) in enumerate(zip(self.motors, cmds)):
            if c != self._last_sent[i]:          # on n'envoie que les changements (charge CAN)
                m.move_velocity(c[0], self.mks_acc, direction=c[1])
                self._last_sent[i] = c
        return cmds

    def stop(self, acc=None):
        self.v[:] = 0
        for m in self.motors:
            m.move_velocity(0, self.mks_acc if acc is None else acc, direction=0)
        self._last_sent = [None] * len(self.motors)


def run(robot, motors):
    import os
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((460, 200))
    pygame.display.set_caption("Teleop ZQSD")
    font = pygame.font.SysFont(None, 26)
    drv = TeleopDriver(robot, motors)
    clock = pygame.time.Clock()
    running = True
    try:
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_SPACE:
                        drv.stop()
                    elif ev.key == pygame.K_UP:
                        drv.speed = min(1000.0, drv.speed + 100)
                    elif ev.key == pygame.K_DOWN:
                        drv.speed = max(10.0, drv.speed - 10)

            k = pygame.key.get_pressed()
            focused = pygame.key.get_focused()
            if focused:
                ay = (k[pygame.K_z] - k[pygame.K_s])
                ax = (k[pygame.K_d] - k[pygame.K_q])
                aw = (k[pygame.K_a] - k[pygame.K_e])   # A = anti-horaire (+)
            else:
                ax = ay = aw = 0        # perte de focus => on freine

            dt = clock.tick(LOOP_HZ) / 1000.0
            cmds = drv.step(ax, ay, aw, dt)

            screen.fill((20, 20, 24))
            lines = [f"vmax {drv.speed:.0f} mm/s   v=({drv.v[0]:.0f}, {drv.v[1]:.0f}) mm/s  w={drv.v[2]:.2f} rad/s",
                     "rpm/sens roues : " + "  ".join(f"{r}{'-' if d else '+'}" for r, d in cmds),
                     "ZQSD bouger | A/E tourner | haut/bas vitesse",
                     "ESPACE stop | ECHAP quitter"]
            for i, t in enumerate(lines):
                screen.blit(font.render(t, True, (230, 230, 230)), (12, 14 + 36 * i))
            pygame.display.flip()
    finally:
        drv.stop(255)       # freinage max quoi qu'il arrive
        pygame.quit()


if __name__ == "__main__":
    import can
    from mks_servo_can import MksServo
    from core.motor_controller import Motor
    gear_ratio = 48/15

    bus = can.interface.Bus(interface="socketcan", channel="can0")
    notifier = can.Notifier(bus, [])
    motors = [Motor(bus, i, MksServo(bus, notifier, i), gear_ratio=gear_ratio) for i in (1, 2, 3)]
    robot = Robot(pos=[0, 0], angle=-np.pi / 2, radius=150,
                  wheel_angles=[np.radians(120), np.radians(240), np.radians(0)])
    try:
        run(robot, motors)
    finally:
        notifier.stop()
        bus.shutdown()

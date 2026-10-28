"""Fusion de capteurs sur le robot : acquisition lidar (étape 0), prédiction de l'EKF par l'odométrie
(étape 2), association des poteaux (étape 3), correction de la pose à l'instant de chaque cluster avec gating
(étape 4), détection de perte et relocalisation par triangulation (étape 6), journal CSV (étape 7).

    python -m tests.test_fusion --x0 1000 --y0 1000 --theta0-deg 0
    python -m tests.test_fusion --x0 1000 --y0 1000 --theta0-deg 0 --drive   # + aller-retour de test_odometry
    python -m tests.test_fusion --x0 1000 --y0 1000 --theta0-deg 0 --log fusion.csv   # journal pour analyse

(x0, y0, theta0) = pose du robot sur le terrain au démarrage, repère du plan : origine sur le poteau
(0, 0), poteaux en (2000, 0) et (0, 2000), θ en sens anti-horaire, 0 = axe x du terrain.
Le dossier rplidarc1/ (avec t_rx_ns) doit être importable depuis la racine du dépôt.

L'acquisition lidar tourne dans son PROPRE PROCESSUS (core/lidar_process.py) : dans le même processus que l'affichage,
le filtre et l'odométrie, la lecture série était affamée (GIL) et le lidar n'atteignait plus 10 tours/s. Dans la ligne
« lidar : » affichée chaque seconde, « trou max » est le plus long silence entre deux points reçus : ~10 à 15 ms est
normal, des dizaines de ms ou plus signalent un lecteur qui n'a pas eu la main. --lidar-inline remet l'ancien mode
(même processus), utile seulement pour comparer.

Ce que le test affiche, dans l'ordre du plan :
  - étape 0 : une ligne par seconde (tours/s, points/s, tours rejetés, erreurs de la lib) ;
  - étape 3 : une ligne par tour (poteau associé ou non, écart mesure - attendu, incertitude).
Robot immobile, les 3 poteaux doivent être "ok" à chaque tour, avec des écarts de quelques mm / dixièmes
de degré. Les tours rejetés doivent rester à 0 et le temps de traitement très inférieur à 100 ms.
"""
import argparse
import math
import queue
import time

import numpy as np

from lidar.core.ekf import EkfParams
from lidar.core.lidar_acquisition import LidarAcquisition
from lidar.core.lidar_process import LidarProcess
from lidar.core.localizer import Localizer
from lidar.core.pole_association import AssocParams

import can
from mks_servo_can import MksServo
from base_holo.core.holo_base import Robot , compute_wheel_distances_global
from base_holo.core.holonomic_base import HolonomicBase
from base_holo.core.motor_controller import Motor, MotorGroup

TAGS = {"matched": "ok", "absent": "absent", "ambiguous": "ambigu"}

LOOP_HZ = 50
MAX_RPM = 3000


class TeleopDriver:
    """Transforme une commande (ax, ay, aw) ∈ [-1,1] en consignes moteurs, avec rampe LOGICIELLE
    sur la vitesse du ROBOT (donc les 3 roues restent proportionnelles à tout instant).
    Les accélérations MKS sont mises au maximum : c'est la rampe logicielle qui décide.
    """

    def __init__(self, robot, motors, speed_mm_s=60.0, rot_rad_s=1.0,
                 accel_mm_s2=1000.0, rot_accel_rad_s2=4.0, mks_acc=200):
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


def run(group):
    import os
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((460, 200))
    pygame.display.set_caption("Teleop ZQSD")
    font = pygame.font.SysFont(None, 26)
    drv = TeleopDriver(group.robot, group.motors)
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
                        drv.speed = max(10.0, drv.speed - 100)

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

            #x0, y0, th0, _ = base.pose()
            #total_dist, total_rot = base.odometry_stats()
            #print(f"Pose : ({x0:.1f}, {y0:.1f}) mm, θ = {math.degrees(th0):.2f}°,total dist = {total_dist:.1f} mm, total rot = {total_rot:.2f}°")
    finally:
        drv.stop(255)       # freinage max quoi qu'il arrive
        pygame.quit()

def build_base():
    """Même montage que tests/test_odometry.py."""


    gear_ratio = 48 / 15
    bus = can.interface.Bus(interface="socketcan", channel="can0")
    notifier = can.Notifier(bus, [])
    motors = [Motor(bus, i, MksServo(bus, notifier, i), gear_ratio=gear_ratio, notifier=notifier)
              for i in (1, 2, 3)]
    robot = Robot(pos=[0, 0], angle=0, radius=106.5,
                  wheel_angles=[np.radians(120), np.radians(240), np.radians(0)])
    group = MotorGroup(motors, robot)

    return bus, notifier, group, HolonomicBase(robot, group)


def format_report(r, ap=None, debug=False) -> str:
    """Formate un rapport d’association pour une lecture rapide dans le terminal."""
    parts = []
    for m in r.result.matches:
        if m.status == "matched":
            parts.append(f"P{m.pole + 1} ok d={m.d_dist_mm:+4.0f}mm a={m.d_angle_deg:+5.2f}deg")
        elif m.status == "ambiguous":
            parts.append(f"P{m.pole + 1} ambigu({m.n_candidates})")
        else:
            # attendu = où le filtre cherchait le poteau (distance au centre, angle lidar) : à comparer aux clusters vus
            parts.append(f"P{m.pole + 1} absent (attendu {m.exp_dist_mm:4.0f}mm {m.exp_angle_deg:6.1f}deg "
                         f"+-{m.win_dist_mm:3.0f}mm +-{m.win_angle_deg:4.1f}deg)")
    x, y, th = r.pose
    sx, sy, st = r.std
    extra = ""
    if debug and ap is not None:
        # clusters de la taille d'un poteau, tels que le lidar les voit (distance au centre = surface + 50 mm)
        seen = [f"{c.dist_mm + 50:4.0f}mm/{c.angle_deg:5.1f}deg(l={c.width_mm:3.0f})" for c in r.clusters
                if c.n_points >= ap.min_points and ap.width_min_mm <= c.width_mm <= ap.width_max_mm]
        extra = "\n      clusters taille poteau : " + (" ".join(seen) if seen else "aucun")
    mahal = f" ecart_norm {sum(r.mahal) / len(r.mahal):4.1f}" if r.mahal else ""
    state = " | RELOCALISE" if r.relocalized else (" | PERDU" if r.lost else (" | pose peu fiable" if not r.reliable else ""))
    return (f"[{time.strftime('%H:%M:%S', time.localtime(r.t_s))}] tour {r.rev_number:5d} | "
            f"{r.n_clusters:2d} clusters ({r.result.n_size_ok} taille poteau) | " + " | ".join(parts) +
            f" | corr {r.corrections_accepted} ok/{r.corrections_rejected} refusees{mahal}"
            f" | pose ({x:6.0f},{y:6.0f},{math.degrees(th):6.1f}deg) sigma ({sx:3.0f}mm,{sy:3.0f}mm,{st:4.1f}deg)"
            f" | {r.proc_ms:4.1f} ms" + (f" | HORS HISTORIQUE x{r.result.n_clamped}" if r.result.n_clamped else "")
            + state + extra)



def points_angle_dist(pts):
    """(angle_deg, dist_mm) d'un tour brut (rev.points).

    POINT_DTYPE est construit à partir de tuples (angle, distance, qualité, temps) dans lidar_acquisition : les
    deux premiers champs sont donc l'angle lidar (deg, sens horaire) et la distance (mm), quels que soient leurs noms.
    Un tableau 2D simple (N, >=2) est aussi accepté.
    """
    if pts is None or len(pts) == 0:
        return np.empty(0), np.empty(0)
    if pts.dtype.names:
        ang = pts[pts.dtype.names[0]].astype(float)
        dist = pts[pts.dtype.names[1]].astype(float)
    else:
        ang, dist = pts[:, 0].astype(float), pts[:, 1].astype(float)
    ok = dist > 0
    return ang[ok], dist[ok]


def run_visualizer(report_queue, poles_mm, pole_radius_mm=50.0, lidar_offset_mm=(0.0, 0.0),
                   lidar_zero_deg=0.0, stop_event=None, group=None):
    """Affiche la fusion lidar et, si un groupe moteur est fourni, intègre la téléopération dans la même fenêtre.

    Une seule boucle Pygame est utilisée : cela évite d'ouvrir/initialiser Pygame depuis plusieurs threads.
    """
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError("L'affichage demande pygame : installez-le avec `pip install pygame`.") from exc

    pygame.init()
    width, height = 1100, 800
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Fusion lidar — test_fusion")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 23)
    small = pygame.font.Font(None, 19)
    driver = TeleopDriver(group.robot, group.motors) if group is not None else None

    # Le repère terrain est en millimètres ; Y est inversé à l'écran.
    margin = 260.0
    xs = [p[0] for p in poles_mm]
    ys = [p[1] for p in poles_mm]
    xmin, xmax = min(0.0, min(xs)) - margin, max(xs) + margin
    ymin, ymax = min(0.0, min(ys)) - margin, max(ys) + margin
    scale = min((width - 100) / (xmax - xmin), (height - 130) / (ymax - ymin))
    ox = (width - (xmax - xmin) * scale) / 2 - xmin * scale
    oy = height - 75 + ymin * scale

    def xy(point):
        return int(ox + point[0] * scale), int(oy - point[1] * scale)

    def line(color, p1, p2, thickness=2):
        pygame.draw.line(screen, color, xy(p1), xy(p2), thickness)

    latest = None
    show_points = True      # touche P : affiche / masque le nuage de points brut du lidar
    running = True
    # Profilage léger : moyennes/maxima sur une fenêtre glissante d'environ 1 s.
    profile_names = ("events", "teleop", "queue", "draw", "flip", "frame")
    profile = {name: {"sum": 0.0, "max": 0.0, "n": 0} for name in profile_names}
    profile_started = time.perf_counter()
    frame_count = 0
    last_queue_items = 0
    displayed_profile = {name: (0.0, 0.0) for name in profile_names}
    displayed_fps = 0.0

    def record_timing(name, elapsed):
        item = profile[name]
        item["sum"] += elapsed
        item["max"] = max(item["max"], elapsed)
        item["n"] += 1

    while running and not (stop_event is not None and stop_event.is_set()):
        frame_started = time.perf_counter()
        space_pressed = False
        t0 = time.perf_counter()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_p:
                    show_points = not show_points
                elif driver is not None and event.key == pygame.K_SPACE:
                    space_pressed = True
                elif driver is not None and event.key == pygame.K_UP:
                    driver.speed = min(1000.0, driver.speed + 100.0)
                elif driver is not None and event.key == pygame.K_DOWN:
                    driver.speed = max(10.0, driver.speed - 100.0)
        record_timing("events", time.perf_counter() - t0)

        # La limite FPS est isolée du reste : son temps inclut l'attente éventuelle.
        dt = clock.tick(30) / 1000.0
        if driver is not None:
            t0 = time.perf_counter()
            keys = pygame.key.get_pressed()
            focused = pygame.key.get_focused()
            if space_pressed:
                driver.stop()
            if focused and running:
                ay = float(keys[pygame.K_z]) - float(keys[pygame.K_s])
                ax = float(keys[pygame.K_d]) - float(keys[pygame.K_q])
                aw = float(keys[pygame.K_a]) - float(keys[pygame.K_e])
            else:
                ax = ay = aw = 0.0
            if running:
                cmds = driver.step(ax, ay, aw, dt)
            else:
                driver.stop()
                cmds = [(0, 0)] * len(driver.motors)
            record_timing("teleop", time.perf_counter() - t0)
        else:
            cmds = None
            record_timing("teleop", 0.0)

        t0 = time.perf_counter()
        queue_items = 0
        try:
            while True:
                latest = report_queue.get_nowait()
                queue_items += 1
        except queue.Empty:
            pass
        last_queue_items = queue_items
        record_timing("queue", time.perf_counter() - t0)

        t0 = time.perf_counter()
        screen.fill((248, 249, 251))
        # Grille de 500 mm pour faciliter la lecture des coordonnées.
        gx = math.floor(xmin / 500) * 500
        while gx <= xmax:
            line((225, 229, 234), (gx, ymin), (gx, ymax), 1)
            label = small.render(str(int(gx)), True, (100, 105, 115))
            screen.blit(label, (xy((gx, ymin))[0] - 10, height - 35))
            gx += 500
        gy = math.floor(ymin / 500) * 500
        while gy <= ymax:
            line((225, 229, 234), (xmin, gy), (xmax, gy), 1)
            gy += 500

        # Poteaux théoriques fixes du modèle.
        for i, pole in enumerate(poles_mm):
            px, py = xy(pole)
            pygame.draw.circle(screen, (35, 105, 210), (px, py), max(5, int(pole_radius_mm * scale)), 2)
            pygame.draw.circle(screen, (35, 105, 210), (px, py), 3)
            screen.blit(font.render(f"P{i + 1} théorique ({pole[0]:.0f}, {pole[1]:.0f})", True,
                                    (25, 75, 150)), (px + 10, py - 23))

        if latest is not None:
            x, y, theta = latest.pose
            robot_xy = (x, y)
            # Position du lidar : décalage exprimé dans le repère du robot puis tourné dans le terrain.
            ox_l, oy_l = lidar_offset_mm
            lidar_xy = (x + ox_l * math.cos(theta) - oy_l * math.sin(theta),
                        y + ox_l * math.sin(theta) + oy_l * math.cos(theta))
            # Nuage de points brut du dernier tour (touche P). Angle lidar horaire -> terrain : theta + zero - angle.
            # La pose est celle de fin de traitement du tour : robot en mouvement, le nuage est légèrement déformé.
            if show_points and latest.points is not None:
                ang_deg, dist_mm = points_angle_dist(latest.points)
                if len(ang_deg):
                    a_t = theta + np.deg2rad(lidar_zero_deg - ang_deg)
                    sx_pts = (ox + (lidar_xy[0] + dist_mm * np.cos(a_t)) * scale).astype(int)
                    sy_pts = (oy - (lidar_xy[1] + dist_mm * np.sin(a_t)) * scale).astype(int)
                    for u, v in zip(sx_pts.tolist(), sy_pts.tolist()):
                        if 0 <= u < width and 0 <= v < height:
                            pygame.draw.circle(screen, (70, 120, 200), (u, v), 2)
            # Corps du robot et direction avant (theta est anti-horaire dans le repère terrain).
            rp = xy(robot_xy)
            pygame.draw.circle(screen, (55, 65, 75), rp, max(8, int(106.5 * scale)), 2)
            heading = (x + 180 * math.cos(theta), y + 180 * math.sin(theta))
            line((45, 50, 60), robot_xy, heading, 4)
            pygame.draw.circle(screen, (45, 50, 60), rp, 5)
            screen.blit(font.render(f"Robot ({x:.0f}, {y:.0f}) mm  θ={math.degrees(theta):.1f}°", True,
                                    (35, 40, 48)), (15, 12))

            for match in latest.result.matches:
                # La fenêtre de recherche est dessinée depuis la position du lidar estimée.
                # L'angle du lidar est horaire, d'où theta - angle dans le repère terrain.
                alpha = math.radians(match.exp_angle_deg)
                a_center = theta + math.radians(lidar_zero_deg) - alpha
                r0 = max(0.0, match.exp_dist_mm - match.win_dist_mm)
                r1 = match.exp_dist_mm + match.win_dist_mm
                a0 = theta + math.radians(lidar_zero_deg - match.exp_angle_deg + match.win_angle_deg)
                a1 = theta + math.radians(lidar_zero_deg - match.exp_angle_deg - match.win_angle_deg)
                # Deux bords angulaires + segment radial central : la zone entre eux est la fenêtre.
                p0 = (lidar_xy[0] + r0 * math.cos(a0), lidar_xy[1] + r0 * math.sin(a0))
                p1 = (lidar_xy[0] + r1 * math.cos(a0), lidar_xy[1] + r1 * math.sin(a0))
                q0 = (lidar_xy[0] + r0 * math.cos(a1), lidar_xy[1] + r0 * math.sin(a1))
                q1 = (lidar_xy[0] + r1 * math.cos(a1), lidar_xy[1] + r1 * math.sin(a1))
                color = (220, 145, 30) if match.status != "matched" else (30, 160, 85)
                line(color, p0, p1, 2)
                line(color, q0, q1, 2)
                line(color, p1, q1, 1)
                expected = (lidar_xy[0] + match.exp_dist_mm * math.cos(a_center),
                            lidar_xy[1] + match.exp_dist_mm * math.sin(a_center))
                line((120, 125, 135), lidar_xy, expected, 1)
                pygame.draw.circle(screen, color, xy(expected), 4)
                if match.status == "matched" and match.cluster is not None:
                    # Position du centre du poteau observé, reconstruite depuis la mesure de surface.
                    dist = match.cluster.dist_mm + pole_radius_mm
                    a_real = theta + math.radians(lidar_zero_deg - match.cluster.angle_deg)
                    real = (lidar_xy[0] + dist * math.cos(a_real), lidar_xy[1] + dist * math.sin(a_real))
                    line((210, 45, 65), lidar_xy, real, 2)
                    pygame.draw.circle(screen, (210, 45, 65), xy(real), 7, 2)
                    pygame.draw.line(screen, (210, 45, 65), (xy(real)[0] - 5, xy(real)[1]),
                                     (xy(real)[0] + 5, xy(real)[1]), 2)
                    pygame.draw.line(screen, (210, 45, 65), (xy(real)[0], xy(real)[1] - 5),
                                     (xy(real)[0], xy(real)[1] + 5), 2)

            # Légende et état d'association.
            legend = [((35, 105, 210), "Poteau théorique"), ((120, 125, 135), "Axe attendu / recherche"),
                      ((220, 145, 30), "Fenêtre de recherche"), ((210, 45, 65), "Position réellement détectée"),
                      ((30, 160, 85), "Poteau associé")]
            yy = 42
            for color, label in legend:
                pygame.draw.line(screen, color, (16, yy + 7), (42, yy + 7), 3)
                screen.blit(small.render(label, True, (45, 50, 58)), (50, yy))
                yy += 21
            screen.blit(small.render(f"[P] nuage de points lidar : {'ON' if show_points else 'OFF'}", True,
                                     (45, 50, 58)), (16, yy + 4))
            statuses = "   ".join(f"P{m.pole + 1}: {TAGS.get(m.status, m.status)}" for m in latest.result.matches)
            screen.blit(font.render(statuses, True, (45, 50, 58)), (15, height - 28))
        else:
            screen.blit(font.render("En attente du premier tour lidar…", True, (45, 50, 58)), (15, 12))

        if driver is not None:
            control_text = (f"Teleop: ZQSD déplacer | A/E tourner | ESPACE stop | ↑/↓ vitesse "
                            f"({driver.speed:.0f} mm/s) | roues: " +
                            " ".join(f"{rpm}{'-' if direction else '+'}" for rpm, direction in cmds))
            screen.blit(small.render(control_text, True, (35, 40, 48)), (15, height - 52))

        # Mesure du rendu avant d'ajouter le panneau de profilage.
        draw_elapsed = time.perf_counter() - t0
        record_timing("draw", draw_elapsed)

        # Publie les statistiques chaque seconde environ, sans écrire sur la console à chaque frame.
        frame_count += 1
        now = time.perf_counter()
        window_elapsed = now - profile_started
        if window_elapsed >= 1.0:
            displayed_fps = frame_count / window_elapsed
            displayed_profile = {
                name: ((v["sum"] / v["n"] * 1000.0) if v["n"] else 0.0, v["max"] * 1000.0)
                for name, v in profile.items()
            }
            profile = {name: {"sum": 0.0, "max": 0.0, "n": 0} for name in profile_names}
            profile_started = now
            frame_count = 0

        report_age_ms = (max(0.0, time.time() - latest.t_s) * 1000.0) if latest is not None else float("inf")
        panel_lines = [
            f"UI {displayed_fps:.1f} FPS | rapports reçus: {last_queue_items}",
            f"âge rapport: {report_age_ms:.0f} ms | localizer: {latest.odo_hz:.1f} odo Hz" if latest is not None else "En attente de rapport",
            "Calcul par tour (ms): cap / clusters / association / total",
            (f"{latest.timings_ms['cap_points']:.2f} / {latest.timings_ms['find_clusters']:.2f} / "
             f"{latest.timings_ms['associate_poles']:.2f} / {latest.timings_ms['total']:.2f}") if latest is not None else "--",
            (f"corrections: {latest.corrections_accepted} ok / {latest.corrections_rejected} refusées | "
             + ("RELOCALISÉ" if latest.relocalized else "PERDU" if latest.lost
                else "pose fiable" if latest.reliable else "pose peu fiable")) if latest is not None else "--",
            "temps UI en ms : moyenne / max (~1 s)",
        ]
        for name in ("events", "teleop", "queue", "draw", "flip", "frame"):
            avg_ms, max_ms = displayed_profile[name]
            panel_lines.append(f"{name:>6}: {avg_ms:6.2f} / {max_ms:6.2f}")
        panel_lines.append("tick(30) attend pour limiter les FPS; non inclus dans draw")
        panel = pygame.Surface((430, 262), pygame.SRCALPHA)
        panel.fill((255, 255, 255, 225))
        for i, text_line in enumerate(panel_lines):
            panel.blit(small.render(text_line, True, (20, 25, 32)), (8, 5 + i * 19))
        screen.blit(panel, (0, 500))

        t0 = time.perf_counter()
        pygame.display.flip()
        record_timing("flip", time.perf_counter() - t0)
        record_timing("frame", time.perf_counter() - frame_started)

    if driver is not None:
        driver.stop()
    pygame.quit()


def main():
    """Configure puis exécute le scénario principal du programme."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x0", type=float, required=True, help="x initial du robot sur le terrain (mm)")
    ap.add_argument("--y0", type=float, required=True, help="y initial (mm)")
    ap.add_argument("--theta0-deg", type=float, required=True, help="cap initial (deg, anti-horaire)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--duration", type=float, default=600.0, help="durée du test (s)")
    ap.add_argument("--every", type=int, default=5, help="affiche 1 tour sur N (défaut 5 = 2 lignes/s)")
    ap.add_argument("--odometry-hz", type=float, default=200.0)
    ap.add_argument("--drive", action="store_true", help="lance l'aller-retour de test_odometry.check_pose")
    ap.add_argument("--no-robot", action="store_true",
                    help="sans moteurs ni CAN : lidar + EKF immobile. Sert à savoir si les moteurs perturbent le lidar")
    ap.add_argument("--min-points", type=int, default=300,
                    help="points minimum par tour pour le publier (un C1 à 10 Hz en donne ~500)")
    ap.add_argument("--lidar-inline", action="store_true",
                    help="lecture du lidar dans CE processus (ancien mode, sensible à la charge) : pour comparer seulement")
    ap.add_argument("--lidar-cpu", type=int, default=None, metavar="N",
                    help="épingle le processus lidar sur le coeur N (ex. 3 sur un Raspberry Pi 4) ; sans effet avec --lidar-inline")
    ap.add_argument("--init-std-mm", type=float, default=100.0,
                    help="incertitude de départ sur x et y (écart-type, mm). Plus elle est grande, plus la fenêtre de "
                         "recherche des poteaux est large (3 sigma, bornée par --win-dist-max-mm)")
    ap.add_argument("--init-std-deg", type=float, default=5.0, help="incertitude de départ sur le cap (écart-type, deg)")
    ap.add_argument("--win-dist-max-mm", type=float, default=600.0, help="demi-fenêtre maximale en distance (mm)")
    ap.add_argument("--win-angle-max-deg", type=float, default=25.0, help="demi-fenêtre maximale en angle (deg)")
    ap.add_argument("--win-scale", type=float, default=AssocParams().window_scale,
                    help="coefficient appliqué à la fenêtre de recherche des poteaux, maxima compris (défaut 3 ; 1 = plan "
                         "d'origine). Plus large : rattrape un filtre trop sûr de lui, mais plus de poteaux ambigus")
    ap.add_argument("--lidar-zero-deg", type=float, default=0.0,
                    help="angle lu par le lidar quand il vise l'axe x du robot (à mesurer ; une erreur ici décale tous les poteaux)")
    ap.add_argument("--lidar-offset-x", type=float, default=0.0,
                    help="position du lidar dans le repère robot, x (mm), vers la roue 3 ; à mesurer depuis l'axe de rotation. "
                         "Une erreur ici est invisible à l'arrêt mais fait refuser les corrections dès que le robot tourne")
    ap.add_argument("--lidar-offset-y", type=float, default=0.0, help="idem, y (mm), à 90° de x (anti-horaire)")
    ap.add_argument("--lidar-latency-s", type=float, default=0.0,
                    help="retard (s) entre la mesure d'un point et son horodatage de réception (essayer 0.01 à 0.03) ; "
                         "une erreur décale le cap estimé de (vitesse de rotation x erreur) pendant les rotations")
    ap.add_argument("--no-reloc", action="store_true",
                    help="désactive la détection de perte / relocalisation par triangulation (test : l'odométrie est fiable, "
                         "on vérifie que la divergence ne vient pas d'une relocalisation fausse)")
    ap.add_argument("--debug", action="store_true", help="affiche aussi les clusters de la taille d'un poteau vus par le lidar")
    ap.add_argument("--visualize", action="store_true", help="ouvre une fenêtre pygame de visualisation en direct")
    ap.add_argument("--log", default=None, metavar="FICHIER.csv",
                    help="journal CSV : une ligne par tour (pose, incertitude, corrections, écart normalisé, perte)")
    ap.add_argument("--pose-only", action="store_true",
                    help="écrit uniquement timestamp,x,y,theta à chaque nouvelle pose d’odométrie (sans graphique ni logs)")
    args = ap.parse_args()
    if args.no_robot and args.drive:
        ap.error("--drive demande le robot : incompatible avec --no-robot")
    if args.pose_only and args.visualize:
        ap.error("--pose-only est incompatible avec --visualize")

    if args.no_robot:
        bus = notifier = group = base = None
        pose_source = lambda: (0.0, 0.0, 0.0, time.time())      # odométrie immobile : l'EKF reste à la pose initiale
    else:
        bus, notifier, group, base = build_base()
        pose_source = base.pose
    if args.lidar_inline:
        acq = LidarAcquisition(args.port, min_raw_points=args.min_points)
    else:
        acq = LidarProcess(args.port, min_raw_points=args.min_points, cpu=args.lidar_cpu)
    loc = None
    ekf_params = EkfParams(init_pos_std_mm=args.init_std_mm, init_theta_std_deg=args.init_std_deg,
                           lidar_zero_deg=args.lidar_zero_deg,
                           lidar_offset_mm=(args.lidar_offset_x, args.lidar_offset_y),
                           lidar_latency_s=args.lidar_latency_s,
                           **({"lost_after_s": 1e9} if args.no_reloc else {}))
    assoc = AssocParams(win_dist_max_mm=args.win_dist_max_mm, win_angle_max_deg=args.win_angle_max_deg,
                        window_scale=args.win_scale)

    visual_queue = queue.Queue(maxsize=1) if args.visualize else None

    def on_pose(ts, pose):
        if args.pose_only:
            x, y, theta = pose
            print(f"{ts:.6f},{x:.3f},{y:.3f},{math.degrees(theta):.4f}", flush=True)

    def on_report(r):
        """Affiche le rapport de traitement d’un tour et publie le dernier pour l’interface."""
        if args.pose_only:
            return
        if r.rev_number % args.every == 0:
            print(format_report(r, assoc, args.debug))
        if visual_queue is not None:
            # L'interface ne doit jamais ralentir le thread de fusion : on ne conserve que le dernier rapport.
            try:
                visual_queue.put_nowait(r)
            except queue.Full:
                try:
                    visual_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    visual_queue.put_nowait(r)
                except queue.Full:
                    pass

    try:
        # Ordre de démarrage : odométrie d'abord (la pose existe avant le premier tour de lidar), puis le
        # lidar (thread de lecture), puis la fusion (thread qui consomme les tours).
        if base is not None:
            base.start_odometry(args.odometry_hz)
        time.sleep(0.3)
        acq.start()
        if not args.lidar_inline and not args.pose_only:
            reglages = {k: v for k, v in acq.info.items() if k != "pid"}
            print(f"Acquisition lidar : processus {acq.pid} dédié" + (f" ({reglages})" if reglages else ""))
        loc = Localizer(pose_source, acq.rev_queue, (args.x0, args.y0, math.radians(args.theta0_deg)),
                        params=ekf_params, assoc=assoc, on_report=on_report, on_pose=on_pose, log_path=args.log)
        loc.start()
        if loc.log_error is not None:
            print(f"Journal impossible ({args.log}) : {loc.log_error!r} : le test continue sans journal.")
        if args.pose_only:
            print("timestamp_s,x_mm,y_mm,theta_deg", flush=True)
        else:
            print(f"Pose initiale : ({args.x0:.0f} mm, {args.y0:.0f} mm, {args.theta0_deg:.1f} deg), "
                  f"incertitude {args.init_std_mm:.0f} mm / {args.init_std_deg:.1f} deg")
        
        import threading
        stop_event = threading.Event()
        t_end = time.time() + args.duration

        def monitor_test():
            """Affiche les statistiques et diagnostique explicitement la cause d'arrêt."""
            reason = None
            while not stop_event.is_set():
                # Tester les causes d'arrêt séparément permet de ne pas masquer
                # une exception derrière la simple fermeture de la fenêtre.
                if loc.error is not None:
                    reason = f"erreur du thread de fusion : {loc.error!r}"
                    break
                if acq.error is not None:
                    reason = f"exception du thread lidar : {acq.error!r}"
                    break
                if not acq.alive:
                    reason = "le thread d'acquisition lidar n'est plus actif, sans exception enregistrée"
                    break
                if time.time() >= t_end:
                    reason = f"durée du test atteinte ({args.duration:.1f} s)"
                    break

                time.sleep(1.0)
                s = acq.snapshot()
                # pts/tour : ~500 attendus (C1, 10 Hz, 5000 pts/s).
                if not args.pose_only:
                    print(f"  lidar : {s['freq_hz']:5.2f} tours/s | {s['pts_per_s']:5.0f} pts/s | {s['pts_per_rev']:4.0f} pts/tour | "
                          f"vus {s['revs_seen']} publies {s['revs_published']} rejetes {s['revs_rejected']} "
                          f"perdus {s['revs_dropped']} | erreurs lib {s['lib_errors']} | rejets {s['reject_reasons'] or '-'} | "
                          f"trou max {s['max_gap_ms']:3.0f} ms" + (f" | cpu lidar {s['cpu_pct']:3.0f} %" if 'cpu_pct' in s else ""))
            
                print(f"Erreur du thread de fusion : {loc.error}")
                
                if args.drive:
                            from base_holo.tests.test_odometry import check_pose
                            check_pose(base)

            if reason is not None and not stop_event.is_set():
                if not args.pose_only:
                    print(f"\n[DIAGNOSTIC] Arrêt du test : {reason}", flush=True)
                if acq.error is not None and not args.pose_only:
                    import traceback
                    print("[DIAGNOSTIC] Traceback du thread lidar :", flush=True)
                    traceback.print_exception(type(acq.error), acq.error, acq.error.__traceback__)
                if loc.error is not None and not args.pose_only:
                    import traceback
                    print("[DIAGNOSTIC] Traceback du thread de fusion :", flush=True)
                    traceback.print_exception(type(loc.error), loc.error, loc.error.__traceback__)
            stop_event.set()

        monitor_thread = threading.Thread(target=monitor_test, daemon=True, name="fusion-monitor")
        monitor_thread.start()

        if args.visualize:
            # Une seule boucle Pygame dans le thread principal : visualisation + téléopération.
            # Ne pas appeler run() ici : run() crée sa propre boucle Pygame et bloque l'exécution.
            run_visualizer(
                visual_queue,
                loc.ekf.p.poles_mm,
                loc.ekf.p.pole_radius_mm,
                loc.ekf.p.lidar_offset_mm,
                loc.ekf.p.lidar_zero_deg,
                stop_event,
                group=group,
            )
            stop_event.set()  # fermeture de la fenêtre : demande l'arrêt du test
        elif group is not None:
            # Sans visualisation de fusion, conserver la fenêtre de téléopération existante.
            run(group)
            stop_event.set()
        else:
            monitor_thread.join()

        stop_event.set()
        monitor_thread.join()
       
        if not args.pose_only and acq.error is not None:
            print(f"Erreur du thread lidar : {acq.error!r}")
        if not args.pose_only and loc.error is not None:
            print(f"Erreur du thread de fusion : {loc.error!r}")
        if not args.pose_only and (acq.clock.recalibrated or loc.odo_clock.recalibrated):
            print("Note : horloge(s) non epoch recalée(s) sur l'heure système "
                  f"(lidar : {acq.clock.recalibrated}, odométrie : {loc.odo_clock.recalibrated}).")
    except KeyboardInterrupt:
        pass
    finally:
        if loc is not None:
            loc.stop()
        acq.stop()
        if base is not None:
            group.stop_all(255)
            base.stop_odometry()
            group.close()
            notifier.stop()
            bus.shutdown()


if __name__ == "__main__":
    main()

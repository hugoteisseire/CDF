"""Programme robot réel : pilotage moteur, acquisition lidar et localisation EKF.

Lancement depuis la racine du dépôt :
    python -m lidar.tests.robot_fusion --x0 1000 --y0 1000 --theta0-deg 0
    python -m lidar.tests.robot_fusion --x0 1000 --y0 1000 --theta0-deg 0 --visualize   # + fenêtre pygame de debug

--visualize ouvre la fenêtre pygame de test_fusion (poteaux, fenêtres de recherche, nuage de points, balise
adverse, profilage) en AFFICHAGE SEUL : le pilotage reste celui de la page web / de la stratégie, il n'y a
aucun pilotage clavier. Fermer la fenêtre (ECHAP ou croix) ne commande rien au robot.

Le programme remplace viz_table/example_robot.py : il expose le même serveur
RobotProtocolServer à la stratégie, mais applique les commandes aux moteurs réels.
Angles du protocole en degrés, angles internes du robot en radians.
"""
import argparse
import math
import queue
import threading
import time

import numpy as np
import can
from mks_servo_can import MksServo

from base_holo.core.holo_base import Robot
from base_holo.core.holonomic_base import HolonomicBase
from base_holo.core.motor_controller import Motor, MotorGroup
from lidar.core.ekf import EkfParams
from lidar.core.lidar_process import LidarProcess
from lidar.core.localizer import Localizer
from lidar.core.thread_monitor import ThreadCpuMonitor
from lidar.core.pole_association import AssocParams
from viz_table.robot_protocol import RobotProtocolServer
from viz_table.viz_client import Viz


GEAR_RATIO = 48 / 15
WHEEL_RADIUS_MM = 106.5
WHEEL_ANGLES = [np.radians(120), np.radians(240), np.radians(0)]
CONTROL_HZ = 200            # aligné sur l'odométrie (200 Hz)
STATE_PUBLISH_HZ = 50       # remontée d'état stratégie / visualiseur (indépendante du contrôle)

# Recalage périodique de l'odométrie (qui dérive) sur la pose EKF (juste et précise).
ODO_RESYNC_PERIOD_S = 1.0

# Rotation simple sur place (fin de goto avec angle d'arrivée, ou commande rotate).
ROTATE_TOL_DEG = 1.0        # arrêt dès que |erreur| <= tolérance
ROTATE_KP = 3.0             # (deg/s) par degré d'erreur
ROTATE_MIN_DEG_S = 15.0     # vitesse mini pour ne pas caler
ROTATE_MAX_DEG_S = 120.0
ROTATE_TIMEOUT_S = 15.0     # sécurité : abandon si l'angle n'est pas atteint

# Goto vectoriel simplifié (200 Hz). A chaque tick : direction vers la cible, vecteur vitesse
# dans cette direction (repère terrain ; l'angle du robot est pris en compte par
# base.set_velocity via l'odométrie), pendant que le robot tourne sur lui-même.
# Pas de cap final asservi : on s'arrête dès que la cible est atteinte à GOTO_POS_TOL_MM.
# La seule accélération appliquée est la rampe interne des moteurs (ramp_acc).
GOTO_POS_TOL_MM = 10.0
GOTO_TIMEOUT_MARGIN_S = 8.0
GOTO_LAG_S = 0.1             # retard consigne -> mouvement réel : la position est prédite avec lui
GOTO_VEL_FILTER_ALPHA = 0.25  # filtre de la vitesse estimée par l'odométrie (par échantillon)
GOTO_BRAKE_MARGIN = 0.7      # freinage planifié à 70 % de l'accélération demandée
GOTO_MIN_SPEED_MM_S = 40.0   # vitesse plancher pour ne pas s'arrêter avant la tolérance

# Rotation pendant le déplacement : démarre à START, descend linéairement avec la distance
# restante (rapportée à la distance initiale) jusqu'à END à l'arrivée.
GOTO_SPIN_START_RAD_S = 1.5
GOTO_SPIN_END_RAD_S = 0.5
GOTO_SPIN_DIR = 1.0          # +1 anti-horaire, -1 horaire

# PID de régulation de la vitesse de rotation (commutable). Désactivé : consigne = profil.
# Activé : consigne = profil + Kp*erreur + Ki*intégrale, erreur = profil - vitesse mesurée.
GOTO_SPIN_PID_ENABLED = True
GOTO_SPIN_KP = 0.5
GOTO_SPIN_KI = 0.8
GOTO_SPIN_I_MAX_RAD_S = 0.3  # contribution max de l'intégrateur


def build_base(odometry_hz: float):
    """Construit la base CAN et sa façade de commande/odométrie."""
    bus = can.interface.Bus(interface="socketcan", channel="can0")
    notifier = can.Notifier(bus, [])
    motors = [
        Motor(bus, i, MksServo(bus, notifier, i), gear_ratio=GEAR_RATIO, notifier=notifier)
        for i in (1, 2, 3)
    ]
    robot = Robot(pos=[0, 0], angle=0, radius=WHEEL_RADIUS_MM, wheel_angles=WHEEL_ANGLES)
    group = MotorGroup(motors, robot)
    base = HolonomicBase(robot, group, watchdog_s=0.3, odometry_hz=odometry_hz)
    return bus, notifier, group, base


def _wrap_rad(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class RobotController:
    """Traduit les commandes du protocole en appels HolonomicBase."""

    def __init__(self, base: HolonomicBase, localizer: Localizer):
        self.base = base
        self.localizer = localizer
        self.mode = "idle"
        self.target = None
        self.target_theta = None
        self._velocity = (0.0, 0.0, 0.0, "robot")
        self._velocity_deadline = None      # monotonic ; None = consigne sans timeout
        self._lock = threading.Lock()
        self._last_state_sample = None
        self._last_fusion = {
            "ready": False, "reliable": False, "lost": False,
            "std_x": None, "std_y": None, "std_theta": None,
            "revs_processed": 0, "revs_skipped": 0,
            "acq_to_correction_ms": None,
            "acq_to_correction_avg_5s_ms": None, "poles": [],
        }
        self.motion_status = "idle"
        self.motion_error = None
        self._opponent = {"valid": False}
        self._rotate_started = 0.0
        # État interne du goto vectoriel.
        self._goto_started = 0.0
        self._goto_timeout = 0.0
        self._goto_speed = 0.0
        self._goto_accel = 0.0
        self._goto_ramp_acc = 220
        self._goto_pid = GOTO_SPIN_PID_ENABLED
        self._reset_goto_state()
        self._last_odo_resync = time.monotonic()

    def _reset_goto_state(self):
        """Remet à zéro l'estimateur de vitesse et l'intégrateur (appeler sous verrou ou à l'init)."""
        self._goto_last_tick = None
        self._goto_prev = None              # (t_odo, x, y, theta) du dernier échantillon utilisé
        self._goto_v_est = (0.0, 0.0, 0.0)  # vx, vy (mm/s), omega (rad/s) mesurés, filtrés
        self._goto_i_w = 0.0                # intégrale de l'erreur de vitesse de rotation (rad)
        self._goto_d0 = 1.0                 # distance initiale à la cible (mm)

    def stop(self):
        self.base.stop(255)
        with self._lock:
            self.mode = "idle"
            self.target = None
            self.target_theta = None
            self._velocity = (0.0, 0.0, 0.0, "robot")
            self._velocity_deadline = None
            self._reset_goto_state()
            self.motion_status = "stopped"
            self.motion_error = None

    def goto(self, x: float, y: float, speed_mm_s: float, acc_mm_s2: float,
             theta_deg=None, pid=None, fastest_boost: int = -2):
        """Goto vers une position terrain.

        Sans theta_deg : conserve le déplacement borné en boucle ouverte.
        Avec theta_deg (valeur non utilisée comme cap final) : goto vectoriel simplifié,
        le robot tourne sur lui-même pendant le déplacement (voir GOTO_SPIN_*).
        pid : True/False force le PID de rotation pour ce goto ; None = GOTO_SPIN_PID_ENABLED.
        """
        self.stop()
        # La translation se réfère à l'odométrie, plus réactive que la pose fusionnée.
        px, py, current_theta, _ = self.base.pose()
        dx, dy = x - px, y - py
        distance = math.hypot(dx, dy)

        if speed_mm_s <= 0:
            raise ValueError("La vitesse du goto doit être strictement positive")
        if acc_mm_s2 <= 0:
            raise ValueError("L'accélération du goto doit être strictement positive")

        speed_limit = min(abs(speed_mm_s), self.base.max_speed_mm_s)
        pulses_acc = acc_mm_s2 * self.base.motors[0].pulses_per_mm
        ramp_acc = max(1, min(255, Motor.linear_to_acc(pulses_acc)))

        # Pas d'angle demandé : on garde exactement le goto position précédent.
        if theta_deg is None:
            if distance <= 1.0:
                with self._lock:
                    self.motion_status = "reached"
                return
            angle = math.atan2(dy, dx)
            try:
                ok = self.base.move_direction(
                    distance, angle,
                    speed_mm_s=speed_limit,
                    dtheta=0.0, ramp_acc=ramp_acc, fastest_boost=fastest_boost,
                    wait=False, verbose=False,
                )
                if not ok:
                    raise RuntimeError("La commande de déplacement n'a pas pu être lancée")
            except Exception as exc:
                with self._lock:
                    self.motion_status = "error"
                    self.motion_error = str(exc)
                raise
            with self._lock:
                self.mode = "goto"
                self.target = (x, y)
                self.target_theta = None
                self.motion_status = "running"
                self.motion_error = None
            return

        # Goto vectoriel simplifié : le cap final n'est pas asservi.
        if distance <= GOTO_POS_TOL_MM:
            with self._lock:
                self.motion_status = "reached"
            return

        translation_time = (distance / max(50.0, speed_limit)
                            + speed_limit / float(acc_mm_s2))
        timeout = max(ROTATE_TIMEOUT_S, 3.0 * translation_time + GOTO_TIMEOUT_MARGIN_S)

        with self._lock:
            self.mode = "goto_vector"
            self.target = (float(x), float(y))
            self.target_theta = float(theta_deg)
            self._reset_goto_state()
            self._goto_d0 = distance
            self._goto_pid = GOTO_SPIN_PID_ENABLED if pid is None else bool(pid)
            self._goto_last_tick = time.monotonic()
            self._goto_started = self._goto_last_tick
            self._goto_timeout = timeout
            self._goto_speed = speed_limit
            self._goto_accel = float(acc_mm_s2)
            self._goto_ramp_acc = ramp_acc
            self.motion_status = "running"
            self.motion_error = None

    def rotate(self, theta_deg: float):
        """Rotation simple sur place jusqu'à l'angle absolu theta_deg (repère terrain)."""
        self.stop()
        self._start_rotation(theta_deg)

    def _start_rotation(self, theta_deg: float):
        """Passe en mode rotation ; la boucle tick() pilote ensuite la vitesse angulaire."""
        theta = self.base.pose()[2]
        error = _wrap_rad(math.radians(theta_deg) - theta)
        with self._lock:
            if abs(error) <= math.radians(ROTATE_TOL_DEG):
                self.mode = "idle"
                self.target = None
                self.target_theta = None
                self.motion_status = "reached"
            else:
                self.mode = "rotate"
                self.target_theta = theta_deg
                self._rotate_started = time.monotonic()
                self.motion_status = "running"
            self.motion_error = None

    def apply(self, cmd: dict):
        kind = cmd["type"]
        if kind == "cmd.stop":
            self.stop()
        elif kind == "cmd.reset_pose":
            self.stop()
            theta = math.radians(cmd["theta"])
            self.base.reset_pose(cmd["x"], cmd["y"], theta)
            x, y, _, t = self.base.pose()
            if t is None:
                t = time.perf_counter()
            ts = self.localizer.odo_clock.epoch_s(t)
            with self.localizer._lock:
                self.localizer.ekf.reset(cmd["x"], cmd["y"], theta, ts)
            with self._lock:
                self.mode = "idle"
        elif kind == "cmd.velocity":
            # La consigne continue est réappliquée à chaque cycle de contrôle (200 Hz).
            with self._lock:
                self.mode = "velocity"
                self.target = None
                self.target_theta = None
                self._velocity = (cmd["vx"], cmd["vy"], cmd["omega"], cmd["frame"])
                # timeout optionnel (s) : sans nouvelle consigne avant l'échéance, tick() arrête le robot.
                timeout = cmd.get("timeout")
                self._velocity_deadline = (time.monotonic() + float(timeout)) if timeout else None
                self.motion_status = "running"
                self.motion_error = None
        elif kind == "cmd.goto":
            self.goto(cmd["x"], cmd["y"], cmd["speed"], cmd["acc"], cmd.get("theta"),
                      cmd.get("pid"))
        elif kind == "cmd.rotate":
            self.rotate(cmd["theta"])

    def tick(self):
        with self._lock:
            mode = self.mode
            vx, vy, omega, frame = self._velocity
            deadline = self._velocity_deadline
        if mode == "velocity" and deadline is not None and time.monotonic() > deadline:
            self.stop()
            with self._lock:
                self.motion_error = "velocity : consignes interrompues (timeout)"
            mode = "idle"
        if mode == "velocity":
            if frame == "robot":
                theta = self.base.pose()[2]
                c, s = math.cos(theta), math.sin(theta)
                vx, vy = c * vx - s * vy, s * vx + c * vy
            self.base.set_velocity(vx, vy, math.radians(omega))

        elif mode == "goto":
            if all(not m.is_running() for m in self.base.motors):
                with self._lock:
                    self.mode = "idle"
                    self.target = None
                    self.motion_status = "reached"
                    self.motion_error = None

        elif mode == "goto_vector":
            self._tick_goto_vector()

        elif mode == "rotate":
            self._tick_rotate()

        self._resync_odometry()

    @staticmethod
    def _estimate_velocity(px, py, theta, t_odo, prev, v_est):
        """Vitesse mesurée (mm/s, rad/s) par différences finies sur l'odométrie, filtrée.

        N'avance que sur un nouvel échantillon odométrique (horodatage strictement croissant) :
        à 200 Hz de contrôle pour 200 Hz d'odométrie, un tick peut relire le même échantillon.
        """
        if prev is None:
            return (t_odo, px, py, theta), v_est
        dt_o = t_odo - prev[0]
        if dt_o <= 1e-4:
            return prev, v_est
        a = GOTO_VEL_FILTER_ALPHA
        raw = ((px - prev[1]) / dt_o,
               (py - prev[2]) / dt_o,
               _wrap_rad(theta - prev[3]) / dt_o)
        v_est = tuple(e + a * (r - e) for e, r in zip(v_est, raw))
        return (t_odo, px, py, theta), v_est

    def _tick_goto_vector(self):
        """Goto vectoriel simplifié, exécuté à 200 Hz.

        1. Position prédite = position odométrique + vitesse mesurée x GOTO_LAG_S.
        2. Vecteur vitesse dans la direction de la cible (repère terrain), recalculé à chaque
           tick ; sa norme suit un freinage en sqrt(2.a.d), l'accélération étant celle de la
           rampe moteur (ramp_acc).
        3. Rotation de GOTO_SPIN_START_RAD_S vers GOTO_SPIN_END_RAD_S selon la distance restante,
           régulée par un PID commutable sur la vitesse de rotation mesurée.
        4. Arrêt dès que la position prédite est dans la tolérance.
        """
        now = time.monotonic()
        px, py, theta, t_odo = self.base.pose()
        if t_odo is None:
            t_odo = time.perf_counter()

        with self._lock:
            target = self.target
            last_tick = self._goto_last_tick
            started = self._goto_started
            timeout = self._goto_timeout
            vmax = self._goto_speed
            accel = self._goto_accel
            ramp_acc = self._goto_ramp_acc
            use_pid = self._goto_pid
            prev = self._goto_prev
            v_est = self._goto_v_est
            i_w = self._goto_i_w
            d0 = max(self._goto_d0, 1.0)

        if target is None:
            return

        if now - started > timeout:
            self.base.stop(255)
            with self._lock:
                self.mode = "idle"
                self.target = None
                self.target_theta = None
                self._reset_goto_state()
                self.motion_status = "error"
                self.motion_error = "goto vectoriel : cible non atteinte (timeout)"
            return

        dt = 1.0 / CONTROL_HZ if last_tick is None else now - last_tick
        dt = min(0.1, max(0.001, dt))

        # Position prédite à l'instant où la consigne d'aujourd'hui fera effet.
        prev, v_est = self._estimate_velocity(px, py, theta, t_odo, prev, v_est)
        vxm, vym, wm = v_est
        dx = target[0] - (px + vxm * GOTO_LAG_S)
        dy = target[1] - (py + vym * GOTO_LAG_S)
        dist = math.hypot(dx, dy)

        if dist <= GOTO_POS_TOL_MM:
            self.base.stop(255)
            with self._lock:
                if self.mode == "goto_vector":
                    self.mode = "idle"
                    self.target = None
                    self.target_theta = None
                    self._reset_goto_state()
                    self.motion_status = "reached"
                    self.motion_error = None
            return

        # Vecteur vitesse vers la cible, freinage planifié sur la distance prédite.
        braking = math.sqrt(2.0 * accel * GOTO_BRAKE_MARGIN * dist)
        speed = min(vmax, max(GOTO_MIN_SPEED_MM_S, braking))
        vx = speed * dx / dist
        vy = speed * dy / dist

        # Rotation : START loin de la cible, END à l'arrivée.
        ratio = min(1.0, dist / d0)
        w_ref = GOTO_SPIN_END_RAD_S + (GOTO_SPIN_START_RAD_S - GOTO_SPIN_END_RAD_S) * ratio
        w_max = self.base.max_omega_rad_s
        if use_pid:
            err = w_ref - wm * GOTO_SPIN_DIR
            w_cmd = w_ref + GOTO_SPIN_KP * err + GOTO_SPIN_KI * i_w
            w_cmd = min(w_cmd, GOTO_SPIN_START_RAD_S)    # jamais au-dessus de la vitesse de départ
            if 0.0 < w_cmd < min(w_max, GOTO_SPIN_START_RAD_S):   # anti-windup : pas si saturé
                i_w += err * dt
                i_lim = GOTO_SPIN_I_MAX_RAD_S / GOTO_SPIN_KI if GOTO_SPIN_KI > 0.0 else 0.0
                i_w = max(-i_lim, min(i_lim, i_w))
        else:
            i_w = 0.0
            w_cmd = w_ref
        omega = GOTO_SPIN_DIR * max(0.0, min(w_max, w_cmd))

        with self._lock:
            # N'écrase pas un arrêt ou une nouvelle commande reçue pendant le calcul.
            if self.mode != "goto_vector":
                return
            self._goto_prev = prev
            self._goto_v_est = v_est
            self._goto_i_w = i_w
            self._goto_last_tick = now

        # Consigne envoyée telle quelle : l'accélération est bornée par la rampe moteur seule.
        self.base.set_velocity(vx, vy, omega, ramp_acc=ramp_acc)

    def _tick_rotate(self):
        """Tourne sur place tant que l'angle cible n'est pas atteint, puis s'arrête."""
        with self._lock:
            target_deg = self.target_theta
            started = self._rotate_started
        if target_deg is None:
            return
        theta = self.base.pose()[2]
        error = _wrap_rad(math.radians(target_deg) - theta)
        if abs(error) <= math.radians(ROTATE_TOL_DEG):
            self.base.stop(255)
            with self._lock:
                self.mode = "idle"
                self.target = None
                self.target_theta = None
                self.motion_status = "reached"
                self.motion_error = None
        elif time.monotonic() - started > ROTATE_TIMEOUT_S:
            self.base.stop(255)
            with self._lock:
                self.mode = "idle"
                self.target = None
                self.target_theta = None
                self.motion_status = "error"
                self.motion_error = "rotation : angle non atteint (timeout)"
        else:
            speed = min(ROTATE_MAX_DEG_S, max(ROTATE_MIN_DEG_S, ROTATE_KP * abs(math.degrees(error))))
            self.base.set_velocity(0.0, 0.0, math.radians(math.copysign(speed, error)))

    def _resync_odometry(self):
        """Recale l'odométrie sur l'EKF, sauf pendant un goto asservi sur l'odométrie."""
        with self._lock:
            if self.mode == "goto_vector":
                return
        now = time.monotonic()
        if now - self._last_odo_resync < ODO_RESYNC_PERIOD_S:
            return
        self._last_odo_resync = now
        self.localizer.resync_odometry(self.base.reset_pose)

    def update_fusion_report(self, report):
        """Conserve les diagnostics lidar et la détection de la balise adverse."""
        matches = []
        for match in report.result.matches:
            matches.append({
                "pole": match.pole + 1,
                "status": match.status,
                "distance_mm": match.cluster.dist_mm if match.cluster else None,
                "angle_deg": match.cluster.angle_deg if match.cluster else None,
            })
        status = self.localizer.status()
        sx, sy, st = status["std"]
        opponent = find_opponent_cluster(
            report.clusters, report.pose, report.result.matches,
            self.localizer.ekf.p.pole_radius_mm,
            self.localizer.ekf.p.lidar_offset_mm,
            self.localizer.ekf.p.lidar_zero_deg,
        )
        opponent_state = {"valid": False, "in_zone": True}
        if opponent is not None:
            (op_x, op_y), op_cluster = opponent
            opponent_state = {
                "valid": True, "in_zone": False, "x": op_x, "y": op_y,
                "width_mm": op_cluster.width_mm,
                "n_points": op_cluster.n_points,
            }
        with self._lock:
            self._opponent = opponent_state
            self._last_fusion = {
                "ready": status["ready"], "reliable": status["reliable"],
                "lost": status["lost"], "std_x": sx, "std_y": sy,
                "std_theta": math.degrees(st),
                "revs_processed": status["revs_processed"],
                "revs_skipped": status["revs_skipped"],
                "acq_to_correction_ms": status["acq_to_correction_ms"],
                "acq_to_correction_avg_5s_ms": status["acq_to_correction_avg_5s_ms"],
                "poles": matches, "processing_ms": report.proc_ms,
                "ekf_ms": report.timings_ms.get("ekf_total", report.timings_ms.get("correct", 0.0)),
                "corrections_accepted": report.corrections_accepted,
                "corrections_rejected": report.corrections_rejected,
            }

    def state(self):
        status = self.localizer.status()
        x, y, theta = status["pose"]
        now = time.perf_counter()
        vx = vy = omega = 0.0
        if self._last_state_sample is not None:
            last_t, last_x, last_y, last_theta = self._last_state_sample
            dt = now - last_t
            if dt > 1e-4:
                vx, vy = (x - last_x) / dt, (y - last_y) / dt
                omega = math.degrees(_wrap_rad(theta - last_theta) / dt)
        self._last_state_sample = (now, x, y, theta)
        with self._lock:
            mode, target, target_theta = self.mode, self.target, self.target_theta
        return {
            "robot": {"x": x, "y": y, "theta": math.degrees(theta),
                      "vx": vx, "vy": vy, "omega": omega},
            "opponent": dict(self._opponent),
            "motion": {"mode": mode, "status": self.motion_status,
                       "error": self.motion_error,
                       "target_x": target[0] if target else None,
                       "target_y": target[1] if target else None,
                       "target_theta": target_theta,
                       "moving": mode != "idle"},
            "fusion": dict(self._last_fusion),
        }


OPPONENT_IGNORE_ZONE = (0.0, 500.0, 800.0, 1500.0)  # xmin, xmax, ymin, ymax (mm)
OPPONENT_BEACON_DIAMETER_MM = 100.0
OPPONENT_BEACON_WIDTH_MIN_MM = 0.0
OPPONENT_BEACON_WIDTH_MAX_MM = 250.0
OPPONENT_BEACON_MIN_POINTS = 2
# La balise adverse n'est cherchée qu'à l'intérieur de la table réduite de OPPONENT_FIELD_INSET_MM sur
# chaque bord : les clusters hors table (public, bordures, poteaux mal associés) sont ignorés, et un
# robot collé au bord de la table n'est volontairement pas pris en compte.
OPPONENT_FIELD_MM = (0.0, 0.0, 3000.0, 2000.0)   # xmin, ymin, xmax, ymax (mm)
OPPONENT_FIELD_INSET_MM = 200.0                    # réglable avec --opponent-inset-mm


def opponent_search_zone(field_mm=None, inset_mm=None):
    """Rectangle (xmin, ymin, xmax, ymax) en mm où un centre de balise adverse est accepté."""
    xmin, ymin, xmax, ymax = OPPONENT_FIELD_MM if field_mm is None else field_mm
    m = OPPONENT_FIELD_INSET_MM if inset_mm is None else inset_mm
    return xmin + m, ymin + m, xmax - m, ymax - m


def find_opponent_cluster(clusters, pose, matched_poles, pole_radius_mm=50.0,
                          lidar_offset_mm=(0.0, 0.0), lidar_zero_deg=0.0,
                          field_mm=None, inset_mm=None):
    """Retourne la position terrain et le cluster de la balise adverse, ou None.

    Seuls les clusters dont le centre estimé est dans la table réduite de inset_mm
    (OPPONENT_FIELD_INSET_MM par défaut) sur chaque bord sont candidats.
    """
    zxmin, zymin, zxmax, zymax = opponent_search_zone(field_mm, inset_mm)
    x, y, theta = pose
    ox, oy = lidar_offset_mm
    lidar_x = x + ox * math.cos(theta) - oy * math.sin(theta)
    lidar_y = y + ox * math.sin(theta) + oy * math.cos(theta)
    matched_ids = {id(match.cluster) for match in matched_poles if match.cluster is not None}
    candidates = []

    for cluster in clusters:
        if id(cluster) in matched_ids:
            continue
        width = float(cluster.width_mm)
        if not (OPPONENT_BEACON_WIDTH_MIN_MM <= width <= OPPONENT_BEACON_WIDTH_MAX_MM):
            continue
        if cluster.n_points < OPPONENT_BEACON_MIN_POINTS:
            continue

        angle = theta + math.radians(lidar_zero_deg - cluster.angle_deg)
        center_dist = cluster.dist_mm + OPPONENT_BEACON_DIAMETER_MM / 2.0
        position = (
            lidar_x + center_dist * math.cos(angle),
            lidar_y + center_dist * math.sin(angle),
        )
        if not (zxmin <= position[0] <= zxmax and zymin <= position[1] <= zymax):
            continue                    # hors de la table réduite : pas un adversaire exploitable
        score = (abs(width - OPPONENT_BEACON_DIAMETER_MM), -cluster.n_points)
        candidates.append((score, position, cluster))

    if not candidates:
        return None
    _, position, cluster = min(candidates, key=lambda item: item[0])
    return position, cluster


def _format_poles(report):
    states = []
    for match in report.result.matches:
        name = f"P{match.pole + 1}"
        if match.status == "matched":
            states.append(f"{name}:OK")
        elif match.status == "ambiguous":
            states.append(f"{name}:AMBIG")
        else:
            states.append(f"{name}:ABS")
    return " ".join(states)


TAGS = {"matched": "ok", "absent": "absent", "ambiguous": "ambigu"}


def run_visualizer(report_queue, poles_mm, pole_radius_mm=50.0, lidar_offset_mm=(0.0, 0.0),
                   lidar_zero_deg=0.0, stop_event=None):
    """Fenêtre pygame de debug de la fusion lidar (affichage seul, aucun pilotage).

    Doit tourner dans le thread principal (SDL/pygame). La fenêtre se ferme avec ECHAP ou la croix ;
    elle s'arrête aussi si stop_event est levé. Fermer la fenêtre ne commande rien au robot.
    """
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError("L'affichage demande pygame : installez-le avec `pip install pygame`.") from exc

    pygame.init()
    width, height = 1100, 800
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Fusion lidar — robot_fusion (debug)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 23)
    small = pygame.font.Font(None, 19)

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
    profile_names = ("events", "queue", "draw", "flip", "frame")
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

    try:
        while running and not (stop_event is not None and stop_event.is_set()):
            frame_started = time.perf_counter()
            t0 = time.perf_counter()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_p:
                        show_points = not show_points
            record_timing("events", time.perf_counter() - t0)

            # La limite FPS est isolée du reste : son temps inclut l'attente éventuelle.
            clock.tick(30)

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

            # Zone attribuée à l'adversaire lorsqu'aucune balise n'est détectée.
            zone_xmin, zone_xmax, zone_ymin, zone_ymax = OPPONENT_IGNORE_ZONE
            zone_top_left = xy((zone_xmin, zone_ymax))
            zone_bottom_right = xy((zone_xmax, zone_ymin))
            zone_rect = pygame.Rect(zone_top_left[0], zone_top_left[1],
                                    zone_bottom_right[0] - zone_top_left[0],
                                    zone_bottom_right[1] - zone_top_left[1])
            pygame.draw.rect(screen, (245, 190, 55), zone_rect, 2)
            screen.blit(small.render("Zone adverse (si balise non détectée)", True, (155, 105, 15)),
                        (zone_rect.x + 4, zone_rect.y + 4))

            # Zone où la balise adverse est cherchée : table réduite de OPPONENT_FIELD_INSET_MM.
            sxmin, symin, sxmax, symax = opponent_search_zone()
            if sxmax > sxmin and symax > symin:
                s_tl, s_br = xy((sxmin, symax)), xy((sxmax, symin))
                search_rect = pygame.Rect(s_tl[0], s_tl[1], s_br[0] - s_tl[0], s_br[1] - s_tl[1])
                pygame.draw.rect(screen, (200, 120, 190), search_rect, 1)
                search_label = small.render(f"Recherche adversaire (table - {OPPONENT_FIELD_INSET_MM:.0f} mm)",
                                            True, (150, 60, 140))
                screen.blit(search_label, (search_rect.right - search_label.get_width() - 4,
                                           search_rect.bottom - search_label.get_height() - 4))

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
                # Nuage filtré : on dessine les impacts projetés au même instant que le filtre,
                # pas en réutilisant la pose courante (ce qui déformait le nuage en mouvement).
                if show_points and latest.points_world_mm is not None:
                    for hit_x, hit_y in latest.points_world_mm:
                        u, v = xy((float(hit_x), float(hit_y)))
                        if 0 <= u < width and 0 <= v < height:
                            pygame.draw.circle(screen, (70, 120, 200), (u, v), 2)
                # Corps du robot et direction avant (theta est anti-horaire dans le repère terrain).
                rp = xy(robot_xy)
                pygame.draw.circle(screen, (55, 65, 75), rp, max(8, int(WHEEL_RADIUS_MM * scale)), 2)
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
                    # Fenêtre = secteur d'anneau [r0, r1] x [exp ± win] centré sur la mesure attendue.
                    # (Avant : deux bords radiaux seuls ; une fenêtre très large, ±60° et ±1500 mm quand
                    # la covariance a explosé dans une direction, devenait deux segments loin du poteau.)
                    n_arc = 32
                    arc = [a1 + (a0 - a1) * i / n_arc for i in range(n_arc + 1)]
                    sector = ([(lidar_xy[0] + r1 * math.cos(a), lidar_xy[1] + r1 * math.sin(a)) for a in arc]
                              + [(lidar_xy[0] + r0 * math.cos(a), lidar_xy[1] + r0 * math.sin(a))
                                 for a in reversed(arc)])
                    pygame.draw.lines(screen, color, True, [xy(q) for q in sector], 2)
                    expected = (lidar_xy[0] + match.exp_dist_mm * math.cos(a_center),
                                lidar_xy[1] + match.exp_dist_mm * math.sin(a_center))
                    line((120, 125, 135), lidar_xy, expected, 1)
                    pygame.draw.circle(screen, color, xy(expected), 4)
                    ex, ey = xy(expected)
                    screen.blit(small.render(f"P{match.pole + 1} ±{match.win_dist_mm:.0f} mm ±{match.win_angle_deg:.0f}°",
                                             True, color), (ex + 8, ey + 6))
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

                # Balise adverse : uniquement lorsqu'un cluster correspond aux critères.
                opponent = find_opponent_cluster(
                    latest.clusters, latest.pose, latest.result.matches, pole_radius_mm,
                    lidar_offset_mm, lidar_zero_deg,
                )
                if opponent is not None:
                    opponent_xy, opponent_cluster = opponent
                    opponent_screen = xy(opponent_xy)
                    opponent_color = (180, 35, 145)
                    pygame.draw.circle(screen, opponent_color, opponent_screen,
                                       max(8, int((OPPONENT_BEACON_DIAMETER_MM / 2) * scale)), 3)
                    pygame.draw.line(screen, opponent_color, (opponent_screen[0] - 8, opponent_screen[1]),
                                     (opponent_screen[0] + 8, opponent_screen[1]), 2)
                    pygame.draw.line(screen, opponent_color, (opponent_screen[0], opponent_screen[1] - 8),
                                     (opponent_screen[0], opponent_screen[1] + 8), 2)
                    label = (f"Balise adverse ({opponent_xy[0]:.0f}, {opponent_xy[1]:.0f}) mm "
                             f"| largeur {opponent_cluster.width_mm:.0f} mm")
                    screen.blit(small.render(label, True, opponent_color),
                                (opponent_screen[0] + 10, opponent_screen[1] + 8))
                else:
                    zone_center = ((zone_xmin + zone_xmax) / 2.0, (zone_ymin + zone_ymax) / 2.0)
                    zone_screen = xy(zone_center)
                    screen.blit(small.render("Adversaire considéré dans sa zone (balise non détectée)",
                                             True, (155, 105, 15)),
                                (zone_screen[0] + 8, zone_screen[1]))

                # Légende et état d'association.
                legend = [((35, 105, 210), "Poteau théorique"), ((120, 125, 135), "Axe attendu / recherche"),
                          ((220, 145, 30), "Fenêtre de recherche"), ((210, 45, 65), "Position réellement détectée"),
                          ((30, 160, 85), "Poteau associé"), ((180, 35, 145), "Adversaire détecté"),
                          ((180, 35, 145), "Balise adverse détectée")]
                yy = 42
                for color, label in legend:
                    pygame.draw.line(screen, color, (16, yy + 7), (42, yy + 7), 3)
                    screen.blit(small.render(label, True, (45, 50, 58)), (50, yy))
                    yy += 21
                screen.blit(small.render(f"[P] nuage de points lidar : {'ON' if show_points else 'OFF'}", True,
                                         (45, 50, 58)), (16, yy + 4))
                screen.blit(small.render(
                    f"Points filtrés: {len(latest.points_world_mm) if latest.points_world_mm is not None else 0} | "
                    f"horodatage non fiable: {latest.points_unverified} | "
                    f"marge hors zone: {getattr(latest, 'field_margin_mm', 0.0):.0f} mm", True, (45, 50, 58)),
                    (16, yy + 24))
                statuses = "   ".join(f"P{m.pole + 1}: {TAGS.get(m.status, m.status)}" for m in latest.result.matches)
                screen.blit(font.render(statuses, True, (45, 50, 58)), (15, height - 28))
            else:
                screen.blit(font.render("En attente du premier tour lidar…", True, (45, 50, 58)), (15, 12))

            screen.blit(small.render("Fenêtre de debug : affichage seul (pilotage via la page web) | ECHAP fermer",
                                     True, (35, 40, 48)), (15, height - 52))

            # Mesure du rendu avant d'ajouter le panneau de profilage.
            record_timing("draw", time.perf_counter() - t0)

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
                "Calcul pur par tour (ms, hors acquisition):",
                (f"cap {latest.timings_ms['cap_points']:.2f} | filtre terrain {latest.timings_ms['filter_points_in_field']:.2f} | "
                 f"clusters {latest.timings_ms['find_clusters']:.2f}") if latest is not None else "--",
                (f"association {latest.timings_ms['associate_poles']:.2f} | corrections EKF {latest.timings_ms['correct']:.2f} | "
                 f"relocalisation {latest.timings_ms['relocalize']:.2f}") if latest is not None else "--",
                (f"TOTAL CALCUL {latest.timings_ms['total']:.2f} ms") if latest is not None else "--",
                (f"Latence depuis début du tour (inclut acquisition + file + calcul): {latest.acq_to_correction_ms:.0f} ms "
                 f"| moyenne 5 s: {latest.acq_to_correction_avg_5s_ms:.0f} ms") if latest is not None else "--",
                (f"corrections: {latest.corrections_accepted} ok / {latest.corrections_rejected} refusées | "
                 + ("RELOCALISÉ" if latest.relocalized else "PERDU" if latest.lost
                    else "pose fiable" if latest.reliable else "pose peu fiable")) if latest is not None else "--",
                "temps UI en ms : moyenne / max (~1 s)",
            ]
            for name in profile_names:
                avg_ms, max_ms = displayed_profile[name]
                panel_lines.append(f"{name:>6}: {avg_ms:6.2f} / {max_ms:6.2f}")
            panel_lines.append("tick(30) attend pour limiter les FPS; non inclus dans draw")
            panel = pygame.Surface((470, 330), pygame.SRCALPHA)
            panel.fill((255, 255, 255, 225))
            for i, text_line in enumerate(panel_lines):
                panel.blit(small.render(text_line, True, (20, 25, 32)), (8, 5 + i * 19))
            screen.blit(panel, (0, 470))

            t0 = time.perf_counter()
            pygame.display.flip()
            record_timing("flip", time.perf_counter() - t0)
            record_timing("frame", time.perf_counter() - frame_started)
    finally:
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x0", type=float, required=True, help="X initial terrain (mm)")
    parser.add_argument("--y0", type=float, required=True, help="Y initial terrain (mm)")
    parser.add_argument("--theta0-deg", type=float, required=True, help="cap initial (degrés)")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--odometry-hz", type=float, default=200.0)
    parser.add_argument("--lidar-cpu", type=int, default=None)
    parser.add_argument("--min-points", type=int, default=300)
    parser.add_argument("--lidar-zero-deg", type=float, default=0.0)
    parser.add_argument("--lidar-offset-x", type=float, default=0.0)
    parser.add_argument("--lidar-offset-y", type=float, default=0.0)
    parser.add_argument("--lidar-latency-s", type=float, default=0.0)
    parser.add_argument("--init-std-mm", type=float, default=20.0)
    parser.add_argument("--init-std-deg", type=float, default=1.0)
    parser.add_argument("--win-dist-max-mm", type=float, default=600.0)
    parser.add_argument("--win-angle-max-deg", type=float, default=25.0)
    parser.add_argument("--win-scale", type=float, default=AssocParams().window_scale)
    parser.add_argument("--opponent-inset-mm", type=float, default=OPPONENT_FIELD_INSET_MM,
                        help="balise adverse cherchée seulement dans la table réduite de cette marge (mm)")
    parser.add_argument("--socket", default=None, help="chemin socket stratégie (défaut CDF_ROBOT_SOCKET)")
    parser.add_argument("--visualize", action="store_true",
                        help="ouvre la fenêtre pygame de debug fusion (affichage seul : le pilotage reste "
                             "celui de la page web / de la stratégie)")
    args = parser.parse_args()
    globals()["OPPONENT_FIELD_INSET_MM"] = args.opponent_inset_mm   # utilisé par find_opponent_cluster

    if args.visualize:
        # Échec immédiat, avant de démarrer moteurs et lidar, si pygame est absent.
        try:
            import pygame  # noqa: F401
        except ImportError:
            parser.error("--visualize demande pygame : installez-le avec `pip install pygame`.")

    bus = notifier = group = base = acq = loc = server = controller = None
    control_thread = None
    stop_event = threading.Event()
    control_error = []      # exception de la boucle de contrôle, relancée dans le thread principal
    visual_queue = queue.Queue(maxsize=1) if args.visualize else None
    visual_active = threading.Event()   # levé tant que la fenêtre de debug est ouverte
    if args.visualize:
        visual_active.set()

    def publish_report(report):
        """Transmet le dernier rapport à la fenêtre de debug sans jamais ralentir le thread de fusion."""
        if visual_queue is None or not visual_active.is_set():
            return
        try:
            visual_queue.put_nowait(report)
        except queue.Full:
            try:
                visual_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                visual_queue.put_nowait(report)
            except queue.Full:
                pass

    try:
        bus, notifier, group, base = build_base(args.odometry_hz)
        # L'odométrie conserve le cap absolu initial : les déplacements globaux
        # de HolonomicBase sont ainsi exprimés dans le repère terrain.
        base.reset_pose(0.0, 0.0, math.radians(args.theta0_deg))
        base.start()
        time.sleep(0.3)

        # Retour au réglage par défaut : réduire cette file n'a pas corrigé le retard observé.
        acq = LidarProcess(args.port, min_raw_points=args.min_points, cpu=args.lidar_cpu)
        acq.start()

        ekf_params = EkfParams(
            init_pos_std_mm=args.init_std_mm,
            init_theta_std_deg=args.init_std_deg,
            lidar_zero_deg=args.lidar_zero_deg,
            lidar_offset_mm=(args.lidar_offset_x, args.lidar_offset_y),
            lidar_latency_s=args.lidar_latency_s,
        )
        assoc = AssocParams(
            win_dist_max_mm=args.win_dist_max_mm,
            win_angle_max_deg=args.win_angle_max_deg,
            window_scale=args.win_scale,
        )

        controller = None

        def on_report(report):
            if controller is not None:
                controller.update_fusion_report(report)
            ekf_ms = report.timings_ms.get("ekf_total", report.timings_ms.get("correct", 0.0))
            print(
                f"Tour {report.rev_number:5d} | {_format_poles(report)} | "
                f"calcul {report.proc_ms:5.1f} ms | EKF {ekf_ms:5.1f} ms",
                flush=True,
            )
            publish_report(report)

        loc = Localizer(
            base.pose, acq.rev_queue,
            (args.x0, args.y0, math.radians(args.theta0_deg)),
            params=ekf_params, assoc=assoc, on_report=on_report,
        )
        loc.start()
        # Diagnostic : %CPU de chaque thread toutes les 2 s (qui prend la main au localizer ?)
        ThreadCpuMonitor(period_s=2.0, extra_pids={"lidar(fils)": acq.pid}).start()
        controller = RobotController(base, loc)
        viz = Viz("robot")
        viz.on("goto", lambda args: controller.goto(
            float(args["x"]), float(args["y"]),
            float(args.get("speed", 300.0)), float(args.get("acc", 500.0)),
            args.get("theta"), args.get("pid"), int(args.get("fastest_boost", -2)),
        ))
        viz.on("rotate", lambda args: controller.rotate(float(args["theta"])))
        viz.on("estop", lambda args: controller.stop())
        # Chase the Mouse : consigne de vitesse continue (mm/s, °/s) renvoyée ~20 Hz par la page ;
        # timeout borné à [0.1, 2] s : le robot s'arrête seul si la page cesse d'envoyer.
        viz.on("velocity", lambda args: controller.apply({
            "type": "cmd.velocity",
            "vx": float(args.get("vx", 0.0)), "vy": float(args.get("vy", 0.0)),
            "omega": float(args.get("omega", 0.0)),
            "frame": "robot" if args.get("frame") == "robot" else "field",
            "timeout": min(2.0, max(0.1, float(args.get("timeout", 0.5)))),
        }))
        server_kwargs = {} if args.socket is None else {"path": args.socket}
        server = RobotProtocolServer(controller.state, **server_kwargs).start()

        print(
            f"Robot prêt | pose initiale ({args.x0:.0f}, {args.y0:.0f}, "
            f"{args.theta0_deg:.1f}°) | stratégie + visualiseur actifs"
            + (" | fenêtre de debug ouverte" if args.visualize else ""),
            flush=True,
        )

        def control_loop():
            """Boucle de contrôle 200 Hz : commandes stratégie/web, moteurs ; état remonté à 50 Hz.

            Elle tourne dans un thread dédié pour laisser le thread principal à pygame (--visualize).
            """
            try:
                period = 1.0 / CONTROL_HZ
                publish_period = 1.0 / STATE_PUBLISH_HZ
                next_tick = time.monotonic()
                next_publish = next_tick
                while not stop_event.is_set():
                    while True:
                        event = server.poll(timeout=0.0)
                        if event is None:
                            break
                        if event.get("type") == "connection":
                            if not event["connected"]:
                                controller.stop()
                            else:
                                controller.stop()  # reconnexion : aucune ancienne consigne ne repart
                        else:
                            controller.apply(event)

                    controller.tick()

                    # La remontée d'état (socket + WebSocket) est découplée du contrôle : elle coûte
                    # plus qu'un tick de 5 ms et ne doit pas faire déraper la boucle à 200 Hz.
                    if time.monotonic() >= next_publish:
                        next_publish += publish_period
                        # Même snapshot fusionné envoyé à la stratégie (socket) et au visualiseur (WebSocket).
                        snapshot = controller.state()
                        robot_state = snapshot["robot"]
                        motion_state = snapshot["motion"]
                        fusion_state = snapshot["fusion"]
                        viz.send("robot", {
                            **robot_state,
                            "radius": base.robot.radius,
                            "target": ({"x": motion_state["target_x"], "y": motion_state["target_y"]}
                                       if motion_state["target_x"] is not None else None),
                            "target_theta": motion_state["target_theta"],
                            "mode": motion_state["mode"], "moving": motion_state["moving"],
                            "motion_status": motion_state["status"],
                            "motion_error": motion_state["error"],
                            "reliable": fusion_state["reliable"], "lost": fusion_state["lost"],
                            "std": {"x": fusion_state["std_x"], "y": fusion_state["std_y"],
                                    "theta": fusion_state["std_theta"]},
                        })
                        viz.send("opponent", snapshot["opponent"])
                        viz.send("fusion", fusion_state)

                    if loc.error is not None:
                        raise RuntimeError(f"Erreur localizer: {loc.error!r}")
                    if acq.error is not None:
                        raise RuntimeError(f"Erreur acquisition lidar: {acq.error!r}")
                    next_tick += period
                    late = time.monotonic() - next_tick
                    if late > 5 * period:
                        # Gros retard (GC, I/O) : on se recale au lieu de rattraper en rafale.
                        next_tick += late
                        next_publish = max(next_publish, time.monotonic())
                    time.sleep(max(0.0, next_tick - time.monotonic()))
            except Exception as exc:
                control_error.append(exc)
            finally:
                stop_event.set()    # fin de la boucle (erreur ou arrêt) : ferme aussi la fenêtre de debug

        control_thread = threading.Thread(target=control_loop, name="robot-control", daemon=True)
        control_thread.start()

        if args.visualize:
            # pygame dans le thread principal ; la fusion et le contrôle continuent dans leurs threads.
            run_visualizer(
                visual_queue,
                loc.ekf.p.poles_mm,
                loc.ekf.p.pole_radius_mm,
                loc.ekf.p.lidar_offset_mm,
                loc.ekf.p.lidar_zero_deg,
                stop_event,
            )
            visual_active.clear()
            if not stop_event.is_set():
                print("Fenêtre de debug fermée : le robot reste actif (Ctrl+C pour arrêter).", flush=True)

        while control_thread.is_alive():
            control_thread.join(0.2)
        if control_error:
            raise control_error[0]

    except KeyboardInterrupt:
        print("\nArrêt demandé.", flush=True)
    finally:
        stop_event.set()
        visual_active.clear()
        if control_thread is not None:
            control_thread.join(timeout=2.0)
        if controller is not None:
            controller.stop()
        elif base is not None:
            base.stop(255)
        if server is not None:
            server.close()
        if loc is not None:
            loc.stop()
        if acq is not None:
            acq.stop()
        if base is not None:
            base.stop_odometry()
        if group is not None:
            group.close()
        if notifier is not None:
            notifier.stop()
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()
"""Façade de pilotage de la base holonome (couche au-dessus de Robot et MotorGroup)."""
import math
import threading
import time

import numpy as np

from .holo_base import Robot
from .motor_controller import MotorGroup


class HolonomicBase:
    """
    Point d'entrée unique de la couche basse : A*, teleop, tests et futur asservissement
    lidar/vision passeront par cet objet.

    Cycle de vie : start() / close() (ou `with HolonomicBase(...) as base:`).
    Commandes : set_velocity, set_velocity_local, move_velocity_direction (continues, à rappeler à
    chaque cycle), move_direction, rotate (mouvements bornés), stop.
    État : pose(), reset_pose(), odometry_stats().
    Toutes les consignes sont plafonnées par max_speed_mm_s / max_omega_rad_s.
    """

    # Sens de la position lue : position_roue_signée = POSITION_SIGN * read_position().
    # Une roue "positive" (bit Motor.POSITIVE_DIRECTION) doit faire AUGMENTER sa position signée.
    # -1 = valeur déduite du code actuel (read_position renvoie -raw) ; à valider sur le robot
    # avec tests/test_odometry.py, puis passer à +1 si le test affiche le signe inverse.
    POSITION_SIGN = -1

    def __init__(self, robot: Robot, group: MotorGroup, max_speed_mm_s: float = 600.0,
                 max_omega_rad_s: float = 3.0, watchdog_s: float = 0.3,
                 odometry_hz: float = 100.0):
        """
        Args:
            max_speed_mm_s: plafond de la vitesse de translation, appliqué à toute consigne
            max_omega_rad_s: plafond de la vitesse de rotation
            watchdog_s: freinage automatique si aucune consigne de vitesse pendant ce délai
                        (None pour désactiver)
            odometry_hz: fréquence de lecture des positions (odométrie)
        """
        self.robot = robot
        self.group = group
        self.max_speed_mm_s = max_speed_mm_s
        self.max_omega_rad_s = max_omega_rad_s
        self.watchdog_s = watchdog_s
        self.odometry_hz = odometry_hz
        self._started = False
        if group.robot is None:          # le groupe a besoin de la même cinématique
            group.robot = robot
        elif group.robot is not robot:
            raise ValueError("Le MotorGroup est déjà lié à un autre Robot")
        self.motors = group.motors

        # --- odométrie ---
        self._odo_lock = threading.Lock()
        self._last_pulses = None                 # positions (pulses) au tour précédent
        self._pose_time = None
        # Matrice de cinématique : d_roue_i = -sin(a_i) dx + cos(a_i) dy + R dθ  (repère robot)
        self._A = np.array([[-math.sin(a), math.cos(a), robot.radius] for a in robot.wheel_angles])
        self._A_inv = np.linalg.inv(self._A)     # exact avec 3 roues (sinon : pseudo-inverse)

        self._dist_mm = 0.0      # longueur de chemin cumulée
        self._rot_rad = 0.0      # rotation cumulée (en valeur absolue)

    # =============================
    #   CYCLE DE VIE
    # =============================

    def start(self) -> None:
        """Lance le watchdog et l'odométrie. Idempotent."""
        if self._started:
            return
        if self.watchdog_s:
            self.group.start_watchdog(self.watchdog_s)
        self.start_odometry(self.odometry_hz)
        self._started = True

    def close(self) -> None:
        """Freine, puis arrête odométrie, watchdog et poller. À appeler dans un finally."""
        try:
            self.stop(255)
        finally:
            self.stop_odometry()
            self.group.close()
            self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # =============================
    #   COMMANDES
    # =============================

    def _limit(self, vx: float, vy: float, omega: float):
        """Plafonne la norme de (vx, vy) en gardant la direction, et |omega|."""
        n = math.hypot(vx, vy)
        if n > self.max_speed_mm_s:
            k = self.max_speed_mm_s / n
            vx, vy = vx * k, vy * k
        omega = max(-self.max_omega_rad_s, min(self.max_omega_rad_s, omega))
        return vx, vy, omega

    def set_velocity(self, vx: float, vy: float, omega: float = 0.0, ramp_acc: int = 220):
        """Vitesse du robot dans le repère GLOBAL (mm/s, mm/s, rad/s, + = anti-horaire).
        À rappeler à chaque cycle de commande (le watchdog freine sinon)."""
        vx, vy, omega = self._limit(vx, vy, omega)
        return self.group.set_velocity(vx, vy, omega, ramp_acc=ramp_acc)

    def set_velocity_local(self, vx: float, vy: float, omega: float = 0.0, ramp_acc: int = 220):
        """Vitesse du robot dans SON repère (x+ vers la roue 3, y+ à 90°) : convertie avec le cap."""
        th = self.pose()[2]
        c, s = math.cos(th), math.sin(th)
        return self.set_velocity(c * vx - s * vy, s * vx + c * vy, omega, ramp_acc)

    def move_velocity_direction(self, direction_angle: float, speed_mm_s: float,
                                omega: float = 0.0, ramp_acc: int = 220):
        """Roule en continu dans une direction globale (rappeler à chaque cycle)."""
        return self.set_velocity(speed_mm_s * math.cos(direction_angle),
                                 speed_mm_s * math.sin(direction_angle), omega, ramp_acc)

    def move_direction(self, distance_mm: float, direction_angle: float, speed_mm_s: float = 50.0,
                       dtheta: float = 0.0, ramp_acc: int = 220, wait: bool = True, **kwargs) -> bool:
        """Déplacement borné (mode position) ; la vitesse est plafonnée par max_speed_mm_s."""
        speed = min(abs(speed_mm_s), self.max_speed_mm_s)
        return self.group.move_direction(distance_mm, direction_angle, speed, dtheta=dtheta,
                                         ramp_acc=ramp_acc, wait=wait, **kwargs)

    def rotate(self, angle_rad: float, omega_rad_s: float = 1.0, ramp_acc: int = 220,
               wait: bool = True, **kwargs) -> bool:
        """Rotation sur place de angle_rad (+ = anti-horaire) ; omega plafonné par max_omega_rad_s."""
        omega = min(abs(omega_rad_s), self.max_omega_rad_s)
        return self.group.rotate(angle_rad, omega, ramp_acc=ramp_acc, wait=wait, **kwargs)

    def stop(self, acceleration: int = 255, emergency: bool = False) -> None:
        """Freine tous les moteurs (annule aussi un mouvement borné en cours)."""
        self.group.stop_all(acceleration, emergency=emergency)

    # =============================
    #   ODOMÉTRIE
    # =============================

    def wheel_deltas_to_body(self, d_wheels_mm):
        """Déplacements de roue (mm) -> (dx_local, dy_local, dθ rad) dans le repère robot."""
        return self._A_inv @ np.asarray(d_wheels_mm, dtype=float)

    def _odo_step(self, samples):
        """Appelé par le poller après chaque tour de lecture ; samples = [(pulses, t), ...]."""
        if any(s is None for s in samples):
            return                                # pas encore de lecture complète : on attend
        pulses = [s[0] for s in samples]
        t = max(s[1] for s in samples)
        with self._odo_lock:
            if self._last_pulses is None:         # première lecture : simple référence
                self._last_pulses, self._pose_time = pulses, t
                return
            d_mm = [self.POSITION_SIGN * (p - q) * m.mm_per_pulse
                    for p, q, m in zip(pulses, self._last_pulses, self.motors)]
            self._last_pulses, self._pose_time = pulses, t

            dxl, dyl, dth = self.wheel_deltas_to_body(d_mm)
            th_mid = self.robot.angle + dth / 2.0            # angle moyen du pas (arc)
            c, s = math.cos(th_mid), math.sin(th_mid)
            self.robot.pos[0] += c * dxl - s * dyl           # repère robot -> global
            self.robot.pos[1] += s * dxl + c * dyl
            self.robot.angle += dth
            step = math.hypot(dxl, dyl)
            if step > 0.05:                 # seuil anti-bruit : ignore les oscillations d'1-2 pulses à l'arrêt
                self._dist_mm += step
            if abs(dth) > 1e-4:
                self._rot_rad += abs(dth)


    def start_odometry(self, hz: float = 100.0) -> None:
        """Lance le poller du groupe (100 Hz) et met la pose à jour à chaque tour."""
        self._last_pulses = None                  # nouvelle référence
        self.group.on_round = self._odo_step
        self.group.start_polling(hz)

    def stop_odometry(self) -> None:
        self.group.stop_polling()
        self.group.on_round = None

    def pose(self):
        """(x mm, y mm, θ rad, timestamp) -- copie cohérente de robot.pos / robot.angle."""
        with self._odo_lock:
            return (float(self.robot.pos[0]), float(self.robot.pos[1]),
                    float(self.robot.angle), self._pose_time)

    def reset_pose(self, x: float, y: float, theta: float) -> None:
        """Recale la pose (ex. contre une bordure, ou depuis le lidar/OTOS)."""
        with self._odo_lock:
            self.robot.pos[0], self.robot.pos[1] = x, y
            self.robot.angle = theta

    def odometry_stats(self):
        """(distance parcourue en mm, rotation cumulée en degrés)"""
        with self._odo_lock:
            return self._dist_mm, math.degrees(self._rot_rad)

    def reset_stats(self):
        with self._odo_lock:
            self._dist_mm = 0.0
            self._rot_rad = 0.0
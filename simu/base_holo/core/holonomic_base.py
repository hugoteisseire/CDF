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

    Pour l'instant : construction + odométrie des roues (start_odometry / pose / reset_pose).
    """

    # Sens de la position lue : position_roue_signée = POSITION_SIGN * read_position().
    # Une roue "positive" (bit Motor.POSITIVE_DIRECTION) doit faire AUGMENTER sa position signée.
    # -1 = valeur déduite du code actuel (read_position renvoie -raw) ; à valider sur le robot
    # avec tests/test_odometry.py, puis passer à +1 si le test affiche le signe inverse.
    POSITION_SIGN = -1

    def __init__(self, robot: Robot, group: MotorGroup):
        self.robot = robot
        self.group = group
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

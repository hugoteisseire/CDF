"""Programme robot réel : pilotage moteur, acquisition lidar et localisation EKF.

Lancement depuis la racine du dépôt :
    python -m lidar.tests.robot_fusion --x0 1000 --y0 1000 --theta0-deg 0

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
from lidar.core.pole_association import AssocParams
from viz_table.robot_protocol import RobotProtocolServer
from viz_table.viz_client import Viz


GEAR_RATIO = 48 / 15
WHEEL_RADIUS_MM = 106.5
WHEEL_ANGLES = [np.radians(120), np.radians(240), np.radians(0)]
CONTROL_HZ = 50


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

    def stop(self):
        self.base.stop(255)
        with self._lock:
            self.mode = "idle"
            self.target = None
            self.target_theta = None
            self._velocity = (0.0, 0.0, 0.0, "robot")
            self.motion_status = "stopped"
            self.motion_error = None

    def goto(self, x: float, y: float, speed_mm_s: float, acc_mm_s2: float,
             theta_deg=None):
        """Lance un goto commun aux commandes stratégie et visualiseur."""
        self.stop()
        px, py, current_theta = self.localizer.status()["pose"]
        dx, dy = x - px, y - py
        distance = math.hypot(dx, dy)
        dtheta = (0.0 if theta_deg is None else
                  _wrap_rad(math.radians(theta_deg) - current_theta))
        if distance <= 1.0 and abs(dtheta) <= math.radians(0.5):
            with self._lock:
                self.motion_status = "reached"
            return
        angle = math.atan2(dy, dx) if distance > 1.0 else 0.0
        pulses_acc = acc_mm_s2 * self.base.motors[0].pulses_per_mm
        ramp_acc = max(1, min(255, Motor.linear_to_acc(pulses_acc)))
        try:
            ok = self.base.move_direction(
                distance, angle,
                speed_mm_s=min(speed_mm_s, self.base.max_speed_mm_s),
                dtheta=dtheta, ramp_acc=ramp_acc, wait=False, verbose=False,
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
            self.target_theta = theta_deg
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
            # La consigne continue est réappliquée à chaque cycle 50 Hz.
            with self._lock:
                self.mode = "velocity"
                self.target = None
                self.target_theta = None
                self._velocity = (cmd["vx"], cmd["vy"], cmd["omega"], cmd["frame"])
                self.motion_status = "running"
                self.motion_error = None
        elif kind == "cmd.goto":
            self.goto(cmd["x"], cmd["y"], cmd["speed"], cmd["acc"], cmd.get("theta"))

    def tick(self):
        with self._lock:
            mode = self.mode
            vx, vy, omega, frame = self._velocity
        if mode == "velocity":
            if frame == "robot":
                theta = self.localizer.status()["pose"][2]
                c, s = math.cos(theta), math.sin(theta)
                vx, vy = c * vx - s * vy, s * vx + c * vy
            self.base.set_velocity(vx, vy, math.radians(omega))

        elif mode == "goto":
            if all(not m.is_running() for m in self.base.motors):
                with self._lock:
                    self.mode = "idle"
                    self.target = None
                    self.target_theta = None
                    self.motion_status = "reached"
                    self.motion_error = None

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


def find_opponent_cluster(clusters, pose, matched_poles, pole_radius_mm=50.0,
                          lidar_offset_mm=(0.0, 0.0), lidar_zero_deg=0.0):
    """Retourne la position terrain et le cluster de la balise adverse, ou None."""
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
    parser.add_argument("--socket", default=None, help="chemin socket stratégie (défaut CDF_ROBOT_SOCKET)")
    args = parser.parse_args()

    bus = notifier = group = base = acq = loc = server = controller = None
    stop_event = threading.Event()
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

        loc = Localizer(
            base.pose, acq.rev_queue,
            (args.x0, args.y0, math.radians(args.theta0_deg)),
            params=ekf_params, assoc=assoc, on_report=on_report,
        )
        loc.start()
        controller = RobotController(base, loc)
        viz = Viz("robot")
        viz.on("goto", lambda args: controller.goto(
            float(args["x"]), float(args["y"]),
            float(args.get("speed", 300.0)), float(args.get("acc", 500.0)),
            args.get("theta"),
        ))
        viz.on("estop", lambda args: controller.stop())
        server_kwargs = {} if args.socket is None else {"path": args.socket}
        server = RobotProtocolServer(controller.state, **server_kwargs).start()

        print(
            f"Robot prêt | pose initiale ({args.x0:.0f}, {args.y0:.0f}, "
            f"{args.theta0_deg:.1f}°) | stratégie + visualiseur actifs",
            flush=True,
        )

        next_tick = time.monotonic()
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
            next_tick += 1.0 / CONTROL_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))

    except KeyboardInterrupt:
        print("\nArrêt demandé.", flush=True)
    finally:
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

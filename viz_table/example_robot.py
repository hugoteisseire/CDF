"""Robot simulé utilisant le même protocole que le futur pilote réel.

Lancer dans cet ordre :
  python server.py
  python example_robot.py
  python example_strategie.py

Le simulateur s'arrête dès que le client stratégie se déconnecte.
"""
import math
import time

from robot_protocol import RobotProtocolServer
from viz_client import Viz

viz = Viz("robot")

robot = {
    "x": 300.0, "y": 300.0, "theta": 0.0,
    "target": None, "mode": "idle", "stopped": True,
    "vx_cmd": 0.0, "vy_cmd": 0.0, "omega_cmd": 0.0,
    "speed": 500.0, "acc": 800.0, "current_speed": 0.0,
}
opponent = {"x": 2200.0, "y": 1300.0, "rmax": 250.0}
protocol_connected = False
DT = 0.02  # boucle simulation 50 Hz
ARRIVAL_MM = 10.0


def protocol_state():
    """Snapshot envoyé à la stratégie à 50 Hz."""
    v = math.hypot(robot["vx_cmd"], robot["vy_cmd"])
    return {
        "robot": {
            "x": robot["x"], "y": robot["y"], "theta": robot["theta"],
            "vx": robot.get("vx", 0.0), "vy": robot.get("vy", 0.0),
            "omega": robot.get("omega", 0.0),
        },
        "opponent": {
            "valid": True, "x": opponent["x"], "y": opponent["y"],
            "theta": 0.0,
        },
        "motion": {
            "mode": robot["mode"],
            "target_x": robot["target"][0] if robot["target"] else None,
            "target_y": robot["target"][1] if robot["target"] else None,
            "target_theta": robot.get("target_theta"),
            "moving": robot["mode"] != "idle" and not robot["stopped"],
        },
    }


protocol = RobotProtocolServer(protocol_state).start()


def stop_motion():
    robot["mode"] = "idle"
    robot["target"] = None
    robot["stopped"] = True
    robot["vx_cmd"] = robot["vy_cmd"] = robot["omega_cmd"] = 0.0
    robot["current_speed"] = 0.0


def apply_command(cmd):
    kind = cmd["type"]
    if kind == "cmd.stop":
        stop_motion()
    elif kind == "cmd.reset_pose":
        # Seul RobotProtocolClient s'identifiant comme strategy peut envoyer ceci.
        stop_motion()
        robot["x"], robot["y"], robot["theta"] = cmd["x"], cmd["y"], cmd["theta"]
    elif kind == "cmd.velocity":
        robot["mode"] = "velocity"
        robot["stopped"] = False
        robot["target"] = None
        robot["frame"] = cmd["frame"]
        robot["vx_cmd"], robot["vy_cmd"] = cmd["vx"], cmd["vy"]
        robot["omega_cmd"] = cmd["omega"]
    elif kind == "cmd.goto":
        robot["mode"] = "goto"
        robot["stopped"] = False
        robot["target"] = (cmd["x"], cmd["y"])
        robot["target_theta"] = cmd.get("theta")
        robot["speed"] = min(cmd["speed"], 1000.0)
        robot["acc"] = min(cmd["acc"], 2000.0)
        robot["current_speed"] = 0.0


# Les commandes de déplacement venant du visualiseur restent possibles.
# La pose, elle, n'est volontairement pas modifiable depuis le visualiseur.
def on_display_goto(args):
    try:
        x, y = float(args["x"]), float(args["y"])
        if math.isfinite(x) and math.isfinite(y):
            robot["target"] = (x, y)
            robot["target_theta"] = None
            robot["mode"] = "goto"
            robot["stopped"] = False
            robot["current_speed"] = 0.0
    except (KeyError, TypeError, ValueError):
        pass


def on_display_stop(args):
    stop_motion()


viz.on("goto", on_display_goto)
viz.on("estop", on_display_stop)
viz.on("set_opponent", lambda args: opponent.update(x=args["x"], y=args["y"]))


def integrate(dt):
    robot["vx"] = robot["vy"] = robot["omega"] = 0.0
    if robot["stopped"] or robot["mode"] == "idle":
        return

    if robot["mode"] == "velocity":
        vx, vy = robot["vx_cmd"], robot["vy_cmd"]
        if robot.get("frame") == "robot":
            a = math.radians(robot["theta"])
            vx, vy = vx * math.cos(a) - vy * math.sin(a), vx * math.sin(a) + vy * math.cos(a)
        robot["x"] += vx * dt
        robot["y"] += vy * dt
        robot["theta"] = (robot["theta"] + robot["omega_cmd"] * dt) % 360.0
        robot["vx"], robot["vy"], robot["omega"] = vx, vy, robot["omega_cmd"]
        return

    if robot["mode"] == "goto" and robot["target"]:
        tx, ty = robot["target"]
        dx, dy = tx - robot["x"], ty - robot["y"]
        dist = math.hypot(dx, dy)
        if dist <= ARRIVAL_MM:
            robot["x"], robot["y"] = tx, ty
            robot["target"] = None
            robot["current_speed"] = 0.0
            robot["mode"] = "idle"
            if robot.get("target_theta") is not None:
                robot["theta"] = robot["target_theta"] % 360.0
            return

        # Profil simple de démonstration : accélération linéaire et freinage
        # pour atteindre la cible sans dépasser le point.
        speed_limit = min(robot["speed"], math.sqrt(2.0 * robot["acc"] * dist))
        robot["current_speed"] += max(-robot["acc"] * dt,
                                      min(robot["acc"] * dt, speed_limit - robot["current_speed"]))
        step = min(dist, robot["current_speed"] * dt)
        ux, uy = dx / dist, dy / dist
        robot["x"] += ux * step
        robot["y"] += uy * step
        robot["theta"] = math.degrees(math.atan2(dy, dx)) % 360.0
        robot["vx"], robot["vy"] = ux * robot["current_speed"], uy * robot["current_speed"]


try:
    next_tick = time.monotonic()
    last_opponent_publish = 0.0
    while True:
        # Ne pas traiter une file ancienne de commandes avant l'état de connexion.
        while True:
            event = protocol.poll(timeout=0.0)
            if event is None:
                break
            if event.get("type") == "connection":
                protocol_connected = event["connected"]
                if not protocol_connected:
                    stop_motion()
            else:
                apply_command(event)

        integrate(DT)
        viz.send("robot", {
            "x": round(robot["x"], 1), "y": round(robot["y"], 1),
            "theta": round(robot["theta"], 1),
            "v": round(math.hypot(robot.get("vx", 0.0), robot.get("vy", 0.0)) / 1000.0, 3),
            "omega": round(robot.get("omega", 0.0), 1), "r": 150,
            "target": list(robot["target"]) if robot["target"] else None,
        })
        if time.monotonic() - last_opponent_publish >= 0.2:
            viz.send("opponent", dict(opponent))
            last_opponent_publish = time.monotonic()

        next_tick += DT
        time.sleep(max(0.0, next_tick - time.monotonic()))
except KeyboardInterrupt:
    pass
finally:
    stop_motion()
    protocol.close()

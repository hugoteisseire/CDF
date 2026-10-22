"""Exemple de programme n°1 : simule un robot et un adversaire.
Montre l'envoi des couches "robot" et "opponent" et la réception de commandes.
Lancer : python example_robot.py   (le serveur doit tourner)
"""
import math
import time

from viz_client import Viz

viz = Viz("robot")

robot = {"x": 300.0, "y": 300.0, "theta": 0.0, "target": None, "stopped": False}
opponent = {"x": 2200.0, "y": 1300.0, "rmax": 250.0}
SPEED = 500.0  # mm/s, soit 0,5 m/s


# ---- commandes venant de la page (fonctions courtes !) -------------------
def on_goto(args):
    robot["target"] = (args["x"], args["y"])
    robot["stopped"] = False


def on_set_pose(args):
    robot["x"], robot["y"] = args["x"], args["y"]
    robot["theta"] = args.get("theta", robot["theta"])
    robot["target"] = None


def on_set_opponent(args):
    opponent["x"], opponent["y"] = args["x"], args["y"]


def on_estop(args):
    robot["stopped"] = True
    robot["target"] = None


viz.on("goto", on_goto)
viz.on("set_robot_pose", on_set_pose)
viz.on("set_opponent", on_set_opponent)
viz.on("estop", on_estop)

# ---- boucle principale : 20 Hz -------------------------------------------
DT = 0.05
last_opp = 0.0
while True:
    v = 0.0
    if robot["target"] and not robot["stopped"]:
        tx, ty = robot["target"]
        dx, dy = tx - robot["x"], ty - robot["y"]
        dist = math.hypot(dx, dy)
        step = SPEED * DT
        if dist <= step:
            robot["x"], robot["y"], robot["target"] = tx, ty, None
        else:
            robot["x"] += dx / dist * step
            robot["y"] += dy / dist * step
            robot["theta"] = math.degrees(math.atan2(dy, dx))
            v = SPEED / 1000.0  # m/s

    viz.send("robot", {
        "x": round(robot["x"], 1), "y": round(robot["y"], 1),
        "theta": round(robot["theta"], 1),
        "v": v, "omega": 0.0, "r": 150,
        "target": list(robot["target"]) if robot["target"] else None,
    })
    if time.time() - last_opp > 0.2:  # l'adversaire est mis à jour plus lentement
        viz.send("opponent", dict(opponent))
        last_opp = time.time()
    time.sleep(DT)

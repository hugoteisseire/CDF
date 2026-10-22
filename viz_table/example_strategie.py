"""Exemple de stratégie et client du protocole robot.

Lancer après example_robot.py. La stratégie reçoit l'état du robot par socket Unix
et peut envoyer robot_link.goto(...), velocity(...), reset_pose(...) ou stop().
"""
import math
import threading
import time

from robot_protocol import RobotProtocolClient
from viz_client import Viz

viz = Viz("strategie")
robot_link = RobotProtocolClient()
lock = threading.Lock()
counter = 0
elements = []


START_ELEMENTS = [
    (600 + 160, 0 + 55, 0), (1100 + 160, 0 + 55, 0), (1600 + 160, 0 + 55, 0), (2100 + 160, 0 + 55, 0),
    (600 + 160, 2000 - 55, 0), (1100 + 160, 2000 - 55, 0), (1600 + 160, 2000 - 55, 0), (2100 + 160, 2000 - 55, 0),
    (1500 - 55 + 55, 500 + 160 - 160, -90),
    (1500 - 55 + 55, 1500 + 160 - 160, -90),
    (1500, 1000, 0, -1),
]
START_VALUE = 3


def initial_elements():
    global counter
    counter = 0
    return [{"id": f"e{i}", "x": e[0], "y": e[1], "theta": e[2],
             "value": e[3] if len(e) > 3 else START_VALUE, "color": "#d4a017"}
            for i, e in enumerate(START_ELEMENTS)]


def publish():
    with lock:
        viz.send("elements", {"items": list(elements)})


# ---- règle de simulation : l'adversaire prend les éléments proches --------
TAKE_DIST_MM = 250
TAKE_TIME_S = 5.0
near_since = {}


def update_opponent(x, y, now=None):
    global elements
    now = time.monotonic() if now is None else now
    with lock:
        ids = {e["id"] for e in elements}
        for k in list(near_since):
            if k not in ids:
                del near_since[k]
        taken = []
        for e in elements:
            if math.hypot(e["x"] - x, e["y"] - y) < TAKE_DIST_MM:
                if now - near_since.setdefault(e["id"], now) >= TAKE_TIME_S:
                    taken.append(e["id"])
            else:
                near_since.pop(e["id"], None)
        if taken:
            elements = [e for e in elements if e["id"] not in taken]
            for k in taken:
                near_since.pop(k, None)
    if taken:
        print(f"adversaire : éléments pris {taken}")
        publish()


# ---- commandes venant de la page : éléments uniquement -------------------
def on_reset_table(args):
    global elements
    with lock:
        elements = initial_elements()
        near_since.clear()


def on_add(args):
    global counter
    with lock:
        counter += 1
        elements.append({"id": f"n{counter}", "x": args["x"], "y": args["y"],
                         "theta": args.get("theta", 0), "value": args.get("value", START_VALUE),
                         "color": "#d4a017"})


def on_remove(args):
    global elements
    with lock:
        elements = [e for e in elements if e["id"] != args["id"]]


def on_move(args):
    with lock:
        for e in elements:
            if e["id"] == args["id"]:
                e["x"], e["y"] = args["x"], args["y"]


def on_rotate(args):
    with lock:
        for e in elements:
            if e["id"] == args["id"]:
                e["theta"] = args["theta"]


for cmd, fn in [("reset_table", on_reset_table), ("add_element", on_add),
                ("remove_element", on_remove), ("move_element", on_move),
                ("rotate_element", on_rotate)]:
    viz.on(cmd, fn)

ZONES = {"shapes": [
    {"type": "rect", "x": 0, "y": 0, "w": 450, "h": 450,
     "color": "#1f6feb", "fill": "rgba(31,111,235,0.15)"},
    {"type": "text", "x": 30, "y": 480, "text": "zone d'exemple", "color": "#1f6feb"},
]}

robot_pose = {"x": None, "y": None, "theta": None, "t": 0.0}


def update_robot(x, y, theta):
    robot_pose.update(x=x, y=y, theta=theta, t=time.monotonic())


def get_robot_pose():
    return dict(robot_pose)


# ---- API stratégie : exemples d'appels depuis la logique de match ----------
def send_goto(x, y, speed=500.0, acc=800.0, theta=None):
    """Demande un déplacement ; speed en mm/s, acc en mm/s², angle en degrés."""
    return robot_link.goto(x, y, speed=speed, acc=acc, theta=theta)


def send_velocity(vx, vy, omega, frame="robot"):
    """Commande de vitesse : mm/s et degrés/s. frame='robot' ou 'field'."""
    return robot_link.velocity(vx, vy, omega, frame=frame)


def reset_robot_pose(x, y, theta):
    """Réinitialise la pose estimée. Cette API n'est pas exposée à l'afficheur."""
    return robot_link.reset_pose(x, y, theta)


def stop_robot():
    return robot_link.stop()


# L'état est produit par le programme robot à 50 Hz ; on consomme la dernière
# trame disponible sans bloquer la boucle d'affichage/stratégie.
def sim_feed():
    last_seq = None
    while True:
        state = robot_link.get_state()
        if state and state.get("seq") != last_seq:
            last_seq = state.get("seq")
            rob = state.get("robot", {})
            if all(k in rob for k in ("x", "y", "theta")):
                update_robot(rob["x"], rob["y"], rob["theta"])
            opp = state.get("opponent", {})
            if opp.get("valid") and "x" in opp and "y" in opp:
                update_opponent(opp["x"], opp["y"])
        time.sleep(0.01)


elements = initial_elements()
threading.Thread(target=sim_feed, daemon=True).start()

try:
    while True:
        publish()
        viz.send("zones", ZONES)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    robot_link.close()

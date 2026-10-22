"""Exemple de programme n°2 : la "stratégie" possède la liste des éléments de jeu.
Elle l'envoie à l'affichage, modifie sa liste quand la page le demande, et supprime
les éléments « pris » par l'adversaire (voir update_opponent).
Lancer : python example_strategie.py   (le serveur doit tourner)
"""
import json
import math
import threading
import time
import urllib.request

from viz_client import Viz

viz = Viz("strategie")
lock = threading.Lock()
counter = 0
elements = []


# Position de départ des éléments : (x, y, theta[, valeur]) en mm et degrés.
# Sans 4e nombre, la valeur de départ est START_VALUE.
START_ELEMENTS = [
    (600 + 160, 0 + 55, 0), (1100 + 160, 0 + 55, 0), (1600 + 160, 0 + 55, 0), (2100 + 160, 0 + 55, 0),
    (600 + 160, 2000 - 55, 0), (1100 + 160, 2000 - 55, 0), (1600 + 160, 2000 - 55, 0), (2100 + 160, 2000 - 55, 0),
    (1500 - 55 + 55, 500 + 160 - 160, -90),
    (1500 - 55 + 55, 1500 + 160 - 160, -90),
    (1500, 1000, 0, -1),   # élément de test : valeur -1, donc 110 x 100 mm
]
START_VALUE = 3   # valeur de départ de chaque élément (0 à 3)


def initial_elements():
    """Liste de départ. Un élément = rectangle 110 x 320 mm ; theta en degrés,
    0 = grand côté le long de x ; value = entier de 0 à 3 affiché sur l'élément
    (-1 : élément plus court, 110 x 100 mm)."""
    global counter
    counter = 0
    return [{"id": f"e{i}", "x": e[0], "y": e[1], "theta": e[2],
             "value": e[3] if len(e) > 3 else START_VALUE, "color": "#d4a017"}
            for i, e in enumerate(START_ELEMENTS)]


def publish():
    with lock:
        viz.send("elements", {"items": list(elements)})


# ---- règle : l'adversaire prend un élément s'il reste à proximité ---------
TAKE_DIST_MM = 250   # distance (centre de l'adversaire à centre de l'élément) sous laquelle il « prend »
TAKE_TIME_S = 5.0    # durée de présence continue avant que l'élément soit supprimé
near_since = {}      # id d'élément -> instant où l'adversaire est entré dans la zone


def update_opponent(x, y, now=None):
    """À appeler à chaque nouvelle position de l'adversaire (en mm).
    Supprime un élément dès que l'adversaire est resté TAKE_TIME_S secondes de suite
    à moins de TAKE_DIST_MM de lui ; la minuterie repart de zéro s'il s'éloigne.
    La minuterie n'avance qu'aux instants où une position est reçue."""
    global elements
    now = time.time() if now is None else now
    with lock:
        ids = {e["id"] for e in elements}
        for k in list(near_since):          # oublier les éléments qui n'existent plus
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


# ---- commandes venant de la page -----------------------------------------
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

# Couche de formes libres : ici une zone d'exemple (n'importe quel programme peut en envoyer)
ZONES = {"shapes": [
    {"type": "rect", "x": 0, "y": 0, "w": 450, "h": 450,
     "color": "#1f6feb", "fill": "rgba(31,111,235,0.15)"},
    {"type": "text", "x": 30, "y": 480, "text": "zone d'exemple", "color": "#1f6feb"},
]}


# ---- position du robot -------------------------------------------------------
robot_pose = {"x": None, "y": None, "theta": None, "t": 0.0}   # t = instant de la dernière mise à jour


def update_robot(x, y, theta):
    """À appeler à chaque nouvelle position du robot (mm, degrés), avec l'odométrie / la
    localisation du vrai programme. En simulation, c'est sim_feed() qui l'appelle."""
    robot_pose.update(x=x, y=y, theta=theta, t=time.time())


def get_robot_pose():
    """Dernière position connue : {"x", "y", "theta", "t"} ; x vaut None tant qu'aucune n'est reçue."""
    return dict(robot_pose)


# ---- SIMULATION UNIQUEMENT : positions du robot et de l'adversaire ---------------
# Ici on relit les couches "robot" et "opponent" que d'autres programmes envoient à l'affichage.
# Dans le vrai programme, ne pas utiliser ce thread : appeler update_robot(...) et
# update_opponent(...) directement depuis les vraies sources (odométrie, lidar).
VIZ_STATE_URL = "http://127.0.0.1:8000/state"


def sim_feed():
    while True:
        try:
            with urllib.request.urlopen(VIZ_STATE_URL, timeout=1) as resp:
                layers_now = json.load(resp)["layers"]
            rob, opp = layers_now.get("robot"), layers_now.get("opponent")
            if rob:
                update_robot(rob["x"], rob["y"], rob.get("theta", 0.0))
            if opp:
                update_opponent(opp["x"], opp["y"])
        except Exception:
            pass                            # serveur absent, ou pas encore de données
        time.sleep(0.2)


elements = initial_elements()
threading.Thread(target=sim_feed, daemon=True).start()
while True:
    publish()
    viz.send("zones", ZONES)
    time.sleep(0.5)  # renvoi régulier : l'affichage reste à jour même après un redémarrage
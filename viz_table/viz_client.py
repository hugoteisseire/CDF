"""Client Python à importer dans chaque programme qui veut afficher quelque chose.

    from viz_client import Viz
    viz = Viz("strategie")
    viz.on("reset_table", lambda args: print("reset !"))
    viz.send("robot", {"x": 1200, "y": 800, "theta": 90})

Dépendance :  pip install websocket-client
- send() ne bloque jamais : si le serveur est absent, le message est simplement perdu
  (l'affichage garde le dernier état, il sera remis à jour au prochain envoi).
- La reconnexion au serveur est automatique.
- Les fonctions données à on() tournent dans le thread de réception : elles doivent
  être courtes (modifier une variable, poser un drapeau), pas bloquer.
"""
import json
import threading
import time
from urllib.parse import quote

import websocket  # paquet "websocket-client"


class Viz:
    def __init__(self, name, host="127.0.0.1", port=8000):
        self.name = name
        self.url = f"ws://{host}:{port}/ws/prog?name={quote(name)}"
        self._handlers = {}
        self._ws = None
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    # ---- envoi ---------------------------------------------------------
    def send(self, layer, data):
        """Remplace le contenu de la couche `layer` par `data` (dict JSON)."""
        return self._send({"type": "layer", "layer": layer, "data": data})

    def clear(self, layer):
        """Supprime une couche de l'affichage."""
        return self._send({"type": "clear", "layer": layer})

    # ---- réception des commandes --------------------------------------
    def on(self, cmd, fn):
        """Appelle fn(args) quand la page envoie la commande `cmd`.
        Utiliser cmd="*" pour recevoir toutes les commandes (fn(args, cmd))."""
        self._handlers[cmd] = fn
        self._hello()

    # ---- interne -------------------------------------------------------
    def _send(self, msg):
        ws = self._ws
        if ws is None:
            return False
        try:
            with self._lock:
                ws.send(json.dumps(msg))
            return True
        except Exception:
            return False

    def _hello(self):
        self._send({"type": "hello", "subscribe": list(self._handlers)})

    def _on_open(self, ws):
        self._ws = ws
        self._hello()

    def _on_message(self, ws, text):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        if msg.get("type") != "cmd":
            return
        cmd, args = msg.get("cmd"), msg.get("args", {})
        try:
            if cmd in self._handlers:
                self._handlers[cmd](args)
            elif "*" in self._handlers:
                self._handlers["*"](args, cmd)
        except Exception as e:  # une erreur dans un handler ne doit pas couper la liaison
            print(f"[viz:{self.name}] erreur dans le handler de {cmd!r} : {e}")

    def _run(self):
        while True:
            try:
                app = websocket.WebSocketApp(
                    self.url, on_open=self._on_open, on_message=self._on_message)
                app.run_forever()
            except Exception:
                pass
            self._ws = None
            time.sleep(1)  # le serveur est parti : on réessaie

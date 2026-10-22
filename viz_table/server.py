"""Serveur de visualisation de la table de jeu.

- Les programmes Python se connectent sur /ws/prog?name=<nom> et envoient des "couches".
- Les navigateurs se connectent sur /ws/view : ils reçoivent les couches
  et envoient des commandes, que le serveur relaie aux programmes concernés.

Lancement :  python server.py     puis ouvrir  http://<ip-de-la-pi>:8000
"""
import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response

HERE = Path(__file__).parent
PORT = 8000

layers = {}     # nom de couche -> dernières données reçues (pas d'historique)
programs = {}   # nom de programme -> {"ws", "subscribe", "last_seen", "connected"}
viewers = set()  # navigateurs connectés


def clients_status():
    now = time.time()
    return [
        {"name": name, "connected": p["connected"], "age": round(now - p["last_seen"], 2)}
        for name, p in programs.items()
    ]


async def broadcast(msg):
    """Envoie un message à tous les navigateurs."""
    text = json.dumps(msg)
    for ws in list(viewers):
        try:
            await ws.send_text(text)
        except Exception:
            viewers.discard(ws)


async def clients_loop():
    """Une fois par seconde, met à jour les pastilles de connexion."""
    while True:
        await asyncio.sleep(1)
        if viewers:
            await broadcast({"type": "clients", "clients": clients_status()})


@asynccontextmanager
async def lifespan(app):
    task = asyncio.create_task(clients_loop())
    yield
    task.cancel()


app = FastAPI(lifespan=lifespan)


NO_CACHE = {"Cache-Control": "no-cache"}   # le navigateur revérifie à chaque chargement


@app.get("/")
async def index():
    return FileResponse(HERE / "index.html", headers=NO_CACHE)


@app.get("/config.js")
async def config_js():
    """Réglages personnels (config.js, créé par l'utilisateur) ; vide s'il n'existe pas."""
    f = HERE / "config.js"
    if f.exists():
        return FileResponse(f, headers=NO_CACHE, media_type="text/javascript")
    return Response("window.CONFIG = {};", media_type="text/javascript", headers=NO_CACHE)


@app.get("/table.png")
async def table_png():
    """Image de fond de la table (table.png à côté de server.py)."""
    f = HERE / "table.png"
    if not f.exists():
        raise HTTPException(status_code=404)
    return FileResponse(f, headers=NO_CACHE)


@app.get("/state")
async def state():
    """Pratique pour déboguer : état courant de toutes les couches."""
    return {"layers": layers, "clients": clients_status()}


@app.websocket("/ws/prog")
async def ws_prog(ws: WebSocket, name: str = "anonyme"):
    await ws.accept()
    info = {"ws": ws, "subscribe": set(), "last_seen": time.time(), "connected": True}
    programs[name] = info  # un programme reconnecté remplace l'ancien du même nom
    await broadcast({"type": "clients", "clients": clients_status()})
    try:
        while True:
            try:
                msg = json.loads(await ws.receive_text())
                kind = msg.get("type")
            except (ValueError, AttributeError):
                continue
            info["last_seen"] = time.time()
            if kind == "hello":
                info["subscribe"] = set(msg.get("subscribe", []))
            elif kind == "layer" and "layer" in msg:
                layers[msg["layer"]] = msg.get("data")
                await broadcast({"type": "layer", "layer": msg["layer"],
                                 "data": msg.get("data"), "from": name})
            elif kind == "clear" and "layer" in msg:
                layers.pop(msg["layer"], None)
                await broadcast({"type": "clear", "layer": msg["layer"]})
    except WebSocketDisconnect:
        pass
    finally:
        if programs.get(name) is info:
            info["connected"] = False
        await broadcast({"type": "clients", "clients": clients_status()})


async def route_command(msg, viewer):
    """Relaie une commande du navigateur aux programmes qui l'ont demandée."""
    cmd = msg.get("cmd")
    out = json.dumps({"type": "cmd", "cmd": cmd, "args": msg.get("args", {})})
    sent = 0
    for p in programs.values():
        if p["connected"] and ("*" in p["subscribe"] or cmd in p["subscribe"]):
            try:
                await p["ws"].send_text(out)
                sent += 1
            except Exception:
                p["connected"] = False
    if sent == 0:
        await viewer.send_text(json.dumps({"type": "nodest", "cmd": cmd}))


@app.websocket("/ws/view")
async def ws_view(ws: WebSocket):
    await ws.accept()
    viewers.add(ws)
    await ws.send_text(json.dumps(
        {"type": "state", "layers": layers, "clients": clients_status()}))
    try:
        while True:
            try:
                msg = json.loads(await ws.receive_text())
            except ValueError:
                continue
            if isinstance(msg, dict) and msg.get("type") == "cmd":
                await route_command(msg, ws)
    except WebSocketDisconnect:
        pass
    finally:
        viewers.discard(ws)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")

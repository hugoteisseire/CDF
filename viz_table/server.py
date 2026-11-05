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
chasing = set()  # navigateurs dont la dernière consigne "velocity" n'est pas nulle (Chase the Mouse)

# Diagnostic : trous dans la couche "robot", envois lents vers un navigateur, boucle bloquée.
DIAG = {"rx": 0, "gap_max": 0.0, "bcast_max": 0.0, "slow": 0, "last_t": None, "last_seq": None,
        "t0": time.monotonic()}


def _stamp():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"


def _peer(ws):
    c = getattr(ws, "client", None)
    return f"{c.host}:{c.port}" if c else "?"


async def diag_loop():
    """Mesure le retard de la boucle asyncio et affiche un bilan toutes les 2 s."""
    while True:
        t0 = time.perf_counter()
        await asyncio.sleep(0.05)
        lag = (time.perf_counter() - t0) * 1000 - 50
        if lag > 100:
            print(f"[SERVEUR] {_stamp()} boucle asyncio bloquée {lag:.0f} ms", flush=True)
        now = time.monotonic()
        if now - DIAG["t0"] >= 2.0:
            print(f"[SERVEUR] {_stamp()} robot reçu {DIAG['rx'] / (now - DIAG['t0']):4.1f}/s | "
                  f"trou max {DIAG['gap_max']:5.0f} ms | diffusion max {DIAG['bcast_max']:5.1f} ms | "
                  f"envois lents {DIAG['slow']} | navigateurs {len(viewers)} "
                  f"({', '.join(_peer(v) for v in viewers)})", flush=True)
            DIAG.update(rx=0, gap_max=0.0, bcast_max=0.0, slow=0, t0=now)


def clients_status():
    now = time.time()
    return [
        {"name": name, "connected": p["connected"], "age": round(now - p["last_seen"], 2)}
        for name, p in programs.items()
    ]


async def broadcast(msg):
    """Envoie un message à tous les navigateurs."""
    text = json.dumps(msg)
    t_all = time.perf_counter()
    for ws in list(viewers):
        t0 = time.perf_counter()
        try:
            await ws.send_text(text)
        except Exception:
            viewers.discard(ws)
        dt = (time.perf_counter() - t0) * 1000
        if dt > 50:
            DIAG["slow"] += 1
            print(f"[SERVEUR] {_stamp()} envoi lent vers le navigateur {_peer(ws)} : {dt:.0f} ms "
                  f"(message {msg.get('layer', msg.get('type'))})", flush=True)
    DIAG["bcast_max"] = max(DIAG["bcast_max"], (time.perf_counter() - t_all) * 1000)


async def clients_loop():
    """Une fois par seconde, met à jour les pastilles de connexion."""
    while True:
        await asyncio.sleep(1)
        if viewers:
            await broadcast({"type": "clients", "clients": clients_status()})


@asynccontextmanager
async def lifespan(app):
    task = asyncio.create_task(clients_loop())
    diag_task = asyncio.create_task(diag_loop())
    yield
    task.cancel()
    diag_task.cancel()


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
                if msg["layer"] == "robot":
                    now = time.perf_counter()
                    seq = (msg.get("data") or {}).get("seq")
                    if DIAG["last_t"] is not None:
                        gap = (now - DIAG["last_t"]) * 1000
                        DIAG["gap_max"] = max(DIAG["gap_max"], gap)
                        if gap > 300:
                            print(f"[SERVEUR] {_stamp()} trou de {gap:.0f} ms dans la couche robot "
                                  f"reçue de {name} (seq {DIAG['last_seq']} -> {seq})", flush=True)
                    DIAG["last_t"], DIAG["last_seq"] = now, seq
                    DIAG["rx"] += 1
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
    if cmd == "velocity" and viewer is not None:
        a = msg.get("args") or {}
        if any(a.get(k) for k in ("vx", "vy", "omega")):
            chasing.add(viewer)
        else:
            chasing.discard(viewer)
    out = json.dumps({"type": "cmd", "cmd": cmd, "args": msg.get("args", {})})
    sent = 0
    for p in programs.values():
        if p["connected"] and ("*" in p["subscribe"] or cmd in p["subscribe"]):
            try:
                await p["ws"].send_text(out)
                sent += 1
            except Exception:
                p["connected"] = False
    if sent == 0 and viewer is not None:
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
        if ws in chasing:   # le navigateur qui pilotait en vitesse est parti : arrêt immédiat
            chasing.discard(ws)
            await route_command({"cmd": "velocity",
                                 "args": {"vx": 0, "vy": 0, "omega": 0, "timeout": 0.2}}, None)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")

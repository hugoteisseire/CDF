"""Protocole local stratégie <-> programme robot.

Transport : socket Unix, JSON Lines (une trame JSON par ligne).
Le robot est le serveur et reste l'unique autorité sur son état.
La connexion est réservée au client logique "strategy".
"""
from __future__ import annotations

import json
import math
import os
import queue
import select
import socket
import threading
import time
from pathlib import Path
from typing import Callable

SOCKET_PATH = os.environ.get("CDF_ROBOT_SOCKET", "/tmp/cdf_robot.sock")
PROTOCOL_VERSION = 1
STATE_PERIOD = 0.020  # 50 Hz
MAX_FRAME_BYTES = 64 * 1024


def _json_line(message: dict) -> bytes:
    return (json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def _finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{field} doit être un nombre fini")
    return float(value)


def validate_command(message: dict) -> dict:
    """Valide et normalise une commande reçue du client stratégie."""
    if not isinstance(message, dict) or message.get("v") != PROTOCOL_VERSION:
        raise ValueError("version de protocole absente ou incompatible")
    kind = message.get("type")
    command_id = message.get("id")
    if not isinstance(command_id, int) or isinstance(command_id, bool):
        raise ValueError("id doit être un entier")
    out = dict(message)
    if kind == "cmd.velocity":
        if message.get("frame") not in ("robot", "field"):
            raise ValueError("frame doit valoir 'robot' ou 'field'")
        for key in ("vx", "vy", "omega"):
            out[key] = _finite_number(message.get(key), key)
    elif kind == "cmd.goto":
        for key in ("x", "y", "speed", "acc"):
            out[key] = _finite_number(message.get(key), key)
        if out["speed"] <= 0 or out["acc"] <= 0:
            raise ValueError("speed et acc doivent être strictement positifs")
        if "theta" in message and message["theta"] is not None:
            out["theta"] = _finite_number(message["theta"], "theta")
    elif kind == "cmd.reset_pose":
        for key in ("x", "y", "theta"):
            out[key] = _finite_number(message.get(key), key)
    elif kind == "cmd.stop":
        pass
    else:
        raise ValueError(f"type de commande inconnu : {kind!r}")
    out["source"] = "strategy"  # l'identité provient de la poignée de main
    return out


class RobotProtocolServer:
    """Serveur à intégrer au programme robot/simulateur.

    state_provider() renvoie un dictionnaire JSON sérialisable.
    Les commandes et événements de connexion sont récupérés avec poll().
    """

    def __init__(self, state_provider: Callable[[], dict], path: str = SOCKET_PATH):
        self.state_provider = state_provider
        self.path = path
        self.events: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="robot-protocol", daemon=True)
        self.connected = threading.Event()

    def start(self):
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        try:
            socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).connect(self.path)
        except OSError:
            pass
        self._thread.join(timeout=1.0)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

    def poll(self, timeout: float = 0.0):
        """Renvoie une commande ou un événement {'type':'connection', 'connected': bool}."""
        try:
            return self.events.get(timeout=timeout)
        except queue.Empty:
            return None

    def _run(self):
        path = Path(self.path)
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            listener.bind(self.path)
            os.chmod(self.path, 0o600)
            listener.listen(1)
            listener.settimeout(0.2)
            while not self._stop.is_set():
                try:
                    conn, _ = listener.accept()
                except socket.timeout:
                    continue
                try:
                    self._serve_client(conn)
                finally:
                    conn.close()
        finally:
            listener.close()
            try:
                os.unlink(self.path)
            except FileNotFoundError:
                pass

    def _serve_client(self, conn: socket.socket):
        # select() cadence le socket ; les petits envois locaux restent simples.
        conn.setblocking(True)
        buffer = b""
        # Première trame obligatoire : identification du client.
        deadline = time.monotonic() + 2.0
        hello = None
        while not self._stop.is_set() and time.monotonic() < deadline and hello is None:
            ready, _, _ = select.select([conn], [], [], 0.1)
            if not ready:
                continue
            data = conn.recv(4096)
            if not data:
                return
            buffer += data
            if len(buffer) > MAX_FRAME_BYTES:
                return
            if b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    hello = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    return
        if not isinstance(hello, dict) or hello.get("type") != "hello" or hello.get("v") != PROTOCOL_VERSION:
            return
        if hello.get("source") != "strategy":
            conn.sendall(_json_line({"v": 1, "type": "error", "reason": "source_not_allowed"}))
            return

        self.connected.set()
        self.events.put({"type": "connection", "connected": True})
        next_state = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                timeout = max(0.0, min(STATE_PERIOD, next_state - now))
                ready, _, _ = select.select([conn], [], [], timeout)
                if ready:
                    data = conn.recv(8192)
                    if not data:
                        break
                    buffer += data
                    if len(buffer) > MAX_FRAME_BYTES:
                        break
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        if not line:
                            continue
                        raw = None
                        try:
                            raw = json.loads(line.decode("utf-8"))
                            command = validate_command(raw)
                            self.events.put(command)
                            conn.sendall(_json_line({"v": 1, "type": "ack", "id": command["id"], "status": "accepted"}))
                        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                            cmd_id = raw.get("id") if isinstance(raw, dict) else None
                            conn.sendall(_json_line({"v": 1, "type": "ack", "id": cmd_id,
                                                     "status": "rejected", "reason": str(exc)}))
                now = time.monotonic()
                if now >= next_state:
                    state = self.state_provider()
                    packet = {"v": 1, "type": "state", "seq": int(now / STATE_PERIOD),
                              "timestamp_ns": time.monotonic_ns(), **state}
                    try:
                        conn.sendall(_json_line(packet))
                    except (BrokenPipeError, ConnectionError, OSError):
                        break
                    next_state = now + STATE_PERIOD
        except (BrokenPipeError, ConnectionError, OSError, ValueError):
            pass
        finally:
            self.connected.clear()
            self.events.put({"type": "connection", "connected": False})


class RobotProtocolClient:
    """Client stratégie avec reconnexion automatique et dernier état disponible.

    Une déconnexion arrête le robot côté serveur. Aucune commande n'est mise en
    attente pendant la coupure ; l'appelant doit renvoyer une nouvelle commande.
    """

    def __init__(self, path: str = SOCKET_PATH, reconnect: bool = True):
        self.path = path
        self.reconnect = reconnect
        self.connected = threading.Event()
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state = None
        self._acks = queue.Queue()
        self._next_id = 1
        self._sock = None
        self._thread = threading.Thread(target=self._run, name="strategy-protocol", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        sock = self._sock
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self._thread.join(timeout=1.0)

    def get_state(self):
        with self._state_lock:
            return None if self._state is None else dict(self._state)

    def get_ack(self, timeout: float = 0.0):
        try:
            return self._acks.get(timeout=timeout)
        except queue.Empty:
            return None

    def velocity(self, vx: float, vy: float, omega: float, frame: str = "robot"):
        return self._command("cmd.velocity", frame=frame, vx=vx, vy=vy, omega=omega)

    def goto(self, x: float, y: float, speed: float, acc: float, theta=None):
        fields = {"x": x, "y": y, "speed": speed, "acc": acc}
        if theta is not None:
            fields["theta"] = theta
        return self._command("cmd.goto", **fields)

    def reset_pose(self, x: float, y: float, theta: float):
        return self._command("cmd.reset_pose", x=x, y=y, theta=theta)

    def stop(self):
        return self._command("cmd.stop")

    def _command(self, kind, **fields):
        if not self.connected.is_set():
            return False
        command_id = self._next_id
        self._next_id += 1
        message = {"v": PROTOCOL_VERSION, "type": kind, "id": command_id, **fields}
        try:
            packet = _json_line(message)
            with self._send_lock:
                sock = self._sock
                if sock is None or not self.connected.is_set():
                    return False
                sock.sendall(packet)
            return command_id
        except (BrokenPipeError, ConnectionError, OSError):
            self.connected.clear()
            return False

    def _run(self):
        while not self._stop.is_set():
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                sock.connect(self.path)
                sock.sendall(_json_line({"v": 1, "type": "hello", "source": "strategy"}))
                sock.settimeout(0.2)
                self._sock = sock
                self.connected.set()
                buffer = b""
                while not self._stop.is_set():
                    try:
                        data = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buffer += data
                    if len(buffer) > MAX_FRAME_BYTES:
                        break
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            msg = json.loads(line.decode("utf-8"))
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            continue
                        if msg.get("type") == "state":
                            with self._state_lock:
                                self._state = msg
                        elif msg.get("type") == "ack":
                            self._acks.put(msg)
            except (FileNotFoundError, ConnectionError, OSError):
                pass
            finally:
                self.connected.clear()
                self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
            if not self.reconnect:
                break
            self._stop.wait(0.5)

"""Acquisition lidar dans son PROPRE PROCESSUS (étape 0 du plan).

Le défaut corrigé ici : le lidar « ne tourne plus à sa vitesse nominale » (moins de 10 tours/s, tours incomplets)
dès que la lecture série partage son processus avec autre chose (affichage pygame, Localizer, odométrie CAN...).

Cause : la bibliothèque lit le port point par point (5 octets, ~3 appels système par point, 5000 points/s). Dans
un même processus Python, tous les threads se partagent le GIL ; à chaque appel système le lecteur le rend, puis
doit le reprendre, et si un autre thread calcule il attend jusqu'à 5 ms (sys.getswitchinterval()). Avec seulement
~200 points/s de capacité au lieu de 5000, le tampon du port déborde, des octets sont perdus, les trames se
désalignent et les tours sont mal découpés. Ce n'est pas un problème de priorité de thread : un thread n'est
jamais « seul » tant qu'un autre thread Python tourne dans le même processus.

Mesuré avec la vraie bibliothèque sur un port série virtuel (5000 points/s) :
    lecture dans le processus, rien d'autre ......... 10,00 tours/s, 500 points/tour, 0 octet perdu
    lecture dans le processus + 1 thread occupé ...... 0 tour/s, ~260 000 octets perdus en 12 s

Ici la lecture série et le découpage en tours tournent dans un processus fils qui ne fait rien d'autre. Il a
son propre GIL : ce que fait le processus principal ne peut plus la ralentir. Les tours complets lui reviennent
par une connexion locale ; LidarProcess a la même interface que LidarAcquisition (start, stop, alive, error,
rev_queue, snapshot, clock.recalibrated), donc Localizer et test_fusion s'en servent sans changement.

    acq = LidarProcess("/dev/ttyUSB0", cpu=3)     # cpu : coeur réservé au lidar (facultatif)
    acq.start()                                    # lève ConnectionError si le port est introuvable
    rev = acq.rev_queue.get()                      # Revolution, horodatages en epoch comme avant
    acq.snapshot()                                 # tours/s, points/s, tours rejetés, plus long trou de réception...
    acq.stop()

Si le processus principal disparaît, le fils s'en aperçoit (connexion coupée), arrête le lidar et se termine.
"""
import argparse
import importlib
import logging
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from multiprocessing.connection import Client, Listener
from types import SimpleNamespace

from .lidar_acquisition import LidarAcquisition, Revolution

_MODULE = __name__           # "lidar.core.lidar_process" ou "core.lidar_process" : le fils le relance par ce nom
STATS_PERIOD_S = 0.2         # le fils publie ses compteurs 5 fois par seconde
STALE_S = 3.0                # sans nouvelles du fils pendant ce temps : on le considère bloqué


def _nodelay(conn):
    """Pas de temporisation de Nagle : un tour de 10 Ko ne doit pas attendre un accusé de réception."""
    try:
        s = socket.fromfd(conn.fileno(), socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.close()
    except OSError:
        pass


def _child_pythonpath() -> str:
    """Dossiers d'import du fils : la racine qui permet de retrouver ce module par son nom, puis ceux du parent."""
    root = os.path.abspath(__file__)
    for _ in range(_MODULE.count(".") + 1):
        root = os.path.dirname(root)
    paths = [root]
    for p in sys.path:
        p = os.path.abspath(p or os.getcwd())
        if os.path.isdir(p) and p not in paths:
            paths.append(p)
    extra = os.environ.get("PYTHONPATH")
    if extra:
        paths.append(extra)
    return os.pathsep.join(paths)


# ============================================================================================ côté principal
class LidarProcess:
    def __init__(self, port="/dev/ttyUSB0", baud=460800, queue_size=20, min_raw_points=300,
                 max_first_angle_deg=10.0, min_last_angle_deg=350.0, cpu=None, factory=None,
                 start_timeout_s=15.0):
        """
        cpu     : numéro de coeur sur lequel épingler le processus lidar (os.sched_setaffinity) ; None = libre.
                  Sur un Raspberry Pi 4 (4 coeurs), cpu=3 le met à l'écart des autres si on garde le reste ailleurs.
        factory : "module:fonction" -> (port, baud) -> objet de type RPLidar, pour tester sans matériel.
        """
        self.port, self.baud = port, baud
        self._args = dict(queue_size=queue_size, min_raw_points=min_raw_points,
                          max_first_angle_deg=max_first_angle_deg, min_last_angle_deg=min_last_angle_deg)
        self._cpu, self._factory, self._start_timeout = cpu, factory, start_timeout_s
        self.rev_queue = queue.Queue(maxsize=queue_size)
        self.clock = SimpleNamespace(recalibrated=False)   # même attribut que LidarAcquisition.clock
        self.error = None
        self.pid = None
        self.info = {}                                     # ce que le fils a pu régler (coeur, priorité)
        self._proc = None
        self._conn = None
        self._thread = None
        self._lock = threading.Lock()
        self._stats = None                                 # derniers compteurs publiés par le fils
        self._prev = None                                  # compteurs au dernier snapshot()
        self._last_rates = (0.0, 0.0, 0.0, 0.0)
        self._parent_dropped = 0                           # tours perdus ici parce que le consommateur était lent
        self._gap_max = 0.0
        self._last_msg = time.monotonic()
        self._stopping = False
        self._closed = False

    # ------------------------------------------------------------------ cycle de vie
    def start(self):
        if self._proc is not None:
            raise RuntimeError("LidarProcess déjà démarré")
        key = os.urandom(16)
        listener = Listener(("127.0.0.1", 0), authkey=key)
        host, port = listener.address
        a = self._args
        cmd = [sys.executable, "-m", _MODULE, "--connect", f"{host}:{port}", "--port", self.port,
               "--baud", str(self.baud), "--queue-size", str(a["queue_size"]), "--min-raw", str(a["min_raw_points"]),
               "--max-first", str(a["max_first_angle_deg"]), "--min-last", str(a["min_last_angle_deg"])]
        if self._cpu is not None:
            cmd += ["--cpu", str(self._cpu)]
        if self._factory:
            cmd += ["--factory", self._factory]
        env = dict(os.environ, PYTHONPATH=_child_pythonpath(), LIDAR_PROC_KEY=key.hex())
        self._proc = subprocess.Popen(cmd, env=env)       # stderr du fils = celui du parent : ses erreurs restent visibles

        box = {}

        def _accept():                                    # Listener.accept() n'a pas de délai d'attente
            try:
                box["conn"] = listener.accept()
            except BaseException as e:
                box["err"] = e

        th = threading.Thread(target=_accept, daemon=True, name="lidar-proc-accept")
        th.start()
        deadline = time.monotonic() + self._start_timeout
        while th.is_alive() and self._proc.poll() is None and time.monotonic() < deadline:
            th.join(0.05)
        if "conn" not in box:
            rc = self._proc.poll()
            self._abort(listener, (host, port))
            raise RuntimeError("le processus d'acquisition lidar n'a pas démarré "
                               + (f"(code de sortie {rc}) : voir le message d'erreur ci-dessus" if rc is not None
                                  else f"(délai de {self._start_timeout:.0f} s dépassé)"))
        listener.close()
        conn = box["conn"]
        _nodelay(conn)

        while True:                                       # le fils ouvre le port série puis annonce "ready" ou "error"
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._abort(None, None, conn)
                raise RuntimeError(f"initialisation du lidar trop longue (> {self._start_timeout:.0f} s)")
            if conn.poll(min(0.05, remaining)):
                try:
                    msg = conn.recv()
                except (EOFError, OSError):
                    self._abort(None, None, conn)
                    raise RuntimeError("le processus lidar s'est arrêté pendant son démarrage")
                if msg[0] == "ready":
                    self.info = msg[1]
                    self.pid = self.info.get("pid")
                    break
                if msg[0] == "error":
                    self._abort(None, None, conn)
                    if msg[1] == "ConnectionError":       # même exception qu'avec LidarAcquisition (port introuvable...)
                        raise ConnectionError(msg[2])
                    raise RuntimeError(f"{msg[1]} : {msg[2]}")
            elif self._proc.poll() is not None:
                self._abort(None, None, conn)
                raise RuntimeError("le processus lidar s'est arrêté pendant son démarrage")

        self._conn = conn
        self._last_msg = time.monotonic()
        self._thread = threading.Thread(target=self._receive, daemon=True, name="lidar-proc-rx")
        self._thread.start()

    def _abort(self, listener, addr, conn=None):
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if listener is not None:
            try:                                          # réveille accept() s'il attend encore
                socket.create_connection(addr, timeout=0.2).close()
            except OSError:
                pass
            listener.close()
        if conn is not None:
            conn.close()

    def stop(self):
        if self._proc is None:
            return
        self._stopping = True
        if self._conn is not None and not self._closed:
            try:
                self._conn.send(("stop",))
            except (OSError, ValueError):
                pass
        try:
            self._proc.wait(10)                           # le fils arrête le lidar (reset + moteur) puis se termine
        except subprocess.TimeoutExpired:
            self._proc.terminate()
            try:
                self._proc.wait(2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass

    @property
    def alive(self) -> bool:
        """Le processus tourne, ses deux threads aussi, et il donne de ses nouvelles."""
        if self._proc is None or self._proc.poll() is not None or self._closed:
            return False
        st = self._stats
        if st is not None and not st["alive"]:
            return False
        return time.monotonic() - self._last_msg < STALE_S

    # ------------------------------------------------------------------ réception des tours
    def _receive(self):
        conn = self._conn
        try:
            while True:
                msg = conn.recv()
                self._last_msg = time.monotonic()
                kind = msg[0]
                if kind == "rev":
                    _, number, points, n_raw, t0, t1 = msg
                    rev = Revolution(number, points, n_raw, t0, t1)
                    try:
                        self.rev_queue.put_nowait(rev)
                    except queue.Full:                    # consommateur trop lent : on garde le plus récent
                        try:
                            self.rev_queue.get_nowait()
                        except queue.Empty:
                            pass
                        self.rev_queue.put_nowait(rev)
                        self._parent_dropped += 1
                elif kind == "stats":
                    st = msg[1]
                    with self._lock:
                        self._stats = st
                        self._gap_max = max(self._gap_max, st["max_gap_ms"])
                    self.clock.recalibrated = st["recalibrated"]
                elif kind == "error":
                    if not self._stopping:
                        self.error = RuntimeError(f"{msg[1]} : {msg[2]}")
                elif kind == "bye":
                    break
        except (EOFError, OSError):
            pass
        finally:
            self._closed = True
            if not self._stopping and self.error is None:
                try:
                    rc = self._proc.wait(1)
                except subprocess.TimeoutExpired:
                    rc = None
                self.error = RuntimeError(f"le processus d'acquisition lidar s'est arrêté (code {rc})")

    # ------------------------------------------------------------------ statistiques
    def snapshot(self) -> dict:
        """Mêmes clés que LidarAcquisition.snapshot(), plus : max_gap_ms, cpu_pct (du processus lidar), pid.

        Les cadences sont calculées entre deux appels (avec l'horloge du fils : le retard éventuel du processus
        principal à lire ses messages ne les fausse pas)."""
        with self._lock:
            st, gap = self._stats, self._gap_max
            self._gap_max = 0.0
            prev = self._prev
            if st is not None and (prev is None or st["t"] > prev["t"]):
                self._prev = st
        if st is None:
            return {"freq_hz": 0.0, "pts_per_s": 0.0, "pts_per_rev": 0.0, "reject_reasons": {}, "last_rev": None,
                    "revs_seen": 0, "revs_published": 0, "revs_rejected": 0, "revs_dropped": 0, "lib_errors": 0,
                    "max_gap_ms": 0.0, "cpu_pct": 0.0, "pid": self.pid}
        if prev is None:
            prev = {"t": st["t_start"], "revs_seen": 0, "points": 0, "raw_total": 0, "cpu_s": 0.0}
        dt = st["t"] - prev["t"]
        if dt > 1e-6:
            n = st["revs_seen"] - prev["revs_seen"]
            self._last_rates = (n / dt, (st["points"] - prev["points"]) / dt,
                                (st["raw_total"] - prev["raw_total"]) / n if n else 0.0,
                                100.0 * (st["cpu_s"] - prev["cpu_s"]) / dt)
        freq, pts_s, pts_rev, cpu = self._last_rates
        return {
            "freq_hz": freq, "pts_per_s": pts_s, "pts_per_rev": pts_rev,
            "reject_reasons": dict(st["reject_reasons"]), "last_rev": st["last_rev"],
            "revs_seen": st["revs_seen"], "revs_published": st["revs_published"],
            "revs_rejected": st["revs_rejected"], "revs_dropped": st["revs_dropped"] + self._parent_dropped,
            "lib_errors": st["lib_errors"], "max_gap_ms": gap, "cpu_pct": cpu, "pid": self.pid,
        }


# ============================================================================================ côté processus fils
def _tune_process(cpu):
    """Réglages facultatifs du processus lidar ; ce qui n'est pas permis est simplement noté, jamais fatal."""
    info = {}
    if cpu is not None:
        try:
            os.sched_setaffinity(0, {cpu})
            info["cpu"] = cpu
        except (AttributeError, OSError) as e:
            info["cpu_error"] = repr(e)
    try:
        info["nice"] = os.nice(-10)                       # demande un peu de priorité (réservé à root / CAP_SYS_NICE)
    except (AttributeError, OSError):
        pass
    return info


def _resolve(spec):
    module, _, name = spec.partition(":")
    return getattr(importlib.import_module(module), name)


def _stats(acq, t_start):
    return {"t": time.monotonic(), "t_start": t_start,
            "revs_seen": acq.revs_seen, "revs_published": acq.revs_published, "revs_rejected": acq.revs_rejected,
            "revs_dropped": acq.revs_dropped, "points": acq.points, "raw_total": acq.raw_total,
            "reject_reasons": dict(acq.reject_reasons), "last_rev": acq.last_rev,
            "lib_errors": acq._errors.count, "alive": acq.alive, "recalibrated": acq.clock.recalibrated,
            "max_gap_ms": acq.take_max_gap_ms(), "cpu_s": time.process_time()}


def _child_main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", required=True)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--queue-size", type=int, default=20)
    ap.add_argument("--min-raw", type=int, default=300)
    ap.add_argument("--max-first", type=float, default=10.0)
    ap.add_argument("--min-last", type=float, default=350.0)
    ap.add_argument("--cpu", type=int, default=None)
    ap.add_argument("--factory", default=None)
    args = ap.parse_args(argv)

    signal.signal(signal.SIGINT, signal.SIG_IGN)          # Ctrl+C vise le parent : c'est lui qui nous arrête proprement
    logging.getLogger().addHandler(logging.NullHandler())  # pas d'affichage sauvage des avertissements de la bibliothèque
    host, port = args.connect.rsplit(":", 1)
    conn = Client((host, int(port)), authkey=bytes.fromhex(os.environ.pop("LIDAR_PROC_KEY")))
    _nodelay(conn)

    acq = None
    try:
        info = {"pid": os.getpid(), **_tune_process(args.cpu)}
        acq = LidarAcquisition(args.port, args.baud, args.queue_size, args.min_raw, args.max_first, args.min_last,
                               lidar_factory=_resolve(args.factory) if args.factory else None)
        acq.start()
    except BaseException as e:
        traceback.print_exc()                             # aussi dans le terminal : l'origine exacte de l'échec
        conn.send(("error", type(e).__name__, str(e) or repr(e)))
        conn.close()
        return 2
    conn.send(("ready", info))

    t_start = time.monotonic()
    next_stats = 0.0
    try:
        while True:
            if conn.poll(0.005):
                if conn.recv()[0] == "stop":
                    break
            while True:
                try:
                    rev = acq.rev_queue.get_nowait()
                except queue.Empty:
                    break
                conn.send(("rev", rev.number, rev.points, rev.n_raw, rev.t_start_ns, rev.t_end_ns))
            now = time.monotonic()
            if now >= next_stats:
                next_stats = now + STATS_PERIOD_S
                conn.send(("stats", _stats(acq, t_start)))
            if acq.error is not None or not acq.alive:
                conn.send(("stats", _stats(acq, t_start)))
                conn.send(("error", type(acq.error).__name__ if acq.error else "RuntimeError",
                           repr(acq.error) if acq.error else "un thread d'acquisition s'est arrêté"))
                break
    except (EOFError, OSError):
        pass                                              # le parent a disparu : on arrête le lidar et on part
    finally:
        try:
            acq.stop()
        except Exception:
            pass
        try:
            conn.send(("bye",))
            conn.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(_child_main())

"""Diagnostic : part de CPU consommée par chaque thread du processus (Linux uniquement).

Sert à trouver QUI prend le processeur quand le thread de localisation « attend » (colonne
`attente` des logs) : un thread qui calcule en Python garde le GIL, les autres patientent.

    mon = ThreadCpuMonitor(period_s=2.0, extra_pids={"lidar(fils)": acq.pid}).start()
    # toutes les 2 s :  [CPU] proc 142% | localizer 38% | MainThread 31% | ... | lidar(fils) 22% | charge 2.10

Lecture de /proc/self/task/<tid>/stat (utime + stime) : coût négligeable, rien à installer.
Sur un système sans /proc (Windows, macOS), start() ne fait rien.
"""
import os
import threading
import time


def _cpu_ticks(stat_path):
    """utime + stime (en ticks d'horloge) d'une tâche, ou None si elle a disparu."""
    try:
        with open(stat_path, "rb") as f:
            data = f.read().decode("ascii", "replace")
    except OSError:
        return None
    # Le nom (2e champ) est entre parenthèses et peut contenir des espaces : couper après ')'.
    fields = data[data.rfind(")") + 2:].split()
    return int(fields[11]) + int(fields[12])      # champs 14 (utime) et 15 (stime) de stat


def _comm(tid):
    try:
        with open(f"/proc/self/task/{tid}/comm", "r", encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return str(tid)


class ThreadCpuMonitor:
    def __init__(self, period_s=2.0, top=8, extra_pids=None, printer=print):
        """
        period_s   : intervalle entre deux affichages (s).
        top        : nombre de threads affichés (les plus gourmands d'abord).
        extra_pids : {nom: pid} d'autres processus à suivre (ex. le processus d'acquisition lidar).
        """
        self.period_s = period_s
        self.top = top
        self.extra_pids = dict(extra_pids or {})
        self._print = printer
        self._stop = threading.Event()
        self._thread = None
        try:
            self._hz = os.sysconf("SC_CLK_TCK")
        except (AttributeError, ValueError, OSError):
            self._hz = 100

    @staticmethod
    def available():
        return os.path.isdir("/proc/self/task")

    def start(self):
        if self.available():
            self._thread = threading.Thread(target=self._run, daemon=True, name="cpu-monitor")
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def _sample(self):
        ticks = {}
        try:
            tids = os.listdir("/proc/self/task")
        except OSError:
            tids = []
        for tid in tids:
            v = _cpu_ticks(f"/proc/self/task/{tid}/stat")
            if v is not None:
                ticks[int(tid)] = v
        extra = {}
        for name, pid in self.extra_pids.items():
            if pid:
                v = _cpu_ticks(f"/proc/{pid}/stat")
                if v is not None:
                    extra[name] = v
        return time.monotonic(), ticks, extra

    def _run(self):
        t0, prev, prev_extra = self._sample()
        while not self._stop.wait(self.period_s):
            t1, cur, cur_extra = self._sample()
            dt = max(t1 - t0, 1e-6)
            # tid natif -> nom du thread Python (à défaut, nom système : notifier CAN, BLAS...).
            names = {t.native_id: t.name for t in threading.enumerate() if t.native_id is not None}

            def pct(now, before):
                return 100.0 * (now - before) / self._hz / dt

            per_thread = sorted(((pct(v, prev.get(tid, v)), names.get(tid) or _comm(tid))
                                 for tid, v in cur.items()), reverse=True)
            parts = [f"proc {sum(p for p, _ in per_thread):4.0f}%"]
            parts += [f"{name} {p:3.0f}%" for p, name in per_thread[:self.top] if p >= 1.0]
            for name, v in cur_extra.items():
                parts.append(f"{name} {pct(v, prev_extra.get(name, v)):3.0f}%")
            try:
                parts.append(f"charge {os.getloadavg()[0]:.2f}")
            except (AttributeError, OSError):
                pass
            self._print("[CPU] " + " | ".join(parts), flush=True)
            t0, prev, prev_extra = t1, cur, cur_extra

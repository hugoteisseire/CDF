#!/usr/bin/env python3
"""
lidar_polar.py - Affichage polaire en direct d'un RPLidar C1 + statistiques
d'acquisition dans le terminal.

Architecture : la lecture du lidar tourne dans un thread a part, l'affichage
dans le thread principal. Dessiner ne bloque donc plus la lecture serie (sinon
le tampon serie deborde, les trames se decalent et on perd des points).

Prerequis :
  - le dossier `rplidarc1/` (celui du repo git) place a cote de ce script
  - pip install pyserial matplotlib numpy
  - sudo apt install python3-tk python3-pil.imagetk   (pour la fenetre)

Utilisation :
  python3 lidar_polar.py                      # affichage + stats
  python3 lidar_polar.py --no-plot            # stats seulement (SSH sans ecran)
  python3 lidar_polar.py --port /dev/ttyUSB0 --rmax 4000 --duration 30

Arret : Ctrl+C ou fermeture de la fenetre.
"""

import argparse
import asyncio
import logging
import threading
import time

import numpy as np

from rplidarc1 import RPLidar


# --------------------------------------------------------------------------- #
# Etat partage entre le thread d'acquisition et le thread d'affichage
# --------------------------------------------------------------------------- #
class ErrorCounter(logging.Handler):
    """Compte les WARNING/ERROR de la bibliotheque (trames mal alignees...)."""

    def __init__(self):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        super().__init__(level=logging.WARNING)
        self.count = 0

    def emit(self, record):
        """Enregistre un événement de diagnostic dans le journal."""
        self.count += 1


class Shared:
    """État partagé entre l’acquisition et l’affichage, protégé par un verrou."""
    def __init__(self):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.lock = threading.Lock()
        self.rev = None  # dernier tour complet : tableau (N, 2) [angle_deg, dist_mm]
        self.rev_id = 0  # incremente a chaque nouveau tour
        self.loop = None  # boucle asyncio du thread d'acquisition
        self.error = None  # exception eventuelle du thread d'acquisition
        self.errors = ErrorCounter()


# --------------------------------------------------------------------------- #
# Affichage (thread principal)
# --------------------------------------------------------------------------- #
class Plotter:
    """Figure polaire avec blitting : seul le nuage de points est redessine."""

    def __init__(self, rmax: float, backend: str):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        import matplotlib

        matplotlib.use(backend, force=True)
        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection="polar")
        ax = self.ax
        ax.set_title("RPLidar C1 - dernier tour complet (mm)")
        ax.set_theta_zero_location("N")  # 0 deg vers le haut
        ax.set_theta_direction(-1)  # le RPLidar tourne dans le sens horaire
        ax.set_rmax(rmax)
        ax.set_rlabel_position(135)
        ax.grid(True)
        self.scatter = ax.scatter([], [], s=5, c="tab:blue", animated=True)
        self.bg = None
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        plt.show(block=False)
        self.fig.canvas.draw()  # declenche draw_event -> capture du fond

    def _on_draw(self, event):
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.ax.draw_artist(self.scatter)

    def is_open(self) -> bool:
        """Indique si la fenêtre d’affichage est encore ouverte."""
        return self.plt.fignum_exists(self.fig.number)

    def update(self, rev) -> None:
        """Met à jour les données affichées à partir du dernier tour."""
        if rev is not None and len(rev):
            theta = np.deg2rad(rev[:, 0])
            self.scatter.set_offsets(np.column_stack((theta, rev[:, 1])))
        if self.bg is None:
            self.fig.canvas.draw()
            return
        self.fig.canvas.restore_region(self.bg)
        self.ax.draw_artist(self.scatter)
        self.fig.canvas.blit(self.fig.bbox)

    def close(self) -> None:
        """Ferme proprement la ressource ou la fenêtre associée."""
        self.plt.close("all")


def run_display(plotter: Plotter, shared: Shared, thread: threading.Thread) -> None:
    """Boucle d'affichage : redessine a chaque nouveau tour complet."""
    last_id = 0
    while thread.is_alive():
        if not plotter.is_open():
            print("Fenetre fermee, arret.")
            return
        with shared.lock:
            rev, rev_id = shared.rev, shared.rev_id
        if rev_id != last_id:
            plotter.update(rev)
            last_id = rev_id
        try:
            plotter.fig.canvas.flush_events()
        except Exception:
            return
        time.sleep(0.02)


# --------------------------------------------------------------------------- #
# Acquisition (thread dedie, avec sa propre boucle asyncio)
# --------------------------------------------------------------------------- #
async def consume(lidar: RPLidar, shared: Shared) -> None:
    """Vide la queue du lidar, detecte les tours, affiche les stats."""
    queue = lidar.output_queue
    stop = lidar.stop_event

    current_rev: list = []  # points du tour en cours : (angle_deg, distance_mm)
    last_angle = None

    win_start = time.perf_counter()
    win_points = 0
    win_revs = 0
    win_errs0 = shared.errors.count

    print("Acquisition en cours... (Ctrl+C pour arreter)")
    while not stop.is_set():
        while True:
            try:
                p = queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            angle = p["a_deg"]
            dist = p["d_mm"]
            win_points += 1

            # Nouveau tour : l'angle repasse de ~360 deg a ~0 deg
            if last_angle is not None and last_angle - angle > 180:
                win_revs += 1
                if current_rev:
                    with shared.lock:
                        shared.rev = np.asarray(current_rev, dtype=float)
                        shared.rev_id += 1
                current_rev = []
            last_angle = angle

            if dist is not None and dist > 0:
                current_rev.append((angle, dist))

        now = time.perf_counter()
        dt = now - win_start
        if dt >= 1.0:
            hhmmss = time.strftime("%H:%M:%S")
            errs = shared.errors.count - win_errs0
            if win_revs > 0 and win_points > 0:
                freq = win_revs / dt
                print(
                    f"[{hhmmss}] {freq:5.2f} Hz ({freq * 60:4.0f} tr/min) | "
                    f"{1000 * dt / win_revs:6.1f} ms/tour | "
                    f"{win_points / dt:6.0f} pts/s ({1e6 * dt / win_points:5.0f} us/pt) | "
                    f"{win_points / win_revs:5.0f} pts/tour | err {errs}"
                )
            else:
                print(f"[{hhmmss}] pas de tour complet sur la derniere seconde | err {errs}")
            win_start = now
            win_points = 0
            win_revs = 0
            win_errs0 = shared.errors.count

        await asyncio.sleep(0.01)


async def wait_and_stop(t: float, event: asyncio.Event) -> None:
    """Attend la durée demandée puis déclenche l’arrêt du scan."""
    await asyncio.sleep(t)
    print(f"Duree de {t:.0f} s atteinte, arret.")
    event.set()


def acquisition_thread(lidar: RPLidar, args, shared: Shared) -> None:
    """Exécute la boucle d’acquisition dans un thread séparé."""
    async def main() -> None:
        """Configure puis exécute le scénario principal du programme."""
        shared.loop = asyncio.get_running_loop()
        async with asyncio.TaskGroup() as tg:
            tg.create_task(consume(lidar, shared))
            tg.create_task(lidar.simple_scan())
            if args.duration > 0:
                tg.create_task(wait_and_stop(args.duration, lidar.stop_event))

    try:
        asyncio.run(main())
    except BaseException as e:  # on remonte l'erreur au thread principal
        shared.error = e


def request_stop(lidar: RPLidar, shared: Shared) -> None:
    """Demande l’arrêt du lidar et signale la fin aux autres tâches."""
    loop = shared.loop
    if loop is not None and loop.is_running():
        loop.call_soon_threadsafe(lidar.stop_event.set)
    else:
        lidar.stop_event.set()


# --------------------------------------------------------------------------- #
# Programme principal
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RPLidar C1 : affichage polaire + frequence")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--rmax", type=float, default=6000, help="rayon max affiche (mm)")
    parser.add_argument("--duration", type=float, default=0, help="duree en s (0 = illimite)")
    parser.add_argument("--no-plot", action="store_true", help="statistiques seulement")
    parser.add_argument("--backend", default="TkAgg", help="backend matplotlib (defaut TkAgg)")
    args = parser.parse_args()

    shared = Shared()

    logging.basicConfig(level=logging.ERROR)
    # Les messages de la bibliotheque (trames decalees...) sont comptes ("err"),
    # pas affiches un par un.
    lib_logger = logging.getLogger("rplidarc1")
    lib_logger.setLevel(logging.WARNING)
    lib_logger.addHandler(shared.errors)
    lib_logger.propagate = False

    lidar = RPLidar(args.port, args.baud)

    plotter = None
    if not args.no_plot:
        try:
            plotter = Plotter(args.rmax, args.backend)
        except Exception as e:
            print(f"Affichage graphique impossible : {e!r} -> mode --no-plot.")

    thread = threading.Thread(target=acquisition_thread, args=(lidar, args, shared), daemon=True)
    try:
        thread.start()
        if plotter is not None:
            run_display(plotter, shared, thread)
        else:
            while thread.is_alive():
                thread.join(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        request_stop(lidar, shared)
        thread.join(timeout=3)
        if plotter is not None:
            plotter.close()
        time.sleep(0.5)
        lidar.reset()

    if shared.error is not None:
        print(f"Erreur dans le thread d'acquisition : {shared.error!r}")
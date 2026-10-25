#!/usr/bin/env python3
"""
lidar_polar.py - Affichage polaire en direct d'un RPLidar C1 + statistiques
d'acquisition horodatées dans le terminal, avec export CSV optionnel.

Architecture (deux threads) :
  - thread d'acquisition : lit le lidar (asyncio), détecte les tours, calcule les
    statistiques, horodate chaque point et écrit éventuellement le CSV ;
  - thread principal     : dessine le graphe polaire (matplotlib).
Les deux sont séparés pour que le dessin ne bloque jamais la lecture du port
série. Sinon le tampon série déborde, les trames se décalent et on perd des
points (c'était la cause des erreurs "angle > 360").

Horodatage :
  Chaque point reçoit l'heure locale de sa réception (précision ~ quelques ms).
  La bibliothèque ne fournit pas l'heure de mesure : l'heure est donc estimée au
  moment où le point est lu dans la file, répartie régulièrement entre deux
  lectures (le lidar échantillonne à cadence constante, ~5000 points/s).

Prérequis :
  - le dossier `rplidarc1/` (celui du repo git) placé à côté de ce script
  - pip install pyserial matplotlib numpy
  - sudo apt install python3-tk python3-pil.imagetk   (pour la fenêtre)

Utilisation :
  python3 lidar_polar.py                          # affichage + stats
  python3 lidar_polar.py --no-plot                # stats seulement (SSH sans écran)
  python3 lidar_polar.py --log mesures.csv        # + enregistre tous les points
  python3 lidar_polar.py --port /dev/ttyUSB0 --rmax 4000 --duration 30

Arrêt : Ctrl+C ou fermeture de la fenêtre.
"""

import argparse
import asyncio
import csv
import logging
import threading
import time
from datetime import datetime

import numpy as np

from lidar.rplidarc1 import RPLidar


def horodatage(t=None) -> str:
    """Formate un instant en heure locale 'AAAA-MM-JJ HH:MM:SS.mmm'.

    t : temps epoch en secondes (time.time()). None = maintenant.
    """
    dt = datetime.fromtimestamp(time.time() if t is None else t)
    return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # microsecondes -> millisecondes


# --------------------------------------------------------------------------- #
# État partagé entre le thread d'acquisition et le thread d'affichage
# --------------------------------------------------------------------------- #
class ErrorCounter(logging.Handler):
    """Compte les WARNING/ERROR émis par la bibliothèque (trames mal alignées...)
    au lieu de les afficher un par un dans le terminal."""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.count = 0

    def emit(self, record):
        self.count += 1


class Shared:
    """Données échangées entre les deux threads (protégées par `lock`)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.rev = None  # dernier tour complet : tableau (N, 2) [angle_deg, dist_mm]
        self.rev_time = None  # heure (epoch) de la fin de ce tour
        self.rev_id = 0  # incrémenté à chaque nouveau tour (sert à détecter la nouveauté)
        self.loop = None  # boucle asyncio du thread d'acquisition (pour l'arrêter proprement)
        self.error = None  # exception éventuelle du thread d'acquisition
        self.errors = ErrorCounter()  # compteur de trames invalides


# --------------------------------------------------------------------------- #
# Affichage (thread principal)
# --------------------------------------------------------------------------- #
class Plotter:
    """Figure polaire avec blitting : seuls le nuage de points et l'horodatage
    sont redessinés à chaque tour, le fond (axes, grille) est mis en cache.
    C'est beaucoup plus léger qu'un redessin complet sur un Raspberry Pi."""

    def __init__(self, rmax: float, backend: str):
        import matplotlib

        matplotlib.use(backend, force=True)  # on force le backend (TkAgg par défaut)
        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection="polar")
        ax = self.ax
        ax.set_title("RPLidar C1 - dernier tour complet (mm)")
        ax.set_theta_zero_location("N")  # 0° vers le haut
        ax.set_theta_direction(-1)  # le RPLidar tourne dans le sens horaire
        ax.set_rmax(rmax)
        ax.set_rlabel_position(135)  # position des étiquettes de distance
        ax.grid(True)

        # Artistes "animés" : exclus du fond mis en cache, redessinés à chaque image.
        self.scatter = ax.scatter([], [], s=5, c="tab:blue", animated=True)
        self.stamp = ax.text(
            0.5, -0.08, "", transform=ax.transAxes, ha="center", animated=True
        )  # horodatage du tour affiché, sous le graphe

        self.bg = None  # image du fond, capturée au premier dessin complet
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        plt.show(block=False)
        self.fig.canvas.draw()  # déclenche draw_event -> capture du fond

    def _on_draw(self, event):
        """Appelé après chaque dessin complet : on mémorise le fond."""
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.ax.draw_artist(self.scatter)
        self.ax.draw_artist(self.stamp)

    def is_open(self) -> bool:
        """False si l'utilisateur a fermé la fenêtre."""
        return self.plt.fignum_exists(self.fig.number)

    def update(self, rev, rev_time) -> None:
        """Affiche un nouveau tour (tableau [angle_deg, dist_mm]) et son horodatage."""
        if rev is not None and len(rev):
            theta = np.deg2rad(rev[:, 0])  # matplotlib attend des radians
            self.scatter.set_offsets(np.column_stack((theta, rev[:, 1])))
        if rev_time is not None:
            self.stamp.set_text(horodatage(rev_time))
        if self.bg is None:  # pas encore de fond en cache : dessin complet
            self.fig.canvas.draw()
            return
        # Blitting : on restaure le fond puis on ne redessine que les points + texte.
        self.fig.canvas.restore_region(self.bg)
        self.ax.draw_artist(self.scatter)
        self.ax.draw_artist(self.stamp)
        self.fig.canvas.blit(self.fig.bbox)

    def close(self) -> None:
        self.plt.close("all")


def run_display(plotter: Plotter, shared: Shared, thread: threading.Thread) -> None:
    """Boucle d'affichage : redessine à chaque nouveau tour complet, tant que le
    thread d'acquisition tourne et que la fenêtre est ouverte."""
    last_id = 0
    while thread.is_alive():
        if not plotter.is_open():
            print("Fenetre fermee, arret.")
            return
        with shared.lock:  # lecture cohérente du dernier tour
            rev, rev_time, rev_id = shared.rev, shared.rev_time, shared.rev_id
        if rev_id != last_id:  # nouveau tour disponible ?
            plotter.update(rev, rev_time)
            last_id = rev_id
        try:
            plotter.fig.canvas.flush_events()  # laisse la GUI traiter souris/fermeture
        except Exception:
            return
        time.sleep(0.02)  # ~50 itérations/s, largement suffisant pour 10 tours/s


# --------------------------------------------------------------------------- #
# Acquisition (thread dédié, avec sa propre boucle asyncio)
# --------------------------------------------------------------------------- #
async def consume(lidar: RPLidar, args, shared: Shared) -> None:
    """Vide la file du lidar, horodate les points, détecte les tours,
    alimente l'affichage, écrit le CSV et imprime les statistiques (1 fois/s)."""
    queue = lidar.output_queue
    stop = lidar.stop_event

    current_rev: list = []  # points valides du tour en cours : (angle_deg, distance_mm)
    last_angle = None  # angle du point précédent (pour détecter le passage 360° -> 0°)
    rev_num = 0  # numéro du tour en cours (écrit dans le CSV)
    prev_ts = time.time()  # horodatage du point précédent

    # Fenêtre de statistiques d'une seconde (chronométrée avec perf_counter,
    # horloge monotone, insensible aux changements d'heure système).
    win_start = time.perf_counter()
    win_points = 0
    win_revs = 0
    win_errs0 = shared.errors.count

    # Export CSV optionnel : un point par ligne, avec son horodatage.
    log_file = writer = None
    if args.log:
        log_file = open(args.log, "w", newline="", buffering=1 << 16)
        writer = csv.writer(log_file)
        writer.writerow(["horodatage", "epoch_s", "tour", "angle_deg", "distance_mm", "qualite"])

    last_drain = time.time()  # instant de la lecture précédente de la file
    print("Acquisition en cours... (Ctrl+C pour arreter)")
    try:
        while not stop.is_set():
            # 1) On récupère d'un coup tous les points disponibles dans la file.
            batch = []
            while True:
                try:
                    batch.append(queue.get_nowait())
                except asyncio.QueueEmpty:
                    break

            # 2) Horodatage : les n points lus ont été reçus entre `last_drain` et
            #    `t_now`. Le lidar échantillonne à cadence constante, donc on les
            #    répartit régulièrement dans cet intervalle.
            t_now = time.time()
            n = len(batch)
            for i, p in enumerate(batch):
                ts = last_drain + (i + 1) * (t_now - last_drain) / n
                angle = p["a_deg"]  # angle en degrés (0-360)
                dist = p["d_mm"]  # distance en mm, None si mesure invalide
                t_rx_ns = p["t_rx_ns"]
                quality = p["q"]  # qualité de mesure (0-255)
                win_points += 1

                # Nouveau tour : l'angle repasse de ~360° à ~0°.
                if last_angle is not None and last_angle - angle > 180:
                    win_revs += 1
                    rev_num += 1
                    if current_rev:
                        with shared.lock:  # publie le tour terminé pour l'affichage
                            shared.rev = np.asarray(current_rev, dtype=float)
                            shared.rev_time = prev_ts  # heure du dernier point du tour
                            shared.rev_id += 1
                    current_rev = []
                last_angle = angle
                prev_ts = ts

                if dist is not None and dist > 0:  # on ne trace que les mesures valides
                    current_rev.append((angle, dist, quality, t_rx_ns))

                if writer is not None:  # on enregistre tout, y compris les invalides
                    writer.writerow(
                        [
                            horodatage(ts),
                            f"{ts:.3f}",
                            rev_num,
                            angle,
                            "" if dist is None else dist,
                            p["q"],
                        ]
                    )
            if n:  # si rien n'est arrivé, on garde l'ancien repère pour le prochain lot
                last_drain = t_now

            # 3) Statistiques une fois par seconde, horodatées.
            now = time.perf_counter()
            dt = now - win_start
            if dt >= 1.0:
                stamp = horodatage()
                errs = shared.errors.count - win_errs0  # trames invalides sur la fenêtre
                if win_revs > 0 and win_points > 0:
                    freq = win_revs / dt  # tours par seconde = fréquence de scan
                    print(
                        f"[{stamp}] {freq:5.2f} Hz ({freq * 60:4.0f} tr/min) | "
                        f"{1000 * dt / win_revs:6.1f} ms/tour | "
                        f"{win_points / dt:6.0f} pts/s ({1e6 * dt / win_points:5.0f} us/pt) | "
                        f"{win_points / win_revs:5.0f} pts/tour | err {errs}"
                    )
                else:
                    print(f"[{stamp}] pas de tour complet sur la derniere seconde | err {errs}")
                win_start = now  # on repart sur une nouvelle fenêtre
                win_points = 0
                win_revs = 0
                win_errs0 = shared.errors.count

            await asyncio.sleep(0.01)  # cède la main à la lecture série (simple_scan)
    finally:
        if log_file is not None:  # vide le tampon et ferme le CSV même en cas d'erreur
            log_file.close()
            print(f"Mesures enregistrees dans {args.log}")


async def wait_and_stop(t: float, event: asyncio.Event) -> None:
    """Déclenche l'arrêt du scan après t secondes (option --duration)."""
    await asyncio.sleep(t)
    print(f"Duree de {t:.0f} s atteinte, arret.")
    event.set()


def acquisition_thread(lidar: RPLidar, args, shared: Shared) -> None:
    """Corps du thread d'acquisition : lance la boucle asyncio et ses 2-3 tâches."""

    async def main() -> None:
        shared.loop = asyncio.get_running_loop()  # mémorisé pour pouvoir l'arrêter
        async with asyncio.TaskGroup() as tg:
            tg.create_task(consume(lidar, args, shared))  # traitement des points
            tg.create_task(lidar.simple_scan())  # lecture du port série (bibliothèque)
            if args.duration > 0:
                tg.create_task(wait_and_stop(args.duration, lidar.stop_event))

    try:
        asyncio.run(main())
    except BaseException as e:  # on remonte l'erreur au thread principal
        shared.error = e


def request_stop(lidar: RPLidar, shared: Shared) -> None:
    """Demande l'arrêt du scan depuis le thread principal.
    asyncio.Event n'est pas thread-safe : on passe par la boucle du thread d'acquisition."""
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
    parser.add_argument("--log", metavar="FICHIER.csv", help="enregistre tous les points horodates")
    args = parser.parse_args()

    shared = Shared()

    # Logs : on coupe les messages de debug, et les alertes de la bibliothèque
    # (trames décalées...) sont comptées ("err") au lieu d'être affichées.
    logging.basicConfig(level=logging.ERROR)
    lib_logger = logging.getLogger("lidar.rplidarc1")
    lib_logger.setLevel(logging.WARNING)
    lib_logger.addHandler(shared.errors)
    lib_logger.propagate = False

    lidar = RPLidar(args.port, args.baud)  # ouvre le port série + test de santé

    # Fenêtre graphique (sauf --no-plot). Si elle ne peut pas s'ouvrir, on
    # continue avec les statistiques seules et on affiche la raison.
    plotter = None
    if not args.no_plot:
        try:
            plotter = Plotter(args.rmax, args.backend)
        except Exception as e:
            print(f"Affichage graphique impossible : {e!r} -> mode --no-plot.")

    # L'acquisition tourne en thread "daemon" : elle ne bloque pas la fermeture du programme.
    thread = threading.Thread(target=acquisition_thread, args=(lidar, args, shared), daemon=True)
    try:
        thread.start()
        if plotter is not None:
            run_display(plotter, shared, thread)  # bloque jusqu'à l'arrêt
        else:
            while thread.is_alive():  # attente active courte, interruptible par Ctrl+C
                thread.join(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt propre : stoppe le scan, attend le thread, ferme la fenêtre,
        # puis remet le lidar à zéro (arrête aussi le moteur).
        request_stop(lidar, shared)
        thread.join(timeout=3)
        if plotter is not None:
            plotter.close()
        time.sleep(0.5)
        lidar.reset()

    if shared.error is not None:
        print(f"Erreur dans le thread d'acquisition : {shared.error!r}")
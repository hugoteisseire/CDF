#!/usr/bin/env python3
"""
lidar_demo.py - Ton script lidar_polar.py reconstruit sur lidar_core.

Le tracé polaire reste dans le thread principal et ne fait que LIRE `latest()` :
il ne peut plus ralentir ni la lecture serie ni les autres consommateurs.
Bleu = points conserves apres capping, rouge = points des clusters, croix noire =
centre du poteau estime.

Utilisation :
  python3 lidar_demo.py                         # vrai lidar, affichage
  python3 lidar_demo.py --no-plot               # statistiques et clusters dans le terminal
  python3 lidar_demo.py --sim --duration 5      # lidar simule (sans materiel)
  python3 lidar_demo.py --sim --duration 3 --snapshot tour.png --backend Agg
"""

import argparse
import math
import time

import numpy as np

from lidar.legacy.lidar_core import LidarConfig, LidarService


class Plotter:
    """Figure polaire avec blitting : seuls les nuages de points sont redessines."""

    def __init__(self, rmax: float, backend: str):
        import matplotlib

        matplotlib.use(backend, force=True)
        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection="polar")
        ax = self.ax
        ax.set_title("RPLidar C1 - dernier tour (mm)")
        ax.set_theta_zero_location("N")  # 0 deg vers le haut
        ax.set_theta_direction(-1)  # le RPLidar tourne dans le sens horaire
        ax.set_rmax(rmax)
        ax.set_rlabel_position(135)
        ax.grid(True)
        self.pts = ax.scatter([], [], s=5, c="tab:blue", animated=True)
        self.clu = ax.scatter([], [], s=14, c="tab:red", animated=True)
        self.ctr = ax.scatter([], [], s=70, c="black", marker="x", animated=True)
        self.artists = (self.pts, self.clu, self.ctr)
        self.bg = None
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        plt.show(block=False)
        self.fig.canvas.draw()  # declenche draw_event -> capture du fond

    def _draw_artists(self):
        for a in self.artists:
            self.ax.draw_artist(a)

    def _on_draw(self, event):
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_artists()

    def is_open(self) -> bool:
        return self.plt.fignum_exists(self.fig.number)

    def update(self, scan) -> None:
        theta = np.deg2rad(scan.angle_deg)
        r_mm = scan.r_m * 1000.0
        in_c = scan.label >= 0
        self.pts.set_offsets(np.column_stack((theta[~in_c], r_mm[~in_c])))
        self.clu.set_offsets(np.column_stack((theta[in_c], r_mm[in_c])))
        cs = scan.clusters
        self.ctr.set_offsets(
            np.column_stack((np.deg2rad([c.a_deg for c in cs]), [c.r_m * 1000.0 for c in cs])) if cs else np.empty((0, 2))
        )
        if self.bg is None:
            self.fig.canvas.draw()
            return
        self.fig.canvas.restore_region(self.bg)
        self._draw_artists()
        self.fig.canvas.blit(self.fig.bbox)

    def save(self, path: str) -> None:
        self.fig.canvas.draw()
        self.fig.savefig(path, dpi=110)

    def close(self) -> None:
        self.plt.close("all")


def describe(scan) -> str:
    if scan is None:
        return "pas encore de tour"
    cl = " ; ".join(
        f"{math.degrees(c.phi_rad):+7.2f} deg {c.r_m:5.3f} m ({c.n} pts)" for c in scan.clusters
    )
    return f"{len(scan.clusters)} cluster(s) : {cl}"


def print_stats(svc: LidarService) -> None:
    st = svc.stats()
    print(
        f"[{time.strftime('%H:%M:%S')}] {st['rev_hz']:5.2f} Hz | {st['pts_per_rev']:4d} pts/tour | "
        f"err lib {st['lib_errors']} | tours rejetes {st['bad_revs']} | perdus {st['proc_skipped']} | "
        f"traitement {st['proc_ms']:4.1f} ms | {describe(svc.latest())}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RPLidar C1 : acquisition, capping, clusters, affichage")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--sim", action="store_true", help="lidar simule (sans materiel)")
    parser.add_argument("--rmax", type=float, default=3000, help="capping : distance max conservee (mm)")
    parser.add_argument("--rmin", type=float, default=100, help="capping : distance min conservee (mm)")
    parser.add_argument("--duration", type=float, default=0, help="duree en s (0 = illimite)")
    parser.add_argument("--no-plot", action="store_true", help="pas de fenetre")
    parser.add_argument("--backend", default="TkAgg", help="backend matplotlib (defaut TkAgg)")
    parser.add_argument("--snapshot", default="", help="enregistre un PNG du dernier tour a l'arret")
    args = parser.parse_args()

    if args.sim:
        from lidar_sim import SimRPLidar

        lidar = SimRPLidar()
    else:
        from lidar.rplidarc1 import RPLidar

        lidar = RPLidar(args.port, args.baud)

    cfg = LidarConfig(rmin_mm=args.rmin, rmax_mm=args.rmax)

    plotter = None
    if not args.no_plot or args.snapshot:
        try:
            plotter = Plotter(args.rmax, args.backend)
        except Exception as e:
            print(f"Affichage graphique impossible : {e!r} -> mode --no-plot.")

    svc = LidarService(lidar, cfg)
    print("Acquisition en cours... (Ctrl+C pour arreter)")
    t0 = time.monotonic()
    last_id, last_print = -1, time.monotonic()
    try:
        svc.start()
        while svc.alive():
            if args.duration > 0 and time.monotonic() - t0 >= args.duration:
                print(f"Duree de {args.duration:.0f} s atteinte, arret.")
                break
            if plotter is not None and not args.no_plot:
                if not plotter.is_open():
                    print("Fenetre fermee, arret.")
                    break
                scan = svc.latest()
                if scan is not None and scan.seq != last_id:
                    plotter.update(scan)
                    last_id = scan.seq
                try:
                    plotter.fig.canvas.flush_events()
                except Exception:
                    break
            if time.monotonic() - last_print >= 1.0:
                print_stats(svc)
                last_print = time.monotonic()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if plotter is not None and args.snapshot and svc.latest() is not None:
            plotter.update(svc.latest())
            plotter.save(args.snapshot)
            print(f"Image enregistree : {args.snapshot}")
        svc.stop()
        if plotter is not None:
            plotter.close()
    if svc.error is not None:
        print(f"Erreur dans le thread d'acquisition : {svc.error!r}")

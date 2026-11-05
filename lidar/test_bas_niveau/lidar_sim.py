#!/usr/bin/env python3
"""
lidar_sim.py - Faux RPLidar pour tester sans materiel.

Meme interface que la classe RPLidar de `rplidarc1` telle que ton script l'utilise :
  .output_queue (asyncio.Queue de dicts {"a_deg", "d_mm"}), .stop_event (asyncio.Event),
  .simple_scan() (coroutine), .reset().

Le monde : 3 poteaux de 10 cm de diametre dans les coins (0,0), (2,0), (0,2) d'un terrain
de 2 m. Par defaut, les bords du terrain sont sous le plan du lidar (walls=False) : seuls
les poteaux renvoient une mesure, le reste est "sans mesure" comme sur un vrai RPLidar.
Avec walls=True, les bords sont vus par le lidar. Le robot est a la pose `pose_fn(t)`.
Le lidar tourne en sens horaire, 10 Hz, 500 points par tour (5000 pts/s),
bruit gaussien sur la distance, quelques points sans mesure.
"""

import asyncio
import math
import time

import numpy as np

POLES = ((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))


def raycast(x, y, psi, size=2.0, poles=POLES, pole_r=0.05, walls=False):
    """Distance (m) du point (x, y) au premier obstacle dans la direction psi (tableaux numpy).
    inf = aucun obstacle."""
    c, s = np.cos(psi), np.sin(psi)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(c > 1e-12, (size - x) / c, np.where(c < -1e-12, -x / c, np.inf))
        ty = np.where(s > 1e-12, (size - y) / s, np.where(s < -1e-12, -y / s, np.inf))
    t = np.minimum(tx, ty) if walls else np.full_like(tx, np.inf)
    for px, py in poles:
        ox, oy = x - px, y - py
        b = c * ox + s * oy
        disc = b * b - (ox * ox + oy * oy - pole_r**2)
        hit = disc > 0
        th = -b - np.sqrt(np.where(hit, disc, 0.0))
        t = np.where(hit & (th > 0) & (th < t), th, t)
    return t


def pole_truth(pose, poles=POLES):
    """Verite terrain : (phi_rad, r_m) du CENTRE de chaque poteau, repere robot."""
    x, y, th = pose
    out = []
    for px, py in poles:
        out.append((math.remainder(math.atan2(py - y, px - x) - th, 2 * math.pi), math.hypot(px - x, py - y)))
    return out


class SimRPLidar:
    """Émule l’interface du lidar pour exécuter les tests sans matériel."""
    def __init__(self, pose_fn=None, noise_mm=30.0, dropout=0.02, rate_hz=10.0, pts_per_rev=500, seed=1, walls=False):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        self.pose_fn = pose_fn or (lambda t: (np.full_like(t, 0.6), np.full_like(t, 0.8), np.full_like(t, 0.3)))
        self.noise_mm, self.dropout, self.walls = noise_mm, dropout, walls
        self.rate_hz, self.pts = rate_hz, pts_per_rev
        self.rng = np.random.default_rng(seed)
        self.output_queue = asyncio.Queue()
        self.stop_event = asyncio.Event()
        self.t0 = None
        self.resets = 0

    async def simple_scan(self):
        """Produit en continu les mesures simulées sous la même interface que le lidar réel."""
        self.t0 = time.perf_counter()
        dt = 1.0 / (self.rate_hz * self.pts)
        i = 0
        while not self.stop_event.is_set():
            n = int((time.perf_counter() - self.t0) / dt)
            if n > i:
                k = np.arange(i, n)
                a_deg = (k % self.pts) * 360.0 / self.pts
                x, y, th = self.pose_fn(self.t0 * 0 + k * dt)
                psi = th - np.radians(a_deg)  # sens horaire : phi = -angle
                d_mm = raycast(x, y, psi, walls=self.walls) * 1000.0 + self.rng.normal(0.0, self.noise_mm, len(k))
                lost = ~np.isfinite(d_mm) | (self.rng.random(len(k)) < self.dropout)
                for j in range(len(k)):
                    if lost[j]:
                        self.output_queue.put_nowait({"a_deg": float(a_deg[j]), "d_mm": None if j % 2 else 0})
                    else:
                        self.output_queue.put_nowait({"a_deg": float(a_deg[j]), "d_mm": float(d_mm[j])})
                i = n
            await asyncio.sleep(0.005)

    def reset(self):
        """Réinitialise l’état de l’interface du lidar."""
        self.resets += 1

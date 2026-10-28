"""Traitement d'un tour de lidar : capping (filtrage en distance) et détection des clusters.

Un cluster est un groupe de points consécutifs qui appartiennent au même objet (poteau, robot
adverse, bord...). Chaque cluster garde l'horodatage de ses points : l'EKF s'en sert pour
corriger la pose à l'instant où le lidar a réellement vu l'objet.
"""
import math
from dataclasses import dataclass

import numpy as np

# Un point mesuré. Tableau structuré : évite de mélanger des angles et des horodatages en ns
# (~1,8e18) dans un même tableau de flottants, ce qui ferait perdre de la précision sur les ns.
POINT_DTYPE = np.dtype(
    [
        ("angle", "f8"),  # angle lidar en degrés, sens horaire, 0-360
        ("dist", "f8"),   # distance en mm
        ("qual", "u1"),   # qualité de la mesure (0-63)
        ("t_ns", "i8"),   # heure de réception, ns depuis 1970 (epoch)
    ]
)


@dataclass
class ClusterParams:
    # --- capping : on ne garde que les distances utiles ---
    """Paramètres de filtrage des points et de regroupement des mesures proches."""
    d_min_mm: float = 80.0       # en dessous : le robot lui-même / bruit de proximité
    d_max_mm: float = 3200.0     # diagonale du terrain 2 m x 2 m = 2,83 m, plus une marge
    # --- découpage en clusters ---
    max_angle_gap_deg: float = 2.2    # trou d'angle toléré dans un cluster (~3 pas du lidar de 0,72°)
    max_dist_jump_mm: float = 150.0   # saut de distance toléré entre points voisins avant de séparer 2 objets
    min_points: int = 2               # moins de points : bruit, pas un objet


@dataclass
class Cluster:
    """Représente un groupe de points lidar correspondant potentiellement à un objet."""
    angle_deg: float      # angle moyen (lidar, sens horaire, 0-360)
    dist_mm: float        # distance moyenne de la SURFACE vue (pas du centre de l'objet)
    n_points: int
    width_mm: float       # largeur : distance entre le premier et le dernier point
    t_ns: int             # instant moyen des points (epoch, ns)
    t_first_ns: int
    t_last_ns: int
    seam: bool            # cluster à cheval sur 0°/360° : ses points viennent du début ET de la fin
                          # du tour (~100 ms d'écart), donc moins fiable si le robot bouge


def cap_points(points: np.ndarray, cp: ClusterParams = None) -> np.ndarray:
    """Capping : retire les points trop proches ou trop loin pour être utiles."""
    cp = cp or ClusterParams()
    keep = (points["dist"] >= cp.d_min_mm) & (points["dist"] <= cp.d_max_mm)
    return points[keep]


def filter_points_in_field(
    points: np.ndarray,
    state_at,
    field_size_mm=(3000.0, 2000.0),
    margin_mm: float = 200.0,
    lidar_offset_mm=(0.0, 0.0),
    lidar_zero_deg: float = 0.0,
) -> np.ndarray:
    """Retire les mesures dont le point d'impact est hors du terrain + marge.

    Chaque mesure est projetée dans le repère terrain avec la pose EKF interpolée à son
    horodatage. Les angles du lidar sont horaires ; ceux du repère terrain sont anti-horaires.
    state_at(t_s) doit renvoyer un état ayant un attribut .x = (x_mm, y_mm, theta_rad),
    ou None si aucun état n'est disponible.
    """
    if len(points) == 0:
        return points

    field_x_mm, field_y_mm = field_size_mm
    xmin, xmax = -margin_mm, field_x_mm + margin_mm
    ymin, ymax = -margin_mm, field_y_mm + margin_mm
    ox, oy = lidar_offset_mm
    keep = np.ones(len(points), dtype=bool)

    for i, point in enumerate(points):
        state = state_at(float(point["t_ns"]) / 1e9)
        if state is None:
            # Pas de pose exploitable : ne pas supprimer arbitrairement la mesure.
            continue
        x, y, theta = state.x
        c, s = math.cos(theta), math.sin(theta)
        lidar_x = x + c * ox - s * oy
        lidar_y = y + s * ox + c * oy

        angle = theta + math.radians(lidar_zero_deg - float(point["angle"]))
        hit_x = lidar_x + float(point["dist"]) * math.cos(angle)
        hit_y = lidar_y + float(point["dist"]) * math.sin(angle)
        keep[i] = xmin <= hit_x <= xmax and ymin <= hit_y <= ymax

    return points[keep]


def find_clusters(points: np.ndarray, cp: ClusterParams = None) -> list:
    """Découpe un tour (points valides) en clusters. Les points voisins en angle et en distance
    sont regroupés ; un cluster à cheval sur 0°/360° est recollé."""
    cp = cp or ClusterParams()
    if len(points) < cp.min_points:
        return []
    pts = points[np.argsort(points["angle"], kind="stable")]
    a, d = pts["angle"], pts["dist"]
    gap = np.diff(a, append=a[0] + 360.0)      # écart d'angle avec le point suivant (le dernier boucle sur le 1er)
    jump = np.abs(np.diff(d, append=d[0]))
    brk = (gap > cp.max_angle_gap_deg) | (jump > cp.max_dist_jump_mm)   # brk[i] : coupure après le point i

    ends = np.flatnonzero(brk)
    if len(ends) == 0:                         # un seul objet continu sur 360°
        groups = [pts]
    else:
        shift = -(ends[0] + 1)                 # on repart juste après une coupure : le recollage de 0°/360° se fait tout seul
        pts, brk = np.roll(pts, shift), np.roll(brk, shift)
        ends = np.flatnonzero(brk)
        starts = np.concatenate(([0], ends[:-1] + 1))
        groups = [pts[s:e + 1] for s, e in zip(starts, ends)]

    return [_make_cluster(g) for g in groups if len(g) >= cp.min_points]


def _make_cluster(g: np.ndarray) -> Cluster:
    a, d, t = g["angle"], g["dist"], g["t_ns"]
    au = a[0] + (a - a[0] + 180.0) % 360.0 - 180.0         # angles "déroulés" autour du 1er point (cluster sur 0°/360°)
    th = np.radians(au)
    x, y = d * np.cos(th), d * np.sin(th)
    t0 = int(t[0])
    return Cluster(
        angle_deg=float(np.mean(au) % 360.0),
        dist_mm=float(np.mean(d)),
        n_points=len(g),
        width_mm=float(math.hypot(x[-1] - x[0], y[-1] - y[0])),
        t_ns=t0 + int(np.mean(t - t0)),
        t_first_ns=t0,
        t_last_ns=int(t[-1]),
        seam=bool(a.max() - a.min() > 180.0),
    )

"""Étape 3 du plan : association des clusters du lidar aux poteaux connus.

Le filtre ne cherche pas les poteaux « dans tout le tour » : pour chaque poteau il calcule où il
devrait apparaître (avec la pose à l'instant du cluster) et ne regarde que dans une fenêtre autour.
  - un cluster de la bonne taille dans la fenêtre : poteau associé ;
  - deux : on ne choisit pas, le poteau est sauté pour ce tour ;
  - un cluster hors de toute fenêtre est ignoré, jamais pris pour un poteau.
Aucune correction de pose ici : c'est l'étape 4. On calcule seulement les écarts (innovations).
"""
from dataclasses import dataclass
from typing import List, Optional

from .ekf import PoseEKF, wrap_deg
from .lidar_clusters import Cluster


@dataclass
class AssocParams:
    # Fenêtre = n_sigma x incertitude attendue de la mesure, bornée par un plancher et un plafond.
    # Au départ (P = 20 mm, 1°) cela donne environ ±3° et ±100 mm, comme dans le plan ; la fenêtre se
    # resserre quand P diminue et s'élargit quand P grandit.
    """Seuils géométriques et d’incertitude utilisés pour associer les clusters aux poteaux."""
    n_sigma: float = 3.0
    win_angle_min_deg: float = 1.5
    win_angle_max_deg: float = 15.0
    win_dist_min_mm: float = 50.0
    win_dist_max_mm: float = 400.0
    # Taille d'un cluster compatible avec un poteau de 100 mm
    width_min_mm: float = 15.0
    width_max_mm: float = 150.0
    min_points: int = 2


@dataclass
class PoleMatch:
    """Association d’un cluster à un poteau attendu, avec les résidus de mesure."""
    pole: int
    status: str                         # "matched", "absent" ou "ambiguous"
    n_candidates: int
    exp_dist_mm: float                  # mesure attendue (centre du poteau)
    exp_angle_deg: float
    win_dist_mm: float                  # demi-fenêtre utilisée
    win_angle_deg: float
    cluster: Optional[Cluster] = None   # seulement si matched
    d_dist_mm: Optional[float] = None   # écart mesure - attendu (distance), seulement si matched
    d_angle_deg: Optional[float] = None  # écart (angle, dans [-180, 180[)


@dataclass
class AssocResult:
    """Bilan des associations de poteaux pour un tour de lidar."""
    matches: List[PoleMatch]
    n_clusters: int          # clusters du tour
    n_size_ok: int           # ... dont la taille correspond à un poteau
    n_unused: int            # clusters qui n'ont été candidats d'aucun poteau (ignorés)
    n_clamped: int           # états demandés hors de l'historique : horloges lidar/odométrie décalées ?


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def _windows(ekf: PoseEKF, P, H, ap: AssocParams):
    sd, sa = ekf.predicted_std(P, H)
    return (_clip(ap.n_sigma * sd, ap.win_dist_min_mm, ap.win_dist_max_mm),
            _clip(ap.n_sigma * sa, ap.win_angle_min_deg, ap.win_angle_max_deg))


def associate_poles(ekf: PoseEKF, clusters: List[Cluster], t_ref_s: float,
                    ap: Optional[AssocParams] = None) -> AssocResult:
    """t_ref_s : instant de référence du tour (epoch, s), utilisé pour afficher la mesure attendue
    des poteaux qui n'ont aucun candidat."""
    ap = ap or AssocParams()
    p = ekf.p
    n_poles = len(p.poles_mm)
    cands = [[] for _ in range(n_poles)]       # par poteau : liste de (index du cluster, détails)
    n_size_ok = n_clamped = 0

    for ci, c in enumerate(clusters):
        if not (c.n_points >= ap.min_points and ap.width_min_mm <= c.width_mm <= ap.width_max_mm):
            continue
        n_size_ok += 1
        # Pose à l'instant des points du cluster (à 1 m/s, 100 ms = 100 mm : bien plus que ±30 mm).
        st = ekf.state_at(c.t_ns / 1e9 - p.lidar_latency_s)
        n_clamped += st.clamped
        meas_d = c.dist_mm + p.pole_radius_mm        # le lidar voit la surface : +rayon = centre
        for j in range(n_poles):
            d_exp, a_exp, H = ekf.measurement(st.x, j)
            wd, wa = _windows(ekf, st.P, H, ap)
            dd, da = meas_d - d_exp, wrap_deg(c.angle_deg - a_exp)
            if abs(dd) <= wd and abs(da) <= wa:
                cands[j].append((ci, d_exp, a_exp, wd, wa, dd, da))

    claimed = {}                                   # cluster -> poteaux qui le revendiquent
    for j, lst in enumerate(cands):
        for e in lst:
            claimed.setdefault(e[0], []).append(j)

    ref = ekf.state_at(t_ref_s)
    matches = []
    for j in range(n_poles):
        lst = cands[j]
        
        if not lst:
            d_exp, a_exp, H = ekf.measurement(ref.x, j)
            wd, wa = _windows(ekf, ref.P, H, ap)
            matches.append(PoleMatch(j, "absent", 0, d_exp, a_exp, wd, wa))
        elif len(lst) > 1 or len(claimed[lst[0][0]]) > 1:     # deux clusters, ou un cluster pour deux poteaux
            ci, d_exp, a_exp, wd, wa, _, _ = lst[0]
            matches.append(PoleMatch(j, "ambiguous", len(lst), d_exp, a_exp, wd, wa))
        else:
            ci, d_exp, a_exp, wd, wa, dd, da = lst[0]
            matches.append(PoleMatch(j, "matched", 1, d_exp, a_exp, wd, wa, clusters[ci], dd, da))

    return AssocResult(matches, len(clusters), n_size_ok, len(clusters) - len(claimed), n_clamped)

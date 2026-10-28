"""Étape 6 du plan : relocalisation depuis zéro par triangulation directe.

Quand le filtre est perdu (poteaux vus mais jamais acceptés pendant plus d'1 s), on ne sait plus quel
cluster est quel poteau. On essaie donc toutes les affectations de 2 clusters de la taille d'un poteau à 2
poteaux connus :
  - les deux distances donnent deux cercles autour des poteaux ; leur intersection donne une première
    position du lidar (2 candidats, symétriques par rapport à la droite des poteaux) et les angles un cap ;
  - chaque candidat est affiné par Gauss-Newton sur les 4 mesures (2 distances + 2 angles). Les angles sont
    indispensables : quand le robot est près de la droite qui joint les deux poteaux, les cercles sont presque
    tangents et la position déduite des seules distances est très imprécise, alors que l'angle sous lequel on
    voit les deux poteaux la contraint fortement ;
  - le candidat symétrique et les mauvaises affectations sont rejetés par leur résidu (chi²) ;
  - la pose doit tomber sur le terrain (marge comprise) ;
  - on garde la pose qui explique le plus de clusters (le 3e poteau valide les deux premiers).

Limite physique : avec seulement 2 poteaux visibles, deux poses peuvent expliquer exactement les mêmes mesures
quand les deux poteaux sont symétriques (échanger leurs rôles et refléter la pose redonne les mêmes distances et le
même écart d'angle). Toutes les solutions valides sont donc collectées ; si elles sont ambiguës, on retient celle
qui est proche de la pose actuelle du filtre (a priori) quand celle-ci est précise ET nettement plus proche d'une
solution que des autres ; sinon on REFUSE de répondre : une mauvaise relocalisation est pire que pas de relocalisation.
"""
import itertools
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .ekf import PoseEKF, wrap_deg, wrap_pi
from .lidar_clusters import Cluster


@dataclass
class RelocParams:
    width_min_mm: float = 15.0          # taille d'un cluster compatible avec un poteau de 100 mm
    width_max_mm: float = 150.0
    min_points: int = 2
    chord_tol_mm: float = 150.0         # pré-filtre : la corde entre deux clusters doit égaler l'écart entre leurs poteaux
    tangent_tol_mm: float = 150.0       # cercles presque tangents : on tolère l'écart dû au bruit de mesure
    start_offset_min_mm: float = 80.0   # écart minimal à la droite des poteaux pour les points de départ (les deux côtés)
    max_chi2: float = 30.0              # résidu normalisé maximal des 4 mesures après affinage (4 ddl : ~13 à 99 %, marge)
    inlier_dist_mm: float = 120.0       # cluster qui « explique » un poteau pour la pose candidate
    inlier_angle_deg: float = 4.0
    min_inliers: int = 2                # les deux poteaux de départ au minimum
    merge_dist_mm: float = 150.0        # deux solutions plus proches que ça (et que merge_angle_deg) sont la même
    merge_angle_deg: float = 5.0
    prior_max_dist_mm: float = 300.0    # ambiguïté : erreur maximale supposée de la pose du filtre pour trancher
    prior_sep_factor: float = 2.0       # ... et toute autre solution doit être à plus de factor x cette distance de la pose


@dataclass
class RelocResult:
    x_mm: float                         # pose du ROBOT dans le repère terrain
    y_mm: float
    theta_rad: float
    n_inliers: int                      # poteaux expliqués par cette pose (2 ou 3)
    residual: float                     # somme des écarts normalisés (plus petit = meilleur)
    assignment: tuple                   # ((poteau, index du cluster), ...) retenu
    ambiguous: bool = False             # True si plusieurs poses expliquaient aussi bien les mesures (tranché par l'a priori)


def _start_points(p0, r0, p1, r1, tol, min_offset):
    """Positions de départ du lidar : intersection des deux cercles, des deux côtés de la droite des poteaux.
    Cercles presque tangents (ou légèrement disjoints à cause du bruit) : on s'écarte de min_offset de la droite,
    l'affinage par les angles fait le reste."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(dx, dy)
    if d < 1e-6 or d > r0 + r1 + tol or d < abs(r0 - r1) - tol:
        return []
    a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
    h = max(math.sqrt(max(r0 * r0 - a * a, 0.0)), min_offset)
    mx, my = p0[0] + a * dx / d, p0[1] + a * dy / d
    return [(mx + h * dy / d, my - h * dx / d), (mx - h * dy / d, my + h * dx / d)]


def _refine(ekf: PoseEKF, pose0, meas, sd_mm, sa_rad, iters=12):
    """Gauss-Newton sur (x mm, y mm, θ rad). meas = [(pôle, distance au centre mm, angle lidar °), ...].
    Retourne (pose, chi²) ou None si le système est singulier."""
    u = np.array(pose0, dtype=float)
    scale = np.array([[1.0 / sd_mm], [1.0 / sa_rad]])
    for _ in range(iters):
        A, b = np.zeros((3, 3)), np.zeros(3)
        for k, d_m, a_m in meas:
            d_e, a_e, H = ekf.measurement(u, k)
            r = np.array([(d_m - d_e) / sd_mm, math.radians(wrap_deg(a_m - a_e)) / sa_rad])
            Hs = H * scale
            A += Hs.T @ Hs
            b += Hs.T @ r
        try:
            du = np.linalg.solve(A + 1e-9 * np.eye(3), b)
        except np.linalg.LinAlgError:
            return None
        du[:2] = np.clip(du[:2], -300.0, 300.0)                 # pas borné : le départ peut être loin
        du[2] = float(np.clip(du[2], -0.3, 0.3))
        u = u + du
        if np.abs(du[:2]).max() < 0.05 and abs(du[2]) < 1e-5:
            break
    chi2 = 0.0
    for k, d_m, a_m in meas:
        d_e, a_e, _ = ekf.measurement(u, k)
        chi2 += ((d_m - d_e) / sd_mm) ** 2 + (math.radians(wrap_deg(a_m - a_e)) / sa_rad) ** 2
    return u, chi2


def triangulate(ekf: PoseEKF, clusters: List[Cluster], rp: Optional[RelocParams] = None) -> Optional[RelocResult]:
    """Retourne la meilleure pose du robot compatible avec les clusters, ou None."""
    rp = rp or RelocParams()
    p = ekf.p
    radius = p.pole_radius_mm
    cands = [c for c in clusters
             if c.n_points >= rp.min_points and rp.width_min_mm <= c.width_mm <= rp.width_max_mm]
    if len(cands) < 2 or len(p.poles_mm) < 2:
        return None

    zero = math.radians(p.lidar_zero_deg)
    ox, oy = p.lidar_offset_mm
    sd, sa = p.meas_dist_std_mm, math.radians(p.meas_angle_std_deg)
    xmin, ymin, xmax, ymax = p.field_mm
    m = p.field_margin_mm
    sols = []

    for i, j in itertools.combinations(range(len(p.poles_mm)), 2):
        qi, qj = p.poles_mm[i], p.poles_mm[j]
        for ca, cb in itertools.permutations(range(len(cands)), 2):     # ca -> poteau i, cb -> poteau j
            A, B = cands[ca], cands[cb]
            di, dj = A.dist_mm + radius, B.dist_mm + radius
            # Pré-filtre bon marché : quelle que soit la pose, la distance entre les centres des deux clusters
            # (loi des cosinus sur les mesures) doit valoir la distance connue entre les deux poteaux. Cela écarte la
            # plupart des mauvaises affectations avant l'affinage, qui est la partie coûteuse.
            dang = math.radians(wrap_deg(B.angle_deg - A.angle_deg))
            chord = math.sqrt(max(di * di + dj * dj - 2.0 * di * dj * math.cos(dang), 0.0))
            if abs(chord - math.hypot(qj[0] - qi[0], qj[1] - qi[1])) > rp.chord_tol_mm:
                continue
            meas = [(i, di, A.angle_deg), (j, dj, B.angle_deg)]
            for lx, ly in _start_points(qi, di, qj, dj, rp.tangent_tol_mm, rp.start_offset_min_mm):
                # cap robot = cap terrain du poteau - cap du poteau dans le repère robot ;
                # angle lidar = (zéro - cap robot) : voir PoseEKF.measurement
                th_a = wrap_pi(math.atan2(qi[1] - ly, qi[0] - lx) - (zero - math.radians(A.angle_deg)))
                th_b = wrap_pi(math.atan2(qj[1] - ly, qj[0] - lx) - (zero - math.radians(B.angle_deg)))
                th0 = wrap_pi(th_a + 0.5 * wrap_pi(th_b - th_a))        # moyenne circulaire des deux caps
                c, s = math.cos(th0), math.sin(th0)
                x0, y0 = lx - (c * ox - s * oy), ly - (s * ox + c * oy)  # du lidar au centre du robot

                refined = _refine(ekf, (x0, y0, th0), meas, sd, sa)
                if refined is None:
                    continue
                pose, chi2 = refined
                if chi2 > rp.max_chi2:
                    continue
                x, y, th = float(pose[0]), float(pose[1]), wrap_pi(float(pose[2]))
                if not (xmin - m <= x <= xmax + m and ymin - m <= y <= ymax + m):
                    continue

                n_in, resid, used = 0, 0.0, []
                pose_w = np.array([x, y, th])
                for k in range(len(p.poles_mm)):
                    d_exp, a_exp, _ = ekf.measurement(pose_w, k)
                    near = None
                    for ci, cl in enumerate(cands):
                        dd = cl.dist_mm + radius - d_exp
                        da = wrap_deg(cl.angle_deg - a_exp)
                        if abs(dd) <= rp.inlier_dist_mm and abs(da) <= rp.inlier_angle_deg:
                            score = (dd / sd) ** 2 + (math.radians(da) / sa) ** 2
                            if near is None or score < near[0]:
                                near = (score, ci)
                    if near is not None:
                        n_in += 1
                        resid += near[0]
                        used.append((k, near[1]))
                if n_in < rp.min_inliers:
                    continue
                sols.append(RelocResult(x, y, th, n_in, float(resid), tuple(used)))
    return _decide(ekf, sols, rp)


def _same(a: RelocResult, b: RelocResult, rp: RelocParams) -> bool:
    return (math.hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm) <= rp.merge_dist_mm
            and abs(math.degrees(wrap_pi(a.theta_rad - b.theta_rad))) <= rp.merge_angle_deg)


def _decide(ekf: PoseEKF, sols: List[RelocResult], rp: RelocParams) -> Optional[RelocResult]:
    """Fusionne les solutions identiques, garde celles qui expliquent le plus de poteaux, et tranche l'ambiguïté."""
    if not sols:
        return None
    top = max(s.n_inliers for s in sols)
    distinct: List[RelocResult] = []
    for s in sorted((s for s in sols if s.n_inliers == top), key=lambda s: s.residual):
        if not any(_same(s, d, rp) for d in distinct):
            distinct.append(s)                      # la meilleure (plus petit résidu) de chaque groupe
    if len(distinct) == 1:
        return distinct[0]
    if not ekf.ready:                               # aucune pose de référence : impossible de trancher
        return None
    px, py, _ = ekf.pose
    ranked = sorted(((math.hypot(s.x_mm - px, s.y_mm - py), k) for k, s in enumerate(distinct)))
    d0, k0 = ranked[0]
    # On ne tranche que si la pose du filtre est assez proche d'UNE solution ET beaucoup plus loin de toutes les
    # autres : si les solutions sont à peine plus écartées que l'erreur possible du filtre, le choix serait un
    # pile ou face. (Ex. robot près de l'axe de symétrie : deux solutions à 150 mm l'une de l'autre.)
    if d0 > rp.prior_max_dist_mm or any(d < rp.prior_sep_factor * rp.prior_max_dist_mm for d, _ in ranked[1:]):
        return None
    nearest = distinct[k0]
    return RelocResult(nearest.x_mm, nearest.y_mm, nearest.theta_rad, nearest.n_inliers, nearest.residual,
                       nearest.assignment, ambiguous=True)

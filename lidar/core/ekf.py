"""EKF de localisation : étapes 1 et 2 du plan (état, paramètres, prédiction par odométrie).

État : pose du robot dans le repère TERRAIN (origine sur le poteau (0, 0)).
    x, y en mm ; θ en rad, sens trigonométrique (anti-horaire), 0 = axe x du terrain.
Incertitude : matrice P (mm², rad²), mise à jour par le filtre.

L'odométrie (HolonomicBase.pose()) vit dans son PROPRE repère, qui démarre à (0, 0, 0) : le filtre
n'en utilise que les déplacements d'un échantillon au suivant, jamais la pose absolue.

Ce module fournit aussi, pour l'étape 3 (association) et l'étape 4 (correction) :
  - l'historique des poses/incertitudes (state_at) pour dater les corrections ;
  - le modèle de mesure d'un poteau (measurement) et sa jacobienne.
"""
import bisect
import math
from dataclasses import dataclass
from typing import NamedTuple, Optional

import numpy as np


def wrap_pi(a: float) -> float:
    """Ramène un angle (rad) dans [-π, π[."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_deg(a: float) -> float:
    """Ramène un angle (°) dans [-180, 180[ : un poteau vers 0°/360° ne donne pas un écart de 360°."""
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class EkfParams:
    """Valeurs de départ du plan (à régler à l'étape 5, pas des valeurs mesurées)."""

    # --- terrain ---
    poles_mm: tuple = ((3094.0, 72.0), (3094.0, 1928.0), (1725.0, 2120.0))   # centres des poteaux
    pole_radius_mm: float = 50.0                                   # poteaux de 100 mm de diamètre

    # --- montage du lidar (à mesurer sur le robot) ---
    lidar_offset_mm: tuple = (0.0, 0.0)   # position du lidar dans le repère robot (x vers la roue 3, y à 90°)
    lidar_zero_deg: float = 0.0           # angle lue par le lidar quand il vise l'axe x du robot
    lidar_latency_s: float = 0.0          # retard entre la mesure et son horodatage de réception

    # --- incertitude de départ (écarts-types) ---
    init_pos_std_mm: float = 20.0
    init_theta_std_deg: float = 1.0

    # --- Q : bruit de l'odométrie ---
    # Modèle "marche aléatoire" calé sur le plan : "2 % -> 1 m parcouru ajoute ~20 mm d'incertitude".
    # Variance ajoutée à chaque pas = err² x distance du pas x distance de référence, donc après
    # 1 m (= la référence) l'écart-type vaut err x 1 m = 20 mm.
    odo_trans_err: float = 0.02           # 2 % de la distance
    odo_rot_err: float = 0.01             # 1 % de l'angle
    odo_ref_dist_mm: float = 1000.0
    odo_ref_angle_rad: float = 1.0

    # --- R : bruit de mesure d'un poteau ---
    meas_dist_std_mm: float = 25.0
    meas_angle_std_deg: float = 0.4

    # --- garde-fou : déplacement maximal provoqué par une seule correction lidar ---
    # Une mesure peut passer le chi² avec une covariance large ; on borne donc aussi l'effet
    # réel sur la pose avant d'appliquer la correction.
    max_correction_pos_mm: float = 100.0
    max_correction_theta_deg: float = 6.0

    # --- historique ---
    history_s: float = 1.0                # au moins 1 tour de lidar (100 ms) ; 1 s laisse de la marge
    clamp_tol_s: float = 0.02             # au-delà, state_at signale un instant hors historique

    # --- perte et relocalisation (étape 6) ---
    lost_after_s: float = 1.0             # des poteaux vus mais jamais acceptés pendant ce temps : on se dit perdu
    reloc_pos_std_mm: float = 150.0       # incertitude de la pose retrouvée par triangulation
    reloc_theta_std_deg: float = 10.0
    safe_std_mm: float = 50.0             # au-delà, la pose n'est plus jugée fiable pour la stratégie
    field_mm: tuple = (0.0, 0.0, 3000.0, 2000.0)   # (xmin, ymin, xmax, ymax) : une pose hors du terrain est refusée
    field_margin_mm: float = 300.0
    field_filter_margin_mm: float = 200.0  # marge autour du terrain pour filtrer les impacts lidar


class StateAt(NamedTuple):
    x: np.ndarray       # (x mm, y mm, θ rad)
    P: np.ndarray       # 3x3
    clamped: bool       # True si l'instant demandé sort de l'historique (horloges décalées ?)


class PoseEKF:
    def __init__(self, params: Optional[EkfParams] = None):
        self.p = params or EkfParams()
        self.x = np.zeros(3)
        self.P = np.eye(3)
        self.ready = False
        self._odo_prev = None                     # dernier échantillon d'odométrie (x, y, θ, t)
        self._ht, self._hx, self._hP = [], [], []  # historique : instants, poses, covariances
        # _hC[i][j] = Cov(x_i, x_j), indispensable pour traiter une mesure retardée.
        self._hC = np.empty((0, 0, 3, 3), dtype=float)

    # ------------------------------------------------------------------ étape 1 : état
    def reset(self, x_mm, y_mm, theta_rad, t_s, pos_std_mm=None, theta_std_rad=None):
        """Initialise la pose (repère terrain) et son incertitude à l'instant t_s (epoch, s)."""
        sp = self.p.init_pos_std_mm if pos_std_mm is None else pos_std_mm
        st = math.radians(self.p.init_theta_std_deg) if theta_std_rad is None else theta_std_rad
        self.x = np.array([x_mm, y_mm, theta_rad], dtype=float)
        self.P = np.diag([sp * sp, sp * sp, st * st])
        self._odo_prev = None
        self._ht, self._hx, self._hP = [], [], []
        self._hC = np.empty((0, 0, 3, 3), dtype=float)
        self._record(t_s)
        self.ready = True

    @property
    def pose(self):
        """(x mm, y mm, θ rad dans [-π, π[)."""
        return float(self.x[0]), float(self.x[1]), wrap_pi(float(self.x[2]))

    @property
    def std(self):
        """(σx mm, σy mm, σθ °)."""
        return (math.sqrt(self.P[0, 0]), math.sqrt(self.P[1, 1]), math.degrees(math.sqrt(self.P[2, 2])))

    # ------------------------------------------------------------------ étape 2 : prédiction
    def predict_odometry(self, x_mm, y_mm, theta_rad, t_s):
        """Avance la pose avec un échantillon d'odométrie (pose dans le repère odométrie, t_s en epoch)."""
        prev = self._odo_prev
        if prev is None:                           # premier échantillon : simple référence
            self._odo_prev = (x_mm, y_mm, theta_rad, t_s)
            self._record(t_s)
            return
        px, py, pth, pt = prev
        if t_s <= pt:                              # pas de nouvelle lecture
            return
        self._odo_prev = (x_mm, y_mm, theta_rad, t_s)
        dth = theta_rad - pth
        th_mid = pth + dth / 2.0                   # même convention que HolonomicBase._odo_step (arc)
        c, s = math.cos(th_mid), math.sin(th_mid)
        dx, dy = x_mm - px, y_mm - py
        self.predict(c * dx + s * dy, -s * dx + c * dy, dth, t_s)   # repère odométrie -> repère robot

    def predict(self, dxl_mm, dyl_mm, dth_rad, t_s):
        """Applique un déplacement exprimé dans le repère ROBOT : il est converti dans le repère
        terrain avec le cap courant (c'est ce qui rend le filtre « étendu »), et l'incertitude grandit."""
        th = self.x[2]
        thm = th + dth_rad / 2.0
        c, s = math.cos(thm), math.sin(thm)
        self.x = np.array([
            self.x[0] + c * dxl_mm - s * dyl_mm,
            self.x[1] + s * dxl_mm + c * dyl_mm,
            th + dth_rad,
        ])
        # Une erreur de cap fait dévier le déplacement : l'effet dépend de la direction du mouvement.
        F = np.array([
            [1.0, 0.0, -s * dxl_mm - c * dyl_mm],
            [0.0, 1.0, c * dxl_mm - s * dyl_mm],
            [0.0, 0.0, 1.0],
        ])
        p = self.p
        var_t = p.odo_trans_err ** 2 * math.hypot(dxl_mm, dyl_mm) * p.odo_ref_dist_mm   # mm²
        var_r = p.odo_rot_err ** 2 * abs(dth_rad) * p.odo_ref_angle_rad                  # rad²
        G = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        Q = G @ np.diag([var_t, var_t, var_r]) @ G.T
        self.P = F @ self.P @ F.T + Q
        self._record(t_s, transition_F=F)

    # ------------------------------------------------------------------ historique
    def _record(self, t_s, transition_F=None):
        """Enregistre un état et propage les covariances croisées de l'historique.

        transition_F est la jacobienne de la transition depuis le dernier état enregistré.
        Sans transition, le nouvel échantillon représente le même état (première lecture odo).
        """
        if self._ht and t_s <= self._ht[-1]:       # même instant : remplace la dernière entrée
            self._hx[-1], self._hP[-1] = self.x.copy(), self.P.copy()
            self._hC[-1, -1] = self.P.copy()
            return
        if not self._ht:
            self._hC = self.P.reshape((1, 1, 3, 3)).copy()
        else:
            F = np.eye(3) if transition_F is None else transition_F
            n = len(self._ht)
            old = self._hC
            cross = old[:, -1] @ F.T              # Cov(x_i, x_new), vectorisé sur i
            expanded = np.empty((n + 1, n + 1, 3, 3), dtype=float)
            expanded[:n, :n] = old
            expanded[:n, n] = cross
            expanded[n, :n] = cross.transpose(0, 2, 1)
            expanded[n, n] = self.P
            self._hC = expanded
        self._ht.append(t_s)
        self._hx.append(self.x.copy())
        self._hP.append(self.P.copy())
        k = bisect.bisect_left(self._ht, t_s - self.p.history_s) - 1   # garde 1 entrée avant la coupure
        if k > 0:
            del self._ht[:k], self._hx[:k], self._hP[:k]
            self._hC = self._hC[k:, k:].copy()

    def state_at(self, t_s) -> Optional[StateAt]:
        """Pose et incertitude à l'instant t_s (interpolées). Hors historique : on prend l'entrée la
        plus proche et on met clamped=True si l'écart dépasse clamp_tol_s."""
        ts = self._ht
        if not ts:
            return None
        tol = self.p.clamp_tol_s
        if t_s <= ts[0]:
            return StateAt(self._hx[0].copy(), self._hP[0].copy(), t_s < ts[0] - tol)
        if t_s >= ts[-1]:
            return StateAt(self._hx[-1].copy(), self._hP[-1].copy(), t_s > ts[-1] + tol)
        i = bisect.bisect_right(ts, t_s)
        i0, i1 = i - 1, i
        w = (t_s - ts[i0]) / (ts[i1] - ts[i0])
        # Covariance de l'état interpolé, avec le terme croisé entre les deux échantillons.
        P0, P1 = self._hP[i0], self._hP[i1]
        P01 = self._hC[i0, i1]
        P = ((1 - w) ** 2 * P0 + w ** 2 * P1
             + w * (1 - w) * (P01 + P01.T))
        P = (P + P.T) * 0.5
        return StateAt((1 - w) * self._hx[i0] + w * self._hx[i1], P, False)

    # ------------------------------------------------------------------ modèle de mesure (étapes 3 et 4)
    def measurement(self, x, pole_index):
        """Mesure attendue d'un poteau pour la pose x = (x mm, y mm, θ rad).

        Retourne (distance mm du lidar au CENTRE du poteau, angle lidar en ° dans [0, 360[, H),
        H = jacobienne 2x3 de (distance mm, angle rad) par rapport à (x mm, y mm, θ rad).
        Angle lidar : sens horaire, alors que le repère robot est anti-horaire.
        """
        qx, qy = self.p.poles_mm[pole_index]
        ox, oy = self.p.lidar_offset_mm
        th = x[2]
        c, s = math.cos(th), math.sin(th)
        lx, ly = x[0] + c * ox - s * oy, x[1] + s * ox + c * oy       # position du lidar sur le terrain
        dlx, dly = -s * ox - c * oy, c * ox - s * oy                   # d(position lidar) / dθ
        dx, dy = qx - lx, qy - ly
        r2 = dx * dx + dy * dy
        r = math.sqrt(r2)
        beta = math.atan2(dy, dx) - th                                  # cap du poteau dans le repère robot
        a = (math.radians(self.p.lidar_zero_deg) - beta) % (2 * math.pi)
        H = np.zeros((2, 3))
        H[0] = [-dx / r, -dy / r, -(dx * dlx + dy * dly) / r]
        dbeta = np.array([dy / r2, -dx / r2, (dy * dlx - dx * dly) / r2 - 1.0])
        H[1] = -dbeta
        return r, math.degrees(a), H


    def correct_pole(self, pole_index: int, measured_dist_mm: float, measured_angle_deg: float,
                     gate_chi2: float = 9.21, t_s: Optional[float] = None):
        """Corrige une mesure de poteau, y compris lorsqu'elle est retardée.

        Les covariances croisées de l'historique donnent Cov(x_i, x_mesure) : la correction
        est propagée à chaque état avec son propre gain, puis toutes les covariances croisées
        sont mises à jour. Une mesure hors de la fenêtre historique est refusée.
        """
        n = len(self._ht)
        if n == 0:
            return False, float("inf")

        if t_s is None:
            x_ref, P_ref = self.x.copy(), self.P.copy()
            cross_ref = self._hC[:, -1].copy()
        else:
            st = self.state_at(t_s - self.p.lidar_latency_s)
            if st is None or st.clamped:
                return False, float("inf")
            x_ref, P_ref = st.x, st.P
            ts = self._ht
            if t_s - self.p.lidar_latency_s <= ts[0]:
                i0 = i1 = 0
                w = 0.0
            elif t_s - self.p.lidar_latency_s >= ts[-1]:
                i0 = i1 = n - 1
                w = 0.0
            else:
                i1 = bisect.bisect_right(ts, t_s - self.p.lidar_latency_s)
                i0 = i1 - 1
                w = ((t_s - self.p.lidar_latency_s - ts[i0]) / (ts[i1] - ts[i0]))
            cross_ref = (1.0 - w) * self._hC[:, i0] + w * self._hC[:, i1]

        expected_dist, expected_angle_deg, H = self.measurement(x_ref, pole_index)
        innovation = np.array([
            measured_dist_mm - expected_dist,
            math.radians(wrap_deg(measured_angle_deg - expected_angle_deg)),
        ], dtype=float)
        R = np.diag([self.p.meas_dist_std_mm ** 2, math.radians(self.p.meas_angle_std_deg) ** 2])
        S = H @ P_ref @ H.T + R
        try:
            S_inv = np.linalg.solve(S, np.eye(2))
        except np.linalg.LinAlgError:
            return False, float("inf")
        mahal = float(innovation.T @ S_inv @ innovation)
        if not math.isfinite(mahal) or mahal > gate_chi2:
            return False, mahal

        # Gain différent pour chaque instant : K_i = Cov(x_i, x_ref) H' S^-1.
        gains = cross_ref @ H.T @ S_inv             # (N, 3, 2)
        dx_now = gains[-1] @ innovation
        correction_pos_mm = math.hypot(float(dx_now[0]), float(dx_now[1]))
        correction_theta_deg = abs(math.degrees(float(dx_now[2])))
        if (correction_pos_mm > self.p.max_correction_pos_mm
                or correction_theta_deg > self.p.max_correction_theta_deg):
            return False, mahal

        # Mise à jour de tous les états historiques avec leur gain propre.
        dx_history = gains @ innovation
        self._hx = [x + dx for x, dx in zip(self._hx, dx_history)]

        # Mise à jour conditionnelle complète, vectorisée : P_ij+ = P_ij - K_i H Cov(x_ref, x_j).
        # Le tenseur contient toutes les covariances croisées; la symétrisation limite les erreurs d'arrondi.
        kh = gains @ H
        cov_ref_j = cross_ref.transpose(0, 2, 1)
        self._hC -= np.einsum("iab,jbc->ijac", kh, cov_ref_j, optimize=True)
        self._hC = 0.5 * (self._hC + self._hC.transpose(1, 0, 3, 2))
        self._hP = [self._hC[i, i].copy() for i in range(n)]

        self.x = self._hx[-1].copy()
        self.P = self._hP[-1].copy()
        return True, mahal

    # ------------------------------------------------------------------ étape 6 : relocalisation
    def relocalize(self, x_mm, y_mm, theta_rad, t_ref_s, pos_std_mm=None, theta_std_rad=None):
        """Remplace la pose par celle trouvée par triangulation, valable à l'instant t_ref_s.

        Le déplacement accumulé depuis t_ref_s (historique de l'odométrie) est rejoué à partir de la
        nouvelle pose pour revenir à « maintenant ». L'incertitude est remise à une grande valeur et
        l'historique est repris à zéro ; la référence de l'odométrie est conservée, donc la prédiction
        continue sans à-coup. Retourne la nouvelle pose courante (x mm, y mm, θ rad).
        """
        sp = self.p.reloc_pos_std_mm if pos_std_mm is None else pos_std_mm
        st = math.radians(self.p.reloc_theta_std_deg) if theta_std_rad is None else theta_std_rad
        now_t = self._ht[-1] if self._ht else t_ref_s
        new = np.array([x_mm, y_mm, theta_rad], dtype=float)
        ref = self.state_at(t_ref_s)
        if ref is not None:
            c, s = math.cos(ref.x[2]), math.sin(ref.x[2])
            dx, dy = self.x[0] - ref.x[0], self.x[1] - ref.x[1]
            dxl, dyl = c * dx + s * dy, -s * dx + c * dy             # déplacement depuis t_ref, repère robot
            dth = wrap_pi(float(self.x[2] - ref.x[2]))
            c2, s2 = math.cos(theta_rad), math.sin(theta_rad)
            new = np.array([x_mm + c2 * dxl - s2 * dyl, y_mm + s2 * dxl + c2 * dyl, theta_rad + dth])
        self.x = new
        self.x[2] = wrap_pi(float(self.x[2]))
        self.P = np.diag([sp * sp, sp * sp, st * st])
        self._ht, self._hx, self._hP = [], [], []
        self._hC = np.empty((0, 0, 3, 3), dtype=float)
        self._record(now_t)
        return self.pose

    def predicted_std(self, P, H):
        """Écarts-types attendus d'une mesure de poteau : (σ distance mm, σ angle °),
        = incertitude de la pose projetée sur la mesure + bruit de mesure R."""
        p = self.p
        R = np.diag([p.meas_dist_std_mm ** 2, math.radians(p.meas_angle_std_deg) ** 2])
        S = H @ P @ H.T + R
        return math.sqrt(S[0, 0]), math.degrees(math.sqrt(S[1, 1]))

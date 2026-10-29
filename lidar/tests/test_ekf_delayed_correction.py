"""Régressions pour les corrections lidar retardées et l'historique EKF.

À lancer depuis le dossier lidar : python -m pytest tests/test_ekf_delayed_correction.py -v
"""
import math

import numpy as np
import pytest

from core.ekf import EkfParams, PoseEKF


T0 = 1000.0


def make_moving_ekf():
    ekf = PoseEKF(EkfParams(history_s=1.0, max_correction_pos_mm=100.0,
                            max_correction_theta_deg=8.0))
    ekf.reset(1500.0, 1000.0, 0.2, T0, pos_std_mm=100.0, theta_std_rad=0.1)
    for i in range(1, 41):
        ekf.predict(5.0, 1.0, 0.002, T0 + i * 0.005)
    return ekf


def test_delayed_measurement_corrects_current_state_through_cross_covariance():
    ekf = make_moving_ekf()
    t_meas = T0 + 0.1
    state_before = ekf.state_at(t_meas)
    current_before = ekf.x.copy()
    expected_dist, expected_angle, _ = ekf.measurement(state_before.x, 0)
    ok, mahal = ekf.correct_pole(0, expected_dist + 8.0, expected_angle + 0.15, t_s=t_meas)

    assert ok, f"mesure synthétique rejetée, mahal={mahal}"
    assert not np.allclose(ekf.x, current_before)
    state_after = ekf.state_at(t_meas)
    assert np.trace(state_after.P) < np.trace(state_before.P)
    assert np.all(np.isfinite(ekf.x))
    assert np.all(np.isfinite(ekf.P))


def test_all_historical_covariances_remain_symmetric_positive_semidefinite():
    ekf = make_moving_ekf()
    t_meas = T0 + 0.1
    state = ekf.state_at(t_meas)
    dist, angle, _ = ekf.measurement(state.x, 1)
    ok, _ = ekf.correct_pole(1, dist + 5.0, angle - 0.1, t_s=t_meas)
    assert ok

    assert len(ekf._hC) == len(ekf._hP) == len(ekf._hx)
    for i, P in enumerate(ekf._hP):
        assert np.allclose(P, P.T, atol=1e-8)
        assert np.linalg.eigvalsh(P).min() >= -1e-7
        assert np.allclose(P, ekf._hC[i][i], atol=1e-8)
        for j in range(len(ekf._hC)):
            assert np.allclose(ekf._hC[i][j], ekf._hC[j][i].T, atol=1e-7)


def test_measurement_older_than_history_is_rejected():
    ekf = make_moving_ekf()
    before = ekf.x.copy()
    ok, mahal = ekf.correct_pole(0, 1000.0, 20.0, t_s=T0 - 10.0)
    assert not ok
    assert math.isinf(mahal)
    assert np.array_equal(ekf.x, before)


def test_history_cross_covariance_is_pruned_with_state_history():
    ekf = PoseEKF(EkfParams(history_s=0.2))
    ekf.reset(1000.0, 1000.0, 0.0, T0)
    for i in range(1, 101):
        ekf.predict(2.0, 0.0, 0.0, T0 + i * 0.005)
    n = len(ekf._ht)
    assert n < 60
    assert len(ekf._hC) == n
    assert all(len(row) == n for row in ekf._hC)

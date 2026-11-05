"""Benchmark diagnostic des coûts EKF liés à l'historique de pose.

Depuis le dossier lidar :
    python -m tests.benchmark_ekf_history --duration 5

Ce test est volontairement séparé du lidar et des moteurs : il mesure uniquement
le coût de predict()/ _record() à 200 Hz, avec les paramètres réels d'historique.
Il ne modifie aucun paramètre de l'EKF.
"""
import argparse
import math
import statistics
import time

from core.ekf import EkfParams, PoseEKF


def percentile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * q))]


def run(duration_s=5.0, hz=200.0, history_s=1.0):
    ekf = PoseEKF(EkfParams(history_s=history_s))
    t0 = time.time()
    ekf.reset(1500.0, 1000.0, 0.2, t0)
    samples = []
    deadline = time.perf_counter() + duration_s
    period = 1.0 / hz
    i = 0
    while time.perf_counter() < deadline:
        started = time.perf_counter()
        t = t0 + (i + 1) * period
        ekf.predict(0.5, 0.1, 0.0005, t)
        samples.append((time.perf_counter() - started) * 1000.0)
        i += 1

    print(f"EKF predict() / _record() — {hz:.0f} Hz demandé, historique {history_s:.2f} s")
    print(f"Échantillons traités : {len(samples)} en {duration_s:.1f} s")
    print(f"Historique final : {len(ekf._ht)} états, tenseur _hC : {ekf._hC.nbytes / 1024:.0f} Kio")
    print(f"Temps par prédiction : médiane {statistics.median(samples):.3f} ms | "
          f"p95 {percentile(samples, .95):.3f} ms | max {max(samples):.3f} ms")
    print(f"Budget à {hz:.0f} Hz : {1000.0 / hz:.2f} ms par prédiction")
    print(f"Charge CPU théorique de predict : "
          f"{statistics.mean(samples) * hz / 10.0:.1f} % d'un cœur")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=200.0)
    parser.add_argument("--history", type=float, default=1.0)
    args = parser.parse_args()
    run(args.duration, args.hz, args.history)

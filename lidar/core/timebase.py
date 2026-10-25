"""Ramène les horloges des différents capteurs sur l'heure système (epoch).

Le lidar et l'odométrie datent leurs mesures chacun avec leur horloge. Pour corriger la pose
"au bon moment" (étape 4 du plan), il faut que tout soit sur la même échelle de temps.
"""
import time


class ClockAligner:
    """Convertit les instants d'une source (lidar, odométrie...) en heure système (epoch).

    - Si la source est déjà en heure epoch (écart < 1 h avec l'heure système au premier
      échantillon), rien ne change.
    - Sinon (time.monotonic(), perf_counter()...), on mesure l'écart au premier échantillon et on
      le retranche ensuite : les durées restent exactes, seule l'origine est recalée, à quelques ms
      près (le délai entre la mesure et la réception du premier échantillon).

    unit : "s" (secondes, float) ou "ns" (nanosecondes, int) pour les instants de la source.
    """

    def __init__(self, name: str, unit: str = "s"):
        """Initialise l’objet et ses ressources à partir des paramètres fournis."""
        if unit not in ("s", "ns"):
            raise ValueError("unit doit valoir 's' ou 'ns'")
        self.name = name
        self._unit_ns = 1 if unit == "ns" else 1_000_000_000
        self.offset_ns = None        # décalage source -> epoch, fixé au premier échantillon
        self.recalibrated = False    # True si la source n'était pas en heure epoch

    def epoch_ns(self, t) -> int:
        """Convertit l’horodatage source en nanosecondes depuis l’époque Unix."""
        t_ns = int(round(t * self._unit_ns))
        if self.offset_ns is None:
            off = time.time_ns() - t_ns
            self.recalibrated = abs(off) > 3600 * 10**9
            self.offset_ns = off if self.recalibrated else 0
        return t_ns + self.offset_ns

    def epoch_s(self, t) -> float:
        """Convertit l’horodatage source en secondes depuis l’époque Unix."""
        return self.epoch_ns(t) / 1e9

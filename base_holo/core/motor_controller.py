"""
Module de contrôle des moteurs MKS SERVO via CAN bus.
Fournit une classe Motor pour gérer individuellement chaque moteur.
"""

import math
import time
import can
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Tuple
from dataclasses import dataclass
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction


@dataclass
class MotorState:
    """État actuel d'un moteur."""
    position: int = 0  # Position en pulses
    speed: int = 0  # Vitesse actuelle en RPM
    target_speed: int = 0  # Vitesse cible en RPM
    target_direction: int = 0  # Direction cible: 0 = CCW, 1 = CW
    direction: int = 0  # Direction actuelle: 0 = CCW, 1 = CW
    acceleration: int = 100  # Accélération par défaut
    is_running: bool = False
    last_update: float = 0.0
    control_state: int = 1  # 1: idle, 2: speed change, 3: direction change


class Motor:
    """
    Classe de contrôle d'un moteur MKS SERVO via CAN bus.
    
    Gère tous les aspects du contrôle moteur :
    - Mouvements relatifs et en vitesse
    - Lecture de position et vitesse
    - Calibration et reset
    - Conversion accélération linéaire ↔ paramètre MKS
    """
    
    # Constantes pour conversion accélération
    TICKS_PER_REVOLUTION = 3200  # 16 subdivisions × 200 pas/tour
    # Bit de direction d'une roue à vitesse/distance POSITIVE (même convention que move_distance)
    POSITIVE_DIRECTION = 1
    
    def __init__(self, bus: can.Bus, can_id: int, mks_servo: Optional[MksServo] = None, 
                 wheel_diameter_mm: float = 55.8,
                 steps_per_revolution: int = 200,
                 subdivisions: int = 16,
                 gear_ratio: float = 15.0/48.0,
                 notifier: Optional[can.Notifier] = None):
        """
        Initialise un moteur.
        
        Args:
            bus: Interface CAN initialisée
            can_id: ID CAN du moteur (1, 2, 3, etc.)
            mks_servo: Instance MksServo optionnelle (pour fonctions avancées)
            wheel_diameter_mm: Diamètre de la roue en mm (par défaut 60mm)
            steps_per_revolution: Nombre de pas par tour moteur (par défaut 200)
            subdivisions: Nombre de subdivisions (par défaut 16)
            gear_ratio: Rapport de réduction (par défaut 1.0, >1 si réducteur)
        """
        self.bus = bus
        self.can_id = can_id
        self.servo = mks_servo
        self.state = MotorState()
        self._rx = None
        if notifier is not None:
            self._rx = can.BufferedReader()
            notifier.add_listener(self._rx)
        # Paramètres mécaniques pour conversions distance/pulses
        self.wheel_diameter_mm = wheel_diameter_mm
        self.steps_per_revolution = steps_per_revolution
        self.subdivisions = subdivisions
        self.gear_ratio = gear_ratio
        
        # Calcul des constantes de conversion
        self._update_conversion_constants()
        
        self.position_sample: Optional[Tuple[int, float]] = None   # (pulses, perf_counter)
        self._io_lock = threading.RLock()   # un seul échange requête/réponse à la fois

    def _query(self, opcode: int, resp_len: int, timeout: float) -> Optional[can.Message]:
        """Requête de lecture au format du manuel ([code, CRC], DLC 2) et attente de la réponse."""
        with self._io_lock:
            return self._query_locked(opcode, resp_len, timeout)

    def _query_locked(self, opcode: int, resp_len: int, timeout: float) -> Optional[can.Message]:
        if self._rx is not None:
            while self._rx.get_message(0) is not None:   # purge des trames anciennes
                pass
        crc = (self.can_id + opcode) & 0xFF
        self.bus.send(can.Message(arbitration_id=self.can_id, data=[opcode, crc], is_extended_id=False))
        deadline = time.perf_counter() + timeout
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            msg = self._rx.get_message(remaining) if self._rx is not None else self.bus.recv(remaining)
            if msg is None:
                return None
            if msg.arbitration_id != self.can_id or len(msg.data) != resp_len or msg.data[0] != opcode:
                continue
            if msg.data[-1] != (self.can_id + sum(msg.data[:-1])) & 0xFF:
                continue
            return msg

    def read_speed(self, timeout: float = 0.2) -> Optional[int]:
        msg = self._query(0x32, 4, timeout)
        if msg is None:
            return None
        speed = int.from_bytes(msg.data[1:3], byteorder="big", signed=True)
        self.state.speed = speed
        self.state.last_update = time.time()
        return speed

    def read_position(self, timeout: float = 0.1) -> Optional[int]:
        msg = self._query(0x31, 8, timeout)
        if msg is None:
            return None
        raw = int.from_bytes(msg.data[1:7], byteorder="big", signed=True)
        position = round(-raw * (self.steps_per_revolution * self.subdivisions) / 0x4000)
        self.state.position = position
        self.state.last_update = time.time()
        self.position_sample = (position, msg.timestamp if msg.timestamp else time.perf_counter())
        return position
    
    # =============================
    #   CONVERSION ACCÉLÉRATION
    # =============================
    
    @staticmethod
    def rpm_s_to_acc(rpm_per_s: float) -> int:
        """acc = 256 - 20000/a (manuel p.69). 0 = pas de rampe, jamais renvoyé pour une vraie rampe."""
        if rpm_per_s <= 0:
            return 0
        n = round(20000 / rpm_per_s)          # n = 256 - acc
        return 256 - max(1, min(255, n))      # [1, 255] : on plafonne à la rampe la plus lente

    @staticmethod
    def linear_to_acc(accel_pulses_s2: float) -> int:
        return Motor.rpm_s_to_acc(accel_pulses_s2 * 60 / Motor.TICKS_PER_REVOLUTION)
    
    @staticmethod
    def acc_to_linear(acc_param: int) -> float:
        """
        Convertit un paramètre MKS en accélération linéaire.
        
        Args:
            acc_param: Paramètre MKS (0-255)
            
        Returns:
            Accélération en pulses/s²
        """
        if acc_param >= 256:
            return float('inf')
        
        rpm_per_s = 20000 / (256 - acc_param)
        pulses_per_s2 = rpm_per_s * Motor.TICKS_PER_REVOLUTION / 60
        
        return pulses_per_s2
    
    # =============================
    #   CONVERSION DISTANCE/PULSES
    # =============================
    
    def _update_conversion_constants(self) -> None:
        """
        Met à jour les constantes de conversion distance/pulses.
        À appeler après modification des paramètres mécaniques.
        """
        import math
        
        # Périmètre de la roue en mm
        self.wheel_perimeter_mm = math.pi * self.wheel_diameter_mm
        
        # Nombre de pulses par tour de roue (en tenant compte du rapport de réduction)
        self.pulses_per_wheel_revolution = (
            self.steps_per_revolution * self.subdivisions * self.gear_ratio
        )
        
        # Conversion mm -> pulses
        self.pulses_per_mm = self.pulses_per_wheel_revolution / self.wheel_perimeter_mm
        
        # Conversion pulses -> mm
        self.mm_per_pulse = self.wheel_perimeter_mm / self.pulses_per_wheel_revolution
    
    def distance_to_pulses(self, distance_mm: float) -> int:
        """
        Convertit une distance linéaire en nombre de pulses.
        
        Args:
            distance_mm: Distance en millimètres
            
        Returns:
            Nombre de pulses correspondant
        """
        return int(distance_mm * self.pulses_per_mm)
    
    def pulses_to_distance(self, pulses: int) -> float:
        """
        Convertit un nombre de pulses en distance linéaire.
        
        Args:
            pulses: Nombre de pulses
            
        Returns:
            Distance en millimètres
        """
        return pulses * self.mm_per_pulse
    
    def speed_mm_s_to_rpm(self, speed_mm_s: float) -> int:
        """
        Convertit une vitesse linéaire (mm/s) en vitesse de rotation (RPM).
        
        Args:
            speed_mm_s: Vitesse linéaire en mm/s
            
        Returns:
            Vitesse en RPM
        """
        # Tours/seconde = (mm/s) / (mm/tour)
        revolutions_per_s = speed_mm_s / self.wheel_perimeter_mm
        
        # RPM = tours/s * 60 * rapport_réduction
        rpm = abs(revolutions_per_s) * 60 * self.gear_ratio
        
        return int(min(rpm, 3000))  # Limité à 3000 RPM max
    
    def rpm_to_speed_mm_s(self, rpm: int) -> float:
        """
        Convertit une vitesse de rotation (RPM) en vitesse linéaire (mm/s).
        
        Args:
            rpm: Vitesse en RPM
            
        Returns:
            Vitesse linéaire en mm/s
        """
        # Tours/seconde = RPM / 60 / rapport_réduction
        revolutions_per_s = rpm / 60 / self.gear_ratio
        
        # mm/s = tours/s * mm/tour
        speed_mm_s = revolutions_per_s * self.wheel_perimeter_mm
        
        return speed_mm_s
    
    # =============================
    #   MOUVEMENTS POSITIONNELS
    # =============================
    
    def move_distance(
        self,
        distance_mm: float,
        speed_mm_s: Optional[float] = None,
        speed_rpm: Optional[int] = None,
        acceleration: int = 100,
        direction: Optional[int] = None,
        acceleration_mks: Optional[int] = None
    ) -> None:
        """
        Déplace la roue d'une distance linéaire précise.
        
        Cette méthode convertit automatiquement la distance en pulses en tenant
        compte du diamètre de roue, du rapport de réduction et des subdivisions.
        
        Args:
            distance_mm: Distance à parcourir en millimètres (peut être négative)
            speed_mm_s: Vitesse linéaire en mm/s (prioritaire sur speed_rpm)
            speed_rpm: Vitesse en RPM (utilisée si speed_mm_s n'est pas fourni)
            acceleration: Accélération MKS (0-255) ou en pulses/s² si > 255
            direction: 0=CCW, 1=CW. Si None, déduit du signe de distance_mm
        
        Raises:
            ValueError: Si aucune vitesse n'est spécifiée ou si les valeurs sont invalides
            
        Example:
            >>> # Avancer de 100mm à 50mm/s
            >>> motor.move_distance(distance_mm=100, speed_mm_s=50)
            
            >>> # Reculer de 50mm à 100 RPM
            >>> motor.move_distance(distance_mm=-50, speed_rpm=100)
        """
        # Détermination de la vitesse
        if speed_mm_s is not None:
            rpm = self.speed_mm_s_to_rpm(speed_mm_s)
        elif speed_rpm is not None:
            rpm = speed_rpm
        else:
            raise ValueError("Vous devez spécifier soit speed_mm_s soit speed_rpm")
        
        # Conversion distance -> pulses
        pulses = self.distance_to_pulses(distance_mm)

        
        # Gestion de la direction
        if direction is None:
            direction = 0 if distance_mm < 0 else 1
        
        # Appel de la méthode move_relative existante
        self.move_relative(
            pulses=abs(pulses),
            speed=rpm,
            acceleration=acceleration,
            direction=direction,
            acceleration_mks=acceleration_mks
        )
        
        print(f"🎯 Déplacement: {distance_mm:.1f}mm ({pulses} pulses) à {rpm} RPM")
    
    # =============================
    #   MOUVEMENTS POSITIONNELS (PULSES)
    # =============================
    
    def move_relative(
        self, 
        pulses: int, 
        speed: int, 
        acceleration: int = 100,
        direction: Optional[int] = None,
        acceleration_mks: Optional[int] = None
    ) -> None:
        """
        Effectue un mouvement relatif.
        
        Args:
            pulses: Nombre de pulses à parcourir (positif)
            speed: Vitesse en RPM (0-3000)
            acceleration: Accélération MKS (0-255) ou en pulses/s² si > 255
            direction: 0=CCW, 1=CW. Si None, déduit du signe de pulses
        """
        # Gestion du sens
        if direction is None:
            direction = 1 if pulses < 0 else 0
            pulses = abs(pulses)
        
        # Conversion de l'accélération
        if acceleration_mks is not None:
            # Valeur MKS calculée en amont : aucune reconversion
            Mks_acceleration = int(acceleration_mks)

        elif acceleration > 255:
            # Accélération exprimée en pulses/s²
            Mks_acceleration = self.linear_to_acc(acceleration)

        else:
            # Accélération déjà exprimée dans l'unité MKS
            Mks_acceleration = int(acceleration)

        print(f"⚡ Accélération MKS : {Mks_acceleration}")
        # Validation
        if not (0 <= speed <= 3000):
            raise ValueError(f"Speed must be 0-3000 RPM, got {speed}")
        if not (0 <= Mks_acceleration <= 255):
            raise ValueError(f"Acceleration must be 0-255, got {Mks_acceleration}")
        if not (0 <= pulses <= 0xFFFFFF):
            raise ValueError(f"Pulses must be 0-{0xFFFFFF}, got {pulses}")
        
        # Construction du message
        code = 0xFD
        speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
        speed_low = speed & 0xFF
        pulse_bytes = [(pulses >> shift) & 0xFF for shift in (16, 8, 0)]
        
        data = [code, speed_high, speed_low, Mks_acceleration] + pulse_bytes
        crc = (int(self.can_id) + int(sum(data))) & 0xFF
        data.append(crc)
        
        data=[int(round(x)) for x in data]

        msg = can.Message(
            arbitration_id=self.can_id, 
            data=data, 
            is_extended_id=False
        )
        msg.dlc = 8
        
        try:
            self.bus.send(msg)
            self.state.target_speed = speed
            self.state.direction = direction
        except can.CanError as e:
            raise RuntimeError(f"CAN error during move_relative: {e}")
    
    def stop_relative(self) -> None:
        """
        Arrête un mouvement relatif en cours.
        Envoie une commande move_relative avec vitesse=0 et pulses=0.
        """
        code = 0xFD
        speed = 0
        acceleration = 255  # Arrêt rapide
        pulses = 0
        direction = 0
        
        # Construction du message (même format que move_relative)
        speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
        speed_low = speed & 0xFF
        pulse_bytes = [(pulses >> shift) & 0xFF for shift in (16, 8, 0)]
        
        data = [code, speed_high, speed_low, acceleration] + pulse_bytes
        crc = (int(self.can_id) + int(sum(data))) & 0xFF
        data.append(crc)
        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )
        msg.dlc = 8
        
        try:
            self.bus.send(msg)
        except can.CanError as e:
            raise RuntimeError(f"CAN error during stop_relative: {e}")
    # =============================
    #   MOUVEMENTS EN VITESSE
    # =============================
    
    def move_velocity(
        self, 
        speed: int, 
        acceleration: int = 100,
        direction: Optional[int] = None
    ) -> None:
        """
        Déplace le moteur à vitesse constante.
        
        Args:
            speed: Vitesse en RPM (0-3000), peut être négative
            acceleration: Accélération MKS (0-255) ou en pulses/s² si > 255
            direction: 0=CCW, 1=CW. Si None, déduit du signe de speed
        """
        # Gestion du sens
        if direction is None:
            direction = 1 if speed < 0 else 0
            speed = abs(speed)
        
        # Conversion accélération si nécessaire
        if acceleration > 255:
            acceleration = self.linear_to_acc(acceleration)
        
        # Validation
        if not (0 <= speed <= 3000):
            raise ValueError(f"Speed must be 0-3000 RPM, got {speed}")
        if not (0 <= acceleration <= 255):
            raise ValueError(f"Acceleration must be 0-255, got {acceleration}")
        
        # Construction du message
        code = 0xF6
        speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
        speed_low = speed & 0xFF
        
        data = [code, speed_high, speed_low, acceleration]
        crc = (int(self.can_id) + int(sum(data))) & 0xFF
        data.append(crc)
        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )
        msg.dlc = 5
        
        try:
            self.bus.send(msg)
            #self.state.target_speed = speed
            #self.state.direction = direction
        except can.CanError as e:
            raise RuntimeError(f"CAN error during move_velocity: {e}")
    
    # =============================
    #   ARRÊT
    # =============================
    
    def stop(self, acceleration: int = 150, soft: bool = True) -> None:
        """
        Arrête le moteur.
        
        Args:
            acceleration: Accélération de freinage (0-255)
            soft: Si True, arrêt progressif, sinon arrêt brutal
        """
        if soft:
            self.move_velocity(0, acceleration, direction=0)
        else:
            # Arrêt brutal (acc=255)
            self.move_velocity(0, 255, direction=0)
        
        self.state.target_speed = 0
    
    # =============================
    #   LECTURE D'ÉTAT
    # =============================
  
    def is_running(self) -> bool:
        """
        Vérifie si le moteur est en mouvement.
        
        Utilise l'instance MksServo si disponible.
        
        Returns:
            True si le moteur tourne, False sinon
        """ 
        pos1=self.read_position(timeout=0.05)
        time.sleep(0.002)
        pos2=self.read_position(timeout=0.05)        
        return pos1!=pos2
        
        
        # Fallback : lire la vitesse
        speed = self.read_speed(timeout=0.05)
        return speed is not None and abs(speed) > 0

        pos=self.read_position()
    
    # =============================
    #   CALIBRATION & RESET
    # =============================
    
    def reset_zero(self) -> None:
        """Réinitialise la position du moteur à zéro."""
        opcode = 0x92
        msg_data = [opcode]
        crc = (int(self.can_id) + int(sum(msg_data))) & 0xFF

        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=msg_data + [crc],
            is_extended_id=False
        )
        msg.dlc = 2
        
        self.bus.send(msg)
        self.state.position = 0
    
    def calibrate(self) -> None:
        """Lance la calibration du moteur."""
        opcode = 0x80
        msg_data = [opcode, 0x00]
        crc = (int(self.can_id) + int(sum(msg_data))) & 0xFF

        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=msg_data + [crc],
            is_extended_id=False
        )
        msg.dlc = 3
        
        self.bus.send(msg)
    
    def set_work_mode(self, mode: WorkMode) -> bool:
        """
        Change le mode de fonctionnement du moteur.
        
        Args:
            mode: Mode de travail (WorkMode.SrvFoc, etc.)
            
        Returns:
            True si succès, False sinon
        """
        if self.servo:
            return self.servo.set_work_mode(mode)
        return False
    
    def set_subdivisions(self, subdivisions: int) -> bool:
        """
        Configure les subdivisions du moteur.
        
        Args:
            subdivisions: Nombre de subdivisions (8, 16, 32, etc.)
            
        Returns:
            True si succès, False sinon
        """
        if self.servo:
            return self.servo.set_subdivisions(subdivisions)
        return False
    
    # =============================
    #   UTILITAIRES
    # =============================
    
    def wait_until_stopped(self, poll_interval: float = 0.01, timeout: float = 30.0) -> bool:
        """
        Attend que le moteur s'arrête complètement.
        
        Args:
            poll_interval: Intervalle de vérification (secondes)
            timeout: Délai maximal d'attente (secondes)
            
        Returns:
            True si le moteur s'est arrêté, False si timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            if not self.is_running():
                return True
            time.sleep(poll_interval)
        
        return False
    
    def __repr__(self) -> str:
        return (
            f"Motor(id={self.can_id}, pos={self.state.position}, speed={self.state.speed} RPM, "
            f"target={self.state.target_speed} RPM, dir={'CW' if self.state.direction else 'CCW'})"
        )


# =============================
#   CLASSE DE GESTION MULTI-MOTEURS
# =============================

class MotorGroup:
    """
    Groupe de moteurs d'une base holonome : consignes synchronisées, lecture de position
    en continu (poller) et watchdog de sécurité.

    Convention de signe UNIQUE pour les deux modes (distance et vitesse) :
    une vitesse ou une distance de roue POSITIVE => bit de direction Motor.POSITIVE_DIRECTION
    (comme Motor.move_distance). Un même Robot (angle, wheel_angles) sert donc aux deux modes.
    """

    def __init__(self, motors: list[Motor], robot=None):
        """
        Args:
            motors: moteurs, dans le même ordre que robot.wheel_angles
            robot: instance de holo_base.Robot (obligatoire pour move_direction / set_velocity)
        """
        self.motors = motors
        self.robot = robot
        self._last_sent = [None] * len(motors)     # (rpm, direction, acc) : évite les trames inutiles
        # watchdog
        self._wd_timeout = None
        self._wd_thread = None
        self._wd_stop = threading.Event()
        self._last_cmd = time.monotonic()
        self._moving = False
        # poller
        self._poll_thread = None
        self._poll_stop = threading.Event()
        self.on_round = None   # callback(samples) appelé après chaque tour de lecture des positions

    # =============================
    #   OUTILS INTERNES
    # =============================

    def _require_robot(self):
        if self.robot is None:
            raise RuntimeError("MotorGroup a besoin d'un Robot : MotorGroup(motors, robot)")

    @staticmethod
    def _wheel_rpms(motors, wheel_v_mm_s):
        """RPM entiers (arrondi au plus proche, plafonné 3000) ; 0 pour une roue immobile."""
        rpms = []
        for m, v in zip(motors, wheel_v_mm_s):
            rpm = round(abs(v) * m.pulses_per_mm * 60 / m.TICKS_PER_REVOLUTION)
            rpms.append(0 if rpm == 0 and abs(v) < 1e-9 else max(1, min(3000, rpm)))
        return rpms

    @staticmethod
    def plan_ramps(rpms, ramp_acc=220, fastest_boost=0):
        """
        Paramètre acc MKS (0-255) par roue pour que toutes les rampes aient la même durée
        (manuel p.69 : durée = RPM x (256-acc) x 50 µs). Référence = roue la plus rapide.
        Une roue très lente est plafonnée à acc=1 (rampe la plus douce possible).
        """
        if not 1 <= ramp_acc <= 255:
            raise ValueError(f"ramp_acc doit être dans [1, 255], reçu {ramp_acc}")
        active = [r for r in rpms if r > 0]
        if not active:
            return [ramp_acc] * len(rpms)
        max_rpm = max(active)
        ramp_steps = max_rpm * (256 - ramp_acc)
        accs = []
        for r in rpms:
            if r == 0:
                accs.append(ramp_acc)
                continue
            n = max(1, min(255, round(ramp_steps / r)))
            acc = 256 - n
            if r == max_rpm:
                acc = min(255, acc + fastest_boost)
            accs.append(acc)
        return accs

    def _wd_touch(self, moving: bool):
        self._last_cmd = time.monotonic()
        self._moving = moving

    # =============================
    #   MODE POSITION : DISTANCE DANS UNE DIRECTION
    # =============================

    def move_direction(self, distance_mm: float, direction_angle: float, speed_mm_s: float = 50,
                       dtheta: float = 0.0, ramp_acc: int = 220, fastest_boost: int = 0,
                       wait: bool = True, verbose: bool = True) -> bool:
        """
        Déplace le robot de `distance_mm` dans la direction `direction_angle` (repère global,
        rad) à `speed_mm_s`, avec une éventuelle rotation `dtheta`. Chaque roue reçoit une
        distance, un RPM entier proportionnel et une rampe de même durée (vitesses
        proportionnelles à tout instant).

        Returns:
            True si tous les moteurs se sont arrêtés (ou wait=False), False sur timeout.
        """
        from .holo_base import compute_wheel_distances_direction
        self._require_robot()
        if speed_mm_s <= 0:
            raise ValueError("speed_mm_s doit être strictement positif")
        if not 1 <= ramp_acc <= 255:
            raise ValueError(f"ramp_acc doit être dans [1, 255], reçu {ramp_acc}")

        wheel_d = compute_wheel_distances_direction(
            self.robot, distance=distance_mm, direction_angle=direction_angle, dtheta=dtheta)
        pulses = [m.distance_to_pulses(d) for m, d in zip(self.motors, wheel_d)]
        if verbose:
            print(f"🎯 move_direction : {distance_mm:.0f} mm à {math.degrees(direction_angle):.0f}°"
                  + (f" + rotation {math.degrees(dtheta):.0f}°" if dtheta else ""))
        active = [i for i, p in enumerate(pulses) if p != 0]
        if not active:
            return True
        max_p = max(abs(pulses[i]) for i in active)

        rpms = [0] * len(self.motors)
        for i in active:
            v = speed_mm_s * abs(pulses[i]) / max_p
            rpms[i] = max(1, min(3000, round(v * self.motors[i].pulses_per_mm
                                             * 60 / self.motors[i].TICKS_PER_REVOLUTION)))
        accs = self.plan_ramps(rpms, ramp_acc, fastest_boost)
        if verbose:
            for i in active:
                print(f"   Roue {i + 1} : {wheel_d[i]:8.2f} mm ({pulses[i]} pulses) | "
                      f"{rpms[i]:4d} RPM | acc={accs[i]:3d}")

        self._wd_touch(False)               # mode position : le watchdog ne s'applique pas
        for i in active:
            self.motors[i].move_distance(distance_mm=wheel_d[i], speed_rpm=rpms[i],
                                         acceleration_mks=accs[i])
        self._last_sent = [None] * len(self.motors)
        if not wait:
            return True

        time.sleep(0.1)                     # laisser démarrer
        est = max(abs(pulses[i]) / (rpms[i] * self.motors[i].TICKS_PER_REVOLUTION / 60)
                  + rpms[i] * (256 - accs[i]) * 50e-6 for i in active)
        deadline = time.time() + est + 5.0
        while time.time() < deadline:
            if all(not self.motors[i].is_running() for i in active):
                return True
            time.sleep(0.01)
        if verbose:
            print("   ⚠️ Timeout")
        return False

    def rotate(self, angle_rad: float, omega_rad_s: float = 1.0, ramp_acc: int = 220,
               wait: bool = True, verbose: bool = True) -> bool:
        """
        Fait tourner le robot sur lui-même de `angle_rad` (+ = anti-horaire, - = horaire),
        à la vitesse angulaire `omega_rad_s` (valeur absolue). Les 3 roues parcourent la même
        distance angle*R : même RPM, même rampe. Mode position (s'arrête seul).

        Returns:
            True si terminé (ou wait=False), False sur timeout.
        """
        self._require_robot()
        if omega_rad_s <= 0:
            raise ValueError("omega_rad_s doit être strictement positif")
        if angle_rad == 0:
            return True
        speed_mm_s = omega_rad_s * self.robot.radius       # vitesse linéaire de chaque roue
        if verbose:
            print(f"🔄 rotate : {math.degrees(angle_rad):.1f}° à {math.degrees(omega_rad_s):.0f}°/s")
        return self.move_direction(0.0, 0.0, speed_mm_s, dtheta=angle_rad,
                                   ramp_acc=ramp_acc, wait=wait, verbose=verbose)

    # =============================
    #   MODE VITESSE
    # =============================

    def set_wheel_speeds_mm_s(self, wheel_v_mm_s: list[float], ramp_acc: int = 220,
                              fastest_boost: int = 0) -> list[tuple]:
        """
        Applique des vitesses de roue (mm/s, signées). RPM entiers, rampes de même durée.
        N'envoie une trame que si la consigne d'une roue a changé. Réarme le watchdog.
        """
        if len(wheel_v_mm_s) != len(self.motors):
            raise ValueError(f"{len(wheel_v_mm_s)} vitesses pour {len(self.motors)} moteurs")
        rpms = self._wheel_rpms(self.motors, wheel_v_mm_s)
        accs = self.plan_ramps(rpms, ramp_acc, fastest_boost)
        cmds = []
        for i, (m, v, rpm, acc) in enumerate(zip(self.motors, wheel_v_mm_s, rpms, accs)):
            pos = Motor.POSITIVE_DIRECTION
            direction = pos if v >= 0 else 1 - pos
            cmd = (rpm, direction if rpm else 0, acc)
            cmds.append(cmd)
            if cmd != self._last_sent[i]:
                m.move_velocity(cmd[0], cmd[2], direction=cmd[1])
                self._last_sent[i] = cmd
        self._wd_touch(any(c[0] for c in cmds))
        return cmds

    def set_velocity(self, vx: float, vy: float, omega: float = 0.0, ramp_acc: int = 220,
                     fastest_boost: int = 0) -> list[tuple]:
        """Vitesse du robot dans le repère global : vx, vy en mm/s, omega en rad/s (+ = anti-horaire)."""
        from .holo_base import compute_wheel_distances_global
        self._require_robot()
        # "distance par seconde" = vitesse de roue
        wheel_v = compute_wheel_distances_global(self.robot, vx, vy, omega)
        return self.set_wheel_speeds_mm_s(wheel_v, ramp_acc, fastest_boost)

    def move_velocity_direction(self, direction_angle: float, speed_mm_s: float, omega: float = 0.0,
                                ramp_acc: int = 220, fastest_boost: int = 0) -> list[tuple]:
        """
        Roule en continu à `speed_mm_s` dans la direction `direction_angle` (repère global).
        Ne s'arrête pas toute seule : rappeler à chaque cycle de commande, finir par stop_all(),
        et activer start_watchdog() pour freiner si les appels cessent.
        """
        return self.set_velocity(speed_mm_s * math.cos(direction_angle),
                                 speed_mm_s * math.sin(direction_angle),
                                 omega, ramp_acc, fastest_boost)

    # =============================
    #   ARRÊT
    # =============================

    def stop_all(self, acceleration: int = 255, emergency: bool = False) -> None:
        """
        Freine tous les moteurs (annule aussi un mouvement relatif en cours).
        emergency=True : en plus, envoie l'arrêt d'urgence MksServo si disponible.
        """
        self._wd_touch(False)
        for m in self.motors:
            m.stop_relative()
        for m in self.motors:
            m.move_velocity(0, acceleration, direction=0)
            if emergency and m.servo is not None:
                m.servo.emergency_stop_motor()
        self._last_sent = [None] * len(self.motors)

    # =============================
    #   WATCHDOG
    # =============================

    def start_watchdog(self, timeout_s: float = 0.3) -> None:
        """
        Les MKS gardent leur dernière consigne de vitesse (pas de timeout interne). Si aucune
        consigne de vitesse n'arrive pendant `timeout_s` alors que les roues tournent,
        le groupe freine tout seul.
        """
        self.stop_watchdog()
        self._wd_timeout = timeout_s
        self._wd_stop.clear()
        self._last_cmd = time.monotonic()
        self._wd_thread = threading.Thread(target=self._wd_loop, daemon=True, name="MotorGroup_watchdog")
        self._wd_thread.start()

    def stop_watchdog(self) -> None:
        if self._wd_thread and self._wd_thread.is_alive():
            self._wd_stop.set()
            self._wd_thread.join(timeout=1.0)
        self._wd_thread = None

    def _wd_loop(self) -> None:
        while not self._wd_stop.wait(0.02):
            if self._moving and time.monotonic() - self._last_cmd > self._wd_timeout:
                print(f"⚠️ Watchdog : aucune consigne depuis {self._wd_timeout:.2f}s, arrêt")
                try:
                    self.stop_all(255)
                except Exception as e:           # ne jamais tuer le thread de sécurité
                    print(f"⚠️ Watchdog : erreur d'arrêt : {e}")

    # =============================
    #   LECTURE DE POSITION EN CONTINU
    # =============================

    def start_polling(self, hz: float = 100.0) -> None:
        """
        Lit la position de chaque moteur à `hz` dans un thread. Résultat : motor.position_sample
        = (pulses, timestamp perf_counter). Les Motor doivent avoir été créés avec notifier=...
        (un seul lecteur du bus : le Notifier).
        """
        if any(m._rx is None for m in self.motors):
            raise RuntimeError("start_polling : créez chaque Motor avec notifier=notifier")
        self.stop_polling()
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, args=(1.0 / hz,),
                                             daemon=True, name="MotorGroup_poller")
        self._poll_thread.start()

    def stop_polling(self) -> None:
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_stop.set()
            self._poll_thread.join(timeout=1.0)
        self._poll_thread = None

    def _poll_loop(self, period: float) -> None:
        # Les réponses des trois moteurs sont indépendantes (IDs CAN distincts).
        # Les lire séquentiellement cumule jusqu'à 3 timeouts de 10 ms et plafonne
        # mécaniquement la cadence autour de 30-40 Hz. Chaque moteur possède son
        # BufferedReader ; on peut donc attendre les trois réponses en parallèle.
        with ThreadPoolExecutor(max_workers=len(self.motors), thread_name_prefix="motor-read") as pool:
            nxt = time.perf_counter()
            while not self._poll_stop.is_set():
                futures = [pool.submit(m.read_position, timeout=min(0.01, period))
                           for m in self.motors]
                for m, future in zip(self.motors, futures):
                    try:
                        future.result()
                    except Exception as e:
                        print(f"⚠️ Poller moteur {m.can_id} : {e}")
                if self.on_round is not None:
                    try:
                        self.on_round([m.position_sample for m in self.motors])
                    except Exception as e:
                        print(f"⚠️ Poller : erreur dans on_round : {e}")
                nxt += period
                delay = nxt - time.perf_counter()
                if delay > 0:
                    self._poll_stop.wait(delay)
                else:
                    nxt = time.perf_counter()        # en retard : on se recale

    def wheel_positions(self) -> list[Optional[tuple]]:
        """Dernières (pulses, timestamp) de chaque roue (None si pas encore lu)."""
        return [m.position_sample for m in self.motors]

    # =============================
    #   LECTURES / ATTENTES
    # =============================

    def wait_all_stopped(self, timeout: float = 30.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            if all(not m.is_running() for m in self.motors):
                return True
            time.sleep(0.01)
        return False

    def read_all_positions(self) -> list[Optional[int]]:
        return [m.read_position() for m in self.motors]

    def read_all_speeds(self) -> list[Optional[int]]:
        return [m.read_speed() for m in self.motors]

    def close(self) -> None:
        self.stop_watchdog()
        self.stop_polling()
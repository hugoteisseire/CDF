"""
Module de contrôle des moteurs MKS SERVO via CAN bus.
Fournit une classe Motor pour gérer individuellement chaque moteur.
"""

import time
import can
import threading
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
    - Thread daemon pour gestion asynchrone des changements de vitesse/direction
    """
    
    # Constantes pour conversion accélération
    TICKS_PER_REVOLUTION = 3200  # 16 subdivisions × 200 pas/tour
    
    def __init__(self, bus: can.Bus, can_id: int, mks_servo: Optional[MksServo] = None, 
                 enable_async_control: bool = False,
                 wheel_diameter_mm: float = 58.0,
                 steps_per_revolution: int = 200,
                 subdivisions: int = 16,
                 gear_ratio: float = 15.0/48.0):
        """
        Initialise un moteur.
        
        Args:
            bus: Interface CAN initialisée
            can_id: ID CAN du moteur (1, 2, 3, etc.)
            mks_servo: Instance MksServo optionnelle (pour fonctions avancées)
            enable_async_control: Active le thread daemon pour contrôle asynchrone
            wheel_diameter_mm: Diamètre de la roue en mm (par défaut 60mm)
            steps_per_revolution: Nombre de pas par tour moteur (par défaut 200)
            subdivisions: Nombre de subdivisions (par défaut 16)
            gear_ratio: Rapport de réduction (par défaut 1.0, >1 si réducteur)
        """
        self.bus = bus
        self.can_id = can_id
        self.servo = mks_servo
        self.state = MotorState()
        
        # Paramètres mécaniques pour conversions distance/pulses
        self.wheel_diameter_mm = wheel_diameter_mm
        self.steps_per_revolution = steps_per_revolution
        self.subdivisions = subdivisions
        self.gear_ratio = gear_ratio
        
        # Calcul des constantes de conversion
        self._update_conversion_constants()
        
        # Thread de contrôle asynchrone
        self._stop_event = threading.Event()
        self._control_thread = None
        self._async_control_enabled = enable_async_control
        
        if enable_async_control:
            self.start_async_control()
    
    def start_async_control(self) -> None:
        """Démarre le thread daemon de contrôle asynchrone."""
        if self._control_thread is None or not self._control_thread.is_alive():
            self._stop_event.clear()
            self._async_control_enabled = True  # Active le flag
            self._control_thread = threading.Thread(
                target=self._control_loop,
                daemon=True,
                name=f"Motor_{self.can_id}_Control"
            )
            self._control_thread.start()
    
    def stop_async_control(self) -> None:
        """Arrête le thread daemon de contrôle asynchrone."""
        if self._control_thread and self._control_thread.is_alive():
            self._stop_event.set()
            self._control_thread.join(timeout=1.0)
            self._async_control_enabled = False  # Désactive le flag
    
    def _control_loop(self) -> None:
        """
        Boucle de contrôle asynchrone (thread daemon).
        Gère automatiquement les changements de vitesse et de direction.
        """
        while not self._stop_event.is_set():
            try:
                # État 1: Idle - Détection de changements
                if self.state.control_state == 1:
                    if self.state.speed != self.state.target_speed:
                        self.state.control_state = 2  # Changement de vitesse
                    elif self.state.direction != self.state.target_direction:
                        self.state.control_state = 3  # Changement de direction
                
                # État 2: Changement de vitesse
                elif self.state.control_state == 2:
                    if self.state.direction != self.state.target_direction:
                        self.state.control_state = 3  # Priorité au changement de direction
                    else:
                        self._apply_velocity_change()
                        self.state.speed = self.state.target_speed
                        self.state.control_state = 1
                
                # État 3: Changement de direction
                elif self.state.control_state == 3:
                    self._apply_direction_change()
                    self.state.direction = self.state.target_direction
                    self.state.speed = self.state.target_speed
                    self.state.control_state = 1
                
                else:
                    print(f"[ERREUR Motor {self.can_id}] État inconnu: {self.state.control_state}")
                    self.state.control_state = 1
                
                # Petite pause pour éviter une boucle trop rapide
                time.sleep(0.001)
                
            except Exception as e:
                print(f"[ERREUR Motor {self.can_id}] Exception dans control_loop: {e}")
                time.sleep(0.01)
    
    def _apply_velocity_change(self) -> None:
        """Applique un changement de vitesse (appelé par le thread)."""
        self.move_velocity(
            speed=self.state.target_speed,
            acceleration=self.state.acceleration,
            direction=self.state.target_direction
        )
    
    def _apply_direction_change(self) -> None:
        """
        Applique un changement de direction (appelé par le thread).
        Arrête le moteur, attend, puis relance dans la nouvelle direction.
        """
        # Arrêt brutal avec direction opposée
        self.move_velocity(
            speed=0,
            acceleration=250,
            direction= self.state.direction
        )
        time.sleep(0.1)  # Pause fixe si pas de servo
        
        # Relance dans la nouvelle direction
        self.move_velocity(
            speed=self.state.target_speed,
            acceleration=self.state.acceleration,
            direction=self.state.target_direction
        )
    
    def set_target_velocity(self, speed: int) -> None:
        """
        Définit la vitesse et direction cible (pour contrôle asynchrone).
        Le thread daemon appliquera les changements automatiquement.
        
        Args:
            speed: Vitesse cible en RPM (0-3000), peut être négative
            direction: 0=CCW, 1=CW. Si None, déduit du signe de speed
            acceleration: Accélération (0-255). Si None, utilise la valeur actuelle
        """
        if not self._async_control_enabled:
            raise RuntimeError("Le contrôle asynchrone n'est pas activé. "
                             "Créez Motor avec enable_async_control=True")
        
        # Gestion du sens
      
        direction = 1 if speed < 0 else 0
        speed = abs(speed)
        
        # Validation
        if not (0 <= speed <= 3000):
            raise ValueError(f"Speed must be 0-3000 RPM, got {speed}")
        
        # Mise à jour des cibles
        self.state.target_speed = speed
        self.state.target_direction = direction
        
    # =============================
    #   CONVERSION ACCÉLÉRATION
    # =============================
    
    @staticmethod
    def linear_to_acc(accel_pulses_s2: float) -> int:
        """
        Convertit une accélération linéaire en paramètre MKS.
        
        Formule : acc_param = 256 - (20000 / a_linear[pulses/s²])
        
        Args:
            accel_pulses_s2: Accélération en pulses/s²
            
        Returns:
            Paramètre d'accélération MKS (0-255)
        """
        if accel_pulses_s2 <= 0:
            return 255  # Accélération instantanée
        
        # Conversion depuis pulses/s² vers RPM/s
        rpm_per_s = accel_pulses_s2 * 60 / Motor.TICKS_PER_REVOLUTION
        
        # Formule MKS : acc = 256 - (20000 / rpm_per_s)
        acc = 256 - (20000 / rpm_per_s)
        
        return max(0, min(255, int(acc)))
    
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
        direction: Optional[int] = None
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
            direction = 1 if distance_mm < 0 else 0
        
        # Appel de la méthode move_relative existante
        self.move_relative(
            pulses=abs(pulses),
            speed=rpm,
            acceleration=acceleration,
            direction=direction
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
        direction: Optional[int] = None
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
        
        # Conversion accélération si nécessaire
        #if acceleration > 255:
        acceleration = self.linear_to_acc(acceleration)
        
        # Validation
        if not (0 <= speed <= 3000):
            raise ValueError(f"Speed must be 0-3000 RPM, got {speed}")
        if not (0 <= acceleration <= 255):
            raise ValueError(f"Acceleration must be 0-255, got {acceleration}")
        if not (0 <= pulses <= 0xFFFFFF):
            raise ValueError(f"Pulses must be 0-{0xFFFFFF}, got {pulses}")
        
        # Construction du message
        code = 0xFD
        speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
        speed_low = speed & 0xFF
        pulse_bytes = [(pulses >> shift) & 0xFF for shift in (16, 8, 0)]
        
        data = [code, speed_high, speed_low, acceleration] + pulse_bytes
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
    
    def read_speed(self, timeout: float = 0.2) -> Optional[int]:
        """
        Lit la vitesse actuelle du moteur.
        
        Args:
            timeout: Délai d'attente maximal (secondes)
            
        Returns:
            Vitesse en RPM (signée), ou None si échec
        """
        opcode = 0x32
        msg_data = [opcode, opcode]
        crc = (int(self.can_id) + int(sum(msg_data))) & 0xFF

        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=msg_data + [crc],
            is_extended_id=False
        )
        msg.dlc = 3
        
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"⚠️ CAN error: {e}")
            return None
        
        start = time.time()
        while time.time() - start < timeout:
            response = self.bus.recv(0.0)
            if response and response.arbitration_id == self.can_id and len(response.data) == 4:
                if response.data[0] != opcode:
                    continue
                
                expected_crc = (self.can_id + sum(response.data[:-1])) & 0xFF
                if response.data[-1] != expected_crc:
                    continue
                
                speed = int.from_bytes(response.data[1:3], byteorder="big", signed=True)
                self.state.speed = speed
                self.state.last_update = time.time()
                return speed
        
        return None
    
    def read_position(self, timeout: float = 0.1) -> Optional[int]:
        """
        Lit la position actuelle du moteur.
        
        Args:
            timeout: Délai d'attente maximal (secondes)
            
        Returns:
            Position en pulses (int48 signé), ou None si échec
        """
        opcode = 0x31
        msg_data = [opcode, opcode]
        crc = (int(self.can_id) + int(sum(msg_data))) & 0xFF

        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=msg_data + [crc],
            is_extended_id=False
        )
        
        self.bus.send(msg)
        
        start = time.perf_counter()
        while time.perf_counter() - start < timeout:
            response = self.bus.recv(0.0)
            if response and response.arbitration_id == self.can_id and len(response.data) == 8:
                if response.data[0] != opcode:
                    continue
                
                expected_crc = (self.can_id + sum(response.data[:-1])) & 0xFF
                if response.data[-1] != expected_crc:
                    continue
                
                pos_bytes = response.data[1:7]
                position = int.from_bytes(pos_bytes, byteorder='big', signed=True)
                self.state.position = position
                self.state.last_update = time.time()
                return position
        
        return None
    
    def is_running(self) -> bool:
        """
        Vérifie si le moteur est en mouvement.
        
        Utilise l'instance MksServo si disponible.
        
        Returns:
            True si le moteur tourne, False sinon
        """
        if self.servo:
            return self.servo.is_motor_running()
        
        # Fallback : lire la vitesse
        speed = self.read_speed(timeout=0.05)
        return speed is not None and abs(speed) > 0
    
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
        """Représentation textuelle du moteur."""
        async_status = "async" if self._async_control_enabled else "sync"
        return (
            f"Motor(id={self.can_id}, "
            f"pos={self.state.position}, "
            f"speed={self.state.speed} RPM, "
            f"target={self.state.target_speed} RPM, "
            f"dir={'CW' if self.state.direction else 'CCW'}, "
            f"mode={async_status})"
        )
    
    def __del__(self):
        """Nettoyage lors de la destruction de l'objet."""
        self.stop_async_control()


# =============================
#   CLASSE DE GESTION MULTI-MOTEURS
# =============================

class MotorGroup:
    """
    Gère plusieurs moteurs simultanément avec synchronisation.
    """
    
    def __init__(self, motors: list[Motor]):
        """
        Initialise un groupe de moteurs.
        
        Args:
            motors: Liste des moteurs à gérer
        """
        self.motors = motors
    
    def move_synchronized(
        self,
        wheel_speeds: list[float],
        max_speed: int,
        pulses_per_step: int,
        base_acceleration: float
    ) -> None:
        """
        Déplace tous les moteurs de manière synchronisée.
        
        Les vitesses et accélérations sont proportionnelles pour que
        tous les moteurs atteignent leur Vmax et s'arrêtent en même temps.
        
        Args:
            wheel_speeds: Liste des vitesses normalisées (-1.0 à 1.0)
            max_speed: Vitesse maximale en RPM
            pulses_per_step: Distance maximale en pulses
            base_acceleration: Accélération de base en pulses/s²
        """
        max_abs_speed = max(abs(s) for s in wheel_speeds)
        if max_abs_speed == 0:
            return
        
        for i, (motor, wheel_speed) in enumerate(zip(self.motors, wheel_speeds)):
            # Calcul du ratio de vitesse
            speed_ratio = abs(wheel_speed) / max_abs_speed
            
            # Vitesse et distance proportionnelles
            motor_speed = int(speed_ratio * max_speed)
            pulses = int(speed_ratio * pulses_per_step)
            
            # Accélération proportionnelle pour synchronisation
            accel_pulses_s2 = speed_ratio * base_acceleration
            
            # Direction
            direction = 1 if wheel_speed < 0 else 0
            
            # Envoi de la commande
            motor.move_relative(
                pulses=pulses,
                speed=motor_speed,
                acceleration=accel_pulses_s2,
                direction=direction
            )

    def move_synchronized_distance(
        self,
        distances_mm: list[float],
        speed_rpm: float,
        base_acceleration: float
    ) -> None:
        
        """
        Déplace tous les moteurs de manière synchronisée sur des distances données.
        
        Les vitesses et accélérations sont proportionnelles pour que
        tous les moteurs atteignent leur Vmax et s'arrêtent en même temps.
        
        Args:
            distances_mm: Liste des distances à parcourir en mm
            speed_rpm: Vitesse maximale en RPM
            base_acceleration: Accélération de base en pulses/s²
        """
        # Conversion des distances en pulses
        pulses_list = [
            motor.distance_to_pulses(dist_mm) for motor, dist_mm in zip(self.motors, distances_mm)
        ]
        
        max_pulses = max(abs(pulses) for pulses in pulses_list)
        if max_pulses == 0:
            return
        
        for i, (motor, pulses) in enumerate(zip(self.motors, pulses_list)):
            # Calcul du ratio de distance
            speed_ratio = abs(pulses) / max_pulses
            
            # Vitesse proportionnelle
            motor_speed = int(speed_ratio * speed_rpm)
            
            # Accélération proportionnelle pour synchronisation
            accel_pulses_s2 = speed_ratio * base_acceleration
            
            # Direction
            direction = 1 if pulses < 0 else 0
            
            # Envoi de la commande
            motor.move_relative(
                pulses=abs(pulses),
                speed=motor_speed,
                acceleration=accel_pulses_s2,
                direction=direction
            )





    def stop_all(self, acceleration: int = 255) -> None:
        """Arrête tous les moteurs."""
        for motor in self.motors:
            motor.stop_relative()
            motor.start_async_control() # Assure que le contrôle asynchrone est actif   
        velocities = [0 for _ in self.motors]
        self.set_velocities(velocities, max_speed=0)
        for motor in self.motors:
            motor.servo.emergency_stop_motor()
        return
       
        for motor in self.motors:
            motor.stop_async_control()
            motor.servo.emergency_stop_motor()
            motor.move_velocity(speed=0, acceleration=255)
            motor.move_relative(
                pulses=abs(1),
                speed=100,
                acceleration=800,
                direction=1
            )

            
    
    def wait_all_stopped(self, timeout: float = 30.0) -> bool:
        """
        Attend que tous les moteurs s'arrêtent.
        
        Returns:
            True si tous arrêtés, False si timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            all_stopped = all(not m.is_running() for m in self.motors)
            if all_stopped:
                return True
            time.sleep(0.01)
        
        return False
    
    def read_all_positions(self) -> list[Optional[int]]:
        """Lit les positions de tous les moteurs."""
        return [m.read_position() for m in self.motors]
    
    def read_all_speeds(self) -> list[Optional[int]]:
        """Lit les vitesses de tous les moteurs."""
        return [m.read_speed() for m in self.motors]
    
    def set_velocities(self, wheel_speeds: list[float], max_speed: float = None) -> None:
        """
        Applique les vitesses normalisées à chaque moteur du groupe (mode asynchrone).
        
        Cette méthode simplifie l'application de vitesses calculées par compute_wheel_speeds_global
        en gérant automatiquement l'indexation et la conversion vers RPM.
        
        Args:
            wheel_speeds: Liste de vitesses normalisées [-1, 1] ou valeurs RPM
            max_speed: Vitesse maximale en RPM (multiplie les vitesses normalisées).
                      Si None, wheel_speeds est interprété comme des valeurs RPM directes.
        
        Raises:
            ValueError: Si le nombre de vitesses ne correspond pas au nombre de moteurs
            RuntimeError: Si les moteurs ne sont pas en mode asynchrone
        
        Example:
            >>> raw_speeds = compute_wheel_speeds_global(robot, vx, vy, omega)
            >>> motors.set_velocities(raw_speeds, max_speed=200)
        """
        if len(wheel_speeds) != len(self.motors):
            raise ValueError(
                f"Nombre de vitesses ({len(wheel_speeds)}) != nombre de moteurs ({len(self.motors)})"
            )
        
        # Vérifier que tous les moteurs sont en mode asynchrone
        for motor in self.motors:
            if not motor._async_control_enabled:
                raise RuntimeError(
                    f"Le moteur {motor.can_id} n'est pas en mode asynchrone. "
                    "Appelez motor.start_async_control() d'abord."
                )
        
        # Appliquer les vitesses
        for i, motor in enumerate(self.motors):
            speed = wheel_speeds[i] * max_speed if max_speed else wheel_speeds[i]
            motor.set_target_velocity(int(speed))

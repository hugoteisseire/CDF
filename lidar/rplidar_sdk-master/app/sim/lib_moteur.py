import time
import can
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction
from mks_servo_can.mks_enums import CalibrationResult, RunMotorResult


def read_motor_speed(bus, can_id, timeout=0.2):
    opcode = 0x32
    msg_data = [opcode,opcode]
    crc = (can_id + sum(msg_data)) & 0xFF
    msg = can.Message(arbitration_id=can_id, data=msg_data + [crc], is_extended_id=False)
    msg.dlc = 3

    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"⚠️ CAN error: {e}")
        return None

    start = time.time()
    while time.time() - start < timeout:        
        response = bus.recv(0.0)
        if response and response.arbitration_id == can_id and len(response.data) == 4:
            if response.data[0] != opcode:
                print(f"⚠️ Unexpected opcode.\nop_code:0x{opcode:02X}\nmessage.data[0]:0x{response.data[0]:02X}")
                continue

            expected_crc = (can_id + sum(response.data[:-1])) & 0xFF
            if response.data[-1] != expected_crc:
                print(f"⚠️ CRC invalide: reçu 0x{response.data[-1]:02X}, attendu 0x{expected_crc:02X}")
                continue

            speed = int.from_bytes(response.data[1:3], byteorder="big", signed=True)
            return speed

    return 0


def read_motor_position(bus, can_id, timeout=0.1):
    """
    Lit la position du moteur (int48) via la commande 0x31.

    Args:
        bus (can.Bus): Bus CAN initialisé
        can_id (int): ID du moteur
        timeout (float): Délai d’attente

    Returns:
        int: Position du moteur (signée, int48)
    """
    opcode = 0x31
    msg_data = [opcode,opcode]
    crc = (can_id + sum(msg_data)) & 0xFF
    msg = can.Message(arbitration_id=can_id, data=msg_data + [crc], is_extended_id=False)

    bus.send(msg)

    start = time.perf_counter()
    while time.perf_counter() - start < timeout:
        response = bus.recv(0.0)
        if response and response.arbitration_id == can_id and len(response.data) == 8:
            if response.data[0] != opcode:
                continue
            expected_crc = (can_id + sum(response.data[:-1])) & 0xFF
            if response.data[-1] != expected_crc:
                print("⚠️ CRC invalide")
                continue

            # 6 octets pour la position int48
            pos_bytes = response.data[1:7]
            position = int.from_bytes(pos_bytes, byteorder='big', signed=True)
            return position
    return 0
    raise TimeoutError("⏱️ Pas de réponse valide pour la position")


def move_relative(bus, can_id, direction, speed, acceleration, pulses):
    """
    Envoie une commande de mouvement moteur via le protocole SERVO042D/57D.

    Paramètres :
        - bus : interface CAN initialisée (type: can.interface.Bus)
        - can_id : ID du périphérique (ex: 0x01 ou 0x02)
        - direction : 0 = CCW (avant), 1 = CW (arrière)
        - speed : vitesse (0 à 3000)
        - acceleration : accélération (0 à 255)
        - pulses : nombre de pas (0 à 0xFFFFFF)
    """
    if not (0 <= speed <= 3000):
        raise ValueError("speed must be between 0 and 3000")
    if not (0 <= acceleration <= 255):
        raise ValueError("acceleration must be between 0 and 255")
    if not (0 <= pulses <= 0xFFFFFF):
        raise ValueError("pulses must be a 24-bit value")

    # Code de commande FD
    code = 0xFD

    # Construction des 2 octets speed (bits sur 12)
    speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    speed_low = speed & 0xFF

    # Pulses sur 3 octets (MSB en premier)
    pulse_bytes = [(pulses >> shift) & 0xFF for shift in (16, 8, 0)]

    # Données à envoyer
    data = [code, speed_high, speed_low, acceleration] + pulse_bytes

    # Calcul du CRC : somme CAN ID + data (sans CRC), modulo 256
    crc = (can_id + sum(data)) & 0xFF
    data.append(crc)

    # Création du message CAN
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    msg.dlc = 8  # 8 octets de données

    # Envoi
    try:
        bus.send(msg)
        print(f"Message envoyé : {msg}")
    except can.CanError as e:
        print(f"Erreur lors de l'envoi du message CAN: {e}")


def reset_zero(bus, can_id):
    opcode = 0x92
    msg_data = [opcode]
    crc = (can_id + sum(msg_data)) & 0xFF
    msg = can.Message(arbitration_id=can_id, data=msg_data + [crc], is_extended_id=False)
    msg.dlc = 2
    bus.send(msg)

def calibrate(bus, can_id):
    opcode = 0x80
    msg_data = [opcode, 0x00]
    crc = (can_id + sum(msg_data)) & 0xFF
    msg = can.Message(arbitration_id=can_id, data=msg_data + [crc], is_extended_id=False)
    msg.dlc = 3
    bus.send(msg)

def stop_pos_soft(bus, can_id, acceleration=4):
    """
    Stoppe le moteur progressivement avec une accélération donnée.
    """
    if not (0 <= acceleration <= 255):
        raise ValueError("acceleration must be between 0 and 255")

    code = 0xFD
    speed_high = 0x00
    speed_low = 0x00
    acc = acceleration
    pulses = [0x00, 0x00, 0x00]

    data = [code, speed_high, speed_low, acc] + pulses
    crc = (can_id + sum(data)) & 0xFF
    data.append(crc)

    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    msg.dlc = 8

    try:
        bus.send(msg)
        print(f"Arrêt progressif envoyé avec acc={acceleration}.")
    except can.CanError as e:
        print(f"Erreur lors de l'arrêt progressif : {e}")

def move_velocity(bus, can_id, direction, speed, acceleration):
    """
    Envoie une commande de mouvement en vitesse via le protocole SERVO042D/57D.

    Paramètres :
        - bus : interface CAN initialisée (type: can.interface.Bus)
        - can_id : ID du périphérique (ex: 0x01 ou 0x02)
        - direction : 0 = CCW (avant), 1 = CW (arrière)
        - speed : vitesse (0 à 3000 RPM)
        - acceleration : accélération (0 à 255)
    """
    if not (0 <= speed <= 3000):
        raise ValueError("speed must be between 0 and 3000")
    if not (0 <= acceleration <= 255):
        raise ValueError("acceleration must be between 0 and 255")

    code = 0xF6

    # Construction des octets de vitesse (12 bits sur 2 octets)
    speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    speed_low = speed & 0xFF

    data = [code, speed_high, speed_low, acceleration]

    # CRC = somme CAN_ID + data (sans CRC) & 0xFF
    crc = (can_id + sum(data)) & 0xFF
    data.append(crc)

    # Création et envoi du message CAN
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    msg.dlc = 5

    try:
        bus.send(msg)
        #print(f"Vitesse envoyée : direction={direction}, speed={speed}, acc={acceleration}")
    except can.CanError as e:
        print(f"Erreur d'envoi CAN : {e}")

def stop_velocity_soft(bus, can_id, acceleration=4):
    """
    Stoppe doucement le moteur en mode vitesse avec une accélération donnée.
    """
    if not (0 <= acceleration <= 255):
        raise ValueError("acceleration must be between 0 and 255")

    code = 0xF6
    speed = 0
    direction = 0  # la direction n’a pas d’importance ici si speed = 0

    # Encodage identique au move_velocity
    speed_high = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    speed_low = speed & 0xFF

    data = [code, speed_high, speed_low, acceleration]
    crc = (can_id + sum(data)) & 0xFF
    data.append(crc)

    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    msg.dlc = 5

    try:
        bus.send(msg)
        print(f"Commande d'arrêt progressif envoyée (acc={acceleration}).")
    except can.CanError as e:
        print(f"Erreur d'envoi CAN : {e}")

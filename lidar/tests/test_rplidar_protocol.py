"""Décodage d'une trame de mesure du mode SCAN standard (0x20) du RPLIDAR C1.

Format (SDK Slamtec, sl_lidar_response_measurement_node_t), 5 octets :
    octet 0      : quality:6 | S_bar:1 | S:1          (bits 7..2 | 1 | 0)
    octets 1-2   : u16 LE = angle_q6:15 | C:1          (C = bit 0 de l'octet 1, toujours 1)
    octets 3-4   : u16 LE = distance_q2 (mm * 4)

    python -m pytest lidar/tests/test_rplidar_protocol.py
"""
import sys
import types

try:
    import serial  # noqa: F401
except ImportError:  # le décodage n'a pas besoin de pyserial : module factice pour l'import
    _serial_stub = types.ModuleType("serial")
    _serial_stub.Serial = object
    _serial_stub.SerialException = Exception
    sys.modules["serial"] = _serial_stub

import pytest

from lidar.rplidarc1.protocol import Response

# le parseur arrondit l'angle au 1/3° le plus proche, puis à 0.01° : erreur max 1/6° + 0.005°
ROUND_TOL_DEG = 1 / 6 + 0.005 + 1e-9


def encode_frame(angle_q6: int, distance_q2: int = 4000, quality: int = 47, start: bool = False) -> bytes:
    s = 1 if start else 0
    b0 = (quality << 2) | ((1 - s) << 1) | s
    angle_field = (angle_q6 << 1) | 1  # bit C = 1
    return bytes([
        b0,
        angle_field & 0xFF,
        (angle_field >> 8) & 0xFF,
        distance_q2 & 0xFF,
        (distance_q2 >> 8) & 0xFF,
    ])


@pytest.mark.parametrize("angle_deg", [0.0, 1.0, 45.0, 90.0, 180.0, 270.0, 359.0])
def test_known_angles_decode_exactly(angle_deg):
    frame = encode_frame(round(angle_deg * 64), quality=47)
    assert Response._check_byte_alignment(frame[0], frame[1])
    quality, angle, _ = Response._parse_simple_scan_result(frame)
    assert quality == 47
    assert angle == pytest.approx(angle_deg)


def test_angle_error_bounded_by_rounding_over_full_turn():
    # Toutes les valeurs angle_q6 d'un tour : l'ancien décodage ((b1 & 0x7F) | b2 << 7)
    # donnait une erreur en dents de scie jusqu'à ~±1° (période 2°).
    worst = 0.0
    for angle_q6 in range(0, 360 * 64):
        _, angle, _ = Response._parse_simple_scan_result(encode_frame(angle_q6))
        worst = max(worst, abs(angle - angle_q6 / 64))
    assert worst <= ROUND_TOL_DEG


def test_sawtooth_case_from_old_formula():
    # 1.5° = q6 96 : l'ancien décodage lisait 65/64 ≈ 1.02° (erreur ~-0.5°).
    _, angle, _ = Response._parse_simple_scan_result(encode_frame(96))
    assert abs(angle - 1.5) <= ROUND_TOL_DEG

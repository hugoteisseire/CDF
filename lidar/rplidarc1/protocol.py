"""Encodage des commandes et décodage des réponses du RPLIDAR C1."""

import asyncio
import logging
from typing import Coroutine, Optional, Tuple, Union
from enum import IntEnum

import time
import serial
from .utils import ByteEnum

"""
This module implements the RPLidar protocol for communication with the device.
It defines request and response handling, byte definitions, and data parsing
according to the RPLidar C1 specifications.
"""


class CommonBytes(ByteEnum):
    """
    Common bytes used in the RPLidar protocol for both requests and responses.
    """

    SYNC_BYTE = b"\xa5"


class RequestBytes(ByteEnum):
    """
    Command bytes used for sending requests to the RPLidar device.

    These bytes are combined with the SYNC_BYTE to form complete commands.
    Some commands require waiting after sending for the device to process.
    """

    STOP_BYTE = b"\x25"  # wait 10ms
    SCAN_BYTE = b"\x20"
    FORCE_SCAN_BYTE = b"\x21"
    INFO_BYTE = b"\x50"
    GET_HEALTH_BYTE = b"\x52"
    RESET_BYTE = b"\x40"  # wait 500ms
    EXPRESS_SCAN_BYTE = b"\x82"
    GET_INFO_BYTE = b"\x50"
    GET_SAMPLE_RATE = b"\x59"
    GET_LIDAR_CONF = b"\x84"


class ResponseBytes(ByteEnum):
    """
    Bytes used in responses from the RPLidar device.

    These bytes help identify and validate the response type.
    """

    RESPONSE_SYNC_BYTE = b"\x5a"
    RESPONSE_HEALTH_BYTE = b"\x03"
    RESPONSE_SCAN_BYTE = b"\x81"


class ResponseMode(IntEnum):
    """
    Enumeration of response modes from the RPLidar device.

    SINGLE_RESPONSE: The device will send a single response packet.
    MUTLI_RESPONSE: The device will send multiple response packets.
    """

    SINGLE_RESPONSE = 0
    MUTLI_RESPONSE = 1


class HealthStatus(IntEnum):
    """
    Enumeration of health status codes returned by the RPLidar device.

    GOOD: The device is functioning normally.
    WARNING: The device has detected a potential issue.
    ERROR: The device has detected a critical error.
    """

    GOOD = 0
    WARNING = 1
    ERROR = 2


class Request:
    """
    Class for creating and sending requests to the RPLidar device.

    This class provides static methods to create properly formatted request
    packets and send them to the device over a serial connection.
    """

    @staticmethod
    def create_request(command: RequestBytes) -> bytes:
        """
        Create a properly formatted request packet.

        Args:
            command (RequestBytes): The command byte to include in the request.

        Returns:
            bytes: The complete request packet (SYNC_BYTE + command).
        """
        request = CommonBytes.SYNC_BYTE + command
        return request

    @staticmethod
    def send_request(serial: serial.Serial, request: bytes):
        """
        Send a request packet to the RPLidar device.

        Args:
            serial (serial.Serial): The serial connection to the device.
            request (bytes): The request packet to send.
        """
        serial.write(request)
        serial.flush()


class Response:
    """
    Class for handling responses from the RPLidar device.

    This class provides static methods to parse response descriptors,
    handle different types of responses, and extract data from response packets.
    """

    logger = logging.getLogger(__name__)

    RESPONSE_DESCRIPTOR_LENGTH = 7  # Length of the response descriptor in bytes
    FRAME_PERIOD_NS = 200_000       # RPLIDAR C1 : 5000 mesures/s, une trame toutes les 200 µs

    @staticmethod
    def parse_response_descriptor(serial: serial.Serial) -> Tuple[int, ResponseMode]:
        """
        Parse the response descriptor from the RPLidar device.

        The response descriptor contains information about the length and mode
        of the response data that follows.

        Args:
            serial (serial.Serial): The serial connection to the device.

        Returns:
            Tuple[int, ResponseMode]: A tuple containing the length of the response
                data and the response mode (single or multi).

        Raises:
            ValueError: If the response descriptor does not contain the expected sync bytes.
        """
        Response.logger.debug("Parsing Response Descriptor.")
        descriptor = serial.read(Response.RESPONSE_DESCRIPTOR_LENGTH)
        Response._check_response_sync_bytes(descriptor)

        length, mode, check = Response._calculate_request_details(descriptor)

        response_mode = ResponseMode(mode)

        Response.logger.warning(f"In waiting: {serial.in_waiting}")

        return length, response_mode

    @staticmethod
    def handle_response(*args, **kwargs) -> Union[bytes, Coroutine]:
        """
        Handle a response from the RPLidar device.

        This method determines the appropriate response handler based on the
        provided arguments and delegates to either parse_single_response or
        multi_response_handler.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments. Expected keys depend on the
                response type:
                - For single responses: 'serial' and 'length'
                - For multi responses: 'serial', 'stop_event', 'output_queue', and 'length'

        Returns:
            Union[bytes, Coroutine]: Either the parsed response data for a single
                response, or a coroutine for handling multi responses.

        Raises:
            NotImplementedError: If the provided arguments do not match the
                expected patterns for either single or multi responses.
        """
        if len(args) + len(kwargs) == 2 and all(
            p in kwargs for p in ["serial", "length"]
        ):
            return Response.parse_single_response(*args, **kwargs)
        elif all(
            p in kwargs for p in ["serial", "stop_event", "output_queue", "length"]
        ):
            Response.logger.debug("Creating async multi response coroutine.")
            return Response.multi_response_handler(*args, **kwargs)
        else:
            raise NotImplementedError

    @staticmethod
    def parse_single_response(serial: serial.Serial, length: int) -> bytes:
        """
        Parse a single response from the RPLidar device.

        Args:
            serial (serial.Serial): The serial connection to the device.
            length (int): The length of the response data to read.

        Returns:
            bytes: The response data.
        """
        data = serial.read(length)
        return data

    @staticmethod
    async def multi_response_handler(
        serial: serial.Serial,
        stop_event: asyncio.Event,
        output_queue: asyncio.Queue,
        length: int,
        output_dict: Optional[dict],
    ):
        """
        Handle multiple responses from the RPLidar device.

        This coroutine continuously reads scan data from the device until
        the stop_event is set. Each scan result is parsed and added to the
        output_queue and optionally to the output_dict.

        Args:
            serial (serial.Serial): The serial connection to the device.
            stop_event (asyncio.Event): Event to signal when to stop reading.
            output_queue (asyncio.Queue): Queue to store the parsed scan results.
            length (int): The length of each response data packet.
            output_dict (Optional[dict]): Dictionary to store the scan results
                indexed by angle. If None, results are only added to the queue.
        """
        Response.logger.debug("Creating multiresponse coroutine.")
        # Lecture PAR BLOCS : tout ce qui attend dans le port est lu en un seul appel, puis découpé en
        # trames. L'ancienne lecture trame par trame faisait ~2 appels système par point (5000 points/s) :
        # chacun rend le GIL et doit le reprendre. Dès qu'un autre thread du processus calculait, le
        # lecteur prenait du retard, et tant que le port contenait >= 5 octets il ne cédait JAMAIS la
        # main à la boucle asyncio (put() sur une file non bornée ne suspend pas) : la tâche de vidage
        # ne tournait plus, aucun tour n'était publié pendant parfois plus d'une seconde, puis tout
        # arrivait d'un coup avec des horodatages déjà vieux (tours « trop anciens » pour l'EKF).
        buf = b""
        last_t_ns = 0
        while not stop_event.is_set():
            waiting = serial.in_waiting
            if len(buf) + waiting < length:
                await asyncio.sleep(0.002)
                continue
            if waiting:
                buf += serial.read(waiting)
            # Horodatage epoch pris immédiatement après réception du bloc.
            # Ne pas utiliser monotonic_ns ici : ClockAligner serait initialisé plus tard
            # par le thread de découpage, après accumulation possible d'une file de points,
            # ce qui décalerait artificiellement les mesures dans le futur.
            t_rx_ns = time.time_ns()

            frames = []
            i, end = 0, len(buf)
            while end - i >= length:
                if not Response._check_byte_alignment(buf[i], buf[i + 1]):
                    Response.logger.warning(
                        "Verification bytes not matching. Continuing one byte along."
                    )
                    i += 1
                    continue
                frames.append(buf[i:i + length])
                i += length
            buf = buf[i:]

            # Les trames d'un même bloc sont arrivées à la cadence du lidar : la dernière à t_rx_ns, les
            # précédentes FRAME_PERIOD_NS plus tôt chacune (sans jamais remonter avant le bloc précédent).
            n = len(frames)
            for k, frame in enumerate(frames):
                t_ns = max(t_rx_ns - (n - 1 - k) * Response.FRAME_PERIOD_NS, last_t_ns + 1)
                last_t_ns = t_ns
                parsed_tuple = Response._parse_simple_scan_result(frame)
                if parsed_tuple is None:
                    continue
                quality, angle, distance = parsed_tuple
                distance = None if distance == 0 else distance
                if output_dict is not None:
                    output_dict[angle] = distance
                output_queue.put_nowait({"q": quality, "a_deg": angle, "d_mm": distance, "t_rx_ns": t_ns})
            # Céder la main après chaque bloc : la tâche de vidage tourne au moins toutes les quelques ms.
            await asyncio.sleep(0)
        output_queue.task_done()
        Response.logger.info("Completed processing.")
        await asyncio.sleep(0)

    @staticmethod
    def _check_byte_alignment(b1: int, b2: int):
        """
        Check if the bytes are properly aligned according to the RPLidar protocol.

        The RPLidar protocol requires specific bit patterns for proper data alignment:
        - The S bit (least significant bit of the first byte) and S̄ bit (second least
          significant bit of the first byte) must be different (one 0, one 1).
        - The C bit (least significant bit of the second byte) must be set to 1.

        Args:
            b1 (int): The first byte to check.
            b2 (int): The second byte to check.

        Returns:
            bool: True if the bytes are properly aligned, False otherwise.
        """
        control_s_bit = b1 & 0b1
        control_s_bar_bit = (b1 >> 1) & 0b1
        if control_s_bit == control_s_bar_bit:
            Response.logger.warning("S bit verification failed. Realigning.")
            return False
        control_c_bit = b2 & 0b1
        if control_c_bit != 1:
            Response.logger.warning("C bit verification failed. Realigning.")
            return False
        return True

    @staticmethod
    def _parse_simple_scan_result(
        response: bytes,
    ) -> Optional[Tuple[int, float, float]]:
        """
        Parse a simple scan result from the RPLidar device.

        Args:
            response (bytes): The response data to parse.

        Returns:
            Tuple[int, float, float]: A tuple containing:
                - quality: The quality of the scan point (0-63)
                - angle: The angle of the scan point in degrees (0-359.99)
                - distance: The distance of the scan point in millimeters
        """
        quality = (response[0] >> 2) & 0b111111
        angle = ((response[1] & 0b01111111) | (response[2] << 7)) / 64
        angle = round(
            (round(angle * 3) / 3), 2
        )  # round to closest 0.33 which is precision of rplidarc1
        if angle > 360:
            Response.logger.error(
                f"calculated angle {angle} from {response}. Angles should not be >360 so this result will be ignored."
            )
            return None
        distance = ((response[3] | (response[4] << 8)) / 4)-50
        return quality, angle, distance

    @staticmethod
    def _check_response_sync_bytes(descriptor: bytes):
        """
        Check that the response descriptor contains the expected sync bytes.

        Args:
            descriptor (bytes): The response descriptor to check.

        Raises:
            ValueError: If the descriptor does not contain the expected sync bytes.
        """
        if not descriptor[0:1] == CommonBytes.SYNC_BYTE:
            raise ValueError
        if not descriptor[1:2] == ResponseBytes.RESPONSE_SYNC_BYTE:
            raise ValueError

    @staticmethod
    def _calculate_request_details(
        descriptor: bytes,
    ) -> Tuple[int, int, Optional[ResponseBytes]]:
        """
        Calculate the length and mode from the response descriptor.

        Args:
            descriptor (bytes): The response descriptor to parse.

        Returns:
            Tuple[int, int]: A tuple containing the length of the response data
                and the response mode (0 for single, 1 for multi).
        """
        b1, b2, b3, b4, b5 = descriptor[2:7]
        composite_32_bit = (
            b1 | (b2 << 8) | (b3 << 16) | (b4 << 24)
        )  # little endian 32 bit
        mask_30_bit = 0b00111111_11111111_11111111_11111111  # mask with first 2 bits 0
        mask_2_bit = 0b11  # mask to ensure only 2 bits
        length = composite_32_bit & mask_30_bit  # bitwise AND operation with mask
        mode = (
            composite_32_bit >> 30
        ) & mask_2_bit  # right shift to get 2 bit and mask

        check = ResponseBytes.RESPONSE_SCAN_BYTE if b5 == 129 else None
        return length, mode, check

    @staticmethod
    def parse_error_code(response: bytes) -> int:
        """
        Parse an error code from a response.

        Args:
            response (bytes): The response data containing the error code.

        Returns:
            int: The parsed error code.
        """
        b1, b2 = response[1:3]
        return b1 | (b2 << 8)

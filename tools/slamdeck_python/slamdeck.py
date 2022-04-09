from dataclasses import dataclass, fields
from gzip import READ
from multiprocessing import Event
from pkgutil import get_data
from numpy import dtype, ndarray, uint16, uint32, uint8
from enum import IntEnum
import numpy as np
import struct
import typing as t
from threading import Thread
import logging

from queue import Queue
from slamdeck_python.cpx import BackendCPX
from slamdeck_python.vl53l5cx import (VL53L5CX_PowerMode, VL53L5CX_Resolution,
                          VL53L5CX_RangingMode, VL53L5CX_Status,
                          VL53L5CX_TargetOrder, VL53L5CX)
from slamdeck_python.utils import Observable, Subscriber, CallbackHandler, Callback
from time import time

logger = logging.getLogger()


""" Slamdeck API commands """

class SlamdeckCommand(IntEnum):
    GET_DATA            = 0
    GET_SETTINGS        = 1
    SET_SETTINGS        = 2
    START_STREAMING     = 3
    START_STREAMING_ACK = 4
    STOP_STREAMING      = 5
    STOP_STREAMING_ACK  = 6

class SlamdeckResult(IntEnum):
    OK             = 0
    ERROR          = 1
    COMMAND_INVALD = 2

class SlamdeckSensorId(IntEnum):
    MAIN    = 0
    FRONT   = 1
    RIGHT   = 2
    BACK    = 3
    LEFT    = 4
    ALL     = 5
    NOT_SET = 255


@dataclass
class SlamdeckSettings:
    integration_time_ms:  uint32
    sharpener_percent:    uint8
    ranging_frequency_hz: uint8
    resolution:           VL53L5CX_Resolution
    power_mode:           VL53L5CX_PowerMode
    target_order:         VL53L5CX_TargetOrder
    ranging_mode:         VL53L5CX_RangingMode

    _struct_format = 'IBBBBBB'

    def to_bytes(self) -> bytes:
        return struct.pack(
            self._struct_format,
            self.integration_time_ms,
            self.sharpener_percent,
            self.ranging_frequency_hz,
            self.resolution,
            self.power_mode,
            self.target_order,
            self.ranging_mode
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> 'SlamdeckSettings':
        params = struct.unpack(cls._struct_format, data)
        return SlamdeckSettings(
            params[0], params[1], params[2],
            VL53L5CX_Resolution(params[3]),
            VL53L5CX_PowerMode(params[4]),
            VL53L5CX_TargetOrder(params[5]),
            VL53L5CX_RangingMode(params[6]),
        )

class ActionType(IntEnum):
    READ_DATA       = 0
    WRITE      = 1
    DISCONNECT = 3

class Action:

    def __init__(self, type: ActionType):
        self.type = type

    def __repr__(self) -> str:
        return self.type.name

class ActionWrite(Action):

    def __init__(self, command: SlamdeckCommand, data: bytes = None):
        super().__init__(type=ActionType.WRITE)
        self.command = command
        self.data = data

    def __repr__(self) -> str:
        return super().__repr__() + str(self.command)

class ActionRead(Action):

    def __init__(self, command: SlamdeckCommand):
        super().__init__(type=ActionType.READ_DATA)
        self.command = command

    def __repr__(self) -> str:
        return super().__repr__() + str(self.command)



class Slamdeck:

    """
        This class represents the Slamdeck.
        It implements the communiation protocol, and requires a backend driver
        to handle the communication with the actual slamdeck.
    """

    SENSOR_IDS = [
        SlamdeckSensorId.MAIN,
        SlamdeckSensorId.FRONT,
        SlamdeckSensorId.RIGHT,
        SlamdeckSensorId.BACK,
        SlamdeckSensorId.LEFT,
    ]

    def __init__(self, backend: BackendCPX) -> None:
        self._backend = backend
        self._sensors = self._create_sensors()

        self.connecting_handler = CallbackHandler()
        self.connected_handler = CallbackHandler()
        self.disconnected_handler = CallbackHandler()
        self.connection_error_handler = CallbackHandler()
        self.on_new_data_handler = CallbackHandler()

        self._action_queue = Queue() # Actions

        self._settings: SlamdeckSettings = None
        self._is_running = Event()

        self._data_size = 128

        # streaming
        self._is_streaming: bool = False
        self._streaming_rate = 0
        self._samples = 0
        self._t0 = time()

    def get_streaming_rate(self) -> int:
        return self._streaming_rate

    def get_sensor(self, sensor: SlamdeckSensorId) -> VL53L5CX:
        return self._sensors.get(sensor, None)

    def is_connected(self) -> bool:
        return self._is_running.is_set()

    def start_streaming(self) -> None:
        if self._is_streaming:
            logger.error('Already streaming')
            return

        self._is_streaming = True
        self._action_queue.put(ActionWrite(SlamdeckCommand.START_STREAMING))

    def stop_streaming(self) -> None:
        self._is_streaming = False
        self._action_queue.put(ActionWrite(SlamdeckCommand.STOP_STREAMING))

    def disconnect(self) -> None:
        self._action_queue.put(Action(type=ActionType.DISCONNECT))

    def connect(self) -> Thread:
        if self.is_connected():
            logger.warning('Already connected to slamdeck.')
            return

        if not self._backend.connect():
            logging.warning('Failed to connected to backend')
            return

        # We're connected
        self._is_streaming = False
        self.connected_handler.call()
        return Thread(target=self._run, name='Slamdeck', daemon=True).start()

    def _disconnect(self) -> None:
        self._is_running.clear()
        self._is_streaming = False
        self.disconnected_handler.call()

    def _get_settings(self) -> SlamdeckSettings:
        action = ActionWrite(SlamdeckCommand.GET_SETTINGS)
        packet = self._make_packet(action)
        self._backend.write(packet)
        response = self._backend.read()
        self._parse_response(action, response)

    def _clear_action_queue(self) -> None:
        self._action_queue = Queue()

    def _run(self) -> None:
        self._is_running.set()
        self._clear_action_queue()
        self._get_settings()

        while self._is_running.is_set():
            action = self._action_queue.get()

            print(action)

            if action.type == ActionType.WRITE:
                packet = self._make_packet(action)
                self._backend.write(packet)
                response = self._backend.read()
                if response is not None:
                    self._parse_response(action, response)
                else:
                    self._is_streaming = False
            elif action.type == ActionType.READ_DATA:
                response = self._backend.read()
                if response is not None:
                    self._parse_response(action, response)
                    self.on_new_data_handler.call()

                    # Update streaming rate
                    if self._is_streaming:
                        self._samples += 1
                        now = time()
                        if (now - self._t0) > 1:
                            self._streaming_rate = self._samples
                            self._samples = 0
                            self._t0 = now
                else:
                    self._is_streaming = False

            elif action.type == ActionType.DISCONNECT:
                if self._backend.disconnect():
                    self._disconnect()
                else:
                    logger.error('Failed to disconnect!')

            if self._is_streaming:
                self._action_queue.put(ActionRead(SlamdeckCommand.GET_DATA))

    def _make_packet(self, action: ActionWrite) -> bytes:
        packet = bytearray([action.command])
        if action.command == SlamdeckCommand.SET_SETTINGS:
            packet.extend(action.settings.to_bytes())
        return packet

    def _parse_response(self, action: Action, response: bytes) -> None:
        if action.command == SlamdeckCommand.GET_DATA:
            offset = 0
            data_size = self._data_size
            for sensor in self._sensors:
                data = response[offset:offset+data_size]
                data_size = len(data)
                if data:
                    try:
                        self._sensors[sensor].data = np.frombuffer(data, dtype=np.int16)
                        print(self._sensors[sensor].data[0])
                    except Exception as e:
                        logger.error(str(e))
                        self._is_streaming = False
                offset += data_size
        elif action.command == SlamdeckCommand.GET_SETTINGS:
            offset = 0
            for sensor in self._sensors:
                self._sensors[sensor].status = ord(response[offset:offset+1])
                offset += 1
            self._settings = SlamdeckSettings.from_bytes(response[offset:])
            print(self._settings)
        elif action.command == SlamdeckCommand.SET_SETTINGS:
            for i, sensor in enumerate(self._sensors):
                self._sensors[sensor].status = ord(response[i])
        elif action.command == SlamdeckCommand.START_STREAMING:
            if response[0] != SlamdeckCommand.START_STREAMING_ACK:
                logging.error('Expected START ACK, got: %d', response[0])
            else:
                self._is_streaming = True
                pass
        elif action.command == SlamdeckCommand.STOP_STREAMING:
            if response[0] != SlamdeckCommand.STOP_STREAMING_ACK:
                logging.error('Expected STOP ACK, got: %d', response[0])
            else:
                self._is_streaming = False
                pass

    def _create_sensors(self) -> t.Dict[SlamdeckSensorId, VL53L5CX]:
        return {
            SlamdeckSensorId.MAIN:  VL53L5CX(id=SlamdeckSensorId.MAIN.value),
            SlamdeckSensorId.FRONT: VL53L5CX(id=SlamdeckSensorId.FRONT.value),
            SlamdeckSensorId.RIGHT: VL53L5CX(id=SlamdeckSensorId.RIGHT.value),
            SlamdeckSensorId.BACK:  VL53L5CX(id=SlamdeckSensorId.BACK.value),
            SlamdeckSensorId.LEFT:  VL53L5CX(id=SlamdeckSensorId.LEFT.value)
        }


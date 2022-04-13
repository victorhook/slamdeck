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
from slamdeck_python.cpx_with_crtp import BackendCPXWithCrtp
from slamdeck_python.vl53l5cx import (VL53L5CX_PowerMode, VL53L5CX_Resolution,
                          VL53L5CX_RangingMode, VL53L5CX_Status,
                          VL53L5CX_TargetOrder, VL53L5CX)
from slamdeck_python.utils import Observable, Subscriber, CallbackHandler, Callback, CrazyflieModel
from time import time

logger = logging.getLogger(__file__)


""" Slamdeck API commands """

class SlamdeckCommand(IntEnum):
    GET_DATA            = 0
    GET_SETTINGS        = 1
    SET_SETTINGS        = 2
    START_STREAMING     = 3
    STOP_STREAMING      = 4

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

class Action(IntEnum):
    GET_DATA            = 0  # Get data requests the data, meaning we send data request, then we read.
    GET_SETTINGS        = 1
    SET_SETTINGS        = 2
    START_STREAMING     = 3
    STOP_STREAMING      = 4
    DISCONNECT          = 5
    READ_DATA           = 6  # Read data is used in streaming mode. Here we never write, only read incoming data.


class Slamdeck:

    """
        This class represents the Slamdeck.
        It implements the communiation protocol, and requires a backend driver
        to handle the communication with the actual slamdeck.
    """

    def __init__(self, backend: BackendCPX, parent_ui = None) -> None:
        self._backend = backend
        self.parent_ui = parent_ui
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

        self._cf = CrazyflieModel()
        if isinstance(backend, BackendCPXWithCrtp):
            backend.cf_logging_callback = self.cf_logging_callback

        # streaming
        self._is_streaming: bool = False
        self._streaming_rate = 0
        self._samples = 0
        self._t0 = time()

    def get_crazyflie(self) -> CrazyflieModel:
        return self._cf

    def get_streaming_rate(self) -> int:
        return self._streaming_rate

    def get_sensor(self, sensor: SlamdeckSensorId) -> VL53L5CX:
        return self._sensors.get(sensor, None)

    def set_settings(self, settings: SlamdeckSettings) -> None:
        self._settings = settings
        self._action_queue.put(Action.SET_SETTINGS)

    def is_connected(self) -> bool:
        return self._is_running.is_set()

    def get_data(self) -> None:
        logger.debug('Getting data...')
        self._action_queue.put(Action.GET_DATA)

    def start_streaming(self) -> None:
        if self._is_streaming:
            logger.error('Already streaming')
            return

        self._is_streaming = True
        self._action_queue.put(Action.START_STREAMING)

    def stop_streaming(self) -> None:
        self._is_streaming = False
        self._action_queue.put(Action.STOP_STREAMING)

    def disconnect(self) -> None:
        self._action_queue.put(Action.DISCONNECT)

    def connect(self) -> Thread:
        if self.is_connected():
            logger.warning('Already connected to slamdeck.')
            return

        def _connect_thread(self, runner_thread: Thread) -> None:
            self.connecting_handler.call()

            if not self._backend.connect():
                self.connection_error_handler.call('Failed to connect')
                logging.warning('Failed to connected to backend')
                return

            # We're connected
            self._is_streaming = False
            self.connected_handler.call()
            runner_thread.start()

        slamdeck_thread = Thread(target=self._run, name='Slamdeck', daemon=True)
        connect_thread = Thread(target=_connect_thread, args=(self, slamdeck_thread), daemon=True)
        connect_thread.start()
        return slamdeck_thread

    def cf_logging_callback(self, x: float, y: float, z: float, roll_degrees: float, pitch_degrees: float, yaw_degrees: float, vbat: float) -> None:
        self._cf.x = x
        self._cf.y = y
        self._cf.z = z
        self._cf.roll_degrees = roll_degrees
        self._cf.pitch_degrees = pitch_degrees
        self._cf.yaw_degrees = yaw_degrees
        self.parent_ui.vbat = vbat

    # --- API methods --- #
    def _get_data(self) -> None:
        self._backend.write(self._make_packet(SlamdeckCommand.GET_DATA))
        self._read_data()

    def _read_data(self) -> None:
        response = self._backend.read()
        if response is not None:
            #print(f'Read {len(response)} bytes')
            self._parse_get_data(response)
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
            logger.warning('Read data is None')
            self._is_streaming = False

    def _get_settings(self) -> None:
        self._backend.write(self._make_packet(SlamdeckCommand.GET_SETTINGS))
        response = self._backend.read()
        if response is None:
            logger.error('No response when getting settings')
            self._disconnect()
            return

        offset = 0
        for sensor in self._sensors:
            self._sensors[sensor].status = VL53L5CX_Status(response[offset])
            offset += 1
        self._settings = SlamdeckSettings.from_bytes(response[offset:])
        print(self._settings)

    def _set_settings(self) -> None:
        self._backend.write(self._make_packet(SlamdeckCommand.SET_SETTINGS, self._settings.to_bytes()))
        response = self._backend.read()
        if response is None:
            logger.error('No response when getting settings')
            return
        for i, sensor in enumerate(self._sensors):
            self._sensors[sensor].status = VL53L5CX_Status(response[i])

    def _start_streaming(self) -> None:
        self._backend.write(self._make_packet(SlamdeckCommand.START_STREAMING))
        self._is_streaming = True

    def _stop_streaming(self) -> None:
        self._backend.write(self._make_packet(SlamdeckCommand.STOP_STREAMING))
        self._is_streaming = False

    # --- Private methods --- #
    def _disconnect(self) -> None:
        if not self._backend.disconnect():
            logger.error('Error during disconnect!')
        logger.debug('Disconnected')
        self._is_running.clear()
        self._is_streaming = False
        self.disconnected_handler.call()

    def _clear_action_queue(self) -> None:
        self._action_queue = Queue()

    def _run(self) -> None:
        self._is_running.set()
        self._clear_action_queue()
        #self._get_settings()

        while self._is_running.is_set():
            action = self._action_queue.get()
            #print(action)

            if action == Action.DISCONNECT:
               self._disconnect()
               continue
            else:
                if action == Action.GET_DATA:
                    self._get_data()
                elif action == Action.READ_DATA:
                    self._read_data()
                elif action == Action.GET_SETTINGS:
                    self._get_settings()
                elif action == Action.SET_SETTINGS:
                    self._set_settings()
                elif action == Action.START_STREAMING:
                    self._start_streaming()
                elif action == Action.STOP_STREAMING:
                    self._stop_streaming()

            if self._is_streaming:
                self._action_queue.put(Action.READ_DATA)

    def _parse_get_data(self, response: bytes) -> None:
        offset = 0
        data_size = self._data_size
        for sensor in self._sensors:
            data = response[offset:offset+data_size]
            if data:
                try:
                    data_size = len(data)
                    self._sensors[sensor].data = np.frombuffer(data, dtype=np.uint16)
                    offset += data_size
                    #print(self._sensors[sensor].data[0])
                except Exception as e:
                    logger.error(str(e))
                    self._is_streaming = False

    def _print_sensor_data(self, sensor: SlamdeckSensorId) -> None:
        grid = 0
        row_size = 8
        for row in range(row_size):
            for col in range(row_size):
                print(self._sensors[sensor].data[grid], end=' ')
                grid += 1
            print("\n")
        print("\n")

    def _make_packet(self, command: SlamdeckCommand, data: bytes = None) -> bytes:
        packet = bytearray([command])
        if data is not None:
            packet.extend(data)
        return packet

    def _create_sensors(self) -> t.Dict[SlamdeckSensorId, VL53L5CX]:
        return {
            SlamdeckSensorId.MAIN:  VL53L5CX(id=SlamdeckSensorId.MAIN.value),
            SlamdeckSensorId.FRONT: VL53L5CX(id=SlamdeckSensorId.FRONT.value),
            SlamdeckSensorId.RIGHT: VL53L5CX(id=SlamdeckSensorId.RIGHT.value),
            SlamdeckSensorId.BACK:  VL53L5CX(id=SlamdeckSensorId.BACK.value),
            SlamdeckSensorId.LEFT:  VL53L5CX(id=SlamdeckSensorId.LEFT.value)
        }


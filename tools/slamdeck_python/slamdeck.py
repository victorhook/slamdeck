from dataclasses import dataclass, fields
from gzip import READ
from multiprocessing import Event
from pkgutil import get_data
from numpy import dtype, ndarray, uint16, uint32, uint8
from enum import IntEnum
import numpy as np
import struct
import typing as t
from threading import Thread, Timer
import logging

from queue import Queue
from slamdeck_python.cpx import BackendCPX
from slamdeck_python.cpx_with_crtp import BackendCPXWithCrtp
from slamdeck_python.vl53l5cx import (VL53L5CX_PowerMode, VL53L5CX_Resolution,
                          VL53L5CX_RangingMode, VL53L5CX_Status,
                          VL53L5CX_TargetOrder, VL53L5CX)
from slamdeck_python.utils import Observable, Subscriber, CallbackHandler, Callback, CrazyflieModel
from time import time

# Cflib
import cflib.crtp  # noqa
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper


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



class SlamdeckCommunicationMode(IntEnum):
    NRF = 0
    WIFI = 1


class SlamdeckChannel(IntEnum):
    INDICATE_NEW_FRAME = 0
    DATA               = 1

class SlamdeckCrtpApiState(IntEnum):
    NEW_FRAME = 0
    DATA      = 1

logging.basicConfig(level=logging.INFO)

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

        self._communication_mode: SlamdeckCommunicationMode = SlamdeckCommunicationMode.NRF
        self._action_queue = Queue() # Actions

        self._settings: SlamdeckSettings = None
        self._is_running = Event()

        self._data_size = 128
        self._expected_frame_size = self._data_size * 5

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
        if self._communication_mode == SlamdeckCommunicationMode.WIFI:
            self._action_queue.put(Action.DISCONNECT)
        elif self._communication_mode == SlamdeckCommunicationMode.NRF:
            self.cf.close_link()

    def connect(self, nrf_uri: str = None) -> Thread:
        print('he')
        if self.is_connected():
            logger.warning('Already connected to slamdeck.')
            return

        def _connect_thread(self, runner_thread: Thread) -> None:
            if not self._backend.connect():
                self.connection_error_handler.call('Failed to connect')
                logging.warning('Failed to connected to backend')
                return

            # We're connected
            self._connected()

        # Notify that we're connecting.
        self.connecting_handler.call()


        if self._communication_mode == SlamdeckCommunicationMode.WIFI:
            # Create background thread that runs forever and handles communication
            Thread(target=_connect_thread, daemon=True).start()

        elif self._communication_mode == SlamdeckCommunicationMode.NRF:
            # The nrf backend runs its own thread with cflib.
            uri = uri_helper.uri_from_env(default=f'radio://{nrf_uri}')
            self.cf = Crazyflie(rw_cache='./cache')
            self.cf.connected.add_callback(self._connected)
            self.cf.disconnected.add_callback(self._disconnected)
            self.cf.connection_failed.add_callback(self._connection_failed)
            self.cf.connection_lost.add_callback(self._connection_lost)

            CRTPPort.SLAMDECK = 0x09
            self.cf.add_port_callback(CRTPPort.SLAMDECK, self._incoming)

            # Data frames
            self.data = 0
            self._data = bytearray()
            self._frames = Queue()
            self._last_frame = None
            self.cf.open_link(uri)
            self._print_data()

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

            if self._communication_mode == SlamdeckCommunicationMode.WIFI:
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

            elif self._communication_mode == SlamdeckCommunicationMode.NRF:
                packet = CRTPPacket()
                packet.set_header(CRTPPort.SLAMDECK, 0)
                packet.data = (0,)
                self.cf.send_packet(packet)

    # -- NRF TMEP -- #
    def _print_data(self):
        print(f'Bad: {self.bad}  OK: {self.ok}')
        Timer(1, self._print_data).start()
        return
        if self._last_frame is not None:
            print(self.data, end=' ')
            print(' '.join(hex(a) for a in self._last_frame[:128]))
            self.data = 0

    ok = 0
    bad = 0

    def _finish_frame(self) -> None:
        frame_size = len(self._data)

        if frame_size == self._expected_frame_size:
            self.ok += 1

            single_sensor_data_size = frame_size // 5
            offset = 0
            #self._sensors[SlamdeckSensorId.MAIN].data = np.frombuffer(self._data[:128], dtype=uint16)
            for sensor in self._sensors.keys():
                data = self._data[offset:offset+single_sensor_data_size]
                data_u16 = np.frombuffer(data, dtype=np.uint16)
                self._sensors[sensor].data = data_u16
                offset += single_sensor_data_size
            self._last_frame = self._data.copy()
        else:
            #logger.warning(f'Got frame of size {frame_size}, expected: {self._expected_frame_size}')
            self.bad += 1

        self._data = bytearray()
        #self._print_sensor_data(SlamdeckSensorId.MAIN)

    def _incoming(self, packet: CRTPPacket):
        if packet.channel == SlamdeckChannel.INDICATE_NEW_FRAME:
            self._finish_frame()
        elif packet.channel == SlamdeckChannel.DATA:
            self._data.extend(packet.data)
    # --- #

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

        for row in reversed(range(row_size)):
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

    def _connected(self, link_uri) -> None:
        print(f'Connected to {link_uri}')
        self._is_streaming = False
        self.connected_handler.call()
        Thread(target=self._run, name='Slamdeck', daemon=True).start()

        print(self._communication_mode)
        if self._communication_mode == SlamdeckCommunicationMode.NRF:
        # The definition of the logconfig can be made before connecting
            self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
            self._lg_stab.add_variable('stateEstimate.x', 'float')
            self._lg_stab.add_variable('stateEstimate.y', 'float')
            self._lg_stab.add_variable('stateEstimate.z', 'float')
            self._lg_stab.add_variable('stabilizer.roll', 'float')
            self._lg_stab.add_variable('stabilizer.pitch', 'float')
            self._lg_stab.add_variable('stabilizer.yaw', 'float')
            # The fetch-as argument can be set to FP16 to save space in the log packet
            self._lg_stab.add_variable('pm.vbat', 'FP16')

            try:
                self.cf.log.add_config(self._lg_stab)
                # This callback will receive the data
                self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
                # This callback will be called on errors
                self._lg_stab.error_cb.add_callback(self._stab_log_error)
                # Start the logging
                self._lg_stab.start()
            except KeyError as e:
                print('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        self.cf_logging_callback(
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z'],
            data['stabilizer.roll'],
            data['stabilizer.pitch'],
            data['stabilizer.yaw'],
            0
        )
        #print(f'[{timestamp}][{logconf.name}]: ', end='')
        #for name, value in data.items():
        #    print(f'{name}: {value:3.3f} ', end='')
        #print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._is_running.clear()
        self.connection_error_handler.call()

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self._is_running.clear()
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self._is_running.clear()
        self.disconnected_handler.call()


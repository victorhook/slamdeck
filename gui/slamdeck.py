from enum import IntEnum
from cflib.utils.callbacks import Caller
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from dataclasses import dataclass
import logging
from queue import Queue
import typing as t
import numpy as np
import struct
from threading import Thread

from gui.data_link import DataLink, DataLinkNrfCRTP, DataLinkSocketTCP, DataLinkType
from gui.models import ModelVL53L5CX, ModelCrazyflie
from gui.models import (VL53L5CX_PowerMode, VL53L5CX_RangingMode,
                        VL53L5CX_Resolution, VL53L5CX_TargetOrder)
from gui import utils

logger = logging.getLogger(__name__)


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

class Action(IntEnum):
    DISCONNECT = 0
    GET_DATA = 1
    START_STREAMING = 2
    STOP_STREAMING = 3
    GET_SENSOR_SETTINGS = 4
    SET_SENSOR_SETTINGS = 5


@dataclass
class SlamdeckSettings:
    integration_time_ms:  np.uint32
    sharpener_percent:    np.uint8
    ranging_frequency_hz: np.uint8
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

class Slamdeck:

    def __init__(self,
            current_data_link: DataLinkType,
            sensors: t.List[ModelVL53L5CX],
            crazyflie: ModelCrazyflie,
            vbat = None
        ) -> None:
        self._model_senors = sensors
        self._model_cf = crazyflie
        self._vbat = vbat

        # Create Crazyflie object
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._disconnected)
        self._cf.connection_lost.add_callback(self._disconnected)

        self.cb_disconnected = Caller()
        self.cb_connecting = Caller()
        self.cb_connected = Caller()
        self.cb_error = Caller()

        self._data_link_nrf_crtp = DataLinkNrfCRTP(self._cf)
        self._data_link_socket_tcp = DataLinkSocketTCP('', 0)

        self._data_links = {
            DataLinkType.NRF_CRTP: self._data_link_nrf_crtp,
            DataLinkSocketTCP: self._data_link_socket_tcp
        }
        self._dlink = self.set_data_link(current_data_link)

        self._is_connected = False
        self._is_streaming = False

        self._bytes_per_sec = 0
        self._frame_per_sec = 0
        self._byte_counter = 0
        self._frame_counter = 0
        self.ui_timer = utils.start_timer(self._update_counters, 1)

        self._action_queue = Queue()

    def connect(self, uri: object) -> None:
        """
            Uri - [ip, port] for TCP
            Uri - NRF uri for nrf, crtp
        """
        self.cb_connecting.call()
        self._dlink.connect(uri)

    def disconnect(self) -> None:
        self._action_queue.put(Action.DISCONNECT)

    def get_data(self) -> None:
        self._action_queue.put(Action.GET_DATA)

    def start_streaming(self) -> None:
        self._action_queue.put(Action.START_STREAMING)

    def stop_streaming(self) -> None:
        self._action_queue.put(Action.STOP_STREAMING)

    def get_sensor_settings(self) -> None:
        self._action_queue.put(Action.GET_SENSOR_SETTINGS)

    def set_sensor_settings(self) -> None:
        self._action_queue.put(Action.SET_SENSOR_SETTINGS)

    def get_frame_rate(self) -> int:
        return self._frame_per_sec

    def get_data_rate(self) -> int:
        return self._bytes_per_sec

    def set_data_link(self, data_link: DataLinkType) -> DataLink:
        self._dlink = self._data_links[data_link]
        return self._dlink

    def is_connected(self) -> bool:
        return self._is_connected

    def _run(self) -> None:
        while self._is_connected:
            action = self._action_queue.get()

            if action == Action.DISCONNECT:
                self._dlink.disconnect()
            elif action == Action.GET_DATA:
                data = self._dlink.read()
                if data is not None:
                    self._update_sensor_models(data)
                else:
                    print('No data')
            elif action == Action.START_STREAMING:
                self._dlink.write(b'0')
                self._is_streaming = True
            elif action == Action.STOP_STREAMING:
                self._is_streaming = False

            if self._is_streaming:
                self._action_queue.put(Action.GET_DATA)

    def _get_single_sensor_data_size(self) -> int:
        resolution = self._model_senors[0].resolution
        if resolution == VL53L5CX_Resolution.RESOLUTION_4X4:
            size = 64
        elif resolution == VL53L5CX_Resolution.RESOLUTION_8X8:
            size = 128
        else:
            size = 0
        return size

    def _get_frame_size(self) -> int:
        return self._get_single_sensor_data_size() * 5

    def _update_sensor_models(self, data: bytes) -> None:
        data_size = len(data)
        self._byte_counter += data_size

        if data_size != self._get_frame_size():
            logger.warning(f'Malformed frame: {data_size} bytes')
            return

        self._frame_counter += 1

        single_sensor_data_size = self._get_single_sensor_data_size()
        offset = 0
        for model in self._model_senors:
            model.data = np.frombuffer(data[offset:offset+single_sensor_data_size], dtype=np.uint16)
            offset += single_sensor_data_size

    def _update_counters(self) -> None:
        self._bytes_per_sec = self._byte_counter
        self._frame_per_sec = self._frame_counter
        self._frame_counter = 0
        self._byte_counter = 0

    def _add_cf_logging(self):
        logconf = LogConfig(name='Stabilizer', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stabilizer.roll', 'float')
        logconf.add_variable('stabilizer.pitch', 'float')
        logconf.add_variable('stabilizer.yaw', 'float')
        logconf.add_variable('pm.vbat', 'FP16')

        self._cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self._log_data)
        logconf.error_cb.add_callback(self._log_error)
        logconf.start()


    # -- Callbacks -- #
    def _disconnected(self, uri: str, msg: str = None) -> None:
        self.cb_disconnected.call()
        print(f'Disconnected: {uri}')
        if msg:
            print(msg)
        self._is_streaming = False
        self._is_connected = False

    def _connecting(self) -> None:
        self.cb_connecting.call()

    def _connected(self, uri: str) -> None:
        self._is_connected = True
        self.cb_connected.call()
        print(f'Connected to {uri}')

        print('Starting streaming directly!')
        self.start_streaming()

        # Attach logger to cf object.
        self._add_cf_logging()

        Thread(target=self._run, daemon=True).start()

    def _error(self) -> None:
        self.cb_error.call()

    def _log_error(self, logconf, msg):
        print(f'Error when logging {logconf.name}: {msg}')

    def _log_data(self, timestamp, data, logconf):
        self._model_cf.x = data['stateEstimate.x']
        self._model_cf.y = data['stateEstimate.y']
        self._model_cf.z = data['stateEstimate.z']
        self._model_cf.roll = data['stabilizer.roll']
        self._model_cf.pitch = data['stabilizer.pitch']
        self._model_cf.yaw = data['stabilizer.yaw']
        if self._vbat is not None:
            self._vbat(data["pm.vbat"])

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    sensors = [ModelVL53L5CX(id=id.value) for id in SlamdeckSensorId]
    cf = ModelCrazyflie()
    slamdeck = Slamdeck(DataLinkType.NRF_CRTP, sensors, cf)
    slamdeck.connect('radio://0/90/2M/E7E7E7E7E7')

    import time
    time.sleep(1)
    slamdeck.start_streaming()
    time.sleep(10)

    slamdeck.disconnect()
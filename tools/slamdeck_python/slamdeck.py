from dataclasses import dataclass
from numpy import dtype, ndarray, uint16, uint32, uint8
from backend import Backend, BackendParams, BackendFactory, Callback
from enum import IntEnum
import numpy as np
import struct
import typing as t
import logging


from utils import Observable, Subscriber


""" Slamdeck API commands """

class SlamdeckCommand(IntEnum):
    GET_SENSOR_STATUS = 0
    GET_DATA_FROM_SENSOR = 1
    GET_I2C_ADDRESS = 2
    SET_I2C_ADDRESS = 3
    GET_POWER_MODE = 4
    SET_POWER_MODE = 5
    GET_RESOLUTION = 6
    SET_RESOLUTION = 7
    GET_RANGING_FREQUENCY_HZ = 8
    SET_RANGING_FREQUENCY_HZ = 9
    GET_INTEGRATION_TIME_MS = 10
    SET_INTEGRATION_TIME_MS = 11
    GET_SHARPENER_PERCENT = 12
    SET_SHARPENER_PERCENT = 13
    GET_TARGET_ORDER = 14
    SET_TARGET_ORDER = 15
    GET_RANGING_MODE = 16
    SET_RANGING_MODE = 17

class SlamdeckResponse(IntEnum):
    RESULT_OK = 0

class VL53L5CX_Status(IntEnum):
    SENSOR_FAILED      = 0
    SENSOR_NOT_ENABLED = 1
    SENSOR_OK          = 2

class VL53L5CX_PowerMode(IntEnum):
    POWER_MODE_SLEEP  = 0
    POWER_MODE_WAKEUP = 1

class VL53L5CX_Resolution(IntEnum):
    RESOLUTION_4X4 = 16
    RESOLUTION_8X8 = 64

class VL53L5CX_TargetOrder(IntEnum):
    TARGET_ORDER_CLOSEST   = 1
    TARGET_ORDER_STRONGEST = 2

class VL53L5CX_RangingMode(IntEnum):
    RANGING_MODE_CONTINUOUS = 1
    RANGING_MODE_AUTONOMOUS = 3


@dataclass
class Sensor(Observable):
    """
        This class represents a single VL53L5CX sensor.
        It implements the observable pattern, which allows subscribers
        to subscribe to any change that happens to the sensor state.
    """
    id:                   int
    name:                 str
    data:                 np.ndarray           = None
    i2C_address:          uint8                = 0x00
    integration_time_ms:  uint32               = 0
    sharpener_percent:    uint8                = 0
    ranging_frequency_hz: uint8                = 0
    resolution:           VL53L5CX_Resolution  = 0
    power_mode:           VL53L5CX_PowerMode   = 0
    target_order:         VL53L5CX_TargetOrder = 0
    ranging_mode:         VL53L5CX_RangingMode = 0

    def __post_init__(self) -> None:
        super().__init__()



class Slamdeck(Subscriber):

    """
        This class represents the Slamdeck.
        It implements the communiation protocl, and requires a backend driver
        to handle the communication with the actual slamdeck.
    """

    def __init__(self, backend_params: BackendParams) -> None:
        self._backend: Backend = BackendFactory.get_backend(backend_params)
        self._sensors = self._create_sensors()

    def get_initial_sensor_settings(self, sensor: Sensor) -> None:
        self.get_sensor_status(sensor)
        self.get_i2c_address(sensor)
        self.get_integration_time_ms(sensor)
        self.get_power_mode(sensor)
        self.get_ranging_frequency_hz(sensor)
        self.get_ranging_mode(sensor)
        self.get_resolution(sensor)
        self.get_sharpener_percent(sensor)
        self.get_target_order(sensor)

    def _create_sensors(self) -> t.Dict[str, Sensor]:
        return {
            'main':  Sensor(id=1, name='main'),
            'front': Sensor(id=2, name='front'),
            'right': Sensor(id=3, name='right'),
            'back':  Sensor(id=4, name='back'),
            'left':  Sensor(id=5, name='left')
        }

    """ --- Commands --- """
    def get_sensor_status(self, sensor: Sensor) -> None:
        def _on_complete(status: VL53L5CX_Status):
            sensor.status = status
        self._cmd_execute(SlamdeckCommand.GET_SENSOR_STATUS, sensor.id, _on_complete, 1)

    # -- I2C address -- #
    def get_i2c_address(self, sensor: Sensor) -> None:
        def _on_complete(i2C_address: uint8):
            sensor.i2C_address = i2C_address
        self._cmd_execute(SlamdeckCommand.GET_I2C_ADDRESS, sensor.id, _on_complete, 1)

    def set_i2c_address(self, sensor: Sensor, address: uint8) -> None:
        if address > 127:
            logging.error(f'Can\t set i2c address of {address}. It must be less than 127.')
            return
        def _on_complete(response: SlamdeckResponse):
            if response == response.RESULT_OK:
                sensor.i2C_address = address
        self._cmd_execute(SlamdeckCommand.SET_I2C_ADDRESS, sensor.id, _on_complete, address, 1)

    # -- Power mode -- #
    def get_power_mode(self, sensor: Sensor) -> None:
        def _on_complete(power_mode: VL53L5CX_PowerMode):
            sensor.power_mode = power_mode
        self._cmd_execute(SlamdeckCommand.GET_POWER_MODE, sensor.id, _on_complete, 1)

    def set_power_mode(self, sensor: Sensor, mode: VL53L5CX_PowerMode) -> None:
        def _on_complete(response: SlamdeckResponse):
            if response == response.RESULT_OK:
                sensor.power_mode = mode
        self._cmd_execute(SlamdeckCommand.SET_POWER_MODE, sensor.id, _on_complete, 1, mode)

    # -- Resolution -- #
    def get_resolution(self, sensor: Sensor) -> None:
        def _on_complete(resolution: VL53L5CX_Resolution):
            sensor.resolution = resolution
        self._cmd_execute(SlamdeckCommand.GET_RESOLUTION, sensor.id, _on_complete, 1)

    def set_resolution(self, sensor: Sensor, resolution: VL53L5CX_Resolution) -> None:
        def _on_complete(response: SlamdeckResponse):
            if response == response.RESULT_OK:
                sensor.resolution = resolution
        self._cmd_execute(SlamdeckCommand.SET_RESOLUTION, sensor.id, _on_complete, 1, resolution)

    # -- Ranging -- #
    def get_ranging_frequency_hz(self, sensor: Sensor) -> None:
        def _on_complete(ranging_frequency_hz: uint8):
            sensor.ranging_frequency_hz = ranging_frequency_hz
        self._cmd_execute(SlamdeckCommand.GET_RANGING_FREQUENCY_HZ, sensor.id, _on_complete, 1)

    def set_ranging_frequency_hz(self, sensor: Sensor, ranging_frequency_hz: uint8) -> None:
        def _on_complete(response: SlamdeckResponse):
            if response == response.RESULT_OK:
                sensor.ranging_frequency_hz = ranging_frequency_hz
        self._cmd_execute(SlamdeckCommand.SET_RANGING_FREQUENCY_HZ, sensor.id, _on_complete, 1, ranging_frequency_hz)

    # -- Integration time -- #
    def get_integration_time_ms(self, sensor: Sensor) -> None:
        def _on_complete(integration_time_ms: uint32):
            sensor.integration_time_ms = integration_time_ms
        self._cmd_execute(SlamdeckCommand.GET_INTEGRATION_TIME_MS, sensor.id, _on_complete, 1)

    def set_integration_time_ms(self, sensor: Sensor, integration_time_ms: uint32) -> None:
        def _on_complete(integration_time_ms: uint8):
            sensor.integration_time_ms = integration_time_ms
        self._cmd_execute(SlamdeckCommand.SET_INTEGRATION_TIME_MS, sensor.id, _on_complete, 1, integration_time_ms)

    # -- Sharpener precent -- #
    def get_sharpener_percent(self, sensor: Sensor) -> None:
        def _on_complete(sharpener_percent: uint8):
            sensor.sharpener_percent = sharpener_percent
        self._cmd_execute(SlamdeckCommand.GET_SHARPENER_PERCENT, sensor.id, _on_complete, 1)

    def set_sharpener_percent(self, sensor: Sensor, sharpener_percent: uint8) -> None:
        def _on_complete(sharpener_percent: uint8):
            sensor.sharpener_percent = sharpener_percent
        self._cmd_execute(SlamdeckCommand.SET_SHARPENER_PERCENT, sensor.id, _on_complete, 1, sharpener_percent)

    # -- Target order -- #
    def get_target_order(self, sensor: Sensor) -> None:
        def _on_complete(target_order: uint8):
            sensor.target_order = target_order
        self._cmd_execute(SlamdeckCommand.GET_TARGET_ORDER, sensor.id, _on_complete, 1)

    def set_target_order(self, sensor: Sensor, target_order: uint8) -> None:
        def _on_complete(target_order: uint8):
            sensor.target_order = target_order
        self._cmd_execute(SlamdeckCommand.SET_TARGET_ORDER, sensor.id, _on_complete, 1, target_order)

    # -- Ranging mode -- #
    def get_ranging_mode(self, sensor: Sensor) -> None:
        def _on_complete(ranging_mode: VL53L5CX_RangingMode):
            sensor.ranging_mode = ranging_mode
        self._cmd_execute(SlamdeckCommand.GET_RANGING_MODE, sensor.id, _on_complete, 1)

    def set_ranging_mode(self, sensor: Sensor, ranging_mode: VL53L5CX_RangingMode) -> None:
        def _on_complete(ranging_mode: uint8):
            sensor.ranging_mode = ranging_mode
        self._cmd_execute(SlamdeckCommand.SET_RANGING_MODE, sensor.id, _on_complete, 1, ranging_mode)

    # -- Get data -- #
    def get_data_from_sensor(self, sensor: Sensor) -> None:
        def _on_complete(data: bytes):
            sensor.data = np.frombuffer(data, dtype=np.uint16)
        self._cmd_execute(SlamdeckCommand.GET_DATA_FROM_SENSOR, sensor.id, _on_complete, sensor.resolution)

    def connect(self) -> None:
        self._backend.start()

    def disconnect(self) -> None:
        self._backend.stop()

    """ --- Callbacks --- """
    def add_cb_connecting(self, callback: Callback) -> None:
        self._backend.cb_connecting.add_callback(callback)

    def add_cb_connected(self, callback: Callback) -> None:
        self._backend.cb_connected.add_callback(callback)

    def add_cb_disconnected(self, callback: Callback) -> None:
        self._backend.cb_disconnected.add_callback(callback)

    def add_cb_connection_error(self, callback: Callback) -> None:
        self._backend.cb_connection_error.add_callback(callback)

    def add_cb_on_new_data(self, callback: Callback) -> None:
        self._backend.cb_on_new_data.add_callback(callback)

    def _cmd_execute(self,
                     type: SlamdeckCommand,
                     sensor: Sensor,
                     on_complete: callable = None,
                     bytes_to_read: uint32 = 0,
                     data: uint8 = b'0x00'
            ) -> None:
        if not self._backend.is_running():
            logging.error('Slamdeck not connected yet, can\'t issue commands.')
            return

        cmd = struct.pack_into('BBB', type, sensor, data)
        self._backend.write(cmd, bytes_to_read, on_complete)

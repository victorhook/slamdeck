from dataclasses import dataclass
from numpy import dtype, ndarray, uint16, uint32, uint8
from enum import IntEnum
import numpy as np
import struct
import typing as t
import logging

from slamdeck_python.backend import Backend, BackendParams, BackendFactory, Callback
from slamdeck_python.slamdeck_api import (SlamdeckApiPacket, VL53L5CX_PowerMode, VL53L5CX_Resolution,
                          VL53L5CX_RangingMode, VL53L5CX_Status,
                          VL53L5CX_TargetOrder, SlamdeckCommand,
                          SlamdeckResponse, SlamdeckSensor)
from slamdeck_python.utils import Observable, Subscriber


@dataclass
class Sensor(Observable):
    """
        This class represents a single VL53L5CX sensor.
        It implements the observable pattern, which allows subscribers
        to subscribe to any change that happens to the sensor state.
    """
    id:                   SlamdeckSensor
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



class Slamdeck:

    """
        This class represents the Slamdeck.
        It implements the communiation protocol, and requires a backend driver
        to handle the communication with the actual slamdeck.
    """

    def __init__(self, backend: Backend) -> None:
        self._backend = backend
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
            SlamdeckSensor.MAIN:  Sensor(id=SlamdeckSensor.MAIN),
            SlamdeckSensor.FRONT: Sensor(id=SlamdeckSensor.FRONT),
            SlamdeckSensor.RIGHT: Sensor(id=SlamdeckSensor.RIGHT),
            SlamdeckSensor.BACK:  Sensor(id=SlamdeckSensor.BACK),
            SlamdeckSensor.LEFT:  Sensor(id=SlamdeckSensor.LEFT)
        }

    def get_sensor(self, sensor: SlamdeckSensor) -> Sensor:
        return self._sensors.get(sensor, None)

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
        sensor = self.get_sensor(sensor)
        def _on_complete(data: bytes):
            print(f'RX: {len(data)} : {data}')
            sensor.data = np.frombuffer(data, dtype=np.uint16)
        self._cmd_execute(SlamdeckCommand.GET_DATA_FROM_SENSOR, sensor.id, _on_complete, 36)

    def connect(self) -> None:
        self._backend.start()

    def disconnect(self) -> None:
        self._backend.stop()

    def is_connected(self) -> bool:
        return self._backend.is_connected()

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
                     command: SlamdeckCommand,
                     sensor: Sensor,
                     on_complete: callable = None,
                     bytes_to_read: uint32 = 0,
                     data: uint8 = 0
            ) -> None:
        if not self._backend.is_connected():
            logging.error('Slamdeck not connected yet, can\'t issue commands.')
            return

        packet = SlamdeckApiPacket(
            command=command,
            sensor=sensor,
            data=data
        )

        self._backend.write(packet, bytes_to_read, on_complete)

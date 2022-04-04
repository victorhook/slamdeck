from dataclasses import dataclass, fields
from numpy import dtype, ndarray, uint16, uint32, uint8
from enum import IntEnum
import numpy as np
import struct
import typing as t
from threading import Thread
import logging

from slamdeck_python.backend import Backend, BackendParams, BackendFactory, Callback
from slamdeck_python.cpx import CPX_Packet
from slamdeck_python.slamdeck_api import (SlamdeckApiPacket, VL53L5CX_PowerMode, VL53L5CX_Resolution,
                          VL53L5CX_RangingMode, VL53L5CX_Status,
                          VL53L5CX_TargetOrder, SlamdeckCommand,
                          SlamdeckResponse, SlamdeckSensor)
from slamdeck_python.utils import Observable, Subscriber

logger = logging.getLogger()


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
    resolution:           VL53L5CX_Resolution  = VL53L5CX_Resolution.RESOLUTION_4X4
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

    def get_initial_sensor_settings(self, sensor: Sensor) -> Sensor:
        if not self.is_connected():
            logging.error('Not connected yet')
            return None
        self.get_i2c_address(sensor)
        self.get_integration_time_ms(sensor)
        self.get_power_mode(sensor)
        self.get_ranging_frequency_hz(sensor)
        self.get_ranging_mode(sensor)
        self.get_resolution(sensor)
        self.get_sharpener_percent(sensor)
        self.get_target_order(sensor)
        return sensor

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

    # --- Commands --- #

    # -- I2C address -- #
    def get_i2c_address(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_I2C_ADDRESS, sensor, 'i2C_address')

    def set_i2c_address(self, sensor: Sensor, i2C_address: uint8) -> None:
        if i2C_address > 127:
            logging.error(f'Can\t set i2c address of {i2C_address}. It must be less than 127.')
            return
        self._cmd_execute(SlamdeckCommand.SET_I2C_ADDRESS, sensor, 'i2C_address', i2C_address)

    # -- Power mode -- #
    def get_power_mode(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_POWER_MODE, sensor, 'power_mode')

    def set_power_mode(self, sensor: Sensor, power_mode: VL53L5CX_PowerMode) -> None:
        self._cmd_execute(SlamdeckCommand.SET_POWER_MODE, sensor, 'power_mode', power_mode)

    # -- Resolution -- #
    def get_resolution(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_RESOLUTION, sensor, 'resolution')

    def set_resolution(self, sensor: Sensor, resolution: VL53L5CX_Resolution) -> None:
        self._cmd_execute(SlamdeckCommand.SET_RESOLUTION, sensor, 'resolution', resolution)

    # -- Ranging -- #
    def get_ranging_frequency_hz(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_RANGING_FREQUENCY_HZ, sensor, 'ranging_frequency_hz')

    def set_ranging_frequency_hz(self, sensor: Sensor, ranging_frequency_hz: uint8) -> None:
        self._cmd_execute(SlamdeckCommand.SET_RANGING_FREQUENCY_HZ, sensor, 'ranging_frequency_hz', ranging_frequency_hz)

    # -- Integration time -- #
    def get_integration_time_ms(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_INTEGRATION_TIME_MS, sensor, 'integration_time_ms')

    def set_integration_time_ms(self, sensor: Sensor, integration_time_ms: uint32) -> None:
        self._cmd_execute(SlamdeckCommand.SET_INTEGRATION_TIME_MS, sensor, 'integration_time_ms', integration_time_ms)

    # -- Sharpener precent -- #
    def get_sharpener_percent(self, sensor: Sensor) -> None:
        print('get!')
        self._cmd_execute(SlamdeckCommand.GET_SHARPENER_PERCENT, sensor, 'sharpener_percent')

    def set_sharpener_percent(self, sensor: Sensor, sharpener_percent: uint8) -> None:
        self._cmd_execute(SlamdeckCommand.SET_SHARPENER_PERCENT, sensor, 'sharpener_percent', sharpener_percent)

    # -- Target order -- #
    def get_target_order(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_TARGET_ORDER, sensor, 'target_order')

    def set_target_order(self, sensor: Sensor, target_order: uint8) -> None:
        self._cmd_execute(SlamdeckCommand.SET_TARGET_ORDER, sensor, 'target_order', target_order)

    # -- Ranging mode -- #
    def get_ranging_mode(self, sensor: Sensor) -> None:
        self._cmd_execute(SlamdeckCommand.GET_RANGING_MODE, sensor, 'ranging_mode')

    def set_ranging_mode(self, sensor: Sensor, ranging_mode: VL53L5CX_RangingMode) -> None:
        self._cmd_execute(SlamdeckCommand.SET_RANGING_MODE, sensor, 'ranging_mode', ranging_mode)

    def get_data_from_sensor(self, sensor: SlamdeckSensor) -> None:
        # Resolution is in bytes, but data as uint16, so multiply by 2.
        #bytes_to_read = (sensor.resolution * 2) + SlamdeckApiPacket.HEADER_SIZE
        self._cmd_execute(SlamdeckCommand.GET_DATA, sensor, 'data')

    def connect(self) -> Thread:
        return self._backend.start()

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

    def _cmd_on_complete(self,
                             command: SlamdeckCommand,
                             sensor: SlamdeckSensor,
                             attribute: str,
                             value: object
            ) -> None:
        # Update the sensor model
        sensor = self._sensors[sensor]

        if command == SlamdeckCommand.GET_SENSOR_STATUS:
            pass
        elif command == SlamdeckCommand.GET_DATA:
            value = np.frombuffer(value, dtype=np.uint16)
        elif command == SlamdeckCommand.GET_I2C_ADDRESS:
            value = ord(value)
        elif command == SlamdeckCommand.GET_POWER_MODE:
            value = VL53L5CX_PowerMode(ord(value))
        elif command == SlamdeckCommand.GET_RESOLUTION:
            value = VL53L5CX_Resolution(ord(value))
        elif command == SlamdeckCommand.GET_RANGING_FREQUENCY_HZ:
            value = ord(value)
        elif command == SlamdeckCommand.GET_INTEGRATION_TIME_MS:
            value = struct.unpack('I', value)
        elif command == SlamdeckCommand.GET_SHARPENER_PERCENT:
            value = ord(value)
        elif command == SlamdeckCommand.GET_TARGET_ORDER:
            value = VL53L5CX_TargetOrder(ord(value))
        elif command == SlamdeckCommand.GET_RANGING_MODE:
            value = VL53L5CX_RangingMode(ord(value))

            """
            field_types = {field.name: field.type for field in fields(Sensor)}
            field_type = field_types.get(attribute, str)

            print(field_type, value)
            if len(value) == 1:
                value = ord(value)
            elif len(value) == 4:
                value = struct.unpack('I', value)
            """

        setattr(sensor, attribute, value)

        # Notify sensor subscribers
        sensor.notify_subscribers(attribute, value)

    def _cmd_execute(self,
                     command: SlamdeckCommand,
                     sensor: SlamdeckSensor,
                     attribute: str,
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

        def _on_complete(result):
            self._cmd_on_complete(command, sensor, attribute, result)

        self._backend.write(packet, _on_complete)

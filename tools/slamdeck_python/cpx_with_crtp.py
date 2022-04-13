
"""
This example illustrates how to use CRTP over CPX while also using CPX
to access other targets on the Crazyflie. This is done by connecting to
the Crazyflie and reading out images from the WiFi streamer and naming
them according to the post of the Crazyflie.

For the example to work you will need the WiFi example flashed on the
GAP8 and an up to date code-base on the STM32/ESP32.

"""

import logging
import time
import queue
import threading
import struct

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from cflib.cpx import CPXPacket, CPXFunction, CPXTarget

from slamdeck_python.cpx import BackendCPX
from slamdeck_python.transport_ip import TransportIp


logger = logging.getLogger(__name__)


class BackendCPXWithCrtp2(BackendCPX):

    def __init__(self, ip: str, port: int):
        self._backend: BackendCPX = None
        self._ip = ip
        self._port = port
        self._uri = uri_helper.uri_from_env(default=f'cpx://{ip}:{port}')

        cflib.crtp.init_drivers()

        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self.connected)
        self._cf.disconnected.add_callback(self.disconnected)

        self._transport = TransportIp(ip, port)
        super().__init__(self._transport)

        self.cf_logging_callback: callable = None

    def connect(self) -> bool:
        self._cf.open_link(self._uri)

        # Must freeze here to sync with caller.
        while not self._cf.is_connected():
            pass

        return True

    def disconnect(self) -> bool:
        self._cf.close_link()
        while self._cf.is_connected():
            pass
        return True

    def connected(self, uri):
        print("Connected to {}".format(uri))
        # Set the socket of transport. Superhacky
        self._transport._sock = self._cf.link.cpx._router._transport._socket

        try:
            self._cf.log.add_config(self._lg_stab)
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

    def disconnected(self, uri):
        print("Disconnected from {}".format(uri))

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
            data['stabilizer.yaw']
        )
        #print(f'[{timestamp}][{logconf.name}]: ', end='')
        #for name, value in data.items():
        #    print(f'{name}: {value:3.3f} ', end='')
        #print()




class BackendCPXWithCrtp(BackendCPX):

    def __init__(self, ip: str, port: int, cf: Crazyflie):
        self._backend: BackendCPX = None
        self._ip = ip
        self._port = port
        self._uri = uri_helper.uri_from_env(default=f'cpx://{ip}:{port}')

        cflib.crtp.init_drivers()

        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('pm.vbat', 'FP16')


        self._cf = cf
        self._cf.connected.add_callback(self.connected)
        self._cf.disconnected.add_callback(self.disconnected)

        self._transport = TransportIp(ip, port)
        super().__init__(self._transport)

        self.cf_logging_callback: callable = None

    def connect(self) -> bool:
        self._cf.open_link(self._uri)

        # Must freeze here to sync with caller.
        while not self._cf.is_connected():
            pass

        return True

    def disconnect(self) -> bool:
        self._cf.close_link()
        while self._cf.is_connected():
            pass
        return True


    def write(self, data: bytes) -> int:
        self._cpx.sendPacket(CPXPacket(
            destination=CPXTarget.ESP32,
            function=CPXFunction.APP,
            data=data
        ))

    def read(self) -> bytes:
        packet = self._cpx.receivePacket(CPXFunction.APP, 2000)
        if packet is not None:
            return packet.data

    def connected(self, uri):
        print("Connected to {}".format(uri))
        # Set the socket of transport. Superhacky
        self._cpx = self._cf.link.cpx

        try:
            self._cf.log.add_config(self._lg_stab)
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

    def disconnected(self, uri):
        print("Disconnected from {}".format(uri))

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
            data['pm.vbat']
        )

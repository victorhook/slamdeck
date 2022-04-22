from enum import IntEnum
from io import BytesIO
import logging
from queue import Queue
import time
import numpy as np
from threading import Timer, Thread

import cflib.crtp  # noqa
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper


logging.basicConfig(level=logging.ERROR)


class SlamdeckChannel(IntEnum):
    INDICATE_NEW_FRAME = 0
    DATA               = 1

class SlamdeckCrtpApiState(IntEnum):
    NEW_FRAME = 0
    DATA      = 1


class BackendNrf:

    data = 0

    def __init__(self, cf: Crazyflie):
        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.add_port_callback(CRTPPort.SLAMDECK, self._incoming)

        # Data frames
        self._data = bytearray()
        self._frames = Queue()
        self._last_frame = None

    def connect(self) -> None:
        # Try to connect to the Crazyflie
        self.cf.open_link(self._uri)

    def disconnect(self) -> None:
        # Try to connect to the Crazyflie
        self.cf.close_link()

    def _print_data(self):
        if self._last_frame is not None:
            print(self.data, end=' ')
            print(' '.join(hex(a) for a in self._last_frame[:128]))
            self.data = 0
        Timer(1, self._print_data).start()

    def _finish_frame(self) -> None:
        data_u16 = np.frombuffer(self._data, dtype=np.uint16)
        self._last_frame = self._data.copy()
        self._frames.put(data_u16)
        self._data = bytearray()

    def _incoming(self, packet: CRTPPacket):
        if packet.channel == SlamdeckChannel.INDICATE_NEW_FRAME:
            sensor_data_size = packet.data[0]
            self._finish_frame()
        elif packet.channel == SlamdeckChannel.DATA:
            self._data.extend(packet.data)

        self.data += len(packet._data)

    def _connected(self, link_uri):
        self.is_connected = True
        print(f'Connected to {link_uri}')
        self.run = True

        # Start a timer to disconnect in 10s
        Timer(20, self._close).start()

        def go():
            while self.run:
                packet = CRTPPacket()
                packet.set_header(CRTPPort.SLAMDECK, 0)
                packet.data = (0,)
                self.cf.send_packet(packet)

        Thread(target=go, daemon=True).start()

    def _close(self):
        self.run = False
        self.cf.close_link()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    slamdeck = SlamdeckExample(uri)

    # are just waiting until we are disconnected.
    while slamdeck.is_connected:
        time.sleep(1)

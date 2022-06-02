from abc import ABC, abstractmethod
from enum import IntEnum
import queue
import socket
from queue import Queue
from threading import Thread
from time import time

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
import cflib.crtp  # noqa

import logging

logger = logging.getLogger(__name__)


USE_CPX = True

if USE_CPX:
    from cpx import CPXPacket


class DataLinkType(IntEnum):
    NRF_CRTP   = 0
    SOCKET_TCP = 1


class DataLink(ABC):

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def read(self) -> bytes:
        pass

    @abstractmethod
    def write(self, bytes) -> int:
        pass


class DataLinkNrfCRTP(DataLink):

    LAST_PACKET_OF_FRAME = 1

    def __init__(self, cf: Crazyflie) -> None:
        cflib.crtp.init_drivers()
        self._cf = cf
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)

        if not hasattr(CRTPPort, 'CPX'):
            setattr(CRTPPort, 'CPX', 0x09)
        self._cf.add_port_callback(CRTPPort.CPX, self._incoming)

        self._tx = Queue()
        self._rx = Queue()
        self._is_connected = False

        self._rx_buf = bytearray()

    def connect(self, uri: str) -> None:
        if not uri.startswith('radio://'):
            uri = f'radio://{uri}'
        self._cf.open_link(uri)

    def disconnect(self) -> None:
        self._is_connected = False
        self._cf.close_link()

    def read(self) -> bytes:
        try:
            return self._rx.get(timeout=1)
        except queue.Empty:
            return None

    def write(self, data: bytes) -> int:
        packet = CRTPPacket()
        packet.set_header(CRTPPort.CPX, 0)
        packet.data = data
        self._tx.put(packet)

    b = 0
    t0 = 0
    a = 0

    def _incoming(self, packet: CRTPPacket):
        t = time()
        if self.t0 == 0:
            self.t0 = t

        if (t - self.t0) > 1:
            print(self.b)
            self.b = 0
            self.t0 = t

        if USE_CPX:
            #print(' '.join([bin(a)[2:].zfill(8) for a in packet.data[:2]]))
            cpx_packet = CPXPacket(raw_data=packet.data)
            #print(cpx_packet)

            self._rx_buf.extend(cpx_packet.data)
            self.b += len(packet.data)

            if cpx_packet.lastPacket:
                self._rx.put(self._rx_buf)
                self._rx_buf = bytearray()
        else:
            if packet.channel == self.LAST_PACKET_OF_FRAME:
                self._rx.put(self._rx_buf)
                self._rx_buf = bytearray()

    def _disconnected(self, uri: str) -> None:
        self._is_connected = False

    def _connected(self, uri: str) -> None:
        self._is_connected = True
        #if not USE_CPX:
        Thread(target=self._run, daemon=True).start()

    def _write(self) -> int:
        packet = self._tx.get()
        self._cf.send_packet(packet)

    def _run(self) -> None:
        while self._is_connected:
            self._write()
            #self.write(b'1')
            #self._write()


class DataLinkSocketTCP(DataLink):

    def __init__(self, ip: str, port: int) -> None:
        self._sock: socket.socket = None
        self._ip = ip
        self._port = port
        self._connect_timeout_seconds = 2

    def connect(self) -> None:
        if self._sock is not None:
            logger.error('Already connected')
            return False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self._connect_timeout_seconds)
        try:
            logger.debug(f'Trying to connect to {self._ip}:{self._port}')
            self._sock.connect(((self._ip, self._port)))
        except Exception as e:
            logger.error(f'Failed to connect to BackendIP: "{e}"')
            self._sock = None
            return False

        logger.info(f'Connected to {self._ip} on port {self._port}')
        self._sock.settimeout(None)
        return True

    def disconnect(self) -> bytes:
        if self._sock is None:
            logger.error('Already disconnected from socket')
            return False

        try:
            self._sock.shutdown(socket.SHUT_RDWR)
            self._sock.close()
            return True
        except OSError as e:
            logger.error(str(e))
            return False
        finally:
            self._sock = None

    def write(self, data: bytes) -> int:
        if self._sock is None:
            logger.error('Failed to write, not connected to socket')
            return 0

        self._sock.send(data)

    def read(self, size: int, timeout_ms: int = 2000) -> bytes:
        if self._sock is None:
            logger.error('Failed to read, not connected to socket')
            return None

        self._sock.settimeout(timeout_ms/1000)
        try:
            data = bytearray()
            while len(data) < size:
                data.extend(self._sock.recv(size - len(data)))
            return data
        except Exception as e:
            logger.warning(f'Connection reset by peer: {e}')
            return None
        finally:
            self._sock.settimeout(None)



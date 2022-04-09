import socket
import logging

from numpy import byte

from slamdeck_python.transport import Transport

logger = logging.getLogger()


class TransportIp(Transport):

    def __init__(self, ip: str, port: int) -> None:
        self._sock: socket.socket = None
        self._ip = ip
        self._port = port
        self._connect_timeout_seconds = 2

    def connect(self) -> bool:
        if self._sock is not None:
            logger.error('Already connected')
            return False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self._connect_timeout_seconds)
        try:
            logger.debug('Trying to connect')
            self._sock.connect(((self._ip, self._port)))
        except Exception as e:
            logger.error(f'Failed to connect to BackendIP: "{e}"')
            return False

        logger.info(f'Connected to {self._ip} on port {self._port}')
        self._sock.settimeout(None)
        return True

    def disconnect(self) -> bytes:
        if self._sock is None:
            logger.error('Already disconnected from socket')
            return False

        self._sock.close()
        self._sock = None
        return True

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






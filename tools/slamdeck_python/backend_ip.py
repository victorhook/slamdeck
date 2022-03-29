import socket
import logging


from slamdeck_python.backend import Backend

class BackendIp(Backend):

    def __init__(self, ip: str, port: int) -> None:
        super().__init__()
        self._sock: socket.socket = None
        self._ip = ip
        self._port = port

    def do_start(self) -> bool:
        if self._sock is not None:
            logging.error('Already connected')
            return False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._sock.connect(((self._ip, self._port)))
        except OSError as e:
            logging.debug(f'Failed to connect to IP backend: {e}')
            return False

        logging.debug(f'Connected to {self._ip} on port {self._port}')
        return True

    def do_stop(self) -> bytes:
        if self._sock is None:
            logging.error('Already disconnected from socket')
            return False

        self._sock.close()
        self._sock = None
        return True

    def do_write(self, data: bytes) -> int:
        if self._sock is None:
            logging.error('Failed to write, not connected to socket')
            return 0

        return self._sock.send(data)

    def do_read(self, size: int) -> bytes:
        if self._sock is None:
            logging.error('Failed to read, not connected to socket')
            return None

        try:
            return self._sock.recv(size)
        except ConnectionResetError as e:
            logging.warning(f'Connection reset by peer: {e}')
            return None





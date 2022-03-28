from backend import Backend
from serial import Serial
from serial.tools import list_ports
import logging


class BackendSerial(Backend):

    def __init__(self, baud: int, timeout_ms: int = 500, auto_select_port: bool = True) -> None:
        super().__init__()
        self._serial: Serial = None
        self._baud = baud
        self._timeout_ms = timeout_ms
        self._auto_select_port = auto_select_port

    def do_start(self) -> bool:
        if self._serial is not None:
            logging.error('Already connected')
            return False

        logging.debug('Scanning ports...')
        ports = list_ports.comports()

        if ports:
            logging.debug(f'Found folowing active ports: ', end='')
            logging.debug({'\n'.join(ports)})
        else:
            logging.error('No active ports found')
            return False

        if self._auto_select_port:
            logging.debug('Auto selecting serial port')
            port = ports[0]
        else:
            pass  #TODO Fix?

        self._serial = Serial(port, self._baud, timeout=self._timeout_ms)
        logging.debug(f'Opened port {port}')

        return True

    def do_stop(self) -> bytes:
        if self._serial is None:
            logging.error('Already disconnected from serial')
            return False

        self._serial.close()
        self._serial = None
        return True

    def write(self, data: bytes) -> int:
        if self._serial is None:
            logging.error('Failed to write, not connected to serial')
            return False

        return self._serial.write(data)

    def read(self, size: int) -> bytes:
        if self._serial is None:
            logging.error('Failed to read, not connected to serial')
            return False

        return self._serial.read(size)

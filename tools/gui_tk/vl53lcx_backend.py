from enum import IntEnum
from dataclasses import dataclass
import typing as t
from serial import Serial
from serial.tools import list_ports
from threading import Thread, Event


class Resolution(IntEnum):
    RES_4x4 = 4
    RES_8x8 = 8


@dataclass
class Reading:
    values: t.List[int]

@dataclass
class SensorParams:
    resolution: Resolution
    grid_size: 50

    # Constants
    MAX_RANGE: int = 4000



class SensorBackend(Thread):

    def __init__(self, baud: int, timeout_ms: int = 500, auto_select_port: bool = True) -> None:
        super().__init__(target=self._run, daemon=True)
        self._serial: Serial = None
        self._baud = baud
        self._timeout_ms = timeout_ms
        self._auto_select_port = auto_select_port
        self._running = Event()

    def start(self) -> None:
        if self._serial is not None:
            print('Already connected')
            return

        print('Scanning ports...')
        ports = list_ports.comports()
        if ports:
            print(f'Found folowing active ports: {"\n".join(ports)}')
        else:
            print('No active ports found')
            return

        if self._auto_select_port:
           port = ports[0]
        else:
            #TODO Fix?
            pass

        self._serial = Serial(port, self._baud, timeout=self._timeout_ms)
        print('Starting serial thread')
        super().start()

    def stop(self) -> None:
        self._

    def _run(self):
        pass
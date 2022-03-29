from abc import ABC, abstractmethod
from ast import Call
from dataclasses import dataclass
from enum import IntEnum
from PyQt5.QtCore import pyqtSignal
import typing as t
from threading import Thread, Event, currentThread
import logging
from queue import Queue

from slamdeck_python.utils import BinaryPacket


class Callback:

    def __init__(self, function: callable, args: tuple = tuple()) -> None:
        self._function = function
        self._args = args

    def __call__(self, *args, **kwargs) -> None:
        self._function(*args, *self._args, **kwargs)


class Callbacks:

    def __init__(self):
        self._callbacks: t.List[Callback] = []

    def add_callback(self, function: callable, args: tuple = tuple()) -> None:
        if type(function) is not Callback:
            cb = Callback(function, args)
        else:
            cb = function
        self._callbacks.append(cb)

    def __call__(self, *args, **kwargs) -> None:
        for callback in self._callbacks:
            callback(*args, **kwargs)


class Action:

    def __init__(self, data_to_write: bytes, bytes_to_read: int, on_complete: callable) -> None:
        self.data_to_write = data_to_write
        self.bytes_to_read = bytes_to_read
        self.on_complete = on_complete


class Backend(ABC):

    def __init__(self) -> None:
        self.cb_connecting = Callbacks()
        self.cb_connected = Callbacks()
        self.cb_disconnected = Callbacks()
        self.cb_connection_error = Callbacks()
        self.cb_on_new_data = Callbacks()
        self._is_running = Event()
        self._action_queue = Queue(maxsize=5)

    def start(self) -> Thread:
        if self._is_running.is_set():
            logging.warning('Backend already running')
            return None

        self._action_queue = Queue()
        thread = Thread(target=self._run, daemon=True)
        thread.start()
        return thread

    def stop(self) -> bool:
        logging.debug('Stopping backend thread')
        self._is_running.clear()
        if self.do_stop():
            self.cb_disconnected()
        else:
            self.cb_connection_error('Failure during disconnect')

    def write(self, data: BinaryPacket, bytes_to_read: int, on_complete: callable) -> None:
        if not self._is_running.is_set():
            logging.warning('Can\'t write data when not connected!')
            return
        self._action_queue.put(Action(
            data_to_write=data,
            bytes_to_read=bytes_to_read,
            on_complete=on_complete
        ))

    def is_connected(self) -> bool:
        return self._is_running.is_set()

    """ --- Abstract methods --- """
    @abstractmethod
    def do_start(self) -> bool:
        pass

    @abstractmethod
    def do_stop(self) -> bytes:
        pass

    @abstractmethod
    def do_write(self, data: bytes) -> int:
        pass

    @abstractmethod
    def do_read(self, size: int) -> bytes:
        pass

    def _run(self) -> None:
        # Emit callback that we're trying to connect
        self.cb_connecting()

        if not self.do_start():
            self.cb_connection_error('Failed to connect to backend')
            logging.error('Failed to connect to backend')
            return

        self.cb_connected()
        self._is_running.set()
        logging.debug('Successful connect to backend')
        logging.debug(f'Backend started in thread {currentThread().getName()}')

        while self._is_running.is_set():
            action: Action = self._action_queue.get()
            self._write(action.data_to_write)
            self._read(action.bytes_to_read, action.on_complete)

    def _write(self, packet: BinaryPacket) -> None:
        logging.debug(f'Writing {len(packet)} bytes')
        print(packet)
        return self.do_write(packet.as_bytes())

    def _read(self, size: int, on_complete: callable) -> None:
        if size == 0:
            logging.debug(f'0 bytes to read, not reading!')
            data = None
        else:
            data = self.do_read(size)
            if data is None:
                self.stop()
                return

            logging.debug(f'Reading {size} bytes: ' + \
                          ' '.join(f'{byte:02x} ' for byte in data))

        if on_complete is not None:
            on_complete(data)


class BackendParams:

    def __init__(self, cls: Backend, params: dict) -> None:
        self.backend_class = cls
        self.params = params


class BackendFactory:

    @staticmethod
    def get_backend(params: BackendParams) -> Backend:
        return params.backend_class(
            **params.params
        )

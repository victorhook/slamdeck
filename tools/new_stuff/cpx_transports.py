from abc import ABC, abstractmethod
import socket


class CPXTransport(ABC):

	@abstractmethod
	def __init__(self) -> None:
		pass

	@abstractmethod
	# Change to uri?
	def connect(host, port) -> None:
		pass

	@abstractmethod
	def disconnect() -> None:
		pass

	@abstractmethod
	def send(self, data) -> None:
		pass

	@abstractmethod
	def receive(self, size) -> None:
		pass


class SocketTransport(CPXTransport):

	def __init__(self, host, port):
		self._host = host
		self._port = port
		self.connect()

	def connect(self):
		print(f'Connecting to socket on {self._host}:{self.port}...')
		self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self._socket.connect((self._host, self._port))
		print('Connected')

	def disconnect(self):
		print('Closing transport')
		self._socket.close()
		self._socket.shutdown(0)
		self._socket = None

	def write(self, data):
		self._socket.send(data)

	def read(self, size):
		data = bytearray()
		while len(data) < size and self._socket is not None:
			data.extend(self._socket.recv(size-len(data)))
		return data


class CRTPTransport(CPXTransport):
	def __init__(self):
		print('CPX CRTP transport')

	# This connection will not really work...
	def connect(host, port):
		pass

	def disconnect():
		pass

	def send(self, data):
		pass

	def receive(self, size):
		pass

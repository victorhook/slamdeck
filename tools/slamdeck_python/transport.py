from abc import ABC, abstractmethod

class Transport(ABC):

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        pass

    @abstractmethod
    def write(self, data: bytes) -> int:
        pass

    @abstractmethod
    def read(self, size: int, timeout_ms: int) -> bytes:
        pass


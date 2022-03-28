from abc import ABC, abstractmethod
from dataclasses import dataclass
import typing as t


class Subscriber(ABC):

    @abstractmethod
    def on_change(self, value: object) -> None:
        pass


def subscriber(on_change: callable) -> Subscriber:
    sub = Subscriber()
    sub.on_change = on_change
    return sub

class Observable:

    def __init__(self) -> None:
        # Subscription is per attribute (string).
        self._subscribers: t.Dict[str, Subscriber] = {}

    def subscribe(self, subscriber: Subscriber, attribute: str) -> None:
        self._subscribers.get(attribute, []).append(subscriber)

    def __setattr__(self, name, value) -> None:
        super().__setattr__(name, value)
        for subscriber in self._subscribers.get(name, []):
            subscriber.on_change(value)


class BinaryPacket(ABC):
    @abstractmethod
    def as_bytes(self) -> bytes:
        pass

    @abstractmethod
    def __len__(self) -> int:
        pass


@dataclass
class SimplePacket(BinaryPacket):
    data: bytes

    def as_bytes(self) -> bytes:
        return self.data

    def __len__(self) -> bytes:
        return len(self.data)

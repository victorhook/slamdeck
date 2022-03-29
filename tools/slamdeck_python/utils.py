from abc import ABC, abstractmethod
from dataclasses import dataclass
import typing as t


class Subscriber:

    def on_change(self, value: object) -> None:
        pass

    def on_change_all(self, attribute: str, value: object) -> None:
        pass


def subscriber(on_change: callable) -> Subscriber:
    sub = Subscriber()
    sub.on_change = on_change
    return sub

class Observable:

    def __init__(self) -> None:
        # Subscription is per attribute (string).
        self._subscribers: t.Dict[str, Subscriber] = {}
        self._subscribers_all: t.List[Subscriber] = []

    def subscribe(self, subscriber: Subscriber, attribute: str) -> None:
        subs = self._subscribers.get(attribute, [])
        subs.append(subscriber)
        self._subscribers[attribute] = subs

    def subscribe_to_all(self, subscriber: Subscriber) -> None:
        self._subscribers_all.append(subscriber)

    def notify_subscribers(self, attribute: str, value: object) -> None:
        for subscriber in self._subscribers_all:
            subscriber.on_change_all(attribute, value)
        for subscriber in self._subscribers.get(attribute, []):
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

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

    def _exists_in_dict(self, subscriber: Subscriber) -> t.Tuple[bool, str]:
        for attr, sub in self._subscribers.items():
            if sub == subscriber:
                return True, attr
        return False, None

    def unsubscribe(self, subscriber: Subscriber) -> None:
        exists, attr = self._exists_in_dict(subscriber)
        if exists:
            self._subscribers.pop(attr)
            print('Unsubscribing!')
        try:
            self._subscribers_all.remove(subscriber)
        except ValueError as e:
            pass

    def subscribe(self, subscriber: Subscriber, attribute: str) -> None:
        subs = self._subscribers.get(attribute, [])
        if subscriber not in subs:
            subs.append(subscriber)
        else:
            print('already subscribed!')
        self._subscribers[attribute] = subs

    def subscribe_to_all(self, subscriber: Subscriber) -> None:
        if subscriber not in self._subscribers_all:
            self._subscribers_all.append(subscriber)
        else:
            print('already subscribed!')

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

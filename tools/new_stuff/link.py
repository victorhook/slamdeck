from new_stuff.cpx import CPXPacket


class Link:

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def write(self, packet: CPXPacket) -> None:
        pass

    def read(self) -> None:
        pass
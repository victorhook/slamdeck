import PyQt5.QtCore

from PyQt5.QtCore import QTimer


def start_timer(callback: callable, frequency: int) -> QTimer:
    timer = QTimer()
    timer.setInterval(int(1000 / frequency))
    timer.timeout.connect(callback)
    timer.start()
    return timer

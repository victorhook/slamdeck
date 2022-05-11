#!/usr/bin/env python3

import logging
import sys
from pathlib import Path

from PyQt5 import QtWidgets
from PyQt5 import uic

# Cflib
import cflib.crtp  # noqa

# Make python happy so we can import from gui module
#print()
sys.path.append(str(Path(__file__).parent.parent))
from gui.slamdeck_tab import SlamdeckTab


(main_window_class,
 main_windows_base_class) = uic.loadUiType(Path(__file__).parent.joinpath('ui/main.ui'))

logging.basicConfig(level=logging.INFO)


class MainUI(QtWidgets.QMainWindow, main_window_class):

    def __init__(self, *args):
        super(MainUI, self).__init__(*args)
        self.setupUi(self)

        self.slamdeck = SlamdeckTab(self.tabs, None)
        self.tabs.addTab(self.slamdeck, "Slamdeck")

        self._init()

    def _init(self):
        self.resize(1300, 900)


def main():
    """
    Check starting conditions and start GUI.

    First, check command line arguments and start loggers. Set log levels. Try
    all imports and exit verbosely if a library is not found. Disable outputs
    to stdout and start the GUI.
    """
    app = None

    cflib.crtp.init_drivers()

    # Connect ctrl-c (SIGINT) signal
    #signal.signal(signal.SIGINT, lambda sig, frame: handle_sigint(app))

    logging.basicConfig(level=logging.DEBUG)

    from main import MainUI
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QIcon
    import qdarkstyle

    app = QApplication(sys.argv)
    #app.setStyle("Fusion")
    app.setStyleSheet(qdarkstyle.load_stylesheet())

    app.setApplicationName("Slamdeck Proof of concept")
    main_window = MainUI()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

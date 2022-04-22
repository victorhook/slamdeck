import logging
import sys

from PyQt5 import QtWidgets
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QDir
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QAction
from PyQt5.QtWidgets import QActionGroup
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtGui import QPalette
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMenu
from PyQt5.QtWidgets import QMessageBox

from pathlib import Path

# Cflib
import cflib.crtp  # noqa

from slamdeck_tab import SlamdeckTab

(main_window_class,
 main_windows_base_class) = uic.loadUiType(Path(__file__).parent.joinpath('ui/main.ui'))



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

    from gui import MainUI
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QIcon

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    app.setApplicationName("Slamdeck Proof of concept")
    main_window = MainUI()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

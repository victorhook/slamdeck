import logging

from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot

logger = logging.getLogger(__name__)


class Tab(QtWidgets.QWidget):
    """Superclass for all tabs that implements common functions."""

    def __init__(self):
        super(Tab, self).__init__()
        self.tabName = "N/A"
        self.menuName = "N/A"
        self.enabled = True

    def getMenuName(self):
        """Return the name of the tab that will be shown in the menu"""
        return self.menuName

    def getTabName(self):
        """Return the name of the tab that will be shown in the tab"""
        return self.tabName


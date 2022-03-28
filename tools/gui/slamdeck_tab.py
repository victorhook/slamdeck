"""
An example template for a tab in the Crazyflie Client. It comes pre-configured
with the necessary QT Signals to wrap Crazyflie API callbacks and also
connects the connected/disconnected callbacks.
"""

import logging

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF
from PyQt5.QtWidgets import QMessageBox, QWidget, QGraphicsScene, QGraphicsView
from PyQt5 import Qt
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock
from pathlib import Path


from tab import Tab

logger = logging.getLogger(__name__)

example_tab_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]


class Sensor:

    GRID_SIZE = 80

    def __init__(self, graphics: QGraphicsView):
        self.graphics = graphics
        scene = QGraphicsScene()
        self.graphics.setScene(scene)
        pen = QPen(QColor(qRgb(172, 50, 99)))

        for i in range(4):
            for j in range(4):
                r = QRectF(QPointF(i*self.GRID_SIZE, j*self.GRID_SIZE),
                           QSizeF(self.GRID_SIZE, self.GRID_SIZE))
                scene.addRect(r, pen)
                text = scene.addText(f'{i} {j}')
                text.setPos(i*self.GRID_SIZE+self.GRID_SIZE/3, j*self.GRID_SIZE+self.GRID_SIZE/3)

class SlamdeckTab(Tab, example_tab_class):
    """Tab for plotting logging data"""

    _connected_signal = pyqtSignal(str)
    _disconnected_signal = pyqtSignal(str)
    _log_data_signal = pyqtSignal(int, object, object)
    _log_error_signal = pyqtSignal(object, str)
    _param_updated_signal = pyqtSignal(str, str)

    def __init__(self, tabWidget, helper, *args):
        super(SlamdeckTab, self).__init__(*args)
        self.setupUi(self)

        self.sensor = Sensor(self.graphics)

        self.tabName = "Example"
        self.menuName = "Example Tab"
        self.tabWidget = tabWidget

        self._helper = helper

        # Always wrap callbacks from Crazyflie API though QT Signal/Slots
        # to avoid manipulating the UI when rendering it
        self._connected_signal.connect(self._connected)
        self._disconnected_signal.connect(self._disconnected)
        self._log_data_signal.connect(self._log_data_received)
        self._param_updated_signal.connect(self._param_updated)


    def _connected(self, link_uri):
        """Callback when the Crazyflie has been connected"""

        logger.debug("Crazyflie connected to {}".format(link_uri))

    def _disconnected(self, link_uri):
        """Callback for when the Crazyflie has been disconnected"""

        logger.debug("Crazyflie disconnected from {}".format(link_uri))

    def _param_updated(self, name, value):
        """Callback when the registered parameter get's updated"""

        logger.debug("Updated {0} to {1}".format(name, value))

    def _log_data_received(self, timestamp, data, log_conf):
        """Callback when the log layer receives new data"""

        logger.debug("{0}:{1}:{2}".format(timestamp, log_conf.name, data))

    def _logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""

        QMessageBox.about(self, "Example error",
                          "Error when using log config"
                          " [{0}]: {1}".format(log_conf.name, msg))

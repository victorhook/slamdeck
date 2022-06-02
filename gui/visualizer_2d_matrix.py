from PyQt5.QtCore import pyqtSignal, QRectF, QObject, QTimer
from PyQt5.QtWidgets import (QGraphicsScene, QGraphicsView,
                            QGraphicsTextItem, QGraphicsRectItem)
from PyQt5.QtGui import QColor, QBrush

from enum import IntEnum
from matplotlib.colors import LinearSegmentedColormap, rgb2hex
from dataclasses import dataclass
import typing as t

from models import ModelVL53L5CX, VL53L5CX_Resolution



class SensorDimension(IntEnum):
    DIMENSION_4X4 = 4
    DIMENSION_8X8 = 8

N = 4000
cmap = LinearSegmentedColormap.from_list('r',["r", "w", "g"], N=N)


class Visualizer2dMatrix(QObject):

    _s_update_ui = pyqtSignal()
    _s_create_grids = pyqtSignal()
    FPS = 30

    colors = [QBrush(QColor(rgb2hex(cmap(value)))) for value in range(N+1)]

    def __init__(self, graphics: QGraphicsView, sensor: ModelVL53L5CX, small: bool = False):
        QObject.__init__(self)
        self.graphics = graphics
        self.sensor = sensor

        # TODO: Check if this makes difference
        self.graphics.setInteractive(False)
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self._is_small = small

        self._s_create_grids.connect(self._create_grids)

        # Add subscriptions
        self.grid_per_row: int = 0
        self.set_sensor(sensor)

        self._graph_timer = QTimer()
        self._graph_timer.setInterval(int(1000 / self.FPS))
        self._graph_timer.timeout.connect(self._update_graphics)
        self._graph_timer.start()

    def set_sensor(self, sensor: ModelVL53L5CX):
        self.sensor = sensor
        if self.sensor.resolution == VL53L5CX_Resolution.RESOLUTION_4X4:
            self.grid_per_row = 4
        else:
            self.grid_per_row = 8
        self._s_create_grids.emit()

    def _constraint_value(self, value) -> int:
        if value > N:
            value = N
        return value

    def _update_graphics(self) -> None:
        g = 0
        distances = self.sensor.data

        for r in range(self.grid_per_row):
            for c in range(self.grid_per_row):
                grid = self.grids[self.grid_per_row-1-r][self.grid_per_row-1-c]
                value = distances[g]
                color_value = self._constraint_value(value)
                grid.rect.setBrush(self.colors[color_value])
                #grid.text.setPlainText(str(color_value))
                g += 1

    def _create_grids(self) -> t.List[t.List['Grid']]:
        self.scene.clear()
        self.grids: t.List[t.List['Grid']] = []

        if self.grid_per_row == SensorDimension.DIMENSION_4X4:
            grid_size = self.graphics.width() // 4
        else:
            grid_size = self.graphics.width() // 8

        for r in range(self.grid_per_row):
            self.grids.append([])
            for c in range(self.grid_per_row):
                rect = QGraphicsRectItem(c*grid_size, r*grid_size,
                                         grid_size, grid_size)
                rect.setBrush(self.colors[0])
                self.scene.addItem(rect)
                #text = self.scene.addText(f'{r} {c}')
                text = self.scene.addText(f'')
                text.setPos(c*grid_size+grid_size/3, r*grid_size+grid_size/3)

                self.grids[r].append(self.Grid(rect, text))

        return self.grids

    @dataclass
    class Grid:
        rect: QRectF
        text: QGraphicsTextItem


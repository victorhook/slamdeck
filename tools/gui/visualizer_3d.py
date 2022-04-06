"""
Shows data for the Loco Positioning system
"""

import logging
from enum import Enum
from collections import namedtuple

import time
from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QLabel
from slamdeck_python.slamdeck import Sensor, Slamdeck
from slamdeck_python.slamdeck_api import SlamdeckSensor
import math
from  matplotlib.colors import LinearSegmentedColormap, rgb2hex
from timeit import default_timer as timer

from tab import Tab

from pathlib import Path

from vispy import scene
import numpy as np

import copy
import typing as t


logger = logging.getLogger(__name__)
example_tab_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]

STYLE_RED_BACKGROUND = "background-color: lightpink;"
STYLE_GREEN_BACKGROUND = "background-color: lightgreen;"
STYLE_NO_BACKGROUND = "background-color: none;"


class Anchor:
    def __init__(self, x=0.0, y=0.0, z=0.0, distance=0.0):
        self.x = x
        self.y = y
        self.z = z
        self._is_position_valid = False
        self._is_active = False
        self.distance = distance

    def set_position(self, position):
        """Sets the position."""
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self._is_position_valid = True

    def get_position(self):
        """Returns the position as a vector"""
        return (self.x, self.y, self.z)

    def is_position_valid(self):
        return self._is_position_valid

    def set_is_active(self, is_active):
        self._is_active = is_active

    def is_active(self):
        return self._is_active


class AxisScaleStep:
    def __init__(self, from_view, from_axis, to_view, to_axis,
                 center_only=False):
        self.from_view = from_view.view
        self.from_axis = from_axis
        self.to_view = to_view.view
        self.to_axis = to_axis
        self.center_only = center_only


class Visualizer3d(scene.SceneCanvas):
    ANCHOR_BRUSH = np.array((0.2, 0.5, 0.2))
    ANCHOR_BRUSH_INVALID = np.array((0.8, 0.5, 0.5))
    HIGHLIGHT_ANCHOR_BRUSH = np.array((0, 1, 0))
    POSITION_BRUSH = np.array((0, 0, 1.0))

    COLOR_POINT_CLOUD = np.array((0, 1.0, 0, 0))

    VICINITY_DISTANCE = 2.5
    HIGHLIGHT_DISTANCE = 0.5

    LABEL_SIZE = 100
    LABEL_HIGHLIGHT_SIZE = 200

    ANCHOR_SIZE = 10
    HIGHLIGHT_SIZE = 20

    FPS = 60
    FOV = 45
    MAX_DISTANCE_CM = 300
    cmap = LinearSegmentedColormap.from_list('rg',["r", "w", "g"], N=MAX_DISTANCE_CM)

    TEXT_OFFSET = np.array((0.0, 0, 0.25))

    def __init__(self, slamdeck: Slamdeck):
        scene.SceneCanvas.__init__(self, keys=None)
        self.unfreeze()

        self._view = self.central_widget.add_view()
        self._view.bgcolor = '#ffffff'
        self._view.camera = scene.TurntableCamera(
            fov=80,
            distance=10.0,
            up='+z',
            center=(0.0, 0.0, 1.0))

        self._cf_pos = np.array([[0, 0, 0]])
        self._cf = scene.visuals.Markers(
            pos=self._cf_pos,
            parent=self._view.scene,
            face_color=self.POSITION_BRUSH
        )
        self._anchor_contexts = {}

        """
        plane_size = 10
        scene.visuals.Plane(
            width=plane_size,
            height=plane_size,
            width_segments=plane_size,
            height_segments=plane_size,
            color=(0.5, 0.5, 0.5, 0.5),
            edge_color="gray",
            parent=self._view.scene)
        """

        self.addArrows(1, 0.02, 0.1, 0.1, self._view.scene)

        self._sensor_directions = {
            SlamdeckSensor.MAIN.value:  (np.array([1, 0, 0]), np.array([0, 1, 0])),
            SlamdeckSensor.FRONT.value: (1,  0,  0),
            SlamdeckSensor.RIGHT.value: (0,  1,  1),
            SlamdeckSensor.BACK.value:  (-1, 0,  0),
            SlamdeckSensor.LEFT.value:  (0,  -1, 0)
        }

        # Subscribe to models
        self.slamdeck = slamdeck
        self.sensor_front = self.slamdeck.get_sensor_model(SlamdeckSensor.BACK)

        # Set grid size and pixel offset
        self.grids = self.sensor_front.resolution.value
        self.grid_size = int(np.sqrt(self.grids))
        self.offset = (self.grid_size / 2)
        self.grid_fov = self.FOV / self.grid_size # How big FoV a single grid covers
        self.grid_pad = 1

        self.coordinates, self.colors, self.point_cloud = self._create_point_cloud()

        self._graph_timer = QTimer()
        self._graph_timer.setInterval(int(1000 / self.FPS))
        self._graph_timer.timeout.connect(self._update_graphics)
        self._graph_timer.start()

        self.t0 = time.time()

    def _get_color(self, distance: float) -> np.array:
        return self.cmap(distance)

    def _set_coordinate(self,
                        distance: float,
                        row: int,
                        col: int,
                        coordinate: np.array
            ) -> np.array:
        x_axis = np.array([1, 0, 0])

        grid_pad = 1 / self.grid_size

        rot = np.array([
            1,
            grid_pad * ( col - (self.grid_size/2)+0.5 ),
            grid_pad * ( row - (self.grid_size/2)+0.5 )
        ])
        rot /= np.linalg.norm(rot)

        # Scale distance
        distance_scaled = distance / 1000
        rot *= distance_scaled

        if distance_scaled > self.MAX_DISTANCE_CM:
            #return np.array([0, 0, 0])
            pass
            # TODO: THis

        #coordinate = self._rpy_to_rot(np.array([0, 0, 0])) @ rot + self._cf_pos
        #print(rot)
        rot = ( self._rpy_to_rot(np.array([0, 0, 0])) @ rot) + self._cf_pos
        rot = rot[0]
        print(np.linalg.norm(rot))

        #  x: col - horizontal
        #  y: row - vertical
        #  z: distance - depth
        coordinate[0] = rot[0] #col - self.offset
        coordinate[1] = rot[1] #+ self.offset #distance
        coordinate[2] = rot[2] #row

        #print(coordinate)


    def _create_point_cloud(self) -> t.Dict[SlamdeckSensor, scene.visuals.Markers]:
        coordinates = np.ndarray((5, self.grids, 3))
        colors = np.ndarray((5, self.grids, 4))

        for sensor in [self.sensor_front]:
            self._fill_sensor_coordinates(sensor, coordinates[sensor.id], colors[sensor.id])

        point_cloud =  scene.visuals.Markers(
            pos=np.array(coordinates[SlamdeckSensor.BACK]),
            parent=self._view.scene,
            face_color=colors[SlamdeckSensor.BACK]
        )

        self.point_cloud = point_cloud
        self.coordinates = coordinates
        return coordinates, colors, point_cloud

    def _fill_sensor_coordinates(self, sensor: Sensor, coordinates_to_fill: np.ndarray, colors_to_fill: np.ndarray):
        grid = 0
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                old_coordinate = coordinates_to_fill[grid]
                self._set_coordinate(sensor.data[grid], self.grid_size-1-row, col, old_coordinate)
                colors_to_fill[grid] = self._get_color(sensor.data[grid])
                grid += 1

    def _update_point_cloud(self, sensor: SlamdeckSensor, points: np.ndarray) -> None:
        self.point_cloud[sensor]

    def _rotate_point_cloud(self, sensor: SlamdeckSensor, points: np.ndarray) -> None:
        np.rot90(points, 1, self._sensor_directions[sensor.value])

    k = 0

    def _update_graphics(self):
        now = time.time()
        self.k += 1

        dt = now - self.t0
        if dt > 1:
            print('FPS', self.k)
            self.t0 = now
            self.k = 0


        grids = self.sensor_front.resolution.value
        grid_size = int(np.sqrt(grids))

        coordinates_front = self.coordinates[SlamdeckSensor.BACK]
        colors = self.colors[SlamdeckSensor.BACK]
        t0 = time.time()
        self._fill_sensor_coordinates(self.sensor_front, coordinates_front, colors)
        #print(f'{(time.time() - t0)*1000:.3} ms', end=' ')
        t0 = time.time()
        self.point_cloud.set_data(pos=coordinates_front, face_color=colors)
        #print(f'{(time.time() - t0)*1000:.3} ms')

    def _rpy_to_rot(self, rpy: np.array) -> np.array:
        # http://planning.cs.uiuc.edu/node102.html
        # Pitch reversed compared to page above
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        cg = math.cos(roll)
        cb = math.cos(-pitch)
        ca = math.cos(yaw)
        sg = math.sin(roll)
        sb = math.sin(-pitch)
        sa = math.sin(yaw)

        r = [
            [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
            [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
            [-sb, cb * sg, cb * cg],
        ]

        return np.array(r)

    def addArrows(self, length, width, head_length, head_width, parent):
        # The Arrow visual in vispy does not seem to work very good,
        # draw arrows using lines instead.
        w = width / 2
        hw = head_width / 2
        base_len = length - head_length

        # X-axis
        scene.visuals.LinePlot([
            [0, w, 0],
            [base_len, w, 0],
            [base_len, hw, 0],
            [length, 0, 0],
            [base_len, -hw, 0],
            [base_len, -w, 0],
            [0, -w, 0]],
            width=1.0, color='red', parent=parent, marker_size=0.0)

        # Y-axis
        scene.visuals.LinePlot([
            [w, 0, 0],
            [w, base_len, 0],
            [hw, base_len, 0],
            [0, length, 0],
            [-hw, base_len, 0],
            [-w, base_len, 0],
            [-w, 0, 0]],
            width=1.0, color='green', parent=parent, marker_size=0.0)

        # Z-axis
        scene.visuals.LinePlot([
            [0, w, 0],
            [0, w, base_len],
            [0, hw, base_len],
            [0, 0, length],
            [0, -hw, base_len],
            [0, -w, base_len],
            [0, -w, 0]],
            width=1.0, color='blue', parent=parent, marker_size=0.0)

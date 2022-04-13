"""
Shows data for the Loco Positioning system
"""

from dataclasses import dataclass
import logging
from enum import Enum
from collections import namedtuple
from re import X

import time
from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from slamdeck_python.slamdeck import Slamdeck, SlamdeckSensorId, VL53L5CX
import math
from  matplotlib.colors import LinearSegmentedColormap, rgb2hex
from timeit import default_timer as timer

from vispy import scene
import numpy as np

import copy
import typing as t

from slamdeck_python.utils import time_function, CrazyflieModel


logger = logging.getLogger(__name__)

class RotationMatrix:
    FRONT = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    MAIN = np.array([
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0]
    ])
    LEFT = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    BACK = -FRONT
    RIGHT = -LEFT

@dataclass
class OrientationArrow:
    base_arrow: np.ndarray
    x: scene.visuals.LinePlot
    y: scene.visuals.LinePlot
    z: scene.visuals.LinePlot


class Visualizer3d(scene.SceneCanvas):
    COLOR_POINT_CLOUD = np.array((0, 1.0, 0, 0))
    COLOR_CRAZYFLIE = np.array((1.0, 1.0, 0, 0))
    FPS = 60
    FOV = 45
    MAX_DISTANCE = 4000
    OUT_OF_BOUNDS = 4000

    WORLD_IS_M = True

    cmap = LinearSegmentedColormap.from_list('r',["r", "w", "g"], N=MAX_DISTANCE)

    def __init__(self, slamdeck: Slamdeck):
        scene.SceneCanvas.__init__(self, keys=None)
        self.unfreeze()

        self._view = self.central_widget.add_view()
        self._view.bgcolor = '#ffffff'
        self._view.camera = scene.TurntableCamera(
            fov=45,
            elevation=45,
            azimuth=-80,
            distance=5.0,
            up='+z',
            center=(0.0, 0.0, 0.0))

        self._cf = slamdeck.get_crazyflie()
        self._cf_line_size = 0.5
        self._cf_pos = np.array([self._cf.x, self._cf.y, self._cf.z])
        self._cf_marker = scene.visuals.Markers(
            pos=np.array([self._cf_pos]),
            parent=self._view.scene,
            face_color=self.COLOR_CRAZYFLIE
        )

        plane_size = 10
        scene.visuals.Plane(
            width=plane_size,
            height=plane_size,
            width_segments=plane_size,
            height_segments=plane_size,
            color=(0.5, 0.5, 0.5, 0.5),
            edge_color="gray",
            parent=self._view.scene)

        self._cf_lines = self._make_cf_lines(self._view.scene)
        self._cf_arrow = self.make_cf_arrow(0.5, 0.02, 0.1, 0.1, self._view.scene)

        # Subscribe to models
        self.slamdeck = slamdeck
        self._sensors = [
            #self.slamdeck.get_sensor(SlamdeckSensorId.MAIN),
            self.slamdeck.get_sensor(SlamdeckSensorId.FRONT),
            self.slamdeck.get_sensor(SlamdeckSensorId.RIGHT),
            self.slamdeck.get_sensor(SlamdeckSensorId.BACK),
            self.slamdeck.get_sensor(SlamdeckSensorId.LEFT)
        ]
        self._rotation_matrices = {
            SlamdeckSensorId.MAIN: RotationMatrix.MAIN,
            SlamdeckSensorId.FRONT: RotationMatrix.FRONT,
            SlamdeckSensorId.RIGHT: RotationMatrix.RIGHT,
            SlamdeckSensorId.BACK: RotationMatrix.BACK,
            SlamdeckSensorId.LEFT: RotationMatrix.LEFT
        }

        # Set grid size and pixel offset
        self.grids = self._sensors[0].resolution.value
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
        return self.cmap(distance, alpha=1)

    def _get_size(self, distance: float) -> float:
        return distance / 50

    def _get_coordinate(self, row: int, col: int, distance: int, rotation_matrix: np.ndarray) -> np.array:
        grid_pad = 1 / self.grid_size
        center = (self.grid_size / 2) - 0.5

        grid_vector = np.array([
            1.0,
            grid_pad * ( col - center ),
            grid_pad * ( row - center )
        ])

        # Scale to unit vector
        unit_vector = grid_vector / np.linalg.norm(grid_vector)

        # Scale to distance
        grid_vector = unit_vector * distance

        # Rotate
        grid_vector = rotation_matrix @ grid_vector

        # Scale down to fit world size
        grid_vector /= 1000

        roll_pitch_yaw = np.array([self._cf.roll_degrees, self._cf.pitch_degrees, self._cf.yaw_degrees])
        grid_vector = ( self._rpy_to_rot(np.deg2rad(roll_pitch_yaw)) @ grid_vector) + self._cf_pos

        #  x: col - horizontal
        #  y: row - vertical
        #  z: distance - depth
        return grid_vector

    def _create_point_cloud(self) -> t.Dict[SlamdeckSensorId, scene.visuals.Markers]:
        coordinates, colors, sizes = self._create_points()

        point_cloud =  scene.visuals.Markers(
            pos=coordinates,
            parent=self._view.scene,
            face_color=colors,
            symbol='o',
            spherical=True
        )

        self.point_cloud = point_cloud
        self.coordinates = coordinates
        return coordinates, colors, point_cloud

    def _create_points(self) -> t.Tuple[np.ndarray, np.ndarray]:
        coordinates = []
        colors = []
        sizes = []

        for sensor in self._sensors:#[self.slamdeck.get_sensor(SlamdeckSensorId.BACK)]:
            rotation_matrix = self._rotation_matrices[sensor.id]
            grid = 0
            for row in range(self.grid_size):
                for col in range(self.grid_size):
                    distance = sensor.data[grid]
                    if distance != self.OUT_OF_BOUNDS:
                        coordinate = self._get_coordinate(self.grid_size-1-row, col, distance, rotation_matrix)
                        coordinates.append(coordinate)
                        colors.append(self._get_color(distance))
                        sizes.append(self._get_size(distance))
                    grid += 1

        return np.array(coordinates), np.array(colors), np.array(sizes)

    def _update_graphics(self):
        # Update point cloud
        coordinates, colors, sizes = self._create_points()
        if len(coordinates) > 0:
            self.point_cloud.set_data(pos=coordinates, face_color=colors, size=sizes, edge_color=(0, 0, 0, 0))

        # Update Crazyflie
        self._cf_pos[2] = 0.5
        #self._cf_pos[0] = self._cf.x
        #self._cf_pos[1] = self._cf.y
        self._cf_pos[2] = self._cf.z
        x, y, z = self._create_cf_lines()
        self._cf_lines[0].set_data(x) # x axis
        self._cf_lines[1].set_data(y) # y axis
        self._cf_lines[2].set_data(z) # z axis
        self._cf_marker.set_data(pos=np.array([self._cf_pos]))

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

    def rotate_to_cf(self, vector: np.ndarray) -> np.ndarray:
        roll_pitch_yaw = np.array([self._cf.roll_degrees, self._cf.pitch_degrees, self._cf.yaw_degrees])
        cf_rotation = self._rpy_to_rot(np.deg2rad(roll_pitch_yaw))
        vector = cf_rotation @ vector # Rotate
        return vector

    def _make_cf_lines(self, parent) -> np.ndarray:
        x, y, z = self._create_cf_lines()
        x = scene.visuals.LinePlot(x, width=1, color='red', parent=parent, marker_size=0.0)
        y = scene.visuals.LinePlot(y, width=1, color='green', parent=parent, marker_size=0.0)
        z = scene.visuals.LinePlot(z, width=1, color='blue', parent=parent, marker_size=0.0)
        return x, y, z

    def _create_cf_lines(self) -> np.ndarray:
        x = [self._cf_line_size, 0.0, 0.0]
        y = [0.0, self._cf_line_size, 0.0]
        z = [0.0, 0.0, self._cf_line_size]

        x = self.rotate_to_cf(x) + self._cf_pos
        y = self.rotate_to_cf(y) + self._cf_pos
        z = self.rotate_to_cf(z) + self._cf_pos

        x = np.array([self._cf_pos, x])
        y = np.array([self._cf_pos, y])
        z = np.array([self._cf_pos, z])

        return x, y, z

    def make_cf_arrow(self, length, width, head_length, head_width, parent) -> OrientationArrow:
        w = width / 2
        hw = head_width / 2
        base_len = length - head_length

        base_arrow = np.array([[0, w, 0],
                         [base_len, w, 0],
                         [base_len, hw, 0],
                         [length, 0, 0],
                         [base_len, -hw, 0],
                         [base_len, -w, 0],
                         [0, -w, 0]
                         ])

        x_axis = (RotationMatrix.FRONT @ base_arrow.T).T
        y_axis = (RotationMatrix.LEFT @ base_arrow.T).T
        z_axis = (RotationMatrix.MAIN @ base_arrow.T).T

        scene.visuals.LinePlot(x_axis, width=width, color='red', parent=parent, marker_size=0.0)
        scene.visuals.LinePlot(y_axis, width=width, color='green', parent=parent, marker_size=0.0)
        scene.visuals.LinePlot(z_axis, width=width, color='blue', parent=parent, marker_size=0.0)

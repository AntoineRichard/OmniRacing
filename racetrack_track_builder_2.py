__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Antoine Richard"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine0richard@gmail.com"
__status__ = "development"

import omni
from WorldBuilders.pxr_utils import (
    createStandaloneInstance,
    setInstancerParameters,
)
from racetrack_utils import createCubeLikeObject, bindMaterial
import dataclasses
from pxr import Sdf, Gf, UsdGeom
import numpy as np
import pickle

from racetrack_configs import CFGF
from racetrack_generators import RGF
from assets_managers import AssetsManagers


@dataclasses.dataclass
class TrackBuilderConfig:
    name: str = dataclasses.field(default_factory=str)
    line_thickness: float = dataclasses.field(default_factory=float)
    line_length: float = dataclasses.field(default_factory=float)
    line_width: float = dataclasses.field(default_factory=float)
    num_tracks: int = dataclasses.field(default_factory=int)
    env_spacing: float = dataclasses.field(default_factory=float)
    track_config: dict = None

    def __post_init__(self):
        self.track_config = CFGF(self.track_config["name"], self.track_config)


class TrackBuilder:
    def __init__(self, cfg: TrackBuilderConfig, assets_managers: AssetsManagers):
        self.settings = cfg
        self.stage = omni.usd.get_context().get_stage()
        self.TrackGenerator = RGF(
            self.settings.track_config.name, self.settings.track_config
        )
        self.assets_managers = assets_managers
        self.grid_x = int(np.sqrt(self.settings.num_tracks))

    def build(self):
        self.buildFloor()
        self.randomize()

    def buildFloor(self):
        x_max = int(self.grid_x * self.settings.env_spacing)
        y_max = int(self.settings.num_tracks // self.grid_x * self.settings.env_spacing)
        x = np.linspace(0, 3 * (x_max // 3), x_max // 3, endpoint=False)
        y = np.linspace(0, 3 * (y_max // 3), y_max // 3, endpoint=False)
        xx, yy = np.meshgrid(x, y)
        self.floor_position = np.stack(
            [xx.flatten(), yy.flatten(), np.zeros_like(xx.flatten())], axis=1
        )

    def randomize(self):
        self.instances_position = []
        self.instances_quaternion = []
        self.instances_id = []
        for i in range(self.settings.num_tracks):
            self.TrackGenerator.randomizeTrack()
            # Get the track contours
            center_line, center_line_theta = self.TrackGenerator.getCenterLine(
                distance=self.settings.line_length * 2 * 0.6
            )
            inner_line, outer_line = self.TrackGenerator.getTrackBoundaries(
                distance=self.settings.line_length * 2 * 0.6
            )
            inner_line, inner_line_theta = inner_line
            outer_line, outer_line_theta = outer_line
            # self.setInstancePoses(center_line, center_line_theta, skip_one=True)
            x_offset = (i % self.grid_x) * self.settings.env_spacing
            y_offset = (i // self.grid_x) * self.settings.env_spacing
            offset = np.array([x_offset, y_offset])
            self.GetInstancePoses(inner_line + offset + 12.0, inner_line_theta)
            self.GetInstancePoses(outer_line + offset + 12.0, outer_line_theta)
        self.updateInstancer()

    def GetInstancePoses(self, line, theta, skip_one=False):
        quat = np.zeros((len(theta), 4))
        quat[:, 0] = 0
        quat[:, 1] = 0
        quat[:, 2] = np.sin(theta / 2)
        quat[:, 3] = np.cos(theta / 2)
        z = np.zeros_like(line[:, 0])
        pos = np.stack([line[:, 0], line[:, 1], z], axis=1)
        self.instances_position.append(pos)
        self.instances_quaternion.append(quat)

    def updateInstancer(self):
        self.instances_position = np.concatenate(self.instances_position, axis=0)
        self.instances_quaternion = np.concatenate(self.instances_quaternion, axis=0)
        self.assets_managers.set_cone_parameters(
            self.instances_position, quat=self.instances_quaternion
        )
        self.assets_managers.set_floor_parameters(self.floor_position)

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

        self.tracks = []
        self.tracks_path = "assets/tracks.pkl"
        self.loadTracks()

    def loadTracks(self):
        with open(self.tracks_path, "rb") as f:
            self.tracks = pickle.load(f)

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
            idx = np.random.randint(0, len(self.tracks))
            inner = self.tracks[idx]["inner"]
            outer = self.tracks[idx]["outer"]

            x_offset = (
                i % self.grid_x
            ) * self.settings.env_spacing + self.settings.env_spacing / 2
            y_offset = (
                i // self.grid_x
            ) * self.settings.env_spacing + self.settings.env_spacing / 2
            offset = np.array([x_offset, y_offset, 0])

            self.instances_position.append(inner[0] + offset)
            self.instances_position.append(outer[0] + offset)
            self.instances_quaternion.append(inner[1])
            self.instances_quaternion.append(outer[1])
        self.updateInstancer()

    def updateInstancer(self):
        self.instances_position = np.concatenate(self.instances_position, axis=0)
        self.instances_quaternion = np.concatenate(self.instances_quaternion, axis=0)
        self.assets_managers.set_cone_parameters(
            self.instances_position, quat=self.instances_quaternion
        )
        self.assets_managers.set_floor_parameters(self.floor_position)

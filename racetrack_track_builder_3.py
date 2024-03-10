__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Antoine Richard"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine0richard@gmail.com"
__status__ = "development"


from assets_managers import AssetsManagers
from WorldBuilders.pxr_utils import (
    createStandaloneInstance,
    setInstancerParameters,
)
from racetrack_generators import RGF
from racetrack_configs import CFGF
import dataclasses
import numpy as np
import pickle
import torch
import omni


@dataclasses.dataclass
class TrackManagerConfig:
    tracks_path: str = dataclasses.field(default_factory=str)
    env_spacing: float = dataclasses.field(default_factory=float)
    assets_root_path: str = dataclasses.field(default_factory=str)
    cones_path: str = dataclasses.field(default_factory=str)
    floors_path: str = dataclasses.field(default_factory=str)
    hdris_path: str = dataclasses.field(default_factory=str)

    track_config: dict = None


class TrackManager:
    def __init__(self, cfg: dict, num_envs: int, device: str):
        self.settings = TrackManagerConfig(**cfg)

        self.stage = omni.usd.get_context().get_stage()
        self.assets_managers = AssetsManagers()
        self.grid_x = int(np.sqrt(self.settings.num_tracks))

        self.tracks = []
        self.tracks_path = "assets/tracks.pkl"
        self.loadTracks()

        self.num_envs = num_envs
        self.device = device
        self.envs_initial_positions = np.zeros((self.num_envs, 3))

    def initialize_tracks(self, initial_positions: torch.Tensor):
        self.envs_initial_positions = initial_positions.cpu().numpy()
        self.build()

    def loadTracks(self):
        with open(self.tracks_path, "rb") as f:
            self.tracks = pickle.load(f)

    def build(self):
        self.loadTracks()
        self.AM = AssetsManagers(
            self.settings.assets_root_path,
            self.settings.cones_path,
            self.settings.floors_path,
            self.settings.hdris_path,
        )
        self.buildFloor()
        self.randomize()

    def buildFloor(self):
        x_max = np.max(self.envs_initial_positions[:, 0]) + self.settings.env_spacing
        y_max = np.max(self.envs_initial_positions[:, 1]) + self.settings.env_spacing
        x_min = np.min(self.envs_initial_positions[:, 0]) - self.settings.env_spacing
        y_min = np.min(self.envs_initial_positions[:, 1]) - self.settings.env_spacing
        dx = x_max - x_min
        dy = y_max - y_min

        x = np.linspace(
            x_min, x_min + int(1 + dx / 3) * 3, 2 + int(dx / 3), endpoint=True
        )
        y = np.linspace(
            y_min, y_min + int(1 + dy / 3) * 3, 2 + int(dy / 3), endpoint=True
        )

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

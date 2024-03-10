__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Antoine Richard"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine0richard@gmail.com"
__status__ = "development"


from assets_managers import AssetsManagers
from racetrack_generators import RGF
from racetrack_configs import CFGF
import dataclasses
import numpy as np
import pickle
import torch


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

        self.tracks = []
        self.loadTracks()

        self.num_envs = num_envs
        self.device = device

        # This data is stored as numpy arrays
        self.envs_initial_positions = np.zeros((self.num_envs, 3))
        self.per_env_track_position = {}
        self.per_env_track_quaternion = {}
        self.per_env_track_id = {}

        # This data is stored as torch tensors

    def initialize_tracks(self, initial_positions: torch.Tensor):
        self.envs_initial_positions = initial_positions.cpu().numpy()
        self.build()

    def loadTracks(self):
        with open(self.settings.tracks_path, "rb") as f:
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
        self.reset_floor()

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

    def reset(self, env_ids: torch.Tensor):
        env_ids = env_ids.cpu().numpy()
        track_indices = np.random.randint(0, len(self.tracks), size=(len(env_ids)))
        for idx, track_idx in zip(env_ids, track_indices):
            self.per_env_track_position[idx] = (
                np.concatenate(
                    [
                        self.tracks[track_idx]["inner"][0],
                        self.tracks[track_idx]["outer"][0],
                        np.zeros_like(self.tracks[track_idx]["inner"][0]),
                    ],
                    axis=0,
                )
                + self.envs_initial_positions[idx]
            )
            self.per_env_track_quaternion[idx] = np.concatenate(
                [
                    self.tracks[track_idx]["inner"][1],
                    self.tracks[track_idx]["outer"][1],
                    np.zeros_like(self.tracks[track_idx]["inner"][1]),
                ],
                axis=0,
            )
            self.per_env_track_id[idx] = np.random.randint(
                0,
                300,
                self.tracks[track_idx]["inner"][0].shape[0]
                + self.tracks[track_idx]["outer"][0].shape[0],
            )

        positions = np.concatenate(
            [self.per_env_track_position[idx] for idx in env_ids], axis=0
        )
        quaternions = np.concatenate(
            [self.per_env_track_quaternion[idx] for idx in env_ids], axis=0
        )
        ids = np.concatenate([self.per_env_track_id[idx] for idx in env_ids], axis=0)

        self.AM.set_cone_parameters(positions, quat=quaternions, ids=ids)

    def reset_floor(self):
        self.AM.set_floor_parameters(self.floor_position)

    def reset_skydome(self):
        self.AM.skydome_manager.randomize()

__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Antoine Richard"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine0richard@gmail.com"
__status__ = "development"


import numpy as np
import pickle

from racetrack_configs import CFGF
from racetrack_generators import RGF

track_cfg = {
    "name": "track",
    "num_tracks": 100,
    "env_spacing": 0.0,
    "line_thickness": 0.001,
    "line_length": 0.8,
    "line_width": 0.05,
    "track_config": {
        "name": "random_angular",
        "max_corners": 20,
        "min_corners": 8,
        "min_radius": 0.25,
        "resampling_samples": 1000,
        "theta_noise": 0.0,
        "track_width": 24.0,
        "track_length": 24.0,
        "track_thickness": 1.5,
        "seed": 42,
        "resolution": 0.01,
    },
}


class TrackBuilderPreGen:
    def __init__(self, cfg: TrackBuilderConfig, save_path: str = None):
        self.settings = cfg
        self.TrackGenerator = RGF(
            self.settings.track_config.name, self.settings.track_config
        )
        self.track_dictionaries = []
        self.save_path = save_path

    def generate(self):
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
            in_pos, in_quat = self.GetPoses(inner_line, inner_line_theta)
            out_pos, out_quat = self.GetPoses(outer_line, outer_line_theta)
            center_pos, center_quat = self.GetPoses(center_line, center_line_theta)
            dict = {
                "inner": (in_pos, in_quat),
                "outer": (out_pos, out_quat),
                "center": (center_pos, center_quat),
            }
            self.track_dictionaries.append(dict)
        self.save()

    def GetPoses(self, line, theta, skip_one=False):
        quat = np.zeros((len(theta), 4))
        quat[:, 0] = 0
        quat[:, 1] = 0
        quat[:, 2] = np.sin(theta / 2)
        quat[:, 3] = np.cos(theta / 2)
        z = np.zeros_like(line[:, 0])
        pos = np.stack([line[:, 0], line[:, 1], z], axis=1)
        return pos, quat

    def save(self):
        with open(self.save_path, "wb") as f:
            pickle.dump(self.track_dictionaries, f)


if __name__ == "__main__":
    from OmniRacing.racetrack_track_builder_2 import (
        TrackBuilderConfig,
    )

    track_cfg = TrackBuilderConfig(**track_cfg)

    TB = TrackBuilderPreGen(track_cfg, save_path="assets/tracks2.pkl")
    TB.generate()

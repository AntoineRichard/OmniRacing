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
    createInstancerAndCache4Variants,
)
from racetrack_utils import createCubeLikeObject, bindMaterial
import dataclasses
from pxr import Sdf, Gf, UsdGeom
import numpy as np

from racetrack_configs import CFGF
from racetrack_generators import RGF

ASSET_LIST = [
    "assets/cones/cone_1.usd",
    "assets/cones/cone_2.usd",
    "assets/cones/cone_3.usd",
    "assets/cones/cone_4.usd",
    "assets/cones/cone_5.usd",
    "assets/cones/cone_6.usd",
]


@dataclasses.dataclass
class TrackBuilderConfig:
    name: str = dataclasses.field(default_factory=str)
    line_thickness: float = dataclasses.field(default_factory=float)
    line_length: float = dataclasses.field(default_factory=float)
    line_width: float = dataclasses.field(default_factory=float)
    track_config: dict = None

    def __post_init__(self):
        self.track_config = CFGF(self.track_config["name"], self.track_config)


class TrackBuilder:
    def __init__(self, cfg: TrackBuilderConfig):
        self.settings = cfg
        self.stage = omni.usd.get_context().get_stage()
        self.TrackGenerator = RGF(
            self.settings.track_config.name, self.settings.track_config
        )
        self.stage_path = "/Track"

    def build(self):
        self.instantiateTrackElements()
        self.randomize()

    def instantiateTrackElements(self):
        xformPrim = UsdGeom.Xform.Define(self.stage, self.stage_path)
        self.instancer_path = self.stage_path + "/instancer"
        createInstancerAndCache4Variants(
            self.stage, self.instancer_path, ASSET_LIST, "shadingVariant"
        )
        # instancer = createStandaloneInstance(self.stage, self.instancer_path)
        # self.white_line_path = self.instancer_path + "/white_line"
        # self.black_line_path = self.instancer_path + "/black_line"
        # createCubeLikeObject(
        #    self.stage, self.white_line_path, (0.15, 0.05, 0.001), (0, 0, 0)
        # )
        # bindMaterial(self.stage, "/Looks/panels", self.white_line_path)
        # createCubeLikeObject(
        #    self.stage, self.black_line_path, (0.15, 0.05, 0.0005), (0, 0, 0)
        # )
        # bindMaterial(self.stage, "/Looks/stairs", self.black_line_path)
        # instancer.GetPrototypesRel().AddTarget(self.white_line_path)
        # instancer.GetPrototypesRel().AddTarget(self.black_line_path)
        # self.white_line_id = 0
        # self.black_line_id = 1

    def randomize(self):
        self.instances_position = []
        self.instances_quaternion = []
        self.instances_id = []
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
        self.setInstancePoses(inner_line, inner_line_theta)
        self.setInstancePoses(outer_line, outer_line_theta)
        self.updateInstancer()

    def setInstancePoses(self, line, theta, skip_one=False):
        quat = np.zeros((len(theta), 4))
        quat[:, 0] = 0
        quat[:, 1] = 0
        quat[:, 2] = np.sin(theta / 2)
        quat[:, 3] = np.cos(theta / 2)
        z = np.zeros_like(line[:, 0])
        pos = np.stack([line[:, 0], line[:, 1], z], axis=1)
        # id = np.ones_like(z)
        # if skip_one:
        #    self.instances_position.append(pos[::2])
        #    self.instances_quaternion.append(quat[::2])
        #    self.instances_id.append(id[::2])
        # else:
        #    self.instances_position.append(pos)
        #    self.instances_quaternion.append(quat)
        #    id[::2] = 0
        #    self.instances_id.append(id)
        self.instances_position.append(pos)
        self.instances_quaternion.append(quat)

    def updateInstancer(self):
        self.instances_position = np.concatenate(self.instances_position, axis=0)
        self.instances_quaternion = np.concatenate(self.instances_quaternion, axis=0)
        # self.instances_id = np.concatenate(self.instances_id, axis=0)
        setInstancerParameters(
            self.stage,
            self.instancer_path,
            self.instances_position,
            quat=self.instances_quaternion,
            # ids=self.instances_id,
        )

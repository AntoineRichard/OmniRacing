import dataclasses
from typing import Union


@dataclasses.dataclass
class RandomAngularTrackGeneratorConfig:
    name: str = dataclasses.field(default_factory=str)
    # Max/min number of corners generated
    max_corners: int = dataclasses.field(default_factory=int)
    min_corners: int = dataclasses.field(default_factory=int)
    # When generating a track, we will use a angular representation,
    # where each corner is defined by a radius and an angle.
    # min_radius is the minimum radius (in percentage) that can be used
    # to generate a corner. Setting it to 1.0, mins you will have a perfect
    # circle. Setting it to 0.0, may create large variation between points
    # and create a very irregular track. Potentially intersecting at the middle.
    # The smaller the value, the sharper the corners.
    min_radius: float = dataclasses.field(default_factory=float)
    # The number of points to use when fitting a spline ontop of the generated
    # points. The higher the value, the smoother the track will be.
    resampling_samples: int = dataclasses.field(default_factory=int)
    # The amount of noise to add to the angle of each corner in radiants.
    # Recommended value is 0.
    theta_noise: float = dataclasses.field(default_factory=float)
    # The width of the track in meters.
    track_width: float = dataclasses.field(default_factory=float)
    # The length of the track in meters.
    track_length: float = dataclasses.field(default_factory=float)
    # The thickness of the track in meters.
    track_thickness: float = dataclasses.field(default_factory=float)
    # The seed used to generate the tracks.
    seed: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.max_corners) is int, "max_corners must be an integer"
        assert type(self.min_corners) is int, "min_corners must be an integer"
        assert type(self.min_radius) is float, "min_radius must be a float"
        assert (
            type(self.resampling_samples) is int
        ), "resampling_samples must be an integer"
        assert type(self.theta_noise) is float, "theta_noise must be a float"
        assert type(self.track_width) is float, "track_width must be a float"
        assert type(self.track_length) is float, "track_length must be a float"
        assert type(self.track_thickness) is float, "track_thickness must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.resolution) is float, "resolution must be a float"

        assert self.max_corners > 0, "max_corners must be greater than 0"
        assert self.min_corners > 0, "min_corners must be greater than 0"
        assert (
            self.max_corners > self.min_corners
        ), "max_corners must be greater than min_corners"
        assert self.min_radius > 0, "min_radius must be greater than 0"
        assert self.min_radius < 1, "min_radius must be less than 1"
        assert self.resampling_samples > 0, "resampling_samples must be greater than 0"
        assert self.theta_noise >= 0, "theta_noise must be greater than 0"
        assert self.track_width > 0, "track_width must be greater than 0"
        assert self.track_length > 0, "track_length must be greater than 0"
        assert self.track_thickness > 0, "track_thickness must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"


@dataclasses.dataclass
class OvalTrackGeneratorConfig:
    name: str = dataclasses.field(default_factory=str)
    resampling_samples: int = dataclasses.field(default_factory=int)
    track_width: float = dataclasses.field(default_factory=float)
    track_length: float = dataclasses.field(default_factory=float)
    track_thickness: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert (
            type(self.resampling_samples) is int
        ), "resampling_samples must be an integer"
        assert type(self.track_width) is float, "track_width must be a float"
        assert type(self.track_length) is float, "track_length must be a float"
        assert type(self.track_thickness) is float, "track_thickness must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.resolution) is float, "resolution must be a float"

        assert self.resampling_samples > 0, "resampling_samples must be greater than 0"
        assert self.track_width > 0, "track_width must be greater than 0"
        assert self.track_length > 0, "track_length must be greater than 0"
        assert self.track_thickness > 0, "track_thickness must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"


@dataclasses.dataclass
class TrackConfig:
    name: str = dataclasses.field(default_factory=str)
    track_width: float = dataclasses.field(default_factory=float)
    track_length: float = dataclasses.field(default_factory=float)
    track_thickness: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)
    stage_path: str = dataclasses.field(default_factory=str)
    resolution: float = dataclasses.field(default_factory=float)
    track_generator_cfg: Union[
        RandomAngularTrackGeneratorConfig, OvalTrackGeneratorConfig
    ] = None

    def __post_init__(self):
        assert type(self.track_width) is float, "track_width must be a float"
        assert type(self.track_length) is float, "track_length must be a float"
        assert type(self.track_thickness) is float, "track_thickness must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.stage_path) is str, "stage_path must be a string"
        assert type(self.resolution) is float, "resolution must be a float"

        self.track_generator_cfg = CFGF(
            self.track_generator_cfg["name"], self.track_generator_cfg
        )

        assert self.track_width > 0, "track_width must be greater than 0"
        assert self.track_length > 0, "track_length must be greater than 0"
        assert self.track_thickness > 0, "track_thickness must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"


class Config_Factory:
    def __init__(self):
        self.cfgs = {}

    def register_cfg(self, name, cfg):
        self.cfgs[name] = cfg

    def __call__(self, name, cfg):
        return self.cfgs[name](**cfg)


CFGF = Config_Factory()
CFGF.register_cfg("random_angular", RandomAngularTrackGeneratorConfig)
CFGF.register_cfg("oval", OvalTrackGeneratorConfig)

__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Antoine Richard"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine0richard@gmail.com"
__status__ = "development"

from typing import Any
from racetrack_configs import (
    RandomAngularTrackGeneratorConfig,
    OvalTrackGeneratorConfig,
    CFGF,
)

from scipy.interpolate import splprep, splev
import numpy as np
import cv2


class BaseTrackGenerator:
    def __init__(self, cfg):
        self.settings = cfg
        self.mask_width = int(
            1.25 * self.settings.track_width / self.settings.resolution
        )
        self.mask_length = int(
            1.25 * self.settings.track_length / self.settings.resolution
        )

        self.mask = np.zeros((self.mask_length, self.mask_width))
        self.outer_line = np.array([], dtype=np.float32)
        self.inner_line = np.array([], dtype=np.float32)
        self.center_line = np.array([], dtype=np.float32)
        self.rng = np.random.default_rng(self.settings.seed)

    def randomizeTrack(self):
        self.generateInitialTrack()
        self.compute_interior_exterior_lines()
        self.generateTrackMask()

    def generateTrackMask(self):
        self.mask = np.zeros((self.mask_length, self.mask_width))
        self.mask = cv2.fillPoly(
            self.mask, [(self.outer_line / self.settings.resolution).astype(int)], 1
        )
        self.mask = cv2.fillPoly(
            self.mask, [(self.inner_line / self.settings.resolution).astype(int)], 0
        )

    def generateInitialTrack(self):
        raise NotImplementedError

    def getMask(self):
        return self.mask

    def getCenterLine(self, distance=0.1):
        return self.getTangentToLine(self.center_line, distance)

    def getTrackBoundaries(self, distance=0.1):
        return (
            self.getTangentToLine(self.inner_line, distance),
            self.getTangentToLine(self.outer_line, distance),
        )

    def compute_interior_exterior_lines(self):
        xy, theta = self.getTangentToLine(self.center_line, 0.1)
        # Computes the outer line
        self.outer_line = self.applyOffsetToLine(
            xy, theta, -self.settings.track_thickness / 2, np.pi / 2
        )
        self.outer_line, self.outer_line_theta = self.getTangentToLine(
            self.outer_line, 0.1
        )
        # Computes the inner line
        self.inner_line = self.applyOffsetToLine(
            xy, theta, self.settings.track_thickness / 2, np.pi / 2
        )
        self.inner_line, self.inner_line_theta = self.getTangentToLine(
            self.inner_line, 0.1
        )

    @staticmethod
    def generateMaskFromLine(mask, line, resolution, value):
        mask = np.zeros_like(mask)
        line = line / resolution + np.array(mask.shape) / 2
        line = np.flip(line, axis=1)
        mask = cv2.fillPoly(mask, [(line).astype(int)], value)
        mask = cv2.erode(mask, np.ones((3, 3)))
        return mask

    @staticmethod
    def interpolateAtFixedDistance(line, interpolation_distance):
        length = np.sum(np.linalg.norm(line[:-1] - line[1:], 1, axis=-1))
        tck, u = splprep([line[:, 0], line[:, 1]], s=0, per=True)
        unew = np.linspace(0, 1, int(length / interpolation_distance))
        xy = splev(unew, tck)
        xy = np.stack([xy[0], xy[1]], axis=1)
        return xy

    @staticmethod
    def deriveLine(line, interpolation_distance):
        length = np.sum(np.linalg.norm(line[:-1] - line[1:], 1, axis=-1))
        tck, u = splprep([line[:, 0], line[:, 1]], s=0, per=True)
        unew = np.linspace(0, 1, int(length / interpolation_distance))
        xy = splev(unew, tck)
        dxdy = splev(unew, tck, der=1)
        xy = np.stack([xy[0], xy[1]], axis=1)
        return xy, dxdy

    @staticmethod
    def getTangentToLine(line, interpolation_distance):
        xy, dxdy = BaseTrackGenerator.deriveLine(line, interpolation_distance)
        theta = np.arctan2(dxdy[1], dxdy[0])
        return xy, theta

    @staticmethod
    def applyOffsetToLine(line, tangent, distance_offset, theta_offset):
        theta_e = tangent + theta_offset
        dx = np.cos(theta_e) * distance_offset
        dy = np.sin(theta_e) * distance_offset
        return np.array([line[:, 0] + dx, line[:, 1] + dy]).T


class OvalTrackGenerator(BaseTrackGenerator):
    def __init__(self, cfg: OvalTrackGeneratorConfig):
        super().__init__(cfg)

    def generateInitialTrack(self):
        theta = np.linspace(0, 2 * np.pi, 10000)
        x_center = np.cos(theta) * self.settings.track_length / 2
        y_center = np.sin(theta) * self.settings.track_width / 2
        self.center_line = np.stack([x_center, y_center], axis=1)


class RandomAngularTrackGenerator(BaseTrackGenerator):
    def __init__(self, cfg: RandomAngularTrackGeneratorConfig):
        super().__init__(cfg)

    def generateInitialTrack(self):
        num_corners = self.rng.integers(
            self.settings.min_corners, self.settings.max_corners
        )
        theta = np.linspace(0, 2 * np.pi, num_corners)
        r = np.zeros(num_corners)
        for i in range(num_corners):
            r[i] = self.rng.uniform(
                self.settings.min_radius, 1
            ) * self.getRectangleDistanceByAngle(
                self.settings.track_width, self.settings.track_length, theta[i]
            )
        x = np.cos(theta) * r
        y = np.sin(theta) * r
        tck, u = splprep([x, y], s=0, per=True)
        x, y = splev(np.linspace(0, 1, 1000), tck)
        self.center_line = np.stack([x, y], axis=1)
        self.cleanTrack()

    def cleanTrack(self):
        xy, theta = self.getTangentToLine(self.center_line, 0.01)
        # Compute the inner line
        inner_line = self.applyOffsetToLine(
            xy, theta, self.settings.track_thickness / 2, np.pi / 2
        )
        # make a mask by filling everything inside the inner line
        mask = self.generateMaskFromLine(
            self.mask, inner_line, self.settings.resolution, 1
        )
        # get the largest contour from the mask (remove self intersecting segments)
        self.inner_line = self.getLineFromLargestContour(mask, self.settings.resolution)
        # Compute the inner line with a very high resolution.
        xy, theta = self.getTangentToLine(self.inner_line, 0.001)
        # Compute the center line by offseting the inner line by half the track thickness.
        center_line = self.applyOffsetToLine(
            xy, theta, self.settings.track_thickness / 2, -np.pi / 2
        )
        # Keep the high res center line
        self.center_line = center_line.copy()
        # Compute the outer line by offseting the inner line by the track thickness.
        outer_line = self.applyOffsetToLine(
            xy, theta, self.settings.track_thickness, -np.pi / 2
        )
        # make a mask by filling everything inside the outer line
        mask = self.generateMaskFromLine(
            self.mask, outer_line, self.settings.resolution, 1
        )
        # get the largest contour from the mask (remove self intersecting segments)
        self.outer_line = self.getLineFromLargestContour(mask, self.settings.resolution)

    def compute_interior_exterior_lines(self):
        pass

    @staticmethod
    def getLineFromLargestContour(mask, resolution):
        mask = mask.astype(np.uint8)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        cnt = contours[0][:, 0, :] * resolution
        cnt = np.flip(cnt, axis=1) - np.array(mask.shape) / 2 * resolution
        length = np.sum(np.linalg.norm(cnt[:-1] - cnt[1:], 1, axis=-1))
        tck, u = splprep([cnt[::20, 0], cnt[::20, 1]], s=0, per=True)
        unew = np.linspace(0, 1, int(length / 0.1))
        line = np.array(splev(unew, tck)).T
        return line

    @staticmethod
    def getRectangleDistanceByAngle(height, width, theta):
        theta_corner = np.arctan2(height, width)
        theta = theta % (2 * np.pi)
        coordinates = []
        r = np.sqrt(height**2 + width**2) / 2
        if (0 <= theta) and (theta <= theta_corner):
            coordinates = [width / 2, np.sin(theta) * r]
        elif (theta_corner <= theta) and (theta <= np.pi - theta_corner):
            coordinates = [np.cos(theta) * r, height / 2]
        elif (np.pi - theta_corner < theta) and theta <= (np.pi + theta_corner):
            coordinates = [-width / 2, np.sin(theta) * r]
        elif (np.pi + theta_corner <= theta) and (theta <= 2 * np.pi - theta_corner):
            coordinates = [np.cos(theta) * r, -height / 2]
        elif (2 * np.pi - theta_corner <= theta) and (theta <= 2 * np.pi):
            coordinates = [width / 2, np.sin(theta) * r]
        return np.linalg.norm(coordinates)


class RacetrackGeneratorFactory:
    def __init__(self):
        self.generators = {}

    def register(self, name, generator):
        self.generators[name] = generator

    def __call__(self, name, cfg):
        return self.generators[name](cfg)


RGF = RacetrackGeneratorFactory()
RGF.register("oval", OvalTrackGenerator)
RGF.register("random_angular", RandomAngularTrackGenerator)

if __name__ == "__main__":
    from matplotlib import pyplot as plt

    cfg = {
        "name": "random_angular",
        "max_corners": 20,
        "min_corners": 8,
        "min_radius": 0.25,
        "resampling_samples": 1000,
        "theta_noise": 0.0,
        "track_width": 10.0,
        "track_length": 20.0,
        "track_thickness": 1.0,
        "seed": 42,
        "resolution": 0.01,
    }
    cfg = CFGF(cfg["name"], cfg)
    track_generator = RGF(cfg.name, cfg)
    fig, axs = plt.subplots(7, 12, layout="constrained")
    for ax in axs.flat:
        track_generator.randomizeTrack()
        center_line = track_generator.getCenterLine()[0]
        inner_line, outer_line = track_generator.getTrackBoundaries()
        inner_line = inner_line[0]
        outer_line = outer_line[0]
        ax.plot(center_line[:, 0], center_line[:, 1])
        ax.plot(inner_line[:, 0], inner_line[:, 1])
        ax.plot(outer_line[:, 0], outer_line[:, 1])
        ax.set_aspect("equal")
        ax.set_xlim(-12, 12)
        ax.set_ylim(-12, 12)
    fig.get_layout_engine().set(w_pad=4 / 72, h_pad=4 / 72, hspace=0, wspace=0)

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

    cfg = {
        "name": "oval",
        "resampling_samples": 1000,
        "track_width": 10.0,
        "track_length": 20.0,
        "track_thickness": 1.0,
        "seed": 42,
        "resolution": 0.01,
    }
    cfg = CFGF(cfg["name"], cfg)
    track_generator = RGF(cfg.name, cfg)
    fig, axs = plt.subplots(7, 12, layout="constrained")
    for ax in axs.flat:
        track_generator.randomizeTrack()
        center_line = track_generator.getCenterLine()[0]
        inner_line, outer_line = track_generator.getTrackBoundaries()
        inner_line = inner_line[0]
        outer_line = outer_line[0]
        ax.plot(center_line[:, 0], center_line[:, 1])
        ax.plot(inner_line[:, 0], inner_line[:, 1])
        ax.plot(outer_line[:, 0], outer_line[:, 1])
        ax.set_aspect("equal")
        ax.set_xlim(-12, 12)
        ax.set_ylim(-12, 12)
    fig.get_layout_engine().set(w_pad=4 / 72, h_pad=4 / 72, hspace=0, wspace=0)

    plt.show()

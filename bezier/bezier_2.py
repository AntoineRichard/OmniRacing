import time
import torch
import numpy as np
from scipy.special import binom
import matplotlib.pyplot as plt
from geo_complexity import complexity
import pickle
import matplotlib.cm as cm
import math

bernstein = lambda n, k, t: binom(n, k) * t**k * (1.0 - t) ** (n - k)
torch.manual_seed(0)


class TrackGenerator:
    def __init__(
        self,
        min_num_points: int = 9,
        max_num_points: int = 13,
        num_points_per_segment: int = 30,
        min_point_distance: float = 0.05,
        min_angle: float = (12.5 / 180) * np.pi,
        scale: float = 1.0,
        rad: float = 0.2,
        edgy: float = 0.0,
        device: str = "cuda",
    ) -> None:
        """Initializes the TrackGenerator.

        Args:
            min_num_points: The minimum number of points that can be generated.
            max_num_points: The maximum number of points that can be generated.
            num_points_per_segment: The number of points to generate for each segment.
            min_point_distance: The minimum distance between the points sampled to create the track. Should be between
                0 and 1. Smaller values can create more complex tracks.
            min_angle: The minimum angle between the segments of the track. Should be between 0 and pi. Smaller values
                can create more complex tracks. Values close too small can create tracks that are not drivable. I.e. the
                the tracks may self-intersect.
            scale: The scale of the unit square. This defines the size of track in meters.
            rad: The radius of the curve.
            edgy: The edginess of the curve.
            device: The device to use for computation."""

        # Assign the parameters
        self._min_num_points = min_num_points
        self._max_num_points = max_num_points
        self._min_point_distance = min_point_distance
        self._min_angle = min_angle
        self._scale = scale
        self._rad = rad
        self._edgy = edgy
        self._device = device
        self._num_points_per_segment = num_points_per_segment

        # Compute the angle between the two segments map it to [0, 1]
        self._p = math.atan(self._edgy) / math.pi + 0.5
        # Compute the number of cells in the grid
        self._num_cells = int(self._scale / (self._min_point_distance * 2))
        # Precompute the bernstein polynomials
        t = np.linspace(0, 1, num=self._num_points_per_segment)
        self._bernstein_0 = torch.tensor(bernstein(3, 0, t), device=self._device)
        self._bernstein_1 = torch.tensor(bernstein(3, 1, t), device=self._device)
        self._bernstein_2 = torch.tensor(bernstein(3, 2, t), device=self._device)
        self._bernstein_3 = torch.tensor(bernstein(3, 3, t), device=self._device)

    @staticmethod
    def ccw_sort(points: torch.Tensor):
        """Computes the mean of all the points, and orders them in the trigonometric direction around it.

        Args:
            points: A 3D tensor, [num_envs, num_points, 2]."""

        # Compute the center of the points
        mean = torch.mean(points, axis=1)
        # Generate a vector from the center to the points
        dist = points - mean.unsqueeze(1)
        # Get the angle between the vector and the positive x-axis
        angles = torch.arctan2(dist[:, :, 0], dist[:, :, 1])
        # Sort the angles
        ids = torch.argsort(angles, dim=1)
        # Gather the points in the sorted order along the points dimension
        points = torch.gather(points, 1, ids.unsqueeze(-1).expand(-1, -1, points.size(2)))
        return points

    def get_random_points(self, num_envs: int, seeds=0) -> torch.Tensor:
        """Create n random points in the unit square, which are at least *mindst* apart, then scale them.

        Args:
            num_envs: The number of environments to generate random points for.
            seeds: The seed to use for the random number generator. This is not used yet. We need to implement our own
                multinomial sampling function to make this work.

        Returns:
            A 3D tensor of shape [num_envs, num_points, 2]."""

        # This creates an artificial grid to sample from
        # Generate equal probability weights for each cell in the grid
        weights = torch.ones((num_envs, self._num_cells * self._num_cells), device=self._device)
        # Using a multinomial distribution, sample N cells without replacement
        ids = torch.multinomial(weights, num_samples=self._max_num_points, replacement=False)
        # Compute the x and y coordinates of the sampled cells
        x = ids % self._num_cells
        y = ids // self._num_cells
        # Add noise to the coordinates so that the problem becomes continuous
        noise = torch.rand((num_envs, self._max_num_points, 2), device=self._device) * self._min_point_distance
        xy = torch.stack([x, y], dim=2) * self._min_point_distance * 2 + noise
        return xy

    @staticmethod
    def cast_to_0_2pi(ang: torch.Tensor) -> torch.Tensor:
        """Make sure the angles are in the range [0, 2*pi].

        Args:
            ang: A tensor of angles in radians. Dim is [num_envs, num_points].

        Returns:
            A tensor of angles in the range [0, 2*pi]. Dim is [num_envs, num_points]."""

        return (ang >= 0) * ang + (ang < 0) * (ang + 2 * math.pi)

    def get_curve_tangents(self, points: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Given a set of points, compute the tangents of the curve that passes through them.

        Args:
            points: A 3D tensor of shape [num_envs, num_points, 2].

        Returns:
            A tuple containing the points and the angles of the tangents.
            The points are a 3D tensor of shape [num_envs, num_points, 2].
            The angles are a 2D tensor of shape [num_envs, num_points]."""

        # Sort the points in the trigonometric direction
        points = self.ccw_sort(points)
        # Add the first point to the end to close the loop
        points = torch.cat([points, points[:, 0, :].unsqueeze(1)], dim=1)
        # Compute the difference between the points
        dist = torch.diff(points, dim=1)
        # Compute the angle between the points
        ang = torch.arctan2(dist[:, :, 1], dist[:, :, 0])
        # Make sure the angles are in the range [0, 2*pi]
        ang = self.cast_to_0_2pi(ang)
        # Compute the angle of the tangents
        ang1 = ang
        ang2 = torch.roll(ang, 1, dims=1)
        ang = self._p * ang1 + (1 - self._p) * ang2 + (torch.abs(ang2 - ang1) > np.pi) * np.pi
        return points[:, :-1], ang

    @staticmethod
    def compute_angle(points: torch.Tensor) -> torch.Tensor:
        """
        Compute the angle between the different segments of the curve.

        Args:
            points: A 3D tensor of shape [num_envs, num_points, 2].

        Returns:
            A 2D tensor of shape [num_envs, num_points]."""

        # Create 3 points ordered in sequence
        p0 = torch.roll(points, -1, dims=1)
        p1 = points
        p2 = torch.roll(points, 1, dims=1)
        # Generate 2 vectors from them
        v1 = p1 - p0
        v2 = p1 - p2
        # Compute the angle between them
        angles = torch.arccos(
            torch.einsum("ijk,ijk->ij", v1, v2) / (torch.linalg.norm(v1, axis=2) * torch.linalg.norm(v2, axis=2))
        )
        return angles

    def compute_angle_unsorted(self, points: torch.Tensor) -> torch.Tensor:
        """
        Compute the angle between the different segments of the curve.

        Args:
            points: A 3D tensor of shape [num_envs, num_points, 2].

        Returns:
            A 2D tensor of shape [num_envs, num_points]."""

        # Sort the points in the trigonometric direction
        points = self.ccw_sort(points)
        # Create 3 points ordered in sequence
        p0 = torch.roll(points, -1, dims=1)
        p1 = points
        p2 = torch.roll(points, 1, dims=1)
        # Generate 2 vectors from them
        v1 = p1 - p0
        v2 = p1 - p2
        # Compute the angle between them
        angles = torch.arccos(
            torch.einsum("ijk,ijk->ij", v1, v2) / (torch.linalg.norm(v1, axis=2) * torch.linalg.norm(v2, axis=2))
        )
        return angles

    def get_segment(
        self, point_1: torch.Tensor, point_2: torch.Tensor, angle_1: torch.Tensor, angle_2: torch.Tensor
    ) -> torch.Tensor:
        """Given two points and their angles, compute the bezier curve that passes through them.

        Args:
            point_1: A tensor of shape [num_envs, 2].
            point_2: A tensor of shape [num_envs, 2].
            angle_1: A tensor of shape [num_envs].
            angle_2: A tensor of shape [num_envs].

        Returns:
            A tensor of shape [num_envs, num_points*num_points_per_segment, 2]."""

        # Get the first two points
        p0 = point_1
        p3 = point_2
        # Compute the distance between the points
        d = torch.sqrt(torch.sum((p3 - p0) ** 2, axis=1))
        # Compute the intermediate points
        p1 = p0 + torch.stack([torch.cos(angle_1), torch.sin(angle_1)], dim=1) * self._rad * d.unsqueeze(-1)
        p2 = p3 + torch.stack([-torch.cos(angle_2), -torch.sin(angle_2)], dim=1) * self._rad * d.unsqueeze(-1)
        # Generate the bezier curve
        return self.bezier(points=[p0, p1, p2, p3])

    @staticmethod
    def batch_outer_product(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
        """Compute the outer product of two tensors.

        Args:
            a: A tensor of shape [b, n, m].
            b: A tensor of shape [b, n, m].

        Returns:
            A tensor of shape [b, n, m]."""

        return torch.einsum("ij,ik->ijk", a, b)

    def bezier(self, points: list[torch.Tensor]) -> torch.Tensor:
        """
        Compute the bezier curve that passes through the given points.

        Args:
            points: A list of 4 tensors. Each tensor has shape [num_envs, 2].

        Returns:
            A tensor of shape [num_envs, num_points_per_segment, 2]."""

        # Compute the bezier curve
        curve = torch.zeros((points[0].shape[0], self._num_points_per_segment, 2), device=self._device)
        curve += self.batch_outer_product(
            self._bernstein_0.unsqueeze(dim=0).expand(points[0].shape[0], -1),
            points[0],
        )
        curve += self.batch_outer_product(
            self._bernstein_1.unsqueeze(dim=0).expand(points[1].shape[0], -1),
            points[1],
        )
        curve += self.batch_outer_product(
            self._bernstein_2.unsqueeze(dim=0).expand(points[2].shape[0], -1),
            points[2],
        )
        curve += self.batch_outer_product(
            self._bernstein_3.unsqueeze(dim=0).expand(points[3].shape[0], -1),
            points[3],
        )
        return curve

    def get_bezier_curve(self, points: torch.Tensor, angles: torch.Tensor) -> torch.Tensor:
        """Given an array of points, create a curve through those points.

        Args:
            points: A 3D tensor of shape [num_envs, num_points, 2].
            angles: A 2D tensor of shape [num_envs, num_points].

        Returns:
            A tensor of shape [num_envs, num_points*num_points_per_segment, 2]."""

        # For each ordered pair of points, compute the bezier curve that passes through them
        segments = []
        for i in range(points.shape[1] - 1):
            seg = self.get_segment(points[:, i, :2], points[:, i + 1, :2], angles[:, i], angles[:, i + 1])
            segments.append(seg)
        seg = self.get_segment(points[:, -1, :2], points[:, 0, :2], angles[:, -1], angles[:, 0])
        segments.append(seg)
        curve = torch.cat(segments, dim=1)
        return curve

    def generate_tracks(self, num_tracks: int) -> dict:
        """Generate random tracks.

        Args:
            num_tracks: The number of tracks to generate.

        Returns:
            A dictionary containing the tracks."""

        points, tangents = self.generate_tracks_points(num_tracks)
        curve = self.get_bezier_curve(points, tangents)
        return points, tangents, curve

    def generate_tracks_points(
        self, num_tracks: int, prev_points: torch.Tensor = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Generate random tracks but only return the points.

        Args:
            num_tracks: The number of tracks to generate.
            prev_points: The previous points to append the new points to.

        Returns:
            A tuple containing the points and the tangents."""
        # Generate twice as many tracks as needed.
        # This is because some tracks may be discarded if they do not meet the minimum angle requirement.
        num_tracks_tmp = num_tracks * 2
        if num_tracks_tmp < 100:
            num_tracks_tmp = 100
        # Generate random points
        points = self.get_random_points(num_tracks_tmp)
        # Check if the angles created by the different segments are greater than the minimum angle allowed
        angles = self.compute_angle_unsorted(points)
        keep = torch.prod(angles > self._min_angle, dim=1) != 0
        # Keep only the tracks that meet the minimum angle requirement
        points = points[keep]
        # Get the number of points in the previous batch
        if prev_points is not None:
            num_prev = prev_points.shape[0]
            points = torch.cat([prev_points, points], dim=0)
        else:
            num_prev = 0
        # If the number of points is less than the number of tracks, generate more points
        tangents = None
        if (keep.sum() + num_prev) < num_tracks:
            points, tangents = self.generate_tracks_points(num_tracks, prev_points=points)
        # If the number of points is greater than the number of tracks, discard the extra points
        if (keep.sum() + num_prev) >= num_tracks:
            points = points[:num_tracks]
            points, tangents = self.get_curve_tangents(points)
        return points, tangents


if __name__ == "__main__":
    rad = 0.2
    edgy = 0.0

    N = 5

    num_tracks = 4096
    num_loops = 1000

    TG = TrackGenerator()
    s = time.time()
    for i in range(num_loops):
        points, tangents, curve = TG.generate_tracks(num_tracks)
    e = time.time()
    print("Time taken:", e - s)
    print("Time per track:", (e - s) / (num_tracks * num_loops))
    print("Tracks per second:", num_tracks * num_loops / (e - s))

    data = {}
    for i in range(1000):
        data[i] = {}
        data[i]["way_points"] = points[i].cpu()
        data[i]["tangent"] = tangents[i].cpu()
        data[i]["trajectory"] = curve[i].cpu()
        data[i]["complexity"] = np.random.rand(1)[0]

    for k in range(20):
        fig, ax = plt.subplots(N, N, figsize=(20, 20))
        for i in range(N):
            for j in range(N):
                idx = i * N + j + k * N * N
                x = data[idx]["trajectory"][:, 0]
                y = data[idx]["trajectory"][:, 1]
                a = data[idx]["way_points"]
                ax[i, j].set_aspect("equal")
                ax[i, j].plot(x, y, lw=2, c="r")
                ax[i, j].scatter(a[:, 0], a[:, 1], c="k")
                ax[i, j].set_title("Track #" + str(idx))
        plt.savefig("bezier_" + str(k) + ".png")
        plt.close()

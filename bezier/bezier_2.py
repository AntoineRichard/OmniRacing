# Copyright (c) Stack Exchange, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the CC BY-SA 3.0 license.

import time
import torch
import numpy as np
import numba as nb
from scipy.special import binom
import matplotlib.pyplot as plt
from geo_complexity import complexity
import pickle
import matplotlib.cm as cm
import math

bernstein = lambda n, k, t: binom(n, k) * t**k * (1.0 - t) ** (n - k)


class TrackGenerator:
    def __init__(self):
        self.min_num_points = 9
        self.max_num_points = 13
        self.min_point_distance = 0.05
        self.scale = 1.0
        self.rad = 0.2
        self.edgy = 0.0
        self.device = "cuda"
        self.num_points_per_segment = 30

        # Compute the angle between the two segments map it to [0, 1]
        self.p = math.atan(self.edgy) / math.pi + 0.5
        # Compute the number of cells in the grid
        self.num_cells = int(self.scale / (self.min_point_distance * 2))
        # Precompute the bernstein polynomials
        t = np.linspace(0, 1, num=self.num_points_per_segment)
        self.bernstein_0 = torch.tensor(bernstein(3, 0, t), device=self.device)
        self.bernstein_1 = torch.tensor(bernstein(3, 1, t), device=self.device)
        self.bernstein_2 = torch.tensor(bernstein(3, 2, t), device=self.device)
        self.bernstein_3 = torch.tensor(bernstein(3, 3, t), device=self.device)

    @staticmethod
    def ccw_sort(points: torch.Tensor):
        """Computes the mean of all the points, and orders them in the trigonometric direction around it.

        Args:
            points: A 3D tensor, [num_envs, num_points, 2]."""
        mean = torch.mean(points, axis=1)
        dist = points - mean.unsqueeze(1)
        angles = torch.arctan2(dist[:, :, 0], dist[:, :, 1])
        ids = torch.argsort(angles, dim=1)
        points = torch.gather(
            points, 1, ids.unsqueeze(-1).expand(-1, -1, points.size(2))
        )
        return points

    def get_random_points(self, num_envs, seeds=0):
        weights = torch.ones(
            (num_envs, self.num_cells * self.num_cells), device=self.device
        )
        ids = torch.multinomial(
            weights, num_samples=self.max_num_points, replacement=False
        )
        x = ids % self.num_cells
        y = ids // self.num_cells
        noise = (
            torch.rand((num_envs, self.max_num_points, 2), device=self.device)
            * self.min_point_distance
        )
        xy = torch.stack([x, y], dim=2) * self.min_point_distance * 2 + noise
        return xy

    def cast_to_0_2pi(self, ang):
        return (ang >= 0) * ang + (ang < 0) * (ang + 2 * math.pi)

    def get_curve_tangents(self, points):
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
        ang2 = torch.roll(ang, 0)
        ang = (
            self.p * ang1
            + (1 - self.p) * ang2
            + (torch.abs(ang2 - ang1) > np.pi) * np.pi
        )
        return points[:, :-1], ang

    @staticmethod
    def compute_angle(points):
        # Create 3 points
        p0 = torch.roll(points, -1, dims=1)
        p1 = points
        p2 = torch.roll(points, 1, dims=1)
        # Generate 2 vectors from them
        v1 = p1 - p0
        v2 = p1 - p2
        # Compute the angle between them
        angles = torch.arccos(
            torch.einsum("ijk,ijk->ij", v1, v2)
            / (torch.linalg.norm(v1, axis=2) * torch.linalg.norm(v2, axis=2))
        )
        return angles

    def get_segment(self, point_1, point_2, angle_1, angle_2):
        p0 = point_1
        p3 = point_2
        d = torch.sqrt(torch.sum((p3 - p0) ** 2, axis=1))
        p1 = p0 + torch.stack(
            [torch.cos(angle_1), torch.sin(angle_1)], dim=1
        ) * self.rad * d.unsqueeze(-1)
        p2 = p3 + torch.stack(
            [-torch.cos(angle_2), -torch.sin(angle_2)], dim=1
        ) * self.rad * d.unsqueeze(-1)
        return self.bezier(points=[p0, p1, p2, p3])

    @staticmethod
    def batch_outer_product(a, b):
        return torch.einsum("ij,ik->ijk", a, b)

    def bezier(self, points):
        curve = torch.zeros(
            (points[0].shape[0], self.num_points_per_segment, 2), device=self.device
        )
        curve += self.batch_outer_product(
            self.bernstein_0.unsqueeze(dim=0).expand(points[0].shape[0], -1),
            points[0],
        )
        curve += self.batch_outer_product(
            self.bernstein_1.unsqueeze(dim=0).expand(points[1].shape[0], -1),
            points[1],
        )
        curve += self.batch_outer_product(
            self.bernstein_2.unsqueeze(dim=0).expand(points[2].shape[0], -1),
            points[2],
        )
        curve += self.batch_outer_product(
            self.bernstein_3.unsqueeze(dim=0).expand(points[3].shape[0], -1),
            points[3],
        )
        return curve

    def get_bezier_curve(self, points, angles):
        segments = []
        for i in range(points.shape[1] - 1):
            seg = self.get_segment(
                points[:, i, :2], points[:, i + 1, :2], angles[:, i], angles[:, i + 1]
            )
            segments.append(seg)
        seg = self.get_segment(
            points[:, -1, :2], points[:, 0, :2], angles[:, -1], angles[:, 0]
        )
        segments.append(seg)
        curve = torch.cat(segments, dim=1)
        return segments, curve


def bezier(points, num=200):
    N = len(points)
    t = np.linspace(0, 1, num=num)
    curve = np.zeros((num, 2))
    for i in range(N):
        curve += np.outer(bernstein(N - 1, i, t), points[i])
    return curve


class Segment:
    def __init__(self, p1, p2, angle1, angle2, **kw):
        self.p1 = p1
        self.p2 = p2
        self.angle1 = angle1
        self.angle2 = angle2
        self.numpoints = kw.get("numpoints", 100)
        r = kw.get("r", 0.3)
        d = np.sqrt(np.sum((self.p2 - self.p1) ** 2))
        self.r = r * d
        self.p = np.zeros((4, 2))
        self.p[0, :] = self.p1[:]
        self.p[3, :] = self.p2[:]
        self.calc_intermediate_points(self.r)

    def calc_intermediate_points(self, r):
        self.p[1, :] = self.p1 + np.array(
            [self.r * np.cos(self.angle1), self.r * np.sin(self.angle1)]
        )
        self.p[2, :] = self.p2 + np.array(
            [self.r * np.cos(self.angle2 + np.pi), self.r * np.sin(self.angle2 + np.pi)]
        )
        self.curve = bezier(self.p, self.numpoints)


def get_curve(points, **kw):
    segments = []
    for i in range(len(points) - 1):
        seg = Segment(
            points[i, :2], points[i + 1, :2], points[i, 2], points[i + 1, 2], **kw
        )
        segments.append(seg)
    curve = np.concatenate([s.curve for s in segments])
    return segments, curve


def ccw_sort(p):
    d = p - np.mean(p, axis=0)
    s = np.arctan2(d[:, 0], d[:, 1])
    return p[np.argsort(s), :]


def get_bezier_curve(a=None, rad=0.2, edgy=0, **kw):
    """Given an array of points *a*, create a curve through
    those points.
    *rad* is a number between 0 and 1 to steer the distance of
          control points.
    *edgy* is a parameter which controls how "edgy" the curve is,
           edgy=0 is smoothest."""
    if a is None:
        a = get_random_points(**kw)

    numpoints = kw.get("numpoints", 30)

    p = np.arctan(edgy) / np.pi + 0.5
    a = ccw_sort(a)
    a = np.append(a, np.atleast_2d(a[0, :]), axis=0)
    d = np.diff(a, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    f = lambda ang: (ang >= 0) * ang + (ang < 0) * (ang + 2 * np.pi)
    ang = f(ang)
    ang1 = ang
    ang2 = np.roll(ang, 1)
    ang = p * ang1 + (1 - p) * ang2 + (np.abs(ang2 - ang1) > np.pi) * np.pi
    ang = np.append(ang, [ang[0]])
    a = np.append(a, np.atleast_2d(ang).T, axis=1)
    s, c = get_curve(a, r=rad, method="var", numpoints=numpoints)
    x, y = c.T
    return x, y, a


def get_bezier_curve_2(a=None, rad=0.2, edgy=0, **kw):
    """Given an array of points *a*, create a curve through
    those points.
    *rad* is a number between 0 and 1 to steer the distance of
          control points.
    *edgy* is a parameter which controls how "edgy" the curve is,
           edgy=0 is smoothest."""

    p = np.arctan(edgy) / np.pi + 0.5
    a = ccw_sort(a)
    a = np.append(a, np.atleast_2d(a[0, :]), axis=0)
    d = np.diff(a, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    f = lambda ang: (ang >= 0) * ang + (ang < 0) * (ang + 2 * np.pi)
    ang = f(ang)
    ang1 = ang
    ang2 = np.roll(ang, 1)
    ang = p * ang1 + (1 - p) * ang2 + (np.abs(ang2 - ang1) > np.pi) * np.pi
    ang = np.append(ang, [ang[0]])
    a = np.append(a, np.atleast_2d(ang).T, axis=1)
    # s, c = get_curve(a, r=rad, method="var", numpoints=numpoints)
    # x, y = c.T
    return a


def get_random_points(n=5, scale=0.8, mindst=None, rec=0, **kw):
    """Create n random points in the unit square, which are *mindst*
    apart, then scale them."""
    mindst = mindst or 0.7 / n
    np_random = kw.get("np_random", np.random)
    a = np_random.rand(n, 2)

    d = np.sqrt(np.sum(np.diff(ccw_sort(a), axis=0), axis=1) ** 2)
    if np.all(d >= mindst) or rec >= 200:
        return a * scale
    else:
        return get_random_points(
            n=n, scale=scale, mindst=mindst, rec=rec + 1, np_random=np_random
        )


def get_random_points_2(n=5, scale=0.8, mindst=None, rec=0, seed=0):
    """Create n random points in the unit square, which are *mindst*
    apart, then scale them."""
    mindst = mindst or 0.7 / n
    # Make a grid to sample from
    num_cell = int(scale / (mindst * 2))

    rng = np.random.default_rng(seed)
    idx = rng.choice(num_cell * num_cell, n, replace=False)
    # idx = rng.integers(0, num_cell * num_cell, n)
    x = idx % num_cell
    y = idx // num_cell
    noise = rng.uniform(0, mindst, n * 2)
    x = x * mindst * 2 + noise[:n]
    y = y * mindst * 2 + noise[n:]

    return np.stack([x, y], axis=1)  # , idx, noise, num_cell, mindst


def compute_angle(xy):
    p0 = np.roll(xy, -1, axis=0)
    p1 = xy
    p2 = np.roll(xy, 1, axis=0)
    v1 = p1 - p0
    v2 = p1 - p2

    angles = np.arccos(
        np.einsum("ij,ij->i", v1, v2)
        / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1))
    )

    return angles


if __name__ == "__main__":
    # fig, ax = plt.subplots()
    # ax.set_aspect("equal")

    rad = 0.2
    edgy = 0.0

    N = 5

    # points = []
    # angles = []
    # xs = []
    # ys = []
    # tan = []
    # comps = []

    TG = TrackGenerator()
    points = TG.get_random_points(1000)
    points, tangents = TG.get_curve_tangents(points)
    segments, curve = TG.get_bezier_curve(points, tangents)
    angles = TG.compute_angle(points)
    s = time.time()
    for i in range(1000):
        points = TG.get_random_points(1000)
        points, tangents = TG.get_curve_tangents(points)
        segments, curve = TG.get_bezier_curve(points, tangents)
        angles = TG.compute_angle(points)
    e = time.time()

    print("tensorized total time", (e - s) / 1000.0)
    data = {}
    for i in range(1000):
        data[i] = {}
        data[i]["way_points"] = points[i].cpu()
        data[i]["tangent"] = angles[i].cpu()
        data[i]["trajectory"] = curve[i].cpu()
        data[i]["angle"] = angles[i].cpu()
        data[i]["complexity"] = np.random.rand(1)[0]

    # exit(0)
    # s = time.time()
    # i = 0
    # k = 0
    # data = {}
    # while i < 1000:
    #    rng = np.random.default_rng(k)
    #    n_turns = rng.integers(9, 13)
    #    a = get_random_points_2(n=n_turns, scale=1, seed=k)
    #    x, y, a = get_bezier_curve(a, rad=rad, edgy=edgy)
    #    angles = compute_angle(a[:-1, :2])
    #    comp = complexity(np.stack([x, y], axis=1))["complexity"]
    #    if np.all(angles > (np.pi * 12.5 / 180)):
    #        data[i] = {}
    #        data[i]["way_points"] = a[:-1, :2]
    #        data[i]["tangent"] = a[:-1, 2]
    #        data[i]["complexity"] = comp
    #        data[i]["trajectory"] = np.stack([x, y], axis=1)
    #        data[i]["angles"] = angles
    #        i += 1
    #    k += 1
    # rng = np.random.default_rng(0)
    # for i in range(1000):
    #    n_turns = rng.integers(9, 13)
    #    a = get_random_points_2(n_turns, scale=1, seed=i)
    e = time.time()

    print("regular total time", (e - s))

    comps = [data[i]["complexity"] for i in range(len(data))]
    normed_comps = (np.array(comps) - np.min(comps)) / (np.max(comps) - np.min(comps))
    for i in range(len(data)):
        data[i]["normed_complexity"] = normed_comps[i]
    data_light = {
        i: {k: v for k, v in data[i].items() if (k != "trajectory") and (k != "angles")}
        for i in range(len(data))
    }
    with open("bezier_data.pkl", "wb") as f:
        pickle.dump(data, f)
    with open("bezier_data_light.pkl", "wb") as f:
        pickle.dump(data_light, f)
    print("total time", (e - s))

    cmap = cm.jet

    for k in range(20):
        fig, ax = plt.subplots(N, N, figsize=(20, 20))
        for i in range(N):
            for j in range(N):
                idx = i * N + j + k * N * N
                c = cmap(normed_comps[idx])
                x = data[idx]["trajectory"][:, 0]
                y = data[idx]["trajectory"][:, 1]
                a = data[idx]["way_points"]
                ax[i, j].set_aspect("equal")
                ax[i, j].plot(x, y, lw=2, c=c)
                ax[i, j].scatter(a[:, 0], a[:, 1], c="k")
                ax[i, j].set_title(
                    "Complexity: " + normed_comps[idx].round(5).astype(str)
                )
        plt.savefig("bezier_" + str(k) + ".png")
        plt.close()

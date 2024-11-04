# Copyright (c) Stack Exchange, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the CC BY-SA 3.0 license.

import time
import torch
import numpy as np
import numba as nb
import warp as wp
from scipy.special import binom
import matplotlib.pyplot as plt
import colorsys
from geo_complexity import complexity
import matplotlib
import pickle

bernstein = lambda n, k, t: binom(n, k) * t**k * (1.0 - t) ** (n - k)


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
    if a is None:
        a = get_random_points_2(**kw)

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


@numba.jit(nopython=True)
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
    # angles = np.zeros(len(xy) - 2)
    p0 = np.roll(xy, -1, axis=0)
    p1 = xy
    p2 = np.roll(xy, 1, axis=0)
    v1 = p1 - p0
    v2 = p1 - p2
    # angle = np.arccos(
    #    np.dot(v1, v2) / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1))
    # )
    angles = np.arccos(
        np.einsum("ij,ij->i", v1, v2)
        / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1))
    )

    # for i in range(len(xy) - 2):
    #    p0 = xy[i]
    #    p1 = xy[i + 1]
    #    p2 = xy[i + 2]
    #    v1 = p0 - p1
    #    v2 = p2 - p1
    #    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    #    if np.cross(v1, v2) < 0:
    #        angle = -angle
    #    angles[i] = angle
    return angles  # * 180 / np.pi


@wp.func
def get_random_points_wp(
    out: wp.array(dtype=float),
    n: wp.array(dtype=int),
    scale: float,
    min_dist: float,
    state: wp.uint32,
):
    num_cell = int(scale / (min_dist * 2))
    for i in range(n):
        wp.randi(state, 0, num_cell * num_cell)


if __name__ == "__main__":
    # fig, ax = plt.subplots()
    # ax.set_aspect("equal")

    rad = 0.2
    edgy = 0.0

    N = 5

    s = time.time()
    # points = []
    # angles = []
    # xs = []
    # ys = []
    # tan = []
    # comps = []
    i = 0
    k = 0
    data = {}
    while i < 100000:
        rng = np.random.default_rng(k)
        n_turns = rng.integers(9, 13)
        a = get_random_points_2(n=n_turns, scale=1, seed=k)
        x, y, a = get_bezier_curve(a, rad=rad, edgy=edgy)
        angles = compute_angle(a[:-1, :2])
        comp = complexity(np.stack([x, y], axis=1))["complexity"]
        if np.all(angles > (np.pi * 12.5 / 180)):
            data[i] = {}
            data[i]["way_points"] = a[:-1, :2]
            data[i]["tangent"] = a[:-1, 2]
            data[i]["complexity"] = comp
            data[i]["trajectory"] = np.stack([x, y], axis=1)
            data[i]["angles"] = angles
            i += 1
        k += 1
        e = time.time()
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

    cm = matplotlib.colormaps["jet"]

    for k in range(20):
        fig, ax = plt.subplots(N, N, figsize=(20, 20))
        for i in range(N):
            for j in range(N):
                idx = i * N + j + k * N * N
                c = cm(normed_comps[idx])
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

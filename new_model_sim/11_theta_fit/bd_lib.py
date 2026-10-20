"""Side-effect-free part of build_dataset.py (constants, helpers, LINK_REF). Generated; do not edit."""

import json
import math
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
RUN = D / "runs"
CASES = json.loads((D / "cases.json").read_text(encoding="utf-8"))
O = CASES["origin_m"]
WALLS = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
LINKS = [["wall_l1"], ["wall_l2"], ["wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]]
JOINTS = [([-4e-06, -0.0366, -0.460702], [0.0, 0.0, -1.0], WALLS[2:]),
          ([0.016686, -0.0666, -0.511202], [-1.0, 0.0, 0.0], WALLS[3:]),
          ([0.014796, -0.1226, -0.313202], [1.0, 0.0, 0.0], WALLS[4:])]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def rot(axis, ang):
    """Rotation matrix (rows) about a unit coordinate axis index 0..2."""
    c, s = math.cos(ang), math.sin(ang)
    if axis == 0:
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == 1:
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def axang(a, t):
    """Rotation matrix (rows) about unit vector a by angle t (Rodrigues)."""
    c, s, C = math.cos(t), math.sin(t), 1 - math.cos(t)
    x, y, z = a
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def mT(R, x):  # R^T x
    return [sum(R[k][i] * x[k] for k in range(3)) for i in range(3)]


def link_centroids():
    """Volume-weighted centroids of the link bodies (body_centroids.json, grid method)."""
    b = dict((x["name"], x) for x in json.loads((D / "body_centroids.json").read_text(encoding="utf-8"))["bodies"])
    groups = [["0:7955"], ["0:8167"], ["0:1425", "0:2393", "0:4783", "0:7716"]]
    out = []
    for g in groups:
        V = sum(b[n]["volume_m3"] for n in g)
        out.append([sum(b[n]["volume_m3"] * b[n]["centroid_m"][i] for n in g) / V for i in range(3)])
    return out


LINK_REF = link_centroids()


def sample(L, Ot, R):
    """Per-zone global loads L (moments about the global origin) -> body-frame quantities about O_b(t).
    flink: per link i the fluid force (3) then the fluid moment about the link reference point (3),
    body axes -- 6 values per link, 18 in total."""
    f, m = {}, {}
    for z in WALLS:
        fg = [L["f%s(%s)" % (q, z)] for q in "xyz"]
        mg = [L["m%s(%s)" % (q, z)] for q in "xyz"]
        f[z] = mT(R, fg)
        m[z] = mT(R, sub(mg, cross(Ot, fg)))          # about O_b(t), body axes
    F = [sum(f[z][i] for z in WALLS) for i in range(3)]
    M = [sum(m[z][i] for z in WALLS) for i in range(3)]
    tau = []
    for c, a, zs in JOINTS:
        cb = sub(c, O)                                   # joint centre relative to O_b, body frame
        tau.append(sum(dot(a, sub(m[z], cross(cb, f[z]))) for z in zs))
    flink = []
    for zs, ref in zip(LINKS, LINK_REF):
        rb = sub(ref, O)                                 # link reference point relative to O_b
        fl = [sum(f[z][i] for z in zs) for i in range(3)]
        ml = [sum(m[z][i] for z in zs) for i in range(3)]
        flink += fl + sub(ml, cross(rb, fl))
    return F, M, tau, flink


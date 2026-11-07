"""Analytic new-CAD kinematics for stage 15; importing this module starts no solver.

``pose_matrix`` maps unposed CAD0 coordinates to q.  Meshes already built at qe
must instead use ``relative_transform``.  Groups 1/2/3 are CAD chain depths,
not three independently moving Fluent zones: this stage locks J2 and J3.
"""

from __future__ import annotations

import json
import numbers
from collections.abc import Mapping
from pathlib import Path

import numpy as np


CONTRACT_PATH = Path(__file__).with_name("contract.json")
PART_GROUPS = {
    "hull": 0, "base0": 0, "wall_auv": 0, "wall_base0": 0,
    "fluid_background": 0,
    "l1": 1, "wall_l1": 1,
    "l2": 2, "wall_l2": 2,
    "link3": 3, "pin4": 3, "jaw1": 3, "jaw2": 3,
    "wall_link3": 3, "wall_pin4": 3, "wall_jaw1": 3, "wall_jaw2": 3,
    "fluid_arm": 3, "overset_arm": 3,
}


def _scalar(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or raw.shape != shape:
            raise ValueError(f"{name} must be a real array with shape {shape}")
        result = raw.astype(np.float64, copy=False)
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _validate_contract(contract):
    if not isinstance(contract, Mapping):
        raise ValueError("contract must be a mapping")
    try:
        frame, law, source = contract["frame"], contract["motion"], contract["source"]
        if frame["length_unit"] != "m" or frame["angle_unit"] != "rad":
            raise ValueError("stage 15 requires metres and radians")
        centres = _array(frame["joint_centres_CAD0_m"], (3, 3), "joint centres")
        axes = _array(frame["joint_axes_CAD0"], (3, 3), "joint axes")
        if not np.allclose(np.linalg.norm(axes, axis=1), 1.0, rtol=0, atol=1e-12):
            raise ValueError("joint axes must be unit vectors")
        if not np.allclose(axes[0], [0, 0, -1], rtol=0, atol=1e-12):
            raise ValueError("this stage requires the new CAD J1 axis -Z")
        q0 = _array(law["initial_q_rad"], (3,), "initial_q_rad")
        qe = _array(source["pose_rad"], (3,), "source pose_rad")
        if not np.allclose(q0, qe, rtol=0, atol=1e-12):
            raise ValueError("initial mesh pose must equal the motion initial pose")
        active = law["active_joint_indices_1based"]
        if (not isinstance(active, (list, tuple)) or len(active) != 1
                or isinstance(active[0], (bool, np.bool_))
                or not isinstance(active[0], numbers.Integral) or active[0] != 1):
            raise ValueError("stage 15 only activates J1; J2/J3 must remain locked")
        origin = _array(law["reference_point_m"], (3,), "reference_point_m")
        if not np.allclose(origin, centres[0], rtol=0, atol=1e-12):
            raise ValueError("the configured Fluent reference point must be the J1 axis origin")
        if _scalar(law["period_s"], "period_s") <= 0:
            raise ValueError("period_s must be positive")
        if _scalar(law["half_excursion_rad"], "half_excursion_rad") <= 0:
            raise ValueError("half_excursion_rad must be positive")
        _scalar(law["start_time_s"], "start_time_s")
    except KeyError as exc:
        raise ValueError(f"required contract field missing: {exc}") from exc
    return contract


def load_contract(path=None):
    """Read and validate a fresh contract; never cache mutable caller state."""
    with Path(path if path is not None else CONTRACT_PATH).open(encoding="utf-8-sig") as stream:
        result = json.load(stream)
    return _validate_contract(result)


def _contract(contract):
    return load_contract() if contract is None else _validate_contract(contract)


def _group_index(group):
    if isinstance(group, str):
        if group not in PART_GROUPS:
            raise ValueError(f"unknown CAD/motion group: {group!r}")
        return PART_GROUPS[group]
    if isinstance(group, numbers.Integral) and not isinstance(group, (bool, np.bool_)):
        group = int(group)
        if 0 <= group <= 3:
            return group
    raise ValueError("group must be 0..3 or a known stage-15 part/zone name")


def joint_state(t, contract=None):
    """Return q, dq/dt, d2q/dt2 (three float64 vectors) at absolute time t.

    The analytic law is periodic, including outside the authorized run window;
    authorizing simulation times remains the runner's responsibility.
    """
    contract = _contract(contract)
    t = _scalar(t, "t")
    law = contract["motion"]
    frequency = 2.0 * np.pi / law["period_s"]
    phase = frequency * (t - law["start_time_s"])
    if not np.isfinite(phase):
        raise ValueError("phase is not finite")
    amplitude = law["half_excursion_rad"]
    q = np.array(law["initial_q_rad"], dtype=np.float64, copy=True)
    qd, qdd = np.zeros(3), np.zeros(3)
    q[0] += amplitude * (1.0 - np.cos(phase))
    qd[0] = amplitude * frequency * np.sin(phase)
    qdd[0] = amplitude * frequency * frequency * np.cos(phase)
    if not all(np.all(np.isfinite(x)) for x in (q, qd, qdd)):
        raise ValueError("non-finite joint state")
    return {"q_rad": q, "qd_rad_s": qd, "qdd_rad_s2": qdd}


def _axis_transform(axis, centre, angle):
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    rotation = (np.eye(3) + np.sin(angle) * skew
                + (1.0 - np.cos(angle)) * (skew @ skew))
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = centre - rotation @ centre
    return result


def pose_matrix(group, q, contract=None):
    """CAD0 -> posed frame, A1(q1) @ A2(q2) @ ... for a chain depth.

    The distal pre-pose 10-micrometre CAD repair is already in the supplied
    geometry and is intentionally not applied a second time here.
    """
    contract = _contract(contract)
    depth = _group_index(group)
    q = _array(q, (3,), "q")
    frame = contract["frame"]
    centres = np.asarray(frame["joint_centres_CAD0_m"], dtype=np.float64)
    axes = np.asarray(frame["joint_axes_CAD0"], dtype=np.float64)
    transform = np.eye(4)
    for j in range(depth):
        transform = transform @ _axis_transform(axes[j], centres[j], q[j])
    return transform


def relative_transform(t, group=3, contract=None):
    """Map already-posed qe mesh x0 to x(t): T(q(t)) @ inverse(T(q0))."""
    contract = _contract(contract)
    q = joint_state(t, contract)["q_rad"]
    initial = pose_matrix(group, contract["motion"]["initial_q_rad"], contract)
    current = pose_matrix(group, q, contract)
    inverse = np.eye(4)
    inverse[:3, :3] = initial[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ initial[:3, 3]
    return current @ inverse


def validate_transform(transform):
    """Validate a finite, proper homogeneous rigid transform and return float64."""
    transform = _array(transform, (4, 4), "transform")
    if not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-12):
        raise ValueError("transform must have homogeneous last row [0,0,0,1]")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-10)
            or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-10)):
        raise ValueError("transform rotation must be proper and orthonormal")
    return transform


def transform_points(points, T):
    """Apply a proper homogeneous transform to (N,3) points or one (3,) point."""
    T = validate_transform(T)
    raw = np.asarray(points)
    if raw.shape == (3,):
        points = _array(raw, (3,), "points")
    elif raw.ndim == 2 and raw.shape[1] == 3 and raw.shape[0] > 0:
        points = _array(raw, raw.shape, "points")
    else:
        raise ValueError("points must have nonempty shape (N,3) or (3,)")
    with np.errstate(over="raise", invalid="raise"):
        try:
            result = points @ T[:3, :3].T + T[:3, 3]
        except FloatingPointError as exc:
            raise ValueError("transformed coordinates are not finite") from exc
    return result


def j1_reference_position(t, reference0_m, contract=None):
    """Absolute analytic position of an arbitrary reference fixed in the qe arm."""
    contract = _contract(contract)
    reference0 = _array(reference0_m, (3,), "reference0_m")
    return transform_points(reference0, relative_transform(t, 1, contract))


def j1_reference_velocity(t, reference0_m, contract=None):
    """omega(t) cross (C(t)-p); C(t) is analytic, never callback-accumulated.

    When reference0_m is the explicitly configured J1 axis origin, this is zero.
    The current Fluent UDF only authorizes that zero-translation configuration.
    """
    contract = _contract(contract)
    current = j1_reference_position(t, reference0_m, contract)
    frame = contract["frame"]
    centre = np.asarray(frame["joint_centres_CAD0_m"][0], dtype=np.float64)
    axis = np.asarray(frame["joint_axes_CAD0"][0], dtype=np.float64)
    omega = axis * joint_state(t, contract)["qd_rad_s"][0]
    return np.cross(omega, current - centre)

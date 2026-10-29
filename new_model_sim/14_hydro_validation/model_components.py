"""Stage-14 split hydrodynamics. Physical loads ON bodies; SI units.

The three dictionaries are fixed from bounding boxes, not measured labels.
Virtual ports and normals are modelling assumptions, not recovered CAD faces.
No added-mass/inertial model is fitted here. Existing dynamics are untouched.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

MODEL_SCHEMA = "split_hydro_static_v1"
BODY_ORDER = ("hull", "base0", "link1", "link2", "link3")
CHANNELS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


def vector(value, n, name):
    a = np.asarray(value, dtype=float)
    if a.size != n or a.ndim > 2 or (a.ndim == 2 and 1 not in a.shape):
        raise ValueError(f"{name} must be a finite {n}-vector")
    a = a.reshape(n)
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite")
    return a


def skew(x):
    x = vector(x, 3, "skew input")
    return np.array([[0., -x[2], x[1]], [x[2], 0., -x[0]], [-x[1], x[0], 0.]])


def rotation(axis, angle):
    a = vector(axis, 3, "axis")
    norm = np.linalg.norm(a)
    if norm < 1e-12 or not np.isfinite(angle):
        raise ValueError("invalid rotation")
    K = skew(a / norm)
    return np.eye(3) + np.sin(angle) * K + (1. - np.cos(angle)) * (K @ K)


def link_kinematics(q, geometry):
    """Original uvms_link_kin geometry and 6x9 Jacobians, no new COM shift."""
    q = vector(q, 3, "q")
    origin = vector(geometry["origin_Ob_CAD_m"], 3, "origin")
    centres = np.asarray(geometry["joint_centres_CAD0_m"], float) - origin
    axes = np.asarray(geometry["joint_axes_CAD0"], float)
    refs = np.asarray(geometry["link_reference_CAD0_m"], float) - origin
    if any(x.shape != (3, 3) or not np.isfinite(x).all() for x in (centres, axes, refs)):
        raise ValueError("invalid geometry matrices")
    R, t = np.eye(3), np.zeros(3)
    P, Rs, pj, aj, Js = [], [], [], [], []
    for i in range(3):
        pj.append(R @ centres[i] + t)
        aj.append(R @ axes[i])
        Ri = rotation(aj[i], q[i])
        t = Ri @ (t - pj[i]) + pj[i]
        R = Ri @ R
        p = R @ refs[i] + t
        Jb = np.zeros((6, 9))
        Jb[:3, :3] = np.eye(3)
        Jb[:3, 3:6] = -skew(p)
        Jb[3:6, 3:6] = np.eye(3)
        for j in range(i + 1):
            Jb[:3, 6 + j] = np.cross(aj[j], p - pj[j])
            Jb[3:6, 6 + j] = aj[j]
        transform = np.zeros((6, 6))
        transform[:3, :3] = R.T
        transform[3:, 3:] = R.T
        Js.append(transform @ Jb)
        P.append(p)
        Rs.append(R.copy())
    return {"J": np.asarray(Js), "R": np.asarray(Rs), "P": np.asarray(P),
            "joint_positions": np.asarray(pj), "joint_axes": np.asarray(aj)}


def body_jacobians(q, geometry):
    result = np.zeros((5, 6, 9))
    result[0, :, :6] = np.eye(6)
    result[1, :, :6] = np.eye(6)
    result[2:] = link_kinematics(q, geometry)["J"]
    return result


def recompose(q, wrenches, geometry):
    """J^T combines local complete wrenches, giving exact power duality."""
    w = np.asarray(wrenches, float)
    if w.shape != (5, 6) or not np.isfinite(w).all():
        raise ValueError("wrenches must be finite 5x6")
    J = body_jacobians(q, geometry)
    generalized = np.einsum("bij,bi->j", J, w)
    return {"hull": w[0].copy(), "base0": w[1].copy(), "links": w[2:].copy(),
            "total": generalized[:6], "tau_hydro": generalized[6:],
            "generalized": generalized}


def build_template(contract, bodies, candidate):
    """A: nine odd drag ports/body; B: A + three even transverse terms/body.

    C: the same nine ports with six signed axis normals (18 coefficients/body).
    Each axis uses three fixed transverse triangle points inside the bbox.
    This minimal triangle avoids the structural rank redundancy of four corners.
    No geometry choice is made using any CFD label, including development labels.
    """
    if candidate not in ("A", "B", "C", "B_pair"):
        raise ValueError("candidate must be A, B, C or the bounded B_pair supplement")
    body_map = {b["id"]: b for b in bodies}
    if set(body_map) != set(BODY_ORDER):
        raise ValueError("body IDs must match contract")
    geometry_keys = ("origin_Ob_CAD_m", "joint_centres_CAD0_m", "joint_axes_CAD0",
                     "link_reference_CAD0_m")
    geometry = {k: contract[k] for k in geometry_keys}
    model = {"schema_version": MODEL_SCHEMA, "candidate": candidate,
             "load_sign": "fluid load ON bodies", "body_order": list(BODY_ORDER),
             "geometry": geometry, "bodies": [body_map[b] for b in BODY_ORDER],
             "dictionary_policy": "fixed bbox interior triangle ports; virtual axis normals, not actual CAD faces",
             "point_source": "bbox midpoint + 0.8*half-extent transverse triangle (angles 0,120,240 deg)",
             "coefficients": [], "parameters": [], "theta": [],
             "scope": {"steady_translation_only": True, "qdot_zero": True,
                       "angular_velocity_zero": True, "acceleration_zero": True,
                       "status": "unfitted", "independent_joint_motion_validated": False,
                       "inertial_model_included": False}}
    for body_index, body_id in enumerate(BODY_ORDER):
        body = body_map[body_id]
        lo = vector(body["bbox_local_min_m"], 3, "bbox min")
        hi = vector(body["bbox_local_max_m"], 3, "bbox max")
        if np.any(hi - lo <= 1e-9):
            raise ValueError(f"degenerate bbox: {body_id}")
        centre, half = (lo + hi) / 2, (hi - lo) / 2
        for axis in range(3):
            a, b = [j for j in range(3) if j != axis]
            for point_index, angle in enumerate((0., 2*np.pi/3, 4*np.pi/3)):
                point = centre.copy()
                point[a] += .8 * half[a] * np.cos(angle)
                point[b] += .8 * half[b] * np.sin(angle)
                signs = (1., -1.) if candidate == "C" else (1.,)
                for sign in signs:
                    normal = np.eye(3)[axis] * sign
                    kind = "one_sided_drag" if candidate == "C" else "odd_drag"
                    name = f"{body_id}.{kind}.axis{axis}.port{point_index}.sign{int(sign):+d}"
                    model["parameters"].append({"name": name, "unit": "kg/m", "block": body_id + "." + kind,
                        "body_index": body_index, "kind": kind, "point_m": point.tolist(),
                        "direction": normal.tolist(), "lower_bound": 0., "upper_bound": None})
        if candidate in ("B", "B_pair"):
            for axis in range(3):
                model["parameters"].append({"name": f"{body_id}.even_transverse.b{axis}",
                    "unit": "kg/m", "block": body_id + ".even_transverse", "body_index": body_index,
                    "kind": "even_transverse", "point_m": centre.tolist(),
                    "direction": np.eye(3)[axis].tolist(), "lower_bound": None, "upper_bound": None})
    if candidate == "B_pair":
        model["schema_version"] = "split_hydro_static_v2"
        model["bounded_supplement"] = "parent-requested B family supplement after ABC development; not blind/preregistered superiority"
        model["pair_point_policy"] = "link2/link3 only: bbox midpoint +/- 0.8*half-extent along longest bbox axis; geometry-only, no label-driven choice"
        for body_index in (3, 4):
            body_id = BODY_ORDER[body_index]
            body = body_map[body_id]
            lo, hi = np.asarray(body["bbox_local_min_m"]), np.asarray(body["bbox_local_max_m"])
            centre, half = (lo+hi)/2, (hi-lo)/2
            axis = int(np.argmax(half))
            offset = np.eye(3)[axis]*.8*half[axis]
            for k in range(3):
                model["parameters"].append({"name": f"{body_id}.even_pair.b{k}", "unit": "kg/m",
                    "block": body_id+".even_pair", "body_index": body_index, "kind": "even_pair",
                    "point_m": (centre+offset).tolist(), "other_point_m": (centre-offset).tolist(),
                    "direction": np.eye(3)[k].tolist(), "lower_bound": None, "upper_bound": None})
        # Record the optional second point uniformly; MATLAB accepts struct/cell decoding.
        for parameter in model["parameters"]:
            parameter.setdefault("other_point_m", parameter["point_m"])
    model["theta"] = [0.] * len(model["parameters"])
    del model["coefficients"]
    return model


def validate_model(model):
    if model.get("schema_version") not in (MODEL_SCHEMA, "split_hydro_static_v2"):
        raise ValueError("unsupported model schema")
    if model.get("body_order") != list(BODY_ORDER):
        raise ValueError("incorrect body order")
    theta = vector(model["theta"], len(model["parameters"]), "theta")
    for p, coefficient in zip(model["parameters"], theta):
        if p["kind"] not in ("odd_drag", "one_sided_drag", "even_transverse", "even_pair"):
            raise ValueError("unknown physical block")
        if p["kind"] == "even_pair":
            vector(p["other_point_m"], 3, "second pair port")
            if model["schema_version"] != "split_hydro_static_v2":
                raise ValueError("even_pair requires schema v2")
        if p["body_index"] not in range(5):
            raise ValueError("invalid body index")
        vector(p["point_m"], 3, "port point")
        normal = vector(p["direction"], 3, "direction")
        if not np.isclose(np.linalg.norm(normal), 1., atol=1e-10):
            raise ValueError("non-unit normal/direction")
        if p["kind"] in ("odd_drag", "one_sided_drag") and coefficient < -1e-10:
            raise ValueError("negative drag coefficient")
    return theta


def component_features(model, q, nu, qdot=None):
    """Linear parameter features: 5 bodies x 6 local wrench channels x P."""
    q, nu = vector(q, 3, "q"), vector(nu, 6, "nu")
    qdot = np.zeros(3) if qdot is None else vector(qdot, 3, "qdot")
    J = body_jacobians(q, model["geometry"])
    twists = J @ np.r_[nu, qdot]
    columns = np.zeros((5, 6, len(model["parameters"])))
    for k, p in enumerate(model["parameters"]):
        body = p["body_index"]
        r, n = np.asarray(p["point_m"]), np.asarray(p["direction"])
        vp = twists[body, :3] + np.cross(twists[body, 3:], r)
        if p["kind"] == "even_pair":
            r2 = np.asarray(p["other_point_m"])
            vp2 = twists[body, :3] + np.cross(twists[body, 3:], r2)
            f1 = np.cross(vp, np.cross(n, vp))
            f2 = -np.cross(vp2, np.cross(n, vp2))
            columns[body, :, k] = np.r_[f1+f2, np.cross(r, f1)+np.cross(r2, f2)]
            continue
        if p["kind"] == "even_transverse":
            force = np.cross(vp, np.cross(n, vp))
        else:
            s = np.dot(n, vp)
            amplitude = abs(s)*s if p["kind"] == "odd_drag" else max(s, 0.)**2
            force = -amplitude*n
        columns[body, :, k] = np.r_[force, np.cross(r, force)]
    return columns


def predict_split_hydro(model, q, nu, qdot=None, nudot=None, *, steady=True, allow_unvalidated=False):
    """Pure predictor. Unsupported kinematics raise unless explicitly marked exploratory.

    Setting allow_unvalidated does NOT add inertia or establish validation.
    generalized=[force,moment,joint torque] is a physical load, not actuator effort.
    """
    theta = validate_model(model)
    q, nu = vector(q, 3, "q"), vector(nu, 6, "nu")
    qdot = np.zeros(3) if qdot is None else vector(qdot, 3, "qdot")
    nudot = np.zeros(6) if nudot is None else vector(nudot, 6, "nudot")
    reasons = []
    if np.max(np.abs(qdot)) > 1e-12:
        reasons.append("independent_joint_motion_not_validated")
    if np.max(np.abs(nu[3:])) > 1e-12:
        reasons.append("angular_velocity_not_validated")
    if not steady or np.max(np.abs(nudot)) > 1e-12:
        reasons.append("dynamic_inertia_not_modelled")
    if reasons and not allow_unvalidated:
        raise ValueError("outside validated static kinematic scope: " + ",".join(reasons))
    output = recompose(q, component_features(model, q, nu, qdot) @ theta, model["geometry"])
    scope = model.get("scope", {})
    poses = np.asarray(scope.get("observed_q", []), float)
    observed = bool(poses.size and np.min(np.linalg.norm(poses.reshape(-1, 3)-q, axis=1)) < 1e-8)
    max_speed = scope.get("max_training_speed_m_s")
    in_speed = bool(max_speed is not None and np.linalg.norm(nu[:3]) <= max_speed + 1e-12)
    output["scope_flags"] = {"supported_static_kinematics": not reasons,
        "empirical_status": scope.get("status", "unfitted"), "blind_validated": False,
        "observed_training_pose": observed, "within_training_max_speed": in_speed,
        "within_training_speed": in_speed, "new_configuration": not observed,
        "out_of_sample_configuration": not observed,
        "configuration_range_status": "not_assessed; an unseen configuration is not necessarily outside the geometric range",
        "outside_training_speed": bool(max_speed is not None and not in_speed), "unvalidated_reasons": reasons,
        "inertial_model_included": False, "independent_joint_motion_validated": False,
        "candidate_retention_status": model.get("retention_status", "synthetic_or_unfitted"),
        "full_fit_identified": bool(model.get("fit", {}).get("full_rank", False)),
        "complete_development_identifiability": bool(model.get("development_cv_complete")) and all(model["development_cv_complete"].values()),
        "existing_dynamic_branch_unchanged": True}
    return output


def frozen87_steady(theta, q, nu, geometry):
    """Read-only original 87-parameter deployment reference, physical sign.

    Original hull block includes base0. It is never reported as pure hull and
    never separated by subtracting held-out measured base0 loads. Added-Coriolis
    moments are retained: at nonzero q/oblique flow the old model need not be odd.
    """
    t, q, nu = vector(theta, 87, "theta87"), vector(q, 3, "q"), vector(nu, 6, "nu")
    if np.max(np.abs(nu[3:])) > 1e-12:
        raise ValueError("frozen87_steady accepts translation only")
    v = nu[:3]
    hull_base = t[6:12]*nu + t[12:18]*np.abs(nu)*nu
    hull_base[3:] += np.cross(v, t[:3]*v)
    hull_base[3:] += t[42:51].reshape(3, 3, order="F") @ v
    hull_base[3:] += t[51:60].reshape(3, 3, order="F") @ (np.abs(v)*v)
    w = np.zeros((5, 6))
    w[0] = -hull_base
    J = body_jacobians(q, geometry)
    for i in range(3):
        vi = (J[i+2] @ np.r_[nu, [0., 0., 0.]])[:3]
        start = 18 + 6*i
        force = t[start+3:start+6]*np.abs(vi)*vi
        moment = np.cross(vi, t[start:start+3]*vi)
        moment += t[60+9*i:69+9*i].reshape(3, 3, order="F") @ (np.abs(vi)*vi)
        w[i+2] = -np.r_[force, moment]
    result = recompose(q, w, geometry)
    result["hull_plus_base0"] = result.pop("hull")
    result.pop("base0")
    result["scope_flags"] = {"pure_hull_available": False, "role": "frozen deployment reference, not same-data refit",
                             "added_coriolis_retained": True, "blind_validated": False}
    return result


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(v) for v in value]
    return value


def save_json(path, data):
    Path(path).write_text(json.dumps(jsonable(data), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")

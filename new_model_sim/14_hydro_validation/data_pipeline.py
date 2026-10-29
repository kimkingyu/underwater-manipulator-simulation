"""Read-only source conversion for stage 14; never imports a solver or old run scripts.

Public interfaces operate on raw SI reports and arbitrary new validation cases.
Run with Python -B to avoid bytecode files outside the explicitly owned outputs.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import h5py
import numpy as np

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
SOURCE = HERE.parent / "11_theta_fit"
POSE_SOURCE = HERE.parent / "13_pose_q1"
AXES = "xyz"
BODY_PARTS = [["0:56"], ["0:916"], ["0:7955"], ["0:8167"],
              ["0:1425", "0:2393", "0:4783", "0:7716"]]
DATASET_FILES = {"q0": "theta_dataset.json", "qb": "theta_dataset_qb8.json",
                 "qc": "theta_dataset_qc.json", "qd": "theta_dataset_qd.json"}
EXPECTED_COMMIT = "3552c585e13dce2d43dc702008c1cdc210fee2d5"
SD_NOTE = ("Population SD (ddof=0) of correlated saved samples in the last 200 solver "
           "iterations; solver-load fluctuation, NOT independent measurement uncertainty, "
           "standard error, discretization error or an uncertainty confidence interval.")


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def relative_path(path):
    return Path(path).resolve().relative_to(WORKSPACE).as_posix()


def finite_array(value, shape=None, name="array"):
    arr = np.asarray(value, dtype=float)
    if shape is not None and arr.shape != tuple(shape):
        raise ValueError(f"{name} shape {arr.shape}, expected {tuple(shape)}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    return arr


def skew(vector):
    x, y, z = finite_array(vector, (3,))
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(axis, angle):
    axis = finite_array(axis, (3,), "rotation axis")
    if not np.isfinite(angle) or not np.isclose(np.linalg.norm(axis), 1., atol=1e-12):
        raise ValueError("rotation requires a finite angle and unit axis")
    K = skew(axis)
    return np.eye(3) + math.sin(angle) * K + (1. - math.cos(angle)) * (K @ K)


def shift_wrench(wrench, old_reference, new_reference):
    """Same axes; M_new = M_old + (r_old-r_new) x F. Supports (..., 6)."""
    wrench = finite_array(wrench, name="wrench")
    if wrench.shape[-1:] != (6,):
        raise ValueError("wrench final dimension must be 6")
    delta = finite_array(old_reference, (3,)) - finite_array(new_reference, (3,))
    return np.concatenate((wrench[..., :3], wrench[..., 3:] + np.cross(delta, wrench[..., :3])), axis=-1)


def validate_rotation(matrix):
    R = finite_array(matrix, (3, 3), "body rotation")
    if not np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-10) or not np.isclose(np.linalg.det(R), 1.):
        raise ValueError("body rotation must be proper orthogonal")
    return R


def reference_from_contract(contract):
    keys = ("body_order", "body_walls", "frame", "origin_Ob_CAD_m", "load_sign", "wrench_order",
            "wrench_units", "moment_reference", "joint_centres_CAD0_m", "joint_axes_CAD0",
            "link_reference_CAD0_m")
    ref = {key: contract[key] for key in keys}
    ref.update({"raw_moment_origin_CAD_m": [0., 0., 0.], "raw_channel_count": 57,
                "nu_order": ["vx", "vy", "vz", "wx", "wy", "wz"],
                "nu_units": ["m/s"] * 3 + ["rad/s"] * 3,
                "nudot_units": ["m/s^2"] * 3 + ["rad/s^2"] * 3,
                "uncertainty_note": SD_NOTE,
                "joint_tau_convention": "J^T times link-local fluid wrenches; current posed axes, not q0 tau_j reports",
                "source_path_base": "repository root", "linear_velocity_sign": "body relative to water; minus steady inlet"})
    return ref


def forward_kinematics(q, reference):
    """Return joint centres/axes and body reference points, all relative to Ob in B.

    body_rotations maps each body's local axes to B; hull and base0 are identity.
    Link reference points are the original uvms_geometry points, never new mass COMs.
    """
    q = finite_array(q, (3,), "q")
    O = finite_array(reference["origin_Ob_CAD_m"], (3,))
    centres = finite_array(reference["joint_centres_CAD0_m"], (3, 3)) - O
    axes = finite_array(reference["joint_axes_CAD0"], (3, 3))
    refs = finite_array(reference["link_reference_CAD0_m"], (3, 3)) - O
    Rc, Tc = np.eye(3), np.zeros(3)
    pj, aj = [], []
    body_refs, body_rotations = [np.zeros(3), np.zeros(3)], [np.eye(3), np.eye(3)]
    for j in range(3):
        p, a = Rc @ centres[j] + Tc, Rc @ axes[j]
        Ri = rotation(a, q[j])
        Tc, Rc = Ri @ (Tc - p) + p, Ri @ Rc
        pj.append(p)
        aj.append(a)
        body_refs.append(Rc @ refs[j] + Tc)
        body_rotations.append(Rc.copy())
    return {"joint_centres": np.array(pj), "joint_axes": np.array(aj),
            "body_references": np.array(body_refs), "body_rotations": np.array(body_rotations)}


def body_jacobians(q, reference):
    """5x6x9 Jacobians: [body v(3), body omega(3), joint rates(3)] -> local twists."""
    fk = forward_kinematics(q, reference)
    J = np.zeros((5, 6, 9))
    for i, (p, R) in enumerate(zip(fk["body_references"], fk["body_rotations"])):
        J[i, :3, :3] = R.T
        J[i, :3, 3:6] = -R.T @ skew(p)
        J[i, 3:, 3:6] = R.T
        if i >= 2:
            for j in range(i - 1):
                axis = fk["joint_axes"][j]
                J[i, :3, 6 + j] = R.T @ np.cross(axis, p - fk["joint_centres"][j])
                J[i, 3:, 6 + j] = R.T @ axis
    return J


def recompose_components(wrenches, q, reference):
    """5x6 local wrenches -> total(6), posed joint_tau(3), generalized(9)."""
    wrenches = finite_array(wrenches, (5, 6), "component wrenches")
    fk = forward_kinematics(q, reference)
    about_Ob = []
    for wrench, p, R in zip(wrenches, fk["body_references"], fk["body_rotations"]):
        rotated = np.concatenate((R @ wrench[:3], R @ wrench[3:]))
        about_Ob.append(shift_wrench(rotated, p, np.zeros(3)))
    total = np.sum(about_Ob, axis=0)
    tau = np.zeros(3)
    for j, (p, a) in enumerate(zip(fk["joint_centres"], fk["joint_axes"])):
        for wrench in about_Ob[j + 2:]:
            tau[j] += a @ (wrench[3:] - np.cross(p, wrench[:3]))
    return {"total": total, "joint_tau": tau, "generalized": np.concatenate((total, tau))}


def loads_to_components(loads, q, reference, origin_global=None, rotation_global_from_body=None,
                        raw_moment_origin=None, force_scale=1., moment_scale=1.):
    """57 raw reports -> independent 5x6 wrenches + total6 + posed joint_tau3.

    Required inputs are the 8 walls' 48 force/moment channels; total and tau reports
    are diagnostics, NOT labels. All available reports must be finite. Scaling is
    explicit for reuse on old coefficient files; selected production sources use 1.
    """
    for name, value in loads.items():
        finite_array(value, (), f"raw {name}")
    if not np.isfinite(force_scale) or not np.isfinite(moment_scale) or force_scale <= 0 or moment_scale <= 0:
        raise ValueError("force and moment scales must be finite and positive")
    O = finite_array(reference["origin_Ob_CAD_m"] if origin_global is None else origin_global, (3,))
    Rg = validate_rotation(np.eye(3) if rotation_global_from_body is None else rotation_global_from_body)
    raw_origin = finite_array(np.zeros(3) if raw_moment_origin is None else raw_moment_origin, (3,))
    fk = forward_kinematics(q, reference)
    body_wrenches = []
    for walls, p, R in zip(reference["body_walls"], fk["body_references"], fk["body_rotations"]):
        fg = np.array([sum(loads[f"f{a}({z})"] for z in walls) for a in AXES]) * force_scale
        mg = np.array([sum(loads[f"m{a}({z})"] for z in walls) for a in AXES]) * moment_scale
        fb = Rg.T @ fg
        mb = Rg.T @ (mg + np.cross(raw_origin - O, fg))
        body_wrenches.append(np.concatenate((R.T @ fb, R.T @ (mb - np.cross(p, fb)))))
    wrenches = finite_array(body_wrenches, (5, 6))
    return {"wrenches": wrenches, **recompose_components(wrenches, q, reference)}


def average_window(history, q, reference, window_iterations=200, **conversion_kwargs):
    """Use last-window < iter <= last, not 200 saved samples; population SD only."""
    if not history or window_iterations <= 0:
        raise ValueError("nonempty history and positive averaging window required")
    iterations = finite_array([h["iter"] for h in history], name="iterations")
    if np.any(np.diff(iterations) <= 0):
        raise ValueError("iteration indices must strictly increase")
    # Reject invalid earlier samples too; never silently truncate a divergent history.
    for h in history:
        for name, value in h["loads"].items():
            finite_array(value, (), f"raw {name}")
    keep = np.flatnonzero(iterations > iterations[-1] - window_iterations)
    samples = [loads_to_components(history[k]["loads"], q, reference, **conversion_kwargs) for k in keep]
    out = {}
    for key in ("wrenches", "total", "joint_tau", "generalized"):
        values = np.array([s[key] for s in samples])
        out[key] = np.mean(values, axis=0)
        out[key + "_sd"] = np.std(values, axis=0, ddof=0)
    out["averaging"] = {"rule": "iter > last_iter - window_iterations", "window_iterations": window_iterations,
                        "interval_exclusive_inclusive": [int(iterations[-1] - window_iterations), int(iterations[-1])],
                        "sample_count": len(keep), "sample_indices_zero_based": keep.tolist(),
                        "sample_iterations": iterations[keep].astype(int).tolist(), "sd_ddof": 0,
                        "uncertainty_note": SD_NOTE}
    out["mean_raw_loads"] = {k: float(np.mean([history[i]["loads"][k] for i in keep]))
                             for k in history[0]["loads"]}
    return out


def ramped_cosine_motion(times, peak_rate, angular_frequency, ramp_s):
    """Analytic integral, velocity and right derivative of V*cos(w*t)*min(t/ramp,1)."""
    t = finite_array(times, name="physical time")
    pars = finite_array([peak_rate, angular_frequency, ramp_s], (3,))
    if np.any(t < 0) or pars[1] <= 0 or pars[2] < 0:
        raise ValueError("times/ramp nonnegative and angular frequency positive required")
    V, w, r = pars
    if r == 0:
        return V / w * np.sin(w * t), V * np.cos(w * t), -V * w * np.sin(w * t)
    tr = np.minimum(t, r)
    integrated_ramp = V / r * (tr * np.sin(w * tr) / w + (np.cos(w * tr) - 1.) / w ** 2)
    displacement = integrated_ramp + np.where(t > r, V / w * (np.sin(w * t) - math.sin(w * r)), 0.)
    factor = np.minimum(t / r, 1.)
    velocity = V * np.cos(w * t) * factor
    derivative = V * (-w * np.sin(w * t) * factor + np.cos(w * t) * (t < r) / r)
    return displacement, velocity, derivative


def read_case_settings(path):
    """Read saved Fluent HDF5 settings only. No solver process or PyFluent import."""
    with h5py.File(path, "r") as handle:
        text = handle["settings/Rampant Variables"][()].tobytes().decode("utf-8").rstrip("\0")
    return text


def extract_oscillation_law(text):
    match = re.search(r'\(name \. "osc_rate"\)\s*\(definition \. "([^"]+)"\)', text)
    if not match:
        raise ValueError("no osc_rate expression in the original case")
    expression = match.group(1)
    numbers = re.match(r'([^ ]+) \[([^\]]+)\] \* cos\(([^ ]+) \[s\^-1\] \* t\) \* min\(t / ([^ ]+) \[s\], 1\)', expression)
    if not numbers:
        raise ValueError(f"unsupported motion expression: {expression}")
    return {"peak_rate": float(numbers.group(1)), "rate_unit": numbers.group(2),
            "angular_frequency_rad_s": float(numbers.group(3)), "ramp_s": float(numbers.group(4)),
            "expression": expression, "source": "settings/Rampant Variables/named-expressions"}


def oscillation_trajectory(history, spec, q, reference, law):
    """Convert actual raw transient samples; unit-excitation pseudo-data are never accepted."""
    dof, amplitude, period = int(spec["dof"]), float(spec["amp"]), float(spec["period"])
    if dof not in range(6) or period <= 0 or amplitude <= 0:
        raise ValueError("valid oscillation dof, period, amplitude required")
    t = finite_array([h["t"] for h in history], name="time_s")
    if not len(t) or np.any(np.diff(t) <= 0):
        raise ValueError("physical time must strictly increase")
    disp, speed, acceleration = ramped_cosine_motion(t, law["peak_rate"], law["angular_frequency_rad_s"], law["ramp_s"])
    O = finite_array(reference["origin_Ob_CAD_m"], (3,))
    nu, nudot = np.zeros((len(t), 6)), np.zeros((len(t), 6))
    nu[:, dof], nudot[:, dof] = speed, acceleration
    rows, origins, rotations = [], [], []
    for h, x in zip(history, disp):
        Ot, R = O.copy(), np.eye(3)
        if dof < 3:
            Ot[dof] += x
        else:
            R = rotation(np.eye(3)[dof - 3], x)
        origins.append(Ot)
        rotations.append(R)
        rows.append(loads_to_components(h["loads"], q, reference, Ot, R))
    return {"q": finite_array(q, (3,)), "viscous_model": "laminar", "dof": dof,
            "amplitude": amplitude, "amplitude_unit": "m" if dof < 3 else "rad", "period_s": period,
            "ramp_s": law["ramp_s"], "time_s": t, "nu": nu, "nudot": nudot,
            "wrenches": np.array([r["wrenches"] for r in rows]), "total": np.array([r["total"] for r in rows]),
            "joint_tau": np.array([r["joint_tau"] for r in rows]),
            "identification_mask": (t > period + 1e-9) & (t >= law["ramp_s"]),
            "motion_displacement_or_angle": disp, "origin_global_m": np.array(origins),
            "rotation_global_from_body": np.array(rotations), "motion_law": law,
            "requested_motion": {"amplitude": amplitude, "amplitude_unit": "m" if dof < 3 else "rad",
                                 "period_s": period, "ramp_s": law["ramp_s"],
                                 "angular_frequency_rad_s": 2 * math.pi / period,
                                 "peak_rate": amplitude * 2 * math.pi / period},
            "actual_motion": {"peak_rate": law["peak_rate"], "rate_unit": law.get("rate_unit"),
                              "angular_frequency_rad_s": law["angular_frequency_rad_s"],
                              "effective_amplitude": law["peak_rate"] / law["angular_frequency_rad_s"],
                              "effective_period_s": 2 * math.pi / law["angular_frequency_rad_s"],
                              "ramp_s": law["ramp_s"], "integration": "exact analytic integral of saved prescribed rate"},
            "trajectory_note": "Physical sampled N/N*m loads. Exact analytic integral of saved velocity expression, including ramp offset; not A*sin(w*t) during/after ramp.",
            "motion_scope": "q fixed; one whole-domain translational or rotational DOF; no independent joint motion"}


def make_bodies(reference):
    raw = {b["name"]: b for b in load_json(SOURCE / "body_centroids.json")["bodies"]}
    O = finite_array(reference["origin_Ob_CAD_m"], (3,))
    refs = [O, O, *finite_array(reference["link_reference_CAD0_m"], (3, 3))]
    bodies = []
    for name, walls, parts, ref in zip(reference["body_order"], reference["body_walls"], BODY_PARTS, refs):
        boxes = np.array([raw[p]["bbox_m"] for p in parts])
        bodies.append({"id": name, "reference_CAD0_m": ref, "bbox_local_min_m": boxes[:, :3].min(axis=0) - ref,
                       "bbox_local_max_m": boxes[:, 3:].max(axis=0) - ref, "walls": walls,
                       "bbox_source": relative_path(SOURCE / "body_centroids.json"), "bbox_source_part_ids": parts,
                       "bbox_note": "Union of original q0 part AABBs, relative to this body's original reference; envelope not an exact solid-membership test."})
    return bodies


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    return value


def write_json_owned(path, value):
    path = Path(path).resolve()
    allowed = [HERE / "data", HERE / "frozen" / "source_manifest.json", HERE / "reports" / "data_audit.json"]
    if not (path.is_relative_to(allowed[0]) or path in allowed[1:]):
        raise ValueError("refusing to write outside the explicitly owned data/audit outputs")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(serializable(value), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def old_label_vector(case, pose):
    pieces = [case[k][0] if pose == "q0" else case[k] for k in ("F", "M", "tau", "flink")]
    return finite_array(np.concatenate(pieces), (27,))


def new_label_vector(row):
    return np.concatenate((row["total"], row["joint_tau"], np.asarray(row["wrenches"])[2:].ravel()))


def generate_dataset():
    contract = load_json(HERE / "contract.json")
    ref = reference_from_contract(contract)
    cases_spec = load_json(SOURCE / "cases.json")
    steady, oscillations, closures = [], [], []
    paths = {HERE / "contract.json", SOURCE / "cases.json", SOURCE / "body_centroids.json",
             SOURCE / "matlab/uvms_geometry.m"}
    q0_osc = []
    for pose, name in DATASET_FILES.items():
        dataset_path = SOURCE / name
        old = load_json(dataset_path)
        paths.add(dataset_path)
        q = [0., 0., 0.] if pose == "q0" else old["q"]
        rundir = SOURCE / ("runs" if pose == "q0" else "runs_" + pose)
        for old_case in old["cases"]:
            cid = old_case["id"]
            if old_case.get("kind") == "oscillation":
                q0_osc.append(old_case)
                continue
            hist_path, result_path = rundir / (cid + "_hist.json"), rundir / (cid + "_result.json")
            paths.update((hist_path, result_path))
            hist, result = load_json(hist_path), load_json(result_path)
            required_failures = result.get("failed_required", []) or [s["step"] for s in result.get("steps", []) if s.get("required") and not s.get("ok")]
            strict = result.get("status") == "ok" and bool(result.get("converged")) and not required_failures
            capped = result.get("status") == "failed" and not required_failures and result.get("iterations", 0) >= 2500
            if not (strict or capped):
                raise ValueError(f"{pose}/{cid}: unacceptable solver state")
            stats = average_window(hist, q, ref)
            row = {"id": pose + "__" + cid, "pose_id": pose, "case_id": cid, "q": q,
                   "nu": cases_spec["steady"][cid]["v"] + [0., 0., 0.],
                   "wrenches": stats["wrenches"], "total": stats["total"], "joint_tau": stats["joint_tau"],
                   "strict": strict, "unsteady_mean": capped, "viscous_model": "SST",
                   "status": result["status"], "converged": result.get("converged"),
                   "iterations": result.get("iterations"), "drift_last": result.get("drift_last"),
                   "failed_required": required_failures, "uncertainty_wrenches_sd": stats["wrenches_sd"],
                   "uncertainty_total_sd": stats["total_sd"], "uncertainty_tau_sd": stats["joint_tau_sd"],
                   "source": {"history": relative_path(hist_path), "result": relative_path(result_path),
                              "legacy_dataset": relative_path(dataset_path), "case_h5": relative_path(rundir / (cid + ".cas.h5")),
                              "averaging": stats["averaging"], "raw_channel_count": len(hist[0]["loads"]),
                              "force_scale_applied": 1., "moment_scale_applied": 1.}}
            diff = new_label_vector(row) - old_label_vector(old_case, pose)
            recomposed = recompose_components(row["wrenches"], q, ref)
            jmap = np.einsum("bij,bi->j", body_jacobians(q, ref), row["wrenches"])
            raw = stats["mean_raw_loads"]
            raw_total = shift_wrench(np.array([raw[k] for k in ["fx", "fy", "fz", "mx", "my", "mz"]]), np.zeros(3), ref["origin_Ob_CAD_m"])
            report_tau = np.array([raw[f"tau_j{k}"] for k in (1, 2, 3)])
            closures.append({"id": row["id"], "legacy_27_max_abs": float(np.max(np.abs(diff))),
                             "legacy_total_max_abs": float(np.max(np.abs(diff[:6]))),
                             "legacy_tau_max_abs": float(np.max(np.abs(diff[6:9]))),
                             "legacy_link_wrench_max_abs": float(np.max(np.abs(diff[9:]))),
                             "raw_total_report_max_abs": float(np.max(np.abs(raw_total - row["total"]))),
                             "recomposition_max_abs": float(np.max(np.abs(recomposed["generalized"] - np.concatenate((row["total"], row["joint_tau"]))))),
                             "jacobian_transpose_max_abs": float(np.max(np.abs(jmap - recomposed["generalized"]))),
                             "q0_tau_report_minus_posed_tau": report_tau - row["joint_tau"],
                             "sample_count": stats["averaging"]["sample_count"]})
            steady.append(row)
    for old_case in q0_osc:
        cid = old_case["id"]
        hist_path, result_path, case_path = [SOURCE / "runs" / (cid + suffix) for suffix in ("_hist.json", "_result.json", ".cas.h5")]
        paths.update((hist_path, result_path, case_path))
        result, hist = load_json(result_path), load_json(hist_path)
        if result.get("status") != "ok" or not result.get("converged") or result.get("failed_required"):
            raise ValueError(f"{cid}: failed transient source")
        law = extract_oscillation_law(read_case_settings(case_path))
        row = oscillation_trajectory(hist, cases_spec["oscillation"][cid], [0., 0., 0.], ref, law)
        row.update({"id": "q0__" + cid, "status": result["status"],
                    "source": {"history": relative_path(hist_path), "result": relative_path(result_path),
                               "case_h5": relative_path(case_path), "legacy_dataset": relative_path(SOURCE / "theta_dataset.json"),
                               "physical_sample_count": len(hist), "raw_channel_count": len(hist[0]["loads"]),
                               "force_scale_applied": 1., "moment_scale_applied": 1.}})
        oscillations.append(row)
    if len(steady) != 40 or sum(r["strict"] for r in steady) != contract["validation"]["strict_steady_count"]:
        raise ValueError("steady/strict count disagrees with contract")
    if sorted(r["id"] for r in steady if r["unsteady_mean"]) != sorted(contract["validation"]["capped_ids"]):
        raise ValueError("capped IDs disagree with contract")
    if len(oscillations) != 6 or any(len(r["time_s"]) != 240 or sum(r["identification_mask"]) != 160 for r in oscillations):
        raise ValueError("expected six actual 240-point oscillations, 160 identification points each")
    if max(r["legacy_27_max_abs"] for r in closures) >= 1e-10:
        raise ValueError("raw steady conversion does not reproduce legacy labels")
    dataset = {"schema_version": contract["schema_version"], "reference": ref, "bodies": make_bodies(ref),
               "steady": steady, "oscillations": oscillations,
               "sources": {"manifest": relative_path(HERE / "frozen/source_manifest.json"),
                           "legacy_datasets": {pose: relative_path(SOURCE / f) for pose, f in DATASET_FILES.items()},
                           "source_git_commit": contract["source_git_commit"],
                           "source_data_are_immutable": True,
                           "excluded": ["runs_qd_interrupted", "runs/_* diagnostic trials", "all CSV exports"],
                           "status": "Retrospective development data; no new blind CFD validation"}}
    return dataset, closures, paths


def write_dataset_outputs(dataset):
    write_json_owned(HERE / "data/component_dataset.json", dataset)
    write_json_owned(HERE / "data/interface_schema.json", {
        "schema_version": 1, "contract": relative_path(HERE / "contract.json"),
        "dataset": relative_path(HERE / "data/component_dataset.json"),
        "functions": {"forward_kinematics(q, reference)": "joint_centres(3,3), joint_axes(3,3), body_references(5,3), body_rotations(5,3,3)",
                      "loads_to_components(loads, q, reference, origin_global=None, rotation_global_from_body=None, raw_moment_origin=None, force_scale=1, moment_scale=1)": "wrenches(5,6), total(6), joint_tau(3), generalized(9)",
                      "average_window(history, q, reference, window_iterations=200, **conversion_kwargs)": "mean/SD arrays plus original averaging indices; future raw steady reuse",
                      "recompose_components(wrenches, q, reference)": "total(6), joint_tau(3), generalized(9)",
                      "body_jacobians(q, reference)": "J(5,6,9): local twist=J@[nu,qdot]; generalized=sum J.T@wrench",
                      "oscillation_trajectory(history, spec, q, reference, law)": "physical sampled trajectory; spec=dof/amp/period; law=peak_rate/angular_frequency_rad_s/ramp_s"},
        "example_steady": dataset["steady"][0],
        "oscillation_shapes": {"time_s": [240], "nu": [240, 6], "nudot": [240, 6], "wrenches": [240, 5, 6], "total": [240, 6], "joint_tau": [240, 3]},
        "uncertainty_note": SD_NOTE})


def sexpression_at(text, start):
    """Extract one balanced saved Scheme expression without executing any Scheme."""
    depth, quoted, escaped = 0, False, False
    for i in range(start, len(text)):
        char = text[i]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    raise ValueError("unbalanced saved settings expression")


def settings_atom(text, key):
    match = re.search(r"^\(" + re.escape(key) + r"\s+([^\n]*)\)$", text, re.MULTILINE)
    if not match:
        raise ValueError(f"missing settings atom {key}")
    return match.group(1)


def inspect_case(path, role, expected_model=None):
    text = read_case_settings(path)
    config = sexpression_at(text, text.index("(case-config "))
    flags = dict(re.findall(r"\(([^ ()]+) \. (#[tf]|[0-9]+)\)", config))
    model = "laminar" if flags.get("rp-lam?") == "#t" else (
        "SST" if flags.get("rp-kw?") == "#t" and settings_atom(text, "kw-sst-on?") == "#t" else "unclassified")
    reports, missing_reports = {}, []
    for name in ("fx", "fy", "fz", "mx", "my", "mz", "tau_j1", "tau_j2", "tau_j3"):
        marker = '((name . "' + name + '")'
        if marker not in text:
            if role != "historical_coefficient_evidence":
                raise ValueError(f"{path}: missing required report {name}")
            missing_reports.append(name)
            continue
        block = sexpression_at(text, text.index(marker))
        scaled = re.search(r"\(scaled\? \. (#[tf])\)", block)
        centre = re.search(r"\(mom-center ([^)]+)\)", block)
        direction = re.search(r"\((?:mom-axis|force-vector) ([^)]+)\)", block)
        frame = re.search(r'\(reference-frame \. "([^"]+)"\)', block)
        walls = re.search(r"\(thread-names ([^)]+)\)", block)
        reports[name] = {"scaled": scaled.group(1) == "#t" if scaled else None,
                         "moment_origin_CAD_m": [float(x) for x in centre.group(1).split()] if centre else None,
                         "direction": [float(x) for x in direction.group(1).split()] if direction else None,
                         "frame": frame.group(1) if frame else None,
                         "walls": re.findall(r'"([^"]+)"', walls.group(1)) if walls else []}
    ref = {k: float(settings_atom(text, "reference-" + k)) for k in ("density", "velocity", "area", "length")}
    coefficient_moment_scale = .5 * ref["density"] * ref["velocity"] ** 2 * ref["area"] * ref["length"]
    gravity = [float(settings_atom(text, "gravity/" + a)) for a in AXES]
    selected = role == "selected_dataset_case"
    passed = (all(r["scaled"] is False for r in reports.values())
              and all(reports[k]["moment_origin_CAD_m"] == [0., 0., 0.] for k in ("mx", "my", "mz"))
              and all(r["frame"] == "global" for r in reports.values())
              and settings_atom(text, "gravity?") == "#f" and gravity == [0., 0., 0.]
              and (expected_model is None or expected_model == model))
    row = {"path": relative_path(path), "role": role, "settings_dataset": "settings/Rampant Variables",
           "settings_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(), "reports": reports, "missing_reports": missing_reports,
           "gravity_enabled": settings_atom(text, "gravity?") == "#t", "gravity_m_s2": gravity,
           "actual_model_from_case_config": model, "expected_model": expected_model,
           "case_config_flags": {k: flags.get(k) for k in ("rp-lam?", "rp-kw?", "rp-turb?", "rp-unsteady?", "rp-dual-time?")},
           "model_evidence_note": "case-config association list is the actual saved model; earlier top-level rp-* defaults can still contain SST even for laminar runs.",
           "reference_values": ref, "coefficient_moment_to_Nm_factor": coefficient_moment_scale,
           "selected_source_checks_pass": passed if selected else None}
    if '"osc_rate"' in text:
        row["motion_law"] = extract_oscillation_law(text)
    return row


def fit_harmonics(times, values, angular_frequency):
    """Physical DC + first/second sin/cos coefficients by joint least squares."""
    t = finite_array(times, name="harmonic times")
    y = finite_array(values, name="physical harmonic loads")
    if t.ndim != 1 or y.ndim != 2 or y.shape[0] != len(t) or len(t) < 5:
        raise ValueError("harmonics require at least 5 time-by-channel samples")
    if not np.isfinite(angular_frequency) or angular_frequency <= 0:
        raise ValueError("positive finite harmonic frequency required")
    wt = t * angular_frequency
    X = np.column_stack((np.ones(len(t)), np.sin(wt), np.cos(wt), np.sin(2 * wt), np.cos(2 * wt)))
    coef, _, rank, _ = np.linalg.lstsq(X, y, rcond=None)
    if rank != 5:
        raise ValueError("harmonic design does not have full rank")
    a1, a2 = np.hypot(coef[1], coef[2]), np.hypot(coef[3], coef[4])
    return {"sample_count": len(t), "angular_frequency_rad_s": angular_frequency, "dc": coef[0],
            "first_sin": coef[1], "first_cos": coef[2], "first_amplitude": a1,
            "second_sin": coef[3], "second_cos": coef[4], "second_amplitude": a2,
            "second_to_first_amplitude_ratio": a2 / np.maximum(a1, 1e-12),
            "fit_residual_rms": np.sqrt(np.mean((y - X @ coef) ** 2, axis=0))}


def unit_acceleration_to_sine(unit_response, amplitude, period_s):
    """Legacy per +1 acceleration response -> physical sine load amplitude (not a new sample)."""
    unit_response = finite_array(unit_response, name="legacy unit response")
    finite_array([amplitude, period_s], (2,))
    if amplitude <= 0 or period_s <= 0:
        raise ValueError("amplitude/period must be positive")
    return -amplitude * (2 * math.pi / period_s) ** 2 * unit_response


def sine_to_unit_acceleration(sine_loads, amplitude, period_s):
    return finite_array(sine_loads) / unit_acceleration_to_sine(1., amplitude, period_s)


def legacy_simpson_displacement(t, amplitude, period_s, ramp_s, subdivisions=4000):
    """Independent reproduction of old build_dataset integration, used only in the audit."""
    if t <= 0:
        return 0.
    if subdivisions <= 0 or subdivisions % 2:
        raise ValueError("Simpson subdivision count must be positive and even")
    points = np.linspace(0., t, subdivisions + 1)
    w = 2 * math.pi / period_s
    rate = amplitude * w * np.cos(w * points)
    if ramp_s > 0:
        rate *= np.minimum(points / ramp_s, 1.)
    weights = np.ones(subdivisions + 1)
    weights[1:-1:2], weights[2:-1:2] = 4., 2.
    return float(np.sum(rate * weights) * t / (3 * subdivisions))


def audit_oscillations(dataset):
    reference = dataset["reference"]
    legacy = {c["id"]: c for c in load_json(SOURCE / "theta_dataset.json")["cases"]}
    results = []
    channel_names = [f"{b}.{c}" for b in reference["body_order"] for c in reference["wrench_order"]]
    channel_names += ["total." + c for c in reference["wrench_order"]] + ["joint_tau." + str(j) for j in (1, 2, 3)]
    units = reference["wrench_units"] * 6 + ["N*m"] * 3
    for row in dataset["oscillations"]:
        cid = row["id"].split("__")[1]
        old = legacy[cid]
        t, mask = np.array(row["time_s"]), np.array(row["identification_mask"])
        A, T, dof = row["amplitude"], row["period_s"], row["dof"]
        w = 2 * math.pi / T
        y = np.column_stack((np.reshape(row["wrenches"], (len(t), 30)), row["total"], row["joint_tau"]))
        physical = fit_harmonics(t[mask], y[mask], w)
        actual_frequency_fit = fit_harmonics(t[mask], y[mask], row["motion_law"]["angular_frequency_rad_s"])
        cycle2, cycle3 = (t > T + 1e-9) & (t <= 2 * T + 1e-9), (t > 2 * T + 1e-9) & (t <= 3 * T + 1e-9)
        if sum(cycle2) != sum(cycle3):
            raise ValueError("cycle2/3 phase sample counts differ")
        phase_diff = y[cycle3] - y[cycle2]
        h2, h3 = fit_harmonics(t[cycle2], y[cycle2], w), fit_harmonics(t[cycle3], y[cycle3], w)
        history = load_json(WORKSPACE / row["source"]["history"])
        old_physical, accurate_27 = [], []
        for index in np.flatnonzero(mask):
            tt = t[index]
            x = legacy_simpson_displacement(tt, A, T, row["ramp_s"])
            O, R = np.array(reference["origin_Ob_CAD_m"]), np.eye(3)
            if dof < 3:
                O[dof] += x
            else:
                R = rotation(np.eye(3)[dof - 3], x)
            raw = loads_to_components(history[index]["loads"], [0., 0., 0.], reference, O, R)
            old_physical.append(new_label_vector(raw))
            accurate_27.append(new_label_vector({k: row[k][index] for k in ("total", "joint_tau", "wrenches")}))
        old_physical, accurate_27 = np.array(old_physical), np.array(accurate_27)
        old_sine = 2 / sum(mask) * (np.sin(w * t[mask]) @ old_physical)
        old_cos = 2 / sum(mask) * (np.cos(w * t[mask]) @ old_physical)
        actual_sine = 2 / sum(mask) * (np.sin(w * t[mask]) @ accurate_27)
        old_unit = old_label_vector(old, "q0")
        unit_recomputed = sine_to_unit_acceleration(old_sine, A, T)
        recovered_sine = unit_acceleration_to_sine(old_unit, A, T)
        velocity_diff = None
        if len(old["t"]) > 1:
            old_velocity_label = np.concatenate([old[k][1] for k in ("F", "M", "tau", "flink")])
            velocity_diff = float(np.max(np.abs(old_velocity_label - old_cos)))
        ideal_x = ramped_cosine_motion(t, A * w, w, row["ramp_s"])[0]
        exact_x = np.array(row["motion_displacement_or_angle"])
        results.append({"id": row["id"], "physical_sample_count": len(t), "identification_sample_count": int(sum(mask)),
                        "channels": channel_names, "channel_units": units,
                        "dc_first_second_harmonics_nominal_frequency": physical,
                        "dc_first_second_harmonics_actual_frequency": actual_frequency_fit,
                        "period_2_3": {"sample_count_each": int(sum(cycle2)),
                                       "intervals_exclusive_inclusive_s": [[T, 2 * T], [2 * T, 3 * T]],
                                       "phase_paired_difference_rms": np.sqrt(np.mean(phase_diff ** 2, axis=0)),
                                       "phase_paired_difference_max_abs": np.max(np.abs(phase_diff), axis=0),
                                       "dc_difference": h3["dc"] - h2["dc"],
                                       "first_sin_difference": h3["first_sin"] - h2["first_sin"],
                                       "first_cos_difference": h3["first_cos"] - h2["first_cos"],
                                       "second_sin_difference": h3["second_sin"] - h2["second_sin"],
                                       "second_cos_difference": h3["second_cos"] - h2["second_cos"]},
                        "legacy_unit_excitation_check": {
                            "old_unit_labels_are_physical_samples": False,
                            "unit_label_units_note": "Each load channel divided by m/s^2 for translation or rad/s^2 for rotation; not plain N or N*m.",
                            "restoration": "physical sine = -amplitude*(2*pi/period)^2 * legacy per-unit-acceleration response",
                            "recomputed_old_unit_max_abs": float(np.max(np.abs(old_unit - unit_recomputed))),
                            "restored_sine_vs_legacy_projection_max_abs": float(np.max(np.abs(recovered_sine - old_sine))),
                            "old_velocity_quadrature_max_abs": velocity_diff,
                            "old_out_of_phase_coefficient_max_abs": float(np.max(np.abs(np.array(old["out_of_phase_per_unit_velocity"]) + old_cos / (A * w)))),
                            "actual_vs_legacy_motion_sine_max_abs": float(np.max(np.abs(actual_sine - old_sine))),
                            "actual_vs_legacy_motion_sample_max_abs": float(np.max(np.abs(accurate_27 - old_physical)))},
                        "requested_motion": {"amplitude": A, "amplitude_unit": row["amplitude_unit"], "period_s": T,
                                             "angular_frequency_rad_s": w, "peak_rate": A * w, "ramp_s": row["ramp_s"]},
                        "actual_saved_motion": row["motion_law"],
                        "actual_vs_ideal_integrated_displacement_or_angle_max_abs": float(np.max(np.abs(exact_x - ideal_x))),
                        "integrated_end_displacement_or_angle": float(exact_x[-1]),
                        "warning": "Nonzero DC, higher harmonics and cycle drift are retained; these runs do not establish general dynamic/joint-motion validation."})
    return results


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def make_source_manifest(paths):
    """Hash originals in place. A later source mutation is an error, never silently re-frozen."""
    current_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=WORKSPACE, text=True).strip()
    files = [{"path": relative_path(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
             for path in sorted(paths, key=lambda p: relative_path(p))]
    target = HERE / "frozen/source_manifest.json"
    if target.exists():
        old = load_json(target)
        previous = {r["path"]: r["sha256"] for r in old["files"]}
        changes = [r["path"] for r in files if r["path"] in previous and previous[r["path"]] != r["sha256"]]
        if changes:
            raise ValueError("frozen source hash changed: " + ", ".join(changes))
    manifest = {"schema_version": 1, "source_git_commit": EXPECTED_COMMIT, "observed_git_HEAD": current_head,
                "HEAD_matches_approved": current_head == EXPECTED_COMMIT,
                "hash_algorithm": "SHA256", "path_base": "repository root", "files": files,
                "hash_count": len(files), "bytes_hashed": sum(r["size_bytes"] for r in files),
                "source_policy": "Read/hash in place only; no source copies, rewrites, solver runs or git state changes.",
                "excluded": ["runs_qd_interrupted (not opened)", "non-task CSVs (not opened)", "archived failed runs except explicit historical coefficient evidence", "existing delivery packages"],
                "repeat_policy": "Own generated data JSON may be updated; previously frozen source SHA256 changes cause failure."}
    write_json_owned(target, manifest)
    return manifest


def geometry_audit(dataset):
    mass = load_json(POSE_SOURCE / "mass_table.json")
    deltas = []
    for body in dataset["bodies"]:
        group = mass["groups"].get(body["id"])
        if group:
            new_com = finite_array(group["centroid_m"], (3,))
            deltas.append({"body": body["id"], "reference_retained_CAD0_m": body["reference_CAD0_m"],
                           "new_rigid_body_COM_CAD0_m": new_com,
                           "new_COM_minus_retained_reference_m": new_com - body["reference_CAD0_m"],
                           "reference_was_replaced": False})
    return {"reference_policy": dataset["reference"]["moment_reference"], "bbox_method": "Group union of original q0 AABBs, minus body-specific original reference; hull and base0 both use Ob.",
            "rigid_body_COM_comparison": deltas,
            "pose_geometry_changes": [
                {"pose": "q0", "cylinder_gap_fill_included": True,
                 "evidence_script": relative_path(HERE.parent / "09_seal_j2j3/sc_build_fluid_domain_v5.py"),
                 "evidence": "Extended l2 unites cyl_fill; b5 uses v5h_e5 real-hull mesh.",
                 "build_report": relative_path(HERE.parent / "09_seal_j2j3/fluid_domain_v5.json")},
                {"pose": "qb", "cylinder_gap_fill_included": False,
                 "evidence_script": relative_path(POSE_SOURCE / "sc_build_fluid_domain_qb.py"),
                 "evidence": "Cylinder fill is calculated/logged but NOT united into extended l2 before posing; header says dropped.",
                 "build_report": relative_path(POSE_SOURCE / "fluid_domain_qb.json"),
                 "stale_report_warning": "Top-level policy and cyl_gap_fill quantities still describe construction, not inclusion."},
                {"pose": "qc", "cylinder_gap_fill_included": False,
                 "evidence_script": relative_path(POSE_SOURCE / "sc_build_fluid_domain_qc.py"),
                 "evidence": "Cylinder fill is calculated/logged but NOT united into extended l2 before posing; header says dropped (gap opens).",
                 "build_report": relative_path(POSE_SOURCE / "fluid_domain_qc.json"),
                 "stale_report_warning": "Top-level policy and cyl_gap_fill quantities still describe construction, not inclusion."},
                {"pose": "qd", "cylinder_gap_fill_included": True,
                 "evidence_script": relative_path(POSE_SOURCE / "sc_build_fluid_domain_qd.py"),
                 "evidence": "Extended l2 explicitly unites cyl_fill before posing.",
                 "build_report": relative_path(POSE_SOURCE / "fluid_domain_qd.json")}],
            "limitation": "Pose comparison is not a pure rigid re-pose experiment: qb/qc omit the cylinder fill, q0/qd retain it. Bboxes are original geometric envelopes, not meshed-surface reconstructions."}


def generate_audit(dataset, closures, paths):
    paths = set(paths)
    cases = []
    for row in dataset["steady"] + dataset["oscillations"]:
        path = WORKSPACE / row["source"]["case_h5"]
        paths.add(path)
        cases.append(inspect_case(path, "selected_dataset_case", row["viscous_model"]))
    for base in ("b5", "bqb", "bqc", "bqd"):
        path = SOURCE / (base + ".cas.h5")
        paths.update((path, SOURCE / (base + "_base.json")))
        cases.append(inspect_case(path, "selected_source_base", "SST"))
    for name in ("b1.cas.h5", "b3.cas.h5", "b4.cas.h5", "runs/_b3_coef_moments/T_px_08.cas.h5"):
        path = SOURCE / name
        paths.add(path)
        cases.append(inspect_case(path, "historical_coefficient_evidence"))
    selected = [r for r in cases if r["role"] == "selected_dataset_case"]
    if not all(r["selected_source_checks_pass"] for r in selected):
        bad = [r["path"] for r in selected if not r["selected_source_checks_pass"]]
        raise ValueError("HDF5 source assumptions not confirmed: " + ", ".join(bad))
    for name in ("build_dataset.py", "build_dataset_pose.py", "build_dataset_qb.py", "bd_lib.py", "fluent_base.py",
                 "fluent_run.py", "export_loads.py", "export_summary.py", "run_steady_batch.py", "run_pose_batch.py",
                 "b1_base.json", "b3_base.json", "b4_base.json", "runs/steady_batch.log", "runs_pose_batch.log",
                 "matlab/theta_fit.json", "matlab/theta_fit.mat", "matlab/fit_theta.m", "matlab/Y_uvms.m",
                 "matlab/theta_names.m", "matlab/uvms_hydro_theta.m", "matlab/uvms_link_kin.m", "matlab/build_chain_cad.m"):
        paths.add(SOURCE / name)
    for name in ("mass_props.json", "mass_table.json", "make_mass_table.py", "sc_mass_props.py",
                 "make_build_pose.py", "make_build_qb.py"):
        paths.add(POSE_SOURCE / name)
    for pose in ("qb", "qc", "qd"):
        paths.update((POSE_SOURCE / ("fluid_domain_" + pose + ".json"), POSE_SOURCE / ("sc_build_fluid_domain_" + pose + ".py")))
    paths.update((HERE.parent / "09_seal_j2j3/fluid_domain_v5.json", HERE.parent / "09_seal_j2j3/sc_build_fluid_domain_v5.py"))
    for name in ("poly_v5h_e5.json", "poly_qb.json", "poly_qc.json", "poly_qd.json"):
        paths.add(HERE.parent / "12_coarse_mesh" / name)
    for name in ("wb_mesh_v5h_e5.wbjn", "make_v5h_e5.py"):
        paths.add(HERE.parent / "12_coarse_mesh" / name)
    manifest = make_source_manifest(paths)
    oscillations = audit_oscillations(dataset)
    powers, maps = [], []
    rng = np.random.default_rng(1401)
    for row in dataset["steady"]:
        J, W = body_jacobians(row["q"], dataset["reference"]), np.array(row["wrenches"])
        g = np.concatenate((row["total"], row["joint_tau"]))
        maps.append(float(np.max(np.abs(np.einsum("bij,bi->j", J, W) - g))))
        for _ in range(4):
            zeta = rng.normal(size=9)
            powers.append(abs(float(np.sum(np.einsum("bij,j->bi", J, zeta) * W) - zeta @ g)))
    theta = load_json(SOURCE / "matlab/theta_fit.json")
    legacy_error = max(c["legacy_27_max_abs"] for c in closures)
    harmonic_error = max(c["legacy_unit_excitation_check"]["recomputed_old_unit_max_abs"] for c in oscillations)
    return {"schema_version": 1, "status": "PASS" if legacy_error < 1e-10 and max(maps) < 1e-10 and max(powers) < 1e-10 and harmonic_error < 1e-8 else "FAIL",
            "source_git_commit": EXPECTED_COMMIT, "observed_git_HEAD": manifest["observed_git_HEAD"],
            "source_manifest": relative_path(HERE / "frozen/source_manifest.json"), "frozen_source_count": manifest["hash_count"],
            "dataset": relative_path(HERE / "data/component_dataset.json"),
            "counts": {"steady": len(dataset["steady"]), "strict": sum(r["strict"] for r in dataset["steady"]),
                       "capped": sum(r["unsteady_mean"] for r in dataset["steady"]),
                       "capped_ids": [r["id"] for r in dataset["steady"] if r["unsteady_mean"]],
                       "oscillations": len(dataset["oscillations"]), "physical_samples_per_oscillation": 240,
                       "identification_samples_per_oscillation": 160,
                       "tail_saved_sample_count_distribution": {str(n): sum(r["sample_count"] == n for r in closures) for n in sorted({r["sample_count"] for r in closures})}},
            "load_units": {"production_force_scale": 1., "production_moment_scale": 1.,
                           "raw_channels": "8 wall x 6 = 48, plus total6 plus three q0 report torques = 57",
                           "moment_origin": "All selected mx/my/mz reports are about global [0,0,0]; tau_j reports have their own fixed q0 centres and axes and are not used for posed labels.",
                           "historical_0_6125": "b1/b3 and archived _b3_coef_moments moment reports scaled=True, reference rho=1.225,V=A=L=1. Conversion to N*m is 0.6125. b4/b5 and every selected case scaled=False; applying 0.6125 again would corrupt physical SI data.",
                           "physical_forces": "Fluid ON body; total includes hull+base0+links, hull-only is wall_auv.",
                           "uncertainty_note": SD_NOTE},
            "steady_recomputation": {"legacy_27_max_abs": legacy_error, "tolerance": 1e-10, "rows": closures},
            "jacobian_power": {"generalized_mapping_max_abs": max(maps), "virtual_power_max_abs_W": max(powers),
                               "number_of_power_checks": len(powers), "tolerance": 1e-10,
                               "note": "Random virtual velocities verify algebra only, not validated independent joint dynamics."},
            "hdf5_settings": {"selected_cases_checked": len(selected), "selected_cases_available": 46,
                              "production_bases_checked": 4, "historical_files_checked": 4,
                              "coverage": "All 40 selected steady and six selected oscillation cas.h5 files plus four production bases; no claim about dat.h5 solver state or excluded failures.",
                              "all_selected_checks_pass": all(r["selected_source_checks_pass"] for r in selected), "files": cases},
            "geometry": geometry_audit(dataset), "oscillations": oscillations,
            "legacy_model": {"original_json": relative_path(SOURCE / "matlab/theta_fit.json"),
                             "original_mat": relative_path(SOURCE / "matlab/theta_fit.mat"), "parameter_count": len(theta["theta"]),
                             "coupling": theta.get("coupling"), "link_cop": theta.get("link_cop"),
                             "policy": "Original 87-parameter artifacts and source scripts hashed only; no default theta or existing package altered."},
            "scope_warnings": ["steady uses SST; oscillations use laminar, not one shared turbulence experiment",
                               "saved snapshots are correlated and sparse (four or eight per 200-iteration tail)",
                               "qb/qc have a seal-geometry change, not pose alone",
                               "fixed-q0 whole-domain oscillations do not validate independent joint motion",
                               "old per-unit-acceleration data are coefficients, not actual sampled loads",
                               "retrospective development only; no new blind validation case was launched"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-only", action="store_true", help="write reusable dataset immediately; skip full source/settings audit")
    parser.add_argument("--run-tests", action="store_true", help="run only the owned data-pipeline unittest file and record actual results")
    args = parser.parse_args()
    if args.dataset_only and args.run_tests:
        parser.error("--run-tests requires full audit, not --dataset-only")
    dataset, closures, paths = generate_dataset()
    write_dataset_outputs(dataset)
    print(json.dumps({"dataset": relative_path(HERE / "data/component_dataset.json"),
                      "steady": len(dataset["steady"]), "strict": sum(r["strict"] for r in dataset["steady"]),
                      "oscillations": len(dataset["oscillations"]), "legacy_max_abs": max(c["legacy_27_max_abs"] for c in closures)}))
    if not args.dataset_only:
        audit = generate_audit(dataset, closures, paths)
        write_json_owned(HERE / "reports/data_audit.json", audit)
        if args.run_tests:
            command = [sys.executable, "-B", "-m", "unittest", "discover", "-s", str(HERE / "tests"),
                       "-p", "test_data_pipeline.py", "-v"]
            completed = subprocess.run(command, cwd=WORKSPACE, capture_output=True, text=True, encoding="utf-8", errors="replace")
            audit["unit_tests"] = {"command": command, "returncode": completed.returncode,
                                   "status": "PASS" if completed.returncode == 0 else "FAIL",
                                   "stdout": completed.stdout, "stderr": completed.stderr}
            if completed.returncode:
                audit["status"] = "FAIL"
            write_json_owned(HERE / "reports/data_audit.json", audit)
            print(completed.stdout + completed.stderr, end="")
        print(json.dumps({"audit": relative_path(HERE / "reports/data_audit.json"), "status": audit["status"]}))
        if audit["status"] != "PASS":
            raise SystemExit(1)


if __name__ == "__main__":
    main()

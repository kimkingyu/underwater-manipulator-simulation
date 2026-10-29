"""Fixed-q0, small-amplitude, 2 s whole-domain dynamic development model.

Physical fluid loads ON bodies, not an actuator compensation term. No steady
records, independent joint excitation, solver launches or legacy theta changes.
Nuisance DC/second harmonics isolate the first harmonic; they are not deployed.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from data_pipeline import body_jacobians

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
SCHEMA = "fixed_q0_common_base_dynamic_v1"
BODY_ORDER = ["hull", "base0", "link1", "link2", "link3"]
CHANNELS = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
INPUTS = ["u", "v", "w", "p", "q", "r"]
KNOWN_THETA_HASHES = {
    "frozen/theta87_baseline.mat": "ce1eb034c2b1a6a396928fc351e4b6dfd6bb361a2f73cb2e214e877bdde35850",
    "frozen/theta87_baseline.json": "d70c8d412497e7502a310cea928762173b4c48fd3d9cb24d48649aeac7aafcbd",
    "../11_theta_fit/matlab/theta_fit.mat": "ce1eb034c2b1a6a396928fc351e4b6dfd6bb361a2f73cb2e214e877bdde35850",
    "../11_theta_fit/matlab/theta_fit.json": "d70c8d412497e7502a310cea928762173b4c48fd3d9cb24d48649aeac7aafcbd",
}
LIMITATIONS = [
    "Development fit to six existing q0 whole-domain laminar oscillations; not independent-case or blind validation.",
    "Only fixed-arm base-twist observation subspace; no full 9x9 independent-joint added-mass identification.",
    "2 s nominal period, 0.02 m translation / 0.05 rad rotation, 0.5 s ramp only; frequency and amplitude generalization unvalidated.",
    "Velocity-phase response is local laminar effective damping, not a replacement for steady SST drag.",
    "PSD/reciprocity imposed only on aggregate six-base response; partial-wall maps need not be reciprocal or individually passive.",
    "Coriolis term is a coordinate/energy-consistent model completion, not an independently identified nonlinear term.",
    "DC and second-harmonic nuisance terms are diagnostics only and are excluded from deployed predictions.",
    "No startup fluid-memory states are identified; the first 80 unfitted onset samples have materially larger residuals than the last two cycles.",
    "Cycle 2 fit / cycle 3 prediction is a repeated-cycle consistency check, not an independent-case generalization test.",
    "Original87 has a combined hull+base0 residual block; separate legacy hull/base0 predictions are unavailable.",
]


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def save_json(path, value):
    path = Path(path)
    if path.parent not in (ROOT / "models", ROOT / "reports") or not path.name.startswith("dynamic_"):
        raise ValueError("Dynamic outputs must stay in owned dynamic_* model/report paths")
    path.write_text(json.dumps(jsonable(value), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def theta_hash_audit():
    actual = {name: sha256_file(ROOT / name) for name in KNOWN_THETA_HASHES}
    if actual != KNOWN_THETA_HASHES:
        raise ValueError("Read-only original theta hashes have changed")
    return actual


def source_fingerprints():
    paths = ["contract.json", "data/interface_schema.json", "data_pipeline.py", "dynamics_linear.py",
             "matlab/predict_dynamic_linear.m", "matlab/dynamic_export_legacy.m",
             "../11_theta_fit/matlab/Y_uvms.m", "../11_theta_fit/matlab/uvms_geometry.m",
             "../11_theta_fit/matlab/uvms_link_kin.m"]
    return {name: sha256_file(ROOT / name) for name in paths}


def finite(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have finite shape {shape}")
    return result


def validate_source(dataset):
    if dataset["reference"]["body_order"] != BODY_ORDER:
        raise ValueError("Wrong source body order")
    cases = dataset["oscillations"]
    if len(cases) != 6 or sorted(r["dof"] for r in cases) != list(range(6)):
        raise ValueError("Exactly six single-axis development oscillations required")
    checks = []
    J = body_jacobians(np.zeros(3), dataset["reference"])
    for r in cases:
        n = len(r["time_s"])
        t = finite(r["time_s"], (n,), "time_s")
        nu = finite(r["nu"], (n, 6), "nu")
        nd = finite(r["nudot"], (n, 6), "nudot")
        W = finite(r["wrenches"], (n, 5, 6), "wrenches")
        total = finite(r["total"], (n, 6), "total")
        tau = finite(r["joint_tau"], (n, 3), "joint_tau")
        mask = np.asarray(r["identification_mask"])
        if mask.dtype != np.bool_ or mask.shape != (n,) or n != 240 or mask.sum() != 160:
            raise ValueError("Source record/mask dimensions differ from agreed data")
        if not np.allclose(r["q"], 0, atol=0, rtol=0) or r["viscous_model"] != "laminar":
            raise ValueError("Source must be fixed-q0 laminar whole-domain motion")
        if r["period_s"] != 2.0 or r["ramp_s"] != 0.5 or not np.all(np.diff(t) > 0):
            raise ValueError("Unexpected source time/frequency specification")
        k = r["dof"]
        if np.max(np.abs(np.delete(nu, k, axis=1))) > 1e-12 or np.max(np.abs(np.delete(nd, k, axis=1))) > 1e-12:
            raise ValueError("This source extractor expects actual single-axis inputs")
        law = r["motion_law"]
        omega, peak, ramp = law["angular_frequency_rad_s"], law["peak_rate"], law["ramp_s"]
        envelope = np.minimum(t / ramp, 1.0)
        rate = peak * np.cos(omega * t) * envelope
        derivative = peak * (-omega * np.sin(omega * t) * envelope + np.cos(omega * t) * (t < ramp) / ramp)
        law_error = max(float(np.max(abs(nu[:, k] - rate))), float(np.max(abs(nd[:, k] - derivative))))
        if law_error > 1e-10 or not np.array_equal(mask, (t > 2.0 + 1e-9) & (t > ramp)):
            raise ValueError("Source trajectory does not match actual HDF named expression/mask")
        closure = np.einsum("bij,tbi->tj", J, W)
        error = np.max(abs(closure - np.column_stack((total, tau))))
        if error > 1e-8:
            raise ValueError("Source component/generalized closure failed")
        checks.append({"id": r["id"], "dof": k, "n": n, "identification_n": int(mask.sum()),
                       "dt_min_s": np.min(np.diff(t)), "dt_max_s": np.max(np.diff(t)),
                       "nominal_period_s": 2.0, "actual_angular_frequency_rad_s": omega,
                       "actual_period_s": 2 * np.pi / omega, "trajectory_max_abs_error": law_error,
                       "component_closure_max_abs": error, "regression_orders": [0, 1, 2],
                       "actual_motion_law": law, "source": r["source"]})
    return checks


def cycle_mask(record, mode):
    t = np.asarray(record["time_s"])
    mask = np.asarray(record["identification_mask"], bool)
    if mode == "last_two_cycles":
        return mask
    if mode == "cycle2":
        return mask & (t <= 2 * record["period_s"] + 1e-9)
    if mode == "cycle3":
        return mask & (t > 2 * record["period_s"] + 1e-9)
    raise ValueError("Unknown training window")


def geometry_scale(dataset):
    hull = next(b for b in dataset["bodies"] if b["id"] == "hull")
    extent = np.asarray(hull["bbox_local_max_m"]) - hull["bbox_local_min_m"]
    length = float(np.linalg.norm(extent) / 2)
    return length, np.diag([1., 1., 1., 1 / length, 1 / length, 1 / length])


def scaled_projection(matrix, T, *, keep_skew=False):
    scaled = T.T @ matrix @ T
    symmetric = (scaled + scaled.T) / 2
    eig, basis = np.linalg.eigh(symmetric)
    clipped = np.maximum(eig, 0.)
    target = (basis * clipped) @ basis.T
    if keep_skew:
        target += (scaled - scaled.T) / 2
    invT = np.linalg.inv(T)
    corrected = invT.T @ target @ invT
    details = {"symmetric_scaled_eigenvalues_before": eig,
               "symmetric_scaled_eigenvalues_after": np.linalg.eigvalsh((target + target.T) / 2),
               "negative_eigenvalues_clipped": int(np.sum(eig < 0)),
               "scaled_frobenius_correction": np.linalg.norm(target - scaled),
               "scaled_relative_frobenius_correction": np.linalg.norm(target - scaled) / max(np.linalg.norm(scaled), 1e-30),
               "scaled_removed_skew_frobenius": 0. if keep_skew else np.linalg.norm((scaled - scaled.T) / 2),
               "delta_physical_rows": corrected - matrix,
               "delta_max_abs_by_output_row": np.max(abs(corrected - matrix), axis=1)}
    return corrected, details


def allocate_correction(X, target, B, row_rms, T):
    """Weighted minimum norm satisfying B*dX=target-B*X exactly.

    Min ||diag(1/sigma) dX T||_F, sigma = training RMS load in each
    physical output row with a 1e-3 N force-equivalent floor. It is an
    estimation constraint, not partial-wall reciprocity or uncertainty proof.
    """
    gram = (B * row_rms ** 2) @ B.T
    correction = (row_rms[:, None] ** 2 * B.T) @ np.linalg.solve(gram, target - B @ X)
    out = X + correction
    return out, {"max_abs_closure": np.max(abs(B @ out - target)),
                 "weighted_scaled_correction_frobenius": np.linalg.norm(correction @ T / row_rms[:, None]),
                 "delta_rows": correction,
                 "delta_max_abs_by_body_output_row": np.max(abs(correction), axis=1).reshape(5, 6)}


def parameter_metadata():
    result = []
    for kind, denom in (("M", ["m/s^2"] * 3 + ["rad/s^2"] * 3), ("D", ["m/s"] * 3 + ["rad/s"] * 3)):
        for b, body in enumerate(BODY_ORDER):
            for row, ch in enumerate(CHANNELS):
                for col, name in enumerate(INPUTS):
                    numerator = "N" if row < 3 else "N*m"
                    result.append({"name": f"{kind}_{body}_{ch}_{name}", "matrix": kind + "_rows",
                                   "row_zero_based": 6 * b + row, "column_zero_based": col,
                                   "unit": numerator + "/(" + denom[col] + ")"})
    return result


def fit_model(dataset, fit_window="last_two_cycles", dataset_sha256=None):
    validate_source(dataset)
    J = body_jacobians(np.zeros(3), dataset["reference"])
    Jrows = J.reshape(30, 9)
    B = Jrows[:, :6].T
    X, Z = np.zeros((30, 6)), np.zeros((30, 6))
    diagnostic, loads = [], []
    for r in sorted(dataset["oscillations"], key=lambda x: x["dof"]):
        mask = cycle_mask(r, fit_window)
        k = r["dof"]
        t, nu, nd, W = [np.asarray(r[key], float) for key in ("time_s", "nu", "nudot", "wrenches")]
        omega = r["motion_law"]["angular_frequency_rad_s"]
        design = np.column_stack((-nd[:, k], -nu[:, k], np.ones(len(t)), np.sin(2 * omega * t), np.cos(2 * omega * t)))
        scale = np.sqrt(np.mean(design[mask] ** 2, axis=0))
        coef_scaled, _, rank, singular = np.linalg.lstsq(design[mask] / scale, W[mask].reshape(mask.sum(), 30), rcond=None)
        if rank != 5:
            raise ValueError("Synchronous regression is not identifiable")
        coef = coef_scaled / scale[:, None]
        X[:, k], Z[:, k] = coef[0], coef[1]
        loads.append(W[mask].reshape(mask.sum(), 30))
        diagnostic.append({"id": r["id"], "dof": k, "sample_count": int(mask.sum()),
                           "rank": int(rank), "condition_after_column_scaling": singular[0] / singular[-1],
                           "design_columns": ["-actual_nudot", "-actual_nu", "constant", "sin(2*actual_omega*t)", "cos(2*actual_omega*t)"],
                           "column_rms": scale, "nuisance_rows": coef[2:],
                           "nuisance_deployed": False,
                           "full_trace_regression_predictions_rows": design @ coef,
                           "fit_residual_rmse_rows": np.sqrt(np.mean((design[mask] @ coef - W[mask].reshape(mask.sum(), 30)) ** 2, axis=0))})
    length, T = geometry_scale(dataset)
    train_loads = np.vstack(loads)
    rms = np.sqrt(np.mean(train_loads ** 2, axis=0))
    local_T = np.tile(np.diag(T), 5)
    rms_equivalent = rms * local_T
    sigma = np.maximum(rms_equivalent, 1e-3) / local_T
    raw_M, raw_D = B @ X, B @ Z
    M, m_project = scaled_projection(raw_M, T)
    D, d_project = scaled_projection(raw_D, T, keep_skew=True)
    Xfinal, m_allocate = allocate_correction(X, M, B, sigma, T)
    Zfinal, d_allocate = allocate_correction(Z, D, B, sigma, T)
    nu_max = np.zeros(6)
    nd_max = np.zeros(6)
    for r in dataset["oscillations"]:
        nu_max = np.maximum(nu_max, np.max(abs(np.asarray(r["nu"])), axis=0))
        nd_max = np.maximum(nd_max, np.max(abs(np.asarray(r["nudot"])), axis=0))
    model = {"schema_version": SCHEMA, "body_order": BODY_ORDER, "output_order": CHANNELS,
             "load_sign": "physical fluid loads ON bodies", "reference": dataset["reference"],
             "M_rows": Xfinal, "D_rows": Zfinal, "J_rows": Jrows,
             "M_total": M, "D_total": D,
             "training": {"dataset_sha256": dataset_sha256,
                          "oscillations_sha256": hashlib.sha256(json.dumps(dataset["oscillations"], sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
                          "case_ids": [r["id"] for r in dataset["oscillations"]], "fit_window": fit_window,
                          "case_count": 6, "fit_sample_count": sum(int(cycle_mask(r, fit_window).sum()) for r in dataset["oscillations"]),
                          "steady_samples_used": 0, "baseline_theta_hashes": theta_hash_audit(),
                          "source_fingerprints": source_fingerprints(),
                          "nuisance_orders": [0, 2], "deployed_linear_order": 1,
                          "viscous_model": "laminar", "nominal_period_s": 2., "ramp_s": .5,
                          "identified_time_window_s": "(2,6]" if fit_window == "last_two_cycles" else "(2,4]",
                          "fluid_memory_states_identified": 0, "startup_transient_validated": False,
                          "amplitude_per_dof": [.02] * 3 + [.05] * 3,
                          "nu_max_abs": nu_max, "nudot_max_abs": nd_max,
                          "motion_laws": [r["motion_law"] for r in dataset["oscillations"]]},
             "scaling": {"characteristic_length_m": length,
                         "rule": "half diagonal of hull geometric bounding box; independent of fitted load labels",
                         "scaled_twist": "[v_x,v_y,v_z,L*w_x,L*w_y,L*w_z] in m/s",
                         "T_diagonal": np.diag(T), "scaled_M_unit": "kg", "scaled_D_unit": "kg/s",
                         "component_RMS_physical_rows": rms, "allocation_sigma_physical_rows": sigma,
                         "force_equivalent_RMS_floor_N": 1e-3},
             "constraint_diagnostics": {"raw_M_total": raw_M, "raw_D_total": raw_D,
                                        "mass_projection": m_project, "damping_projection": d_project,
                                        "mass_component_allocation": m_allocate, "damping_component_allocation": d_allocate,
                                        "allocation_statement": "Training-RMS weighted equality-constrained minimum norm; no partial-wall reciprocity assumption."},
             "parameter_metadata": parameter_metadata(), "limitations": LIMITATIONS,
             "scope": {"q": [0., 0., 0.], "joint_motion_identified": False,
                       "independent_case_validated": False, "blind_validated": False,
                       "candidate_status": "development_only_pending_parent_freeze"}}
    validate_model(model)
    return model, diagnostic


def validate_model(model):
    if model["schema_version"] != SCHEMA or model["body_order"] != BODY_ORDER:
        raise ValueError("Unexpected dynamic model schema/body order")
    M = finite(model["M_rows"], (30, 6), "M_rows")
    D = finite(model["D_rows"], (30, 6), "D_rows")
    J = finite(model["J_rows"], (30, 9), "J_rows")
    mt = finite(model["M_total"], (6, 6), "M_total")
    dt = finite(model["D_total"], (6, 6), "D_total")
    if max(np.max(abs(J[:, :6].T @ M - mt)), np.max(abs(J[:, :6].T @ D - dt))) > 1e-9:
        raise ValueError("Model component closure failed")
    T = np.diag(model["scaling"]["T_diagonal"])
    if np.max(abs(mt - mt.T)) > 1e-9 or np.linalg.eigvalsh(T @ mt @ T).min() < -1e-9:
        raise ValueError("Aggregate inertia must be symmetric PSD")
    if np.linalg.eigvalsh(T @ (dt + dt.T) @ T / 2).min() < -1e-9:
        raise ValueError("Aggregate effective damping symmetric part must be PSD")
    return M, D, J


def inertial_cross(velocity, momentum):
    v, p = np.asarray(velocity), np.asarray(momentum)
    return np.concatenate((np.cross(v[..., 3:], p[..., :3]),
                           np.cross(v[..., :3], p[..., :3]) + np.cross(v[..., 3:], p[..., 3:])), axis=-1)


def _kernel(model, nu, nudot):
    M, D, J = [np.asarray(model[k]) for k in ("M_rows", "D_rows", "J_rows")]
    V, P = (J[:, :6] @ nu).reshape(5, 6), (M @ nu).reshape(5, 6)
    c = inertial_cross(V, P)
    inertia = (M @ nudot).reshape(5, 6)
    damping = (D @ nu).reshape(5, 6)
    W = -(inertia + c + damping)
    generalized = J.T @ W.reshape(30)
    return {"hull": W[0], "base0": W[1], "links": W[2:], "total": generalized[:6],
            "tau_hydro": generalized[6:], "generalized": generalized,
            "inertial_cross_wrenches": c, "inertial_cross_generalized": J.T @ c.reshape(30),
            "dissipation_power_W": float(nu @ np.asarray(model["D_total"]) @ nu)}


def predict_dynamic_linear(model, q, nu, nudot, *, qdot=None, qddot=None, period_s=None,
                           ramp_s=None, motion_kind=None, allow_unvalidated=False):
    if isinstance(model, (str, Path)):
        model = json.loads(Path(model).read_text(encoding="utf-8"))
    validate_model(model)
    q, nu, nd = finite(q, (3,), "q"), finite(nu, (6,), "nu"), finite(nudot, (6,), "nudot")
    qd = np.zeros(3) if qdot is None else finite(qdot, (3,), "qdot")
    qdd = np.zeros(3) if qddot is None else finite(qddot, (3,), "qddot")
    if np.max(abs(q)) > 1e-12 or np.max(abs(qd)) > 1e-12 or np.max(abs(qdd)) > 1e-12:
        raise ValueError("Only q0 with fixed joints is identified; joint/pose extrapolation is refused")
    train = model["training"]
    nr = np.linalg.norm(nu / np.asarray(train["nu_max_abs"]))
    ar = np.linalg.norm(nd / np.asarray(train["nudot_max_abs"]))
    active = np.flatnonzero(abs(nu) / np.asarray(train["nu_max_abs"]) + abs(nd) / np.asarray(train["nudot_max_abs"]) > 1e-12)
    is_zero = not np.any(nu) and not np.any(nd)
    reasons = []
    if not is_zero:
        if period_s is None or not np.isfinite(period_s) or abs(period_s - 2.) > 1e-9:
            reasons.append("period_missing_or_outside_2s_training")
        if ramp_s is None or not np.isfinite(ramp_s) or abs(ramp_s - .5) > 1e-9:
            reasons.append("ramp_missing_or_outside_0p5s_training")
        if motion_kind != "whole_domain_oscillation":
            reasons.append("motion_not_declared_whole_domain_oscillation")
    if nr > 1 + 1e-9:
        reasons.append("outside_training_velocity_envelope")
    if ar > 1 + 1e-9:
        reasons.append("outside_training_acceleration_envelope")
    if len(active) > 1:
        reasons.append("combined_axis_motion_not_independently_validated")
    if reasons and not allow_unvalidated:
        raise ValueError("Dynamic scope: " + ", ".join(reasons))
    out = _kernel(model, nu, nd)
    out["scope_flags"] = {"fixed_q0": True, "independent_joint_motion_identified": False,
                          "full_9x9_inertia_identified": False, "blind_validated": False,
                          "independent_case_validated": False, "empirical_status": "development_only",
                          "within_training_envelope": bool(nr <= 1 + 1e-9 and ar <= 1 + 1e-9),
                          "training_design_axis_or_zero": bool(len(active) <= 1),
                          "extrapolation": bool(reasons), "unvalidated_reasons": reasons,
                          "single_state_cannot_verify_waveform": True, "replaces_steady_SST_drag": False,
                          "startup_transient_validated": False, "fluid_memory_model_included": False}
    return out


def predict_oblique_yz(model, times=None):
    """A priori design prediction only. Does not read any blind CFD artifacts."""
    t = np.arange(1, 241) * .025 if times is None else np.asarray(times, float)
    omega, ramp, amp = np.pi, .5, .02
    direction = np.array([0., 1., 1.]) / np.sqrt(2.)
    envelope = np.minimum(t / ramp, 1.)
    rate = amp * omega * np.cos(omega * t) * envelope
    acceleration = amp * omega * (-omega * np.sin(omega * t) * envelope + np.cos(omega * t) * (t < ramp) / ramp)
    nu, nd = np.zeros((len(t), 6)), np.zeros((len(t), 6))
    nu[:, :3], nd[:, :3] = rate[:, None] * direction, acceleration[:, None] * direction
    predictions = [predict_dynamic_linear(model, np.zeros(3), v, a, period_s=2., ramp_s=.5,
                                         motion_kind="whole_domain_oscillation", allow_unvalidated=True)
                   for v, a in zip(nu, nd)]
    return {"id": "O_V_q0_YZ", "role": "a_priori_nominal_design_prediction_no_labels",
            "direction": direction, "amplitude_m": amp, "period_s": 2., "ramp_s": .5,
            "motion_expression_note": "Nominal double precision pi; actual blind HDF expression must be used for later scoring, without refit.",
            "model_dataset_sha256": model["training"]["dataset_sha256"], "time_s": t, "nu": nu, "nudot": nd,
            "predictions": predictions}


def trace_prediction(model, record):
    predictions = [_kernel(model, np.asarray(v), np.asarray(a)) for v, a in zip(record["nu"], record["nudot"])]
    W = np.asarray([np.vstack((p["hull"], p["base0"], p["links"])) for p in predictions])
    return {"wrenches": W, "total": np.asarray([p["total"] for p in predictions]),
            "joint_tau": np.asarray([p["tau_hydro"] for p in predictions])}


def load_views(record):
    result = {"total": np.asarray(record["total"]), "tau_hydro": np.asarray(record["joint_tau"])}
    if "wrenches" in record:
        W = np.asarray(record["wrenches"])
        for b, body in enumerate(BODY_ORDER):
            result[body] = W[:, b, :]
        result["hull_base0"] = W[:, 0] + W[:, 1]
    else:
        result["hull_base0"] = np.asarray(record["hull_base0"])
        links = np.asarray(record["links_rows"]).reshape(-1, 3, 6)
        for i in range(3):
            result[f"link{i + 1}"] = links[:, i]
    return result


def si_metrics(actual, predicted):
    err = predicted - actual
    return {"MAE": np.mean(abs(err), axis=0), "RMSE": np.sqrt(np.mean(err ** 2, axis=0)),
            "bias": np.mean(err, axis=0), "max_abs": np.max(abs(err), axis=0),
            "target_RMS": np.sqrt(np.mean(actual ** 2, axis=0))}


def metric_rows(actual_views, predicted_views, mask, model_name, window, case_id):
    rows = []
    for name, predicted in predicted_views.items():
        actual = actual_views[name]
        metrics = si_metrics(actual[mask], predicted[mask])
        channels = ["tau1", "tau2", "tau3"] if name == "tau_hydro" else CHANNELS
        for j, channel in enumerate(channels):
            rows.append({"model": model_name, "window": window, "case_id": case_id,
                         "body_or_projection": name, "channel": channel,
                         "unit": "N*m" if name == "tau_hydro" or j >= 3 else "N",
                         "n": int(np.sum(mask)), **{k: float(v[j]) for k, v in metrics.items()}})
    return rows


def pool_time_window_metrics(rows):
    groups = {}
    for row in rows:
        if row["window"] not in ("full_240_including_startup", "startup_first80_unfitted"):
            continue
        key = tuple(row[k] for k in ("window", "model", "body_or_projection", "channel"))
        groups.setdefault(key, []).append(row)
    result = []
    for key, parts in groups.items():
        count = sum(p["n"] for p in parts)
        pooled = {k: v for k, v in zip(("window", "model", "body_or_projection", "channel"), key)}
        pooled.update({"case_id": "six_development_cases", "unit": parts[0]["unit"], "n": count,
                       "max_abs": max(p["max_abs"] for p in parts)})
        for metric in ("MAE", "bias"):
            pooled[metric] = sum(p["n"] * p[metric] for p in parts) / count
        for metric in ("RMSE", "target_RMS"):
            pooled[metric] = np.sqrt(sum(p["n"] * p[metric] ** 2 for p in parts) / count)
        result.append(pooled)
    return result


def harmonic_rows(record, actual_views, predicted_views, mask, model_name):
    t = np.asarray(record["time_s"])[mask]
    omega = record["motion_law"]["angular_frequency_rad_s"]
    design = np.column_stack((np.ones(len(t)), np.sin(omega * t), np.cos(omega * t), np.sin(2 * omega * t), np.cos(2 * omega * t)))
    rows = []
    for name, predicted in predicted_views.items():
        ac = np.linalg.lstsq(design, actual_views[name][mask], rcond=None)[0]
        pc = np.linalg.lstsq(design, predicted[mask], rcond=None)[0]
        chs = ["tau1", "tau2", "tau3"] if name == "tau_hydro" else CHANNELS
        for j, channel in enumerate(chs):
            for order, inds in ((1, [1, 2]), (2, [3, 4])):
                a, p = ac[inds, j], pc[inds, j]
                residual = design[:, inds] @ (p - a)
                rows.append({"model": model_name, "case_id": record["id"], "window": "identification_last160",
                             "body_or_projection": name, "channel": channel, "harmonic_order": order,
                             "unit": "N*m" if name == "tau_hydro" or j >= 3 else "N",
                             "actual_sin_cos": a, "predicted_sin_cos": p,
                             "actual_amplitude": np.linalg.norm(a), "predicted_amplitude": np.linalg.norm(p),
                             "RMSE": np.sqrt(np.mean(residual ** 2)), "MAE": np.mean(abs(residual)),
                             "max_abs": np.max(abs(residual)), "mean_bias": pc[0, j] - ac[0, j]})
    return rows


def kernel_invariants(model, samples=128):
    rng = np.random.default_rng(140014)
    J = np.asarray(model["J_rows"]).reshape(5, 6, 9)
    M, D = np.asarray(model["M_total"]), np.asarray(model["D_total"])
    checks = {"max_abs_component_total_closure": 0., "max_abs_cross_coordinate_closure": 0.,
              "max_abs_cross_zero_power": 0., "max_abs_jacobian_power_closure": 0.,
              "min_dissipation_W": float("inf")}
    for _ in range(samples):
        nu = rng.normal(size=6) * np.asarray(model["training"]["nu_max_abs"]) / 4
        nd = rng.normal(size=6) * np.asarray(model["training"]["nudot_max_abs"]) / 4
        p = _kernel(model, nu, nd)
        cross_total = inertial_cross(nu, M @ nu)
        expected = -M @ nd - cross_total - D @ nu
        checks["max_abs_component_total_closure"] = max(checks["max_abs_component_total_closure"], float(np.max(abs(p["total"] - expected))))
        checks["max_abs_cross_coordinate_closure"] = max(checks["max_abs_cross_coordinate_closure"], float(np.max(abs(p["inertial_cross_generalized"][:6] - cross_total))))
        checks["max_abs_cross_zero_power"] = max(checks["max_abs_cross_zero_power"], abs(float(nu @ cross_total)))
        test_z = rng.normal(size=9)
        W = np.vstack((p["hull"], p["base0"], p["links"]))
        local_power = np.sum(np.einsum("bij,j->bi", J, test_z) * W)
        checks["max_abs_jacobian_power_closure"] = max(checks["max_abs_jacobian_power_closure"], abs(float(test_z @ p["generalized"] - local_power)))
        checks["min_dissipation_W"] = min(checks["min_dissipation_W"], p["dissipation_power_W"])
    checks["random_algebra_cases"] = samples
    checks["zero_state_max_abs"] = float(np.max(abs(_kernel(model, np.zeros(6), np.zeros(6))["generalized"])))
    checks["algebraic_only_not_new_empirical_validation"] = True
    return checks


def build_reports(dataset, model, repeat_model, diagnostic, legacy=None):
    metrics, harmonics, trace_cases = [], [], []
    pooled_actual, pooled_new, pooled_old, pooled_repeat = {}, {}, {}, {}
    old_by_id = {} if legacy is None else {r["id"]: r for r in legacy["cases"]}
    diag_by_id = {r["id"]: r for r in diagnostic}
    for r in dataset["oscillations"]:
        actual = load_views(r)
        new_trace = trace_prediction(model, r)
        repeat_trace = trace_prediction(repeat_model, r)
        new = load_views(new_trace)
        repeat = load_views(repeat_trace)
        mask = np.asarray(r["identification_mask"], bool)
        c3 = cycle_mask(r, "cycle3")
        for window, use in (("full_240_including_startup", np.ones(len(mask), bool)), ("identification_last160", mask), ("startup_first80_unfitted", ~mask)):
            metrics.extend(metric_rows(actual, new, use, "constrained_dynamic", window, r["id"]))
        metrics.extend(metric_rows(actual, repeat, c3, "cycle2_fit_cycle3_repeat", "cycle3_repeat80", r["id"]))
        harmonics.extend(harmonic_rows(r, actual, new, mask, "constrained_dynamic"))
        # Diagnostic-only nuisance reconstruction, kept visibly separate from candidate.
        Wdiag = np.asarray(diag_by_id[r["id"]]["full_trace_regression_predictions_rows"]).reshape(-1, 5, 6)
        gdiag = Wdiag.reshape(-1, 30) @ np.asarray(model["J_rows"])
        diag_views = load_views({"wrenches": Wdiag, "total": gdiag[:, :6], "joint_tau": gdiag[:, 6:]})
        harmonics.extend(harmonic_rows(r, actual, diag_views, mask, "diagnostic_regression_with_nuisance_NOT_DEPLOYED"))
        case = {"id": r["id"], "time_s": r["time_s"], "nu": r["nu"], "nudot": r["nudot"],
                "actual": {"wrenches": r["wrenches"], "total": r["total"], "joint_tau": r["joint_tau"]},
                "candidate": new_trace, "cycle2_fit_cycle3_repeat": repeat_trace}
        for name in actual:
            pooled_actual.setdefault(name, []).append(actual[name][mask])
            pooled_new.setdefault(name, []).append(new[name][mask])
            pooled_repeat.setdefault(name, []).append(repeat[name][c3])
        if r["id"] in old_by_id:
            old_record = old_by_id[r["id"]]
            old = load_views(old_record)
            for window, use in (("full_240_including_startup", np.ones(len(mask), bool)), ("identification_last160", mask), ("startup_first80_unfitted", ~mask)):
                metrics.extend(metric_rows(actual, old, use, "original87", window, r["id"]))
            harmonics.extend(harmonic_rows(r, actual, old, mask, "original87"))
            case["original87"] = old_record
            for name in old:
                pooled_old.setdefault(name, []).append(old[name][mask])
        trace_cases.append(case)
    actual_pool = {k: np.vstack(v) for k, v in pooled_actual.items()}
    new_pool = {k: np.vstack(v) for k, v in pooled_new.items()}
    all_mask = np.ones(960, bool)
    aggregate = metric_rows(actual_pool, new_pool, all_mask, "constrained_dynamic", "identification_last160_pooled6", "six_development_cases")
    if pooled_old:
        aggregate.extend(metric_rows(actual_pool, {k: np.vstack(v) for k, v in pooled_old.items()}, all_mask,
                                     "original87", "identification_last160_pooled6", "six_development_cases"))
    repeat_actual = {k: np.vstack([load_views(r)[k][cycle_mask(r, "cycle3")] for r in dataset["oscillations"]]) for k in actual_pool}
    aggregate.extend(metric_rows(repeat_actual, {k: np.vstack(v) for k, v in pooled_repeat.items()}, np.ones(480, bool),
                                 "cycle2_fit_cycle3_repeat", "cycle3_repeat80_pooled6", "six_development_cases"))
    report = {"status": "development_only_pending_parent_freeze", "dataset_sha256": model["training"]["dataset_sha256"],
              "limits": LIMITATIONS, "source_checks": validate_source(dataset),
              "fit_samples": model["training"]["fit_sample_count"], "repeat_fit_samples": repeat_model["training"]["fit_sample_count"],
              "aggregate_SI_metrics": aggregate, "aggregate_full_timeseries_SI_metrics": pool_time_window_metrics(metrics),
              "per_case_channel_SI_metrics": metrics, "harmonic_SI_metrics": harmonics,
              "constraint_diagnostics": model["constraint_diagnostics"], "algebraic_tests": kernel_invariants(model),
              "baseline_available": legacy is not None, "baseline_separate_hull_base0_available": False,
              "baseline_component_policy": "Use only combined hull_base0, actual local link wrenches, total and J-projected tau; no arbitrary split.",
              "theta_hash_audit": theta_hash_audit()}
    traces = {"dataset_sha256": model["training"]["dataset_sha256"], "body_order": BODY_ORDER,
              "role": "Physical SI development time histories, not unit-acceleration pseudo-samples or blind tests", "cases": trace_cases}
    return report, traces


def save_html_report(report):
    new = {(r["body_or_projection"], r["channel"]): r for r in report["aggregate_SI_metrics"] if r["model"] == "constrained_dynamic"}
    old = {(r["body_or_projection"], r["channel"]): r for r in report["aggregate_SI_metrics"] if r["model"] == "original87"}
    repeat = {(r["body_or_projection"], r["channel"]): r for r in report["aggregate_SI_metrics"] if r["model"] == "cycle2_fit_cycle3_repeat"}
    rows = []
    for key, r in new.items():
        old_r = old.get(key)
        values = [key[0], key[1], r["unit"], "not separable" if old_r is None else f'{old_r["RMSE"]:.7g}',
                  f'{r["RMSE"]:.7g}', f'{r["MAE"]:.7g}', f'{r["bias"]:.7g}', f'{r["max_abs"]:.7g}', f'{repeat[key]["RMSE"]:.7g}']
        rows.append("<tr>" + "".join("<td>" + html.escape(v) + "</td>" for v in values) + "</tr>")
    contents = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Dynamic q0 development</title><style>body{font:15px sans-serif;max-width:1200px;margin:32px auto}table{border-collapse:collapse}td,th{padding:7px;border:1px solid #bbb;text-align:right}h1{font-size:24px}li{margin:8px}</style><h1>固定臂 q0 动态候选：开发数据与重复周期检查</h1>'
    contents += "<p>未做盲测或独立工况验证；未替换旧默认参数。末160点拟合，共960物理时刻。周期2→周期3仅重复周期检查。</p><ul>"
    contents += "".join("<li>" + html.escape(s) + "</li>" for s in LIMITATIONS) + "</ul>"
    full = report["aggregate_full_timeseries_SI_metrics"]
    old_full = {(r["window"], r["body_or_projection"], r["channel"]): r for r in full if r["model"] == "original87"}
    startup_rows = []
    for r in full:
        if r["model"] != "constrained_dynamic" or r["body_or_projection"] not in ("total", "tau_hydro"):
            continue
        key = (r["window"], r["body_or_projection"], r["channel"])
        old_r = old_full.get(key)
        values = ["全240点" if r["window"] == "full_240_including_startup" else "启动前80点（未拟合）", key[1], key[2], r["unit"],
                  "unavailable" if old_r is None else f'{old_r["RMSE"]:.7g}', f'{r["RMSE"]:.7g}', f'{r["max_abs"]:.7g}']
        startup_rows.append("<tr>" + "".join("<td>" + html.escape(v) + "</td>" for v in values) + "</tr>")
    contents += '<h2>先看全时序与启动段</h2><p>未辨识启动流场记忆；全程误差显著大于末两周期误差，不能把周期拟合替代全程动态验证。</p>'
    contents += '<table><tr><th>窗口</th><th>组</th><th>通道</th><th>SI单位</th><th>旧87 RMSE</th><th>新 RMSE</th><th>新 max</th></tr>' + "".join(startup_rows) + '</table>'
    contents += '<h2>末160点：各分体与投影通道</h2><p>下表不是独立工况验证。link1 Mz 与 link2 Mz 的极小通道误差并未改善。</p>'
    contents += '<table><tr><th>组</th><th>通道</th><th>SI单位</th><th>旧87 RMSE</th><th>新 RMSE</th><th>新 MAE</th><th>新 bias</th><th>新 max</th><th>周期2→3 RMSE</th></tr>' + "".join(rows) + "</table>"
    contents += "<h2>约束与代数检查</h2><pre>" + html.escape(json.dumps(jsonable({"mass_projection": report["constraint_diagnostics"]["mass_projection"], "damping_projection": report["constraint_diagnostics"]["damping_projection"], "algebraic_tests": report["algebraic_tests"]}), indent=2)) + "</pre></html>"
    (ROOT / "reports/dynamic_development.html").write_text(contents, encoding="utf-8")


def cross_language_fixture(model):
    rng = np.random.default_rng(14087)
    cases = []
    for _ in range(48):
        nu = rng.normal(size=6) * np.asarray(model["training"]["nu_max_abs"]) / 5
        nd = rng.normal(size=6) * np.asarray(model["training"]["nudot_max_abs"]) / 5
        options = {"period_s": 2., "ramp_s": .5, "motion_kind": "whole_domain_oscillation", "allow_unvalidated": True}
        expected = predict_dynamic_linear(model, np.zeros(3), nu, nd, **options)
        cases.append({"q": [0., 0., 0.], "nu": nu, "nudot": nd, "options": options, "expected": expected})
    return {"dataset_sha256": model["training"]["dataset_sha256"], "cases": cases}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab", action="store_true", help="Algebraic prediction cross-check and read-only legacy export, never simulation")
    args = parser.parse_args(argv)
    dataset_path = ROOT / "data/component_dataset.json"
    raw = dataset_path.read_bytes()
    data_sha = hashlib.sha256(raw).hexdigest()
    dataset = json.loads(raw)
    model, diag = fit_model(dataset, dataset_sha256=data_sha)
    repeat_model, repeat_diag = fit_model(dataset, "cycle2", data_sha)
    save_json(ROOT / "models/dynamic_linear_q0.json", model)
    save_json(ROOT / "models/dynamic_repeat_cycle2_q0.json", repeat_model)
    save_json(ROOT / "reports/dynamic_regression_diagnostics.json", {"dataset_sha256": data_sha, "final": diag, "cycle2_only": repeat_diag})
    report, traces = build_reports(dataset, model, repeat_model, diag)
    save_json(ROOT / "reports/dynamic_traces.json", traces)
    save_json(ROOT / "reports/dynamic_cross_language_fixture.json", cross_language_fixture(model))
    save_json(ROOT / "reports/dynamic_oblique_yz_design_prediction.json", predict_oblique_yz(model))
    if args.matlab:
        folder = str(ROOT / "matlab").replace("\\", "/").replace("'", "''")
        with (ROOT / "reports/dynamic_matlab_run.log").open("w", encoding="utf-8") as log:
            proc = subprocess.run(["matlab", "-batch", f"addpath('{folder}'); dynamic_export_legacy;"], stdout=log, stderr=subprocess.STDOUT, timeout=600)
        if proc.returncode:
            raise RuntimeError("MATLAB algebra check failed; inspect dynamic_matlab_run.log")
    legacy_path = ROOT / "reports/dynamic_legacy_traces.json"
    if legacy_path.exists():
        legacy = json.loads(legacy_path.read_text(encoding="utf-8"))
        if legacy.get("dataset_sha256") != data_sha:
            raise ValueError("Stale legacy export: rerun --matlab against current input")
        report, traces = build_reports(dataset, model, repeat_model, diag, legacy)
    matlab_path = ROOT / "reports/dynamic_matlab_tests.json"
    if matlab_path.exists():
        matlab = json.loads(matlab_path.read_text(encoding="utf-8"))
        if matlab.get("dataset_sha256") == data_sha and matlab.get("model_sha256") == sha256_file(ROOT / "models/dynamic_linear_q0.json"):
            report["matlab_tests"] = matlab
        elif args.matlab:
            raise RuntimeError("MATLAB report does not match current model/data hash")
    if sha256_file(dataset_path) != data_sha or source_fingerprints() != model["training"]["source_fingerprints"]:
        raise RuntimeError("Input/source changed during fitting; parent must rerun before freezing")
    report["artifact_hashes"] = {"models/dynamic_linear_q0.json": sha256_file(ROOT / "models/dynamic_linear_q0.json"),
                                 "models/dynamic_repeat_cycle2_q0.json": sha256_file(ROOT / "models/dynamic_repeat_cycle2_q0.json")}
    report["source_fingerprints"] = source_fingerprints()
    report["theta_hash_audit"] = theta_hash_audit()
    save_json(ROOT / "reports/dynamic_development.json", report)
    save_json(ROOT / "reports/dynamic_traces.json", traces)
    save_html_report(report)
    compact = {"dataset_sha256": data_sha, "source_cases": 6, "fit_samples": 960,
               "M_scaled_relative_correction": model["constraint_diagnostics"]["mass_projection"]["scaled_relative_frobenius_correction"],
               "D_scaled_relative_correction": model["constraint_diagnostics"]["damping_projection"]["scaled_relative_frobenius_correction"],
               "algebraic_tests": report["algebraic_tests"], "baseline_available": report["baseline_available"]}
    print(json.dumps(jsonable(compact), indent=2))
    return report


if __name__ == "__main__":
    main()

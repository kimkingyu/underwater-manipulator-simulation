"""Read-only, freeze-gated prospective CFD evaluation. No solver or fitting calls.

Default/status mode never opens cfd/runs. Only explicit --score reads blind loads.
--prepare-legacy-prior uses frozen inputs and MATLAB algebra, never CFD results.
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import numpy as np
import data_pipeline as dp
import model_components as mc
import dynamics_linear as dl

_RUNNER_SPEC = importlib.util.spec_from_file_location("stage14_blind_runner_readonly", HERE / "cfd/run_case.py")
runner = importlib.util.module_from_spec(_RUNNER_SPEC)
_RUNNER_SPEC.loader.exec_module(runner)

REQUIRED_FROZEN = {
    "evaluate_blind.py", "data_pipeline.py", "model_components.py", "dynamics_linear.py", "contract.json",
    "cfd/run_case.py", "cfd/validation_cases.json", "frozen/evaluation_protocol.json",
    "frozen/steady_prior_predictions.json", "frozen/oscillation_prior_predictions.json",
    "frozen/legacy87_oscillation_prior.json", "frozen/theta87_baseline.json",
    "models/candidate_A_strict37.json", "models/candidate_B_strict37.json",
    "models/candidate_B_pair_strict37.json", "models/dynamic_linear_q0.json",
}
GROUP_CHANNELS = {
    "hull_force3": ["Fx", "Fy", "Fz"], "hull_moment3": ["Mx", "My", "Mz"],
    "total6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"], "joint3": ["J1", "J2", "J3"],
    "base0_6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
    "link1_6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
    "link2_6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
    "link3_6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
    "hull_plus_base0_6": ["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
}


class EvaluationError(ValueError):
    pass


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def finite(value, shape=None, name="value"):
    try:
        return dp.finite_array(value, shape, name)
    except (ValueError, TypeError) as exc:
        raise EvaluationError(str(exc)) from exc


def require_close(actual, expected, name, tolerance=1e-10):
    a, b = finite(actual, name=name), finite(expected, name="expected " + name)
    if a.shape != b.shape or not np.allclose(a, b, rtol=0, atol=tolerance):
        raise EvaluationError(name + " mismatch")


def write_report(path, value):
    path = Path(path).resolve()
    if path.parent != (HERE / "reports").resolve() or not path.name.startswith("evaluator_") or path.suffix != ".json":
        raise EvaluationError("only reports/evaluator_*.json output is allowed")
    path.write_text(json.dumps(dp.serializable(value), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def verify_evaluation_freeze(root, manifest_path=None):
    """Verify runner freeze first, then require all scoring dependencies in that freeze."""
    root = Path(root).resolve()
    config_path = root / "cfd/validation_cases.json"
    manifest_path = Path(manifest_path) if manifest_path else root / "frozen/blind_protocol.json"
    verified = runner.verify_freeze(manifest_path, config_path, root)
    manifest = read_json(manifest_path)
    records = {r["path"]: r["sha256"] for r in manifest["files"]}
    missing = REQUIRED_FROZEN - set(records)
    if missing:
        raise EvaluationError("scoring dependencies absent from blind freeze: " + ", ".join(sorted(missing)))
    return verified, records


def validate_metadata(report, case, cfg, reference):
    if report.get("case_id") != case["id"] or report.get("mode") != case["mode"]:
        raise EvaluationError("result case/mode identity mismatch")
    if report.get("spec") != case:
        raise EvaluationError("result spec/pose differs from frozen case")
    require_close(report.get("origin_m"), reference["origin_Ob_CAD_m"], "original Ob")
    require_close(cfg["origin_m"], reference["origin_Ob_CAD_m"], "config Ob")
    require_close(cfg["joint_centres_CAD0_m"], reference["joint_centres_CAD0_m"], "joint reference centres")
    require_close(cfg["joint_axes_CAD0"], reference["joint_axes_CAD0"], "joint reference axes")
    expected_joints = runner.posed_joints(cfg, case["q"])
    actual_joints = report.get("joint_reports_initial")
    if not isinstance(actual_joints, list) or len(actual_joints) != 3:
        raise EvaluationError("missing initial posed joint reports")
    for actual, expected in zip(actual_joints, expected_joints):
        if actual.get("name") != expected["name"] or actual.get("zones") != expected["zones"]:
            raise EvaluationError("joint report zones/name mismatch")
        require_close(actual.get("centre_m"), expected["centre_m"], "posed joint centre")
        require_close(actual.get("axis"), expected["axis"], "posed joint axis")
    if report.get("failed_required") != [] or any(s.get("required") and not s.get("ok") for s in report.get("steps", [])):
        raise EvaluationError("required runner step failed")


def validate_loads(history):
    if not history:
        raise EvaluationError("empty history")
    for sample in history:
        loads = sample.get("loads", {})
        if set(loads) != runner.CHANNELS:
            raise EvaluationError("history must contain all 57 raw load channels")
        values = finite(list(loads.values()), (57,), "raw loads")
        if np.max(np.abs(values)) > 1e6:
            raise EvaluationError("raw loads exceed runner divergence guard")


def read_actual_motion_law(case_path, spec):
    """Use the validated saved case's actual 17-digit expressions, not requested pi."""
    text = dp.read_case_settings(case_path)
    rates, frequencies, ramps, expressions = [], [], [], []
    for axis in "xyz":
        match = re.search(r'\(name \. "blind_osc_rate_' + axis + r'"\)\s*\(definition \. "([^"]+)"\)', text)
        if not match:
            raise EvaluationError("saved blind motion expression missing for " + axis)
        expression = match.group(1)
        fields = re.fullmatch(r'([^ ]+) \[m s\^-1\] \* cos\(([^ ]+) \[s\^-1\] \* t\) \* min\(t / ([^ ]+) \[s\], 1\)', expression)
        if not fields:
            raise EvaluationError("unsupported saved motion expression")
        rate, frequency, ramp = map(float, fields.groups())
        rates.append(rate)
        frequencies.append(frequency)
        ramps.append(ramp)
        expressions.append(expression)
    require_close(frequencies, [frequencies[0]] * 3, "actual common frequency", 1e-14)
    require_close(ramps, [ramps[0]] * 3, "actual common ramp", 1e-14)
    require_close(rates, np.asarray(spec["direction"]) * spec["amplitude_m"] * (2 * math.pi / spec["period_s"]), "saved velocity amplitude", 1e-12)
    require_close([frequencies[0], ramps[0]], [2 * math.pi / spec["period_s"], spec["ramp_s"]], "saved motion/requested law", 1e-12)
    return {"peak_velocity_m_s": rates, "angular_frequency_rad_s": frequencies[0], "ramp_s": ramps[0],
            "expressions": expressions, "source": "validated start.cas.h5 named-expressions", "case_sha256": runner.sha256(case_path)}


def collect_steady(history, report, case, reference, cfg):
    validate_loads(history)
    last = report["terminal_state"]["iterations"]
    expected_iters = list(range(25, last + 1, 25))
    if [h.get("iter") for h in history] != expected_iters:
        raise EvaluationError("steady saved iteration sequence does not match frozen runner")
    if last < cfg["solver"]["steady_min_iterations"]:
        raise EvaluationError("insufficient steady iterations")
    stats = dp.average_window(history, case["q"], reference, 200)
    recomputed_drift = runner.load_drift(history, 8)
    strict = report.get("converged") is True and recomputed_drift["maximum"] < cfg["solver"]["steady_drift_tolerance"]
    if report.get("converged") is True and not strict:
        raise EvaluationError("claimed convergence does not reproduce the original load-drift criterion")
    capped = report.get("converged") is False and last == cfg["solver"]["steady_max_iterations"]
    if not (strict or capped):
        raise EvaluationError("execution complete is not a valid strict/capped steady result")
    expected_outcome = "strict_window_drift" if strict else "capped_unsteady_mean"
    if report.get("outcome") != expected_outcome:
        raise EvaluationError("steady outcome/convergence mismatch")
    return {"id": case["id"], "mode": case["mode"], "q": case["q"], "strict": strict, "capped": capped,
            "converged": strict, "execution_complete": True, "iterations": last, "drift_recomputed": recomputed_drift,
            "wrenches": stats["wrenches"][None, ...], "total": stats["total"][None, ...],
            "joint_tau": stats["joint_tau"][None, ...], "nu": np.array([case["body_velocity_m_s"] + [0., 0., 0.]]),
            "uncertainty_wrenches_sd": stats["wrenches_sd"], "uncertainty_total_sd": stats["total_sd"],
            "uncertainty_tau_sd": stats["joint_tau_sd"], "averaging": stats["averaging"],
            "uncertainty_note": dp.SD_NOTE}


def collect_oscillation(history, report, case, reference, law):
    validate_loads(history)
    t = finite([h.get("t") for h in history], (240,), "physical times")
    require_close(t, np.arange(1, 241) * case["time_step_s"], "physical time grid", 1e-12)
    nu, nudot, origins, mesh, commanded, rows = [], [], [], [], [], []
    O = np.asarray(reference["origin_Ob_CAD_m"])
    expected_v, expected_a = np.zeros((240, 3)), np.zeros((240, 3))
    for k, rate in enumerate(law["peak_velocity_m_s"]):
        _, expected_v[:, k], expected_a[:, k] = dp.ramped_cosine_motion(t, rate, law["angular_frequency_rad_s"], law["ramp_s"])
    for i, sample in enumerate(history):
        motion = sample.get("motion", {})
        if "moving_origin_m" not in sample or "mesh_displacement_m" not in motion:
            raise EvaluationError("actual moving_origin_m and mesh_displacement_m are required; commanded fallback forbidden")
        origin = finite(sample["moving_origin_m"], (3,), "actual moving origin")
        displacement = finite(motion["mesh_displacement_m"], (3,), "actual mesh displacement")
        require_close(origin, O + displacement, "actual moving origin / mesh displacement", 1e-12)
        velocity = finite(motion.get("body_velocity_m_s"), (3,), "recorded velocity")
        acceleration = finite(motion.get("body_acceleration_m_s2"), (3,), "recorded acceleration")
        require_close(velocity, expected_v[i], "history velocity / saved motion law")
        require_close(acceleration, expected_a[i], "history acceleration / saved motion law")
        require_close(motion.get("water_relative_velocity_m_s"), -velocity, "water/body velocity sign")
        command = finite(motion.get("displacement_m"), (3,), "commanded displacement metadata")
        # Deliberately pass the measured mesh point, never O + command.
        row = dp.loads_to_components(sample["loads"], case["q"], reference, origin_global=origin,
                                     rotation_global_from_body=np.eye(3))
        rows.append(row)
        origins.append(origin)
        mesh.append(displacement)
        commanded.append(command)
        nu.append(np.r_[velocity, [0., 0., 0.]])
        nudot.append(np.r_[acceleration, [0., 0., 0.]])
    return {"id": case["id"], "mode": case["mode"], "q": case["q"], "time_s": t,
            "execution_complete": True, "converged": None,
            "periodicity_assumed": False, "viscous_model": "laminar", "actual_motion_law": law,
            "nu": np.array(nu), "nudot": np.array(nudot), "moving_origin_m": np.array(origins),
            "mesh_displacement_m": np.array(mesh), "commanded_displacement_m": np.array(commanded),
            "mesh_minus_commanded_displacement_m": np.array(mesh) - np.array(commanded),
            "reference_source": "history[*].moving_origin_m (actual measured hull-centroid displacement)",
            "wrenches": np.array([r["wrenches"] for r in rows]), "total": np.array([r["total"] for r in rows]),
            "joint_tau": np.array([r["joint_tau"] for r in rows]), "period_s": case["period_s"], "ramp_s": case["ramp_s"]}


def read_verified_case(attempt, case, cfg, freeze, reference, root):
    """No history conversion until full final artifact/identity verification succeeds."""
    attempt = Path(attempt)
    base = runner.base_status(cfg, case, root)
    if not base["ready"]:
        raise EvaluationError("frozen base missing/hash mismatch: " + case["pose_id"])
    expected = runner.identity_for(case, freeze, base)
    if not runner.valid_completed(attempt, expected):
        raise EvaluationError("valid_completed rejected artifact hashes/state/identity: " + case["id"])
    result_hash_before = runner.sha256(attempt / "result.json")
    report = read_json(attempt / "result.json")
    validate_metadata(report, case, cfg, reference)
    history = read_json(attempt / "history.json")
    if case["mode"] == "steady_translation":
        record = collect_steady(history, report, case, reference, cfg)
    else:
        law = read_actual_motion_law(attempt / "start.cas.h5", case)
        record = collect_oscillation(history, report, case, reference, law)
    if runner.sha256(attempt / "result.json") != result_hash_before or not runner.valid_completed(attempt, expected):
        raise EvaluationError("artifact/result changed during read-only collection")
    record["source"] = {"attempt": str(attempt), "artifacts": report["artifacts"], "identity": report["identity"],
                        "result_sha256": result_hash_before, "terminal_state": report["terminal_state"]}
    return record


def observed_views(record):
    W = finite(record["wrenches"], name="observed wrenches")
    if W.ndim != 3 or W.shape[1:] != (5, 6):
        raise EvaluationError("observed wrenches shape must be Nx5x6")
    return {"hull_force3": W[:, 0, :3], "hull_moment3": W[:, 0, 3:], "base0_6": W[:, 1],
            "link1_6": W[:, 2], "link2_6": W[:, 3], "link3_6": W[:, 4],
            "total6": finite(record["total"], (len(W), 6)), "joint3": finite(record["joint_tau"], (len(W), 3)),
            "hull_plus_base0_6": W[:, 0] + W[:, 1]}


def predicted_views(predictions):
    if not predictions:
        raise EvaluationError("empty prediction list")
    result = {"total6": finite([p["total"] for p in predictions], (len(predictions), 6)),
              "joint3": finite([p["tau_hydro"] for p in predictions], (len(predictions), 3))}
    if all("hull" in p and "base0" in p for p in predictions):
        hull, base0 = finite([p["hull"] for p in predictions]), finite([p["base0"] for p in predictions])
        result.update(hull_force3=hull[:, :3], hull_moment3=hull[:, 3:], base0_6=base0,
                      hull_plus_base0_6=hull + base0)
    elif all("hull_plus_base0" in p and "hull" not in p and "base0" not in p for p in predictions):
        result["hull_plus_base0_6"] = finite([p["hull_plus_base0"] for p in predictions])
    else:
        raise EvaluationError("inconsistent component/combined-hull prediction scope")
    links = finite([p["links"] for p in predictions], (len(predictions), 3, 6))
    for i in range(3):
        result[f"link{i+1}_6"] = links[:, i]
    return result


def si_metrics(actual, predicted, case_indices=None):
    """Equal whole-case contribution; never pool mixed force/moment units into one score."""
    actual, predicted = finite(actual), finite(predicted)
    if actual.shape != predicted.shape or actual.ndim != 2 or not len(actual):
        raise EvaluationError("metrics need equally shaped nonempty sample-by-channel arrays")
    error = predicted - actual
    groups = np.arange(len(error)) if case_indices is None else np.asarray(case_indices)
    if groups.shape != (len(error),):
        raise EvaluationError("case index shape mismatch")
    ids = np.unique(groups)
    return {"MAE": np.mean([np.mean(np.abs(error[groups == i]), axis=0) for i in ids], axis=0),
            "RMSE": np.sqrt(np.mean([np.mean(error[groups == i] ** 2, axis=0) for i in ids], axis=0)),
            "signed_bias": np.mean([np.mean(error[groups == i], axis=0) for i in ids], axis=0),
            "maximum_absolute_error": np.max(np.abs(error), axis=0), "case_count": len(ids), "sample_count": len(error)}


def metric_groups(actual_views, prediction_views, mask=None, case_indices=None):
    result = {}
    for group, predicted in prediction_views.items():
        actual = actual_views[group]
        keep = np.ones(len(actual), dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
        if not np.any(keep):
            continue
        groups = None if case_indices is None else np.asarray(case_indices)[keep]
        metrics = si_metrics(actual[keep], predicted[keep], groups)
        names = GROUP_CHANNELS[group]
        units = ["N" if name.startswith("F") else "N*m" for name in names]
        metrics.update(channels=names, units=units)
        if len(set(units)) == 1:
            metrics["macro_RMSE"] = float(np.mean(metrics["RMSE"]))
            metrics["macro_RMSE_unit"] = units[0]
        result[group] = metrics
    return result


def frozen87_dynamic(theta, q, nu, nudot, geometry):
    """Original 87 model, q0 locked-arm translation only; no new fitted coefficients."""
    q, v, a, t = finite(q, (3,)), finite(nu, (6,)), finite(nudot, (6,)), finite(theta, (87,))
    if np.max(abs(q)) > 1e-12 or np.max(abs(v[3:])) > 1e-12 or np.max(abs(a[3:])) > 1e-12:
        raise EvaluationError("legacy dynamic reference supports fixed q0 translation only")
    old = mc.frozen87_steady(t, q, v, geometry)
    W = np.zeros((5, 6))
    W[0] = np.asarray(old["hull_plus_base0"]) - t[:6] * a
    W[2:] = np.asarray(old["links"])
    J = mc.body_jacobians(q, geometry)
    for i in range(3):
        local_acceleration = J[i + 2] @ np.r_[a, [0., 0., 0.]]
        W[i + 2, :3] -= t[18 + 6*i:21 + 6*i] * local_acceleration[:3]
    result = mc.recompose(q, W, geometry)
    result["hull_plus_base0"] = result.pop("hull")
    result.pop("base0")
    result["scope_flags"] = {"pure_hull_available": False, "fixed_q0_translation_only": True,
                             "parameters_refitted": False, "physical_fluid_load_sign": "negative Y_uvms theta",
                             "note": "hull_plus_base0 is a model block, never legacy total minus measured base0"}
    return result


def assert_prediction_closure(prediction, q, reference):
    W = np.zeros((5, 6))
    if "hull" in prediction:
        W[0], W[1] = prediction["hull"], prediction["base0"]
    else:
        W[0] = prediction["hull_plus_base0"]
    W[2:] = prediction["links"]
    recalculated = dp.recompose_components(W, q, reference)
    require_close(prediction["total"], recalculated["total"], "frozen prediction total closure")
    require_close(prediction["tau_hydro"], recalculated["joint_tau"], "frozen prediction joint closure")


def steady_prior_map(prior, cases, reference):
    records = {r["id"]: r for r in prior.get("cases", [])}
    expected = {c["id"] for c in cases if c["mode"] == "steady_translation"}
    if set(records) != expected or len(prior["cases"]) != len(expected):
        raise EvaluationError("frozen steady predictions must match exactly eight case IDs")
    for case in cases:
        if case["mode"] != "steady_translation":
            continue
        row = records[case["id"]]
        require_close(row["q"], case["q"], "prior q")
        require_close(row["nu"], case["body_velocity_m_s"] + [0., 0., 0.], "prior velocity")
        if set(row["models"]) != {"A", "B", "B_pair", "legacy87"}:
            raise EvaluationError("unfrozen model name/architecture in predictions")
        for name, prediction in row["models"].items():
            assert_prediction_closure(prediction, case["q"], reference)
            if name == "legacy87" and ("hull" in prediction or "base0" in prediction):
                raise EvaluationError("legacy87 cannot provide pure hull/base0 labels")
    return records


def score_steadies(records, prior, protocol):
    available = [r for r in records if r["mode"] == "steady_translation"]
    results = {}
    for label, subset in (("strict_only", [r for r in available if r["strict"]]), ("strict_plus_capped_sensitivity", available)):
        if not subset:
            results[label] = {"case_ids": [], "status": "pending", "models": {}}
            continue
        actual = {g: np.vstack([observed_views(r)[g] for r in subset]) for g in GROUP_CHANNELS}
        models = {}
        for name in ("A", "B", "B_pair", "legacy87"):
            predictions = predicted_views([prior[r["id"]]["models"][name] for r in subset])
            models[name] = metric_groups(actual, predictions, case_indices=np.arange(len(subset)))
        results[label] = {"case_ids": [r["id"] for r in subset], "case_count": len(subset), "models": models,
                          "capped_ids": [r["id"] for r in subset if r["capped"]], "status": "scored"}
    required = protocol["acceptance"]["new_steady_required_count_for_complete_claim"]
    strict_count = sum(r["strict"] for r in available)
    results["complete_steady_claim"] = "eligible" if strict_count >= required else "inconclusive"
    results["strict_count"] = strict_count
    results["required_strict_count"] = required
    results["execution_complete_count"] = len(available)
    results["execution_complete_is_convergence"] = False
    results["acceptance_by_candidate"] = assess_acceptance(results["strict_only"], protocol, strict_count)
    return results


def assess_acceptance(strict_metrics, protocol, strict_count):
    if strict_count < protocol["acceptance"]["new_steady_required_count_for_complete_claim"]:
        return {name: {"status": "inconclusive", "reason": "fewer_than_eight_strict_new_steadies"} for name in ("B", "B_pair")}
    models = strict_metrics["models"]
    baseline = models["A"]
    reduction = protocol["acceptance"]["stage_target_relative_reduction"]
    out = {}
    for candidate in ("B", "B_pair"):
        metrics, groups = models[candidate], {}
        for label, group, index in (("hull_force_macro_RMSE", "hull_force3", None), ("hull_moment_macro_RMSE", "hull_moment3", None),
                                    ("J2_RMSE", "joint3", 1), ("J3_RMSE", "joint3", 2)):
            base = baseline[group]["macro_RMSE"] if index is None else baseline[group]["RMSE"][index]
            value = metrics[group]["macro_RMSE"] if index is None else metrics[group]["RMSE"][index]
            groups[label] = {"baseline": float(base), "candidate": float(value), "target_max": float((1 - reduction) * base),
                             "passed": bool(value <= (1 - reduction) * base)}
        guards = {}
        for group in ("hull_force3", "hull_moment3", "joint3"):
            b, c = np.asarray(baseline[group]["RMSE"]), np.asarray(metrics[group]["RMSE"])
            # No post-hoc increase from measured tail SD. Passing the stricter 5% rule
            # suffices for max(5%, nonnegative variability); a failure remains unresolved.
            allowed = .05 * b
            guards[group] = {"allowed_regression_SI": allowed, "actual_regression_SI": c - b,
                             "passes_conservative_five_percent_guard": (c - b <= allowed),
                             "additional_variability_allowance_used": False}
        target_pass = all(r["passed"] for r in groups.values())
        guard_pass = all(np.all(r["passes_conservative_five_percent_guard"]) for r in guards.values())
        status = "meets_preregistered_steady_targets" if target_pass and guard_pass else (
            "not_improved" if not target_pass else "inconclusive_unspecified_variability_allowance")
        out[candidate] = {"status": status, "target_groups": groups, "per_channel_guards": guards,
                          "note": "No winner selected or model refit on new truth; correlated tail SD is reported separately, not used to relax acceptance after seeing results."}
    return out


def predict_actual_dynamics(record, dynamic_model, theta, reference, prior, legacy_prior):
    require_close(prior["time_s"], record["time_s"], "frozen dynamic prior time grid")
    require_close(legacy_prior["time_s"], record["time_s"], "frozen legacy prior time grid")
    # Verify replay of the original nominal prior before computing actual-input predictions.
    replay_error = {"dynamic_linear": 0., "legacy87": 0.}
    for i, (v, a) in enumerate(zip(prior["nu"], prior["nudot"])):
        new = dl.predict_dynamic_linear(dynamic_model, [0., 0., 0.], v, a, period_s=2., ramp_s=.5,
                                         motion_kind="whole_domain_oscillation", allow_unvalidated=True)
        old = frozen87_dynamic(theta, [0., 0., 0.], v, a, reference)
        for name, prediction, saved in (("dynamic_linear", new, prior["predictions"][i]), ("legacy87", old, legacy_prior["predictions"][i])):
            assert_prediction_closure(saved, [0., 0., 0.], reference)
            left, right = predicted_views([prediction]), predicted_views([saved])
            error = max(float(np.max(abs(left[g] - right[g]))) for g in left)
            replay_error[name] = max(replay_error[name], error)
            if error >= 1e-10:
                raise EvaluationError("frozen nominal waveform replay failed: " + name)
    period = 2 * math.pi / record["actual_motion_law"]["angular_frequency_rad_s"]
    predictions = {"dynamic_linear": [], "legacy87": []}
    for v, a in zip(record["nu"], record["nudot"]):
        predictions["dynamic_linear"].append(dl.predict_dynamic_linear(dynamic_model, record["q"], v, a, period_s=period,
            ramp_s=record["actual_motion_law"]["ramp_s"], motion_kind="whole_domain_oscillation", allow_unvalidated=True))
        predictions["legacy87"].append(frozen87_dynamic(theta, record["q"], v, a, reference))
    return predictions, replay_error


def score_oscillation(record, predictions):
    actual, t = observed_views(record), np.asarray(record["time_s"])
    period, ramp = record["period_s"], record["ramp_s"]
    windows = {"full": np.ones(len(t), dtype=bool), "startup_first_cycle": t <= period + 1e-10,
               "ramp": t <= ramp + 1e-10, "last_two_cycles": t > period + 1e-10,
               "cycle2": (t > period + 1e-10) & (t <= 2 * period + 1e-10), "cycle3": t > 2 * period + 1e-10}
    metrics, harmonics = {}, {}
    w = record["actual_motion_law"]["angular_frequency_rad_s"]
    for name, preds in predictions.items():
        predicted = predicted_views(preds)
        metrics[name], harmonics[name] = {}, {}
        for window, mask in windows.items():
            metrics[name][window] = metric_groups(actual, predicted, mask, np.zeros(len(t), dtype=int))
        for window in ("full", "startup_first_cycle", "last_two_cycles"):
            mask = windows[window]
            harmonics[name][window] = {}
            for group in predicted:
                obs = dp.fit_harmonics(t[mask], actual[group][mask], w)
                pred = dp.fit_harmonics(t[mask], predicted[group][mask], w)
                keys = ("dc", "first_sin", "first_cos", "first_amplitude", "second_sin", "second_cos", "second_amplitude")
                harmonics[name][window][group] = {"channels": GROUP_CHANNELS[group], "actual": obs, "predicted": pred,
                    "predicted_minus_actual": {k: pred[k] - obs[k] for k in keys}, "load_amplitudes_are_physical_SI": True}
    cycle_delta = {group: {"phase_paired_rms_difference": np.sqrt(np.mean((value[windows["cycle3"]] - value[windows["cycle2"]]) ** 2, axis=0)),
                           "phase_paired_max_abs_difference": np.max(abs(value[windows["cycle3"]] - value[windows["cycle2"]]), axis=0)}
                   for group, value in actual.items()}
    return {"id": record["id"], "metrics": metrics, "harmonics": harmonics, "cycle2_vs_cycle3_repeatability": cycle_delta,
            "sample_counts": {k: int(np.sum(v)) for k, v in windows.items()},
            "harmonic_angular_frequency_rad_s": w, "frequency_source": "actual saved HDF5 named expressions, same for measured and predicted",
            "legacy_pure_hull_comparison": "unavailable; never formed by subtracting measured base0",
            "periodicity_assumed": False, "startup_memory_model_validated": False}


def evaluate(root=HERE, manifest_path=None, score=False):
    root = Path(root).resolve()
    manifest_path = Path(manifest_path) if manifest_path else root / "frozen/blind_protocol.json"
    if not manifest_path.is_file():
        return {"status": "pending", "reason": "blind_protocol_missing", "blind_loads_opened": False,
                "next_action": "Freeze the complete protocol and scoring dependencies before running/scoring CFD."}
    freeze, frozen_files = verify_evaluation_freeze(root, manifest_path)
    if not score:
        return {"status": "pending", "reason": "explicit_score_not_requested", "freeze_verified": True,
                "blind_loads_opened": False, "next_action": "Parent may explicitly run --score after final artifacts exist."}
    cfg = runner.load_config(root / "cfd/validation_cases.json")
    reference = dp.reference_from_contract(read_json(root / "contract.json"))
    protocol = read_json(root / "frozen/evaluation_protocol.json")
    prior = steady_prior_map(read_json(root / "frozen/steady_prior_predictions.json"), cfg["cases"], reference)
    records, pending = [], []
    for case in cfg["cases"]:
        parent = root / "cfd/runs" / case["id"]
        attempts = sorted(parent.glob("attempt_[0-9][0-9][0-9]")) if parent.is_dir() else []
        if not attempts:
            pending.append({"id": case["id"], "reason": "no_attempt"})
            continue
        if len(attempts) > 2:
            raise EvaluationError("more than the approved two attempts: " + case["id"])
        attempt = attempts[-1]
        if not (attempt / "result.json").is_file():
            pending.append({"id": case["id"], "reason": "result_missing", "attempt": str(attempt)})
            continue
        result = read_json(attempt / "result.json")
        if result.get("status") != "completed":
            pending.append({"id": case["id"], "reason": "latest_attempt_not_completed", "status": result.get("status"), "attempt": str(attempt)})
            continue
        records.append(read_verified_case(attempt, case, cfg, freeze, reference, root))
    steady = score_steadies(records, prior, protocol)
    oscillations = []
    for record in records:
        if record["mode"] != "oscillation_translation":
            continue
        model = read_json(root / "models/dynamic_linear_q0.json")
        theta = read_json(root / "frozen/theta87_baseline.json")["theta"]
        dynamic_prior = read_json(root / "frozen/oscillation_prior_predictions.json")
        legacy_prior = read_json(root / "frozen/legacy87_oscillation_prior.json")
        predictions, replay_error = predict_actual_dynamics(record, model, theta, reference, dynamic_prior, legacy_prior)
        score_row = score_oscillation(record, predictions)
        score_row["nominal_frozen_prior_replay_max_abs"] = replay_error
        score_row["parameters_changed_or_refitted"] = False
        oscillations.append(score_row)
    # TOCTOU guard: no modified model/protocol bytes can sneak through mid-evaluation.
    after, after_files = verify_evaluation_freeze(root, manifest_path)
    if after != freeze or after_files != frozen_files:
        raise EvaluationError("frozen inputs changed during evaluation")
    complete = (steady["strict_count"] >= 8 and len(oscillations) >= 1 and not pending)
    return {"schema_version": 1, "status": "scored_complete_scope" if complete else "inconclusive" if records else "pending",
            "complete_acceptance_eligible": complete, "complete_scope_is_target_acceptance": False,
            "full_nine_channel_validation_claim": False, "default_theta_replacement_authorized": False,
            "new_steady_strict_count": steady["strict_count"], "required_steady_strict_count": 8,
            "execution_complete_is_steady_convergence": False, "frozen_manifest": freeze,
            "frozen_file_hashes": frozen_files, "pending_cases": pending,
            "steady": steady, "oscillations": oscillations, "collected_records": records,
            "metric_convention": "predicted minus CFD; physical SI per channel; equal whole-case weights; no force/moment unit mixing",
            "parameters_changed_or_refitted": False, "pure_hull_legacy_prediction_available": False,
            "blind_loads_opened": bool(records), "scope": "Only fixed-pose translations and the one q0 oblique laminar base-motion experiment; no independent joint-motion validation."}


def prediction27(prediction):
    return np.r_[prediction["total"], prediction["tau_hydro"], np.asarray(prediction["links"]).ravel()]


def legacy_fixture(root=HERE):
    root = Path(root)
    theta = read_json(root / "frozen/theta87_baseline.json")["theta"]
    ref = dp.reference_from_contract(read_json(root / "contract.json"))
    prior = read_json(root / "frozen/oscillation_prior_predictions.json")
    nu, nudot = np.asarray(prior["nu"]), np.asarray(prior["nudot"])
    planned = [frozen87_dynamic(theta, [0., 0., 0.], v, a, ref) for v, a in zip(nu, nudot)]
    rng = np.random.default_rng(140087)
    random_nu, random_a = np.zeros((48, 6)), np.zeros((48, 6))
    random_nu[:, :3], random_a[:, :3] = rng.uniform(-.8, .8, (48, 3)), rng.uniform(-.5, .5, (48, 3))
    all_nu, all_a = np.vstack((nu, random_nu)), np.vstack((nudot, random_a))
    predictions = planned + [frozen87_dynamic(theta, [0., 0., 0.], v, a, ref) for v, a in zip(random_nu, random_a)]
    fixture = {"role": "planned/random algebra cross-check; no blind CFD inputs", "planned_count": 240, "random_count": 48,
               "q": [0., 0., 0.], "nu": all_nu, "nudot": all_a, "expected27": [prediction27(p) for p in predictions]}
    return fixture, planned, prior


def prepare_legacy_prior(matlab_executable, root=HERE):
    root = Path(root).resolve()
    if root != HERE:
        raise EvaluationError("prior export is restricted to the real approved stage root")
    destination = root / "frozen/legacy87_oscillation_prior.json"
    if destination.exists():
        raise EvaluationError("legacy prior already exists; immutable file will not be overwritten")
    source = root.parent / "11_theta_fit/matlab"
    sources = [root / "frozen/theta87_baseline.json", root / "model_components.py", root / "contract.json",
               root / "frozen/oscillation_prior_predictions.json", root / "evaluate_blind.py",
               source / "Y_uvms.m", source / "uvms_link_kin.m", source / "uvms_geometry.m"]
    before = {p: runner.sha256(p) for p in sources}
    research = {r["path"]: r["sha256"] for r in read_json(root / "frozen/research_model_manifest.json")["files"]}
    originals = {r["path"]: r["sha256"] for r in read_json(root / "frozen/source_manifest.json")["files"]}
    for path in sources:
        if path.name == "evaluate_blind.py":
            continue
        expected = research.get(path.relative_to(root).as_posix()) if path.is_relative_to(root) else originals.get(path.relative_to(root.parent.parent).as_posix())
        if expected is None or before[path] != expected:
            raise EvaluationError("legacy prior source is not the already-frozen input: " + str(path))
    fixture, predictions, dynamic_prior = legacy_fixture(root)
    fixture_path = root / "reports/evaluator_legacy87_matlab_fixture.json"
    report_path = root / "reports/evaluator_legacy87_matlab_check.json"
    write_report(fixture_path, fixture)
    source = root.parent / "11_theta_fit/matlab"
    escaped = lambda path: str(path).replace("\\", "/").replace("'", "''")
    command = (
        "addpath('" + escaped(source) + "');"
        "d=jsondecode(fileread('" + escaped(fixture_path) + "'));"
        "s=jsondecode(fileread('" + escaped(root / "frozen/theta87_baseline.json") + "'));th=s.theta(:);"
        "g=uvms_geometry();n=size(d.nu,1);out=zeros(n,27);"
        "for k=1:n,[Y,Yl]=Y_uvms(zeros(3,1),[d.nu(k,:)';zeros(3,1)],[d.nudot(k,:)';zeros(3,1)],g);"
        "out(k,:)=[(-Y*th)' (-Yl*th)'];end;"
        "e=abs(out-d.expected27);r=struct('status','PASS','planned_count',240,'random_count',n-240,"
        "'max_abs',max(e,[],'all'),'planned_max_abs',max(e(1:240,:),[],'all'),"
        "'random_max_abs',max(e(241:end,:),[],'all'),'tolerance',1e-10,'matlab_version',version);"
        "if r.max_abs>=1e-10,r.status='FAIL';end;"
        "f=fopen('" + escaped(report_path) + "','w');fprintf(f,'%s',jsonencode(r));fclose(f);"
        "disp(jsonencode(r));assert(r.max_abs<1e-10,'legacy87 prediction mismatch');"
    )
    completed = subprocess.run([str(matlab_executable), "-batch", command], cwd=root.parent.parent,
                               capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=240)
    if completed.returncode or not report_path.is_file():
        raise EvaluationError("MATLAB algebra cross-check failed: " + (completed.stdout + completed.stderr)[-2500:])
    report = read_json(report_path)
    if report.get("status") != "PASS" or report.get("max_abs", 1.) >= 1e-10:
        raise EvaluationError("MATLAB cross-check did not meet <1e-10")
    if {p: runner.sha256(p) for p in sources} != before:
        raise EvaluationError("frozen legacy inputs changed during MATLAB algebra cross-check")
    waveform = {"schema_version": 1, "id": "O_V_q0_YZ", "role": "a_priori_legacy87_planned_waveform_no_blind_truth",
                "parameters_refitted": False, "pure_hull_available": False, "q": [0., 0., 0.],
                "time_s": dynamic_prior["time_s"], "nu": dynamic_prior["nu"], "nudot": dynamic_prior["nudot"],
                "predictions": predictions, "matlab_cross_check": report,
                "source_hashes": [{"path": str(p.relative_to(root)) if p.is_relative_to(root) else "../11_theta_fit/matlab/" + p.name,
                                    "sha256": runner.sha256(p)} for p in sources],
                "fixture_sha256": runner.sha256(fixture_path), "matlab_report_sha256": runner.sha256(report_path),
                "physical_sign": "negative Y_uvms*theta; original diagonal added mass plus added-Coriolis and drag",
                "scope": "q0 locked-arm translation only. Combined hull+base0 is not a pure hull prediction. Actual-input replay later may change nu/nudot only, never parameters."}
    with destination.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(dp.serializable(waveform), indent=2, allow_nan=False) + "\n")
    return {"status": "PASS", "path": str(destination), "sha256": runner.sha256(destination), "matlab_cross_check": report,
            "blind_loads_opened": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--score", action="store_true", help="explicitly read validated new CFD artifacts; parent execution only")
    parser.add_argument("--freeze-manifest", type=Path)
    parser.add_argument("--output", type=Path, help="optional reports/evaluator_*.json result")
    parser.add_argument("--prepare-legacy-prior", action="store_true", help="create the new immutable prior after MATLAB algebra cross-check")
    parser.add_argument("--matlab", type=Path, default=Path("D:/Program Files/MATLAB/R2025b/bin/matlab.exe"))
    args = parser.parse_args(argv)
    if args.score and args.prepare_legacy_prior:
        parser.error("prior preparation and truth scoring cannot run in the same invocation")
    try:
        result = prepare_legacy_prior(args.matlab) if args.prepare_legacy_prior else evaluate(manifest_path=args.freeze_manifest, score=args.score)
        if args.output:
            write_report(args.output, result)
        print(json.dumps(dp.serializable({k: result[k] for k in ("status", "reason", "path", "matlab_cross_check", "blind_loads_opened", "new_steady_strict_count", "complete_acceptance_eligible") if k in result}), allow_nan=False))
        return 0
    except (EvaluationError, runner.ProtocolError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "rejected", "error": str(exc)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

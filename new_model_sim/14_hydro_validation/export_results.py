"""Export already-scored independent validation, without fitting or changing frozen inputs.

Writes only reports/validation_* and handoff/. MATLAB builds the MAT file; a
separate temporary copy proves that the MATLAB handoff is not tied to this tree.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import numpy as np
import evaluate_blind as ev

REPORTS = HERE / "reports"
HANDOFF = HERE / "handoff"
GROUPS = ("hull_force3", "hull_moment3", "total6", "joint3")
GROUP_CN = {"hull_force3": "纯 AUV 本体力（N）", "hull_moment3": "纯 AUV 本体关于 Ob 的力矩（N·m）",
            "total6": "整机六维载荷（力 N；矩 N·m）", "joint3": "关节流体扭矩（N·m）"}
WINDOW_CN = {"strict_only": "仅 6 条 strict", "strict_plus_capped_sensitivity": "8 条：含 2 条尾均值敏感性",
             "full": "全程 240 点", "startup_first_cycle": "启动首周期 80 点", "ramp": "速度渐入 20 点",
             "last_two_cycles": "末两周期 160 点", "cycle2": "第二周期 80 点", "cycle3": "第三周期 80 点"}
STATIC_FILES = {"A": "candidate_A_strict37.json", "B": "candidate_B_strict37.json", "B_pair": "candidate_B_pair_strict37.json"}


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def ensure_owned(path):
    path = Path(path).resolve()
    if not (path.is_relative_to(HANDOFF) or (path.parent == REPORTS and path.name.startswith("validation_"))):
        raise ValueError("write outside export-owned handoff/reports paths refused")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path, data):
    ensure_owned(path).write_text(json.dumps(ev.dp.serializable(data), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def full_number(value):
    if isinstance(value, (bool, np.bool_)):
        return "true" if value else "false"
    if value is None:
        return ""
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(float(value)):
            raise ValueError("nonfinite export value")
        return format(float(value), ".17g")
    return str(value)


def write_csv(path, rows):
    if not rows:
        raise ValueError("empty CSV is not a validation export")
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with ensure_owned(path).open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: full_number(v) for k, v in row.items()})


def relative_stage(path):
    path = Path(path).resolve()
    if path.is_relative_to(HERE):
        return path.relative_to(HERE).as_posix()
    if path.is_relative_to(HERE.parent):
        return "../" + path.relative_to(HERE.parent).as_posix()
    raise ValueError("unexpected external source path")


def source_guard(score):
    """Verify all frozen source bytes and all nine indexed final artifact pairs."""
    expected = {}
    for record in load_json(HERE / "frozen/source_manifest.json")["files"]:
        expected[(HERE.parent.parent / record["path"]).resolve()] = record["sha256"]
    for record in load_json(HERE / "frozen/blind_protocol.json")["files"]:
        expected[(HERE / record["path"]).resolve()] = record["sha256"]
    for record in score["collected_records"]:
        attempt = Path(record["source"]["attempt"])
        if not attempt.resolve().is_relative_to(HERE / "cfd/runs"):
            raise ValueError("nonstage prospective artifact path")
        for artifact in record["source"]["artifacts"].values():
            expected[(attempt / artifact["path"]).resolve()] = artifact["sha256"]
        expected[(attempt / "result.json").resolve()] = record["source"]["result_sha256"]
    for path in (HERE / "reports/evaluator_blind_scores.json", HERE / "cfd/final_pair_readback.json",
                 HERE / "frozen/blind_protocol.json", HERE / "frozen/research_model_manifest.json",
                 HERE / "frozen/source_manifest.json", HERE / "reports/model_provenance_note.json"):
        expected[path] = sha256(path)
    mismatch = [relative_stage(path) for path, digest in expected.items() if sha256(path) != digest]
    if mismatch:
        raise ValueError("source/frozen artifact hash mismatch: " + ", ".join(mismatch))
    return expected


def verify_unchanged(expected):
    mismatch = [relative_stage(path) for path, digest in expected.items() if sha256(path) != digest]
    if mismatch:
        raise ValueError("read-only export changed source bytes: " + ", ".join(mismatch))
    return {"all_unchanged": True, "source_files_verified": len(expected),
            "files": [{"stage_relative_path": relative_stage(path), "sha256": digest, "included_in_handoff": False}
                      for path, digest in sorted(expected.items(), key=lambda item: relative_stage(item[0]))]}


def pack_truth(record):
    w = np.asarray(record["wrenches"])
    return {"hull": w[:, 0], "base0": w[:, 1], "links": w[:, 2:], "total": np.asarray(record["total"]),
            "joint_tau": np.asarray(record["joint_tau"]), "generalized": np.column_stack((record["total"], record["joint_tau"]))}


def pack_predictions(rows):
    out = {"total": np.array([p["total"] for p in rows]), "joint_tau": np.array([p["tau_hydro"] for p in rows]),
           "generalized": np.array([p["generalized"] for p in rows]), "links": np.array([p["links"] for p in rows])}
    if "hull" in rows[0]:
        out["hull"] = np.array([p["hull"] for p in rows])
        out["base0"] = np.array([p["base0"] for p in rows])
    else:
        out["hull_plus_base0"] = np.array([p["hull_plus_base0"] for p in rows])
    out["scope_flags_unchanged"] = [p.get("scope_flags", {}) for p in rows]
    return out


def readback_summary(readback):
    return {"status": readback["status"], "scope": readback["scope"], "all_nine_restarted": False,
            "initialization_called": readback["initialization_called"], "iterations_called": readback["iterations_called"],
            "all_inputs_unchanged_at_end": readback["all_inputs_unchanged_at_end"],
            "cases": [{"case_id": c["case_id"], "status": c["status"],
                       "channels_compared": c["load_comparison"]["channels_compared"],
                       "passing_channels": c["load_comparison"]["passing_channels"],
                       "maximum_absolute_error": c["load_comparison"]["maximum_absolute_error"],
                       "final_case_sha256": c["input_hashes_before"]["final.cas.h5"]["sha256"],
                       "final_data_sha256": c["input_hashes_before"]["final.dat.h5"]["sha256"]}
                      for c in readback["cases"]], "source_sha256": sha256(HERE / "cfd/final_pair_readback.json")}


def model_inventory():
    rows = []
    for name, filename in STATIC_FILES.items():
        model = load_json(HERE / "models" / filename)
        rows.append({"id": name, "file": "models/" + filename, "schema_version": model["schema_version"],
                     "parameter_count": len(model["theta"]), "parameter_metadata_count": len(model["parameters"]),
                     "parameter_interpretation": "静态虚拟作用点/阻力及耦合系数，不是电机驱动矩",
                     "freeze_status_original": model.get("freeze_status"), "scope_original": model["scope"],
                     "sha256": sha256(HERE / "models" / filename)})
    dynamic = load_json(HERE / "models/dynamic_linear_q0.json")
    rows.append({"id": "dynamic_q0", "file": "models/dynamic_linear_q0.json", "schema_version": dynamic["schema_version"],
                 "parameter_count": len(dynamic["parameter_metadata"]), "M_rows_shape": list(np.shape(dynamic["M_rows"])),
                 "D_rows_shape": list(np.shape(dynamic["D_rows"])), "M_total_shape": list(np.shape(dynamic["M_total"])),
                 "parameter_interpretation": "M_rows和D_rows各30×6，共360响应系数；不是原87参数，也不是完整9×9质量矩阵",
                 "scope_original": dynamic["scope"], "sha256": sha256(HERE / "models/dynamic_linear_q0.json")})
    legacy = load_json(HERE / "frozen/theta87_baseline.json")
    rows.append({"id": "legacy87", "file": "models/theta87_baseline.json", "schema_version": "original_theta_json",
                 "parameter_count": len(legacy["theta"]), "parameter_interpretation": "原部署对照；组合本体块包含base0，不提供独立纯hull预测",
                 "default_replaced": False, "sha256": sha256(HERE / "frozen/theta87_baseline.json")})
    return rows


def build_bundle(score):
    steady = [r for r in score["collected_records"] if r["mode"] == "steady_translation"]
    oscillations = [r for r in score["collected_records"] if r["mode"] == "oscillation_translation"]
    if len(steady) != 8 or len(oscillations) != 1 or score["new_steady_strict_count"] != 6:
        raise ValueError("expected the reviewed eight-steady/one-oscillation result with six strict rows")
    capped = [r["id"] for r in steady if r["capped"]]
    if sorted(capped) != ["T_V_q0_YZ_p055", "T_V_qe_YZ_p055"]:
        raise ValueError("capped direction coverage changed from reviewed disposition")
    reference = ev.dp.reference_from_contract(load_json(HERE / "contract.json"))
    static_prior = {r["id"]: r for r in load_json(HERE / "frozen/steady_prior_predictions.json")["cases"]}
    st = {"case_ids": [r["id"] for r in steady], "q": [r["q"] for r in steady], "nu": [r["nu"][0] for r in steady],
          "strict": [r["strict"] for r in steady], "capped": [r["capped"] for r in steady],
          "iterations": [r["iterations"] for r in steady], "averaging": [r["averaging"] for r in steady],
          "truth": {k: np.concatenate([pack_truth(r)[k] for r in steady], axis=0) for k in pack_truth(steady[0])},
          "predictions": {m: pack_predictions([static_prior[r["id"]]["models"][m] for r in steady]) for m in ("A", "B", "B_pair", "legacy87")},
          "uncertainty_wrenches_sd": [r["uncertainty_wrenches_sd"] for r in steady],
          "uncertainty_total_sd": [r["uncertainty_total_sd"] for r in steady],
          "uncertainty_tau_sd": [r["uncertainty_tau_sd"] for r in steady], "uncertainty_note": ev.dp.SD_NOTE}
    record = oscillations[0]
    model = load_json(HERE / "models/dynamic_linear_q0.json")
    theta = load_json(HERE / "frozen/theta87_baseline.json")["theta"]
    predictions, replay = ev.predict_actual_dynamics(record, model, theta, reference,
        load_json(HERE / "frozen/oscillation_prior_predictions.json"), load_json(HERE / "frozen/legacy87_oscillation_prior.json"))
    recomputed = ev.score_oscillation(record, predictions)
    max_metric_delta = 0.
    for name, windows in recomputed["metrics"].items():
        for window, groups in windows.items():
            for group, metric in groups.items():
                for key in ("RMSE", "MAE", "signed_bias", "maximum_absolute_error"):
                    old = score["oscillations"][0]["metrics"][name][window][group][key]
                    delta = float(np.max(np.abs(np.asarray(old) - metric[key])))
                    max_metric_delta = max(max_metric_delta, delta)
    if max_metric_delta >= 1e-10:
        raise ValueError("exported dynamic predictions do not reproduce the parent's frozen-model scores")
    t = np.array(record["time_s"])
    os = {"case_id": record["id"], "q": record["q"], "time_s": t, "nu": record["nu"], "nudot": record["nudot"],
          "moving_origin_m": record["moving_origin_m"], "reference_source": record["reference_source"],
          "actual_motion_law": record["actual_motion_law"], "period_s": record["period_s"], "ramp_s": record["ramp_s"],
          "viscous_model": record["viscous_model"], "truth": pack_truth(record),
          "predictions": {name: pack_predictions(values) for name, values in predictions.items()},
          "masks": {"full": np.ones(240, bool), "startup_first_cycle": t <= 2 + 1e-10,
                    "ramp": t <= .5 + 1e-10, "last_two_cycles": t > 2 + 1e-10,
                    "cycle2": (t > 2 + 1e-10) & (t <= 4 + 1e-10), "cycle3": t > 4 + 1e-10},
          "nominal_replay_max_abs": replay, "export_vs_score_metric_max_abs": max_metric_delta}
    # No host paths or process/license details enter the portable case index.
    readback = readback_summary(load_json(HERE / "cfd/final_pair_readback.json"))
    checked = {r["case_id"] for r in readback["cases"]}
    index = []
    for row in score["collected_records"]:
        attempt = Path(row["source"]["attempt"])
        index.append({"id": row["id"], "mode": row["mode"], "q": row["q"], "strict": row.get("strict"),
                      "capped": row.get("capped"), "iterations": row.get("iterations"),
                      "physical_samples": len(row.get("time_s", [])) if row["mode"] == "oscillation_translation" else 1,
                      "execution_complete": row["execution_complete"], "readback_pair_verified": row["id"] in checked,
                      "terminal_state_id": row["source"]["terminal_state"]["id"],
                      "artifacts": {key: {"stage_relative_path": relative_stage(attempt / item["path"]),
                                          "sha256": item["sha256"], "included": False}
                                    for key, item in row["source"]["artifacts"].items()},
                      "result": {"stage_relative_path": relative_stage(attempt / "result.json"),
                                 "sha256": row["source"]["result_sha256"], "included": False}})
    summary = {"overall_status": score["status"], "strict_steady_count": 6, "capped_steady_count": 2,
               "capped_ids": capped, "independent_oscillation_count": 1, "default_theta_replaced": False,
               "static_conclusion": "已有明确退化且无全面优胜，不支持替换默认87；strict样本还缺失两条正YZ，代表性受限",
               "dynamic_conclusion": "独立改善仅限q0锁臂、0.02m、2s、YZ斜向平移、laminar；不能外推任意运动或完整9×9模型",
               "startup_cause": "启动差异可能包含初始条件、离散和流场历史影响，尚未辨别原因",
               "pure_hull_legacy_prediction_available": False, "full_nine_channel_validation_claim": False,
               "readback_all_nine_pairs_verified": False}
    return {"schema_version": "stage14_validation_handoff_v1", "summary": summary, "reference": reference,
            "steady": st, "oscillation": os, "case_index": index, "readback": readback,
            "metrics": {"steady": score["steady"], "oscillations": score["oscillations"]},
            "model_inventory": model_inventory(), "numeric_policy": "JSON/CSV retain round-trip double precision; MAT is built by MATLAB",
            "force_sign": "physical fluid loads ON bodies; new predictors return this sign directly; only old Y*theta must be negated",
            "not_motor_command": True, "validation_coverage_is_separate_from_original_model_scope_flags": True}


def fields_for_loads(prefix, values, sample):
    out = {}
    for field, array in values.items():
        if field in ("scope_flags_unchanged", "generalized"):
            continue
        arr = np.asarray(array)[sample]
        if field == "links":
            for link in range(3):
                for j, channel in enumerate(("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
                    out[f"{prefix}_link{link+1}_{channel}_{'N' if j < 3 else 'Nm'}"] = arr[link, j]
        elif field == "joint_tau":
            for j in range(3):
                out[f"{prefix}_J{j+1}_Nm"] = arr[j]
        else:
            for j, channel in enumerate(("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
                out[f"{prefix}_{field}_{channel}_{'N' if j < 3 else 'Nm'}"] = arr[j]
    return out


def table_rows(bundle):
    steady, oscillation = bundle["steady"], bundle["oscillation"]
    steady_rows = []
    for i, cid in enumerate(steady["case_ids"]):
        row = {"case_id": cid, "strict": steady["strict"][i], "capped_tail_mean": steady["capped"][i],
               "iterations": steady["iterations"][i], "tail_saved_samples": steady["averaging"][i]["sample_count"]}
        row.update({f"q{k+1}_rad": steady["q"][i][k] for k in range(3)})
        row.update({f"nu{k+1}_{'m_s' if k<3 else 'rad_s'}": steady["nu"][i][k] for k in range(6)})
        row.update(fields_for_loads("CFD", steady["truth"], i))
        for name, predictions in steady["predictions"].items():
            row.update(fields_for_loads(name, predictions, i))
        steady_rows.append(row)
    dynamic_rows = []
    for i, t in enumerate(oscillation["time_s"]):
        row = {"case_id": oscillation["case_id"], "time_s": t,
               "startup_first_cycle": oscillation["masks"]["startup_first_cycle"][i],
               "last_two_cycles": oscillation["masks"]["last_two_cycles"][i]}
        row.update({f"nu{k+1}_{'m_s' if k<3 else 'rad_s'}": oscillation["nu"][i][k] for k in range(6)})
        row.update({f"nudot{k+1}_{'m_s2' if k<3 else 'rad_s2'}": oscillation["nudot"][i][k] for k in range(6)})
        row.update({"moving_Ob_" + axis + "_m": oscillation["moving_origin_m"][i][k] for k, axis in enumerate("xyz")})
        row.update(fields_for_loads("CFD", oscillation["truth"], i))
        for name, predictions in oscillation["predictions"].items():
            row.update(fields_for_loads(name, predictions, i))
        dynamic_rows.append(row)
    metrics = []
    for population in ("strict_only", "strict_plus_capped_sensitivity"):
        for model, groups in bundle["metrics"]["steady"][population]["models"].items():
            metrics.extend(flatten_metrics("steady", population, model, groups))
    dynamic_metric = bundle["metrics"]["oscillations"][0]
    for model, windows in dynamic_metric["metrics"].items():
        for window, groups in windows.items():
            metrics.extend(flatten_metrics("oscillation", window, model, groups))
    harmonics = []
    for model, windows in dynamic_metric["harmonics"].items():
        for window, groups in windows.items():
            for group, record in groups.items():
                for i, channel in enumerate(record["channels"]):
                    row = {"case_id": oscillation["case_id"], "model": model, "window": window, "group": group,
                           "channel": channel, "unit": "N" if channel.startswith("F") else "N*m",
                           "actual_angular_frequency_rad_s": dynamic_metric["harmonic_angular_frequency_rad_s"]}
                    for side in ("actual", "predicted", "predicted_minus_actual"):
                        for component in ("dc", "first_sin", "first_cos", "first_amplitude", "second_sin", "second_cos", "second_amplitude"):
                            row[side + "_" + component] = record[side][component][i]
                    harmonics.append(row)
    case_rows = []
    for record in bundle["case_index"]:
        row = {k: record[k] for k in ("id", "mode", "strict", "capped", "iterations", "physical_samples", "readback_pair_verified")}
        for key, value in {**record["artifacts"], "result": record["result"]}.items():
            row[key + "_stage_relative_path"] = value["stage_relative_path"]
            row[key + "_sha256"] = value["sha256"]
        case_rows.append(row)
    repeatability = []
    for group, values in dynamic_metric["cycle2_vs_cycle3_repeatability"].items():
        for i, channel in enumerate(ev.GROUP_CHANNELS[group]):
            repeatability.append({"case_id": oscillation["case_id"], "group": group, "channel": channel,
                                  "unit": "N" if channel.startswith("F") else "N*m",
                                  "phase_paired_rms_difference": values["phase_paired_rms_difference"][i],
                                  "phase_paired_max_abs_difference": values["phase_paired_max_abs_difference"][i]})
    return {"validation_steady.csv": steady_rows, "validation_oscillation.csv": dynamic_rows,
            "validation_metrics.csv": metrics, "validation_harmonics.csv": harmonics,
            "validation_case_index.csv": case_rows, "validation_cycle_repeatability.csv": repeatability}


def flatten_metrics(domain, window, model, groups):
    rows = []
    for group, metric in groups.items():
        for i, channel in enumerate(metric["channels"]):
            row = {"domain": domain, "window": window, "model": model, "group": group, "channel": channel,
                   "unit": metric["units"][i], "case_count": metric["case_count"], "sample_count": metric["sample_count"]}
            for key in ("RMSE", "MAE", "signed_bias", "maximum_absolute_error"):
                row[key] = metric[key][i]
            rows.append(row)
    return rows


def svg_timeseries(bundle, hull_only=False):
    o = bundle["oscillation"]
    t = np.asarray(o["time_s"])
    if hull_only:
        actual = np.asarray(o["truth"]["hull"])
        curves = [("CFD", actual, "#17232d"), ("dynamic_q0", np.asarray(o["predictions"]["dynamic_linear"]["hull"]), "#176bc1")]
        channels = ["hull " + c for c in ("Fx", "Fy", "Fz", "Mx", "My", "Mz")]
        units = ["N"] * 3 + ["N*m"] * 3
    else:
        actual = np.column_stack((o["truth"]["total"], o["truth"]["joint_tau"]))
        curves = [("CFD", actual, "#17232d")]
        for name, color in (("dynamic_linear", "#176bc1"), ("legacy87", "#cf6b1b")):
            curves.append((name, np.column_stack((o["predictions"][name]["total"], o["predictions"][name]["joint_tau"])), color))
        channels = ["total " + c for c in ("Fx", "Fy", "Fz", "Mx", "My", "Mz")] + ["J1", "J2", "J3"]
        units = ["N"] * 3 + ["N*m"] * 6
    rows = math.ceil(len(channels) / 3)
    width, height = 1180, 55 + rows * 235
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" aria-label="真实CFD与冻结预测时序">',
           '<style>text{font-family:Arial,sans-serif;font-size:12px} .axis{stroke:#a9b3bc;stroke-width:1} .curve{fill:none;stroke-width:1.5}</style>',
           '<rect width="100%" height="100%" fill="white"/>']
    for j, (name, _, color) in enumerate(curves):
        out.append(f'<line x1="{35+220*j}" x2="{60+220*j}" y1="20" y2="20" stroke="{color}" stroke-width="2"/><text x="{68+220*j}" y="24">{name}</text>')
    out.append('<text x="720" y="24">shaded: startup first cycle (0–2 s); no truth smoothing</text>')
    for k, channel in enumerate(channels):
        x0, y0 = 52 + (k % 3) * 390, 70 + (k // 3) * 235
        pw, ph = 315, 165
        vals = np.concatenate([y[:, k] for _, y, _ in curves])
        lo, hi = float(vals.min()), float(vals.max())
        pad = max((hi - lo) * .08, 1e-8)
        lo, hi = lo - pad, hi + pad
        out.append(f'<text x="{x0}" y="{y0-12}">{html.escape(channel)} [{units[k]}]</text>')
        out.append(f'<rect x="{x0}" y="{y0}" width="{pw/3}" height="{ph}" fill="#f3ead9"/>')
        for f in (0., .5, 1.):
            yy, value = y0 + ph * (1-f), lo + f * (hi-lo)
            out.append(f'<line class="axis" x1="{x0}" x2="{x0+pw}" y1="{yy}" y2="{yy}"/><text text-anchor="end" x="{x0-5}" y="{yy+4}">{value:.4g}</text>')
        for tt in (0, 2, 4, 6):
            xx = x0 + pw * tt / 6
            out.append(f'<text text-anchor="middle" x="{xx}" y="{y0+ph+20}">{tt}</text>')
        out.append(f'<text x="{x0+pw-20}" y="{y0+ph+37}">t [s]</text>')
        for name, y, color in curves:
            coords = " ".join(f"{x0+pw*tt/6:.5f},{y0+ph*(hi-v)/(hi-lo):.5f}" for tt, v in zip(t, y[:, k]))
            out.append(f'<polyline class="curve" stroke="{color}" points="{coords}"><title>{name}; full-precision samples in CSV/MAT</title></polyline>')
    out.append('</svg>')
    return "\n".join(out)


def metric_table(groups_by_model, group):
    rows = []
    for model, groups in groups_by_model.items():
        if group not in groups:
            rows.append(f'<tr><td>{html.escape(model)}</td><td colspan="6">无独立纯 hull 预测；不以真实 base0 相减构造。</td></tr>')
            continue
        metric = groups[group]
        for i, channel in enumerate(metric["channels"]):
            values = ''.join('<td>' + full_number(metric[key][i]) + '</td>' for key in ("RMSE", "MAE", "signed_bias", "maximum_absolute_error"))
            rows.append(f'<tr><td>{model}</td><td>{channel}</td><td>{metric["units"][i]}</td>{values}</tr>')
    return '<div class="scroll"><table><thead><tr><th>模型</th><th>通道</th><th>单位</th><th>RMSE</th><th>MAE</th><th>bias（预测−CFD）</th><th>max |error|</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table></div>'


def render_html(bundle, csv_rows, asset_prefix=""):
    metrics = bundle["metrics"]
    strict = metrics["steady"]["strict_only"]["models"]
    dynamic = metrics["oscillations"][0]
    startup = dynamic["metrics"]["dynamic_linear"]["startup_first_cycle"]["total6"]
    fy = {name: group["total6"]["RMSE"][1] for name, group in strict.items()}
    top = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>阶段14：独立水动力验证交付</title>
<style>body{{font:16px/1.6 system-ui,"Microsoft YaHei",sans-serif;color:#182638;background:#f3f5f8;margin:0}}main{{max-width:1240px;margin:auto;padding:24px}}h1{{font-size:28px;margin:0 0 12px}}h2{{margin-top:34px}}.banner{{background:#fff3d6;border-left:7px solid #a66a00;padding:18px 22px}}.bad{{color:#9a2f26;font-weight:700}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px;margin-top:16px}}.card,details{{background:white;border:1px solid #d8e1e8;border-radius:6px;padding:14px;margin:12px 0}}.card b{{font-size:23px}}.scroll{{overflow-x:auto}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{border-bottom:1px solid #e0e5ea;padding:7px 10px;text-align:right;white-space:nowrap}}th:first-child,td:first-child{{text-align:left}}th{{background:#eaf0f6}}small,.muted{{color:#556579}}summary{{cursor:pointer;font-weight:650}}code{{background:#e9edf1;padding:2px 4px}}svg{{width:100%;height:auto}}a{{color:#155ea5}}li{{margin:4px 0}}</style><main>
<h1>阶段14 · 独立水动力验证</h1><section class="banner"><strong>总体结论：INCONCLUSIVE；保留默认旧87，不替换。</strong>
<ul><li><b>6 strict / 2 capped / 1 独立振荡</b>。两条 capped 都是正向 YZ：<code>T_V_q0_YZ_p055</code>、<code>T_V_qe_YZ_p055</code>，并非随机缺失；strict6 的方向代表性受限。</li>
<li class="bad">稳态已有明确退化，不只是“少两条收敛”：strict6 整机 Fy RMSE，旧87={fy['legacy87']:.2f} N；A/B/B_pair={fy['A']:.2f}/{fy['B']:.2f}/{fy['B_pair']:.2f} N。无全面优胜，不支持替换。</li>
<li class="bad">动态启动前80点：Fy RMSE={startup['RMSE'][1]:.2f} N；Fy 最大绝对误差={startup['maximum_absolute_error'][1]:.2f} N，Fz 最大绝对误差={startup['maximum_absolute_error'][2]:.2f} N。末160点改善不能掩盖启动误差。</li>
<li>动态独立改善仅限 <b>q0锁臂 / 0.02 m / 2 s / YZ斜向平移 / laminar</b>。不是任意运动、独立关节运动或完整9×9质量模型的证明。纯hull旧87没有独立基线，不能声称“纯本体六轴+三关节均相对旧模型改善”。</li>
<li>启动差异可能包含初始条件、离散和流场历史影响；原因尚未辨别，不归因于单一机理。</li></ul></section>
<div class="grid"><div class="card"><b>物理SI载荷</b><br>力N，矩/关节扭矩N·m；fluid ON bodies，不是电机驱动矩。</div><div class="card"><b>Ob=[0, 0.29386, −0.06630] m</b><br>CAD轴，+Y向上；本体/整机矩关于Ob，振荡关于history记录的移动Ob。</div><div class="card"><b>2对代表性读回</b><br>仅验证qe稳态与q0振荡两对final case/data；不得扩写为全9对均重启验证。</div></div>
<h2>数据与用法</h2><p>下列CSV/JSON/MAT保留双精度往返数值。严格集与含尾均值敏感性分开；尾窗SD不是独立测量不确定度。新MATLAB预测器直接返回流体ON负载，不再取负；旧<code>Y*theta</code>才需取负。</p><ul>'''
    for name in csv_rows:
        top += f'<li><a href="{asset_prefix}{name}">{name}</a></li>'
    top += f'<li><a href="{asset_prefix}validation_bundle.json">完整结构化数据 JSON</a>；<a href="{asset_prefix}validation_bundle.mat">MATLAB构建的MAT</a></li></ul>'
    top += '<p>便携包内：<code>addpath(fullfile(packageRoot,\'matlab\')); read_validation_example(packageRoot);</code>。示例显示纯AUV6维、整机6维、关节3扭矩，并复核新预测器和旧87全部保存预测。模型冻结状态原文保留，测试覆盖只从本报告读取。</p>'
    top += '<h2>稳态：旧87与所有研究候选并列</h2>'
    for population in ("strict_only", "strict_plus_capped_sensitivity"):
        top += f'<section class="card"><h3>{WINDOW_CN[population]}</h3>'
        groups = metrics["steady"][population]["models"]
        for group in GROUPS:
            top += f'<details><summary>{GROUP_CN[group]} · 四项逐通道指标</summary>{metric_table(groups,group)}</details>'
        top += '</section>'
    top += '<h2>动态：改善发生在哪个窗口</h2><p>下面同时列出全程、启动、渐入、末两周期及各周期。原模型在总载荷/关节扭矩上作为部署对照；纯hull只列CFD与新模型的绝对误差，不伪造旧hull。</p>'
    last = {m: d["last_two_cycles"] for m,d in dynamic["metrics"].items()}
    top += '<div class="card"><h3>末160点关键对照（RMSE）</h3><table><tr><th>通道</th><th>旧87</th><th>dynamic_q0</th><th>单位</th></tr>'
    for group, channel, idx in (("total6","Fx",0),("total6","Fy",1),("total6","Fz",2),("joint3","J1",0),("joint3","J2",1),("joint3","J3",2)):
        top += '<tr><td>'+channel+'</td><td>'+full_number(last["legacy87"][group]["RMSE"][idx])+'</td><td>'+full_number(last["dynamic_linear"][group]["RMSE"][idx])+'</td><td>'+('N' if group=="total6" else 'N*m')+'</td></tr>'
    top += '</table></div>'
    for window in ("full","startup_first_cycle","ramp","last_two_cycles","cycle2","cycle3"):
        top += f'<details><summary>{WINDOW_CN[window]} · RMSE / MAE / bias / max</summary>'
        bymodel = {m: values[window] for m, values in dynamic["metrics"].items()}
        for group in GROUPS:
            top += f'<h4>{GROUP_CN[group]}</h4>'+metric_table(bymodel,group)
        top += '</details>'
    top += '<h2>真实时序：整机6维＋关节3量</h2>'+svg_timeseries(bundle)
    top += '<h2>真实时序：纯AUV本体6维</h2><p>旧87无独立纯hull预测，因此没有旧模型曲线。</p>'+svg_timeseries(bundle,True)
    top += '<h2>DC / 一阶 / 二阶谐波与周期重复性</h2><p>预测和测量使用同一个实际HDF运动角频率 '+full_number(dynamic["harmonic_angular_frequency_rad_s"])+' rad/s。幅值为实际N/N·m，不是单位加速度伪样本；第二/三周期比较只表示重复性。</p>'
    for window in ("full","startup_first_cycle","last_two_cycles"):
        top += f'<details><summary>{WINDOW_CN[window]} · 谐波逐通道</summary><div class="scroll"><table><tr><th>模型/组/通道</th><th>单位</th><th>DC CFD</th><th>DC预测</th><th>一阶幅值CFD</th><th>一阶幅值预测</th><th>二阶幅值CFD</th><th>二阶幅值预测</th></tr>'
        for r in csv_rows["validation_harmonics.csv"]:
            if r["window"]!=window or r["group"] not in GROUPS: continue
            fields=("actual_dc","predicted_dc","actual_first_amplitude","predicted_first_amplitude","actual_second_amplitude","predicted_second_amplitude")
            top += '<tr><td>'+html.escape(r["model"]+'/'+r["group"]+'/'+r["channel"])+'</td><td>'+r["unit"]+'</td>'+''.join('<td>'+full_number(r[k])+'</td>' for k in fields)+'</tr>'
        top += '</table></div></details>'
    top += '<h2>冻结模型与参数含义</h2><table><tr><th>模型</th><th>schema</th><th>参数数</th><th>解释</th></tr>'
    for item in bundle["model_inventory"]:
        top += f'<tr><td>{item["id"]}</td><td>{item["schema_version"]}</td><td>{item["parameter_count"]}</td><td style="white-space:normal">{item["parameter_interpretation"]}</td></tr>'
    top += '</table><p>计数直接从文件读取。dynamic_q0的M_rows/D_rows是分部件对共同基座输入的响应系数；并非87参数替代品，也非完整9×9独立关节质量矩阵。保留原模型scope/freeze_status，不改为全域validated。</p>'
    top += '<h2>最终产物读回与来源</h2><ul>'
    for c in bundle["readback"]["cases"]:
        top += '<li>'+c["case_id"]+'：'+str(c["passing_channels"])+'/'+str(c["channels_compared"])+'通道通过，最大绝对差 '+full_number(c["maximum_absolute_error"])+'.</li>'
    top += '</ul><p>两对代表性只读验证未初始化、未迭代、未改变网格/边界/参考点；不声称全9对逐个重启验证。轻量包只提供case/data相对路径和SHA256索引，不包含大型case/dat/mesh，也不包含旧52短时数据或许可证/个人环境信息。</p>'
    top += '<p class="muted">静态A/B/B_pair是研究候选；B_pair是受限开发补充，不把开发历史说成盲选。新增独立试验不授权更改默认θ，亦不构成控制器跟踪或驱动矩验证。</p></main></html>'
    return top


def copy_checked(source, destination):
    destination = ensure_owned(destination)
    shutil.copyfile(source, destination)
    if sha256(source) != sha256(destination):
        raise ValueError("copy hash mismatch")


def package_files(bundle, expected_sources):
    HANDOFF.mkdir(exist_ok=True)
    for folder in ("data", "models", "matlab", "matlab/legacy", "provenance"):
        (HANDOFF / folder).mkdir(parents=True, exist_ok=True)
    for filename in STATIC_FILES.values():
        copy_checked(HERE / "models" / filename, HANDOFF / "models" / filename)
    copy_checked(HERE / "models/dynamic_linear_q0.json", HANDOFF / "models/dynamic_linear_q0.json")
    copy_checked(HERE / "frozen/theta87_baseline.json", HANDOFF / "models/theta87_baseline.json")
    for name in ("predict_split_hydro.m", "predict_dynamic_linear.m", "export_validation_mat.m", "read_validation_example.m"):
        copy_checked(HERE / "matlab" / name, HANDOFF / "matlab" / name)
    for name in ("Y_uvms.m", "uvms_geometry.m", "uvms_link_kin.m"):
        copy_checked(HERE.parent / "11_theta_fit/matlab" / name, HANDOFF / "matlab/legacy" / name)
    for name in ("blind_protocol.json", "research_model_manifest.json", "source_manifest.json", "evaluation_protocol.json",
                 "steady_prior_predictions.json", "oscillation_prior_predictions.json", "legacy87_oscillation_prior.json"):
        copy_checked(HERE / "frozen" / name, HANDOFF / "provenance" / name)
    # Keep geometry and the original fingerprint, not installation/environment paths.
    old_full_contract = HANDOFF / "provenance/contract.json"
    if old_full_contract.is_file():
        ensure_owned(old_full_contract).unlink()
    write_json(HANDOFF / "provenance/contract_reference.json", {
        "role": "derived geometry/reference only; original contract is not rewritten",
        "original_contract_sha256": sha256(HERE / "contract.json"), "reference": bundle["reference"]})
    copy_checked(HERE / "reports/model_provenance_note.json", HANDOFF / "provenance/model_provenance_note.json")
    write_json(HANDOFF / "models/model_inventory.json", bundle["model_inventory"])
    write_json(HANDOFF / "data/validation_case_index.json", bundle["case_index"])
    write_json(HANDOFF / "provenance/readback_coverage.json", bundle["readback"])
    write_json(HANDOFF / "provenance/export_source_fingerprints.json", {"path_base": "original stage14 directory",
        "original_sources_copied_only_when_explicitly_listed": True,
        "files": [{"stage_relative_path": relative_stage(path), "sha256": digest} for path,digest in sorted(expected_sources.items(),key=lambda item:relative_stage(item[0]))]})


def refresh_package_manifest():
    files = [p for p in HANDOFF.rglob("*") if p.is_file() and p.name != "package_manifest.json"]
    forbidden = [p for p in files if p.name.endswith((".cas.h5", ".dat.h5", ".msh", ".scdoc", ".zip", ".xlsx", ".md"))]
    if forbidden:
        raise ValueError("large/unrequested material found in handoff")
    rows = [{"path": p.relative_to(HANDOFF).as_posix(), "sha256": sha256(p), "size_bytes": p.stat().st_size} for p in sorted(files)]
    manifest = {"schema_version": 1, "role": "lightweight unzipped handoff, no automatic sharing", "overall_status": "inconclusive",
                "default_theta_replaced": False, "all_model_bytes_preserved": True, "files": rows,
                "file_count": len(rows), "total_size_bytes_excluding_manifest": sum(r["size_bytes"] for r in rows),
                "excluded": ["raw case/data/mesh", "old52 short traces", "license/individual environment information", "final ZIP"]}
    write_json(HANDOFF / "package_manifest.json", manifest)
    return manifest


def run_matlab(package_root, matlab_executable):
    package_root = Path(package_root)
    esc = lambda p: str(p).replace("\\", "/").replace("'", "''")
    command = "addpath('"+esc(package_root / "matlab")+"');export_validation_mat('"+esc(package_root)+"');r=read_validation_example('"+esc(package_root)+"');disp(['VALIDATION_CHECK_JSON=' jsonencode(r)]);assert(strcmp(r.status,'PASS'));"
    completed = subprocess.run([str(matlab_executable), "-batch", command], cwd=package_root,
                               capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=240)
    marker = "VALIDATION_CHECK_JSON="
    result_lines = [line.split(marker,1)[1] for line in completed.stdout.splitlines() if marker in line]
    if completed.returncode or not result_lines:
        raise RuntimeError("MATLAB export/example failed: " + (completed.stdout+completed.stderr)[-3000:])
    return json.loads(result_lines[-1])


def export_all(with_matlab=False, portability=False, matlab_executable=None):
    score = load_json(REPORTS / "evaluator_blind_scores.json")
    expected = source_guard(score)
    bundle = build_bundle(score)
    rows = table_rows(bundle)
    for name, table in rows.items():
        write_csv(REPORTS / name, table)
    write_json(REPORTS / "validation_bundle.json", bundle)
    write_json(REPORTS / "validation_case_index.json", bundle["case_index"])
    write_json(REPORTS / "validation_model_inventory.json", bundle["model_inventory"])
    write_json(REPORTS / "validation_readback_coverage.json", bundle["readback"])
    for hull, filename in ((False,"validation_dynamic_total_joint.svg"),(True,"validation_dynamic_hull.svg")):
        ensure_owned(REPORTS / filename).write_text(svg_timeseries(bundle,hull),encoding="utf-8")
    ensure_owned(REPORTS / "validation_report.html").write_text(render_html(bundle,rows),encoding="utf-8")
    package_files(bundle,expected)
    for name in rows:
        copy_checked(REPORTS / name, HANDOFF / "data" / name)
    for name in ("validation_bundle.json","validation_dynamic_total_joint.svg","validation_dynamic_hull.svg"):
        copy_checked(REPORTS / name,HANDOFF / "data" / name)
    ensure_owned(HANDOFF / "index.html").write_text(render_html(bundle,rows,"data/"),encoding="utf-8")
    matlab_result, portable_result = None,None
    if with_matlab:
        matlab_result = run_matlab(HANDOFF,matlab_executable)
        copy_checked(HANDOFF / "data/validation_bundle.mat", REPORTS / "validation_bundle.mat")
        write_json(REPORTS / "validation_matlab_checks.json",matlab_result)
        write_json(HANDOFF / "provenance/validation_matlab_checks.json",matlab_result)
    if portability:
        if not with_matlab:
            raise ValueError("portability requires the actual MATLAB test")
        # Explicitly requested temporary-copy test; no original data/source is modified.
        with tempfile.TemporaryDirectory(prefix="stage14_validation_portable_") as tmp:
            copied = Path(tmp) / "renamed_hydrodynamic_package"
            shutil.copytree(HANDOFF,copied)
            portable_result = run_matlab(copied,matlab_executable)
            portable_result["tested_after_temporary_copy"] = True
            portable_result["temporary_copy_removed_on_exit"] = True
        write_json(REPORTS / "validation_portability_checks.json",portable_result)
        write_json(HANDOFF / "provenance/validation_portability_checks.json",portable_result)
    integrity = verify_unchanged(expected)
    write_json(REPORTS / "validation_source_integrity.json",integrity)
    write_json(HANDOFF / "provenance/validation_source_integrity.json",integrity)
    manifest = refresh_package_manifest()
    summary = {"status":"PASS","validation_status":"inconclusive","handoff_directory":"handoff",
               "strict_steady":6,"capped_steady":2,"independent_oscillation":1,
               "csv_row_counts":{name:len(values) for name,values in rows.items()},
               "model_parameter_counts":{r["id"]:r["parameter_count"] for r in bundle["model_inventory"]},
               "source_hashes_unchanged":integrity["all_unchanged"],"source_files_verified":len(expected),
               "matlab":matlab_result,"portability":portable_result,"file_count":manifest["file_count"],
               "handoff_bytes_excluding_manifest":manifest["total_size_bytes_excluding_manifest"],
               "no_model_refit":True,"no_default_theta_change":True,"zip_created":False}
    write_json(REPORTS / "validation_export_summary.json",summary)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab",action="store_true",help="build MAT using MATLAB and verify all saved predictions")
    parser.add_argument("--portability",action="store_true",help="copy handoff to a temporary renamed directory and repeat MATLAB")
    parser.add_argument("--matlab-executable",type=Path,default=Path("D:/Program Files/MATLAB/R2025b/bin/matlab.exe"))
    args=parser.parse_args()
    result=export_all(args.matlab,args.portability,args.matlab_executable)
    print(json.dumps(result,ensure_ascii=False))


if __name__=="__main__":
    main()

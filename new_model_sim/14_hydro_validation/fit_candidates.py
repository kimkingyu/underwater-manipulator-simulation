"""Fit and retrospectively validate the three geometry-fixed static dictionaries.

Only data/component_dataset.json supplies CFD labels. This module never opens
cfd/, blind results, old raw CFD loads, or synthesizes substitute real data.
All normalizers and feature scales are training-fold only; ridge is selected
inside each outer training set. Capped cases are a separate sensitivity run.
"""
from __future__ import annotations

import argparse
import copy
import csv
import html
import json
import os
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np
from scipy.optimize import lsq_linear

from model_components import (BODY_ORDER, CHANNELS, body_jacobians, build_template, component_features,
    frozen87_steady, jsonable, predict_split_hydro, recompose, save_json, sha256_file)

ROOT = Path(__file__).resolve().parent
REGULARIZERS = (0., .001, .03, .3)
GROUP_NAMES = (*BODY_ORDER, "total", "joint")
FLOOR6 = np.array([1., 1., 1., .1, .1, .1])
PRIMARY_GROUPS = ("hull", "total", "joint")


def direction_family(record):
    v = np.asarray(record["nu"], float)[:3]
    speed = np.linalg.norm(v)
    if speed < 1e-12:
        return "zero"
    v = v/speed
    pivot = np.flatnonzero(np.abs(v) > 1e-10)[0]
    if v[pivot] < 0:
        v = -v
    # IEEE negative zero must not split the two signs of an axial direction.
    v[np.abs(v) < 5e-7] = 0.
    return "direction_" + "_".join(f"{x:.6f}" for x in v)


def split_groups(records, scheme):
    return [r["pose_id"] if scheme == "pose" else direction_family(r) for r in records]


def target_groups(records):
    w = np.asarray([r["wrenches"] for r in records], float)
    groups = {name: w[:, i] for i, name in enumerate(BODY_ORDER)}
    groups["total"] = np.asarray([r["total"] for r in records], float)
    groups["joint"] = np.asarray([r["joint_tau"] for r in records], float)
    return groups


def training_scales(targets, indices):
    return {g: np.maximum(np.sqrt(np.mean(y[indices]**2, axis=0)),
                          FLOOR6 if y.shape[1] == 6 else np.full(3, .1))
            for g, y in targets.items()}


def rank_diagnostic(A, names):
    _, singular, vh = np.linalg.svd(A, full_matrices=A.shape[0] < A.shape[1])
    tol = max(A.shape) * max(singular[0] if singular.size else 0., 1.) * 1e-10
    rank = int(np.sum(singular > tol))
    null = vh[rank:]
    unidentified = [names[j] for j in range(len(names)) if null.size and np.linalg.norm(null[:, j]) > 1e-7]
    return {"effective_rank": rank, "parameter_count": len(names), "nullity": len(names)-rank,
            "rank_tolerance": float(tol), "singular_values": singular.tolist(),
            "unidentifiable_parameters": unidentified, "full_rank": rank == len(names)}


def fit_once(template, features, records, train_indices, regularizer):
    model = copy.deepcopy(template)
    y = np.asarray([r["wrenches"] for r in records], float)
    theta = np.zeros(len(template["parameters"]))
    diagnostics = []
    for b, body_id in enumerate(BODY_ORDER):
        cols = [k for k, p in enumerate(template["parameters"]) if p["body_index"] == b]
        target_scale = np.maximum(np.sqrt(np.mean(y[train_indices, b]**2, axis=0)), FLOOR6)
        A = features[train_indices, b][:, :, cols] / target_scale[None, :, None]
        A = A.reshape(-1, len(cols))
        target = (y[train_indices, b] / target_scale).reshape(-1)
        feature_scale = np.sqrt(np.mean(A*A, axis=0))
        zero = feature_scale < 1e-14
        safe_scale = feature_scale.copy()
        safe_scale[zero] = 1.  # Numerical placeholder only; rank failure is NEVER silently scored.
        As = A/safe_scale
        diag = rank_diagnostic(As, [template["parameters"][k]["name"] for k in cols])
        diag.update({"body": body_id, "target_scale_train_only": target_scale.tolist(),
                     "feature_scale_train_only": feature_scale.tolist(),
                     "zero_feature_parameters": [template["parameters"][cols[j]]["name"] for j in np.flatnonzero(zero)]})
        lower = np.array([-np.inf if template["parameters"][k]["lower_bound"] is None else 0. for k in cols])
        if regularizer > 0:
            fit_A = np.vstack((As, np.sqrt(regularizer * len(target))*np.eye(len(cols))))
            fit_y = np.r_[target, np.zeros(len(cols))]
        else:
            fit_A, fit_y = As, target
        result = lsq_linear(fit_A, fit_y, bounds=(lower, np.inf), method="bvls", tol=1e-10, max_iter=2000)
        if not result.success or not np.isfinite(result.x).all():
            raise RuntimeError(f"constrained least-squares failed for {body_id}: {result.message}")
        theta[cols] = result.x/safe_scale
        diag.update({"solver_success": bool(result.success), "solver_status": int(result.status),
                     "cost": float(result.cost), "active_lower_bounds": int(np.sum(result.active_mask == -1))})
        diagnostics.append(diag)
    model["theta"] = theta.tolist()
    model["fit"] = {"regularizer": float(regularizer), "training_ids": [records[i]["id"] for i in train_indices],
                    "component_diagnostics": diagnostics, "full_rank": all(d["full_rank"] for d in diagnostics),
                    "objective": "independent split-body six-wrench weighted residuals plus ridge; no direct joint correction",
                    "normalization_floor_note": "1 N / 0.1 N*m is a normalization floor, NOT error precision"}
    model["scope"].update({"status": "development_data_only_not_blind",
        "observed_q": np.unique(np.asarray([records[i]["q"] for i in train_indices]), axis=0).tolist(),
        "max_training_speed_m_s": max(float(np.linalg.norm(records[i]["nu"][:3])) for i in train_indices)})
    return model


def select_regularizer(template, features, records, indices, scheme):
    labels = split_groups(records, scheme)
    groups = sorted({labels[i] for i in indices})
    scores = {reg: [] for reg in REGULARIZERS}
    fold_details = []
    if len(groups) < 2:
        return REGULARIZERS[-1], {"status": "no_inner_folds", "scores": {}, "folds": []}
    truth = np.asarray([r["wrenches"] for r in records])
    for group in groups:
        train = np.array([i for i in indices if labels[i] != group], int)
        test = np.array([i for i in indices if labels[i] == group], int)
        fold_info = {"held_group": group, "train_ids": [records[i]["id"] for i in train],
                     "test_ids": [records[i]["id"] for i in test], "valid_bodies": []}
        for reg in REGULARIZERS:
            fitted = fit_once(template, features, records, train, reg)
            body_scores = []
            for b, diag in enumerate(fitted["fit"]["component_diagnostics"]):
                if not diag["full_rank"]:
                    continue
                prediction = features[test, b] @ np.asarray(fitted["theta"])
                scale = np.asarray(diag["target_scale_train_only"])
                body_scores.extend(((prediction - truth[test, b])/scale).reshape(-1)**2)
                if reg == REGULARIZERS[0]:
                    fold_info["valid_bodies"].append(BODY_ORDER[b])
            if body_scores:
                scores[reg].append(float(np.mean(body_scores)))
        fold_details.append(fold_info)
    averaged = {reg: float(np.mean(value)) if value else None for reg, value in scores.items()}
    eligible = [reg for reg in REGULARIZERS if averaged[reg] is not None]
    chosen = min(eligible, key=lambda r: (averaged[r], -r)) if eligible else REGULARIZERS[-1]
    return chosen, {"status": "training_only_inner_cv" if eligible else "no_identifiable_inner_predictions",
                    "scores": {str(k): v for k, v in averaged.items()}, "folds": fold_details,
                    "tie_break": "larger ridge", "partial_rank_policy": "only individually full-rank body predictions enter inner loss; all omissions listed"}


def predict_groups(model, features, records, indices):
    all_outputs = {g: [] for g in GROUP_NAMES}
    for i in indices:
        w = features[i] @ np.asarray(model["theta"])
        output = recompose(records[i]["q"], w, model["geometry"])
        for b, name in enumerate(BODY_ORDER):
            all_outputs[name].append(w[b])
        all_outputs["total"].append(output["total"])
        all_outputs["joint"].append(output["tau_hydro"])
    return {g: np.asarray(v) for g, v in all_outputs.items()}


def metric_array(truth, pred, scales, valid):
    n_total = len(truth)
    if not np.any(valid):
        return {"n": 0, "n_total": n_total, "eligible_complete_score": False, "channels": [], "group_nrmse": None}
    residual = pred[valid]-truth[valid]
    scaled = residual/scales[valid]
    names = CHANNELS if truth.shape[1] == 6 else (CHANNELS+("J1", "J2", "J3") if truth.shape[1] == 9 else ("J1", "J2", "J3"))
    rows = []
    for k, name in enumerate(names):
        e = residual[:, k]
        rows.append({"channel": name, "unit": "N" if truth.shape[1] in (6,9) and k < 3 else "N*m",
                     "MAE": float(np.mean(np.abs(e))), "RMSE": float(np.sqrt(np.mean(e*e))),
                     "bias": float(np.mean(e)), "maxabs": float(np.max(np.abs(e))),
                     "NRMSE": float(np.sqrt(np.mean(scaled[:, k]**2)))})
    return {"n": int(np.sum(valid)), "n_total": n_total, "eligible_complete_score": bool(np.all(valid)),
            "channels": rows, "group_nrmse": float(np.sqrt(np.mean(scaled*scaled)))}


def parity_metrics(records, truths, preds, valid):
    pairs = []
    for i, r in enumerate(records):
        vi = np.asarray(r["nu"])
        for j in range(i+1, len(records)):
            if records[j]["pose_id"] == r["pose_id"] and np.allclose(np.asarray(records[j]["nu"]), -vi, atol=1e-10):
                pairs.append((i, j))
    result = {}
    for g in GROUP_NAMES:
        good = [(i, j) for i, j in pairs if valid[g][i] and valid[g][j]]
        if not good:
            result[g] = {"pair_count": 0}
            continue
        even, odd = [], []
        for i, j in good:
            ei, ej = preds[g][i]-truths[g][i], preds[g][j]-truths[g][j]
            even.append((ei+ej)/2)
            odd.append((ei-ej)/2)
        result[g] = {"pair_count": len(good), "even_residual_RMSE": np.sqrt(np.mean(np.asarray(even)**2, axis=0)).tolist(),
                     "odd_residual_RMSE": np.sqrt(np.mean(np.asarray(odd)**2, axis=0)).tolist()}
    return result


def development_cv(template, features, records, scheme):
    targets = target_groups(records)
    predictions = {g: np.zeros_like(y) for g, y in targets.items()}
    scales = {g: np.zeros_like(y) for g, y in targets.items()}
    valid = {g: np.zeros(len(records), dtype=bool) for g in GROUP_NAMES}
    labels = split_groups(records, scheme)
    folds = []
    for group in sorted(set(labels)):
        train = np.array([i for i, x in enumerate(labels) if x != group], int)
        test = np.array([i for i, x in enumerate(labels) if x == group], int)
        regularizer, inner = select_regularizer(template, features, records, train, scheme)
        fitted = fit_once(template, features, records, train, regularizer)
        predicted = predict_groups(fitted, features, records, test)
        train_scales = training_scales(targets, train)
        rank_flags = [d["full_rank"] for d in fitted["fit"]["component_diagnostics"]]
        for g in GROUP_NAMES:
            predictions[g][test] = predicted[g]
            scales[g][test] = train_scales[g]
            flag = rank_flags[BODY_ORDER.index(g)] if g in BODY_ORDER else all(rank_flags if g == "total" else rank_flags[2:])
            valid[g][test] = flag
        folds.append({"held_group": group, "train_ids": [records[i]["id"] for i in train],
                      "test_ids": [records[i]["id"] for i in test], "regularizer": regularizer,
                      "inner_selection": inner, "component_diagnostics": fitted["fit"]["component_diagnostics"]})
    metrics = {g: metric_array(targets[g], predictions[g], scales[g], valid[g]) for g in GROUP_NAMES}
    result = {"role": "retrospective development CV, not blind", "scheme": scheme, "folds": folds,
              "metrics": metrics, "parity": parity_metrics(records, targets, predictions, valid),
              "all_primary_predictions_identified": all(np.all(valid[g]) for g in PRIMARY_GROUPS),
              "prediction_ids": [r["id"] for r in records], "predictions": predictions,
              "prediction_identified": valid, "normalizers_train_only": scales}
    return result


def compare_acceptance(baseline, candidate):
    group_changes, channel_changes, failures = {}, {}, []
    for g in PRIMARY_GROUPS:
        a, b = baseline["metrics"][g], candidate["metrics"][g]
        if not a["eligible_complete_score"] or not b["eligible_complete_score"]:
            failures.append(f"{g}: nonidentifiable or missing held-out predictions")
            continue
        denom = a["group_nrmse"]
        improvement = (1 - b["group_nrmse"]/denom)*100 if denom > 1e-15 else None
        group_changes[g] = improvement
        if improvement is None or improvement < 10:
            failures.append(f"{g}: group NRMSE improvement below 10%")
        for ac, bc in zip(a["channels"], b["channels"]):
            name = g+"."+ac["channel"]
            imp = (1-bc["RMSE"]/ac["RMSE"])*100 if ac["RMSE"] > 1e-15 else None
            channel_changes[name] = imp
            if bc["RMSE"] > 1.05*ac["RMSE"]+1e-12:
                failures.append(name+": RMSE worsens more than 5%")
            if g == "joint" and ac["channel"] in ("J2", "J3") and (imp is None or imp < 10):
                failures.append(name+": RMSE improvement below 10%")
    return {"passed": not failures, "group_nrmse_improvement_percent": group_changes,
            "channel_RMSE_improvement_percent": channel_changes, "failures": failures,
            "predefined_tolerance": "no fluctuation relaxation; 1e-12 only arithmetic tolerance, not measurement precision"}


def frozen_reference(records, theta, geometry):
    predictions = {"total": [], "joint": []}
    for r in records:
        out = frozen87_steady(theta, r["q"], r["nu"], geometry)
        predictions["total"].append(out["total"])
        predictions["joint"].append(out["tau_hydro"])
    targets = target_groups(records)
    result = {"role": "unchanged old87 deployment reference; its original training differs and has seen existing data",
              "pure_hull_available": False, "same_data_refit_performed": False, "metrics_by_development_scaling": {},
              "predictions": predictions, "prediction_ids": [r["id"] for r in records]}
    for scheme in ("pose", "direction"):
        labels = split_groups(records, scheme)
        scales = {g: np.zeros_like(targets[g]) for g in ("total", "joint")}
        for group in set(labels):
            tr = np.array([i for i, x in enumerate(labels) if x != group])
            te = np.array([i for i, x in enumerate(labels) if x == group])
            s = training_scales(targets, tr)
            for g in scales:
                scales[g][te] = s[g]
        result["metrics_by_development_scaling"][scheme] = {g: metric_array(targets[g], np.asarray(predictions[g]),
            scales[g], np.ones(len(records), bool)) for g in scales}
    return result


def common_strict_capped_sensitivity(report):
    strict = report["populations"]["strict37"]
    capped = report["populations"]["including_capped40"]
    rows = strict["records_for_reporting"]
    targets = target_groups(rows)
    sensitivity = {"comparison": "same 37 strict held-out cases; adding capped cases only changes eligible training/inner folds",
        "normalization": "both predictions use the original strict37 outer-fold normalizers for this comparison",
        "note": "capped cases in the held-out pose/direction group stay held out; no test-group labels enter training",
        "candidates": {}}
    for c in ("A", "B", "C"):
        sensitivity["candidates"][c] = {}
        for scheme in ("pose", "direction"):
            reference = strict["candidates"][c]["development_cv"][scheme]
            expanded = capped["candidates"][c]["development_cv"][scheme]
            positions = {rid: i for i, rid in enumerate(expanded["prediction_ids"])}
            selected = np.array([positions[r["id"]] for r in rows], int)
            metrics = {g: metric_array(targets[g], np.asarray(expanded["predictions"][g])[selected],
                np.asarray(reference["normalizers_train_only"][g]), np.asarray(expanded["prediction_identified"][g])[selected]) for g in GROUP_NAMES}
            sensitivity["candidates"][c][scheme] = {"excluding_capped": reference["metrics"],
                "including_capped_scored_on_strict37": metrics,
                "complete_score_required_for_comparison": True}
    return sensitivity


def q0_x_pair_diagnostic(report):
    result = report["populations"]["strict37"]
    rows = result["records_for_reporting"]
    ids = [r["id"] for r in rows]
    pos, neg = ids.index("q0__T_px_08"), ids.index("q0__T_nx_08")
    truth = target_groups(rows)
    diagnostic = {"ids": [ids[pos], ids[neg]], "note": "q0 axial odd basis cannot reproduce same-sign J2/J3 pair; this is NOT a claim that all hydrodynamics must be odd",
        "true_loads": {}, "development_predictions": {}}
    for g in PRIMARY_GROUPS:
        diagnostic["true_loads"][g] = {"positive": truth[g][pos].tolist(), "negative": truth[g][neg].tolist(),
            "even_part": ((truth[g][pos]+truth[g][neg])/2).tolist(), "odd_part": ((truth[g][pos]-truth[g][neg])/2).tolist()}
    for c in ("A", "B", "C"):
        diagnostic["development_predictions"][c] = {}
        for scheme in ("pose", "direction"):
            cv = result["candidates"][c]["development_cv"][scheme]
            diagnostic["development_predictions"][c][scheme] = {}
            for g in PRIMARY_GROUPS:
                pred = np.asarray(cv["predictions"][g])
                diagnostic["development_predictions"][c][scheme][g] = {"positive": pred[pos].tolist(),
                    "negative": pred[neg].tolist(), "even_part": ((pred[pos]+pred[neg])/2).tolist(),
                    "odd_part": ((pred[pos]-pred[neg])/2).tolist(),
                    "prediction_identified": bool(cv["prediction_identified"][g][pos] and cv["prediction_identified"][g][neg])}
    return diagnostic


def write_tables(report, output_dir):
    with (output_dir/"model_metrics.csv").open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["population", "candidate", "split", "group", "n", "n_total", "eligible_complete_score",
                  "channel", "unit", "MAE", "RMSE", "bias", "maxabs", "NRMSE"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for population, result in report["populations"].items():
            for candidate, cv in result["candidates"].items():
                for scheme, run in cv["development_cv"].items():
                    for group, metrics in run["metrics"].items():
                        for row in metrics["channels"] or [{"channel": "NONIDENTIFIABLE"}]:
                            writer.writerow(dict(population=population, candidate=candidate, split=scheme, group=group,
                                n=metrics["n"], n_total=metrics["n_total"], eligible_complete_score=metrics["eligible_complete_score"], **row))
            for scheme, metrics_by_group in result["old87_deployment_reference"]["metrics_by_development_scaling"].items():
                for group, metrics in metrics_by_group.items():
                    for row in metrics["channels"]:
                        writer.writerow(dict(population=population, candidate="old87_frozen", split=scheme, group=group,
                            n=metrics["n"], n_total=metrics["n_total"], eligible_complete_score=True, **row))
    with (output_dir/"model_predictions.csv").open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["population", "candidate", "split", "id", "group", "channel", "prediction_identified", "truth", "prediction", "error", "training_normalizer"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for population, result in report["populations"].items():
            records = result["records_for_reporting"]
            truth = target_groups(records)
            for candidate, cr in result["candidates"].items():
                for scheme, cv in cr["development_cv"].items():
                    for i, rid in enumerate(cv["prediction_ids"]):
                        for g in GROUP_NAMES:
                            for k, channel in enumerate(CHANNELS if g != "joint" else ("J1", "J2", "J3")):
                                p = float(cv["predictions"][g][i, k])
                                identified = bool(cv["prediction_identified"][g][i])
                                writer.writerow(dict(population=population,candidate=candidate,split=scheme,id=rid,group=g,channel=channel,
                                    prediction_identified=identified,truth=float(truth[g][i,k]),prediction=p,
                                    error=p-float(truth[g][i,k]) if identified else "",training_normalizer=float(cv["normalizers_train_only"][g][i,k])))
    rows = []
    for population, result in report["populations"].items():
        for candidate, cr in result["candidates"].items():
            for scheme, cv in cr["development_cv"].items():
                for g in PRIMARY_GROUPS:
                    m=cv["metrics"][g]
                    rows.append("<tr>"+"".join("<td>"+html.escape(str(v))+"</td>" for v in
                        (population,candidate,scheme,g,f'{m["n"]}/{m["n_total"]}',m["group_nrmse"],m["eligible_complete_score"]))+"</tr>")
    doc = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Stage14 development CV</title>'
    doc += '<style>body{font:15px system-ui;margin:30px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:6px}pre{white-space:pre-wrap}</style>'
    doc += '<h1>阶段14：稳态分部件候选 / development CV</h1><p>旧数据均已观察，不是盲测。NRMSE的1 N / 0.1 N·m分母下限不是误差精度。C的非识别预测保留但不得当成正常泛化分数。</p>'
    doc += '<pre>'+html.escape(json.dumps(report["recommendation"], ensure_ascii=False, indent=2))+'</pre>'
    doc += '<table><tr><th>Population</th><th>Candidate</th><th>Split</th><th>Group</th><th>Identified n</th><th>NRMSE</th><th>Complete</th></tr>'+''.join(rows)+'</table>'
    doc += '<h2>strict37：逐通道SI误差</h2><p>行顺序Fx/Fy/Fz（N）、Mx/My/Mz（N·m）；关节J1/J2/J3（N·m）。NRMSE不能代替绝对误差。</p>'
    doc += '<table><tr><th>候选</th><th>折分</th><th>组</th><th>通道</th><th>单位</th><th>N</th><th>MAE</th><th>RMSE</th><th>bias</th><th>maxabs</th></tr>'
    for c, cr in report["populations"]["strict37"]["candidates"].items():
        for scheme, cv in cr["development_cv"].items():
            for g in PRIMARY_GROUPS:
                metrics = cv["metrics"][g]
                for ch in metrics["channels"]:
                    values = [c,scheme,g,ch["channel"],ch["unit"],f'{metrics["n"]}/{metrics["n_total"]}']
                    values.extend(f'{ch[k]:.6g}' for k in ("MAE","RMSE","bias","maxabs"))
                    doc += '<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>'
    doc += '</table><h2>验收：B/C相对同数据A</h2><pre>'+html.escape(json.dumps(report["populations"]["strict37"]["acceptance_vs_A"],ensure_ascii=False,indent=2))+'</pre>'
    doc += '<h2>capped敏感性：共同37个strict测试样本</h2><table><tr><th>候选</th><th>折分</th><th>组</th><th>排除capped NRMSE</th><th>纳入capped NRMSE</th><th>完整可识别</th></tr>'
    for c, schemes in report["capped_common_strict37_sensitivity"]["candidates"].items():
        for scheme, comparison in schemes.items():
            for g in PRIMARY_GROUPS:
                a, b = comparison["excluding_capped"][g], comparison["including_capped_scored_on_strict37"][g]
                values = [c,scheme,g,a["group_nrmse"],b["group_nrmse"],a["eligible_complete_score"] and b["eligible_complete_score"]]
                doc += '<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>'
    doc += '</table><h2>q0 ±X 0.8真实偶/奇分量</h2><pre>'+html.escape(json.dumps(report["q0_x_pair_diagnostic"]["true_loads"],ensure_ascii=False,indent=2))+'</pre>'
    doc += '<p>原87 frozen仅作部署参考，原hull块包含base0，不能冒充纯hull。原added-Coriolis完整保留；非零姿态/斜向流动可有偶项。原训练量不同，未用其部署误差冒充同数据结构优劣。</p>'
    doc += '<p>完整SI通道表：model_metrics.csv。折分/秩/内层选择/逐案例预测：model_development.json与model_predictions.csv。物理端口均为bbox约束的虚拟面假设，不是真实CAD法向。静态模型不验证惯性/独立关节运动；默认对此拒绝。</p></html>'
    (output_dir/"model_development.html").write_text(doc, encoding="utf-8")


def load_shared_dataset(path, contract):
    if not path.is_file():
        raise FileNotFoundError(f"Waiting for the authorized shared dataset: {path}; no substitute data are permitted")
    data = json.loads(path.read_text(encoding="utf-8"))
    missing = set(contract["dataset_required_top_keys"])-set(data)
    if missing:
        raise ValueError(f"shared dataset missing keys: {sorted(missing)}")
    rows = data["steady"]
    if len(rows) != 40 or sum(bool(r["strict"]) for r in rows) != 37:
        raise ValueError("expected 37 strict + 3 capped steady cases")
    if {r["id"] for r in rows if not r["strict"]} != set(contract["validation"]["capped_ids"]):
        raise ValueError("capped ID mismatch")
    if len({r["id"] for r in rows}) != len(rows):
        raise ValueError("duplicate case IDs")
    for r in rows:
        if np.asarray(r["wrenches"]).shape != (5,6) or not np.isfinite(r["wrenches"]).all():
            raise ValueError("invalid component wrenches")
        if np.max(np.abs(np.asarray(r["nu"])[3:])) > 1e-12:
            raise ValueError("static fit only accepts translational steady cases")
        assembled = recompose(r["q"], r["wrenches"], contract)
        if not np.allclose(assembled["total"],r["total"],atol=1e-8,rtol=1e-9) or not np.allclose(assembled["tau_hydro"],r["joint_tau"],atol=1e-8,rtol=1e-9):
            raise ValueError("shared dataset wrench projection does not close: "+r["id"])
    return data


def export_generalized9_summary():
    """Additional nine-target view; no refit and no change to ABC/B_pair results."""
    original=json.loads((ROOT/"reports/model_development.json").read_text(encoding="utf-8"))
    pair=json.loads((ROOT/"reports/model_b_pair_development.json").read_text(encoding="utf-8"))
    data=json.loads((ROOT/"data/component_dataset.json").read_text(encoding="utf-8"))
    row_map={r["id"]:r for r in data["steady"]}
    result={"role":"nine generalized targets supplementary view of unchanged development predictions",
        "order":list(CHANNELS)+["J1","J2","J3"],
        "normalization":"outer-training-only scales, floors 1 N /0.1 N*m; floors are NOT error precision",
        "frozen87_role":"deployment reference only; different original training, no same-data superiority claim", "populations":{}}
    for population,source in original["populations"].items():
        result["populations"][population]={}
        for candidate in ("A","B","C","B_pair","old87_frozen"):
            result["populations"][population][candidate]={}
            for scheme in ("pose","direction"):
                cv=pair["populations"][population]["development_cv"][scheme] if candidate=="B_pair" else source["candidates"][candidate if candidate!="old87_frozen" else "A"]["development_cv"][scheme]
                ids=cv["prediction_ids"]
                truth=np.asarray([row_map[rid]["total"]+row_map[rid]["joint_tau"] for rid in ids],float)
                norms=np.column_stack((cv["normalizers_train_only"]["total"],cv["normalizers_train_only"]["joint"]))
                if candidate=="old87_frozen":
                    old=source["old87_deployment_reference"]
                    assert old["prediction_ids"]==ids
                    pred=np.column_stack((old["predictions"]["total"],old["predictions"]["joint"]))
                    valid=np.ones(len(ids),bool)
                else:
                    pred=np.column_stack((cv["predictions"]["total"],cv["predictions"]["joint"]))
                    valid=np.asarray(cv["prediction_identified"]["total"],bool)&np.asarray(cv["prediction_identified"]["joint"],bool)
                result["populations"][population][candidate][scheme]=metric_array(truth,pred,norms,valid)
    save_json(ROOT/"reports/model_generalized9_metrics.json",result)
    with (ROOT/"reports/model_generalized9_metrics.csv").open("w",newline="",encoding="utf-8-sig") as f:
        writer=csv.DictWriter(f,fieldnames=["population","candidate","split","n","n_total","eligible_complete_score","channel","unit","MAE","RMSE","bias","maxabs","NRMSE"])
        writer.writeheader()
        for population,models in result["populations"].items():
            for candidate,schemes in models.items():
                for scheme,metrics in schemes.items():
                    for row in metrics["channels"]:
                        writer.writerow(dict(population=population,candidate=candidate,split=scheme,n=metrics["n"],n_total=metrics["n_total"],eligible_complete_score=metrics["eligible_complete_score"],**row))
    return result


def fit_b_pair_supplement(dataset_path, contract, data):
    """One parent-approved bounded extension. ABC artefacts remain byte-identical."""
    original_report_path = ROOT/"reports/model_development.json"
    original = json.loads(original_report_path.read_text(encoding="utf-8"))
    guarded_paths = [original_report_path, ROOT/"reports/model_metrics.csv", ROOT/"reports/model_predictions.csv"]
    guarded_paths += [ROOT/"models"/f"candidate_{c}_{population}.json"
        for c in ("A","B","C") for population in ("strict37","including_capped40")]
    guarded_paths += [ROOT.parent/"11_theta_fit/matlab/theta_fit.json",ROOT.parent/"11_theta_fit/matlab/theta_fit.mat"]
    before = {str(p):sha256_file(p) for p in guarded_paths}
    report = {"schema_version":"split_hydro_b_pair_development_v1",
        "role":"one bounded parent-requested development extension after ABC; not blind",
        "dataset_sha256":sha256_file(dataset_path),"original_ABC_report_sha256":before[str(original_report_path)],
        "prespecified_supplement":"B + link2/link3 pair lift at bbox-centre +/-0.8 half-length on longest bbox axis; 6 new coefficients, no direction/point tuning",
        "protocol":original["protocol"],"original_ABC_hashes_before":before,"populations":{}}
    report["component_passivity_assumption"] = {"interpretation":"independent component passivity is a candidate restriction, NOT a fluid-interference law or a CFD quality filter",
        "all_cases_retained":True,"positive_component_power_cases":[],"total_power_range_W":None}
    total_power = []
    for r in data["steady"]:
        if not r["strict"]:continue
        z = np.r_[r["nu"],np.zeros(3)]
        power = np.sum((body_jacobians(r["q"],contract)@z)*np.asarray(r["wrenches"]),axis=1)
        total_power.append(float(np.asarray(r["nu"])@np.asarray(r["total"])))
        for i in np.flatnonzero(power>1e-12):
            report["component_passivity_assumption"]["positive_component_power_cases"].append({"id":r["id"],"body":BODY_ORDER[i],"power_W":float(power[i])})
    report["component_passivity_assumption"]["total_power_range_W"]=[min(total_power),max(total_power)]
    for population,rows in (("strict37",[r for r in data["steady"] if r["strict"]]),("including_capped40",data["steady"])):
        print(f"Fitting bounded B_pair supplement: {population}",flush=True)
        template = build_template(contract,data["bodies"],"B_pair")
        features = np.asarray([component_features(template,r["q"],r["nu"]) for r in rows])
        cvs = {s:development_cv(template,features,rows,s) for s in ("pose","direction")}
        reg,inner = select_regularizer(template,features,rows,np.arange(len(rows)),"pose")
        fitted = fit_once(template,features,rows,np.arange(len(rows)),reg)
        fitted.update({"training_dataset_sha256":report["dataset_sha256"],"training_population":population,
            "final_regularizer_selection":inner,"freeze_status":"candidate_only_parent_controls_freeze",
            "development_cv_complete":{s:cvs[s]["all_primary_predictions_identified"] for s in cvs}})
        fitted["retention_status"]="eligible_for_comparison" if fitted["fit"]["full_rank"] and all(fitted["development_cv_complete"].values()) else "rejected_nonidentifiable_development_folds"
        fitted["component_passivity_note"]=report["component_passivity_assumption"]["interpretation"]
        model_path=ROOT/"models"/f"candidate_B_pair_{population}.json"
        save_json(model_path,fitted)
        with (ROOT/"models"/f"candidate_B_pair_{population}_theta.csv").open("w",newline="",encoding="utf-8-sig") as f:
            writer=csv.DictWriter(f,fieldnames=["schema_version","name","unit","block","theta","lower_bound","upper_bound"])
            writer.writeheader()
            for p,t in zip(fitted["parameters"],fitted["theta"]):
                writer.writerow({"schema_version":fitted["schema_version"],"theta":t,**{k:p[k] for k in ("name","unit","block","lower_bound","upper_bound")}})
        report["populations"][population]={"n":len(rows),"model_path":str(model_path),"n_parameters":len(fitted["theta"]),
            "full_fit_rank":fitted["fit"]["full_rank"],"retention_status":fitted["retention_status"],"development_cv":cvs,
            "comparison_vs_A":{s:compare_acceptance(original["populations"][population]["candidates"]["A"]["development_cv"][s],cvs[s]) for s in cvs},
            "comparison_vs_B":{s:compare_acceptance(original["populations"][population]["candidates"]["B"]["development_cv"][s],cvs[s]) for s in cvs}}
    rows=[r for r in data["steady"] if r["strict"]]
    truth=target_groups(rows)
    report["capped_common_strict37_sensitivity"]={}
    for scheme in ("pose","direction"):
        strict=report["populations"]["strict37"]["development_cv"][scheme]
        expanded=report["populations"]["including_capped40"]["development_cv"][scheme]
        positions={rid:i for i,rid in enumerate(expanded["prediction_ids"])}
        indices=np.array([positions[r["id"]] for r in rows])
        report["capped_common_strict37_sensitivity"][scheme]={"normalization":"same strict37 training-only scales for both predictions", "excluding_capped":strict["metrics"],
            "including_capped_scored_on_strict37":{g:metric_array(truth[g],expanded["predictions"][g][indices],strict["normalizers_train_only"][g],expanded["prediction_identified"][g][indices]) for g in GROUP_NAMES}}
    passed=all(x["passed"] for x in report["populations"]["strict37"]["comparison_vs_A"].values())
    report["recommendation"]={"acceptance_passed":passed,"winner":"B_pair" if passed else None,
        "freeze_recommendation":"parent may freeze for research validation only; no superiority established" if not passed else "parent controls freeze before new blind CFD",
        "further_model_search_performed":False,"original_ABC_preserved":True,"default_theta_unchanged":True}
    after={str(p):sha256_file(p) for p in guarded_paths}
    if before!=after:raise RuntimeError("B_pair supplement changed guarded ABC or default theta artefacts")
    report["original_ABC_hashes_after"]=after
    with (ROOT/"reports/model_b_pair_metrics.csv").open("w",newline="",encoding="utf-8-sig") as f:
        fields=["population","candidate","split","group","n","n_total","eligible_complete_score","channel","unit","MAE","RMSE","bias","maxabs","NRMSE"]
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for population,result in report["populations"].items():
            for scheme,cv in result["development_cv"].items():
                for g,m in cv["metrics"].items():
                    for ch in m["channels"] or [{"channel":"NONIDENTIFIABLE"}]:
                        writer.writerow(dict(population=population,candidate="B_pair",split=scheme,group=g,n=m["n"],n_total=m["n_total"],eligible_complete_score=m["eligible_complete_score"],**ch))
    with (ROOT/"reports/model_b_pair_predictions.csv").open("w",newline="",encoding="utf-8-sig") as f:
        fields=["population","split","id","group","channel","prediction_identified","truth","prediction","error","training_normalizer"]
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        row_map={r["id"]:r for r in data["steady"]}
        for population,result in report["populations"].items():
            for scheme,cv in result["development_cv"].items():
                targets=target_groups([row_map[rid] for rid in cv["prediction_ids"]])
                for i,rid in enumerate(cv["prediction_ids"]):
                    for g in GROUP_NAMES:
                        for k,ch in enumerate(CHANNELS if g!="joint" else ("J1","J2","J3")):
                            identified=bool(cv["prediction_identified"][g][i]);pred=float(cv["predictions"][g][i,k]);actual=float(targets[g][i,k])
                            writer.writerow(dict(population=population,split=scheme,id=rid,group=g,channel=ch,prediction_identified=identified,
                                truth=actual,prediction=pred,error=pred-actual if identified else "",training_normalizer=float(cv["normalizers_train_only"][g][i,k])))
    save_json(ROOT/"reports/model_b_pair_development.json",report)
    summary={"recommendation":report["recommendation"],"definition":report["prespecified_supplement"],
        "comparison_vs_A":report["populations"]["strict37"]["comparison_vs_A"],
        "comparison_vs_B":report["populations"]["strict37"]["comparison_vs_B"],
        "component_passivity_assumption":report["component_passivity_assumption"],
        "primary_metrics":{p:{s:{g:v["metrics"][g] for g in PRIMARY_GROUPS} for s,v in r["development_cv"].items()} for p,r in report["populations"].items()}}
    save_json(ROOT/"reports/model_b_pair_summary.json",summary)
    text='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>B_pair development supplement</title><style>body{font:15px system-ui;margin:30px}pre{white-space:pre-wrap}</style>'
    text+='<h1>B_pair：一次有界开发补充，不是盲测</h1><p>L2/L3各增加3个成对零功横向力系数；固定几何点，不调位置。原A/B/C数据/模型/报告哈希保持不变。每体被动性只是候选约束，局部正功CFD全部保留。</p><pre>'
    text+=html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre></html>'
    (ROOT/"reports/model_b_pair_development.html").write_text(text,encoding="utf-8")
    print(json.dumps(report["recommendation"],ensure_ascii=False),flush=True)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT/"data/component_dataset.json")
    parser.add_argument("--frozen-theta", type=Path, default=ROOT.parent/"11_theta_fit/matlab/theta_fit.json")
    parser.add_argument("--supplement-b-pair", action="store_true", help="fit only the one approved B_pair supplement, preserving ABC outputs")
    args = parser.parse_args(argv)
    # Constrain outputs; callers cannot redirect reports to old engineering files.
    contract_path, report_dir, model_dir = ROOT/"contract.json", ROOT/"reports", ROOT/"models"
    if args.dataset.resolve() != (ROOT/"data/component_dataset.json").resolve():
        raise ValueError("Only the common contracted component dataset may supply labels")
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    data = load_shared_dataset(args.dataset, contract)
    if args.supplement_b_pair:
        return fit_b_pair_supplement(args.dataset, contract, data)
    original_paths = [ROOT.parent/"11_theta_fit/matlab/theta_fit.json", ROOT.parent/"11_theta_fit/matlab/theta_fit.mat"]
    hashes_before = {str(p): sha256_file(p) for p in original_paths}
    frozen_json = json.loads(args.frozen_theta.read_text(encoding="utf-8"))
    theta_old = frozen_json["theta"]
    if len(theta_old) != 87:
        raise ValueError("deployment reference must be the original frozen87")
    report = {"schema_version": "split_hydro_development_v1", "dataset_sha256": sha256_file(args.dataset),
              "contract_sha256": sha256_file(contract_path), "data_role": "retrospective development CV, not blind",
              "candidate_dictionary_policy": "geometry-only fixed ports before this fit; historical labels were already seen, not preregistered/blind", 
              "protocol": {"outer_schemes": ["leave_one_pose_out", "leave_one_direction_family_out"],
                  "grouping": "positive/negative direction and same-axis different speeds grouped across poses; no within-case splitting",
                  "regularizer_grid": REGULARIZERS, "regularizer_selection": "nested training-only grouped CV, same grouping scheme",
                  "fit_targets": "five independently weighted six-channel component wrenches",
                  "accepted_improvement": "hull6,total6,joint3 group NRMSE and J2,J3 SI RMSE >=10%; all 15 primary channel RMSE no worse than5%; both schemes",
                  "rank_policy": "no regularization can create identifiability; incomplete body ranks exclude their heldout metrics, never silently zero-and-score",
                  "units": "N / N*m", "normalizer": "max(training RMS,1N or0.1N*m), not error precision",
                  "blind_results_accessed": False, "old_structure_same_data_refit": False,
                  "dynamic_status": "6 actual oscillations acknowledged only; static model does not validate inertia or independent joint motion"},
              "oscillation_record_count_not_used_for_static_fit": len(data["oscillations"]),
              "old87_source": str(args.frozen_theta), "old87_sha256": sha256_file(args.frozen_theta),
              "default_theta_hashes_before": hashes_before, "populations": {}}
    for population, rows in (("strict37",[r for r in data["steady"] if r["strict"]]),("including_capped40",data["steady"])):
        population_result = {"n": len(rows), "candidates": {}, "records_for_reporting": rows,
                             "old87_deployment_reference": frozen_reference(rows,theta_old,contract)}
        for candidate in ("A","B","C"):
            print(f"Fitting {population} candidate {candidate}",flush=True)
            template=build_template(contract,data["bodies"],candidate)
            features=np.asarray([component_features(template,r["q"],r["nu"]) for r in rows])
            cvs={scheme:development_cv(template,features,rows,scheme) for scheme in ("pose","direction")}
            # Full-data final fit is NOT a holdout score. Choose ridge by pose-group inner CV.
            indices=np.arange(len(rows))
            reg,inner=select_regularizer(template,features,rows,indices,"pose")
            fitted=fit_once(template,features,rows,indices,reg)
            fitted["training_dataset_sha256"]=report["dataset_sha256"]
            fitted["training_population"]=population
            fitted["final_regularizer_selection"]=inner
            fitted["freeze_status"]="candidate_only_parent_controls_freeze"
            fitted["development_cv_complete"]={s:cvs[s]["all_primary_predictions_identified"] for s in cvs}
            fitted["retention_status"]="eligible_for_comparison" if all(fitted["development_cv_complete"].values()) and fitted["fit"]["full_rank"] else "rejected_nonidentifiable_development_folds"
            save_json(model_dir/f"candidate_{candidate}_{population}.json",fitted)
            with (model_dir/f"candidate_{candidate}_{population}_theta.csv").open("w",newline="",encoding="utf-8-sig") as f:
                writer=csv.DictWriter(f,fieldnames=["schema_version","name","unit","block","theta","lower_bound","upper_bound"])
                writer.writeheader()
                for p,t in zip(fitted["parameters"],fitted["theta"]):
                    writer.writerow({"schema_version":fitted["schema_version"],"theta":t,**{k:p[k] for k in ("name","unit","block","lower_bound","upper_bound")}})
            population_result["candidates"][candidate]={"n_parameters":len(fitted["theta"]),"full_fit_rank":fitted["fit"]["full_rank"],
                "full_fit_regularizer":reg,"retention_status":fitted["retention_status"],"development_cv":cvs}
        population_result["acceptance_vs_A"]={c:{s:compare_acceptance(population_result["candidates"]["A"]["development_cv"][s],
            population_result["candidates"][c]["development_cv"][s]) for s in ("pose","direction")} for c in ("B","C")}
        report["populations"][population]=population_result
    strict=report["populations"]["strict37"]
    winners=[c for c in ("B","C") if all(x["passed"] for x in strict["acceptance_vs_A"][c].values())]
    report["recommendation"]={"acceptance_passed_candidates":winners,"winner":winners[0] if len(winners)==1 else None,
        "freeze_recommendation":"no proven winner; parent may freeze labelled research candidates without claiming improvement" if not winners else "parent must perform explicit freeze before new blind CFD",
        "deployment_change_authorized":False,"default_theta_unchanged":True}
    report["capped_common_strict37_sensitivity"] = common_strict_capped_sensitivity(report)
    report["q0_x_pair_diagnostic"] = q0_x_pair_diagnostic(report)
    report["default_theta_hashes_after"]={str(p):sha256_file(p) for p in original_paths}
    if report["default_theta_hashes_before"] != report["default_theta_hashes_after"]:
        raise RuntimeError("default theta changed during candidate fitting")
    write_tables(report,report_dir)
    for result in report["populations"].values():
        del result["records_for_reporting"]
    save_json(report_dir/"model_development.json",report)
    save_json(report_dir/"model_capped_sensitivity.json",report["capped_common_strict37_sensitivity"])
    save_json(report_dir/"model_q0_x_pair.json",report["q0_x_pair_diagnostic"])
    summary={"role":report["data_role"],"recommendation":report["recommendation"],
        "strict37_acceptance":strict["acceptance_vs_A"],"rank_policy":report["protocol"]["rank_policy"],
        "primary_metrics":{p:{c:{s:{g:v["metrics"][g] for g in PRIMARY_GROUPS} for s,v in cr["development_cv"].items()}
            for c,cr in r["candidates"].items()} for p,r in report["populations"].items()},
        "rank_failures":[{"population":p,"candidate":c,"scheme":s,"held_group":fold["held_group"],"body":diag["body"],
                          "rank":diag["effective_rank"],"parameter_count":diag["parameter_count"],"unidentifiable_parameters":diag["unidentifiable_parameters"]}
            for p,r in report["populations"].items() for c,cr in r["candidates"].items()
            for s,v in cr["development_cv"].items() for fold in v["folds"] for diag in fold["component_diagnostics"] if not diag["full_rank"]]}
    save_json(report_dir/"model_summary.json",summary)
    save_json(report_dir/"model_hash_audit.json",{"before":hashes_before,"after":report["default_theta_hashes_after"],"unchanged":True})
    print(json.dumps(report["recommendation"],ensure_ascii=False),flush=True)
    return report


if __name__ == "__main__":
    main()

"""Restart-read two completed final case/data pairs, without changing physical state.

Only two representative pairs are verified, not all nine validation cases.
No initialization, iteration, mesh alteration, boundary/report-definition setter,
case/data write, model fit, or protocol update is performed.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time
import uuid
from pathlib import Path

import psutil

HERE = Path(__file__).resolve().parent
STAGE = HERE.parent
OUTPUT = HERE / "final_pair_readback.json"
CASE_IDS = ("T_V_qe_XY_p055", "O_V_q0_YZ")
BODY = ("wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4")
REPS = ["fx", "fy", "fz", "mx", "my", "mz", "tau_j1", "tau_j2", "tau_j3"]
CHANNELS = set(REPS + [name + "(" + zone + ")" for name in REPS[:6] for zone in BODY])
CENTROID_REPS = ["motion_centroid_" + axis for axis in "xyz"]
ABS_TOL = 1e-8
REL_TOL = 1e-8
FLOW_TIME_ABS_TOL_S = 1e-8


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(data)
    return digest.hexdigest()


def state_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")).hexdigest()


def save_report(value):
    # Only the authorized JSON and corresponding diagnostic .log files are written.
    candidate = HERE / ("final_pair_readback." + uuid.uuid4().hex + ".pending.log")
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
    with candidate.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    delays = (0.02, 0.05, 0.10, 0.20, 0.40)
    for attempt in range(len(delays) + 1):
        try:
            os.replace(str(candidate), str(OUTPUT))
            return
        except OSError as exc:
            if getattr(exc, "winerror", None) not in (5, 32, 33) or attempt == len(delays):
                raise
            time.sleep(delays[attempt])


def relative(path):
    return Path(path).resolve().relative_to(STAGE).as_posix()


def compare(actual, expected):
    actual, expected = float(actual), float(expected)
    if not math.isfinite(actual) or not math.isfinite(expected):
        return {"actual": repr(actual), "expected": repr(expected), "passed": False,
                "error": "nonfinite value"}
    absolute = abs(actual - expected)
    scale = max(abs(actual), abs(expected))
    threshold = max(ABS_TOL, REL_TOL * scale)
    return {"actual": actual, "expected": expected, "absolute_error": absolute,
            "relative_error_symmetric": absolute / scale if scale else 0.0,
            "allowed_absolute_error": threshold, "error_to_tolerance_ratio": absolute / threshold,
            "passed": absolute <= threshold}


def flatten(raw):
    values = {}
    for row in raw:
        if not isinstance(row, dict):
            raise RuntimeError("unexpected native report response")
        for name, value in row.items():
            while isinstance(value, (list, tuple)):
                value = value[0]
            if name in values:
                raise RuntimeError("duplicate native report channel: " + name)
            values[name] = float(value)
    return values


def solver_family():
    records = {}
    for process in psutil.process_iter(["pid", "name", "create_time"]):
        try:
            name = (process.info["name"] or "").lower()
            matched = any(token in name for token in ("fluent", "cortex", "fl_mpi", "3ddp", "2ddp"))
            if matched:
                records[process.pid] = {"pid": process.pid, "name": process.info["name"],
                                        "create_time": process.info["create_time"]}
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return records


def identify_session_processes(solver):
    records = solver_family()
    properties = solver.connection_properties
    public_pids = {}
    for name in ("cortex_pid", "fluent_host_pid"):
        value = getattr(properties, name, None)
        if value is None:
            continue
        try:
            pid = int(value)
            public_pids[name] = pid
            root = psutil.Process(pid)
            for process in [root] + root.children(recursive=True):
                if process.pid == os.getpid():
                    continue
                records[process.pid] = {"pid": process.pid, "name": process.name(),
                                        "create_time": process.create_time()}
        except (ValueError, psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return records, public_pids


def wait_for_exit(records):
    processes = []
    for item in records.values():
        try:
            process = psutil.Process(item["pid"])
            if process.create_time() == item["create_time"]:
                processes.append(process)
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(processes, timeout=15)
    remaining = []
    for process in alive:
        try:
            if process.is_running() and process.create_time() == records[process.pid]["create_time"]:
                remaining.append({"pid": process.pid, "name": process.name()})
        except psutil.NoSuchProcess:
            pass
    return {"owned_processes_identified": len(records), "owned_process_records": list(records.values()),
            "all_owned_processes_exited": bool(records) and not remaining,
            "remaining_owned_processes": remaining, "remaining_solver_family": list(solver_family().values())}


def input_preflight(case_id):
    if case_id not in CASE_IDS:
        raise RuntimeError("case outside explicitly authorized read-back scope")
    attempt = HERE / "runs" / case_id / "attempt_001"
    files = {name: attempt / name for name in ("final.cas.h5", "final.dat.h5", "history.json", "result.json")}
    for name, path in files.items():
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError("missing/empty input: " + str(path))
    before = {name: {"path": relative(path), "size_bytes": path.stat().st_size, "sha256": sha256(path)}
              for name, path in files.items()}
    result = json.loads(files["result.json"].read_text(encoding="utf-8"))
    history = json.loads(files["history.json"].read_text(encoding="utf-8"))
    if result.get("case_id") != case_id or result.get("status") != "completed" or result.get("execution_complete") is not True or result.get("failed_required") != []:
        raise RuntimeError("input result is not a valid completed case")
    if not history or set(history[-1].get("loads", {})) != CHANNELS:
        raise RuntimeError("last history sample does not contain the original 57 channels")
    if any(not math.isfinite(float(v)) for v in history[-1]["loads"].values()):
        raise RuntimeError("history contains nonfinite reference loads")
    terminal = result.get("terminal_state", {})
    if not terminal.get("id") or terminal.get("sample_count") != len(history):
        raise RuntimeError("terminal-state/history count mismatch")
    for label, filename in (("final_case", "final.cas.h5"), ("final_data", "final.dat.h5")):
        artifact = result["artifacts"][label]
        if artifact.get("path") != filename or artifact.get("state_id") != terminal["id"] or artifact.get("sha256") != before[filename]["sha256"]:
            raise RuntimeError("final case/data pair does not match completed same-state metadata")
    if result["artifacts"]["history"].get("sha256") != before["history.json"]["sha256"]:
        raise RuntimeError("history bytes differ from completed artifact metadata")
    if case_id == "O_V_q0_YZ":
        if result.get("mode") != "oscillation_translation" or len(history) != 240 or abs(float(history[-1]["t"]) - 6.0) > FLOW_TIME_ABS_TOL_S:
            raise RuntimeError("oscillation terminal sample is not the expected 240-step/6-second state")
        if len(result.get("mesh_reference_centroid_initial_m", [])) != 3 or len(history[-1]["motion"].get("mesh_displacement_m", [])) != 3:
            raise RuntimeError("missing original geometric centroid or actual mesh displacement")
    return files, before, result, history[-1]


def physical_reference_state(solver):
    rd = solver.solution.report_definitions
    return {"boundary_conditions": solver.setup.boundary_conditions.get_state(),
            "fluid_zone_settings": solver.setup.cell_zone_conditions.fluid.get_state(),
            "force_report_definitions": {name: rd.force[name].get_state() for name in REPS[:3]},
            "moment_report_definitions": {name: rd.moment[name].get_state() for name in REPS[3:]}}


def launch_fluent():
    sys.path.insert(0, str(STAGE.parent / "04_workbench"))
    from run_wb_fullenv import full_environment
    os.environ.update(full_environment())
    import ansys.fluent.core as pyfluent
    return pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=1,
                                 ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                 cleanup_on_exit=True, start_transcript=False, cwd=str(HERE))


def verify_one(case_id, report):
    row = {"case_id": case_id, "attempt": "attempt_001", "status": "started", "processor_count": 1, "steps": []}
    report["cases"].append(row)
    save_report(report)
    solver, files, before, owned = None, None, None, {}
    def step(name, action):
        record = {"name": name, "ok": False}
        row["steps"].append(record)
        started = time.monotonic()
        try:
            value = action()
            record["ok"] = True
            return value
        except Exception as exc:
            record["error"] = repr(exc)
            raise
        finally:
            record["seconds"] = round(time.monotonic() - started, 6)
            save_report(report)
    try:
        files, before, result, last = step("read_only_input_preflight", lambda: input_preflight(case_id))
        row["input_hashes_before"] = before
        row["terminal_state_id"] = result["terminal_state"]["id"]
        row["reference_history_index"] = result["terminal_state"]["sample_count"] - 1
        row["reference_last_iteration"] = last.get("iter")
        row["reference_last_time_s"] = last.get("t")
        if solver_family():
            raise RuntimeError("another solver is active; serial read-back cannot start")
        solver = step("launch_single_rank_readback_session", launch_fluent)
        owned, row["native_session_pids"] = identify_session_processes(solver)
        transcript = HERE / ("final_pair_readback_" + case_id + ".trn")
        row["transcript"] = relative(transcript)
        step("start_new_readback_transcript", lambda: solver.transcript.start(str(transcript)))
        step("read_final_case_only", lambda: solver.file.read_case(file_name=str(files["final.cas.h5"])))
        step("read_matching_final_data_only", lambda: solver.file.read_data(file_name=str(files["final.dat.h5"])))
        state_before = step("snapshot_physical_boundaries_and_report_references", lambda: physical_reference_state(solver))
        row["physical_settings_sha256_before_compute"] = state_digest(state_before)
        rd = solver.solution.report_definitions
        actual = step("compute_existing_original_57_reports", lambda: flatten(rd.compute(report_defs=REPS)))
        if set(actual) != CHANNELS:
            raise RuntimeError("read-back did not produce exactly the original 57 channels")
        comparisons = {name: compare(actual[name], last["loads"][name]) for name in sorted(CHANNELS)}
        finite_rows = {k: v for k, v in comparisons.items() if "absolute_error" in v}
        row["load_comparison"] = {"channels_compared": len(comparisons), "passing_channels": sum(v["passed"] for v in comparisons.values()),
                                  "all_passed": all(v["passed"] for v in comparisons.values()), "channels": comparisons,
                                  "maximum_absolute_error": max((v["absolute_error"] for v in finite_rows.values()), default=None),
                                  "maximum_error_to_tolerance_ratio": max((v["error_to_tolerance_ratio"] for v in finite_rows.values()), default=None),
                                  "worst_absolute_error_channel": max(finite_rows, key=lambda k: finite_rows[k]["absolute_error"]) if finite_rows else None}
        if case_id == "O_V_q0_YZ":
            flow_time = float(step("read_flow_time_without_advance", lambda: solver.rp_vars("flow-time")))
            if not math.isfinite(flow_time):
                raise RuntimeError("nonfinite saved flow time")
            row["flow_time"] = {"actual_s": flow_time, "expected_s": 6.0, "history_s": float(last["t"]),
                                "absolute_tolerance_s": FLOW_TIME_ABS_TOL_S, "absolute_error_s": abs(flow_time - 6.0),
                                "passed": abs(flow_time - 6.0) <= FLOW_TIME_ABS_TOL_S and abs(flow_time - last["t"]) <= FLOW_TIME_ABS_TOL_S}
            def existing_geometry_definitions():
                states = {}
                for axis, name in zip("xyz", CENTROID_REPS):
                    state = rd.surface[name].get_state()
                    if state.get("report_type") != "surface-areaavg" or state.get("field") != axis + "-coordinate" or state.get("surface_names") != ["wall_auv"]:
                        raise RuntimeError("saved hull centroid report does not match the frozen geometry definition")
                    states[name] = state
                return states
            geometry_before = step("read_existing_centroid_report_definitions_without_changes", existing_geometry_definitions)
            centroid_values = step("compute_existing_wall_auv_geometric_centroid", lambda: flatten(rd.compute(report_defs=CENTROID_REPS)))
            if set(centroid_values) != set(CENTROID_REPS):
                raise RuntimeError("incomplete saved centroid reports")
            centroid = [centroid_values[name] for name in CENTROID_REPS]
            initial = result["mesh_reference_centroid_initial_m"]
            displacement = last["motion"]["mesh_displacement_m"]
            expected_centroid = [a + b for a, b in zip(initial, displacement)]
            actual_displacement = [a - b for a, b in zip(centroid, initial)]
            cc = {axis: compare(a, b) for axis, a, b in zip("xyz", centroid, expected_centroid)}
            dc = {axis: compare(a, b) for axis, a, b in zip("xyz", actual_displacement, displacement)}
            geometry_after = step("recheck_unchanged_centroid_report_definitions", existing_geometry_definitions)
            row["oscillation_geometry"] = {"original_hull_surface_area_centroid_m": initial,
                                           "history_actual_mesh_displacement_m": displacement,
                                           "expected_centroid_m": expected_centroid, "readback_centroid_m": centroid,
                                           "readback_displacement_from_original_centroid_m": actual_displacement,
                                           "centroid_comparison": cc, "actual_displacement_comparison": dc,
                                           "centroid_report_definitions_unchanged": state_digest(geometry_before) == state_digest(geometry_after),
                                           "reference_note": "Original wall_auv area centroid, not original Ob; use recorded actual mesh displacement, not commanded analytic displacement",
                                           "passed": all(v["passed"] for v in list(cc.values()) + list(dc.values())) and state_digest(geometry_before) == state_digest(geometry_after)}
        state_after = step("recheck_unchanged_physical_boundaries_and_report_references", lambda: physical_reference_state(solver))
        row["physical_settings_sha256_after_compute"] = state_digest(state_after)
        row["physical_settings_unchanged"] = state_digest(state_before) == state_digest(state_after)
        row["numerical_checks_passed"] = row["load_comparison"]["all_passed"] and row["physical_settings_unchanged"]
        if case_id == "O_V_q0_YZ":
            row["numerical_checks_passed"] = row["numerical_checks_passed"] and row["flow_time"]["passed"] and row["oscillation_geometry"]["passed"]
        row["status"] = "checks_completed"
    except Exception as exc:
        row["status"] = "failed"
        row["error"] = repr(exc)
    finally:
        if solver is not None:
            try:
                additional, _ = identify_session_processes(solver)
                owned.update(additional)
            except Exception as exc:
                row["process_identification_note"] = repr(exc)
            try:
                step("stop_readback_transcript", solver.transcript.stop)
            except Exception as exc:
                row["transcript_stop_error"] = repr(exc)
                row["status"] = "failed"
            try:
                step("exit_readback_session_without_writing_inputs", solver.exit)
                row["normal_exit_requested"] = True
            except Exception as exc:
                row["exit_error"] = repr(exc)
                row["status"] = "failed"
            try:
                row["process_exit_verification"] = wait_for_exit(owned)
            except Exception as exc:
                row["process_exit_verification"] = {"all_owned_processes_exited": False, "error": repr(exc),
                                                     "remaining_solver_family": list(solver_family().values())}
                row["status"] = "failed"
        else:
            row["process_exit_verification"] = {"solver_not_launched": True, "remaining_solver_family": list(solver_family().values())}
        if files is not None and before is not None:
            row["input_hashes_after"] = {}
            for name, path in files.items():
                try:
                    row["input_hashes_after"][name] = {"path": relative(path), "sha256": sha256(path)}
                except OSError as exc:
                    row["input_hashes_after"][name] = {"path": relative(path), "sha256": None, "error": repr(exc)}
            row["all_input_hashes_unchanged"] = all(row["input_hashes_after"][name]["sha256"] == before[name]["sha256"] for name in files)
        if row["status"] == "checks_completed":
            exit_check = row["process_exit_verification"]
            row["status"] = "passed" if row.get("numerical_checks_passed") and row.get("all_input_hashes_unchanged") and exit_check.get("all_owned_processes_exited") and not exit_check.get("remaining_solver_family") else "failed"
        save_report(report)
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="read-only file/metadata preflight; no solver and no output writes")
    args = parser.parse_args(argv)
    if args.check:
        checked = []
        for cid in CASE_IDS:
            _, _, result, last = input_preflight(cid)
            checked.append({"case_id": cid, "status": result["status"], "history_channels": len(last["loads"]), "solver_launched": False})
        print(json.dumps({"status": "input_preflight_passed", "scope": "two specified pairs only", "cases": checked}, indent=2))
        return 0
    outputs = [OUTPUT] + [HERE / ("final_pair_readback_" + cid + ".trn") for cid in CASE_IDS]
    if any(path.exists() for path in outputs):
        raise RuntimeError("refusing to overwrite previous read-back evidence")
    if solver_family():
        raise RuntimeError("another Fluent session is active")
    report = {"schema_version": 1, "status": "running", "scope": "two representative final case/data pairs only; not a restart verification of all nine cases",
              "case_ids": list(CASE_IDS), "processor_count": 1, "solver_sessions_parallel": 1,
              "tolerance": {"absolute": ABS_TOL, "relative": REL_TOL, "rule": "abs(actual-reference) <= max(1e-8, 1e-8*max(abs(actual),abs(reference)))",
                            "geometry_uses_same_rule_in_m": True, "flow_time_absolute_tolerance_s": FLOW_TIME_ABS_TOL_S},
              "initialization_called": False, "iterations_called": False, "mesh_changed": False,
              "physical_boundaries_axes_reference_points_changed": False, "input_files_written": False,
              "script_sha256": sha256(Path(__file__)), "cases": []}
    save_report(report)
    for cid in CASE_IDS:
        verify_one(cid, report)
        if solver_family():
            report["serial_execution_blocker"] = "session did not fully exit; no further session launched"
            break
    # A final cross-check ensures reading the second pair did not alter the first pair's inputs.
    report["final_input_sha_recheck"] = {}
    for row in report["cases"]:
        for name, item in row.get("input_hashes_before", {}).items():
            path = STAGE / item["path"]
            try:
                actual = sha256(path)
                report["final_input_sha_recheck"][item["path"]] = {"sha256": actual, "unchanged": actual == item["sha256"]}
            except OSError as exc:
                report["final_input_sha_recheck"][item["path"]] = {"sha256": None, "unchanged": False, "error": repr(exc)}
    report["all_inputs_unchanged_at_end"] = bool(report["final_input_sha_recheck"]) and all(item["unchanged"] for item in report["final_input_sha_recheck"].values())
    report["remaining_solver_processes_at_end"] = list(solver_family().values())
    report["physical_boundaries_axes_reference_points_changed"] = any(row.get("physical_settings_unchanged") is False for row in report["cases"])
    report["status"] = "passed_two_representative_pairs" if len(report["cases"]) == 2 and all(row["status"] == "passed" for row in report["cases"]) and report["all_inputs_unchanged_at_end"] and not report["remaining_solver_processes_at_end"] else "failed_readback_no_input_repairs"
    save_report(report)
    print(json.dumps({"status": report["status"], "scope": report["scope"],
                      "cases": [{"case_id": row["case_id"], "status": row["status"],
                                 "maximum_load_absolute_error": row.get("load_comparison", {}).get("maximum_absolute_error"),
                                 "maximum_error_to_tolerance_ratio": row.get("load_comparison", {}).get("maximum_error_to_tolerance_ratio"),
                                 "flow_time": row.get("flow_time"), "geometry": row.get("oscillation_geometry"),
                                 "error": row.get("error")} for row in report["cases"]]}, indent=2))
    return 0 if report["status"] == "passed_two_representative_pairs" else 1


if __name__ == "__main__":
    raise SystemExit(main())

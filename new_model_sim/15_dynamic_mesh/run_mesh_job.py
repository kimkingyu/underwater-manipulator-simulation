"""Run one generated Workbench mesh attempt with owned-process cleanup.

A normal exit is execution evidence only. Actual mesh statistics, exported files
and Fluent geometry/overset checks remain separate acceptance gates.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import subprocess
import time
from pathlib import Path

import psutil
import preflight
from runtime import AnsysTaskLock, OwnedProcessTree, RuntimeGateError, full_environment, write_json_new

STAGE = Path(__file__).resolve().parent


def validate_plan(attempt, contract):
    attempt = preflight.within(attempt, STAGE)
    plan_path = attempt / "mesh_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    journal = preflight.within(plan["journal"]["path"], attempt)
    if not journal.is_file() or preflight.sha256(journal) != plan["journal"]["sha256"]:
        raise RuntimeGateError("Generated Workbench journal changed")
    verification = plan.get("cad_verification")
    if not verification or preflight.sha256(verification["path"]) != verification["sha256"]:
        raise RuntimeGateError("Bound CAD verification receipt missing or changed")
    cad = json.loads(Path(verification["path"]).read_text(encoding="utf-8"))
    gate = cad.get("cad_domains", {})
    if (cad.get("mode") != "verify" or cad.get("state") != "passed" or cad.get("source_unchanged") is not True
            or gate.get("state") != "passed" or gate.get("independent_reopen_completed") is not True
            or gate.get("physical_face_ownership_checked") is not True):
        raise RuntimeGateError("Real independent CAD acceptance is required before meshing")
    if sorted(cad.get("artifacts", {})) != ["arm", "background"] or len(plan.get("cad", [])) != 2:
        raise RuntimeGateError("Two independently verified CAD artifacts are required")
    if len({str(Path(row["path"]).resolve()).lower() for row in plan["cad"]}) != 2:
        raise RuntimeGateError("Background and arm must be different CAD artifacts")
    for name, selected in zip(("background", "arm"), plan["cad"]):
        accepted = cad["artifacts"][name]
        if (Path(selected["path"]).resolve() != Path(accepted["path"]).resolve()
                or selected["sha256"] != accepted["sha256"] or preflight.sha256(selected["path"]) != accepted["sha256"]):
            raise RuntimeGateError("CAD input does not match independent acceptance: " + name)
    frozen = json.loads((attempt / "inputs.json").read_text(encoding="utf-8"))
    for entry in frozen.get("inputs", []) + frozen.get("sources", []):
        if preflight.sha256(entry["path"]) != entry["sha256"]:
            raise RuntimeGateError("Frozen meshing input/source changed: " + entry["path"])
    if not frozen.get("inputs") or not frozen.get("sources"):
        raise RuntimeGateError("Input/source manifest is incomplete")
    if len(plan.get("domains", [])) != 2:
        raise RuntimeGateError("This stage permits exactly two independent meshing systems")
    return attempt, plan, journal


def trace_phase(text):
    phase = None
    for line in text.splitlines():
        line = line.strip()
        if line in ("generate background", "generate arm"):
            phase = line.split()[1]
        elif line.startswith("export completed "):
            phase = None
    return phase


def observe_completion(path):
    """Do not wait for a modal-blocked launcher after the journal reports failure."""
    path = Path(path)
    if not path.is_file():
        return None
    try:
        receipt = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        # A concurrently written partial JSON is not an empty successful result.
        return None
    if not isinstance(receipt, dict):
        raise RuntimeGateError("Workbench completion receipt is not an object")
    if receipt.get("status") == "failed":
        detail = receipt.get("exception") or "journal reported failure"
        raise RuntimeGateError("Workbench failure receipt observed before launcher exit: " + str(detail))
    return receipt


def child_environment(attempt, contract):
    helper = STAGE.parent / "04_workbench/run_wb_fullenv.py"
    spec = importlib.util.spec_from_file_location("stage15_proven_wb_environment", helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Proven Windows logon-environment reconstruction, confined to this child.
    return full_environment(base=module.full_environment(), registry=(), attempt=attempt, contract=contract)


def run(attempt):
    contract = preflight.load_contract()
    attempt, plan, journal = validate_plan(attempt, contract)
    receipt = attempt / "workbench_execution.json"
    if receipt.exists() or (attempt / "workbench.trace.txt").exists() or (attempt / "workbench_completion.json").exists():
        raise RuntimeGateError("Refusing to rerun an already-started Workbench attempt")
    protected = json.loads((STAGE / "frozen/input_manifest.json").read_text(encoding="utf-8"))
    if preflight.verify_inputs(protected)["status"] != "passed":
        raise RuntimeGateError("Protected project input changed")
    environment = preflight.inspect_environment(contract)
    if environment["status"] != "passed":
        raise RuntimeGateError("Workbench preflight blocked: " + repr(environment["blockers"]))
    command = [contract["software"]["workbench"], "-I", "-X", "-R", str(journal)]
    report = {"status": "not_run", "valid_mesh": False, "command": command,
              "attempt": str(attempt), "script_sha256": preflight.sha256(__file__),
              "journal_sha256": plan["journal"]["sha256"], "solver_launched": False,
              "automatic_retries": 0, "system_timeout_seconds": contract["budget"]["mesh_system_timeout_s"],
              "total_timeout_seconds": 2 * contract["budget"]["mesh_system_timeout_s"],
              "preflight": environment}
    process = tree = None
    started = time.monotonic()
    phase, phase_start = None, started
    with AnsysTaskLock(attempt):
        try:
            env = child_environment(attempt, contract)
            with (attempt / "workbench.stdout.log").open("xb") as stdout, (attempt / "workbench.stderr.log").open("xb") as stderr:
                process = subprocess.Popen(command, cwd=str(attempt), env=env, stdout=stdout, stderr=stderr, shell=False)
                report["launcher_pid"] = process.pid
                tree = OwnedProcessTree(process, psutil)
                if tree.errors:
                    raise RuntimeGateError("Cannot establish process ownership: " + repr(tree.errors))
                while True:
                    tree.capture()
                    observe_completion(attempt / "workbench_completion.json")
                    now = time.monotonic()
                    trace = attempt / "workbench.trace.txt"
                    current_phase = trace_phase(trace.read_text(encoding="utf-8-sig", errors="replace")) if trace.exists() else None
                    if current_phase != phase:
                        phase, phase_start = current_phase, now
                    if now - started > report["total_timeout_seconds"] or (phase is not None and now - phase_start > report["system_timeout_seconds"]):
                        raise TimeoutError("Workbench exceeded the finite meshing budget")
                    if shutil.disk_usage(attempt).free < contract["budget"]["minimum_free_disk_gib"] * 1024 ** 3:
                        raise RuntimeGateError("Free disk fell below the configured budget")
                    try:
                        code = process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        continue
                    report["exit_code"] = code
                    break
            completion_path = attempt / "workbench_completion.json"
            if process.returncode != 0 or not completion_path.is_file():
                raise RuntimeGateError("Workbench did not provide a completed execution receipt")
            completion = json.loads(completion_path.read_text(encoding="utf-8-sig"))
            report["completion"] = completion
            if completion.get("status") != "completed_pending_mesh_inventory" or len(completion.get("systems", [])) != 2:
                raise RuntimeGateError("Workbench journal did not complete both meshing systems")
            validate_plan(attempt, contract)
            report["status"] = "completed_execution_pending_mesh_inventory"
        except Exception as exc:
            report["status"] = "failed"
            report["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            cleanup = True
            if tree is not None:
                try:
                    cleanup = tree.close()
                except Exception as exc:
                    cleanup = False
                    report["cleanup_error"] = type(exc).__name__ + ": " + str(exc)
                report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
                report["ownership_errors"] = tree.errors
            report["owned_cleanup_verified"] = cleanup
            if not cleanup:
                report["status"] = "failed"
            report["seconds"] = time.monotonic() - started
            report["protected_inputs_after"] = preflight.verify_inputs(protected)
            if report["protected_inputs_after"]["status"] != "passed":
                report["status"] = "failed"
            write_json_new(receipt, report)
    print(json.dumps({key: report.get(key) for key in ("status", "exit_code", "seconds", "owned_cleanup_verified", "error")}, ensure_ascii=False))
    return 0 if report["status"] == "completed_execution_pending_mesh_inventory" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", required=True, type=Path)
    raise SystemExit(run(parser.parse_args().attempt))

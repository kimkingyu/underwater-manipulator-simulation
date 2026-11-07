"""Run one two-domain Workbench attempt; execution is not mesh acceptance."""
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
    if [row.get("domain") for row in plan.get("domains", [])] != ["background", "arm"]:
        raise RuntimeGateError("This stage permits exactly the two independent meshing systems")
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
    """Do not wait for a modal-blocked launcher after a journal failure."""
    path = Path(path)
    if not path.is_file():
        return None
    try:
        receipt = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
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
    return full_environment(base=module.full_environment(), registry=(), attempt=attempt, contract=contract)


def run(attempt, *, total_deadline_unix=None, cancel_check=None, on_progress=None):
    # Import lazily: definition_probe imports this module for the proven child env.
    from definition_probe import bounded_call, UncertainCall, ProbeCancelled
    started = time.monotonic()
    total_deadline = time.time() + 7200 if total_deadline_unix is None else float(total_deadline_unix)
    # Both wall-clock and monotonic bounds apply; wall-clock rollback cannot extend a job.
    end = started + max(0, total_deadline - time.time())
    work_end = end - 30
    def remaining(cleanup=False):
        return max(0, min((end if cleanup else work_end) - time.monotonic(),
                          total_deadline - (0 if cleanup else 30) - time.time()))
    def call(function, maximum=10, *, cleanup=False):
        return bounded_call(function, min(maximum, remaining(cleanup)))
    def check():
        if cancel_check is not None and cancel_check():
            raise ProbeCancelled("mesh construction cancellation requested")
        if remaining() <= 0:
            raise TimeoutError("Workbench work deadline reached; cleanup reserve starts")
    def progress(**values):
        if on_progress is not None:
            try:
                on_progress(dict(values, owned_cleanup_verified=False))
            except Exception as exc:
                # A broken outer progress sink must never skip native process cleanup.
                report['progress_error'] = type(exc).__name__ + ': ' + str(exc)
    check()
    contract = call(preflight.load_contract)
    attempt, plan, journal = call(lambda: validate_plan(attempt, contract))
    receipt = attempt / "workbench_execution.json"
    if receipt.exists() or (attempt / "workbench.trace.txt").exists() or (attempt / "workbench_completion.json").exists():
        raise RuntimeGateError("Refusing to rerun an already-started Workbench attempt")
    protected = call(lambda: json.loads((STAGE / "frozen/input_manifest.json").read_text(encoding="utf-8")))
    if call(lambda: preflight.verify_inputs(protected))["status"] != "passed":
        raise RuntimeGateError("Protected project input changed")
    environment = call(lambda: preflight.inspect_environment(contract), 25)
    if environment["status"] != "passed":
        raise RuntimeGateError("Workbench preflight blocked: " + repr(environment["blockers"]))
    command = [contract["software"]["workbench"], "-I", "-X", "-R", str(journal)]
    report = {"status": "not_run", "valid_mesh": False, "command": command,
              "attempt": str(attempt), "script_sha256": preflight.sha256(__file__),
              "journal_sha256": plan["journal"]["sha256"], "solver_launched": False,
              "automatic_retries": 0, "system_timeout_seconds": min(3600, contract["budget"]["mesh_system_timeout_s"]),
              "total_timeout_seconds": 7200, "total_deadline_unix": total_deadline,
              "work_deadline_unix": total_deadline - 30, "preflight": environment,
              "owned_cleanup_verified": False, "launch_attempted": False}
    process = tree = None
    launch_uncertain = False
    phase, phase_start = None, started
    with AnsysTaskLock(attempt) as lock:
        try:
            check()
            env = call(lambda: child_environment(attempt, contract))
            with (attempt / "workbench.stdout.log").open("xb") as stdout, (attempt / "workbench.stderr.log").open("xb") as stderr:
                # Any exception/timeout before ownership is captured is uncertain, not permission to retry.
                launch_uncertain = True
                report['launch_attempted'] = True
                progress(status='running', phase='launching', launch_attempted=True)
                process = call(lambda: subprocess.Popen(command, cwd=str(attempt), env=env,
                                                       stdout=stdout, stderr=stderr, shell=False))
                report["launcher_pid"] = process.pid
                tree = call(lambda: OwnedProcessTree(process, psutil), 3)
                if tree.errors or not tree.members:
                    raise RuntimeGateError("Cannot establish process ownership: " + repr(tree.errors))
                launch_uncertain = False
                progress(status="running", phase="workbench", owned_processes=[
                    {"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()])
                while True:
                    check()
                    call(tree.capture, 2)
                    call(lambda: observe_completion(attempt / "workbench_completion.json"), 2)
                    now = time.monotonic()
                    trace = attempt / "workbench.trace.txt"
                    def read_phase():
                        return trace_phase(trace.read_text(encoding="utf-8-sig", errors="replace")) if trace.exists() else None
                    current_phase = call(read_phase, 2)
                    if current_phase != phase:
                        phase, phase_start = current_phase, now
                    if phase is not None and now - phase_start >= report["system_timeout_seconds"]:
                        raise TimeoutError("Workbench exceeded the per-domain 3600-second budget")
                    if call(lambda: shutil.disk_usage(attempt).free, 2) < contract["budget"]["minimum_free_disk_gib"] * 1024 ** 3:
                        raise RuntimeGateError("Free disk fell below the configured budget")
                    progress(status="running", phase=phase or "workbench", owned_processes=[
                        {"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()])
                    try:
                        code = call(lambda: process.wait(timeout=min(0.5, remaining())), 1)
                    except subprocess.TimeoutExpired:
                        continue
                    report["exit_code"] = code
                    break
            completion = call(lambda: observe_completion(attempt / "workbench_completion.json"))
            if process.returncode != 0 or completion is None:
                raise RuntimeGateError("Workbench did not provide a completed execution receipt")
            report["completion"] = completion
            if (completion.get("status") != "completed_pending_mesh_inventory" or
                    [x.get("domain") for x in completion.get("systems", [])] != ["background", "arm"] or
                    completion.get("mesh_exports_written") is not True):
                raise RuntimeGateError("Workbench journal did not complete both mesh exports")
            call(lambda: validate_plan(attempt, contract))
            report["status"] = "completed_execution_pending_mesh_inventory"
        except Exception as exc:
            report["status"] = "cancelled" if isinstance(exc, ProbeCancelled) else "failed"
            report["error"] = type(exc).__name__ + ": " + str(exc)
            if isinstance(exc, UncertainCall):
                # In-flight native calls can still create/extend a process tree.
                launch_uncertain = True
        finally:
            progress(status="cleaning", phase="cleanup")
            cleanup = not launch_uncertain and process is None
            if tree is not None:
                try:
                    # Uncertainty does not erase verified PID/create-time ownership.
                    # Stop every known-owned member, but retain the lock because an
                    # in-flight call may still have extended the process tree.
                    known_owned_closed = call(tree.close, 20, cleanup=True)
                    cleanup = bool(known_owned_closed) and not launch_uncertain
                except Exception as exc:
                    cleanup = False
                    report["cleanup_error"] = type(exc).__name__ + ": " + str(exc)
            if tree is not None:
                report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
                report["ownership_errors"] = tree.errors
            report["owned_cleanup_verified"] = bool(cleanup)
            if not cleanup:
                lock.retain("meshing launch/cleanup ownership remains uncertain; no automatic retry")
                report["status"] = "cleanup_unverified"
                report["task_lock_retained"] = True
            report["protected_inputs_after"] = {"status": "unverified"}
            try:
                report["protected_inputs_after"] = call(lambda: preflight.verify_inputs(protected), 5, cleanup=True)
            except Exception as exc:
                report["reverification_error"] = type(exc).__name__ + ": " + str(exc)
            if report["protected_inputs_after"]["status"] != "passed" and report["status"] != "cleanup_unverified":
                report["status"] = "failed"
            report["seconds"] = time.monotonic() - started
            write_json_new(receipt, report)
    print(json.dumps({key: report.get(key) for key in ("status", "exit_code", "seconds", "owned_cleanup_verified", "error")}, ensure_ascii=False))
    return 0 if report["status"] == "completed_execution_pending_mesh_inventory" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", required=True, type=Path)
    raise SystemExit(run(parser.parse_args().attempt))

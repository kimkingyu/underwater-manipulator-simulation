"""Native implementation of the separately authorized three-slot batch002.

Import and --preflight never launch Fluent. Only fixed slots are accepted. No
free-form journal, Scheme, Python, paths or budget parameters enter from MCP.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

sys.dont_write_bytecode = True
import psutil
from definition_probe import bounded_call, UncertainCall, checked_path, validate_frozen, write_state, write_new
from runtime import AnsysTaskLock, OwnedProcessTree, RuntimeGateError
import run_mesh_job
from mcp_bridge import STAGE, read_json, sha256
from mesh_inspection_connection import safe_connection_error
from mesh_inspection_evidence import finite, parse_native_output, validate_native_evidence
from inspection_batch002_connection import connect_ready, readiness_passed
from inspection_batch002_bridge import Batch002Slot, BATCH, SLOTS, MESHES, EXECUTION_SLOTS, execution_slot, SUPPLEMENT, POLICY

NATIVE_APIS = ["one Popen fixed fluent.exe 3ddp -g -t1 -sifile=<fixed attempt>/server.info -nm",
    "private native stdin=PIPE kept open without writes through connection/inspection; close only in controlled final cleanup",
    "child-only REMOTING_SERVER_ADDRESS=127.0.0.1", "one grpc.insecure_channel on exact 127.0.0.1",
    "grpc.channel_ready_future(same_channel).result(timeout=min(20,original_45s_remaining)) once",
    "HealthCheckService.check_health same-channel bounded polling; only completed UNAVAILABLE may wait; auth/hung calls abort",
    "FluentConnection(channel=same,password=private,cleanup_on_exit=False,inside_container=False)",
    "fixed official read-only (cx-solver-mode?) and (cx-version) queries", "Solver(fluent_connection=same)",
    "inspection slots only: file.read_mesh; mesh.quality; native fluid/wall queries; wall.adjacent_cell_zone; list_zones; mesh.check",
    "file.start_transcript; file.stop_transcript; session.exit"]


def slot_for_attempt(attempt, root=STAGE):
    path = checked_path(attempt, root)
    for name in EXECUTION_SLOTS:
        slot = execution_slot(name, root)
        if path == slot.safe(slot.ATTEMPT):
            return slot, path
    raise RuntimeGateError("Only pre-reserved fixed batch002 attempts are permitted")


def validate_request(slot, attempt):
    if slot.slot == POLICY["stdin_validation_slot"] and POLICY["stdin_validation_retired"]:
        raise RuntimeGateError("Consumed stdin validation is retired; no new native allowance")
    value = read_json(attempt / "request.json", 65536)
    keys = {"schema_version", "operation", "job_id", "created_unix", "deadline_unix", "work_deadline_unix",
            "authorization", "sources", "batch_id", "slot", "mesh"}
    if (set(value) != keys or type(value.get("schema_version")) is not int or value["schema_version"] != 1 or
            value.get("operation") != slot.OPERATION or value.get("job_id") != slot.JOB_ID or
            value.get("batch_id") != BATCH or value.get("slot") != slot.slot or
            value.get("mesh") != (None if slot.is_connection else MESHES[slot.slot])):
        raise RuntimeGateError("Fixed batch002 request identity mismatch")
    if not all(finite(value.get(k)) for k in ("created_unix", "deadline_unix", "work_deadline_unix")):
        raise RuntimeGateError("Nonfinite batch002 deadline")
    created = value["created_unix"]
    if (created > time.time() + 1 or value["deadline_unix"] != created + 300 or
            value["work_deadline_unix"] != created + 270 or value["deadline_unix"] <= time.time()):
        raise RuntimeGateError("Invalid/expired batch002 300-second total budget")
    if (value["sources"] != slot._sources() or value["authorization"] != {
            "path": str(slot.safe(slot.AUTHORIZATION)), "sha256": sha256(slot.safe(slot.AUTHORIZATION))}):
        raise RuntimeGateError("Batch002 request sources/grant changed")
    return value


def validate_launcher(slot, attempt, remaining):
    end = time.monotonic() + min(20, remaining())
    while time.monotonic() < end:
        if (attempt / "launcher.json").exists():
            row = read_json(attempt / "launcher.json", 8192)
            if (row.get("job_id") != slot.JOB_ID or row.get("pid") != os.getpid() or
                    row.get("creation_time") != psutil.Process(os.getpid()).create_time()):
                raise RuntimeGateError("Batch002 worker launcher PID/create-time mismatch")
            return
        time.sleep(min(0.05, max(0, end - time.monotonic())))
    raise RuntimeGateError("Batch002 launcher ledger missing")


def collect_mesh(solver, slot, attempt, report):
    """No connection-slot path can enter this collector."""
    if slot.slot not in MESHES:
        raise RuntimeGateError("Connection probe cannot read a mesh")
    mesh = {"provenance": "live_fluent_read_only", "commands": {}, "errors": {},
            "fluid_zones": [], "wall_adjacency": {}, "metrics": {}}
    report["mesh_evidence"] = mesh
    def step(name, function):
        try:
            result = function()
            mesh["commands"][name] = "completed"
            return result
        except Exception as exc:
            mesh["commands"][name] = "failed"
            mesh["errors"][name] = safe_connection_error(exc)
    step("read_mesh", lambda: solver.file.read_mesh(file_name=str(slot.safe(MESHES[slot.slot]["path"]))))
    if mesh["commands"].get("read_mesh") != "completed":
        raise RuntimeGateError("Batch002 read_mesh failed")
    step("quality", lambda: solver.tui.mesh.quality())
    fluids = step("fluid_zones", lambda: solver.setup.cell_zone_conditions.fluid.get_object_names())
    if isinstance(fluids, list) and len(fluids) <= 100 and all(isinstance(x, str) and len(x) <= 256 for x in fluids):
        mesh["fluid_zones"] = fluids
    names = step("wall_zones", lambda: solver.setup.boundary_conditions.wall.get_object_names())
    if not isinstance(names, list) or len(names) > 100:
        names = []
    for name in MESHES[slot.slot]["walls"]:
        if name not in names:
            mesh["wall_adjacency"][name] = {"native_query_completed": False}
            continue
        key = "adjacent_cell_zone:" + name
        value = step(key, lambda name=name: solver.setup.boundary_conditions.wall[name].adjacent_cell_zone())
        mesh["wall_adjacency"][name] = {"native_query_completed": mesh["commands"].get(key) == "completed",
            "adjacent_cell_zone": value if isinstance(value, str) and len(value) <= 256 else None}
    step("list_zones", lambda: solver.tui.mesh.modify_zones.list_zones())
    step("mesh_check", lambda: solver.tui.mesh.check())


class NativeInputLease:
    """Own only the private empty stdin writer; never write/flush/communicate.

    Popen may finish after bounded_call has timed out. Mark closure requested in
    finally; a late returned writer is closed immediately, without killing or
    adopting an unverified process. Outer worker still preserves ownership locks.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._writer = None
        self._closing = False
        self._closed = False
        self._late_return = False

    def adopt(self, process):
        with self._lock:
            self._writer = process.stdin
            if self._writer is None or getattr(self._writer, "closed", False) is True:
                raise RuntimeGateError("Native private stdin pipe was absent or already closed")
            if self._closing:
                self._late_return = True
                self._writer.close()
                self._closed = True
        return process

    def close(self):
        with self._lock:
            self._closing = True
            if self._writer is not None and not self._closed:
                self._writer.close()
                self._closed = True

    def evidence(self):
        with self._lock:
            return {"mode": "private_subprocess_PIPE", "bytes_written": 0,
                    "close_requested_in_cleanup": self._closing, "pipe_returned": self._writer is not None,
                    "writer_closed": self._closed, "late_return_after_close_request": self._late_return}


def fixed_transcript_target(slot, attempt):
    """Fresh fixed file only; no caller string, reparse path or TUI quoting guess."""
    expected = slot.safe(slot.ATTEMPT)
    if checked_path(attempt, slot.root) != expected:
        raise RuntimeGateError("Transcript must belong to this exact fixed attempt")
    target = slot.safe(slot.ATTEMPT + "/native.transcript.txt")
    if target.exists():
        raise RuntimeGateError("Refusing existing transcript file or directory; never overwrite")
    text = target.as_posix()
    # Current fixed roots have no spaces. Fail closed instead of introducing a
    # new quoting rule for whitespace, quotes, control characters or TUI syntax.
    if not target.is_absolute() or not re.fullmatch(r"(?:[A-Za-z]:/|/)[A-Za-z0-9_./-]+", text):
        raise RuntimeGateError("Fixed transcript needs a safe absolute forward-slash path without quoting")
    return target, text


def transcript_call(solver, action, slot, attempt, report):
    """Record safe per-call checkpoints; preserve the exact existing two TUI APIs."""
    if action not in ("start", "stop"):
        raise RuntimeGateError("Only fixed transcript start/stop actions are allowed")
    calls = report.setdefault("transcript_calls", {})
    state = {"stage": "attempted", "attempted": True, "completed": False, "failed": False}
    calls[action] = state
    write_new(attempt / ("transcript_" + action + "_attempted.json"), {
        "job_id": slot.JOB_ID, "action": action, "stage": "attempted"}, attempt)
    try:
        if action == "start":
            _, value = fixed_transcript_target(slot, attempt)
            solver.tui.file.start_transcript(value)
        else:
            if calls.get("start", {}).get("stage") != "completed":
                raise RuntimeGateError("Cannot stop an uncompleted transcript start")
            solver.tui.file.stop_transcript()
        state.update(stage="completed", completed=True)
        write_new(attempt / ("transcript_" + action + "_completed.json"), {
            "job_id": slot.JOB_ID, "action": action, "stage": "completed"}, attempt)
    except Exception as exc:
        state.update(stage="failed", failed=True, completed=False, safe_error=safe_connection_error(exc))
        write_new(attempt / ("transcript_" + action + "_failed.json"), {
            "job_id": slot.JOB_ID, "action": action, "stage": "failed", "safe_error": state["safe_error"]}, attempt)
        raise


def native_collect(attempt, *, stage_root=STAGE):
    slot, attempt = slot_for_attempt(attempt, stage_root)
    if slot.slot == "connection" or slot.slot == SUPPLEMENT and POLICY["supplement_connection_retired"]:
        raise RuntimeGateError("Consumed connection worker permanently retired; no new native allowance")
    request = validate_request(slot, attempt)
    slot.authorize()
    if not slot.is_connection:
        slot.connection_gate()
    parent = psutil.Process(os.getppid())
    owner = read_json(attempt / "launcher.json", 8192)
    lock = read_json(slot.safe(".ansys-task.lock"), 8192)
    if (owner.get("pid") != parent.pid or owner.get("creation_time") != parent.create_time() or
            owner.get("job_id") != slot.JOB_ID or lock.get("pid") != parent.pid or lock.get("attempt") != str(attempt)):
        raise RuntimeGateError("Batch002 native adapter requires exact parent/lock ownership")
    if os.environ.get("REMOTING_SERVER_ADDRESS") != "127.0.0.1":
        raise RuntimeGateError("Batch002 child loopback environment binding missing")
    write_new(attempt / "native_claim.json", {"job_id": slot.JOB_ID, "pid": os.getpid(),
        "creation_time": psutil.Process().create_time(), "parent_pid": parent.pid,
        "parent_creation_time": parent.create_time()}, attempt)
    report = {"job_id": slot.JOB_ID, "batch_id": BATCH, "slot": slot.slot, "provenance": "live_fluent_batch002",
        "commands": {}, "errors": {}, "launch_uncertain": True, "mesh_generated": False, "flow_iterations": 0,
        "readiness": {}}
    solver = None
    native_input = NativeInputLease()
    transcript = attempt / "native.transcript.txt"
    try:
        # Refuse stale/redirected transcript artifacts before starting Fluent;
        # revalidate immediately before the existing TUI start call as well.
        transcript, _ = fixed_transcript_target(slot, attempt)
        frozen = validate_frozen(stage_root)
        executable = checked_path(frozen["contract"]["software"]["fluent"], Path(frozen["contract"]["software"]["ansys_root"]))
        server = attempt / "server.info"
        with server.open("xb"):
            pass
        command = [str(executable), "3ddp", "-g", "-t1", "-sifile=" + str(server), "-nm"]
        report["native_command"] = command
        end = time.monotonic() + min(45, request["work_deadline_unix"] - time.time())
        write_new(attempt / "fluent_launch_started.json", {"job_id": slot.JOB_ID, "startup_seconds": 45}, attempt)
        def launch_with_private_input():
            # Do not inherit the adapter's DEVNULL input and do not use a Popen
            # context manager (which would close this writer before connection).
            process = subprocess.Popen(command, cwd=str(attempt), stdin=subprocess.PIPE, shell=False)
            return native_input.adopt(process)
        process = bounded_call(launch_with_private_input, min(10, end - time.monotonic()))
        identity = psutil.Process(process.pid)
        write_new(attempt / "fluent_process.json", {"job_id": slot.JOB_ID, "pid": process.pid,
            "creation_time": identity.create_time(), "parent_pid": identity.ppid(), "executable": identity.exe()}, attempt)
        solver = connect_ready(attempt, process, end, report["readiness"])
        report["launch_uncertain"] = False
        write_new(attempt / "fluent_launch_ready.json", {"job_id": slot.JOB_ID, "health": "SERVING", "solver_mode": True}, attempt)
        # Connection probe starts transcript only after authenticated connection;
        # its structured native readiness includes direct health/mode/version.
        transcript_call(solver, "start", slot, attempt, report)
        if not slot.is_connection:
            collect_mesh(solver, slot, attempt, report)
        transcript_call(solver, "stop", slot, attempt, report)
        if transcript.is_file() and transcript.stat().st_size <= 2 * 1024 * 1024:
            report["transcript_sha256"] = sha256(transcript)
            if slot.slot in MESHES:
                report["mesh_evidence"]["metrics"] = parse_native_output(transcript.read_text(encoding="utf-8", errors="replace"))
    except Exception as exc:
        report["errors"]["native_adapter"] = safe_connection_error(exc)
    finally:
        # Connection/inspection has ended. Keep the empty input writer alive
        # through the bounded session shutdown request, then explicitly close
        # only this owned handle, including every failure/timeout path.
        try:
            if solver is not None:
                report["stdin_close_phase"] = "after_bounded_session_exit_request"
                try:
                    bounded_call(solver.exit, min(2, max(0, request["deadline_unix"] - time.time())))
                except Exception as exc:
                    report["errors"]["session_exit"] = safe_connection_error(exc)
            else:
                report["stdin_close_phase"] = "native_failure_final_cleanup"
        finally:
            try:
                native_input.close()
            except Exception as exc:
                report["errors"]["stdin_close"] = safe_connection_error(exc)
            report["stdin_lifecycle"] = native_input.evidence()
            write_new(attempt / "readiness.json", report["readiness"], attempt)
            report["readiness_sha256"] = sha256(attempt / "readiness.json")
            if slot.slot in MESHES:
                report["inspection"] = validate_native_evidence(report.get("mesh_evidence", {}), slot.slot)
            write_new(attempt / "native_result.json", report, attempt)
    return report


def run(attempt, *, stage_root=STAGE):
    slot, attempt = slot_for_attempt(attempt, stage_root)
    if (slot.slot == "connection" or slot.slot == SUPPLEMENT and POLICY["supplement_connection_retired"] or
            (attempt / "probe_result.json").exists()):
        raise RuntimeGateError("Batch002 slot consumed or retired; never rerun")
    write_new(attempt / "worker_claim.json", {"job_id": slot.JOB_ID, "pid": os.getpid()}, attempt)
    began = time.monotonic()
    report = {"job_id": slot.JOB_ID, "batch_id": BATCH, "slot": slot.slot, "status": "running",
        "phase": "validating", "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False,
        "owned_cleanup_verified": True, "inputs_reverified_after_cleanup": False, "sources_reverified_after_cleanup": False,
        "solver_launched": False}
    request = frozen = process = tree = lock = None
    uncertain = False
    launcher_verified = False
    identities = {}
    deadline, work_deadline = began + 300, began + 270
    def remaining():
        return max(0, min(deadline - time.monotonic(), request["deadline_unix"] - time.time() if request else 300))
    def working():
        if (attempt / "cancel_request.json").exists():
            cancel = read_json(attempt / "cancel_request.json", 4096)
            if cancel.get("job_id") != slot.JOB_ID or cancel.get("operation") != slot.CANCEL_OPERATION:
                raise RuntimeGateError("Batch002 cancellation identity mismatch")
            raise InterruptedError("Batch002 cancellation requested")
        if time.monotonic() >= work_deadline or request and time.time() >= request["work_deadline_unix"]:
            raise TimeoutError("270-second work deadline reached; owned cleanup now")
    def state(phase):
        report.update(phase=phase, elapsed_seconds=max(time.monotonic() - began, 300 - remaining()))
        write_state(attempt, report)
    def recheck():
        slot.authorize()
        validate_request(slot, attempt)
        if not slot.is_connection:
            report["connection_gate"] = slot.connection_gate()
        return validate_frozen(stage_root)
    def capture():
        tree.capture()
        if len(tree.members) > 256:
            raise RuntimeGateError("Unexpected owned process tree size")
        for pid, stamp in tree.members.items():
            if pid not in identities:
                try:
                    item = psutil.Process(pid)
                    if item.create_time() == stamp:
                        identities[pid] = {"pid": pid, "creation_time": stamp, "name": item.name(), "executable": item.exe()}
                except psutil.NoSuchProcess:
                    pass
    try:
        request = validate_request(slot, attempt)
        deadline = began + max(0, request["deadline_unix"] - time.time())
        work_deadline = deadline - 30
        report.update({k: request[k] for k in ("created_unix", "deadline_unix", "work_deadline_unix")})
        validate_launcher(slot, attempt, remaining)
        launcher_verified = True
        dispatch_path = slot.safe(".mesh-inspection-dispatch.lock")
        dispatch = read_json(dispatch_path, 8192)
        if dispatch.get("job_id") != slot.JOB_ID or not isinstance(dispatch.get("token"), str):
            raise RuntimeGateError("Batch002 serial dispatch ownership missing")
        dispatch_hash = sha256(dispatch_path)
        working()
        frozen = bounded_call(recheck, min(15, remaining() - 30))
        report["protected_inputs_before"] = frozen["protected_inputs"]
        lock = AnsysTaskLock(attempt, stage_root=stage_root)
        lock.__enter__()
        write_new(attempt / "command_audit.json", {"job_id": slot.JOB_ID, "batch_id": BATCH,
            "slot": slot.slot, "fixed_native_apis": NATIVE_APIS, "native_start_timeout": 45,
            "total_seconds": 300, "cleanup_begins_seconds": 270, "free_form_commands_accepted": False}, attempt)
        env = bounded_call(lambda: run_mesh_job.child_environment(attempt, frozen["contract"]), min(10, remaining() - 30))
        for key in list(env):
            if key.upper().startswith(("PYFLUENT", "PYTHON")):
                env.pop(key)
        env.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUTF8="1", PYTHONIOENCODING="utf-8",
                   AWP_ROOT242=frozen["contract"]["software"]["ansys_root"], REMOTING_SERVER_ADDRESS="127.0.0.1")
        command = [slot._python(), "-B", str(slot.safe(slot.WORKER_SCRIPT)), "--native", "--attempt", str(attempt)]
        bounded_call(recheck, min(15, remaining() - 30))
        working()
        state("starting_native_adapter")
        with (attempt / "native.stdout.log").open("xb") as out, (attempt / "native.stderr.log").open("xb") as err:
            try:
                process = bounded_call(lambda: subprocess.Popen(command, cwd=str(attempt), env=env,
                    stdin=subprocess.DEVNULL, stdout=out, stderr=err, shell=False), min(10, remaining() - 30))
                tree = bounded_call(lambda: OwnedProcessTree(process, psutil), min(3, remaining() - 30))
            except Exception:
                uncertain = True
                raise
            if tree.errors:
                uncertain = True
                raise RuntimeGateError("Batch002 process ownership unverified")
            while True:
                working()
                bounded_call(capture, min(2, remaining() - 30))
                code = bounded_call(process.poll, min(2, remaining() - 30))
                if code is not None:
                    report["native_adapter_exit_code"] = code
                    break
                time.sleep(min(0.1, max(0, work_deadline - time.monotonic())))
        native = read_json(attempt / "native_result.json")
        report["native"] = native
        if (native.get("job_id") != slot.JOB_ID or native.get("batch_id") != BATCH or native.get("slot") != slot.slot or
                native.get("provenance") != "live_fluent_batch002" or native.get("mesh_generated") is not False or
                type(native.get("flow_iterations")) is not int or native["flow_iterations"] != 0):
            raise RuntimeGateError("Batch002 native scope/provenance mismatch")
        if native.get("launch_uncertain") is not False:
            uncertain = True
            raise RuntimeGateError("Batch002 authenticated readiness uncertain; preserve lock")
        readiness = read_json(attempt / "readiness.json")
        if (not readiness_passed(readiness) or readiness != native.get("readiness") or
                native.get("readiness_sha256") != sha256(attempt / "readiness.json")):
            raise RuntimeGateError("Batch002 native readiness artifact mismatch")
        if slot.is_connection and native.get("commands") != {}:
            raise RuntimeGateError("Connection probe must never inspect a mesh")
        transcript = attempt / "native.transcript.txt"
        if (not transcript.is_file() or transcript.stat().st_size > 2 * 1024 * 1024 or
                native.get("transcript_sha256") != sha256(transcript)):
            raise RuntimeGateError("Batch002 native transcript binding failed")
        if slot.slot in MESHES and native.get("mesh_evidence", {}).get("metrics") != parse_native_output(
                transcript.read_text(encoding="utf-8", errors="replace")):
            raise RuntimeGateError("Batch002 native mesh aggregate binding failed")
        report["status"] = "completed" if code == 0 and not native.get("errors") else "failed"
    except InterruptedError as exc:
        report.update(status="cancelled", error=str(exc))
    except Exception as exc:
        report.update(status="failed", error=type(exc).__name__ + ": fixed batch002 worker operation failed",
                      safe_error_classification=safe_connection_error(exc))
    finally:
        try:
            state("cleanup")
        except Exception:
            report["state_write_failed"] = True
        report["solver_launch_attempted"] = (attempt / "fluent_launch_started.json").exists()
        report["solver_launched"] = (attempt / "fluent_process.json").exists()
        report["owned_process_identity"] = list(identities.values())
        if report["solver_launch_attempted"] and not (attempt / "fluent_launch_ready.json").exists():
            uncertain = True
        if report["solver_launched"]:
            prefix = str(Path(frozen["contract"]["software"]["ansys_root"]) / "fluent").replace("\\", "/").casefold() + "/" if frozen else ""
            if not prefix or not any(row["executable"].replace("\\", "/").casefold().startswith(prefix) for row in identities.values()):
                uncertain = True
                report["native_identity_unverified"] = True
        cleanup = not uncertain
        if tree is not None:
            try:
                if remaining() < 15:
                    raise UncertainCall("Insufficient owned cleanup reserve")
                cleanup = bool(bounded_call(tree.close, min(20, remaining() - 1))) and cleanup
            except Exception as exc:
                cleanup = False
                report["cleanup_error"] = safe_connection_error(exc)
            report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
            report["ownership_errors"] = list(tree.errors)
        elif process is not None:
            cleanup = False
        if request is not None:
            try:
                after = bounded_call(recheck, min(10, remaining() - 1))
                report["protected_inputs_after"] = after["protected_inputs"]
                report.update(inputs_reverified_after_cleanup=True, sources_reverified_after_cleanup=True)
            except Exception as exc:
                report.update(status="failed", recheck_error=safe_connection_error(exc))
        if lock is not None and lock.acquired:
            if not cleanup:
                lock.retain("Batch002 launch/ownership/cleanup unverified; preserve for operator review")
            try:
                lock.__exit__(None, None, None)
            except Exception:
                cleanup = False
                lock.retain("Batch002 lock identity changed")
        if cleanup and launcher_verified and "dispatch_hash" in locals():
            try:
                if sha256(dispatch_path) != dispatch_hash or read_json(dispatch_path).get("job_id") != slot.JOB_ID:
                    raise RuntimeGateError("Batch002 dispatch lock changed; never remove unknown lock")
                dispatch_path.unlink()
                report["serial_dispatch_released"] = True
            except Exception:
                cleanup = False
                report["serial_dispatch_released"] = False
        if not cleanup:
            report["status"] = "cleanup_unverified"
        if remaining() <= 0:
            report.update(status="cleanup_unverified" if not cleanup else "failed", deadline_exceeded=True)
        report.update(owned_cleanup_verified=cleanup, phase="finished",
            elapsed_seconds=max(time.monotonic() - began, 300 - remaining()))
        write_state(attempt, report)
        write_new(attempt / "probe_result.json", report, attempt)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--preflight", action="store_true")
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--native", action="store_true")
    parser.add_argument("--slot", choices=EXECUTION_SLOTS)
    parser.add_argument("--attempt", type=Path)
    args = parser.parse_args(argv)
    if args.preflight:
        if args.slot not in EXECUTION_SLOTS or args.attempt is not None:
            parser.error("Preflight accepts fixed slot only")
        slot = execution_slot(args.slot)
        slot.authorize()
        frozen = validate_frozen()
        if not Path(frozen["contract"]["software"]["fluent"]).is_file():
            raise RuntimeGateError("Fixed Fluent executable missing")
        report = {"status": "passed", "batch_id": BATCH, "slot": args.slot, "job_id": slot.JOB_ID,
            "slot_reserved": slot.safe(slot.ATTEMPT).exists(), "connection_gate_required": not slot.is_connection,
            "maximum_seconds_including_cleanup": 300, "CFD_authorized": False}
    else:
        if args.attempt is None or args.slot is not None:
            parser.error("Execution requires only fixed attempt")
        report = native_collect(args.attempt) if args.native else run(args.attempt)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if args.native or report.get("status") in ("passed", "completed") else 1


if __name__ == "__main__":
    raise SystemExit(main())

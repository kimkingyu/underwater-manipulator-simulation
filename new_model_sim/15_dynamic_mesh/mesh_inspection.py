"""Fixed read-only Fluent inspection worker. Import/preflight never launches ANSYS.

Native APIs are fixed below, not supplied by MCP callers. The supervisor owns a
separate Python child and every verified descendant; launch uncertainty retains
its shared lock. No journal/Scheme/Python expression is accepted from a caller.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import socket
import sys
import time

sys.dont_write_bytecode = True
import psutil
from definition_probe import (bounded_call, UncertainCall, checked_path, validate_frozen,
                              write_state, write_new)
from runtime import AnsysTaskLock, OwnedProcessTree, RuntimeGateError
import run_mesh_job
from mcp_bridge import STAGE, read_json, sha256
from mesh_inspection_mcp_bridge import InspectionSlot, MESHES, SOURCES
from mesh_inspection_connection import wait_private_credentials, safe_connection_error

NATIVE_APIS = (
    "one shell=False Popen: fluent.exe 3ddp -g -t1 -sifile=<fixed-slot>/server.info -nm; startup<=45s; no retry",
    "strict stable private server.info snapshot -> memory-only local ip/port/password",
    "pyfluent.connect_to_fluent(ip=<private snapshot>,port=<private snapshot>,password=<memory only>,cleanup_on_exit=False,start_watchdog=False,start_transcript=False)",
    "solver.tui.file.start_transcript", "solver.file.read_mesh", "solver.tui.mesh.quality",
    "solver.setup.cell_zone_conditions.fluid.get_object_names",
    "solver.setup.boundary_conditions.wall.get_object_names",
    "solver.setup.boundary_conditions.wall[name].adjacent_cell_zone",
    "solver.tui.mesh.modify_zones.list_zones", "solver.tui.mesh.check",
    "solver.tui.file.stop_transcript", "solver.exit",
)
from mesh_inspection_evidence import finite, parse_native_output, validate_native_evidence


def slot_for_attempt(attempt, root=STAGE):
    path = checked_path(attempt, root)
    for domain in MESHES:
        slot = InspectionSlot(domain, root)
        if path == slot.safe(slot.ATTEMPT):
            return slot, path
    raise RuntimeGateError("Only the fixed pre-reserved inspection attempts are allowed")


def validate_request(slot, attempt):
    request = read_json(attempt / "request.json", 65536)
    keys = {"schema_version", "operation", "job_id", "created_unix", "deadline_unix", "work_deadline_unix",
            "authorization", "sources", "domain", "mesh"}
    if (set(request) != keys or type(request.get("schema_version")) is not int or request["schema_version"] != 1 or
            request.get("operation") != "mesh_inspection" or request.get("job_id") != slot.JOB_ID or
            request.get("domain") != slot.domain or request.get("mesh") != MESHES[slot.domain]):
        raise RuntimeGateError("Fixed inspection request mismatch")
    if not all(finite(request.get(key)) for key in ("created_unix", "deadline_unix", "work_deadline_unix")):
        raise RuntimeGateError("Nonfinite request deadline")
    created = request["created_unix"]
    if (request["deadline_unix"] != created + 300 or request["work_deadline_unix"] != created + 270 or
            created > time.time() + 1 or request["deadline_unix"] <= time.time()):
        raise RuntimeGateError("Expired/invalid 300-second total budget")
    if request["sources"] != slot._sources():
        raise RuntimeGateError("Request source hashes changed")
    if request["authorization"] != {"path": str(slot.safe(slot.AUTHORIZATION)), "sha256": sha256(slot.safe(slot.AUTHORIZATION))}:
        raise RuntimeGateError("Request authorization hash changed")
    return request


def validate_launcher(slot, attempt, remaining):
    end = time.monotonic() + min(20, remaining())
    while time.monotonic() < end:
        if (attempt / "launcher.json").exists():
            row = read_json(attempt / "launcher.json", 8192)
            if (row.get("job_id") != slot.JOB_ID or row.get("pid") != os.getpid() or
                    row.get("creation_time") != psutil.Process(os.getpid()).create_time()):
                raise RuntimeGateError("Worker PID/create-time launcher mismatch")
            return
        time.sleep(min(0.05, max(0, end - time.monotonic())))
    raise RuntimeGateError("No verified worker launcher ledger")


def native_collect(attempt, *, stage_root=STAGE):
    """Child-only fixed API adapter; supervisor enforces the outer time budget."""
    slot, attempt = slot_for_attempt(attempt, stage_root)
    request = validate_request(slot, attempt)
    slot.authorize()
    parent = psutil.Process(os.getppid())
    launcher = read_json(attempt / "launcher.json", 8192)
    lock = read_json(slot.safe(".ansys-task.lock"), 8192)
    if (launcher.get("pid") != parent.pid or launcher.get("creation_time") != parent.create_time() or
            launcher.get("job_id") != slot.JOB_ID or lock.get("pid") != parent.pid or lock.get("attempt") != str(attempt)):
        raise RuntimeGateError("Native adapter requires verified owning worker and task lock")
    if time.time() >= request["work_deadline_unix"]:
        raise TimeoutError("No work budget remains")
    write_new(attempt / "native_claim.json", {"pid": os.getpid(), "creation_time": psutil.Process().create_time(),
              "parent_pid": parent.pid, "parent_creation_time": parent.create_time(), "job_id": slot.JOB_ID}, attempt)
    import ansys.fluent.core as pyfluent
    report = {"job_id": slot.JOB_ID, "domain": slot.domain, "provenance": "live_fluent_read_only",
              "commands": {}, "errors": {}, "fluid_zones": [], "wall_adjacency": {},
              "metrics": {}, "launch_uncertain": True, "flow_iterations": 0, "mesh_generated": False}
    solver = None
    transcript = attempt / "native.transcript.txt"
    def step(name, function):
        if time.time() >= request["work_deadline_unix"]:
            raise TimeoutError("270-second work deadline reached")
        try:
            value = function()
            report["commands"][name] = "completed"
            return value
        except Exception as exc:
            report["commands"][name] = "failed"
            report["errors"][name] = (type(exc).__name__ + ": " + str(exc))[:4000]
            return None
    try:
        # PyFluent 242's Windows launch_fluent retries Popen after TimeoutError.
        # Do NOT use it. Exactly one direct, owned process launch is permitted.
        frozen = validate_frozen(stage_root)
        fluent_exe = checked_path(frozen["contract"]["software"]["fluent"], Path(frozen["contract"]["software"]["ansys_root"]))
        server_info = attempt / "server.info"
        with server_info.open("xb"):
            pass
        native_command = [str(fluent_exe), "3ddp", "-g", "-t1", "-sifile=" + str(server_info), "-nm"]
        report["fluent_command"] = native_command
        launch_end = time.monotonic() + min(45, request["work_deadline_unix"] - time.time())
        write_new(attempt / "fluent_launch_started.json", {"job_id": slot.JOB_ID, "created_unix": time.time(), "start_timeout": 45}, attempt)
        native_process = bounded_call(lambda: subprocess.Popen(native_command, cwd=str(attempt),
                                      stdin=subprocess.DEVNULL, shell=False), min(10, launch_end - time.monotonic()))
        identity = psutil.Process(native_process.pid)
        write_new(attempt / "fluent_process.json", {"job_id": slot.JOB_ID, "pid": native_process.pid,
                  "creation_time": identity.create_time(), "parent_pid": identity.ppid(),
                  "executable": identity.exe()}, attempt)
        local_hosts = {"localhost", "127.0.0.1", socket.gethostname()}
        if os.environ.get("USERDNSDOMAIN"):
            local_hosts.add(socket.gethostname() + "." + os.environ["USERDNSDOMAIN"])
        for addresses in psutil.net_if_addrs().values():
            local_hosts.update(row.address for row in addresses if row.family == socket.AF_INET)
        report["connection_state"] = "waiting_private_credentials"
        credentials = wait_private_credentials(server_info, attempt, native_process, launch_end, local_hosts)
        report["connection_state"] = "connecting_once_with_validated_private_snapshot"
        # Public explicit fields avoid a second file read/TOCTOU and environment
        # fallbacks. Authentication is retained; nothing is logged or persisted.
        solver = bounded_call(lambda: pyfluent.connect_to_fluent(ip=credentials.host, port=credentials.port,
                    password=credentials.password, cleanup_on_exit=False, start_watchdog=False,
                    start_transcript=False), launch_end - time.monotonic())
        del credentials
        report["connection_state"] = "connected"
        report["launch_uncertain"] = False
        write_new(attempt / "fluent_launch_ready.json", {"job_id": slot.JOB_ID, "created_unix": time.time()}, attempt)
        step("start_transcript", lambda: solver.tui.file.start_transcript(str(transcript)))
        if report["commands"].get("start_transcript") != "completed":
            raise RuntimeGateError("Native transcript unavailable; refusing an unauditable read")
        step("read_mesh", lambda: solver.file.read_mesh(file_name=str(slot.safe(MESHES[slot.domain]["path"]))))
        if report["commands"].get("read_mesh") != "completed":
            raise RuntimeGateError("read_mesh failed; no diagnostic on an empty domain")
        # Quality deliberately precedes mesh check: unassigned overset can make
        # the latter complain, and must not suppress independent native quality.
        step("quality", lambda: solver.tui.mesh.quality())
        fluids = step("fluid_zones", lambda: solver.setup.cell_zone_conditions.fluid.get_object_names())
        if isinstance(fluids, list) and len(fluids) <= 100 and all(isinstance(x, str) and len(x) <= 256 for x in fluids):
            report["fluid_zones"] = fluids
        names = step("wall_zones", lambda: solver.setup.boundary_conditions.wall.get_object_names())
        if not isinstance(names, list) or len(names) > 100:
            names = []
        for name in MESHES[slot.domain]["walls"]:
            if name not in names:
                report["wall_adjacency"][name] = {"native_query_completed": False, "reason": "required native wall missing"}
                continue
            key = "adjacent_cell_zone:" + name
            value = step(key, lambda name=name: solver.setup.boundary_conditions.wall[name].adjacent_cell_zone())
            # The documented query may vary across releases: retain the bounded
            # actual value, but never guess a zone from an integer or a label.
            raw = repr(value)[:2000]
            zone = value if isinstance(value, str) and len(value) <= 256 else None
            report["wall_adjacency"][name] = {"native_query_completed": report["commands"].get(key) == "completed",
                                              "adjacent_cell_zone": zone, "raw_native_query": raw}
        step("list_zones", lambda: solver.tui.mesh.modify_zones.list_zones())
        step("mesh_check", lambda: solver.tui.mesh.check())
    except Exception as exc:
        report["errors"]["native_adapter"] = type(exc).__name__ + ": native operation failed; exception text omitted"
        report["safe_error_classification"] = safe_connection_error(exc)
    finally:
        if solver is not None:
            step("stop_transcript", lambda: solver.tui.file.stop_transcript())
        if transcript.exists() and transcript.stat().st_size <= 2 * 1024 * 1024:
            report["metrics"] = parse_native_output(transcript.read_text(encoding="utf-8", errors="replace"))
            report["transcript_sha256"] = sha256(transcript)
        report["inspection"] = validate_native_evidence(report, slot.domain)
        write_new(attempt / "native_result.json", report, attempt)
        if solver is not None:
            # Never cleanup through broad/global Fluent shutdown. The owner still
            # verifies/cleans its PID+create-time descendants after this call.
            solver.exit()
    return report


def run(attempt, *, stage_root=STAGE):
    slot, attempt = slot_for_attempt(attempt, stage_root)
    if (attempt / "probe_result.json").exists():
        raise RuntimeGateError("Consumed inspection slot cannot be rerun")
    write_new(attempt / "worker_claim.json", {"job_id": slot.JOB_ID, "pid": os.getpid(), "created_unix": time.time()}, attempt)
    began = time.monotonic()
    report = {"job_id": slot.JOB_ID, "operation": "mesh_inspection", "domain": slot.domain, "status": "running",
              "phase": "validating", "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False,
              "owned_cleanup_verified": True, "inputs_reverified_after_cleanup": False,
              "sources_reverified_after_cleanup": False, "solver_launched": False}
    request = frozen = process = tree = lock = None
    uncertain = False
    identities = {}
    deadline = began + 300
    work_deadline = deadline - 30
    def remaining():
        return max(0, min(deadline - time.monotonic(), request["deadline_unix"] - time.time() if request else 300))
    def working():
        cancel = attempt / "cancel_request.json"
        if cancel.exists():
            row = read_json(cancel, 4096)
            if row.get("job_id") != slot.JOB_ID or row.get("operation") != slot.CANCEL_OPERATION:
                raise RuntimeGateError("Invalid inspection cancel sentinel")
            raise InterruptedError("Read-only mesh inspection cancellation requested")
        if time.monotonic() >= work_deadline or request and time.time() >= request["work_deadline_unix"]:
            raise TimeoutError("270-second work deadline reached; beginning owned cleanup")
    def state(phase):
        report.update(phase=phase, elapsed_seconds=max(time.monotonic() - began, 300 - remaining()))
        write_state(attempt, report)
    def capture():
        tree.capture()
        if len(tree.members) > 256:
            raise RuntimeGateError("Unexpected owned process tree size")
        for pid, stamp in tree.members.items():
            if pid in identities:
                continue
            try:
                item = psutil.Process(pid)
                if item.create_time() == stamp:
                    identities[pid] = {"pid": pid, "creation_time": stamp, "name": item.name(), "executable": item.exe()}
            except psutil.NoSuchProcess:
                pass
    def recheck():
        slot.authorize()
        validate_request(slot, attempt)
        return validate_frozen(stage_root)
    try:
        request = validate_request(slot, attempt)
        deadline = began + max(0, request["deadline_unix"] - time.time())
        work_deadline = deadline - 30
        report.update({key: request[key] for key in ("created_unix", "deadline_unix", "work_deadline_unix")})
        validate_launcher(slot, attempt, remaining)
        frozen = bounded_call(recheck, min(15, remaining() - 30))
        report["protected_inputs_before"] = frozen["protected_inputs"]
        working()
        lock = AnsysTaskLock(attempt, stage_root=stage_root)
        lock.__enter__()
        write_new(attempt / "command_audit.json", {"job_id": slot.JOB_ID, "fixed_native_apis": list(NATIVE_APIS),
                  "arbitrary_code_accepted": False, "mesh": MESHES[slot.domain], "start_timeout_seconds": 45,
                  "maximum_total_seconds": 300, "cleanup_begins_seconds": 270}, attempt)
        env = bounded_call(lambda: run_mesh_job.child_environment(attempt, frozen["contract"]), min(10, remaining() - 30))
        for key in list(env):
            if key.upper().startswith(("PYFLUENT", "PYTHON")):
                env.pop(key)
        env.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUTF8="1", PYTHONIOENCODING="utf-8",
                   AWP_ROOT242=frozen["contract"]["software"]["ansys_root"])
        command = [slot._python(), "-B", str(slot.safe("mesh_inspection.py")), "--native", "--attempt", str(attempt)]
        report["native_adapter_command"] = command
        bounded_call(recheck, min(15, remaining() - 30))
        working()
        state("starting_native_adapter")
        with (attempt / "native.stdout.log").open("xb") as stdout, (attempt / "native.stderr.log").open("xb") as stderr:
            try:
                process = bounded_call(lambda: subprocess.Popen(command, cwd=str(attempt), env=env,
                                       stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, shell=False), min(10, remaining() - 30))
                tree = bounded_call(lambda: OwnedProcessTree(process, psutil), min(3, remaining() - 30))
            except Exception:
                uncertain = True
                raise
            if tree.errors:
                uncertain = True
                raise RuntimeGateError("Native adapter process ownership unverified")
            state("native_read_only_inspection")
            while True:
                working()
                bounded_call(capture, min(2, remaining() - 30))
                code = bounded_call(process.poll, min(2, remaining() - 30))
                if code is not None:
                    report["native_adapter_exit_code"] = code
                    break
                time.sleep(min(0.1, max(0, work_deadline - time.monotonic())))
        native = read_json(attempt / "native_result.json")
        if (native.get("job_id") != slot.JOB_ID or native.get("domain") != slot.domain or
                native.get("provenance") != "live_fluent_read_only" or native.get("mesh_generated") is not False or
                type(native.get("flow_iterations")) is not int or native.get("flow_iterations") != 0):
            raise RuntimeGateError("Native receipt identity/provenance/read-only scope mismatch")
        report["native"] = native
        if native.get("launch_uncertain") is not False:
            uncertain = True
            raise RuntimeGateError("Native Fluent startup ownership uncertain; no retry")
        transcript = attempt / "native.transcript.txt"
        if (not transcript.is_file() or transcript.stat().st_size > 2 * 1024 * 1024 or native.get("transcript_sha256") != sha256(transcript) or
                native.get("metrics") != parse_native_output(transcript.read_text(encoding="utf-8", errors="replace"))):
            raise RuntimeGateError("Native transcript/aggregate binding missing or changed")
        report["status"] = "completed" if code == 0 else "failed"
    except InterruptedError as exc:
        report.update(status="cancelled", error=str(exc))
    except Exception as exc:
        report.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
    finally:
        try:
            state("cleanup")
        except Exception as exc:
            report.update(status="failed", state_write_error=str(exc))
        report["owned_process_identity"] = list(identities.values())
        report["solver_launch_attempted"] = (attempt / "fluent_launch_started.json").exists()
        report["solver_launched"] = (attempt / "fluent_process.json").exists()
        # Startup can be detached/uncertain. Never release ownership just because
        # the Python launcher exited or no currently visible child was found.
        if report["solver_launch_attempted"] and not (attempt / "fluent_launch_ready.json").exists():
            uncertain = True
        if report["solver_launched"]:
            native_prefix = str(Path(frozen["contract"]["software"]["ansys_root"]) / "fluent").replace("\\", "/").casefold() + "/" if frozen else ""
            if not native_prefix or not any(row["executable"].replace("\\", "/").casefold().startswith(native_prefix)
                                            for row in identities.values()):
                uncertain = True
                report["native_identity_error"] = "No Fluent descendant executable identity was captured; preserve lock"
        cleanup = not uncertain
        if tree is not None:
            try:
                if remaining() < 15:
                    raise UncertainCall("Insufficient 8+5-second cleanup reserve")
                cleanup = bool(bounded_call(tree.close, min(20, remaining() - 1))) and cleanup
            except Exception as exc:
                cleanup = False
                report["cleanup_error"] = type(exc).__name__ + ": " + str(exc)
            report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
            report["ownership_errors"] = list(tree.errors)
        elif process is not None:
            cleanup = False
        if request is not None:
            try:
                latest = bounded_call(recheck, min(10, remaining() - 1))
                report["protected_inputs_after"] = latest["protected_inputs"]
                report["inputs_reverified_after_cleanup"] = True
                report["sources_reverified_after_cleanup"] = True
            except Exception as exc:
                report.update(status="failed", recheck_error=type(exc).__name__ + ": " + str(exc))
        if lock is not None and lock.acquired:
            if not cleanup:
                lock.retain("Read-only inspection launch/ownership/cleanup uncertain; preserve for review")
            try:
                lock.__exit__(None, None, None)
            except Exception as exc:
                cleanup = False
                lock.retain(str(exc))
                report["lock_error"] = str(exc)
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
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--run", action="store_true")
    mode.add_argument("--native", action="store_true")
    mode.add_argument("--preflight", action="store_true")
    parser.add_argument("--domain", choices=tuple(MESHES))
    parser.add_argument("--attempt", type=Path)
    args = parser.parse_args(argv)
    if args.preflight:
        if args.domain not in MESHES or args.attempt is not None:
            parser.error("Preflight requires domain only")
        slot = InspectionSlot(args.domain)
        slot.authorize()
        frozen = validate_frozen()
        fluent = Path(frozen["contract"]["software"]["fluent"])
        if not fluent.is_file():
            raise RuntimeGateError("Fixed Fluent executable missing")
        report = {"status": "passed", "domain": args.domain, "job_id": slot.JOB_ID,
                  "slot_reserved": slot.safe(slot.ATTEMPT).exists(), "mesh": MESHES[args.domain],
                  "native_adjacency_verified": False, "CFD_authorized": False,
                  "maximum_total_seconds": 300, "cleanup_reserve_seconds": 30}
    else:
        if args.attempt is None or args.domain is not None:
            parser.error("Execution requires only a fixed attempt")
        report = native_collect(args.attempt) if args.native else run(args.attempt)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report.get("status") in ("completed", "passed") or args.native else 1


if __name__ == "__main__":
    raise SystemExit(main())

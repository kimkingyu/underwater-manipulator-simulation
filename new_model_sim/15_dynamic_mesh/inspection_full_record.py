"""Fixed full-record validation evidence. No launcher, arbitrary path or command API."""
from __future__ import annotations
import os
import re
import time

from definition_probe import bounded_call, write_new
from mcp_bridge import sha256
from runtime import RuntimeGateError

EXIT_REQUEST_SECONDS = 8
EXIT_WAIT_SECONDS = 12
TRANSCRIPT_CALL_SECONDS = 15
MAX_LOG_BYTES = 2 * 1024 * 1024
ERROR_NAMES = re.compile(r"fluent-[0-9]{1,3}-error\.log\Z")
MARKERS = {
    "abnormal_exit": re.compile(r"abnormal\s+exit", re.I),
    "transcript_open_error": re.compile(r"sopenoutputfile|unable to open file for output", re.I),
    "stdin_eof_halt": re.compile(r"halting due to end of file on input", re.I),
    "fatal_error": re.compile(r"\bfatal\b|\bsegmentation\s+(?:fault|violation)\b", re.I),
}


def clean_native_environment(environment):
    """Remove every case variant; caller adds only its fixed Python environment."""
    return {k: v for k, v in environment.items() if not k.upper().startswith(("PYFLUENT", "PYTHON"))}


def native_log_review(slot, attempt, *, adapter_finished=False):
    """Bounded, fixed local log names only. Persist hashes/classes, never raw text."""
    attempt = slot.safe(slot.ATTEMPT) if attempt == slot.safe(slot.ATTEMPT) else None
    if attempt is None:
        raise RuntimeGateError("Log review requires the fixed owned attempt")
    names = []
    for count, entry in enumerate(attempt.iterdir(), 1):
        if count > 128:
            raise RuntimeGateError("Fixed attempt contains too many entries for bounded error review")
        if entry.name.startswith("fluent-") and entry.name.endswith("-error.log"):
            if not ERROR_NAMES.fullmatch(entry.name):
                raise RuntimeGateError("Unexpected native error filename; no arbitrary log reads")
            names.append(entry.name)
        if len(names) > 16:
            raise RuntimeGateError("Too many fixed native error files")
    names.sort()
    if adapter_finished:
        names += ["native.stdout.log", "native.stderr.log", "native.transcript.txt"]
    files = []
    for name in names:
        path = slot.safe(slot.ATTEMPT + "/" + name)
        if not path.is_file() or path.stat().st_size > MAX_LOG_BYTES:
            raise RuntimeGateError("Native log is missing or oversized")
        with path.open("rb") as stream:
            data = stream.read(MAX_LOG_BYTES + 1)
        if len(data) > MAX_LOG_BYTES:
            raise RuntimeGateError("Native log grew beyond fixed read budget")
        text = data.decode("utf-8", errors="replace")
        classes = sorted(k for k, pattern in MARKERS.items() if pattern.search(text))
        # An unexplained nonempty Fluent error file is not proof of a clean exit.
        if ERROR_NAMES.fullmatch(name) and text.strip() and not classes:
            classes = ["unclassified_native_error"]
        files.append({"name": name, "size": len(data), "sha256": sha256(path), "error_classes": classes})
    return {"scope": "fixed_attempt_error_files_and_adapter_logs" if adapter_finished else "fixed_attempt_error_files",
            "files": files, "error_classes": sorted({k for row in files for k in row["error_classes"]}),
            "raw_text_persisted": False, "server_info_processed": False}


def normal_shutdown(solver, process, native_input, slot, attempt, work_deadline_unix):
    """Request normal exit once; keep PIPE open through observed process.wait.

    Public exit() gets NO kwargs/internal timeout. PYFLUENT_TIMEOUT_FORCE_EXIT
    must be absent. Only the existing outer worker may clean its verified tree.
    A returned exit request is never substituted for a returned native process.
    """
    from mesh_inspection_connection import safe_connection_error
    result = {"job_id": slot.JOB_ID, "cleanup_on_exit": True, "public_exit_kwargs": {},
        "force_exit_environment_absent": not any(k.upper() == "PYFLUENT_TIMEOUT_FORCE_EXIT" for k in os.environ),
        "library_force_exit_called": False, "exit_request_attempted": False, "exit_request_completed": False,
        "process_wait_completed": False, "native_returncode": None, "pipe_open_at_request": False,
        "pipe_open_after_wait": False, "exit_before_pipe_close": False, "normal_exit_observed": False,
        "request_budget_seconds": EXIT_REQUEST_SECONDS, "wait_budget_seconds": EXIT_WAIT_SECONDS, "errors": {}}
    limit = lambda cap: min(cap, max(0, work_deadline_unix - time.time()))
    write_new(attempt / "normal_exit_attempted.json", {"job_id": slot.JOB_ID, "stage": "attempted"}, attempt)
    try:
        if not result["force_exit_environment_absent"]:
            raise RuntimeGateError("Force-exit environment must be absent")
        result["pipe_open_at_request"] = process.stdin is not None and not process.stdin.closed
        if not result["pipe_open_at_request"] or process.poll() is not None:
            raise RuntimeGateError("Normal exit requires a still-live owned launcher and open PIPE")
        identity = slot._json(slot.ATTEMPT + "/fluent_process.json")
        if identity.get("job_id") != slot.JOB_ID or identity.get("pid") != process.pid:
            raise RuntimeGateError("Normal exit process must match the fixed ownership ledger")
        result["process_identity"] = identity
        result["exit_request_attempted"] = True
        bounded_call(solver.exit, limit(EXIT_REQUEST_SECONDS))
        result["exit_request_completed"] = True
        code = bounded_call(lambda: process.wait(timeout=limit(EXIT_WAIT_SECONDS)), limit(EXIT_WAIT_SECONDS))
        result.update(process_wait_completed=True, native_returncode=code,
                      pipe_open_after_wait=not process.stdin.closed)
        result["exit_before_pipe_close"] = result["pipe_open_after_wait"]
        result["normal_exit_observed"] = type(code) is int and code == 0 and result["pipe_open_after_wait"]
        if not result["normal_exit_observed"]:
            raise RuntimeGateError("Native normal exit was not observed")
    except Exception as exc:
        result["errors"]["normal_exit"] = safe_connection_error(exc)
    result["stage"] = "completed" if result["normal_exit_observed"] and not result["errors"] else "failed"
    write_new(attempt / ("normal_exit_" + result["stage"] + ".json"), result, attempt)
    return result


def full_record_passed(native):
    """Pure fail-closed check; hashes and persisted stage files are checked separately."""
    if not isinstance(native, dict):
        return False
    r = native.get("normal_exit", {})
    ready = native.get("readiness", {})
    calls = native.get("transcript_calls", {})
    return (native.get("errors") == {} and ready.get("cleanup_on_exit") is True and
        ready.get("constructor_watchdog_process_started") is False and
        native.get("stdin_close_phase") == "after_observed_normal_exit_and_wait" and
        all(calls.get(a) == {"stage": "completed", "attempted": True, "completed": True, "failed": False} for a in ("start", "stop")) and
        r.get("stage") == "completed" and r.get("cleanup_on_exit") is True and r.get("public_exit_kwargs") == {} and
        r.get("force_exit_environment_absent") is True and r.get("library_force_exit_called") is False and
        all(r.get(k) is True for k in ("exit_request_attempted", "exit_request_completed", "process_wait_completed",
            "pipe_open_at_request", "pipe_open_after_wait", "exit_before_pipe_close", "normal_exit_observed")) and
        type(r.get("native_returncode")) is int and r["native_returncode"] == 0 and r.get("errors") == {} and
        r.get("request_budget_seconds") == EXIT_REQUEST_SECONDS and r.get("wait_budget_seconds") == EXIT_WAIT_SECONDS and
        native.get("native_error_review", {}).get("error_classes") == [])


def verify_full_record_artifacts(conn, native, final, *, classified=False):
    """Every original artifact remains mandatory; only explicit helper classification differs."""
    if not full_record_passed(native):
        raise RuntimeGateError("Complete normal exit evidence required")
    if classified:
        from inspection_license_review import classify_live_helpers
        classify_live_helpers(conn, final)
    elif final.get("owned_live_before_cleanup") != []:
        raise RuntimeGateError("Unclassified live process blocks normal exit acceptance")
    path = conn.safe(conn.ATTEMPT)
    for action in ("start", "stop"):
        for stage in ("attempted", "completed"):
            if conn._json(conn.ATTEMPT + "/transcript_" + action + "_" + stage + ".json") != {
                    "job_id": conn.JOB_ID, "action": action, "stage": stage}:
                raise RuntimeGateError("Transcript per-call stage identity mismatch")
        if conn.safe(conn.ATTEMPT + "/transcript_" + action + "_failed.json").exists():
            raise RuntimeGateError("Failed transcript marker cannot qualify")
    if (native.get("transcript_target") != (path / "native.transcript.txt").as_posix() or
            conn._json(conn.ATTEMPT + "/normal_exit_attempted.json") != {"job_id": conn.JOB_ID, "stage": "attempted"} or
            conn._json(conn.ATTEMPT + "/normal_exit_completed.json") != native["normal_exit"] or
            native["normal_exit"].get("job_id") != conn.JOB_ID or
            native["normal_exit"].get("process_identity") != conn._json(conn.ATTEMPT + "/fluent_process.json") or
            conn.safe(conn.ATTEMPT + "/normal_exit_failed.json").exists()):
        raise RuntimeGateError("Normal exit or fixed posix transcript artifact mismatch")
    if native["native_error_review"] != native_log_review(conn, path):
        raise RuntimeGateError("Native error evidence changed")
    current = native_log_review(conn, path, adapter_finished=True)
    if current != final.get("native_log_review") or current["error_classes"]:
        raise RuntimeGateError("Native log errors or changed log binding block QA")
    return True

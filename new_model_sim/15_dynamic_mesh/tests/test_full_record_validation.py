"""Independent user-approved slot, normal exit, error evidence: pure mocks only."""
import ast
import concurrent.futures
import copy
import json
import os
from pathlib import Path
import sys
import time
import types

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import inspection_batch002_bridge as b
import inspection_batch002 as w
import inspection_full_record as f
import mcp_bridge as legacy
from test_definition_probe import sandbox, dump
from test_inspection_batch002 import batch, completed_connection, reserve, connection_deps
from test_native_stdin_lifetime import native_input


def test_new_fixed_slot_and_original_QA_paths_no_extra_budget(batch, monkeypatch):
    root, _, popen = batch
    for key in ("supplement_connection_retired", "stdin_validation_retired"):
        monkeypatch.setitem(b.POLICY, key, True)
    for old in (b.Batch002Slot("connection", root), b.SupplementalConnectionSlot(root), b.StdinValidationSlot(root)):
        with pytest.raises(legacy.BridgeError, match="retired"): old.start(old.START_CONFIRMATION)
        assert not old.safe(old.ATTEMPT).exists() and old.status()["remaining_attempts"] == 0
    slot = b.FullRecordValidationSlot(root)
    assert slot.ATTEMPT == b.FULL_RECORD_ATTEMPT and slot.JOB_ID == b.FULL_RECORD_JOB and slot.is_connection
    assert slot.status()["remaining_attempts"] == 1
    assert b.batch_job(b.FULL_RECORD_JOB, root).slot == b.FULL_RECORD
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        assert qa.ATTEMPT == "runs/mcp_mesh_inspection/batch_002/" + name
        assert qa.status()["remaining_attempts"] == 1
        with pytest.raises(legacy.BridgeError): qa.start(b.START)
        assert not qa.safe(qa.ATTEMPT).exists()
    assert b.POLICY["additional_attempts_authorized"] == b.POLICY["additional_connection_attempts"] == 1
    assert b.POLICY["additional_QA_attempts"] == 0
    popen.assert_not_called()


@pytest.mark.parametrize("token", [b.START, b.SUPPLEMENT_START, b.STDIN_VALIDATION_START, b.FULL_RECORD_CANCEL, "NO"])
def test_only_new_explicit_confirmation_can_spend_full_record(batch, token):
    root, _, popen = batch
    slot = b.FullRecordValidationSlot(root)
    with pytest.raises(legacy.BridgeError): slot.start(token)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_new_concurrent_once_and_cancel_scope(batch, monkeypatch):
    root, _, popen = batch
    slot = b.FullRecordValidationSlot(root)
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    def start():
        try: return slot.start(b.FULL_RECORD_START)
        except legacy.BridgeError: return {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _: start(), range(4)))
    assert sum(r.get("new_worker_started") is True for r in results) == popen.call_count == 1
    assert slot.start(b.FULL_RECORD_START)["new_worker_started"] is False
    for wrong in (b.CANCEL, b.STDIN_VALIDATION_CANCEL, b.SUPPLEMENT_CANCEL):
        with pytest.raises(legacy.BridgeError): slot.cancel(slot.JOB_ID, wrong)
    assert slot.cancel(slot.JOB_ID, b.FULL_RECORD_CANCEL)["status"] == "cancellation_requested"
    assert legacy.read_json(slot.safe(slot.ATTEMPT + "/cancel_request.json"))["operation"] == "cancel_full_record_validation"


@pytest.mark.parametrize("old", [b.STDIN_VALIDATION, b.SUPPLEMENT, "connection"])
def test_old_native_success_cannot_unlock_full_record_gate(batch, old):
    root, _, popen = batch
    completed_connection(root, b.execution_slot(old, root))
    with pytest.raises(legacy.BridgeError): b.Batch002Slot("background", root).connection_gate()
    assert not b.FullRecordValidationSlot(root).safe(b.FULL_RECORD_ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("fault", ["returncode", "no_wait", "early_pipe", "request_only", "internal_timeout", "force_exit",
    "environment", "constructor", "stage", "failed_marker", "wrong_posix", "live_owned", "stderr_abnormal", "native_error",
    "transcript_empty", "mixed_grant", "source", "process_identity", "missing_log"])
def test_only_full_exact_current_native_evidence_can_unlock_QA(batch, fault):
    root, _, popen = batch
    path = completed_connection(root)
    native = legacy.read_json(path / "native_result.json")
    final = legacy.read_json(path / "probe_result.json")
    request = legacy.read_json(path / "request.json")
    if fault == "returncode": native["normal_exit"]["native_returncode"] = 1
    if fault == "no_wait": native["normal_exit"]["process_wait_completed"] = False
    if fault == "early_pipe": native["normal_exit"]["pipe_open_after_wait"] = False
    if fault == "request_only": native["normal_exit"]["normal_exit_observed"] = False
    if fault == "internal_timeout": native["normal_exit"]["public_exit_kwargs"] = {"timeout": 5, "timeout_force": False, "wait": False}
    if fault == "force_exit": native["normal_exit"]["library_force_exit_called"] = True
    if fault == "environment": native["normal_exit"]["force_exit_environment_absent"] = False
    if fault == "constructor": native["readiness"]["cleanup_on_exit"] = False
    if fault == "stage": (path / "transcript_stop_completed.json").unlink()
    if fault == "failed_marker": dump(path / "transcript_start_failed.json", {"stage": "failed"})
    if fault == "wrong_posix": native["transcript_target"] = str(path / "native.transcript.txt").replace("/", "\\")
    if fault == "live_owned": final["owned_live_before_cleanup"] = [{"pid": 333, "creation_time": 333.5}]
    if fault == "stderr_abnormal": (path / "native.stderr.log").write_text("Abnormal Exit!", encoding="utf-8")
    if fault == "native_error": (path / "fluent-0-error.log").write_text("Abnormal Exit!", encoding="utf-8")
    if fault == "transcript_empty": (path / "native.transcript.txt").write_bytes(b"")
    if fault == "mixed_grant": request["authorization"]["sha256"] = "0" * 64
    if fault == "source": request["sources"][0]["sha256"] = "0" * 64
    if fault == "process_identity": native["normal_exit"]["process_identity"]["creation_time"] += 1
    if fault == "missing_log": (path / "native.stdout.log").unlink()
    final["native"] = native
    dump(path / "native_result.json", native); dump(path / "probe_result.json", final); dump(path / "request.json", request)
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        with pytest.raises(legacy.BridgeError): qa.start(b.START)
        assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


def test_completed_new_slot_opens_only_original_QA_with_full_records(batch):
    root, _, popen = batch
    path = completed_connection(root)
    for name in ("background", "arm"):
        assert b.Batch002Slot(name, root).connection_gate()["job_id"] == b.FULL_RECORD_JOB
    assert legacy.read_json(path / "probe_result.json")["native"]["normal_exit"]["native_returncode"] == 0
    popen.assert_not_called()


@pytest.mark.parametrize("fault", ["ok", "nonzero", "wait_timeout", "exit_timeout", "already_exited", "closed_pipe", "force_env", "wrong_owner"])
def test_normal_exit_is_bounded_and_observed_before_pipe_close(native_input, monkeypatch, fault):
    root, slot, path, pipe, process, solver, events, _, _ = native_input
    # Only the full slot should be used in production; this unit tests the shared
    # native cleanup helper with the historical mock pipe and exact mock ledger.
    dump(path / "fluent_process.json", {"job_id": slot.JOB_ID, "pid": process.pid, "creation_time": 123.5})
    if fault == "nonzero": process.wait = lambda **k: 3
    if fault == "wait_timeout": process.wait = lambda **k: (_ for _ in ()).throw(TimeoutError("SECRET"))
    if fault == "already_exited": process.poll = lambda: 0
    if fault == "closed_pipe": pipe.close()
    if fault == "force_env": monkeypatch.setenv("PYFLUENT_TIMEOUT_FORCE_EXIT", "0.01")
    if fault == "wrong_owner": dump(path / "fluent_process.json", {"job_id": "wrong", "pid": process.pid})
    calls = []
    def bounded(function, seconds):
        calls.append(seconds)
        if fault == "exit_timeout" and function == solver.exit: raise w.UncertainCall("SECRET")
        return function()
    monkeypatch.setattr(f, "bounded_call", bounded)
    result = f.normal_shutdown(solver, process, w.NativeInputLease(), slot, path, time.time() + 25)
    assert result["normal_exit_observed"] is (fault == "ok")
    assert result["public_exit_kwargs"] == {} and result["library_force_exit_called"] is False
    assert all(0 < v <= 12 for v in calls) and len(calls) <= 2
    if fault == "ok":
        assert calls[0] <= 8 and calls[1] <= 12 and not pipe.closed
        assert result["native_returncode"] == 0 and result["pipe_open_after_wait"] is True
    assert "SECRET" not in json.dumps(result)


@pytest.mark.parametrize("text,category", [("Abnormal Exit!", "abnormal_exit"),
    ("sopenoutputfile: unable to open file for output", "transcript_open_error"),
    ("Halting due to end of file on input.", "stdin_eof_halt"), ("FATAL signal", "fatal_error"),
    ("Unexpected undocumented error", "unclassified_native_error")])
def test_native_error_files_keep_only_safe_class_hash_and_block(batch, text, category):
    root, _, _ = batch
    slot = b.FullRecordValidationSlot(root); path = slot.safe(slot.ATTEMPT); path.mkdir(parents=True)
    (path / "fluent-0-error.log").write_text(text + " SECRET", encoding="utf-8")
    # Must never inspect arbitrary filenames, particularly credentials.
    (path / "server.info").write_text("OFFLINE-SECRET", encoding="utf-8")
    review = f.native_log_review(slot, path)
    assert category in review["error_classes"] and len(review["files"]) == 1
    assert "SECRET" not in json.dumps(review) and "server.info" not in json.dumps(review)


def test_environment_cleaning_removes_force_timeout_case_variants():
    assert f.clean_native_environment({"PYFLUENT_TIMEOUT_FORCE_EXIT": "1", "pyfluent_timeout_force_exit": "2",
        "PYTHONPATH": "unsafe", "KEEP": "ok"}) == {"KEEP": "ok"}


def test_same_single_authenticated_constructor_changes_only_cleanup_policy(connection_deps, tmp_path):
    import inspection_batch002_connection as c
    clock, deps, calls, mode = connection_deps
    captured = []
    original = deps["connection"]
    def constructor(**kwargs): captured.append(kwargs); return original(**kwargs)
    deps["connection"] = constructor
    evidence = {}
    c.connect_ready(tmp_path, object(), clock.monotonic() + 45, evidence, dependencies=deps, normal_exit=True)
    assert len(captured) == 1 and captured[0]["cleanup_on_exit"] is True
    assert captured[0]["start_transcript"] is False and captured[0]["inside_container"] is False
    assert evidence["constructor_watchdog_process_started"] is False and evidence["single_channel_count"] == 1


@pytest.mark.parametrize("fault", ["oversize", "too_many", "unknown_name", "directory"])
def test_error_log_reader_refuses_unbounded_or_nonregular_candidates(batch, fault):
    root, _, _ = batch
    slot = b.FullRecordValidationSlot(root); path = slot.safe(slot.ATTEMPT); path.mkdir(parents=True)
    if fault == "oversize": (path / "fluent-0-error.log").write_bytes(b"x" * (f.MAX_LOG_BYTES + 1))
    if fault == "too_many":
        for i in range(17): (path / ("fluent-" + str(i) + "-error.log")).write_bytes(b"")
    if fault == "unknown_name": (path / "fluent-arbitrary-error.log").write_bytes(b"")
    if fault == "directory": (path / "fluent-0-error.log").mkdir()
    with pytest.raises(w.RuntimeGateError): f.native_log_review(slot, path)


def test_official_no_timeout_exit_calls_only_finalizer_not_library_force(monkeypatch):
    official = Path("D:/Program Files/ANSYS Inc/v242/commonfiles/CPython/3_10/winx64/Release/python/lib/site-packages/ansys/fluent/core/fluent_connection.py")
    source = official.read_text(encoding="utf-8")
    cls = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == "FluentConnection")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "exit")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), method], type_ignores=[])
    ast.fix_missing_locations(module)
    calls = []
    scope = {"os": os, "logger": types.SimpleNamespace(info=lambda *a: None, debug=lambda *a: None)}
    exec(compile(module, "<offline-official-exit-no-RPC>", "exec"), scope)
    monkeypatch.delenv("PYFLUENT_TIMEOUT_FORCE_EXIT", raising=False)
    session = types.SimpleNamespace(_remote_instance=None, _finalizer=lambda: calls.append("finalizer"),
        force_exit=lambda: pytest.fail("Library force exit forbidden"),
        wait_process_finished=lambda **k: pytest.fail("Library process waiting forbidden"))
    scope["exit"](session)
    assert calls == ["finalizer"]
    # Demonstrate why supplying internal timeout plus wait=False is unsafe.
    assert isinstance(False, (float, int))
    constructor = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    assert not any(isinstance(n, ast.Call) and ast.unparse(n.func).endswith(("Popen", "launch_watchdog")) for n in ast.walk(constructor))


def test_public_exit_no_internal_timeout_or_force_or_launcher_in_helper():
    source = (STAGE / "inspection_full_record.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)]
    assert not any(x.endswith(("force_exit", "Popen", "launch_fluent", "connect_to_fluent", "wait_process_finished")) for x in calls)
    normal = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "normal_shutdown")
    assert "bounded_call(solver.exit, limit(EXIT_REQUEST_SECONDS))" in ast.get_source_segment(source, normal)
    protocol = (STAGE / "check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_full_record_validation", {"confirmation": "NO"})' in protocol
    assert b.FULL_RECORD_START not in protocol
    assert b.POLICY["normal_exit_request_seconds"] + b.POLICY["normal_exit_wait_seconds"] == 20
    assert b.POLICY["maximum_total_seconds_per_slot"] == 300 and b.POLICY["cleanup_reserve_seconds"] == 30

"""No process/socket is launched: private native-input lifetime via fake pipe/Popen."""
import ast
import json
import os
from pathlib import Path
import sys
import threading
import types

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import inspection_batch002 as w
import inspection_batch002_bridge as b
import mcp_bridge as legacy
from test_definition_probe import sandbox
from test_inspection_batch002 import batch, reserve, ready, completed_connection, dump


class FakePipe:
    def __init__(self, events):
        self.closed = False
        self.close_calls = 0
        self.events = events
        self.closed_event = threading.Event()
    def write(self, *a): pytest.fail("Native stdin must never receive bytes/commands")
    def flush(self, *a): pytest.fail("No native stdin writes/flushes are allowed")
    def close(self):
        self.close_calls += 1
        self.closed = True
        self.events.append("pipe_close")
        self.closed_event.set()


@pytest.fixture
def native_input(batch, monkeypatch):
    root, slot, _ = batch
    path, _ = reserve(root, slot)
    parent = os.getppid()
    dump(path / "launcher.json", {"job_id": slot.JOB_ID, "pid": parent, "creation_time": 123.5})
    dump(root / ".ansys-task.lock", {"pid": parent, "attempt": str(path)})
    monkeypatch.setenv("REMOTING_SERVER_ADDRESS", "127.0.0.1")
    executable = root / "OFFLINE_DO_NOT_EXECUTE.exe"
    executable.write_bytes(b"OFFLINE FIXTURE ONLY")
    monkeypatch.setattr(w, "validate_frozen", lambda *a: {"contract": {"software": {
        "fluent": str(executable), "ansys_root": str(root)}}})
    monkeypatch.setattr(w.psutil, "Process", lambda pid=None: types.SimpleNamespace(
        pid=os.getpid() if pid is None else pid, create_time=lambda: 123.5,
        ppid=lambda: os.getpid(), exe=lambda: str(executable)))
    events = []
    pipe = FakePipe(events)
    process = types.SimpleNamespace(pid=501, stdin=pipe)
    calls = []
    def popen(command, **kwargs):
        calls.append((command, kwargs))
        assert kwargs["stdin"] is w.subprocess.PIPE
        assert kwargs["shell"] is False
        assert not pipe.closed
        events.append("popen")
        return process
    monkeypatch.setattr(w.subprocess, "Popen", popen)
    def start_transcript(*a):
        assert not pipe.closed
        events.append("start_transcript")
        (path / "native.transcript.txt").write_text("OFFLINE connection transcript", encoding="utf-8")
    def stop_transcript(*a):
        assert not pipe.closed
        events.append("stop_transcript")
    def exit_session():
        assert not pipe.closed
        events.append("session_exit")
    solver = types.SimpleNamespace(tui=types.SimpleNamespace(file=types.SimpleNamespace(
        start_transcript=start_transcript, stop_transcript=stop_transcript)), exit=exit_session)
    connects = []
    def connect(attempt, actual_process, deadline, evidence):
        assert actual_process is process and not pipe.closed
        connects.append(True)
        events.append("connect_wait")
        evidence.update(ready())
        return solver
    monkeypatch.setattr(w, "connect_ready", connect)
    return root, slot, path, pipe, process, solver, events, calls, connects


def test_private_pipe_kept_open_through_connection_and_session_exit(native_input):
    root, _, path, pipe, _, _, events, calls, connects = native_input
    result = w.native_collect(path, stage_root=root)
    assert len(calls) == len(connects) == 1
    assert events == ["popen", "connect_wait", "start_transcript", "stop_transcript", "session_exit", "pipe_close"]
    assert pipe.close_calls == 1
    assert result["stdin_lifecycle"] == {"mode": "private_subprocess_PIPE", "bytes_written": 0,
        "close_requested_in_cleanup": True, "pipe_returned": True, "writer_closed": True,
        "late_return_after_close_request": False}
    assert result["stdin_close_phase"] == "after_bounded_session_exit_request"
    assert "OFFLINE-SECRET" not in json.dumps(result)
    assert legacy.read_json(path / "native_result.json")["stdin_lifecycle"]["writer_closed"] is True


@pytest.mark.parametrize("error", [TimeoutError, RuntimeError, w.UncertainCall])
def test_connection_failure_or_timeout_closes_only_in_final_cleanup(native_input, monkeypatch, error):
    root, _, path, pipe, _, _, events, calls, _ = native_input
    attempts = []
    def fail(*a):
        assert not pipe.closed
        attempts.append(True); events.append("connect_failure")
        raise error("OFFLINE-SECRET; never persist details")
    monkeypatch.setattr(w, "connect_ready", fail)
    report = w.native_collect(path, stage_root=root)
    assert len(calls) == len(attempts) == 1
    assert events == ["popen", "connect_failure", "pipe_close"]
    assert pipe.closed and pipe.close_calls == 1
    assert report["launch_uncertain"] is True
    assert report["stdin_close_phase"] == "native_failure_final_cleanup"
    assert "OFFLINE-SECRET" not in json.dumps(report)


def test_popen_failure_no_pipe_to_leak_and_no_connect(native_input, monkeypatch):
    root, _, path, pipe, _, _, events, _, connects = native_input
    calls = []
    def fail(*a, **kwargs):
        calls.append(True)
        assert kwargs["stdin"] is w.subprocess.PIPE
        raise OSError("OFFLINE-SECRET")
    monkeypatch.setattr(w.subprocess, "Popen", fail)
    report = w.native_collect(path, stage_root=root)
    assert len(calls) == 1 and connects == []
    assert pipe.close_calls == 0 and events == []
    assert report["stdin_lifecycle"]["pipe_returned"] is False
    assert report["stdin_lifecycle"]["close_requested_in_cleanup"] is True
    assert report["launch_uncertain"] is True and "OFFLINE-SECRET" not in json.dumps(report)


def test_session_exit_error_still_closes_writer_after_exit_attempt(native_input):
    root, _, path, pipe, _, solver, events, calls, _ = native_input
    def fail_exit():
        assert not pipe.closed
        events.append("session_exit_failed")
        raise RuntimeError("OFFLINE-SECRET")
    solver.exit = fail_exit
    result = w.native_collect(path, stage_root=root)
    assert events[-2:] == ["session_exit_failed", "pipe_close"]
    assert pipe.close_calls == 1 and len(calls) == 1
    assert "session_exit" in result["errors"] and "OFFLINE-SECRET" not in json.dumps(result)


def test_receipt_write_failure_cannot_skip_input_close(native_input, monkeypatch):
    root, _, path, pipe, _, _, events, _, _ = native_input
    write = w.write_new
    def failing_write(destination, *a, **k):
        if Path(destination).name == "readiness.json": raise OSError("OFFLINE write error")
        return write(destination, *a, **k)
    monkeypatch.setattr(w, "write_new", failing_write)
    with pytest.raises(OSError): w.native_collect(path, stage_root=root)
    assert events[-2:] == ["session_exit", "pipe_close"] and pipe.close_calls == 1


def test_bounded_popen_timeout_closes_late_returned_private_writer(native_input, monkeypatch):
    root, _, path, pipe, process, _, _, _, connects = native_input
    release, returned = threading.Event(), threading.Event()
    calls = []
    def slow_popen(*a, **kwargs):
        assert kwargs["stdin"] is w.subprocess.PIPE
        calls.append(True)
        release.wait(timeout=2)
        returned.set()
        return process
    real_bounded = w.bounded_call
    def bounded(function, timeout):
        return real_bounded(function, 0.02 if function.__name__ == "launch_with_private_input" else timeout)
    monkeypatch.setattr(w.subprocess, "Popen", slow_popen)
    monkeypatch.setattr(w, "bounded_call", bounded)
    try:
        report = w.native_collect(path, stage_root=root)
        assert len(calls) == 1 and connects == []
        assert report["launch_uncertain"] is True
        assert report["stdin_lifecycle"]["close_requested_in_cleanup"] is True
        assert report["stdin_lifecycle"]["pipe_returned"] is False
    finally:
        release.set()
        assert returned.wait(timeout=1)
        assert pipe.closed_event.wait(timeout=1)
    assert pipe.close_calls == 1


def test_hung_session_exit_still_closes_input_in_cleanup_without_repeat(native_input, monkeypatch):
    root, _, path, pipe, _, solver, events, calls, _ = native_input
    release, finished = threading.Event(), threading.Event()
    exits = []
    def slow_exit():
        assert not pipe.closed
        exits.append(True); events.append("session_exit_started")
        try: release.wait(timeout=2)
        finally: finished.set()
    solver.exit = slow_exit
    bounded = w.bounded_call
    def short(function, timeout):
        return bounded(function, 0.02 if function.__name__ == "slow_exit" else timeout)
    monkeypatch.setattr(w, "bounded_call", short)
    try:
        report = w.native_collect(path, stage_root=root)
        assert len(exits) == len(calls) == 1
        assert events[-2:] == ["session_exit_started", "pipe_close"]
        assert pipe.closed and "session_exit" in report["errors"]
    finally:
        release.set(); assert finished.wait(timeout=1)


def test_private_writer_close_failure_is_reported_not_retried_or_hidden(native_input):
    root, _, path, pipe, _, _, events, calls, _ = native_input
    def fail_close():
        pipe.close_calls += 1
        events.append("pipe_close_failed")
        raise OSError("OFFLINE-SECRET close failure")
    pipe.close = fail_close
    result = w.native_collect(path, stage_root=root)
    assert pipe.close_calls == len(calls) == 1
    assert events[-2:] == ["session_exit", "pipe_close_failed"]
    assert "stdin_close" in result["errors"] and "OFFLINE-SECRET" not in json.dumps(result)
    assert result["stdin_lifecycle"]["writer_closed"] is False


def test_already_closed_private_writer_is_rejected():
    events = []
    writer = FakePipe(events); writer.close()
    lease = w.NativeInputLease()
    with pytest.raises(w.RuntimeGateError): lease.adopt(types.SimpleNamespace(stdin=writer))
    lease.close()
    assert lease.evidence()["close_requested_in_cleanup"] is True


def test_late_pipe_adoption_after_cleanup_closes_without_writes_or_process_signals():
    events = []
    writer = FakePipe(events)
    lease = w.NativeInputLease()
    lease.close()
    lease.adopt(types.SimpleNamespace(stdin=writer))
    lease.close()
    assert writer.close_calls == 1
    assert lease.evidence()["late_return_after_close_request"] is True
    assert lease.evidence()["bytes_written"] == 0


def test_native_pipe_overrides_outer_devnull_and_no_context_or_communicate():
    source = (STAGE / "inspection_batch002.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    native = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "native_collect")
    outer = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    native_popen = [n for n in ast.walk(native) if isinstance(n, ast.Call) and ast.unparse(n.func) == "subprocess.Popen"]
    outer_popen = [n for n in ast.walk(outer) if isinstance(n, ast.Call) and ast.unparse(n.func) == "subprocess.Popen"]
    assert len(native_popen) == len(outer_popen) == 1
    assert next(ast.unparse(k.value) for k in native_popen[0].keywords if k.arg == "stdin") == "subprocess.PIPE"
    assert next(ast.unparse(k.value) for k in outer_popen[0].keywords if k.arg == "stdin") == "subprocess.DEVNULL"
    for node in ast.walk(native):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in ("communicate", "write", "flush", "writelines")
    for node in ast.walk(tree):
        if isinstance(node, ast.With):
            assert not any(isinstance(item.context_expr, ast.Call) and ast.unparse(item.context_expr.func) == "subprocess.Popen" for item in node.items)
    assert "min(45, request[\"work_deadline_unix\"]" in source
    assert b.POLICY["maximum_total_seconds_per_slot"] == 300
    assert b.POLICY["cleanup_reserve_seconds"] == 30
    assert b.POLICY["native_start_timeout_seconds"] == 45


def test_revision_has_no_new_native_allowance_and_retired_supplement_cannot_recreate(batch, monkeypatch):
    root, slot, popen = batch
    monkeypatch.setitem(b.POLICY, "supplement_connection_retired", True)
    with pytest.raises(legacy.BridgeError, match="permanently retired"):
        slot.start(b.SUPPLEMENT_START)
    assert not slot.safe(slot.ATTEMPT).exists()
    slot.safe(slot.ATTEMPT).mkdir(parents=True)
    with pytest.raises(w.RuntimeGateError): w.run(slot.safe(slot.ATTEMPT), stage_root=root)
    with pytest.raises(w.RuntimeGateError): w.native_collect(slot.safe(slot.ATTEMPT), stage_root=root)
    assert not slot.safe(slot.ATTEMPT + "/worker_claim.json").exists()
    assert slot.status()["remaining_attempts"] == 0
    assert b.POLICY["additional_attempts_authorized"] == b.POLICY["additional_connection_attempts"] == 0
    assert b.POLICY["additional_QA_attempts"] == 0
    popen.assert_not_called()


def test_original_consumed_failed_supplement_stays_unverified_and_QA_uncreated(batch):
    root, slot, popen = batch
    path = completed_connection(root)
    req = legacy.read_json(path / "request.json")
    req["authorization"] = {"path": str(root / b.POLICY["baseline_batch_authorization_relative"]),
                            "sha256": b.POLICY["baseline_batch_authorization_sha256"]}
    dump(path / "request.json", req)
    final = legacy.read_json(path / "probe_result.json")
    final.update(status="cleanup_unverified", owned_cleanup_verified=False)
    dump(path / "probe_result.json", final)
    before = (path / "probe_result.json").read_bytes()
    answer = slot.start(b.SUPPLEMENT_START)
    assert answer["status"] == "already_reserved" and answer["job"]["status"] == "cleanup_unverified"
    assert answer["job"]["slot_authorization"] == req["authorization"]
    assert answer["job"]["connection_gate"]["status"] == "unverified"
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        with pytest.raises(legacy.BridgeError): qa.start(b.START)
        assert not qa.safe(qa.ATTEMPT).exists()
    assert (path / "probe_result.json").read_bytes() == before
    popen.assert_not_called()

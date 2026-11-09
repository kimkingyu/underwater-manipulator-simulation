"""Pure offline batch002 tests. No native executable, socket or real worker launch."""
import ast
import copy
import json
import os
from pathlib import Path
import sys
import time
import types
from unittest import mock

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import mcp_bridge as legacy
import inspection_batch002_bridge as b
import inspection_batch002 as w
import inspection_batch002_connection as c
import mesh_inspection_connection as private
from test_definition_probe import sandbox, dump


@pytest.fixture
def batch(sandbox, monkeypatch):
    root, _, _ = sandbox
    # Internal fake-process regression of historical launches only; production
    # policy retires the consumed supplement and no MCP parameter can alter it.
    monkeypatch.setitem(b.POLICY, "supplement_connection_retired", False)
    monkeypatch.setitem(b.POLICY, "stdin_validation_retired", False)
    for relative in b.SOURCES:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists(): path.write_text("# offline batch002 fixture\n", encoding="utf-8")
    for name, original in list(b.MESHES.items()):
        row = copy.deepcopy(original)
        path = root / row["path"]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("OFFLINE MESH " + name).encode())
        row.update(size=path.stat().st_size, sha256=legacy.sha256(path))
        monkeypatch.setitem(b.MESHES, name, row)
    baseline = root / b.POLICY["baseline_batch_authorization_relative"]
    dump(baseline, {"OFFLINE": "original unconsumed three-slot grant fixture"})
    monkeypatch.setitem(b.POLICY, "baseline_batch_authorization_sha256", legacy.sha256(baseline))
    slot = b.SupplementalConnectionSlot(root)
    dump(root / b.MANIFEST, {"schema_version": 1, "files": slot._sources()})
    grant = copy.deepcopy(b.POLICY)
    grant.update(authorization_source="OFFLINE fixture, not a real grant", recorded_local="fixture",
                 execution_policy="Never execute a native binary", source_manifest_sha256=legacy.sha256(root / b.MANIFEST))
    dump(root / b.GRANT, grant)
    monkeypatch.setattr(b.Batch002Slot, "_python", lambda self: "offline-python")
    monkeypatch.setattr(b.Batch002Slot, "preflight", lambda self: self.authorize())
    monkeypatch.setattr(b.Batch002Slot, "_inspect_pid", lambda self, pid: {"pid": pid, "exists": False})
    popen = mock.Mock(return_value=types.SimpleNamespace(pid=123))
    monkeypatch.setattr(legacy.subprocess, "Popen", popen)
    return root, slot, popen


def ready():
    return {"provenance": "live_fluent_batch002", "requested_address": "127.0.0.1", "actual_address": "127.0.0.1",
        "actual_endpoint_class": "ipv4_loopback", "channel_target_address": "127.0.0.1",
        "server_info_alias_class": "literal_ipv4_loopback", "alias_resolution": "literal_only_no_dns",
        "port": 12345, "health_status": "SERVING", "health_observations": ["SERVING"],
        "health_poll_count": 1, "health_unavailable_count": 0, "health_rpc_observations": [], "health_call_uncertain": False,
        "transport_status": "READY", "transport_wait_count": 1, "transport_wait_limit_seconds": 20, "startup_window_within_deadline": True,
        "solver_mode_native": True, "native_version": "24.2.0", "solver_session_constructed": True,
        "single_channel_count": 1, "connection_constructor_count": 1, "credentials_persisted": False,
        "scheme_identity_queries_completed": True}


def reserve(root, slot):
    path = slot.safe(slot.ATTEMPT)
    path.mkdir(parents=True)
    req = slot.make_request(time.time(), slot._sources())
    dump(path / "request.json", req)
    slot.preserve_sources(req)
    dump(root / ".mesh-inspection-dispatch.lock", {"job_id": slot.JOB_ID, "token": "offline-owned-dispatch"})
    dump(path / "launcher.json", {"job_id": slot.JOB_ID, "pid": os.getpid(), "creation_time": 123.5})
    return path, req


def completed_connection(root, slot=None):
    slot = b.FullRecordValidationSlot(root) if slot is None else slot
    path, req = reserve(root, slot)
    req.update(created_unix=req["created_unix"] - 30, deadline_unix=req["deadline_unix"] - 30,
               work_deadline_unix=req["work_deadline_unix"] - 30)
    dump(path / "request.json", req)
    readiness = ready()
    (path / "native.transcript.txt").write_text("OFFLINE transcript fixture", encoding="utf-8")
    dump(path / "command_audit.json", {"job_id": slot.JOB_ID, "batch_id": b.BATCH, "slot": slot.slot, "free_form_commands_accepted": False})
    dump(path / "readiness.json", readiness)
    native = {"job_id": slot.JOB_ID, "slot": slot.slot, "batch_id": b.BATCH,
        "provenance": "live_fluent_batch002", "readiness": readiness,
        "readiness_sha256": legacy.sha256(path / "readiness.json"), "launch_uncertain": False,
        "commands": {}, "errors": {}, "mesh_generated": False, "flow_iterations": 0,
        "stdin_lifecycle": {"mode": "private_subprocess_PIPE", "bytes_written": 0, "close_requested_in_cleanup": True,
            "pipe_returned": True, "writer_closed": True, "late_return_after_close_request": False},
        "stdin_close_phase": "after_bounded_session_exit_request",
        "transcript_sha256": legacy.sha256(path / "native.transcript.txt")}
    dump(path / "native_result.json", native)
    executable = legacy.read_json(root / "contract.json")["software"]["fluent"].replace("/", "\\")
    dump(path / "native_claim.json", {"job_id": slot.JOB_ID, "pid": 222, "creation_time": 222.5,
        "parent_pid": os.getpid(), "parent_creation_time": 123.5})
    dump(path / "fluent_process.json", {"job_id": slot.JOB_ID, "pid": 333, "creation_time": 333.5,
        "parent_pid": 222, "executable": executable})
    from preflight import verify_inputs
    inputs = verify_inputs(legacy.read_json(root / "frozen/input_manifest.json"), workspace=root.parent.parent)
    final = {"job_id": slot.JOB_ID, "slot": slot.slot, "batch_id": b.BATCH, "status": "completed",
        "protected_inputs_before": inputs, "protected_inputs_after": inputs,
        "created_unix": req["created_unix"], "deadline_unix": req["deadline_unix"], "work_deadline_unix": req["work_deadline_unix"],
        "serial_dispatch_released": True, "mesh_motion_verified": False, "ownership_errors": [],
        "native": native, "owned_cleanup_verified": True, "inputs_reverified_after_cleanup": True,
        "sources_reverified_after_cleanup": True, "solver_launched": True, "mesh_generated": False,
        "flow_iterations": 0, "native_adapter_exit_code": 0, "elapsed_seconds": 22.0,
        "owned_processes": [{"pid": 222, "creation_time": 222.5}, {"pid": 333, "creation_time": 333.5}],
        "owned_process_identity": [{"pid": 333, "creation_time": 333.5, "executable": executable}]}
    complete_record_fixture(slot, path, native, final)
    dump(path / "native_result.json", native)
    dump(path / "probe_result.json", final)
    (root / ".mesh-inspection-dispatch.lock").unlink()  # offline fixture models completed owned cleanup
    return path


def complete_record_fixture(slot, path, native, final):
    """Synthetic offline evidence only; never used with a real attempt."""
    from inspection_full_record import native_log_review
    native["readiness"].update(cleanup_on_exit=True, constructor_watchdog_process_started=False)
    dump(path / "readiness.json", native["readiness"])
    native["readiness_sha256"] = legacy.sha256(path / "readiness.json")
    native["stdin_close_phase"] = "after_observed_normal_exit_and_wait"
    native["transcript_target"] = (path / "native.transcript.txt").as_posix()
    native["transcript_calls"] = {a: {"stage": "completed", "attempted": True, "completed": True, "failed": False} for a in ("start", "stop")}
    for action in ("start", "stop"):
        for stage in ("attempted", "completed"):
            dump(path / ("transcript_" + action + "_" + stage + ".json"), {"job_id": slot.JOB_ID, "action": action, "stage": stage})
    native["normal_exit"] = {"job_id": slot.JOB_ID, "stage": "completed", "cleanup_on_exit": True, "public_exit_kwargs": {},
        "force_exit_environment_absent": True, "library_force_exit_called": False, "exit_request_attempted": True,
        "exit_request_completed": True, "process_wait_completed": True, "native_returncode": 0, "pipe_open_at_request": True,
        "pipe_open_after_wait": True, "exit_before_pipe_close": True, "normal_exit_observed": True,
        "request_budget_seconds": 8, "wait_budget_seconds": 12, "errors": {},
        "process_identity": legacy.read_json(path / "fluent_process.json")}
    dump(path / "normal_exit_attempted.json", {"job_id": slot.JOB_ID, "stage": "attempted"})
    dump(path / "normal_exit_completed.json", native["normal_exit"])
    native["native_error_review"] = native_log_review(slot, path)
    for name in ("native.stdout.log", "native.stderr.log"):
        if not (path / name).exists(): (path / name).write_bytes(b"")
    final.update(owned_live_before_cleanup=[], native_log_review=native_log_review(slot, path, adapter_finished=True))


@pytest.mark.parametrize("name", ["background", "arm"])
def test_connection_gate_missing_refuses_without_consuming(batch, name):
    root, _, popen = batch
    slot = b.Batch002Slot(name, root)
    with pytest.raises(legacy.BridgeError): slot.start(slot.START_CONFIRMATION)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_connection_slot_has_no_mesh_and_is_consumed_on_first_launch(batch, monkeypatch):
    root, slot, popen = batch
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    first = slot.start(slot.START_CONFIRMATION)
    assert first["new_worker_started"] is True
    req = legacy.read_json(slot.safe(slot.ATTEMPT + "/request.json"))
    assert req["mesh"] is None and req["batch_id"] == b.BATCH
    assert req["deadline_unix"] - req["created_unix"] == 300
    assert req["work_deadline_unix"] - req["created_unix"] == 270
    assert slot.start(slot.START_CONFIRMATION)["status"] == "already_reserved"
    assert popen.call_count == 1


@pytest.mark.parametrize("token", ["NO", "RUN_STAGE15_READ_ONLY_MESH_INSPECTION_ONCE", "RUN_STAGE15_FULL_MESH_CONSTRUCTION_ONCE", b.CANCEL])
def test_no_old_token_can_spend_new_grant(batch, token):
    _, slot, popen = batch
    with pytest.raises(legacy.BridgeError): slot.start(token)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_exact_three_slots_no_extra_or_old_aliases(batch, monkeypatch):
    root, _, popen = batch
    completed_connection(root)
    for name in ("background", "arm"):
        slot = b.Batch002Slot(name, root)
        monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
        assert slot.start(slot.START_CONFIRMATION)["new_worker_started"] is True
        assert slot.start(slot.START_CONFIRMATION)["new_worker_started"] is False
        (root / ".mesh-inspection-dispatch.lock").unlink()  # offline fake worker completes between serial slots
    assert popen.call_count == 2  # fixture wrote connection; only two fake mesh workers
    assert all(b.Batch002Slot(n, root).status()["remaining_attempts"] == 0 for n in b.SLOTS)
    for name in ("combined", "connection2", "batch001", "../arm"):
        with pytest.raises(legacy.BridgeError): b.Batch002Slot(name, root)


@pytest.mark.parametrize("key,value", [("maximum_attempts", 4), ("maximum_attempts_per_slot", 2),
    ("maximum_total_seconds_per_slot", 301), ("native_start_timeout_seconds", 60), ("remoting_server_address", "0.0.0.0"),
    ("connection_gate_required", False), ("automatic_retry_allowed", True), ("old_slots_reusable", True),
    ("additional_attempts_authorized", 2), ("additional_QA_attempts", 1), ("additional_connection_attempts", 2), ("transport_ready_max_seconds", 45),
    ("mesh_generation_allowed", True), ("initialization_allowed", True), ("iterations_allowed", True),
    ("motion_allowed", True), ("CFD_allowed", True), ("maximum_attempts", True), ("arbitrary_command", "exec")])
def test_new_grant_cannot_expand_scope(batch, key, value):
    root, slot, popen = batch
    grant = legacy.read_json(root / b.GRANT); grant[key] = value; dump(root / b.GRANT, grant)
    with pytest.raises(legacy.BridgeError): slot.start(slot.START_CONFIRMATION)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("file,key,value", [
    ("probe_result.json", "status", "failed"), ("probe_result.json", "owned_cleanup_verified", False),
    ("probe_result.json", "inputs_reverified_after_cleanup", False), ("probe_result.json", "sources_reverified_after_cleanup", False),
    ("probe_result.json", "solver_launched", False), ("probe_result.json", "elapsed_seconds", 301),
    ("probe_result.json", "batch_id", "old-batch"), ("probe_result.json", "native_adapter_exit_code", 0),
    ("probe_result.json", "serial_dispatch_released", False), ("probe_result.json", "mesh_motion_verified", True),
    ("probe_result.json", "deadline_unix", 1),
    ("native_result.json", "provenance", "mock"), ("native_result.json", "launch_uncertain", True),
    ("native_result.json", "commands", {"read_mesh": "completed"}),
    ("request.json", "batch_id", "inspection-batch001"), ("request.json", "sources", []),
    ("readiness.json", "health_status", "NOT_SERVING"), ("readiness.json", "solver_mode_native", False),
    ("readiness.json", "actual_address", "192.0.2.1"), ("readiness.json", "native_version", "23.1.0"),
    ("readiness.json", "health_observations", []), ("readiness.json", "health_observations", ["NOT_SERVING"]),
    ("readiness.json", "transport_status", "WAITING"), ("readiness.json", "transport_wait_count", 2),
    ("readiness.json", "transport_wait_limit_seconds", 45), ("readiness.json", "startup_window_within_deadline", False),
    ("readiness.json", "health_call_uncertain", True), ("readiness.json", "health_rpc_observations", [{"poll": 1, "rpc_status_codes": ["UNAUTHENTICATED"]}]),
    ("probe_result.json", "ownership_errors", ["unknown PID"]), ("native_result.json", "errors", {"transport": "UNAVAILABLE"}),
    ("native_claim.json", "parent_pid", 999), ("fluent_process.json", "creation_time", 0)])
def test_connection_gate_tampering_or_exit0_alone_never_passes(batch, file, key, value):
    root, _, popen = batch
    path = completed_connection(root)
    row = legacy.read_json(path / file)
    if file == "probe_result.json" and key == "native_adapter_exit_code":
        row = {"job_id": row["job_id"], "native_adapter_exit_code": 0, "status": "completed"}
    else: row[key] = value
    dump(path / file, row)
    mesh = b.Batch002Slot("background", root)
    with pytest.raises(legacy.BridgeError): mesh.start(b.START)
    assert not mesh.safe(mesh.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("relative", [*b.SOURCES, b.MANIFEST, "runs/mesh/attempt_004/background.msh", "runs/mesh/attempt_004/arm.msh"])
def test_source_mesh_or_manifest_changes_block_new_slots(batch, relative):
    root, slot, popen = batch
    with (root / relative).open("ab") as out: out.write(b"changed")
    with pytest.raises(legacy.BridgeError): slot.start(slot.START_CONFIRMATION)
    popen.assert_not_called()


@pytest.mark.parametrize("relative", ["execution_binding.json", "execution_binding/mesh_inspection_full_record_authorization_001.json",
    "execution_binding/mesh_inspection_full_record_sources_001.json"])
def test_archived_execution_grant_or_manifest_tamper_blocks_mesh(batch, relative):
    root, _, popen = batch
    path = completed_connection(root)
    with (path / "source_snapshot" / relative).open("ab") as out: out.write(b"tampered")
    with pytest.raises((legacy.BridgeError, ValueError)):
        b.Batch002Slot("arm", root).start(b.START)
    assert not (root / "runs/mcp_mesh_inspection/batch_002/arm").exists()
    popen.assert_not_called()


def test_archived_connection_source_tamper_blocks_mesh(batch):
    root, _, popen = batch
    path = completed_connection(root)
    with (path / "source_snapshot/stage15_mcp.py").open("ab") as out: out.write(b"tampered")
    with pytest.raises(legacy.BridgeError): b.Batch002Slot("arm", root).start(b.START)
    assert not (root / "runs/mcp_mesh_inspection/batch_002/arm").exists()
    popen.assert_not_called()


@pytest.mark.parametrize("lock", [".ansys-task.lock", ".sizing-dispatch.lock", ".mesh-inspection-dispatch.lock"])
def test_shared_lock_preserved_without_consuming(batch, lock):
    root, slot, popen = batch
    dump(root / lock, {"token": "not-owned"})
    with pytest.raises(legacy.BridgeError): slot.start(slot.START_CONFIRMATION)
    assert legacy.read_json(root / lock)["token"] == "not-owned"
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_uncertain_worker_launch_preserves_dispatch_and_once_budget(batch, monkeypatch):
    root, slot, _ = batch
    monkeypatch.setattr(slot, "launch_worker", lambda *a, **k: (_ for _ in ()).throw(legacy.WorkerLaunchUncertain("offline")))
    result = slot.start(slot.START_CONFIRMATION)
    assert result["status"] == "cleanup_unverified"
    assert (root / ".mesh-inspection-dispatch.lock").exists()
    assert slot.start(slot.START_CONFIRMATION)["new_worker_started"] is False
    assert slot.cancel(slot.JOB_ID, slot.CANCEL_CONFIRMATION)["lock_must_be_preserved"] is True


def test_old_slots_permanently_consumed_new_slots_separate(batch):
    from mesh_inspection_mcp_bridge import InspectionSlot, START_TOKEN
    root, _, popen = batch
    for domain in ("background", "arm"):
        old = InspectionSlot(domain, root)
        path = old.safe(old.ATTEMPT); path.mkdir(parents=True)
        dump(path / "job_state.json", {"job_id": old.JOB_ID, "status": "reserved", "created_unix": time.time()})
        assert old.start(START_TOKEN)["status"] == "already_reserved"
        assert old.status()["remaining_inspections"] == 0
        assert b.Batch002Slot(domain, root).status()["remaining_attempts"] == 1
    popen.assert_not_called()


class Clock:
    now = 0.0
    def monotonic(self): return self.now
    def sleep(self, d): self.now += d


@pytest.fixture
def connection_deps(tmp_path, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(c.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(c.time, "sleep", clock.sleep)
    credentials = private.PrivateCredentials("127.0.0.1", 12345, "OFFLINE-SECRET")
    monkeypatch.setattr(c, "wait_private_credentials", lambda *a: credentials)
    calls = []
    channel = types.SimpleNamespace(close=lambda: calls.append("close"))
    statuses = iter(["NOT_SERVING", "SERVING"])
    class Health:
        def __init__(self, ch, metadata, error):
            assert ch is channel and metadata == [("password", "OFFLINE-SECRET")]
            calls.append("health_construct")
        def check_health(self):
            calls.append("health")
            return next(statuses)
    scheme = types.SimpleNamespace(scheme_eval=lambda query: calls.append(query) or True,
                                   string_eval=lambda query: calls.append(query) or "(24 2 0)")
    connection = types.SimpleNamespace(scheme_eval=scheme)
    def build(**kwargs):
        assert kwargs["channel"] is channel and kwargs["password"] == "OFFLINE-SECRET"
        assert "health" in calls
        calls.append("connection")
        return connection
    def solver(**kwargs):
        assert kwargs["fluent_connection"] is connection
        calls.append("solver")
        return "OFFLINE SOLVER"
    def one_channel(*a, **k): calls.append("channel"); return channel
    def ready_future(ch):
        assert ch is channel
        calls.append("ready_future")
        return types.SimpleNamespace(result=lambda timeout: calls.append("transport_READY"), cancel=lambda: calls.append("future_cancel"))
    deps = {"channel": one_channel, "ready_future": ready_future, "health": Health, "error_state": lambda: None, "connection": build, "solver": solver}
    return clock, deps, calls, scheme


def test_health_polling_same_channel_precedes_identity_and_solver(connection_deps, tmp_path):
    clock, deps, calls, _ = connection_deps
    evidence = {}
    result = c.connect_ready(tmp_path, None, 10, evidence, dependencies=deps)
    assert result == "OFFLINE SOLVER" and c.readiness_passed(evidence)
    assert calls.count("channel") == calls.count("connection") == calls.count("solver") == 1
    assert calls.count("health") == 2
    assert calls.index("connection") > max(i for i, x in enumerate(calls) if x == "health")
    assert calls.index(c.FIXED_MODE_QUERY) < calls.index("solver")
    assert "OFFLINE-SECRET" not in json.dumps(evidence)


@pytest.mark.parametrize("fault", ["not_serving", "mode_false", "bad_version", "transport", "remote_endpoint"])
def test_health_mode_transport_failure_never_opens_gate(connection_deps, tmp_path, monkeypatch, fault):
    clock, deps, calls, scheme = connection_deps
    if fault == "not_serving": deps["health"] = lambda *a: types.SimpleNamespace(check_health=lambda: "NOT_SERVING")
    if fault == "mode_false": scheme.scheme_eval = lambda q: False
    if fault == "bad_version": scheme.string_eval = lambda q: "(23 1 0)"
    if fault == "transport": deps["health"] = lambda *a: types.SimpleNamespace(check_health=lambda: (_ for _ in ()).throw(RuntimeError("OFFLINE-SECRET")))
    if fault == "remote_endpoint": monkeypatch.setattr(c, "wait_private_credentials", lambda *a: private.PrivateCredentials("192.0.2.1", 12345, "OFFLINE-SECRET"))
    evidence = {}
    with pytest.raises(Exception): c.connect_ready(tmp_path, None, 0.6, evidence, dependencies=deps)
    assert not c.readiness_passed(evidence)
    assert calls.count("channel") <= 1 and calls.count("connection") <= 1 and "solver" not in calls
    assert "OFFLINE-SECRET" not in json.dumps(evidence)


class StartupRpcError(RuntimeError):
    def __init__(self, status):
        super().__init__("OFFLINE-SECRET-DETAIL")
        self.status = status
    def code(self): return types.SimpleNamespace(name=self.status)


def test_transport_never_ready_times_out_without_health_or_new_channel(connection_deps, tmp_path):
    clock, deps, calls, _ = connection_deps
    timeouts = []
    def ready_future(channel):
        calls.append("ready_future")
        def result(timeout):
            timeouts.append(timeout); clock.now += timeout
            raise TimeoutError("OFFLINE-SECRET-DETAIL")
        return types.SimpleNamespace(result=result, cancel=lambda: calls.append("future_cancel"))
    deps["ready_future"] = ready_future
    evidence = {}
    with pytest.raises(TimeoutError): c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps)
    assert timeouts == [20] and clock.now == 20
    assert calls.count("channel") == calls.count("ready_future") == 1
    assert "health_construct" not in calls and "connection" not in calls
    assert evidence["transport_status"] == "FAILED_OR_TIMED_OUT"
    assert not c.readiness_passed(evidence) and "OFFLINE-SECRET" not in json.dumps(evidence)


def test_completed_unavailable_then_serving_same_channel_succeeds(connection_deps, tmp_path):
    clock, deps, calls, _ = connection_deps
    statuses = iter(["UNAVAILABLE", "NOT_SERVING", "SERVING"])
    def check():
        calls.append("health")
        status = next(statuses)
        if status == "UNAVAILABLE": raise StartupRpcError(status)
        return status
    deps["health"] = lambda *a: types.SimpleNamespace(check_health=check)
    evidence = {}
    assert c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps) == "OFFLINE SOLVER"
    assert c.readiness_passed(evidence)
    assert evidence["health_observations"] == ["RPC_UNAVAILABLE", "NOT_SERVING", "SERVING"]
    assert evidence["health_rpc_observations"] == [{"poll": 1, "rpc_status_codes": ["UNAVAILABLE"]}]
    assert evidence["health_poll_count"] == 3 and evidence["health_unavailable_count"] == 1
    assert calls.count("channel") == calls.count("ready_future") == calls.count("connection") == 1
    assert calls.index("transport_READY") < calls.index("health")
    assert "OFFLINE-SECRET" not in json.dumps(evidence)


@pytest.mark.parametrize("status", ["UNAUTHENTICATED", "PERMISSION_DENIED", "UNKNOWN", "INTERNAL"])
def test_authentication_or_uncertain_rpc_code_immediately_fails(connection_deps, tmp_path, status):
    _, deps, calls, _ = connection_deps
    health_calls = []
    def check():
        health_calls.append(True)
        raise StartupRpcError(status)
    deps["health"] = lambda *a: types.SimpleNamespace(check_health=check)
    evidence = {}
    with pytest.raises(StartupRpcError): c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps)
    assert len(health_calls) == 1 and "connection" not in calls
    assert evidence["health_failure_rpc_codes"] == [status]
    assert not c.readiness_passed(evidence) and "OFFLINE-SECRET" not in json.dumps(evidence)


def test_transport_health_and_constructor_share_original_45_second_window(connection_deps, tmp_path):
    clock, deps, calls, _ = connection_deps
    timeouts = []
    def ready_future(channel):
        calls.append("ready_future")
        def result(timeout):
            timeouts.append(timeout); clock.now += 19.5
        return types.SimpleNamespace(result=result, cancel=lambda: None)
    deps["ready_future"] = ready_future
    def check(): calls.append("health"); return "NOT_SERVING"
    deps["health"] = lambda *a: types.SimpleNamespace(check_health=check)
    evidence = {}
    with pytest.raises(TimeoutError): c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps)
    assert clock.now == pytest.approx(45) and timeouts == [20]
    assert calls.count("channel") == calls.count("ready_future") == 1
    assert "connection" not in calls and evidence["startup_window_within_deadline"] is False
    assert evidence["health_poll_count"] <= c.MAX_HEALTH_POLLS


def test_hung_health_call_is_not_overlapped_or_polled_again(connection_deps, tmp_path, monkeypatch):
    import threading
    from definition_probe import UncertainCall
    _, deps, calls, _ = connection_deps
    gate, ended = threading.Event(), threading.Event()
    live = {"calls": 0, "active": 0, "maximum": 0}
    def check():
        live["calls"] += 1; live["active"] += 1; live["maximum"] = max(live["maximum"], live["active"])
        try: gate.wait(timeout=1)
        finally: live["active"] -= 1; ended.set()
        return "SERVING"
    deps["health"] = lambda *a: types.SimpleNamespace(check_health=check)
    monkeypatch.setattr(c, "HEALTH_CALL_MAX_SECONDS", 0.02)
    evidence = {}
    try:
        with pytest.raises(UncertainCall): c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps)
        assert live["calls"] == live["maximum"] == 1
        assert evidence["health_call_uncertain"] is True and "connection" not in calls
        assert not c.readiness_passed(evidence)
    finally:
        gate.set(); assert ended.wait(timeout=1)


def test_fixed_policy_matches_ready_wait_limits():
    assert b.POLICY["native_start_timeout_seconds"] == 45
    assert b.POLICY["transport_ready_max_seconds"] == c.TRANSPORT_READY_MAX_SECONDS == 20
    assert b.POLICY["health_call_max_seconds"] == c.HEALTH_CALL_MAX_SECONDS == 2
    assert b.POLICY["maximum_health_polls"] == c.MAX_HEALTH_POLLS == 200
    assert b.POLICY["additional_attempts_authorized"] == 1
    assert b.POLICY["additional_connection_attempts"] == 1
    assert b.POLICY["original_additional_connection_attempts"] == 1
    assert b.POLICY["additional_QA_attempts"] == 0


@pytest.mark.parametrize("host,category", [("localhost", "literal_localhost"), ("LOCALHOST", "literal_localhost"),
    ("LocalHost", "literal_localhost"), ("127.0.0.1", "literal_ipv4_loopback")])
def test_literal_alias_uses_same_fixed_channel_without_dns(connection_deps, tmp_path, monkeypatch, host, category):
    import socket
    _, deps, calls, _ = connection_deps
    monkeypatch.setattr(c, "wait_private_credentials", private.wait_private_credentials)
    def forbidden_dns(*a, **k): pytest.fail("DNS/hosts lookup is forbidden for literal alias normalization")
    for name in ("getaddrinfo", "gethostbyname", "getfqdn", "gethostbyname_ex"):
        monkeypatch.setattr(socket, name, forbidden_dns)
    (tmp_path / "server.info").write_bytes((host + ":12345\nOFFLINE-SECRET\n").encode("ascii"))
    target = []
    original = deps["channel"]
    def channel(address, **kwargs): target.append(address); return original(address, **kwargs)
    deps["channel"] = channel
    evidence = {}
    assert c.connect_ready(tmp_path, types.SimpleNamespace(poll=lambda: None), 45, evidence, dependencies=deps) == "OFFLINE SOLVER"
    assert target == ["127.0.0.1:12345"]
    assert evidence["requested_address"] == evidence["channel_target_address"] == "127.0.0.1"
    assert evidence["server_info_alias_class"] == category and evidence["alias_resolution"] == "literal_only_no_dns"
    assert c.readiness_passed(evidence) and "OFFLINE-SECRET" not in json.dumps(evidence)
    assert calls.count("channel") == calls.count("ready_future") == calls.count("connection") == 1


@pytest.mark.parametrize("host", ["evil.example", "MACHINE-NAME", "localhost.evil", "localhost.", "localhost.localdomain",
    "127.0.0.2", "127.1", "127.000.000.001", "2130706433", "0x7f000001", "::1", "[::1]",
    "0.0.0.0", "192.0.2.1", " localhost", "localhost ", "localhost/", "localhost@evil", "127.0.0.1.evil",
    "localho\u017ft", "l\u043ecalhost", "\uff4cocalhost", "localhost\x00", "local\nhost", ""])
def test_other_hosts_ips_and_confusables_are_rejected_before_channel(connection_deps, tmp_path, monkeypatch, host):
    _, deps, calls, _ = connection_deps
    monkeypatch.setattr(c, "wait_private_credentials", lambda *a: private.PrivateCredentials(host, 12345, "OFFLINE-SECRET"))
    evidence = {}
    with pytest.raises(ValueError) as captured:
        c.connect_ready(tmp_path, None, 45, evidence, dependencies=deps)
    assert "channel" not in calls and evidence["single_channel_count"] == 0
    assert not c.readiness_passed(evidence)
    assert host not in str(captured.value) if host else True
    assert "OFFLINE-SECRET" not in json.dumps(evidence)


def test_alias_class_does_not_replace_transport_health_or_solver_proof():
    value = ready()
    value.update(server_info_alias_class="literal_localhost", health_status="unverified", solver_mode_native=False,
                 single_channel_count=0, transport_wait_count=0, health_poll_count=0, health_observations=[])
    assert not c.readiness_passed(value)


def test_consumed_failed_connection_stays_failed_after_alias_binding_repair(batch):
    root, slot, popen = batch
    path = completed_connection(root, slot)
    request = legacy.read_json(path / "request.json")
    request["authorization"] = {"path": str(root / b.POLICY["baseline_batch_authorization_relative"]),
                                "sha256": b.POLICY["baseline_batch_authorization_sha256"]}
    dump(path / "request.json", request)
    final = legacy.read_json(path / "probe_result.json")
    final.update(status="cleanup_unverified", owned_cleanup_verified=False)
    final["native"]["readiness"] = {"single_channel_count": 0, "transport_wait_count": 0, "health_poll_count": 0}
    dump(path / "probe_result.json", final)
    original = (path / "probe_result.json").read_bytes()
    result = slot.start(slot.START_CONFIRMATION)
    assert result["status"] == "already_reserved" and result["job"]["status"] == "cleanup_unverified"
    assert result["job"]["slot_authorization"] == request["authorization"]
    assert result["job"]["connection_gate"]["status"] == "unverified"
    for name in ("background", "arm"):
        mesh = b.Batch002Slot(name, root)
        with pytest.raises(legacy.BridgeError): mesh.start(b.START)
        assert not mesh.safe(mesh.ATTEMPT).exists()
    assert (path / "probe_result.json").read_bytes() == original
    popen.assert_not_called()


def test_loopback_alias_normalizer_has_no_network_or_environment_lookup():
    source = (STAGE / "inspection_batch002_connection.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    node = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "normalize_loopback_alias")
    calls = {ast.unparse(x.func) for x in ast.walk(node) if isinstance(x, ast.Call)}
    assert calls <= {"type", "host.isascii", "host.casefold", "ValueError"}
    for forbidden in ("getaddrinfo", "gethostbyname", "getfqdn", "socket.", "os.getenv"):
        assert forbidden not in ast.get_source_segment(source, node)


def test_worker_uncertain_popen_retains_task_lock(batch, monkeypatch):
    root, slot, _ = batch
    path, _ = reserve(root, slot)
    monkeypatch.setattr(w.subprocess, "Popen", lambda *a, **k: (_ for _ in ()).throw(w.UncertainCall("offline")))
    result = w.run(path, stage_root=root)
    assert result["status"] == "cleanup_unverified"
    assert (root / ".ansys-task.lock").exists()


@pytest.mark.parametrize("cleanup_ok", [True, False])
def test_worker_deadline_owned_cleanup_preserves_unknown(batch, monkeypatch, cleanup_ok):
    root, slot, _ = batch
    path, _ = reserve(root, slot)
    clock = Clock(); clock.now = 100
    monkeypatch.setattr(w.time, "monotonic", clock.monotonic)
    process = types.SimpleNamespace(pid=500, poll=lambda: None)
    monkeypatch.setattr(w.subprocess, "Popen", lambda *a, **k: process)
    events = []
    class Tree:
        def __init__(self, *a):
            clock.now = 371
            self.members = {500: 123.5}; self.errors = []
        def capture(self): pass
        def live(self): return []
        def close(self): events.append("owned_close"); return cleanup_ok
    monkeypatch.setattr(w, "OwnedProcessTree", Tree)
    result = w.run(path, stage_root=root)
    assert "owned_close" in events and result["elapsed_seconds"] >= 270
    assert result["owned_cleanup_verified"] is cleanup_ok
    assert (root / ".ansys-task.lock").exists() is not cleanup_ok


def test_accepted_worker_blocks_second_domain_before_task_lock_exists(batch, monkeypatch):
    root, _, popen = batch
    completed_connection(root)
    first = b.Batch002Slot("background", root)
    monkeypatch.setattr(first, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    first.start(b.START)
    assert (root / ".mesh-inspection-dispatch.lock").exists()
    second = b.Batch002Slot("arm", root)
    with pytest.raises(legacy.BridgeError, match="dispatch"):
        second.start(b.START)
    assert not second.safe(second.ATTEMPT).exists() and popen.call_count == 1


def test_connection_probe_cannot_enter_mesh_collector(batch):
    root, slot, _ = batch
    with pytest.raises(w.RuntimeGateError): w.collect_mesh(None, slot, root, {})


def test_complete_fake_connection_worker_produces_gate_evidence_after_cleanup(batch, monkeypatch):
    root, _, _ = batch
    slot = b.FullRecordValidationSlot(root)
    path, _ = reserve(root, slot)
    contract = legacy.read_json(root / "contract.json")
    executable = contract["software"]["fluent"].replace("/", "\\")
    def launch(*a, **k):
        assert k["env"]["REMOTING_SERVER_ADDRESS"] == "127.0.0.1"
        (path / "native.transcript.txt").write_text("OFFLINE authenticated connection transcript", encoding="utf-8")
        dump(path / "fluent_launch_started.json", {"job_id": slot.JOB_ID})
        dump(path / "fluent_launch_ready.json", {"job_id": slot.JOB_ID})
        dump(path / "native_claim.json", {"job_id": slot.JOB_ID, "pid": 500, "creation_time": 500.5,
            "parent_pid": os.getpid(), "parent_creation_time": 123.5})
        dump(path / "fluent_process.json", {"job_id": slot.JOB_ID, "pid": 600, "creation_time": 600.5,
            "parent_pid": 500, "executable": executable})
        evidence = ready(); dump(path / "readiness.json", evidence)
        dump(path / "native_result.json", {"job_id": slot.JOB_ID, "batch_id": b.BATCH, "slot": slot.slot,
            "provenance": "live_fluent_batch002", "launch_uncertain": False, "mesh_generated": False,
            "flow_iterations": 0, "commands": {}, "errors": {}, "readiness": evidence,
            "stdin_lifecycle": {"mode": "private_subprocess_PIPE", "bytes_written": 0, "close_requested_in_cleanup": True,
                "pipe_returned": True, "writer_closed": True, "late_return_after_close_request": False},
            "stdin_close_phase": "after_bounded_session_exit_request",
            "readiness_sha256": legacy.sha256(path / "readiness.json"),
            "transcript_sha256": legacy.sha256(path / "native.transcript.txt")})
        native = legacy.read_json(path / "native_result.json")
        complete_record_fixture(slot, path, native, {})
        dump(path / "native_result.json", native)
        return types.SimpleNamespace(pid=500, poll=lambda: 0)
    class Tree:
        def __init__(self, *a): self.members = {500: 500.5, 600: 600.5}; self.errors = []
        def capture(self): pass
        def live(self): return []
        def close(self): return True
    def identity(pid=None):
        return types.SimpleNamespace(create_time=lambda: {500: 500.5, 600: 600.5}.get(pid, 123.5),
            name=lambda: "fluent.exe" if pid == 600 else "python.exe", exe=lambda: executable if pid == 600 else "offline-python")
    monkeypatch.setattr(w.subprocess, "Popen", launch)
    monkeypatch.setattr(w, "OwnedProcessTree", Tree)
    monkeypatch.setattr(w.psutil, "Process", identity)
    final = w.run(path, stage_root=root)
    assert final["status"] == "completed" and final["owned_cleanup_verified"] is True
    assert final["serial_dispatch_released"] is True
    assert not (root / ".ansys-task.lock").exists() and not (root / ".mesh-inspection-dispatch.lock").exists()
    assert b.Batch002Slot("arm", root).connection_gate()["status"] == "passed"


def test_new_native_command_and_api_audit():
    source = (STAGE / "inspection_batch002.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    native = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "native_collect")
    calls = [ast.unparse(x.func) for x in ast.walk(native) if isinstance(x, ast.Call)]
    assert calls.count("subprocess.Popen") == 1
    assert "connect_ready" in calls
    assert "launch_fluent" not in source and "connect_to_fluent" not in source
    body = ast.get_source_segment(source, native)
    assert 'if not slot.is_connection:\n            collect_mesh' in body
    assert 'REMOTING_SERVER_ADDRESS="127.0.0.1"' in source
    for danger in ("iterate(", "initialize(", "write_mesh(", "GenerateMesh", "read_case(", "find_remoting_ip("):
        assert danger not in source
    connection = (STAGE / "inspection_batch002_connection.py").read_text(encoding="utf-8")
    queries = [x.value for x in ast.walk(ast.parse(connection)) if isinstance(x, ast.Constant) and isinstance(x.value, str) and x.value.startswith("(cx-")]
    assert set(queries) == {"(cx-solver-mode?)", "(cx-version)"}


@pytest.mark.parametrize("name", [b.SUPPLEMENT, b.STDIN_VALIDATION, "background", "arm"])
def test_native_adapter_single_popen_and_connection_only_scope(batch, monkeypatch, name):
    root, _, _ = batch
    slot = b.execution_slot(name, root)
    path, _ = reserve(root, slot)
    parent_pid = os.getppid()
    dump(path / "launcher.json", {"job_id": slot.JOB_ID, "pid": parent_pid, "creation_time": 123.5})
    dump(root / ".ansys-task.lock", {"pid": parent_pid, "attempt": str(path)})
    monkeypatch.setenv("REMOTING_SERVER_ADDRESS", "127.0.0.1")
    monkeypatch.setattr(slot.__class__, "connection_gate", lambda self: {"status": "passed"})
    exe = root / "offline-never-execute.exe"; exe.write_bytes(b"OFFLINE ONLY")
    monkeypatch.setattr(w, "validate_frozen", lambda *a: {"contract": {"software": {"fluent": str(exe), "ansys_root": str(root)}}})
    monkeypatch.setattr(w.psutil, "Process", lambda pid=None: types.SimpleNamespace(pid=os.getpid() if pid is None else pid,
        create_time=lambda: 123.5, ppid=lambda: os.getpid(), exe=lambda: str(exe)))
    popen = mock.Mock(return_value=types.SimpleNamespace(pid=501, stdin=types.SimpleNamespace(closed=False, close=lambda: None), poll=lambda: None, wait=lambda **k: 0))
    monkeypatch.setattr(w.subprocess, "Popen", popen)
    calls = []
    def record(label):
        def action(*a, **k):
            calls.append(label)
            if label == "transcript_start": (path / "native.transcript.txt").write_text("Minimum Orthogonal Quality = 0.02\nminimum volume: 1e-9\n", encoding="utf-8")
        return action
    class Walls:
        def get_object_names(self): return b.MESHES[name]["walls"]
        def __getitem__(self, key): return types.SimpleNamespace(adjacent_cell_zone=lambda: b.MESHES[name]["fluid"])
    solver = types.SimpleNamespace(file=types.SimpleNamespace(read_mesh=record("read_mesh")),
        tui=types.SimpleNamespace(file=types.SimpleNamespace(start_transcript=record("transcript_start"), stop_transcript=record("transcript_stop")),
            mesh=types.SimpleNamespace(quality=record("quality"), check=record("check"), modify_zones=types.SimpleNamespace(list_zones=record("zones")))),
        setup=types.SimpleNamespace(boundary_conditions=types.SimpleNamespace(wall=Walls()),
            cell_zone_conditions=types.SimpleNamespace(fluid=types.SimpleNamespace(get_object_names=lambda: [b.MESHES[name]["fluid"]]))), exit=record("exit"))
    connects = []
    def connect(*a, **kwargs):
        assert kwargs == {"normal_exit": True}
        connects.append(True); a[3].update(ready()); return solver
    monkeypatch.setattr(w, "connect_ready", connect)
    result = w.native_collect(path, stage_root=root)
    assert popen.call_count == len(connects) == 1
    assert "OFFLINE-SECRET" not in json.dumps(result)
    if slot.is_connection:
        assert "read_mesh" not in calls and "quality" not in calls and result["commands"] == {}
        assert "mesh_evidence" not in result
    else:
        assert calls.index("quality") < calls.index("check")
        assert result["inspection"]["inspection_gate_passed"] is True


def test_same_batch_protected_input_change_blocks_gate(batch):
    root, _, popen = batch
    completed_connection(root)
    manifest = legacy.read_json(root / "frozen/input_manifest.json")
    external = next(row for row in manifest["files"] if row["path"].endswith(".step"))
    (root.parent.parent / external["path"]).write_bytes(b"changed STEP offline fixture")
    slot = b.Batch002Slot("arm", root)
    with pytest.raises(legacy.BridgeError): slot.start(slot.START_CONFIRMATION)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_old_twelve_tools_and_new_routes_preserved():
    tree = ast.parse((STAGE / "stage15_mcp.py").read_text(encoding="utf-8"))
    names = {x.name for x in tree.body if isinstance(x, ast.AsyncFunctionDef)}
    assert len(names) == 20
    assert {"stage15_supplement_connection_preflight", "stage15_start_supplement_connection"} <= names
    assert {"stage15_mesh_inspection_preflight", "stage15_start_mesh_inspection", "stage15_batch002_preflight", "stage15_start_batch002"} <= names
    protocol = (STAGE / "check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_batch002", {"slot": "connection", "confirmation": "NO"})' in protocol
    assert b.START not in protocol
    assert b.batch_job("inspection-b002-connection").slot == "connection"
    with pytest.raises(legacy.BridgeError): b.batch_job("mesh-inspection-001-arm")

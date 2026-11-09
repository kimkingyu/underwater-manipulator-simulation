"""Offline fakes only. No Fluent, Workbench, real worker or license launch."""
import ast
import copy
import concurrent.futures
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
import mesh_inspection_mcp_bridge as bridge
import mesh_inspection as worker
import mesh_inspection_evidence as evidence
from test_definition_probe import sandbox, dump


@pytest.fixture
def inspection(sandbox, monkeypatch):
    stage, _, _ = sandbox
    for name in bridge.SOURCES:
        path = stage / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_text("# offline inspection fixture\n", encoding="utf-8")
    for domain, original in list(bridge.MESHES.items()):
        row = copy.deepcopy(original)
        path = stage / row["path"]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("OFFLINE ONLY " + domain).encode())
        row.update(size=path.stat().st_size, sha256=legacy.sha256(path))
        monkeypatch.setitem(bridge.MESHES, domain, row)
    baseline = stage / bridge.AUTH_VALUES["baseline_authorization_relative"]
    dump(baseline, {"scope": "OFFLINE baseline only; never a live authorization"})
    monkeypatch.setitem(bridge.AUTH_VALUES, "baseline_authorization_sha256", legacy.sha256(baseline))
    slot = bridge.InspectionSlot("background", stage)
    manifest = {"schema_version": 1, "files": slot._sources()}
    dump(stage / bridge.MANIFEST, manifest)
    grant = copy.deepcopy(bridge.AUTH_VALUES)
    grant.update(authorization_source="OFFLINE TEST ONLY", recorded_local="fixture", execution_policy="never execute native",
                 source_manifest_sha256=legacy.sha256(stage / bridge.MANIFEST))
    dump(stage / bridge.GRANT, grant)
    monkeypatch.setattr(bridge.InspectionSlot, "_python", lambda self: "offline-python")
    monkeypatch.setattr(bridge.InspectionSlot, "preflight", lambda self: self.authorize())
    monkeypatch.setattr(bridge.InspectionSlot, "_inspect_pid", lambda self, pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    popen = mock.Mock(return_value=types.SimpleNamespace(pid=123))
    monkeypatch.setattr(legacy.subprocess, "Popen", popen)
    return stage, slot, popen


def reserve_native(stage, slot):
    attempt = slot.safe(slot.ATTEMPT)
    attempt.mkdir(parents=True)
    request = slot.make_request(time.time(), slot._sources())
    dump(attempt / "request.json", request)
    dump(attempt / "launcher.json", {"job_id": slot.JOB_ID, "pid": os.getpid(), "creation_time": 123.5,
                                    "recorded_unix": time.time()})
    return attempt, request


def native_good(domain="background"):
    return {"provenance": "live_fluent_read_only", "metrics": evidence.parse_native_output(
                "Minimum Orthogonal Quality = 1.0e-02\nminimum volume (m3): 1e-12\n"),
            "commands": {"read_mesh": "completed", "quality": "completed", "mesh_check": "completed"},
            "fluid_zones": [bridge.MESHES[domain]["fluid"]],
            "wall_adjacency": {name: {"native_query_completed": True, "adjacent_cell_zone": bridge.MESHES[domain]["fluid"]}
                               for name in bridge.MESHES[domain]["walls"]}}


@pytest.mark.parametrize("domain", ["background", "arm"])
def test_domain_once_and_request_bounds(inspection, domain):
    stage, _, popen = inspection
    slot = bridge.InspectionSlot(domain, stage)
    old_request = stage / "runs/mcp_definition_probe/attempt_001/request.json"
    old_bytes = old_request.read_bytes()
    value = slot.start(bridge.START_TOKEN)
    assert value["new_worker_started"] is True
    request = legacy.read_json(slot.safe(slot.ATTEMPT + "/request.json"))
    assert request["domain"] == domain
    assert request["mesh"] == bridge.MESHES[domain]
    assert request["deadline_unix"] - request["created_unix"] == 300
    assert request["work_deadline_unix"] - request["created_unix"] == 270
    assert slot.start(bridge.START_TOKEN)["new_worker_started"] is False
    assert popen.call_count == 1
    assert not (stage / "runs/mcp_mesh_construction").exists()
    assert old_request.read_bytes() == old_bytes
    archive = legacy.read_json(slot.safe(slot.ATTEMPT + "/source_snapshot/archive_manifest.json"))
    assert len(archive["files"]) == len(bridge.SOURCES) + 1


def test_old_background_receipt_remains_bound_to_original_grant_and_consumed(inspection):
    stage, slot, popen = inspection
    slot.start(bridge.START_TOKEN)
    request_path = slot.safe(slot.ATTEMPT + "/request.json")
    request = legacy.read_json(request_path)
    request["authorization"]["path"] = str(stage / bridge.AUTH_VALUES["baseline_authorization_relative"])
    dump(request_path, request)
    original = {"job_id": slot.JOB_ID, "domain": slot.domain, "operation": slot.OPERATION,
        "status": "cleanup_unverified", "owned_cleanup_verified": False,
        "sources_reverified_after_cleanup": True, "inputs_reverified_after_cleanup": True,
        "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False}
    final_path = slot.safe(slot.ATTEMPT + "/probe_result.json")
    dump(final_path, original)
    before = final_path.read_bytes()
    with (stage / "mesh_inspection.py").open("a") as out: out.write("# offline repair\n")
    result = slot.start(bridge.START_TOKEN)
    assert result["status"] == "already_reserved" and popen.call_count == 1
    assert result["job"]["historical_receipt_not_rebound"] is True
    assert result["job"]["slot_sources_match_current_code"] is False
    assert result["job"]["status"] == "cleanup_unverified"
    assert final_path.read_bytes() == before
    assert bridge.InspectionSlot("arm", stage).status()["remaining_inspections"] == 1


def test_original_authorization_cannot_change(inspection):
    stage, slot, popen = inspection
    (stage / bridge.AUTH_VALUES["baseline_authorization_relative"]).write_bytes(b"changed baseline")
    with pytest.raises(legacy.BridgeError, match="Original read-only grant"):
        slot.start(bridge.START_TOKEN)
    popen.assert_not_called()


def test_two_distinct_slots_no_third_domain(inspection):
    stage, slot, popen = inspection
    slot.start(bridge.START_TOKEN)
    bridge.InspectionSlot("arm", stage).start(bridge.START_TOKEN)
    assert popen.call_count == 2
    with pytest.raises(legacy.BridgeError):
        bridge.InspectionSlot("combined", stage)


@pytest.mark.parametrize("confirmation", ["", "NO", "RUN_STAGE15_FULL_MESH_CONSTRUCTION_ONCE", bridge.CANCEL_TOKEN])
def test_bad_confirmation_never_launches(inspection, confirmation):
    stage, slot, popen = inspection
    with pytest.raises(legacy.BridgeError):
        slot.start(confirmation)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("key,value", [("maximum_attempts", 3), ("maximum_attempts_per_domain", 2),
    ("maximum_total_seconds_per_domain", 301), ("mesh_generation_allowed", True), ("mesh_write_allowed", True),
    ("iterations_allowed", True), ("motion_allowed", True), ("overset_assembly_allowed", True),
    ("initialization_allowed", True), ("UDF_allowed", True), ("automatic_retry_allowed", True),
    ("CFD_allowed", True), ("maximum_attempts", True), ("extra_argument", "arbitrary Python"),
    ("additional_attempts_authorized", 1), ("authorization_revision", 3)])
def test_grant_scope_cannot_expand(inspection, key, value):
    stage, slot, popen = inspection
    grant = legacy.read_json(stage / bridge.GRANT)
    grant[key] = value
    dump(stage / bridge.GRANT, grant)
    with pytest.raises(legacy.BridgeError):
        slot.start(bridge.START_TOKEN)
    popen.assert_not_called()


@pytest.mark.parametrize("relative", [*bridge.SOURCES, bridge.MANIFEST,
                                      "runs/mesh/attempt_004/background.msh", "runs/mesh/attempt_004/arm.msh"])
def test_source_and_mesh_changes_refuse_start(inspection, relative):
    stage, slot, popen = inspection
    with (stage / relative).open("ab") as out:
        out.write(b"changed")
    with pytest.raises(legacy.BridgeError):
        slot.start(bridge.START_TOKEN)
    popen.assert_not_called()


@pytest.mark.parametrize("lock", [".ansys-task.lock", ".sizing-dispatch.lock"])
def test_other_locks_preserved(inspection, lock):
    stage, slot, popen = inspection
    dump(stage / lock, {"token": "someone else"})
    with pytest.raises(legacy.BridgeError):
        slot.start(bridge.START_TOKEN)
    assert legacy.read_json(stage / lock)["token"] == "someone else"
    popen.assert_not_called()


def test_concurrent_same_slot_has_single_launch(inspection):
    stage, slot, popen = inspection
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        answers = list(pool.map(lambda _: slot.start(bridge.START_TOKEN), range(4)))
    assert sum(x.get("new_worker_started") is True for x in answers) == 1
    assert popen.call_count == 1


def test_uncertain_worker_start_consumes_slot_and_cancel_cannot_kill_unknown(inspection, monkeypatch):
    stage, slot, popen = inspection
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: (_ for _ in ()).throw(RuntimeError("unknown")))
    answer = slot.start(bridge.START_TOKEN)
    assert answer["status"] == "cleanup_unverified"
    assert (stage / ".mesh-inspection-dispatch.lock").exists()
    assert slot.safe(slot.ATTEMPT + "/cancel_request.json").exists()
    assert slot.start(bridge.START_TOKEN)["status"] == "already_reserved"
    assert popen.call_count == 1
    result = slot.cancel(slot.JOB_ID, bridge.CANCEL_TOKEN)
    assert result["lock_must_be_preserved"] is True


def test_bounded_worker_launch_without_pid_retains_dispatch(inspection, monkeypatch):
    stage, slot, popen = inspection
    monkeypatch.setattr(slot, "launch_worker", lambda *a, **k: (_ for _ in ()).throw(legacy.WorkerLaunchUncertain("OFFLINE timeout")))
    result = slot.start(bridge.START_TOKEN)
    assert result["status"] == "cleanup_unverified"
    assert result["owned_cleanup_verified"] is False
    assert (stage / ".mesh-inspection-dispatch.lock").exists()
    assert slot.safe(slot.ATTEMPT + "/cancel_request.json").exists()
    assert slot.start(bridge.START_TOKEN)["new_worker_started"] is False
    popen.assert_not_called()


def test_cancel_is_fixed_sentinel_and_no_process_kill(inspection):
    _, slot, _ = inspection
    slot.start(bridge.START_TOKEN)
    with pytest.raises(legacy.BridgeError):
        slot.cancel(slot.JOB_ID, "NO")
    assert slot.cancel(slot.JOB_ID, bridge.CANCEL_TOKEN)["status"] == "cancellation_requested"
    assert legacy.read_json(slot.safe(slot.ATTEMPT + "/cancel_request.json"))["operation"] == "cancel_mesh_inspection"
    with pytest.raises(legacy.BridgeError):
        slot.job_log(slot.JOB_ID, "../contract.json")


@pytest.mark.parametrize("value,passed", [(0.01, True), (0.00999, False), (0, False), (-1, False),
    (1, True), (1.01, False), (None, False), (float("nan"), False), (float("inf"), False), (True, False)])
def test_orthogonal_quality_threshold(value, passed):
    native = native_good()
    native["metrics"]["minimum_orthogonal_quality"] = value
    assert evidence.validate_native_evidence(native, "background")["quality_verified"] is passed


@pytest.mark.parametrize("text", ["", "Maximum Skewness = 0.001", "Minimum Orthogonal Quality = NaN",
                                  "Minimum Orthogonal Quality = inf", "Minimum Orthogonal Quality = 0.1\nMinimum Orthogonal Quality = NaN"])
def test_missing_nonfinite_or_skewness_not_quality(text):
    assert evidence.parse_native_output(text)["minimum_orthogonal_quality"] is None


@pytest.mark.parametrize("volume,negative,passed", [(1e-12, None, True), (0, 0, False), (-1e-12, 2, False),
    (float("nan"), 0, False), (float("inf"), 0, False), (None, 0, False), (1e-12, 1, False), (True, 0, False)])
def test_positive_volume_and_negative_count(volume, negative, passed):
    native = native_good()
    native["metrics"].update(minimum_cell_volume=volume, reported_negative_volume_count=negative)
    assert evidence.validate_native_evidence(native, "background")["zero_negative_volumes_verified"] is passed


@pytest.mark.parametrize("domain", ["background", "arm"])
def test_native_gate_good_fixture_is_offline_only(domain):
    result = evidence.validate_native_evidence(native_good(domain), domain)
    assert result["inspection_gate_passed"] is True
    assert result["overset_connectivity_verified"] is False
    assert result["mesh_motion_verified"] is False
    assert result["CFD_validated"] is False


@pytest.mark.parametrize("mutation", ["missing_wall", "missing_fluid", "no_query", "wrong_adjacent", "name_only", "mock", "check_error"])
def test_topology_or_mock_cannot_pass(mutation):
    native = native_good()
    wall = bridge.MESHES["background"]["walls"][0]
    if mutation == "missing_wall": native["wall_adjacency"].pop(wall)
    if mutation == "missing_fluid": native["fluid_zones"] = []
    if mutation == "no_query": native["wall_adjacency"][wall]["native_query_completed"] = False
    if mutation == "wrong_adjacent": native["wall_adjacency"][wall]["adjacent_cell_zone"] = "solid"
    if mutation == "name_only": native["wall_adjacency"][wall] = wall
    if mutation == "mock": native["provenance"] = "mock"
    if mutation == "check_error": native["metrics"]["native_error_reported"] = True
    assert evidence.validate_native_evidence(native, "background")["inspection_gate_passed"] is False


def test_service_does_not_promote_fixture_without_native_files(inspection):
    stage, slot, _ = inspection
    slot.start(bridge.START_TOKEN)
    dump(slot.safe(slot.ATTEMPT + "/probe_result.json"), {"job_id": slot.JOB_ID, "operation": slot.OPERATION,
         "domain": slot.domain, "status": "completed", "native": native_good(), "owned_cleanup_verified": True,
         "inputs_reverified_after_cleanup": True, "sources_reverified_after_cleanup": True})
    assert slot.job_status(slot.JOB_ID)["inspection"]["inspection_gate_passed"] is False


def test_raw_errors_and_aggregate_parser():
    text = "Minimum Orthogonal Quality = 2.0e-02\nminimum volume (m3): 1e-9\nnegative cells: 0\nError: unassigned overset\n"
    parsed = evidence.parse_native_output(text)
    assert parsed["minimum_cell_volume"] == 1e-9
    assert parsed["reported_negative_volume_count"] == 0
    assert parsed["native_error_reported"] is True
    with pytest.raises(ValueError):
        evidence.parse_native_output("x" * (2 * 1024 * 1024 + 1))


def test_worker_uncertain_popen_retains_lock(inspection, monkeypatch):
    stage, slot, popen = inspection
    attempt, _ = reserve_native(stage, slot)
    monkeypatch.setattr(worker.subprocess, "Popen", lambda *a, **k: (_ for _ in ()).throw(worker.UncertainCall("blocked launch")))
    result = worker.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert (stage / ".ansys-task.lock").exists()
    assert legacy.read_json(attempt / "probe_result.json")["owned_cleanup_verified"] is False


def test_worker_cancel_before_launch_keeps_inputs(inspection):
    stage, slot, popen = inspection
    attempt, _ = reserve_native(stage, slot)
    dump(attempt / "cancel_request.json", {"job_id": slot.JOB_ID, "operation": slot.CANCEL_OPERATION})
    result = worker.run(attempt, stage_root=stage)
    assert result["status"] == "cancelled"
    assert result["owned_cleanup_verified"] is True
    popen.assert_not_called()


@pytest.mark.parametrize("field,value", [("domain", "arm"), ("operation", "mesh_construction"),
    ("deadline_unix", float("nan")), ("work_deadline_unix", 0), ("extra", "arbitrary TUI")])
def test_worker_request_whitelist(inspection, field, value):
    stage, slot, _ = inspection
    attempt, request = reserve_native(stage, slot)
    request[field] = value
    dump(attempt / "request.json", request)
    with pytest.raises(worker.RuntimeGateError):
        worker.validate_request(slot, attempt)


@pytest.mark.parametrize("cleanup_ok", [True, False])
def test_worker_270_deadline_uses_cleanup_and_retains_uncertain_lock(inspection, monkeypatch, cleanup_ok):
    stage, slot, _ = inspection
    attempt, _ = reserve_native(stage, slot)
    clock = types.SimpleNamespace(now=100.0)
    monkeypatch.setattr(worker.time, "monotonic", lambda: clock.now)
    events = []
    process = types.SimpleNamespace(pid=500, poll=lambda: None)
    def launch(*a, **k):
        return process
    class Tree:
        def __init__(self, *a):
            clock.now = 371.0
            self.members = {500: 123.5}
            self.errors = []
        def capture(self): events.append("capture")
        def close(self):
            events.append("owned_close")
            return cleanup_ok
    monkeypatch.setattr(worker.subprocess, "Popen", launch)
    monkeypatch.setattr(worker, "OwnedProcessTree", Tree)
    result = worker.run(attempt, stage_root=stage)
    assert "270-second work deadline" in result["error"]
    assert "owned_close" in events
    assert result["elapsed_seconds"] >= 270
    assert result["owned_cleanup_verified"] is cleanup_ok
    assert (stage / ".ansys-task.lock").exists() is not cleanup_ok
    assert (attempt / "probe_result.json").exists()


def test_native_adapter_refuses_direct_shell_parent(inspection, monkeypatch):
    stage, slot, popen = inspection
    attempt, _ = reserve_native(stage, slot)
    dump(stage / ".ansys-task.lock", {"pid": 11, "attempt": str(attempt)})
    monkeypatch.setattr(worker.psutil, "Process", lambda pid: types.SimpleNamespace(pid=999, create_time=lambda: 123.5))
    with pytest.raises(worker.RuntimeGateError, match="verified owning worker"):
        worker.native_collect(attempt, stage_root=stage)
    assert not (attempt / "fluent_launch_started.json").exists()
    popen.assert_not_called()


@pytest.mark.parametrize("fault", [None, "read_mesh", "quality", "mesh_check", "adjacency_format", "native_start_timeout", "connect_secret_error"])
def test_fixed_native_adapter_with_fake_solver_preserves_independent_evidence(inspection, monkeypatch, fault):
    stage, slot, _ = inspection
    attempt, _ = reserve_native(stage, slot)
    parent_pid = os.getppid()
    dump(attempt / "launcher.json", {"job_id": slot.JOB_ID, "pid": parent_pid, "creation_time": 123.5})
    dump(stage / ".ansys-task.lock", {"pid": parent_pid, "attempt": str(attempt)})
    fake_exe = stage / "offline_fluent.exe"
    fake_exe.write_bytes(b"OFFLINE FAKE; NEVER EXECUTE")
    monkeypatch.setattr(worker.psutil, "Process", lambda pid=None: types.SimpleNamespace(
        pid=os.getpid() if pid is None else pid, create_time=lambda: 123.5,
        ppid=lambda: os.getpid(), exe=lambda: str(fake_exe)))
    monkeypatch.setattr(worker, "validate_frozen", lambda root: {"contract": {"software": {
        "fluent": str(fake_exe), "ansys_root": str(stage)}}})
    native_launches = []
    def fake_popen(*a, **k):
        native_launches.append(a[0])
        if fault == "native_start_timeout":
            raise worker.UncertainCall("OFFLINE FAKE native startup timeout")
        (attempt / "server.info").write_bytes(b"127.0.0.1:12345\nOFFLINE-FAKE-SECRET\n")
        return types.SimpleNamespace(pid=501, poll=lambda: None)
    monkeypatch.setattr(worker.subprocess, "Popen", fake_popen)
    calls = []
    transcript = attempt / "native.transcript.txt"
    def command(name):
        def execute(*a, **k):
            calls.append(name)
            if name == "start_transcript": transcript.write_text("", encoding="utf-8")
            if name == "quality" and fault != name:
                with transcript.open("a", encoding="utf-8") as out:
                    out.write("Minimum Orthogonal Quality = 0.02\n")
            if name == "mesh_check":
                with transcript.open("a", encoding="utf-8") as out:
                    out.write("minimum volume (m3): 1e-12\n" + ("Error: unassigned overset\n" if fault == name else ""))
            if fault == name: raise RuntimeError("OFFLINE FAKE " + name)
        return execute
    class Walls:
        def get_object_names(self): return bridge.MESHES[slot.domain]["walls"]
        def __getitem__(self, name):
            return types.SimpleNamespace(adjacent_cell_zone=lambda: ["unexpected format"] if fault == "adjacency_format" else bridge.MESHES[slot.domain]["fluid"])
    solver = types.SimpleNamespace(
        file=types.SimpleNamespace(read_mesh=command("read_mesh")),
        tui=types.SimpleNamespace(file=types.SimpleNamespace(start_transcript=command("start_transcript"), stop_transcript=command("stop_transcript")),
            mesh=types.SimpleNamespace(quality=command("quality"), check=command("mesh_check"), modify_zones=types.SimpleNamespace(list_zones=command("list_zones")))),
        setup=types.SimpleNamespace(boundary_conditions=types.SimpleNamespace(wall=Walls()),
            cell_zone_conditions=types.SimpleNamespace(fluid=types.SimpleNamespace(get_object_names=lambda: [bridge.MESHES[slot.domain]["fluid"]]))),
        exit=command("exit"))
    launch = mock.Mock(return_value=solver)
    if fault == "connect_secret_error":
        launch.side_effect = RuntimeError("OFFLINE-FAKE-SECRET")
    ansys = types.ModuleType("ansys"); ansys.__path__ = []
    fluent = types.ModuleType("ansys.fluent"); fluent.__path__ = []
    core = types.ModuleType("ansys.fluent.core"); core.connect_to_fluent = launch
    monkeypatch.setitem(sys.modules, "ansys", ansys)
    monkeypatch.setitem(sys.modules, "ansys.fluent", fluent)
    monkeypatch.setitem(sys.modules, "ansys.fluent.core", core)
    native = worker.native_collect(attempt, stage_root=stage)
    assert len(native_launches) == 1
    assert (attempt / "native_result.json").exists()
    if fault == "native_start_timeout":
        assert launch.call_count == 0
        assert native["launch_uncertain"] is True
        assert native["inspection"]["inspection_gate_passed"] is False
        return
    assert launch.call_count == 1
    assert launch.call_args.kwargs["password"] == "OFFLINE-FAKE-SECRET"
    assert "server_info_file_name" not in launch.call_args.kwargs
    assert "OFFLINE-FAKE-SECRET" not in json.dumps(native)
    if fault == "connect_secret_error":
        assert native["safe_error_classification"]["exception_text_omitted"] is True
        assert native["launch_uncertain"] is True
        return
    assert calls[-1] == "exit"
    if fault == "read_mesh":
        assert "quality" not in calls
    else:
        assert calls.index("quality") < calls.index("mesh_check")
    if fault == "mesh_check":
        assert native["inspection"]["quality_verified"] is True
        assert native["inspection"]["inspection_gate_passed"] is False
        assert "OFFLINE FAKE" in native["errors"]["mesh_check"]
    if fault == "adjacency_format":
        assert native["inspection"]["wall_fluid_adjacency_verified"] is False
    if fault is None:
        assert native["inspection"]["inspection_gate_passed"] is True


def test_native_api_ast_is_read_only_and_quality_precedes_check():
    source = (STAGE / "mesh_inspection.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    native = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "native_collect")
    calls = [ast.unparse(x.func) for x in ast.walk(native) if isinstance(x, ast.Call)]
    solver_calls = {x for x in calls if x.startswith("solver.")}
    allowed = {"solver.tui.file.start_transcript", "solver.file.read_mesh", "solver.tui.mesh.quality",
               "solver.setup.cell_zone_conditions.fluid.get_object_names", "solver.setup.boundary_conditions.wall.get_object_names",
               "solver.setup.boundary_conditions.wall[name].adjacent_cell_zone", "solver.tui.mesh.modify_zones.list_zones",
               "solver.tui.mesh.check", "solver.tui.file.stop_transcript", "solver.exit"}
    assert solver_calls == allowed
    body = ast.get_source_segment(source, native)
    assert body.index('step("quality"') < body.index('step("mesh_check"')
    for forbidden in ("scheme_eval", "exec(", "eval(", "iterate(", "initialize(", "read_case(", "write_mesh(", "GenerateMesh"):
        assert forbidden not in body
    assert "pyfluent.launch_fluent" not in calls
    connect = next(x for x in ast.walk(native) if isinstance(x, ast.Call) and ast.unparse(x.func) == "pyfluent.connect_to_fluent")
    kwargs = {x.arg: ast.literal_eval(x.value) for x in connect.keywords if x.arg not in ("ip", "port", "password")}
    assert {x.arg for x in connect.keywords} == {"ip", "port", "password", "cleanup_on_exit", "start_watchdog", "start_transcript"}
    assert kwargs["start_watchdog"] is False and kwargs["cleanup_on_exit"] is False
    assert kwargs["start_transcript"] is False
    popens = [x for x in ast.walk(native) if isinstance(x, ast.Call) and ast.unparse(x.func) == "subprocess.Popen"]
    assert len(popens) == 1
    assert any(x.arg == "shell" and ast.literal_eval(x.value) is False for x in popens[0].keywords)
    assert '[str(fluent_exe), "3ddp", "-g", "-t1", "-sifile=" + str(server_info), "-nm"]' in body


def test_server_schema_old_ten_and_new_routes_are_preserved():
    source = (STAGE / "stage15_mcp.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    tools = {x.name for x in tree.body if isinstance(x, ast.AsyncFunctionDef)}
    old = {"stage15_status", "stage15_preflight", "stage15_start_definition_probe", "stage15_job_status",
           "stage15_job_log", "stage15_cancel_job", "stage15_sizing_preflight", "stage15_start_sizing_diagnostic",
           "stage15_mesh_preflight", "stage15_start_mesh_construction"}
    assert tools == old | {"stage15_mesh_inspection_preflight", "stage15_start_mesh_inspection",
                           "stage15_batch002_preflight", "stage15_start_batch002",
                           "stage15_supplement_connection_preflight", "stage15_start_supplement_connection",
                           "stage15_stdin_validation_preflight", "stage15_start_stdin_validation",
                           "stage15_full_record_preflight", "stage15_start_full_record_validation",
                           "stage15_review_license_classification", "stage15_local_quality_preflight",
                           "stage15_start_local_quality_diagnostic", "stage15_fixed_refinement_preflight",
                           "stage15_start_fixed_refinement_construction", "stage15_start_fixed_refinement_QA"}
    assert "return inspection_job(job_id)" in source
    check = (STAGE / "check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_mesh_inspection", {"domain": "arm", "confirmation": "NO"})' in check
    assert bridge.START_TOKEN not in check
    assert bridge.inspection_job("mesh-inspection-001-arm").domain == "arm"
    with pytest.raises(legacy.BridgeError): bridge.inspection_job("../../bad")

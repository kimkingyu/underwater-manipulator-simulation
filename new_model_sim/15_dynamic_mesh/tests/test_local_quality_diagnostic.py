"""New two slots and bounded native evidence; every process/session is a fake."""
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
import local_quality_diagnostic as l
import local_quality_evidence as e
import inspection_batch002 as w
import inspection_batch002_bridge as b
import mcp_bridge as legacy
from test_definition_probe import sandbox, dump
from test_inspection_batch002 import batch, reserve, ready
from test_native_stdin_lifetime import FakePipe


def transcript(domain="arm", q=0.000214865):
    text = "Minimum Orthogonal Quality = %s cell 115539 on zone 2 (ID: 842319 on partition: 0) at location (-.0153238, -.170113, -.349268)\n" % q
    text += "2 fluid_" + domain + " fluid air cell\n"
    text += "minimum volume (m3): 1.172144e-12\nmaximum volume (m3): 1.548054e-6\n"
    if q < .01: text += "Warning: minimum Orthogonal Quality below 0.01.\n"
    if domain == "arm": text += "WARNING: overset zone 5 does not belong to any overset interface.\nWARNING: Mesh check failed.\n"
    return text


@pytest.fixture
def local(batch, monkeypatch):
    root, _, popen = batch
    for relative in l.SOURCES:
        p = root / relative; p.parent.mkdir(parents=True, exist_ok=True)
        if not p.exists(): p.write_text("# offline local source\n", encoding="utf-8")
    slot = l.LocalQualitySlot("background", root)
    dump(root / l.MANIFEST, {"schema_version": 1, "files": slot._sources()})
    grant = copy.deepcopy(l.POLICY)
    grant.update(authorization_source="OFFLINE fake only", recorded_local="offline", execution_policy="No native execution",
                 source_manifest_sha256=legacy.sha256(root / l.MANIFEST))
    dump(root / l.GRANT, grant)
    monkeypatch.setattr(l.LocalQualitySlot, "preflight", lambda self: self.authorize())
    return root, slot, popen


def test_two_new_persistent_slots_do_not_reuse_old_QA(local, monkeypatch):
    root, _, popen = local
    monkeypatch.setattr(l.LocalQualitySlot, "_inspect_pid", lambda self, pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    for name in ("background", "arm"):
        slot = l.LocalQualitySlot(name, root)
        assert slot.ATTEMPT == "runs/mcp_local_quality/batch_001/" + name
        assert slot.JOB_ID == "local-quality-001-" + name
        assert slot.start(l.START)["new_worker_started"] is True
        assert slot.start(l.START)["new_worker_started"] is False and slot.status()["remaining_attempts"] == 0
        request = legacy.read_json(slot.safe(slot.ATTEMPT + "/request.json"))
        assert request["batch_id"] == l.BATCH_ID and request["mesh"] == l.MESHES[name]
        assert request["deadline_unix"] - request["created_unix"] == 300
        assert request["work_deadline_unix"] - request["created_unix"] == 270
        assert not b.Batch002Slot(name, root).safe(b.Batch002Slot(name, root).ATTEMPT).exists()
        (root / ".mesh-inspection-dispatch.lock").unlink()  # fake worker finishes between slots
    assert popen.call_count == 2 and l.POLICY["maximum_attempts"] == 2


@pytest.mark.parametrize("token", ["NO", b.START, b.FULL_RECORD_START, b.CANCEL, l.CANCEL])
def test_wrong_token_never_consumes_new_slot(local, token):
    root, slot, popen = local
    with pytest.raises(legacy.BridgeError): slot.start(token)
    assert not slot.safe(slot.ATTEMPT).exists(); popen.assert_not_called()


@pytest.mark.parametrize("domain", ["connection", "all", "background2", "../arm", "arm_retry"])
def test_no_new_aliases_or_extra_slots(local, domain):
    with pytest.raises(legacy.BridgeError): l.LocalQualitySlot(domain, local[0])


@pytest.mark.parametrize("lock", [".ansys-task.lock", ".mesh-inspection-dispatch.lock", ".sizing-dispatch.lock"])
def test_serial_lock_refusal_preserves_bytes_and_budget(local, lock):
    root, slot, popen = local
    dump(root / lock, {"owner": "other"}); before = (root / lock).read_bytes()
    with pytest.raises(legacy.BridgeError): slot.start(l.START)
    assert (root / lock).read_bytes() == before and not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_concurrent_once_and_cancel_fixed_identity(local, monkeypatch):
    root, slot, popen = local
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    def start():
        try: return slot.start(l.START)
        except legacy.BridgeError: return {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool: results = list(pool.map(lambda _: start(), range(4)))
    assert sum(r.get("new_worker_started") is True for r in results) == popen.call_count == 1
    with pytest.raises(legacy.BridgeError): slot.cancel(slot.JOB_ID, b.CANCEL)
    assert slot.cancel(slot.JOB_ID, l.CANCEL)["status"] == "cancellation_requested"
    assert legacy.read_json(slot.safe(slot.ATTEMPT + "/cancel_request.json"))["operation"] == slot.CANCEL_OPERATION


@pytest.mark.parametrize("target", ["source", "manifest", "mesh", "input", "grant"])
def test_changed_binding_refuses_without_consumption(local, target):
    root, slot, popen = local
    p = {"source": root / "local_quality_evidence.py", "manifest": root / l.MANIFEST, "mesh": root / l.MESHES["arm"]["path"],
        "input": root / "contract.json", "grant": root / l.GRANT}[target]
    p.write_bytes(p.read_bytes() + b"changed")
    with pytest.raises((legacy.BridgeError, ValueError)): slot.start(l.START)
    assert not slot.safe(slot.ATTEMPT).exists(); popen.assert_not_called()


def test_old_QA_and_connection_paths_never_resolve_or_recreate(local, monkeypatch):
    root, slot, popen = local
    monkeypatch.setattr(b, "QA_SLOTS_RETIRED", True)
    for name in ("background", "arm"):
        old = b.Batch002Slot(name, root)
        with pytest.raises(legacy.BridgeError, match="retired"): old.start(b.START)
        assert old.status()["remaining_attempts"] == 0
        with pytest.raises(legacy.BridgeError): l.local_slot_for_attempt(old.safe(old.ATTEMPT), root)
    assert slot.connection_gate()["old_connection_or_QA_reused"] is False
    popen.assert_not_called()


@pytest.mark.parametrize("fault", ["future", "deadline", "scope", "job", "old_path", "source"])
def test_fixed_request_scope_deadline_and_sources(local, fault):
    root, slot, popen = local
    path, req = reserve(root, slot)
    if fault == "future": req.update(created_unix=time.time()+100, deadline_unix=time.time()+400)
    if fault == "deadline": req["work_deadline_unix"] += 1
    if fault == "scope": req["mesh"] = None
    if fault == "job": req["job_id"] = "inspection-b002-background"
    if fault == "old_path":
        with pytest.raises(legacy.BridgeError): l.local_slot_for_attempt(b.Batch002Slot("background", root).safe(b.Batch002Slot("background", root).ATTEMPT), root)
        return
    if fault == "source": req["sources"][0]["sha256"] = "0"*64
    dump(path / "request.json", req)
    with pytest.raises(legacy.BridgeError): l.validate_local_request(slot, path)
    popen.assert_not_called()


def test_native_minimum_is_one_record_not_top20_and_wall_not_inferred():
    result = e.extract_diagnostic(transcript(), "arm")
    assert result["status"] == "fail_quality" and result["threshold_count"]["value"] is None
    assert result["threshold_count"]["status"] == "unavailable"
    worst = result["worst_cells"]; assert worst["returned_count"] == 1 and worst["requested_maximum"] == 20
    record = worst["records"][0]
    assert record["cell_index"] == 115539 and record["native_cell_ID"] == 842319 and record["cell_zone_name"] == "fluid_arm"
    assert record["wall_assignment"]["status"] == "unavailable"
    assert result["mesh_check_failed_marker"] is True and not result["complete_topology_verified"]
    assert "WARNING: overset zone 5 does not belong to any overset interface." in result["native_warnings"]
    assert len(json.dumps(result)) < e.MAX_REPORT_BYTES and not result["whole_cell_arrays_requested"]


@pytest.mark.parametrize("text", ["", "no native minimum", transcript()+transcript(), transcript().replace(".000214865", "nan")])
def test_no_fabricated_minimum_on_missing_ambiguous_or_invalid_output(text):
    # Replacement may leave the original exponential representation unchanged;
    # explicit missing/ambiguous cases are the assertions that matter.
    if text == transcript(): text = "Minimum Orthogonal Quality = nan"
    result = e.extract_diagnostic(text, "arm")
    assert result["worst_cells"]["returned_count"] == 0 and result["status"] == "unavailable"


class Histogram:
    def __init__(self, payload=None):
        self.state = {"cell_function": "pressure", "auto_range": True, "minimum": 0., "maximum": 1., "num_divisions": 10, "all_zones": False}
        self.initial = dict(self.state); self.writes = []; self.queries = 0
        self.payload = payload if payload is not None else {"x": [.08,.25,.42,.58,.75,.92], "y": [1,2,3,4,5,6]}
        for key in self.state: setattr(self, key, types.SimpleNamespace(get_state=lambda key=key: self.state[key]))
        self.cell_function.allowed_values = lambda: ["native-orth-quality"]
    def set_state(self, value): self.writes.append(dict(value)); self.state.update(value)
    def get_values(self): self.queries += 1; return self.payload


def histogram_solver(histogram, fields=None):
    return types.SimpleNamespace(field_info=types.SimpleNamespace(get_scalar_fields_info=lambda: fields if fields is not None else {
        "native-orth-quality": {"display_name": "Orthogonal Quality"}}), results=types.SimpleNamespace(plot=types.SimpleNamespace(histogram=histogram)))


def test_only_native_confirmed_six_bin_report_query_and_restore_no_count_inference():
    h = Histogram(); result = e.query_histogram(histogram_solver(h), lambda: 100)
    assert result["status"] == "native_aggregate_uninterpreted" and not result["bin_weight_and_endpoint_semantics_verified"]
    assert result["requested_bins"] == 6 and h.queries == 1 and h.state == h.initial
    assert h.writes[0]["num_divisions"] == 6 and h.writes[0]["cell_function"] == "native-orth-quality"
    assert result["report_settings_restored"] and not result["physical_or_mesh_state_changed"]


@pytest.mark.parametrize("payload", [list(range(7)), {"counts": list(range(100))}, {"credential": "SECRET"}, "SECRET", {"x": [float("nan")]}, [[[[[1]]]]]])
def test_no_unbounded_arrays_unknown_schema_or_credentials(payload):
    result = e.query_histogram(histogram_solver(Histogram(payload)), lambda: 100)
    assert result["status"] == "unavailable" and "SECRET" not in json.dumps(result)


def test_array_or_generator_is_not_converted_or_consumed():
    class Array:
        def tolist(self): pytest.fail("Never convert volume arrays")
        def __iter__(self): pytest.fail("Never stream bulk cell arrays")
    with pytest.raises(ValueError): e.bounded_histogram_payload(Array())
    with pytest.raises(ValueError): e.bounded_histogram_payload((x for x in range(100)))


@pytest.mark.parametrize("fields", [{}, {"pressure": {"display_name": "Pressure"}}, {"unsafe;cmd": {"display_name": "Orthogonal Quality"}}])
def test_missing_quality_field_never_guesses_token_or_queries(fields):
    h = Histogram(); result = e.query_histogram(histogram_solver(h, fields), lambda: 100)
    assert result["status"] == "unavailable" and h.queries == 0 and h.writes == []


def test_uncertain_histogram_is_not_retried_or_overlapped_with_restore(monkeypatch):
    h = Histogram()
    def failed(): h.queries += 1; raise w.UncertainCall("SECRET")
    h.get_values = failed
    result = e.query_histogram(histogram_solver(h), lambda: 100)
    assert h.queries == 1 and len(h.writes) == 1 and result["rpc_uncertain"]
    assert "SECRET" not in json.dumps(result)


@pytest.mark.parametrize("domain", ["background", "arm"])
def test_new_native_path_single_launch_connect_PIPE_exit_and_bounded_query(local, monkeypatch, domain):
    root, _, _ = local
    slot = l.LocalQualitySlot(domain, root); path, _ = reserve(root, slot)
    parent = os.getppid()
    dump(path / "launcher.json", {"job_id": slot.JOB_ID, "pid": parent, "creation_time": 123.5})
    dump(root / ".ansys-task.lock", {"pid": parent, "attempt": str(path)})
    monkeypatch.setenv("REMOTING_SERVER_ADDRESS", "127.0.0.1")
    exe = root / "OFFLINE_NEVER_EXECUTE.exe"; exe.write_bytes(b"mock only")
    monkeypatch.setattr(w, "validate_frozen", lambda *a: {"contract": {"software": {"fluent": str(exe), "ansys_root": str(root)}}})
    monkeypatch.setattr(w.psutil, "Process", lambda pid=None: types.SimpleNamespace(pid=os.getpid() if pid is None else pid,
        create_time=lambda: 123.5, ppid=lambda: os.getpid(), exe=lambda: str(exe)))
    events = []; pipe = FakePipe(events)
    process = types.SimpleNamespace(pid=501, stdin=pipe, poll=lambda: None, wait=lambda **kwargs: 0)
    launches = []
    def popen(*args, **kwargs):
        launches.append(kwargs); assert kwargs["stdin"] is w.subprocess.PIPE and kwargs["shell"] is False; return process
    monkeypatch.setattr(w.subprocess, "Popen", popen)
    h = Histogram(); solver = histogram_solver(h)
    solver.file = types.SimpleNamespace(read_mesh=lambda **kwargs: events.append("read_mesh"))
    solver.tui = types.SimpleNamespace(file=types.SimpleNamespace(
        start_transcript=lambda value: Path(value).write_text(transcript(domain), encoding="utf-8"), stop_transcript=lambda: None),
        mesh=types.SimpleNamespace(quality=lambda: None, check=lambda: None, modify_zones=types.SimpleNamespace(list_zones=lambda: None)))
    class Walls:
        def get_object_names(self): return l.MESHES[domain]["walls"]
        def __getitem__(self, name): return types.SimpleNamespace(adjacent_cell_zone=lambda: l.MESHES[domain]["fluid"])
    solver.setup = types.SimpleNamespace(boundary_conditions=types.SimpleNamespace(wall=Walls()),
        cell_zone_conditions=types.SimpleNamespace(fluid=types.SimpleNamespace(get_object_names=lambda: [l.MESHES[domain]["fluid"]])))
    solver.exit = lambda: events.append("exit_while_open") if not pipe.closed else pytest.fail("PIPE closed before exit")
    connects = []
    def connect(*args, **kwargs):
        assert kwargs == {"normal_exit": True}; connects.append(True); args[3].update(ready(), cleanup_on_exit=True, constructor_watchdog_process_started=False); return solver
    monkeypatch.setattr(w, "connect_ready", connect)
    result = w.native_collect(path, stage_root=root, local_quality=True)
    assert len(launches) == len(connects) == 1 and h.queries == 1 and pipe.closed
    assert result["batch_id"] == l.BATCH_ID and result["provenance"] == l.PROVENANCE and result["errors"] == {}
    assert result["normal_exit"]["native_returncode"] == 0 and result["stdin_close_phase"] == "after_observed_normal_exit_and_wait"
    assert result["local_diagnostic"]["status"] == "fail_quality" and result["local_diagnostic"]["worst_cells"]["returned_count"] == 1
    assert not result["local_diagnostic"]["complete_topology_verified"]


def test_fixed_MCP_surface_no_free_commands_or_bulk_fields():
    source = (STAGE / "stage15_mcp.py").read_text(encoding="utf-8")
    tools = {n.name: n for n in ast.parse(source).body if isinstance(n, ast.AsyncFunctionDef)}
    assert len(tools) == 26
    assert [x.arg for x in tools["stage15_start_local_quality_diagnostic"].args.args] == ["domain", "confirmation"]
    evidence = (STAGE / "local_quality_evidence.py").read_text(encoding="utf-8")
    calls = [ast.unparse(n.func) for n in ast.walk(ast.parse(evidence)) if isinstance(n, ast.Call)]
    assert not any(name.endswith(("get_scalar_field_data", "get_fields", "get_surface_data", "new_transaction", "tolist", "repair", "iterate", "initialize")) for name in calls)
    protocol = (STAGE / "check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_local_quality_diagnostic", {"domain": "background", "confirmation": "NO"})' in protocol
    assert l.START not in protocol and l.POLICY["maximum_histogram_bins"] == 6 and l.POLICY["maximum_worst_records"] == 20

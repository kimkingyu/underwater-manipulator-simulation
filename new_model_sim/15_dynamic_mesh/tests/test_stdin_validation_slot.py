"""One new explicit stdin-validation slot. All execution/pipe objects are fakes."""
import ast
import concurrent.futures
import hashlib
from pathlib import Path
import sys
import time

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import inspection_batch002_bridge as b
import mcp_bridge as legacy
from test_definition_probe import sandbox
from test_inspection_batch002 import batch, completed_connection, dump
from test_supplement_connection import old_failure


def test_native_PIPE_worker_and_connection_adapter_match_original_1036_bytes():
    source = (STAGE / "inspection_batch002.py").read_text(encoding="utf-8")
    expected = {"NativeInputLease": "e082fd88fa2177cc9e4ba6e1f16d60c31cdc03c651c46e6efd1f6e08c979ac21",
                "collect_mesh": "d8e107eee19a8380dc8c7d4735101a270edf3d105f43b30a6d4cb4cdddb9fa95"}
    actual = {n.name: hashlib.sha256(ast.get_source_segment(source,n).encode("utf-8")).hexdigest()
              for n in ast.parse(source).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in expected}
    assert actual == expected
    connection = (STAGE / "inspection_batch002_connection.py").read_text(encoding="utf-8")
    assert "cleanup_on_exit=normal_exit" in connection  # new explicit full-record normal-exit contract
    native = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == "native_collect")
    popens = [n for n in ast.walk(native) if isinstance(n, ast.Call) and ast.unparse(n.func) == "subprocess.Popen"]
    assert len(popens) == 1
    assert ast.unparse(popens[0]) == "subprocess.Popen(command, cwd=str(attempt), stdin=subprocess.PIPE, shell=False)"
    assert any(isinstance(n, ast.Call) and ast.unparse(n) == "connect_ready(attempt, process, end, report['readiness'], normal_exit=True)" for n in ast.walk(native))


@pytest.mark.parametrize("token", [b.START, b.SUPPLEMENT_START, b.SUPPLEMENT_CANCEL, "NO", "RUN_STAGE15_READ_ONLY_MESH_INSPECTION_ONCE"])
def test_only_new_validation_confirmation_can_spend_new_slot(batch, token):
    root, _, popen = batch
    slot = b.StdinValidationSlot(root)
    with pytest.raises(legacy.BridgeError): slot.start(token)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_new_validation_once_leaves_original_QA_paths_and_old_failures_unchanged(batch, monkeypatch):
    root, old_supp, popen = batch
    monkeypatch.setitem(b.POLICY, "supplement_connection_retired", True)
    grant = legacy.read_json(root / b.GRANT)
    grant["supplement_connection_retired"] = True
    dump(root / b.GRANT, grant)
    old, path = old_failure(root)
    old_supp_path = old_supp.safe(old_supp.ATTEMPT); old_supp_path.mkdir(parents=True)
    dump(old_supp_path / "request.json", {"authorization": {"path": "old-binding"}, "sources": []})
    dump(old_supp_path / "probe_result.json", {"job_id": old_supp.JOB_ID, "batch_id": b.BATCH, "slot": old_supp.slot,
        "status": "cleanup_unverified", "owned_cleanup_verified": False, "mesh_generated": False,
        "mesh_motion_verified": False, "flow_iterations": 0})
    originals = {p: p.read_bytes() for p in (path / "probe_result.json", old_supp_path / "probe_result.json")}
    slot = b.StdinValidationSlot(root)
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    first = slot.start(b.STDIN_VALIDATION_START)
    assert first["job_id"] == b.STDIN_VALIDATION_JOB
    assert slot.start(b.STDIN_VALIDATION_START)["status"] == "already_reserved" and popen.call_count == 1
    for retired, token in ((old, b.START), (old_supp, b.SUPPLEMENT_START)):
        answer = retired.start(token)
        assert answer["job"]["job_id"] == retired.JOB_ID and answer["job"]["status"] == "cleanup_unverified"
        assert answer["job"]["connection_gate"]["status"] == "unverified"
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        assert qa.ATTEMPT == "runs/mcp_mesh_inspection/batch_002/" + name
        assert qa.status()["remaining_attempts"] == 1
        assert not qa.safe(qa.ATTEMPT).exists()
    assert all(p.read_bytes() == data for p,data in originals.items())
    assert b.POLICY["additional_attempts_authorized"] == b.POLICY["additional_connection_attempts"] == 0
    assert b.POLICY["original_stdin_validation_attempts"] == 1
    assert b.POLICY["additional_QA_attempts"] == 0


def test_retired_old_slots_cannot_be_recreated_but_new_slot_is_separate(batch, monkeypatch):
    root, _, popen = batch
    monkeypatch.setitem(b.POLICY, "supplement_connection_retired", True)
    for slot in (b.Batch002Slot("connection",root), b.SupplementalConnectionSlot(root)):
        with pytest.raises(legacy.BridgeError, match="retired"): slot.start(slot.START_CONFIRMATION)
        assert not slot.safe(slot.ATTEMPT).exists() and slot.status()["remaining_attempts"] == 0
    new = b.StdinValidationSlot(root)
    assert new.status()["remaining_attempts"] == 1 and new.is_connection
    popen.assert_not_called()


def test_old_supplemental_success_cannot_open_new_validation_gate(batch):
    root, old, popen = batch
    completed_connection(root, old)
    qa = b.Batch002Slot("background", root)
    with pytest.raises(legacy.BridgeError): qa.start(b.START)
    assert not qa.safe(qa.ATTEMPT).exists()
    assert not b.StdinValidationSlot(root).safe(b.STDIN_VALIDATION_ATTEMPT).exists()
    popen.assert_not_called()


def test_only_new_job_native_evidence_opens_original_QA_gate(batch):
    root, _, popen = batch
    completed_connection(root)
    result = b.Batch002Slot("arm",root).connection_gate()
    assert result["job_id"] == b.FULL_RECORD_JOB and result["job_id"] not in (b.SUPPLEMENT_JOB, b.STDIN_VALIDATION_JOB)
    popen.assert_not_called()


@pytest.mark.parametrize("field,value", [("mode", "DEVNULL"), ("bytes_written", 1), ("bytes_written", False),
    ("close_requested_in_cleanup", False), ("pipe_returned", False), ("writer_closed", False),
    ("late_return_after_close_request", True), ("stdin_close_phase", "before_connection")])
def test_validation_gate_rejects_incomplete_or_early_closed_PIPE_proof(batch, field, value):
    root, _, popen = batch
    path = completed_connection(root)
    native = legacy.read_json(path / "native_result.json")
    if field == "stdin_close_phase": native[field] = value
    else: native["stdin_lifecycle"][field] = value
    dump(path / "native_result.json", native)
    final = legacy.read_json(path / "probe_result.json"); final["native"] = native; dump(path / "probe_result.json", final)
    qa = b.Batch002Slot("background",root)
    with pytest.raises(legacy.BridgeError): qa.start(b.START)
    assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("target", ["old_job", "old_slot", "old_grant", "wrong_batch", "future_time", "cleanup"])
def test_validation_gate_mixed_binding_or_future_cleanup_never_consumes_QA(batch,target):
    root, _, popen = batch
    path = completed_connection(root)
    req = legacy.read_json(path / "request.json")
    final = legacy.read_json(path / "probe_result.json")
    if target == "old_job": req["job_id"] = b.SUPPLEMENT_JOB
    if target == "old_slot": req["slot"] = b.SUPPLEMENT
    if target == "old_grant": req["authorization"]["path"] = str(root / b.POLICY["baseline_batch_authorization_relative"])
    if target == "wrong_batch": req["batch_id"] = "different-batch"
    if target == "future_time":
        future = time.time()+100
        req.update(created_unix=future,deadline_unix=future+300,work_deadline_unix=future+270)
        for key in ("created_unix","deadline_unix","work_deadline_unix"): final[key] = req[key]
    if target == "cleanup": final["owned_cleanup_verified"] = False
    dump(path / "request.json",req); dump(path / "probe_result.json",final)
    qa=b.Batch002Slot("arm",root)
    with pytest.raises(legacy.BridgeError): qa.start(b.START)
    assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("lock", [".ansys-task.lock", ".mesh-inspection-dispatch.lock", ".sizing-dispatch.lock"])
def test_new_validation_serial_locks_do_not_consume_slot(batch,lock):
    root,_,popen=batch
    dump(root/lock,{"token":"someone-else"})
    slot=b.StdinValidationSlot(root)
    with pytest.raises(legacy.BridgeError): slot.start(b.STDIN_VALIDATION_START)
    assert not slot.safe(slot.ATTEMPT).exists() and legacy.read_json(root/lock)["token"]=="someone-else"
    popen.assert_not_called()


def test_new_validation_uncertain_worker_launch_is_once_and_preserves_dispatch(batch,monkeypatch):
    root,_,popen=batch
    slot=b.StdinValidationSlot(root)
    monkeypatch.setattr(slot,"launch_worker",lambda *a,**k: (_ for _ in ()).throw(legacy.WorkerLaunchUncertain("offline timeout")))
    result=slot.start(b.STDIN_VALIDATION_START)
    assert result["status"]=="cleanup_unverified" and (root/".mesh-inspection-dispatch.lock").exists()
    assert slot.start(b.STDIN_VALIDATION_START)["new_worker_started"] is False
    popen.assert_not_called()


def test_new_validation_concurrent_reservation_is_once(batch,monkeypatch):
    root,_,popen=batch
    slot=b.StdinValidationSlot(root)
    monkeypatch.setattr(slot,"_inspect_pid",lambda pid:{"pid":pid,"exists":True,"creation_time":123.5})
    def start():
        try:return slot.start(b.STDIN_VALIDATION_START)
        except legacy.BridgeError:return {"new_worker_started":False}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results=list(pool.map(lambda _:start(),range(4)))
    assert sum(row.get("new_worker_started") is True for row in results)==popen.call_count==1


def test_new_cancel_has_own_fixed_job_operation_and_confirmation(batch,monkeypatch):
    root,_,_=batch
    slot=b.StdinValidationSlot(root)
    monkeypatch.setattr(slot,"_inspect_pid",lambda pid:{"pid":pid,"exists":True,"creation_time":123.5})
    slot.start(b.STDIN_VALIDATION_START)
    with pytest.raises(legacy.BridgeError):slot.cancel(slot.JOB_ID,b.SUPPLEMENT_CANCEL)
    assert slot.cancel(slot.JOB_ID,b.STDIN_VALIDATION_CANCEL)["status"]=="cancellation_requested"
    row=legacy.read_json(slot.safe(slot.ATTEMPT+"/cancel_request.json"))
    assert row["job_id"]==b.STDIN_VALIDATION_JOB and row["operation"]=="cancel_stdin_connection_validation"


def test_eighteen_tools_fixed_schema_and_old_routes_preserved():
    source=(STAGE/"stage15_mcp.py").read_text(encoding="utf-8")
    tools={n.name:n for n in ast.parse(source).body if isinstance(n,ast.AsyncFunctionDef)}
    assert len(tools)==23
    assert tools["stage15_stdin_validation_preflight"].args.args==[]
    assert [a.arg for a in tools["stage15_start_stdin_validation"].args.args]==["confirmation"]
    assert b.batch_job(b.STDIN_VALIDATION_JOB).ATTEMPT==b.STDIN_VALIDATION_ATTEMPT
    assert b.batch_job(b.SUPPLEMENT_JOB).ATTEMPT==b.SUPPLEMENT_ATTEMPT
    assert b.batch_job("inspection-b002-connection").slot=="connection"
    for unknown in ("stdin_validation_002","connection_stdin_validation_002",b.STDIN_VALIDATION):
        with pytest.raises(legacy.BridgeError):b.Batch002Slot(unknown)
    protocol=(STAGE/"check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_stdin_validation", {"confirmation": "NO"})' in protocol
    assert b.STDIN_VALIDATION_START not in protocol

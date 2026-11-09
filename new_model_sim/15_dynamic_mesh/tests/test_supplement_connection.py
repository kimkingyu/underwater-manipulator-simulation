"""One explicit supplemental connection, original QA slots only; offline fakes."""
import ast
import concurrent.futures
import copy
from pathlib import Path
import sys
import time
from unittest import mock

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import inspection_batch002_bridge as b
import inspection_batch002 as w
import mcp_bridge as legacy
from test_inspection_batch002 import batch, completed_connection, reserve, dump, ready
from test_definition_probe import sandbox


def old_failure(root):
    old = b.Batch002Slot("connection", root)
    path = old.safe(old.ATTEMPT); path.mkdir(parents=True)
    req = old.make_request(time.time() - 30, old._sources())
    req["authorization"] = {"path": str(root / b.POLICY["baseline_batch_authorization_relative"]),
                            "sha256": b.POLICY["baseline_batch_authorization_sha256"]}
    dump(path / "request.json", req)
    dump(path / "probe_result.json", {"job_id": old.JOB_ID, "batch_id": b.BATCH, "slot": "connection",
        "status": "cleanup_unverified", "owned_cleanup_verified": False, "mesh_generated": False,
        "flow_iterations": 0, "mesh_motion_verified": False, "native": {"readiness": {"single_channel_count": 0}}})
    return old, path


def test_original_start_always_returns_original_failure_never_supplement(batch):
    root, new, popen = batch
    old, path = old_failure(root)
    before = {name: (path / name).read_bytes() for name in ("request.json", "probe_result.json")}
    answer = old.start(b.START)
    assert answer["status"] == "already_reserved" and answer["new_worker_started"] is False
    assert answer["job"]["job_id"] == old.JOB_ID and answer["job"]["status"] == "cleanup_unverified"
    assert answer["job"]["connection_gate"]["status"] == "unverified"
    assert not new.safe(new.ATTEMPT).exists()
    assert all((path / n).read_bytes() == data for n, data in before.items())
    assert all(b.Batch002Slot(n, root).status()["remaining_attempts"] == 1 for n in ("background", "arm"))
    popen.assert_not_called()


def test_original_connection_cannot_be_recreated_even_if_directory_absent(batch):
    root, new, popen = batch
    old = b.Batch002Slot("connection", root)
    with pytest.raises(legacy.BridgeError, match="permanently retired"): old.start(b.START)
    assert not old.safe(old.ATTEMPT).exists() and not new.safe(new.ATTEMPT).exists()
    assert old.status()["remaining_attempts"] == 0
    popen.assert_not_called()


@pytest.mark.parametrize("confirmation", [b.START, "RUN_STAGE15_READ_ONLY_MESH_INSPECTION_ONCE", b.CANCEL, "NO"])
def test_only_new_confirmation_may_reserve_supplement(batch, confirmation):
    _, slot, popen = batch
    with pytest.raises(legacy.BridgeError): slot.start(confirmation)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_exactly_one_new_connection_no_new_QA_paths(batch, monkeypatch):
    root, slot, popen = batch
    old, old_path = old_failure(root)
    old_bytes = (old_path / "probe_result.json").read_bytes()
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    result = slot.start(b.SUPPLEMENT_START)
    assert result["job_id"] == b.SUPPLEMENT_JOB
    assert slot.safe(slot.ATTEMPT).as_posix().endswith("batch_002/connection_supplement_001")
    assert slot.start(b.SUPPLEMENT_START)["new_worker_started"] is False and popen.call_count == 1
    assert slot.status()["remaining_attempts"] == 0
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        assert qa.ATTEMPT == "runs/mcp_mesh_inspection/batch_002/" + name
        assert qa.JOB_ID == "inspection-b002-" + name and qa.status()["remaining_attempts"] == 1
    assert (old_path / "probe_result.json").read_bytes() == old_bytes
    assert b.POLICY["additional_attempts_authorized"] == b.POLICY["additional_connection_attempts"] == 0
    assert b.POLICY["stdin_validation_job_id"] != slot.JOB_ID
    assert b.POLICY["original_additional_connection_attempts"] == 1
    assert b.POLICY["additional_QA_attempts"] == 0
    for wrong in ("connection_supplement_002", "background2", "arm_retry", b.SUPPLEMENT):
        with pytest.raises(legacy.BridgeError): b.Batch002Slot(wrong, root)


def test_concurrent_supplement_calls_never_start_twice(batch, monkeypatch):
    _, slot, popen = batch
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    def start():
        try: return slot.start(b.SUPPLEMENT_START)
        except legacy.BridgeError: return {"status": "blocked", "new_worker_started": False}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _: start(), range(4)))
    assert sum(r.get("new_worker_started") is True for r in results) == popen.call_count == 1


def test_only_supplemental_success_opens_original_QA_gate(batch):
    root, slot, popen = batch
    old_failure(root)
    with pytest.raises(legacy.BridgeError): b.Batch002Slot("background", root).connection_gate()
    completed_connection(root)
    gate = b.Batch002Slot("background", root).connection_gate()
    assert gate["job_id"] == b.FULL_RECORD_JOB
    assert gate["job_id"] != b.SUPPLEMENT_JOB
    assert gate["job_id"] != b.Batch002Slot("connection", root).JOB_ID
    popen.assert_not_called()


@pytest.mark.parametrize("field,bad", [("job_id", "inspection-b002-connection"), ("slot", "connection"),
    ("batch_id", "inspection-batch001"), ("job_id", "inspection-b002-connection-supplement-002")])
def test_cross_job_batch_or_original_receipt_rejected(batch, field, bad):
    root, _, popen = batch
    path = completed_connection(root)
    for name in ("request.json", "native_result.json", "probe_result.json", "command_audit.json"):
        row = legacy.read_json(path / name); row[field] = bad; dump(path / name, row)
    qa = b.Batch002Slot("arm", root)
    with pytest.raises(legacy.BridgeError): qa.start(b.START)
    assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("which", ["request_grant", "current_grant", "grant_baseline", "mock", "cleanup", "clock", "future_completion", "source_snapshot", "inputs"])
def test_supplement_gate_authorization_hash_cleanup_clock_and_inputs_fail_closed(batch, which):
    root, _, popen = batch
    path = completed_connection(root)
    if which == "request_grant":
        row = legacy.read_json(path / "request.json"); row["authorization"]["path"] = str(root / b.POLICY["baseline_batch_authorization_relative"]); dump(path / "request.json", row)
    if which == "current_grant":
        row = legacy.read_json(root / b.GRANT); row["source_manifest_sha256"] = "0" * 64; dump(root / b.GRANT, row)
    if which == "grant_baseline": (root / b.POLICY["baseline_batch_authorization_relative"]).write_bytes(b"changed")
    if which == "mock":
        row = legacy.read_json(path / "native_result.json"); row["provenance"] = "mock"; dump(path / "native_result.json", row)
    if which == "cleanup":
        row = legacy.read_json(path / "probe_result.json"); row["owned_cleanup_verified"] = False; dump(path / "probe_result.json", row)
    if which == "clock":
        req = legacy.read_json(path / "request.json"); final = legacy.read_json(path / "probe_result.json")
        req.update(created_unix=time.time() + 100, deadline_unix=time.time() + 400, work_deadline_unix=time.time() + 370)
        for key in ("created_unix", "deadline_unix", "work_deadline_unix"): final[key] = req[key]
        dump(path / "request.json", req); dump(path / "probe_result.json", final)
    if which == "future_completion":
        req = legacy.read_json(path / "request.json"); final = legacy.read_json(path / "probe_result.json")
        created = time.time(); req.update(created_unix=created, deadline_unix=created+300, work_deadline_unix=created+270)
        for key in ("created_unix", "deadline_unix", "work_deadline_unix"): final[key] = req[key]
        final["elapsed_seconds"] = 200; dump(path / "request.json", req); dump(path / "probe_result.json", final)
    if which == "source_snapshot": (path / "source_snapshot/inspection_batch002.py").write_bytes(b"changed")
    if which == "inputs":
        row = legacy.read_json(path / "probe_result.json"); row["protected_inputs_after"]["files"][0]["passed"] = False; dump(path / "probe_result.json", row)
    qa = b.Batch002Slot("background", root)
    with pytest.raises(legacy.BridgeError): qa.start(b.START)
    assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


def test_successful_supplement_cannot_add_QA_retries(batch, monkeypatch):
    root, _, popen = batch
    completed_connection(root)
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        monkeypatch.setattr(qa, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
        qa.start(b.START)
        assert qa.start(b.START)["status"] == "already_reserved"
        assert qa.status()["remaining_attempts"] == 0
        # Offline fixture models the fake worker's cleanup, never a real lock.
        (root / ".mesh-inspection-dispatch.lock").unlink()
    assert popen.call_count == 2


def test_worker_original_retired_even_without_receipt_and_supplement_never_reads_mesh(batch):
    root, slot, popen = batch
    old = b.Batch002Slot("connection", root)
    path = old.safe(old.ATTEMPT); path.mkdir(parents=True)
    with pytest.raises(w.RuntimeGateError): w.run(path, stage_root=root)
    with pytest.raises(w.RuntimeGateError): w.native_collect(path, stage_root=root)
    with pytest.raises(w.RuntimeGateError): w.collect_mesh(None, slot, root, {})
    assert not (path / "worker_claim.json").exists()
    popen.assert_not_called()


def test_supplement_cancel_uses_only_its_own_token_and_sentinel(batch, monkeypatch):
    _, slot, _ = batch
    monkeypatch.setattr(slot, "_inspect_pid", lambda pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    slot.start(b.SUPPLEMENT_START)
    with pytest.raises(legacy.BridgeError): slot.cancel(slot.JOB_ID, b.CANCEL)
    answer = slot.cancel(slot.JOB_ID, b.SUPPLEMENT_CANCEL)
    assert answer["status"] == "cancellation_requested"
    sentinel = legacy.read_json(slot.safe(slot.ATTEMPT + "/cancel_request.json"))
    assert sentinel["job_id"] == b.SUPPLEMENT_JOB and sentinel["operation"] == slot.CANCEL_OPERATION


def test_sixteen_schema_routes_only_one_new_native_start_surface():
    source = (STAGE / "stage15_mcp.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    tools = {n.name:n for n in tree.body if isinstance(n, ast.AsyncFunctionDef)}
    assert len(tools) == 21
    start = tools["stage15_start_supplement_connection"]
    assert [a.arg for a in start.args.args] == ["confirmation"]
    assert tools["stage15_supplement_connection_preflight"].args.args == []
    assert "SupplementalConnectionSlot().start" in ast.get_source_segment(source, start)
    assert "SUPPLEMENT_JOB" in source
    assert b.batch_job(b.SUPPLEMENT_JOB).ATTEMPT == b.SUPPLEMENT_ATTEMPT
    assert b.batch_job("inspection-b002-connection").ATTEMPT.endswith("/connection")
    protocol = (STAGE / "check_mcp_connection.py").read_text(encoding="utf-8")
    assert '("stage15_start_supplement_connection", {"confirmation": "NO"})' in protocol
    assert b.SUPPLEMENT_START not in protocol

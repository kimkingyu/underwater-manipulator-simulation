"""No native/socket/real review: exact helper classification and dual-source gates."""
import copy
import json
import os
from pathlib import Path
import sys
import types
import time

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import inspection_license_review as r
import inspection_batch002_bridge as b
import inspection_batch002 as w
from mcp_bridge import BridgeError, read_json, sha256
from test_definition_probe import sandbox, dump
from test_inspection_batch002 import batch, completed_connection, PRODUCTION_GATE


def refresh_new_binding(root):
    slot = b.Batch002Slot("background", root)
    dump(root / b.MANIFEST, {"schema_version": 1, "files": slot._sources()})
    grant = copy.deepcopy(b.POLICY)
    grant.update(source_manifest_sha256=sha256(root / b.MANIFEST), authorization_source="OFFLINE fake only",
        recorded_local="offline", execution_policy="Never launch a native binary")
    dump(root / b.GRANT, grant)


def bind_runtime(root, monkeypatch):
    old = r.HistoricalConnection(root)
    rows = [{"path": p, "size": old.safe(p).stat().st_size, "sha256": sha256(old.safe(p))} for p in r.runtime_paths(old)]
    dump(root / r.RUNTIME_MANIFEST, {"files": rows})
    approval = {"additional_connection_attempts": 0, "additional_QA_attempts": 0, "original_result_must_remain_failed": True,
        "user_choice": "按此分类继续（推荐）", "runtime_manifest_relative": r.RUNTIME_MANIFEST,
        "runtime_manifest_sha256": sha256(root / r.RUNTIME_MANIFEST), "old_authorization_sha256": sha256(root / r.OLD_GRANT),
        "old_source_manifest_sha256": sha256(root / r.OLD_SOURCES),
        "expected_historical_helper": read_json(old.safe(old.ATTEMPT + "/probe_result.json"))["owned_live_before_cleanup"][0]}
    dump(root / r.APPROVAL, approval)
    monkeypatch.setattr(r, "APPROVAL_SHA256", sha256(root / r.APPROVAL))


@pytest.fixture
def license_batch(batch, monkeypatch):
    root, _, popen = batch
    (root / "reports").mkdir(exist_ok=True)
    path = completed_connection(root)
    slot = b.FullRecordValidationSlot(root)
    rows = [x for x in slot._sources() if Path(x["path"]).name != "inspection_license_review.py"]
    assert len(rows) == 20
    dump(root / r.OLD_SOURCES, {"schema_version": 1, "files": rows})
    old_grant = {"full_record_job_id": slot.JOB_ID, "full_record_slot": slot.slot, "full_record_attempt_relative": slot.ATTEMPT,
        "additional_connection_attempts": 1, "additional_QA_attempts": 0, "source_manifest_relative": r.OLD_SOURCES,
        "source_manifest_sha256": sha256(root / r.OLD_SOURCES)}
    dump(root / r.OLD_GRANT, old_grant)
    monkeypatch.setitem(b.POLICY, "baseline_batch_authorization_sha256", sha256(root / r.OLD_GRANT))
    refresh_new_binding(root)
    request = read_json(path / "request.json")
    request.update(sources=rows, authorization={"path": str(root / r.OLD_GRANT), "sha256": sha256(root / r.OLD_GRANT)})
    dump(path / "request.json", request)
    manifest = read_json(path / "source_snapshot/archive_manifest.json")
    manifest["files"] = [x for x in manifest["files"] if Path(x["original_path"]).name != "inspection_license_review.py"]
    dump(path / "source_snapshot/archive_manifest.json", manifest)
    bindings = []
    for relative in (r.OLD_GRANT, r.OLD_SOURCES):
        target = "execution_binding/" + Path(relative).name
        dump(path / "source_snapshot" / target, read_json(root / relative))
        bindings.append({"original": relative, "archive_relative": target, "sha256": sha256(root / relative)})
    dump(path / "source_snapshot/execution_binding.json", {"job_id": slot.JOB_ID, "files": bindings})
    final = read_json(path / "probe_result.json")
    native = final["native"]
    created = request["created_unix"]
    final["owned_processes"] = [{"pid": 222, "creation_time": created + 1}, {"pid": 333, "creation_time": created + 2}]
    final["owned_process_identity"][0]["creation_time"] = created + 2
    for name, field, stamp in (("native_claim.json", "creation_time", created + 1), ("fluent_process.json", "creation_time", created + 2)):
        value = read_json(path / name); value[field] = stamp; dump(path / name, value)
    native["normal_exit"]["process_identity"] = read_json(path / "fluent_process.json")
    dump(path / "normal_exit_completed.json", native["normal_exit"])
    helper = root / "official/licensingclient/winx64/ansyscl.exe"
    helper.parent.mkdir(parents=True); helper.write_bytes(b"OFFLINE helper, never execute")
    helper_id = {"pid": 444, "creation_time": created + 3}
    final["owned_processes"].append(helper_id)
    final["owned_process_identity"].append(dict(helper_id, name="ansyscl.exe", executable=str(helper)))
    final.update(status="failed", owned_live_before_cleanup=[helper_id])
    dump(path / "native_result.json", native); dump(path / "probe_result.json", final)
    bind_runtime(root, monkeypatch)
    monkeypatch.setattr(r, "helper_installation", lambda slot: helper)
    monkeypatch.setattr(r.psutil, "Process", lambda pid: (_ for _ in ()).throw(r.psutil.NoSuchProcess(pid)))
    monkeypatch.setattr(r.psutil, "process_iter", lambda *a: [])
    monkeypatch.setattr(b.Batch002Slot, "connection_gate", PRODUCTION_GATE)
    monkeypatch.setitem(b.POLICY, "full_record_retired", True)
    refresh_new_binding(root)
    return root, path, helper, popen


def test_review_is_independent_and_old_failed_never_rewritten(license_batch):
    root, path, helper, popen = license_batch
    before = (path / "probe_result.json").read_bytes()
    candidate = r.evaluate_review(root)
    assert candidate["original_status"] == "failed" and candidate["conclusion"] == r.CONCLUSION
    assert not (root / r.REVIEW).exists()
    with pytest.raises(BridgeError): b.Batch002Slot("arm", root).connection_gate()
    assert r.write_review(r.REVIEW_CONFIRMATION, root)["status"] == "reviewed"
    assert r.write_review(r.REVIEW_CONFIRMATION, root)["status"] == "already_reviewed"
    assert b.Batch002Slot("arm", root).connection_gate()["status"] == "passed"
    assert (path / "probe_result.json").read_bytes() == before and read_json(path / "probe_result.json")["status"] == "failed"
    assert r.HistoricalConnection(root)._sources() != b.Batch002Slot("arm", root)._sources()
    assert len(r.HistoricalConnection(root)._sources()) == 20 and len(b.Batch002Slot("arm", root)._sources()) == 21
    popen.assert_not_called()


@pytest.mark.parametrize("token", ["NO", b.START, b.FULL_RECORD_START, b.FULL_RECORD_CANCEL])
def test_invalid_review_confirmation_does_not_write_or_launch(license_batch, token):
    root, path, helper, popen = license_batch
    with pytest.raises(BridgeError): r.write_review(token, root)
    assert not (root / r.REVIEW).exists() and not (root / r.REVIEW_SEAL).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("fault", ["basename", "similar_path", "unknown", "solver", "cortex", "python", "mixed", "missing_identity", "wrong_stamp", "duplicate", "not_owned", "bool_pid"])
def test_only_exact_official_helper_owned_identity_is_classified(license_batch, fault):
    root, path, helper, popen = license_batch
    slot = b.Batch002Slot("arm", root)
    final = read_json(path / "probe_result.json")
    identity = final["owned_process_identity"][-1]
    if fault == "basename": identity["executable"] = "ansyscl.exe"
    if fault == "similar_path":
        fake = root / "external/licensingclient/winx64/ansyscl.exe"; fake.parent.mkdir(parents=True); fake.write_bytes(helper.read_bytes()); identity["executable"] = str(fake)
    if fault in ("unknown", "solver", "cortex", "python"): identity["name"] = {"unknown": "helper.exe", "solver": "fluent.exe", "cortex": "cx2420.exe", "python": "python.exe"}[fault]
    if fault == "mixed": final["owned_live_before_cleanup"].append(final["owned_processes"][1])
    if fault == "missing_identity": final["owned_process_identity"].pop()
    if fault == "wrong_stamp": identity["creation_time"] += 1
    if fault == "duplicate": final["owned_live_before_cleanup"].append(final["owned_live_before_cleanup"][0])
    if fault == "not_owned": final["owned_processes"].pop()
    if fault == "bool_pid": final["owned_live_before_cleanup"][0]["pid"] = True
    with pytest.raises((BridgeError, OSError)): r.classify_live_helpers(slot, final)
    popen.assert_not_called()


@pytest.mark.parametrize("target", ["old_source", "old_grant", "snapshot", "runtime", "approval", "new_source", "new_manifest", "new_grant", "review", "seal", "normal_exit", "stage", "final_cleanup", "input", "mesh", "log"])
def test_each_binding_or_artifact_change_blocks_without_QA_consumption(license_batch, target):
    root, path, helper, popen = license_batch
    r.write_review(r.REVIEW_CONFIRMATION, root)
    destination = {"old_source": root / r.OLD_SOURCES, "old_grant": root / r.OLD_GRANT,
        "snapshot": path / "source_snapshot/stage15_mcp.py", "runtime": root / r.RUNTIME_MANIFEST,
        "approval": root / r.APPROVAL, "new_source": root / "inspection_license_review.py",
        "new_manifest": root / b.MANIFEST, "new_grant": root / b.GRANT, "review": root / r.REVIEW,
        "seal": root / r.REVIEW_SEAL, "normal_exit": path / "normal_exit_completed.json", "stage": path / "transcript_stop_completed.json",
        "final_cleanup": path / "probe_result.json", "input": root / "contract.json",
        "mesh": root / b.MESHES["arm"]["path"], "log": path / "native.stderr.log"}[target]
    if target == "review":
        value = read_json(destination); value["owned_process_exit_states"] = []; dump(destination, value)
    elif target == "log": destination.write_text("Abnormal Exit!", encoding="utf-8")
    else: destination.write_bytes(destination.read_bytes() + b"changed")
    for name in ("background", "arm"):
        qa = b.Batch002Slot(name, root)
        with pytest.raises((BridgeError, ValueError, OSError)): qa.start(b.START)
        assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize("fault", ["cleanup", "exit_missing", "normal_nonzero", "adapter_nonzero", "log_error", "early_pipe", "future", "deadline", "elapsed", "status_completed", "error", "identity_time"])
def test_rebinding_tampered_runtime_does_not_bypass_original_strong_gate(license_batch, monkeypatch, fault):
    root, path, helper, popen = license_batch
    final = read_json(path / "probe_result.json")
    native = final["native"]
    if fault == "cleanup": final["owned_cleanup_verified"] = False
    if fault == "exit_missing": native.pop("normal_exit")
    if fault == "normal_nonzero": native["normal_exit"]["native_returncode"] = 2
    if fault == "adapter_nonzero": final["native_adapter_exit_code"] = 1
    if fault == "log_error": (path / "native.stderr.log").write_text("Abnormal Exit!", encoding="utf-8")
    if fault == "early_pipe": native["stdin_lifecycle"]["writer_closed"] = False
    if fault == "future":
        request = read_json(path / "request.json"); created = time.time() + 100
        request.update(created_unix=created, deadline_unix=created + 300, work_deadline_unix=created + 270)
        for k in ("created_unix", "deadline_unix", "work_deadline_unix"): final[k] = request[k]
        dump(path / "request.json", request)
    if fault == "deadline": final["deadline_unix"] += 1
    if fault == "elapsed": final["elapsed_seconds"] = 301
    if fault == "status_completed": final["status"] = "completed"
    if fault == "error": final["error"] = "do not ignore"
    if fault == "identity_time": final["owned_processes"][-1]["creation_time"] = time.time() + 100
    dump(path / "native_result.json", native); dump(path / "probe_result.json", final)
    bind_runtime(root, monkeypatch)  # offline simulate an independent freeze, still strong checks must fail
    with pytest.raises((BridgeError, ValueError, OSError, KeyError)): r.evaluate_review(root)
    popen.assert_not_called()


@pytest.mark.parametrize("state", ["alive", "reused", "exited", "unknown"])
def test_all_old_owned_processes_must_exit_or_be_proven_reused(license_batch, monkeypatch, state):
    root, path, helper, popen = license_batch
    final = read_json(path / "probe_result.json")
    stamp = final["owned_processes"][0]["creation_time"]
    if state == "alive": monkeypatch.setattr(r.psutil, "Process", lambda pid: types.SimpleNamespace(create_time=lambda: stamp))
    if state == "reused": monkeypatch.setattr(r.psutil, "Process", lambda pid: types.SimpleNamespace(create_time=lambda: time.time()))
    if state == "unknown": monkeypatch.setattr(r.psutil, "Process", lambda pid: (_ for _ in ()).throw(r.psutil.AccessDenied(pid)))
    if state in ("alive", "unknown"):
        with pytest.raises((BridgeError, r.psutil.AccessDenied)): r.process_quiescence(b.Batch002Slot("arm", root), final)
    else:
        states = r.process_quiescence(b.Batch002Slot("arm", root), final)
        assert all(s["state"] == ("exited" if state == "exited" else "pid_reused_not_old_job") for s in states)
    popen.assert_not_called()


def test_original_worker_root_must_exit_even_when_all_native_children_exited(license_batch, monkeypatch):
    root, path, helper, popen = license_batch
    worker = read_json(path / "launcher.json")
    def get(pid):
        if pid == worker["pid"]: return types.SimpleNamespace(create_time=lambda: worker["creation_time"])
        raise r.psutil.NoSuchProcess(pid)
    monkeypatch.setattr(r.psutil, "Process", get)
    with pytest.raises(BridgeError, match="still alive"): r.evaluate_review(root)
    popen.assert_not_called()


@pytest.mark.parametrize("name", ["fluent.exe", "cx2420.exe", "fl2420.exe", "cortex.exe", "runwb2.exe"])
def test_global_native_process_blocks_review(license_batch, monkeypatch, name):
    root, path, helper, popen = license_batch
    process = types.SimpleNamespace(info={"pid": 999, "name": name}, pid=999, parents=lambda: [], create_time=lambda: 1)
    monkeypatch.setattr(r.psutil, "process_iter", lambda *a: [process])
    with pytest.raises(BridgeError): r.evaluate_review(root)
    popen.assert_not_called()


@pytest.mark.parametrize("name", [".ansys-task.lock", ".mesh-inspection-dispatch.lock", ".sizing-dispatch.lock"])
def test_review_never_clears_locks(license_batch, name):
    root, path, helper, popen = license_batch
    dump(root / name, {"owner": "other"}); before = (root / name).read_bytes()
    with pytest.raises(BridgeError): r.evaluate_review(root)
    assert (root / name).read_bytes() == before
    popen.assert_not_called()


def test_fullrecord_retired_even_missing_directory_no_recreation(batch, monkeypatch):
    root, _, popen = batch
    monkeypatch.setitem(b.POLICY, "full_record_retired", True)
    slot = b.FullRecordValidationSlot(root)
    assert slot.status()["remaining_attempts"] == 0
    with pytest.raises(BridgeError, match="retired"): slot.start(b.FULL_RECORD_START)
    assert not slot.safe(slot.ATTEMPT).exists()
    with pytest.raises(w.RuntimeGateError, match="retired"): w.validate_request(slot, slot.safe(slot.ATTEMPT))
    popen.assert_not_called()


def test_review_admits_only_original_QA_once_and_never_reruns_connection(license_batch, monkeypatch):
    root, path, helper, popen = license_batch
    r.write_review(r.REVIEW_CONFIRMATION, root)
    monkeypatch.setattr(b.Batch002Slot, "_inspect_pid", lambda self, pid: {"pid": pid, "exists": True, "creation_time": 123.5})
    qa = b.Batch002Slot("background", root)
    assert qa.start(b.START)["new_worker_started"] is True
    assert qa.start(b.START)["new_worker_started"] is False
    assert popen.call_count == 1 and not b.Batch002Slot("arm", root).safe(b.Batch002Slot("arm", root).ATTEMPT).exists()
    old = b.FullRecordValidationSlot(root)
    assert old.start(b.FULL_RECORD_START)["new_worker_started"] is False and read_json(path / "probe_result.json")["status"] == "failed"


@pytest.mark.parametrize("owned", [True, False])
def test_running_QA_recheck_allows_only_exact_owned_native_descendants(license_batch, monkeypatch, owned):
    root, path, helper, popen = license_batch
    qa = b.Batch002Slot("arm", root)
    qa.safe(qa.ATTEMPT).mkdir(parents=True)
    dump(qa.safe(qa.ATTEMPT + "/launcher.json"), {"job_id": qa.JOB_ID, "pid": 999, "creation_time": 1000.5})
    dump(root / ".ansys-task.lock", {"attempt": str(qa.safe(qa.ATTEMPT)), "pid": 999})
    dump(root / ".mesh-inspection-dispatch.lock", {"job_id": qa.JOB_ID})
    anchor = types.SimpleNamespace(pid=999, create_time=lambda: 1000.5)
    def get(pid):
        if pid == 999: return anchor
        raise r.psutil.NoSuchProcess(pid)
    monkeypatch.setattr(r.psutil, "Process", get)
    native = types.SimpleNamespace(info={"pid": 1001, "name": "fluent.exe"}, pid=1001,
        create_time=lambda: 1001.5, parents=lambda: [anchor] if owned else [])
    monkeypatch.setattr(r.psutil, "process_iter", lambda *a: [native])
    final = read_json(path / "probe_result.json")
    if owned: assert r.process_quiescence(qa, final, active_qa=qa)
    else:
        with pytest.raises(BridgeError): r.process_quiescence(qa, final, active_qa=qa)
    popen.assert_not_called()


def test_official_helper_installation_checks_root_file_hash_and_reparse(tmp_path, monkeypatch):
    helper = tmp_path / "licensingclient/winx64/ansyscl.exe"
    helper.parent.mkdir(parents=True); helper.write_bytes(b"OFFLINE helper")
    approval = {"helper": {"path": str(helper), "size": helper.stat().st_size, "sha256": sha256(helper)}}
    slot = types.SimpleNamespace(_json=lambda name: {"software": {"ansys_root": str(tmp_path)}})
    monkeypatch.setattr(r, "load_approval", lambda s: approval)
    assert r.helper_installation(slot) == helper
    helper.write_bytes(b"different")
    with pytest.raises(BridgeError): r.helper_installation(slot)
    source = (STAGE / "inspection_license_review.py").read_text(encoding="utf-8")
    assert "GetLongPathNameW" in source and "os.path.samefile" in source and "0x400" in source
    with pytest.raises(BridgeError): r.canonical_file("ansyscl.exe")


def test_review_dispatch_is_fixed_readonly_and_refuses_wrong_scope(monkeypatch):
    monkeypatch.setattr(r, "psutil", None)
    calls = []
    import subprocess
    def run(command, **kwargs):
        calls.append((command, kwargs))
        assert "--evaluate" in command and "--qa-slot" in command and command[-1] == "arm"
        assert kwargs["shell"] is False and kwargs["stdin"] == subprocess.DEVNULL
        return types.SimpleNamespace(returncode=0, stdout=b'{"candidate":true}')
    monkeypatch.setattr(subprocess, "run", run)
    assert r.read_only_dispatch(STAGE, qa_slot="arm") == {"candidate": True}
    with pytest.raises(BridgeError): r.read_only_dispatch(STAGE, qa_slot=b.FULL_RECORD)
    assert len(calls) == 1


def test_future_QA_helper_classification_still_requires_complete_normal_exit_and_cleanup(license_batch):
    root, path, helper, popen = license_batch
    final = read_json(path / "probe_result.json")
    assert len(r.classify_live_helpers(b.Batch002Slot("arm", root), final)) == 1
    # A helper classification alone does not satisfy the native/full evidence gate.
    final["native"]["normal_exit"]["normal_exit_observed"] = False
    from inspection_full_record import verify_full_record_artifacts
    with pytest.raises(w.RuntimeGateError): verify_full_record_artifacts(b.FullRecordValidationSlot(root), final["native"], final, classified=True)
    popen.assert_not_called()

"""Offline only: no real slot, license utility, Workbench or solver is invoked."""
import copy
import importlib.util
import json
import os
import sys
import time
import types
from pathlib import Path

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import definition_probe as probe


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=True), encoding="utf-8")


def sources(stage):
    return [{"path": str(path), "sha256": probe.preflight.sha256(path), "size": path.stat().st_size}
            for path in probe.source_paths(stage)]


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    stage = tmp_path / "workspace/new_model_sim/15_dynamic_mesh"
    stage.mkdir(parents=True)
    contract = json.loads((STAGE / "contract.json").read_text(encoding="utf-8"))
    for index, relative in enumerate(contract["preserve_inputs"]):
        path = stage.parent.parent / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("protected-%d" % index).encode("ascii"))
    dump(stage / "contract.json", contract)
    manifest = probe.preflight.snapshot_inputs(contract, stage / "contract.json", stage.parent.parent)
    dump(stage / "frozen/input_manifest.json", manifest)
    for relative in (probe.BACKGROUND_RELATIVE, probe.ARM_RELATIVE):
        path = stage / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode("ascii"))
    cad = {name: {"path": str(stage / relative), "sha256": probe.preflight.sha256(stage / relative)}
           for name, relative in (("background", probe.BACKGROUND_RELATIVE), ("arm", probe.ARM_RELATIVE))}
    receipt = {"mode": "verify", "state": "passed", "source_unchanged": True,
               "cad_domains": {"state": "passed", "independent_reopen_completed": True,
                               "physical_face_ownership_checked": True}, "artifacts": cad}
    dump(stage / probe.VERIFICATION_RELATIVE, receipt)
    auth = dict(probe.AUTHORIZATION_VALUES)
    auth.update({key: "bounded author-supplied metadata" for key in probe.AUTHORIZATION_METADATA})
    auth.update(background_sha256=cad["background"]["sha256"], arm_sha256=cad["arm"]["sha256"],
                cad_verification_sha256=probe.preflight.sha256(stage / probe.VERIFICATION_RELATIVE),
                input_manifest_sha256=probe.preflight.sha256(stage / "frozen/input_manifest.json"),
                contract_sha256=probe.preflight.sha256(stage / "contract.json"))
    dump(stage / probe.AUTHORIZATION_RELATIVE, auth)
    for relative in probe.SOURCE_RELATIVES:
        path = stage / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if relative.startswith("mesh/"):
            path.write_text((STAGE / relative).read_text(encoding="utf-8"), encoding="utf-8")
        else:
            path.write_text("# offline immutable source\n", encoding="utf-8")
    helper = stage.parent / "04_workbench/run_wb_fullenv.py"
    helper.parent.mkdir()
    helper.write_text("# offline immutable helper\n", encoding="utf-8")
    attempt = stage / probe.ATTEMPT_RELATIVE
    attempt.mkdir(parents=True)
    created = time.time()
    request = {"schema_version": 1, "operation": "definition_probe", "job_id": probe.JOB_ID,
               "created_unix": created, "deadline_unix": created + 300, "work_deadline_unix": created + 270,
               "authorization": {"path": str(stage / probe.AUTHORIZATION_RELATIVE),
                                 "sha256": probe.preflight.sha256(stage / probe.AUTHORIZATION_RELATIVE)},
               "sources": sources(stage)}
    dump(attempt / "request.json", request)
    dump(attempt / "launcher.json", {"job_id": probe.JOB_ID, "pid": os.getpid(), "creation_time": 123.5,
                                    "recorded_unix": time.time()})
    monkeypatch.setattr(probe.psutil, "Process", lambda pid: types.SimpleNamespace(create_time=lambda: 123.5))
    monkeypatch.setattr(probe.preflight, "inspect_environment", lambda *a, **k: {"status": "passed", "blockers": []})
    monkeypatch.setattr(probe.run_mesh_job, "child_environment", lambda *a: {})
    # Never fall through to a native application, even if a guard regresses.
    monkeypatch.setattr(probe.subprocess, "Popen", lambda *a, **k: pytest.fail("unexpected real launch"))
    return stage, attempt, request


def rewrite_request(attempt, request):
    dump(attempt / "request.json", request)


def rebind_authorization(stage, attempt, request, auth):
    dump(stage / probe.AUTHORIZATION_RELATIVE, auth)
    request["authorization"]["sha256"] = probe.preflight.sha256(stage / probe.AUTHORIZATION_RELATIVE)
    rewrite_request(attempt, request)


def definitions(stage, attempt):
    contract = json.loads((stage / "contract.json").read_text(encoding="utf-8"))
    settings = probe.domain_settings(contract, attempt)[0]
    names = {name: [index + 1] for index, name in enumerate(settings["required_selections"])}
    stats = dict(probe.SAFE_DEFAULTS, status="definitions_ready", script_entered=True,
                 definitions_ready=True, message_capture_complete=True, messages=[], named_selections=names,
                 body_geometry_ids=[100], body_face_ids=[1, 2, 3, 4, 5], controls=[],
                 mesh_counts_before={"nodes": 0, "elements": 0}, mesh_counts_after={"nodes": 0, "elements": 0},
                 global_readback={key: settings[key] for key in ("global_size_m", "maximum_size_m", "minimum_size_m", "growth_rate")})
    def control(name, kind, ids, values):
        stats["controls"].append({"name": name, "kind": kind, "object_state": "FullyDefined",
                                  "actual_geometry_ids": ids, "expected_geometry_ids": ids, "values": values})
    control("stage15_background_tet", "tet_method", [100], {"method": "AllTriAllTet"})
    for item in settings["face_controls"]:
        control("stage15_" + item["name"], "face_size", names[item["name"]],
                {"type": "ElementSize", "size_m": item["size_m"], "growth_rate": 0.0,
                 "local_growth_override_applied": False, "global_growth_rate_requested": settings["growth_rate"]})
    for index, sphere in enumerate(settings["spheres"]):
        control("stage15_overlap_%d" % index, "sphere", [100],
                dict(sphere, type="SphereOfInfluence", coordinate_object_state="FullyDefined"))
    completion = dict(probe.SAFE_DEFAULTS, status="journal_completed", journal_completed=True, definitions_ready=True)
    return stats, completion, settings

def test_global_growth_is_still_required_after_local_override_removed(sandbox):
    stage, attempt, _ = sandbox
    stats, _, settings = definitions(stage, attempt)
    probe.validate_definition_stats(stats, settings)
    stats["global_readback"]["growth_rate"] = 1.1
    with pytest.raises(probe.RuntimeGateError, match="global size readback"):
        probe.validate_definition_stats(stats, settings)


def test_local_growth_override_is_not_accepted_as_fixed(sandbox):
    stage, attempt, _ = sandbox
    stats, _, settings = definitions(stage, attempt)
    stats["controls"][1]["values"]["local_growth_override_applied"] = True
    with pytest.raises(probe.RuntimeGateError, match="control value"):
        probe.validate_definition_stats(stats, settings)


def write_receipts(stage, attempt, mutate=None):
    stats, completion, settings = definitions(stage, attempt)
    if mutate:
        mutate(stats, completion)
    dump(attempt / "background_meshing_stats.json", stats)
    dump(attempt / "workbench_completion.json", completion)
    (attempt / "workbench.trace.txt").write_text("journal-entered\ngeometry-loaded\nmeshing-script-dispatched\ndefinitions-ready\njournal-completed\n", encoding="utf-8")
    return settings


class FakeTree:
    cleanup = True
    initial_errors = []
    def __init__(self, process, psutil_module):
        self.members = {process.pid: 222.0}
        self.errors = list(self.initial_errors)
        self.closed = False
    def capture(self):
        pass
    def close(self):
        self.closed = True
        return self.cleanup and not self.errors


def fake_launch(monkeypatch, stage, attempt, *, mutate=None, exit_code=0, receipts=True, tree=FakeTree, callback=None):
    calls = []
    process = types.SimpleNamespace(pid=56789, poll=lambda: exit_code)
    def popen(*args, **kwargs):
        calls.append((args, kwargs))
        if receipts:
            write_receipts(stage, attempt, mutate)
        if callback:
            callback()
        return process
    monkeypatch.setattr(probe.subprocess, "Popen", popen)
    monkeypatch.setattr(probe, "OwnedProcessTree", tree)
    return calls


def test_frozen_readback_and_preflight_are_read_only(sandbox):
    stage, attempt, _ = sandbox
    before = {str(path): path.read_bytes() for path in stage.parent.parent.rglob("*") if path.is_file()}
    report = probe.inspect(stage)
    assert report["status"] == "passed"
    assert report["ansys_launched"] is False
    assert len(report["protected_inputs"]["files"]) == 8
    after = {str(path): path.read_bytes() for path in stage.parent.parent.rglob("*") if path.is_file()}
    assert before == after


@pytest.mark.parametrize("key,value", [
    ("maximum_attempts", 2), ("maximum_attempts", True), ("maximum_total_seconds", 301),
    ("cleanup_reserve_seconds", 29), ("full_mesh_allowed", True), ("fluent_allowed", True),
    ("automatic_retry_allowed", True), ("old_fluent_mcp_modification_allowed", True),
    ("original_geometry_modification_allowed", True), ("default_theta_modification_allowed", True),
    ("license_service_modification_allowed", True), ("original_mesh_attempt_allowance_increased", True),
    ("new_service_name", "ansys-fluent"), ("attempt_relative", "runs/elsewhere"),
    ("schema_version", True), ("authorization_source", 1), ("recorded_local", ""),
    ("completion_scope", "x" * 4097), ("unexpected", False),
])
def test_authorization_strict_fields_fail_closed(sandbox, key, value):
    stage, _, _ = sandbox
    path = stage / probe.AUTHORIZATION_RELATIVE
    auth = json.loads(path.read_text(encoding="utf-8"))
    auth[key] = value
    dump(path, auth)
    assert probe.inspect(stage)["status"] == "blocked"


def test_missing_authorization_never_created(sandbox):
    stage, _, _ = sandbox
    path = stage / probe.AUTHORIZATION_RELATIVE
    path.unlink()
    assert probe.inspect(stage)["status"] == "blocked"
    assert not path.exists()


@pytest.mark.parametrize("which", ["protected", "cad", "verification", "manifest", "contract"])
def test_actual_inputs_not_metadata_acceptance(sandbox, which):
    stage, _, _ = sandbox
    paths = {"protected": stage.parent.parent / "new_model_sim/14_hydro_validation/frozen/theta87_baseline.json",
             "cad": stage / probe.BACKGROUND_RELATIVE, "verification": stage / probe.VERIFICATION_RELATIVE,
             "manifest": stage / "frozen/input_manifest.json", "contract": stage / "contract.json"}
    with paths[which].open("ab") as stream:
        stream.write(b"changed")
    assert probe.inspect(stage)["status"] == "blocked"


@pytest.mark.parametrize("mutation", ["not_reopened", "not_passed", "ownership_missing", "artifact_hash", "artifact_path"])
def test_independent_cad_receipt_must_pass(sandbox, mutation):
    stage, attempt, request = sandbox
    path = stage / probe.VERIFICATION_RELATIVE
    value = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "not_reopened":
        value["cad_domains"]["independent_reopen_completed"] = False
    elif mutation == "not_passed":
        value["state"] = "failed"
    elif mutation == "ownership_missing":
        del value["cad_domains"]["physical_face_ownership_checked"]
    elif mutation == "artifact_hash":
        value["artifacts"]["background"]["sha256"] = "a" * 64
    else:
        value["artifacts"]["background"]["path"] = str(stage / probe.ARM_RELATIVE)
    dump(path, value)
    auth = json.loads((stage / probe.AUTHORIZATION_RELATIVE).read_text(encoding="utf-8"))
    auth["cad_verification_sha256"] = probe.preflight.sha256(path)
    rebind_authorization(stage, attempt, request, auth)
    assert probe.inspect(stage)["status"] == "blocked"


def test_success_still_is_not_mesh_or_motion_acceptance(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    calls = fake_launch(monkeypatch, stage, attempt)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "passed", result
    assert len(calls) == 1
    assert calls[0][0][0][1:4] == ["-I", "-X", "-R"]
    assert calls[0][1]["shell"] is False
    assert result["owned_cleanup_verified"] is True
    assert all(result[key] == value for key, value in probe.SAFE_DEFAULTS.items())
    assert not (stage / ".ansys-task.lock").exists()
    assert json.loads((attempt / "job_state.json").read_text())["status"] == "passed"
    assert json.loads((attempt / "probe_result.json").read_text())["status"] == "passed"
    before = (attempt / "probe_result.json").read_bytes()
    with pytest.raises(probe.RuntimeGateError, match="no rerun"):
        probe.run(attempt, stage_root=stage)
    assert len(calls) == 1 and (attempt / "probe_result.json").read_bytes() == before


@pytest.mark.parametrize("mutate", [
    lambda s, c: s.update(script_entered=False),
    lambda s, c: s.update(definitions_ready=False),
    lambda s, c: s.update(mesh_generated=True),
    lambda s, c: s.update(message_capture_complete=False),
    lambda s, c: s.update(messages=[{"severity": "Error", "text": "中文原生错误"}]),
    lambda s, c: s.update(messages=[{"severity": "FatalError", "text": "error"}]),
    lambda s, c: s.update(controls=s["controls"][:1]),
    lambda s, c: s["controls"][0].update(object_state="UnderDefined"),
    lambda s, c: s["controls"][1]["values"].update(size_m=999),
    lambda s, c: s["controls"][1].update(actual_geometry_ids=[999]),
    lambda s, c: s["controls"][-1]["values"].update(centre_m=[0, 0, 0]),
    lambda s, c: s["named_selections"].update(wall_base0=[]),
    lambda s, c: s["global_readback"].update(minimum_size_m=999),
    lambda s, c: c.update(status="running"),
    lambda s, c: c.update(journal_completed=False),
])
def test_incomplete_or_wrong_definition_never_passes(sandbox, monkeypatch, mutate):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, mutate=mutate)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed"
    assert result["owned_cleanup_verified"] is True


def test_zero_exit_without_receipt_is_failure(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, receipts=False)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed"
    assert "receipt" in result["error"]


def test_nonzero_exit_is_failure_even_with_passed_receipts(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, exit_code=3)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and result["exit_code"] == 3


def test_failure_receipt_breaks_wait_before_launcher_exit(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, exit_code=None,
                mutate=lambda s, c: c.update(status="failed", exception="中文后台失联"))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed"
    assert "中文后台失联" in result["error"]
    assert result["elapsed_seconds"] < 2


def test_cancel_before_launch(sandbox):
    stage, attempt, _ = sandbox
    dump(attempt / "cancel_request.json", {"job_id": probe.JOB_ID, "operation": "cancel_definition_probe"})
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cancelled"
    assert result["ansys_launched"] is False


def test_cancel_after_launch_closes_owned_tree(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, exit_code=None, callback=lambda: dump(
        attempt / "cancel_request.json", {"job_id": probe.JOB_ID, "operation": "cancel_definition_probe"}))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cancelled"
    assert result["owned_cleanup_verified"] is True
    assert not (stage / ".ansys-task.lock").exists()


@pytest.mark.parametrize("ledger_update", [{"pid": 1}, {"creation_time": 999}, {"job_id": "foreign-job"}])
def test_launcher_pid_creation_time_must_match(sandbox, ledger_update):
    stage, attempt, _ = sandbox
    path = attempt / "launcher.json"
    value = json.loads(path.read_text())
    value.update(ledger_update)
    dump(path, value)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert "ledger mismatch" in result["error"]


def test_launcher_wait_is_bounded_without_ansys(sandbox, monkeypatch):
    stage, attempt, request = sandbox
    (attempt / "launcher.json").unlink()
    ticks = iter(range(100))
    monkeypatch.setattr(probe.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(probe.time, "sleep", lambda _: None)
    assert probe.LAUNCHER_WAIT_SECONDS == 20
    with pytest.raises(probe.RuntimeGateError, match="20-second"):
        probe.validate_launcher(attempt, request, lambda: 300)


@pytest.mark.parametrize("mutate", [
    lambda r: r.update(deadline_unix=r["created_unix"] + 301),
    lambda r: r.update(work_deadline_unix=r["created_unix"] + 271),
    lambda r: r.update(operation="mesh"),
    lambda r: r.update(schema_version=True),
    lambda r: r.update(deadline_unix=float("nan")),
    lambda r: r.update(sources=r["sources"][:-1]),
    lambda r: r["sources"][0].update(path="../escape"),
    lambda r: r["sources"][0].update(sha256="f" * 64),
    lambda r: r["sources"][0].update(size=True),
    lambda r: r.update(authorization={"path": "foreign", "sha256": "a" * 64}),
])
def test_request_scope_deadline_and_sources_fail_closed(sandbox, mutate):
    stage, attempt, request = sandbox
    mutate(request)
    rewrite_request(attempt, request)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]


def test_existing_lock_not_removed_or_replaced(sandbox):
    stage, attempt, _ = sandbox
    path = stage / ".ansys-task.lock"
    dump(path, {"token": "foreign-owner"})
    before = path.read_bytes()
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed"
    assert path.read_bytes() == before


def test_cleanup_uncertain_retains_lock(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    class UncleanTree(FakeTree):
        cleanup = False
    fake_launch(monkeypatch, stage, attempt, tree=UncleanTree)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert result["owned_cleanup_verified"] is False
    assert (stage / ".ansys-task.lock").exists()


def test_unknown_ownership_retains_lock(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    class UnknownTree(FakeTree):
        initial_errors = ["PID reused / unavailable"]
    fake_launch(monkeypatch, stage, attempt, tree=UnknownTree)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert (stage / ".ansys-task.lock").exists()


def test_lock_token_changed_retains_new_owners_lock(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    def replace_lock():
        dump(stage / ".ansys-task.lock", {"token": "different-owner"})
    fake_launch(monkeypatch, stage, attempt, callback=replace_lock)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert json.loads((stage / ".ansys-task.lock").read_text())["token"] == "different-owner"


def test_work_deadline_stops_at_270_including_ledger_time(sandbox, monkeypatch):
    stage, attempt, request = sandbox
    request["created_unix"] = time.time() - 271
    request["deadline_unix"] = request["created_unix"] + 300
    request["work_deadline_unix"] = request["created_unix"] + 270
    rewrite_request(attempt, request)
    ledger = json.loads((attempt / "launcher.json").read_text())
    ledger["recorded_unix"] = time.time()
    dump(attempt / "launcher.json", ledger)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert "270-second" in result["error"]


def test_changed_input_after_launch_cannot_pass(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, callback=lambda: (stage / probe.ARM_RELATIVE).write_bytes(b"changed"))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and result["owned_cleanup_verified"] is True
    assert "changed" in result["error"]


def test_wrong_attempt_rejected_without_claim(sandbox):
    stage, attempt, _ = sandbox
    wrong = attempt.parent / "attempt_002"
    wrong.mkdir()
    with pytest.raises(probe.RuntimeGateError, match="fixed"):
        probe.run(wrong, stage_root=stage)
    assert not (wrong / "worker_claim.json").exists()


def test_junction_attribute_and_traversal_rejected(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    with pytest.raises(probe.RuntimeGateError):
        probe.checked_path(attempt / "../attempt_001/request.json", stage)
    original = Path.lstat
    target = attempt / "request.json"
    def lstat(path):
        value = original(path)
        if path == target:
            return types.SimpleNamespace(st_mode=value.st_mode, st_file_attributes=0x400)
        return value
    monkeypatch.setattr(Path, "lstat", lstat)
    with pytest.raises(probe.RuntimeGateError, match="reparse"):
        probe.checked_path(target, stage)


def test_duplicate_json_keys_rejected(sandbox):
    _, attempt, _ = sandbox
    path = attempt / "duplicate.json"
    path.write_text('{"status":"passed","status":"failed"}', encoding="utf-8")
    with pytest.raises(probe.RuntimeGateError, match="duplicate"):
        probe.load_json(path, attempt)


def test_templates_have_only_definition_operations(sandbox):
    stage, attempt, _ = sandbox
    frozen = probe.validate_frozen(stage)
    journal = probe.prepare_scripts(attempt, stage, frozen)
    assert probe.audit_template(journal.read_text(encoding="utf-8"), journal=True)
    assert probe.audit_template((attempt / "meshing_definition_probe.py").read_text(encoding="utf-8"))
    assert "AllDependencies=False" in journal.read_text(encoding="utf-8")
    generated = journal.read_text(encoding="utf-8")
    assert generated.count("Save(") == 1
    assert 'Save(FilePath=C["project"], Overwrite=False)' in generated
    assert "stage15_definition_probe.wbpj" in generated
    assert generated.index("geometry_component.Update(") < generated.index("Save(") < generated.index("mesh_container.Edit(")


@pytest.mark.parametrize("unsafe,journal", [
    ("mesh.GenerateMesh()", False), ("mesh.GenerateMesh()", True),
    ("mesh_component.Update(AllDependencies=False)", True),
    ("geometry_component.Update(AllDependencies=True)", True),
    ("container.Edit()", True), ("fluent.iterate(1)", False),
    ("exec('hidden')", True), ("eval('hidden')", False),
])
def test_ast_and_text_guard_forbid_generation_paths(unsafe, journal):
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(unsafe, journal=journal)


def test_bounded_native_call_returns_or_times_out():
    assert probe.bounded_call(lambda: 5, 1) == 5
    import threading
    gate = threading.Event()
    with pytest.raises(probe.UncertainCall):
        probe.bounded_call(lambda: gate.wait(1), 0.01)
    gate.set()


def test_cli_only_json_and_nonzero_failure(capsys, monkeypatch):
    monkeypatch.setattr(probe, "inspect", lambda: dict(probe.SAFE_DEFAULTS, status="blocked", blockers=["中文阻塞"]))
    assert probe.main(["--preflight"]) == 2
    output = json.loads(capsys.readouterr().out)
    assert output["blockers"] == ["中文阻塞"]
    assert probe.main(["--preflight", "--attempt", "foreign"]) == 2
    assert json.loads(capsys.readouterr().out)["status"] == "failed"


def test_popen_failure_consumes_claim_and_releases_known_safe_lock(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    def fail(*a, **k):
        raise OSError("mock launch failed")
    monkeypatch.setattr(probe.subprocess, "Popen", fail)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert result["owned_cleanup_verified"] is True
    assert not (stage / ".ansys-task.lock").exists()
    assert (attempt / "worker_claim.json").exists()


def test_uncertain_popen_retains_lock_without_retry(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    calls = []
    def fail(*a, **k):
        calls.append(1)
        raise probe.UncertainCall("mock Popen timeout")
    monkeypatch.setattr(probe.subprocess, "Popen", fail)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified" and not result["owned_cleanup_verified"]
    assert (stage / ".ansys-task.lock").exists()
    assert len(calls) == 1


def test_cleanup_budget_is_not_restarted_after_long_work(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    original_time = time.time
    original_mono = time.monotonic
    offset = [0]
    monkeypatch.setattr(probe.time, "time", lambda: original_time() + offset[0])
    monkeypatch.setattr(probe.time, "monotonic", lambda: original_mono() + offset[0])
    class BudgetTree(FakeTree):
        def capture(self):
            offset[0] = 290
        def close(self):
            pytest.fail("13-second close may not run with only 10 seconds remaining")
    fake_launch(monkeypatch, stage, attempt, tree=BudgetTree)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified" and not result["owned_cleanup_verified"]
    assert "reserve" in result["cleanup_error"]
    assert (stage / ".ansys-task.lock").exists()
    assert result["elapsed_seconds"] >= 290


def test_expired_total_budget_cannot_launch(sandbox):
    stage, attempt, request = sandbox
    request["created_unix"] = time.time() - 301
    request["deadline_unix"] = request["created_unix"] + 300
    request["work_deadline_unix"] = request["created_unix"] + 270
    rewrite_request(attempt, request)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert "expired" in result["error"]


def test_source_changed_during_script_preparation_prevents_launch(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    original = probe.prepare_scripts
    def prepare(*a):
        journal = original(*a)
        (stage / "mesh/wb_definition_probe.wbjn.in").write_text("changed", encoding="utf-8")
        return journal
    monkeypatch.setattr(probe, "prepare_scripts", prepare)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert "source changed" in result["error"]


@pytest.mark.parametrize("unsafe", ["import subprocess", "os.system('anything')", "Save()", "File.Save()"])
def test_template_launch_and_project_save_paths_forbidden(unsafe):
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(unsafe, journal=True)


@pytest.mark.parametrize("phase", ["mesh_counts_before", "mesh_counts_after"])
@pytest.mark.parametrize("counts", [None, {}, {"nodes": 0}, {"nodes": 0, "elements": False},
    {"nodes": 0.0, "elements": 0}, {"nodes": -1, "elements": 0}, {"nodes": 0, "elements": 1},
    {"nodes": "0", "elements": 0}, {"nodes": 0, "elements": 0, "extra": 0}])
def test_worker_requires_complete_native_integer_zero_counts(sandbox, phase, counts):
    stage, attempt, _ = sandbox
    stats, _, settings = definitions(stage, attempt)
    stats[phase] = counts
    with pytest.raises(probe.RuntimeGateError, match="native mesh counts"):
        probe.validate_definition_stats(stats, settings)


def test_missing_native_count_evidence_cannot_pass_worker(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, mutate=lambda s, c: s.pop("mesh_counts_after"))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and "native mesh counts" in result["error"]


def test_empty_mesh_database_is_not_mistaken_for_generated_mesh(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, callback=lambda: (attempt / "empty.mshdb").write_bytes(b"empty native database"))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "passed", result
    assert result["definition_receipt"]["mesh_counts_before"] == {"nodes": 0, "elements": 0}
    assert result["definition_receipt"]["mesh_counts_after"] == {"nodes": 0, "elements": 0}
    assert result["mesh_generated"] is False


def test_inspect_renders_without_creating_even_the_slot_parent(sandbox):
    stage, attempt, _ = sandbox
    import shutil
    shutil.rmtree(attempt.parent)
    before = {str(path): path.read_bytes() for path in stage.rglob("*") if path.is_file()}
    result = probe.inspect(stage)
    assert result["status"] == "passed", result
    assert result["template_preflight"]["rendered_in_memory"] is True
    assert result["template_preflight"]["files_written"] is False
    assert not attempt.parent.exists()
    assert before == {str(path): path.read_bytes() for path in stage.rglob("*") if path.is_file()}


@pytest.mark.parametrize("mutation", ["syntax", "forbidden", "missing_placeholder", "duplicate_placeholder", "config"])
def test_inspect_blocks_template_or_configuration_errors_in_memory(sandbox, monkeypatch, mutation):
    stage, attempt, _ = sandbox
    path = stage / "mesh/meshing_definition_probe.py.in"
    original = path.read_text(encoding="utf-8")
    if mutation == "config":
        monkeypatch.setattr(probe, "domain_settings", lambda *a: (_ for _ in ()).throw(ValueError("mock configuration error")))
    elif mutation == "syntax":
        path.write_text(original + "\nif (", encoding="utf-8")
    elif mutation == "forbidden":
        path.write_text(original + "\nmesh.GenerateMesh()", encoding="utf-8")
    elif mutation == "missing_placeholder":
        path.write_text(original.replace("__CONFIG_JSON__", "'{}'"), encoding="utf-8")
    else:
        path.write_text(original + "\n# __CONFIG_JSON__", encoding="utf-8")
    result = probe.inspect(stage)
    assert result["status"] == "blocked" and result["blockers"]
    assert not (attempt / "meshing_definition_probe.py").exists()
    assert not (attempt / "wb_definition_probe.wbjn").exists()


def test_prepare_and_inspect_reuse_identical_memory_renderer(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    calls = []
    original = probe.render_scripts
    def render(*args):
        result = original(*args)
        calls.append(result)
        return result
    monkeypatch.setattr(probe, "render_scripts", render)
    assert probe.inspect(stage)["status"] == "passed"
    assert len(calls) == 1 and not (attempt / "wb_definition_probe.wbjn").exists()
    path = probe.prepare_scripts(attempt, stage, probe.validate_frozen(stage))
    assert len(calls) == 2
    assert calls[0] == calls[1]
    assert path.read_text(encoding="utf-8") == calls[0]["journal_script"]
    assert (attempt / "meshing_definition_probe.py").read_text(encoding="utf-8") == calls[0]["meshing_script"]


def test_launcher_ledger_can_arrive_after_fifteen_seconds(sandbox, monkeypatch):
    _, attempt, request = sandbox
    (attempt / "launcher.json").unlink()
    now = [0]
    monkeypatch.setattr(probe.time, "monotonic", lambda: now[0])
    def pause(seconds):
        now[0] += 1
        if now[0] == 15:
            dump(attempt / "launcher.json", {"job_id": probe.JOB_ID, "pid": os.getpid(),
                 "creation_time": 123.5, "recorded_unix": time.time()})
    monkeypatch.setattr(probe.time, "sleep", pause)
    ledger = probe.validate_launcher(attempt, request, lambda: 300 - now[0])
    assert ledger["pid"] == os.getpid() and now[0] == 15


def test_launcher_wait_uses_remaining_total_budget_not_fresh_twenty_seconds(sandbox, monkeypatch):
    _, attempt, request = sandbox
    (attempt / "launcher.json").unlink()
    now = [0]
    monkeypatch.setattr(probe.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(probe.time, "sleep", lambda _: now.__setitem__(0, now[0] + 1))
    with pytest.raises(probe.RuntimeGateError, match="20-second"):
        probe.validate_launcher(attempt, request, lambda: max(0, 3 - now[0]))
    assert now[0] == 3


def test_fixed_new_project_save_is_the_only_allowed_save():
    source = 'Save(FilePath=C["project"], Overwrite=False)'
    assert probe.audit_template(source, journal=True)
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(source, journal=False)


@pytest.mark.parametrize("source", [
    'Save(FilePath=C["project"], Overwrite=True)',
    'Save(FilePath=C["project"])',
    'Save(FilePath=C["cad_path"], Overwrite=False)',
    'Save(FilePath="old_project.wbpj", Overwrite=False)',
    'Save(C["project"], Overwrite=False)',
    'Save(FilePath=C["project"], Overwrite=False, Extra=True)',
    'Save(FilePath=C["project"], Overwrite=False)\nSave(FilePath=C["project"], Overwrite=False)',
    'C["project"] = "old_project.wbpj"\nSave(FilePath=C["project"], Overwrite=False)',
])
def test_save_arbitrary_path_overwrite_and_duplicate_are_forbidden(source):
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(source, journal=True)


def test_existing_temporary_project_prevents_native_launch(sandbox):
    stage, attempt, _ = sandbox
    project = attempt / "stage15_definition_probe.wbpj"
    project.write_bytes(b"prior project must not change")
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert project.read_bytes() == b"prior project must not change"
    assert "existing definition probe project" in result["error"]


def test_existing_claim_without_result_still_prevents_direct_rerun(sandbox):
    stage, attempt, _ = sandbox
    dump(attempt / "worker_claim.json", {"job_id": probe.JOB_ID, "pid": 999})
    with pytest.raises(FileExistsError):
        probe.run(attempt, stage_root=stage)
    assert not (attempt / "probe_result.json").exists()


def test_terminal_result_alone_prevents_rerun(sandbox):
    stage, attempt, _ = sandbox
    dump(attempt / "probe_result.json", {"job_id": probe.JOB_ID, "status": "failed"})
    before = (attempt / "probe_result.json").read_bytes()
    with pytest.raises(probe.RuntimeGateError, match="no rerun"):
        probe.run(attempt, stage_root=stage)
    assert not (attempt / "worker_claim.json").exists()
    assert (attempt / "probe_result.json").read_bytes() == before


@pytest.mark.parametrize("constant", ["NaN", "Infinity", "-Infinity"])
def test_nonfinite_json_is_rejected_before_it_can_poison_state(sandbox, constant):
    _, attempt, _ = sandbox
    path = attempt / "nonfinite.json"
    path.write_text('{"value":' + constant + '}', encoding="utf-8")
    with pytest.raises(probe.RuntimeGateError, match="nonfinite"):
        probe.load_json(path, attempt)


def test_trace_completion_cannot_be_substituted_by_json(sandbox, monkeypatch):
    stage, attempt, _ = sandbox
    fake_launch(monkeypatch, stage, attempt, callback=lambda: (attempt / "workbench.trace.txt").write_text("journal-entered\n"))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and "phase evidence" in result["error"]

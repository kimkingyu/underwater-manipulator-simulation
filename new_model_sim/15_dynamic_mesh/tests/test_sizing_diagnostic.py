"""Offline fixed-batch and cleanup tests; no real slot or native process is used."""
import copy
import json
import os
import sys
import time
import types
from pathlib import Path

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import sizing_diagnostic as probe


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    stage = tmp_path / "workspace/new_model_sim/15_dynamic_mesh"
    stage.mkdir(parents=True)
    for path in probe.source_paths(stage):
        path.parent.mkdir(parents=True, exist_ok=True)
        original = STAGE / path.relative_to(stage) if path.is_relative_to(stage) else None
        path.write_text(original.read_text(encoding="utf-8") if original and original.suffix == ".in" else "# mock source\n", encoding="utf-8")
    contract = json.loads((STAGE / "contract.json").read_text(encoding="utf-8"))
    dump(stage / "contract.json", contract)
    dump(stage / probe.AUTHORIZATION_RELATIVE, dict(probe.AUTHORIZATION_VALUES))
    old = stage / probe.AUTHORIZATION_VALUES["baseline_authorization_relative"]
    old.write_text("mock original grant", encoding="utf-8")
    original_digest = probe.digest
    monkeypatch.setattr(probe, "digest", lambda path, root: probe.BASELINE_AUTHORIZATION_SHA256 if Path(path) == old else original_digest(path, root))
    auth = probe.validate_authorization(stage)
    frozen = dict(contract=contract, protected_inputs={"status": "passed"}, cad={"background": {"path": str(stage / "background.scdoc")}},
                  authorization=auth, baseline_authorization={"path": str(old), "sha256": probe.BASELINE_AUTHORIZATION_SHA256})
    monkeypatch.setattr(probe, "validate_frozen", lambda root: copy.deepcopy(frozen))
    attempt = stage / probe.ATTEMPT_ROOT_RELATIVE / "attempt_001"
    attempt.mkdir(parents=True)
    created = time.time()
    request = dict(schema_version=1, operation="sizing_diagnostic", job_id="sizing-b001-a001", attempt_index=1,
                   profile="baseline", hypothesis="observe unmodified local defaults", created_unix=created,
                   deadline_unix=created + 300, work_deadline_unix=created + 270, authorization=auth,
                   sources=[dict(path=str(path), size=path.stat().st_size, sha256=original_digest(path, stage.parent)) for path in probe.source_paths(stage)])
    dump(attempt / "request.json", request)
    dump(attempt / "launcher.json", dict(job_id=request["job_id"], pid=os.getpid(), creation_time=123.5, recorded_unix=time.time()))
    monkeypatch.setattr(probe.psutil, "Process", lambda pid: types.SimpleNamespace(create_time=lambda: 123.5))
    monkeypatch.setattr(probe.preflight, "inspect_environment", lambda *a, **k: dict(status="passed", blockers=[]))
    monkeypatch.setattr(probe.run_mesh_job, "child_environment", lambda *a: {})
    monkeypatch.setattr(probe.subprocess, "Popen", lambda *a, **k: pytest.fail("unexpected native launch"))
    return stage, attempt, request, frozen


def native_receipt(request, phase, state="UnderDefined"):
    override_applied = request["profile"] not in probe.UNSET_LOCAL_GROWTH_PROFILES
    fields = {name: dict(available=False, error="not exposed") for name in (
        "ScopingMethod", "NamedSelection", "Active", "Suppressed", "Behavior", "CaptureCurvature", "CurvatureNormalAngle",
        "CaptureProximity", "LocalMinimumSize", "ProximityMinimumSize", "ProximityGapFactor", "ProximitySizeSources", "DefeatureSize")}
    fields.update(Location=dict(available=True, value=[7]), Type=dict(available=True, value="ElementSize"),
                  ElementSize=dict(available=True, value=dict(value=2.0, unit="mm", text="2 [mm]")), GrowthRate=dict(available=True, value=1.3 if override_applied else 0.0))
    step = ("set_growth_rate" if override_applied else "leave_local_growth_unset") if phase == "define" else "final"
    control = dict(step=step, name="stage15_wall_base0", object_state=state, mesh_counts=dict(nodes=0, elements=0), fields=fields)
    value = dict(schema_version=1, job_id=request["job_id"], profile=request["profile"], phase=phase, status="completed",
                diagnostic_completed=True, zero_mesh_verified=True, mesh_generated=False, flow_iterations=0,
                messages=[], message_capture_complete=True, messages_truncated=False, message_capture_error=None,
                message_characters=0, native_errors_present=False, local_growth_override_applied=override_applied,
                mesh_motion_verified=False, mesh_counts=dict(nodes=0, elements=0), snapshots=[control], control=control,
                control_fully_defined=state == "FullyDefined", tet_object_state="FullyDefined", scope="background_tet_and_wall_base0_only",
                global_readback={"ElementSize": dict(available=True, value=dict(value=.15, unit="m")),
                                 "MaximumSize": dict(available=True, value=dict(value=.2, unit="m")),
                                 "MinimumSize": dict(available=True, value=dict(value=.0003, unit="m")),
                                 "GrowthRate": dict(available=True, value=1.3),
                                 "CaptureCurvature": dict(available=True, value=False),
                                 "CaptureProximity": dict(available=True, value=False)})
    if request["profile"] == "background_all_controls":
        value["scope"] = "background_all_required_controls"
        value.update(full_background_data(state))
    return value


def full_background_data(state="FullyDefined"):
    contract = json.loads((STAGE / "contract.json").read_text(encoding="utf-8"))
    settings = probe.domain_settings(contract, Path("."))[0]
    named = {"wall_base0": [7], "wall_auv": [8], "inlet": [9], "outlet": [10], "tank_wall": [11]}
    q = lambda number: dict(value=number, unit="m", text=str(number) + " [m]")
    rows = [dict(name="stage15_background_tet", kind="tet_method", object_state=state,
                 expected_geometry_ids=[1], actual_geometry_ids=[1], method="AllTriAllTet")]
    for config in settings["face_controls"]:
        rows.append(dict(name="stage15_" + config["name"], kind="face_size", object_state=state,
                         expected_geometry_ids=named[config["name"]], actual_geometry_ids=list(named[config["name"]]),
                         source_named_selection=config["name"], type="ElementSize", element_size=q(config["size_m"]),
                         expected_element_size_m=config["size_m"], local_growth_raw=dict(available=True, value=0.0),
                         local_growth_override_applied=False))
    config = settings["spheres"][0]
    rows.append(dict(name="stage15_overlap_0", kind="sphere", object_state=state, expected_geometry_ids=[1], actual_geometry_ids=[1],
                     type="SphereOfInfluence", element_size=q(config["size_m"]), expected_element_size_m=config["size_m"],
                     sphere_radius=q(config["radius_m"]), expected_sphere_radius_m=config["radius_m"],
                     expected_sphere_center_m=config["centre_m"], sphere_center_object_id=99,
                     coordinate_system=dict(name="stage15_overlap_0", object_id=99, object_state=state,
                       origins={key: q(number) for key, number in zip(("OriginX", "OriginY", "OriginZ"), config["centre_m"])})))
    return dict(background_controls=rows, named_scopes=named, body_geometry_ids=[1], body_face_ids=[7, 8, 9, 10, 11])


def mock_success(monkeypatch, attempt, request, *, cleanup=True, state="UnderDefined"):
    states = []
    original = probe.write_state
    monkeypatch.setattr(probe, "write_state", lambda path, value: (states.append(copy.deepcopy(value)), original(path, value))[-1])
    def launch(*args, **kwargs):
        for phase, name in probe.RECEIPTS.items():
            dump(attempt / name, native_receipt(request, phase, state))
        dump(attempt / "workbench_completion.json", dict(job_id=request["job_id"], profile=request["profile"], status="completed",
             journal_completed=True, diagnostic_completed=True, mesh_generated=False, flow_iterations=0, mesh_motion_verified=False))
        return types.SimpleNamespace(pid=444, poll=lambda: 0)
    class Tree:
        def __init__(self, *a):
            self.errors = []
            self.members = {444: 321.5}
        def capture(self):
            return None
        def close(self):
            return cleanup
    monkeypatch.setattr(probe.subprocess, "Popen", launch)
    monkeypatch.setattr(probe, "OwnedProcessTree", Tree)
    return states


def test_fixed_paths_and_no_fourth(sandbox):
    stage, attempt, request, _ = sandbox
    assert probe.attempt_identity(attempt, stage)[1:] == (1, "sizing-b001-a001")
    for index in (0, 4):
        with pytest.raises(probe.RuntimeGateError):
            probe.attempt_identity(attempt.parent / ("attempt_%03d" % index), stage, must_exist=False)


def test_request_and_fourteen_sources(sandbox):
    stage, attempt, request, _ = sandbox
    assert probe.validate_request(attempt, stage) == request
    assert len(probe.source_paths(stage)) == 14
    request["sources"].pop()
    dump(attempt / "request.json", request)
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_request(attempt, stage)


@pytest.mark.parametrize("field,value", [("attempt_index", True), ("profile", "script"), ("hypothesis", " "),
    ("hypothesis", "x" * 2001), ("deadline_unix", float("nan")), ("schema_version", True), ("extra", "command")])
def test_request_rejects_outside_contract(sandbox, field, value):
    stage, attempt, request, _ = sandbox
    request[field] = value
    dump(attempt / "request.json", request)
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_request(attempt, stage)


def test_new_grant_not_original_grant(sandbox):
    stage, attempt, request, _ = sandbox
    request["authorization"]["path"] = str(stage / probe.AUTHORIZATION_VALUES["baseline_authorization_relative"])
    dump(attempt / "request.json", request)
    with pytest.raises(probe.RuntimeGateError, match="new sizing"):
        probe.validate_request(attempt, stage)


@pytest.mark.parametrize("change", [{"maximum_attempts": 4}, {"full_mesh_allowed": True}, {"allowed_profiles": ["baseline"]}, {"arbitrary_script": "x"}])
def test_grant_rejects_scope_expansion(sandbox, change):
    stage, _, _, _ = sandbox
    dump(stage / probe.AUTHORIZATION_RELATIVE, dict(probe.AUTHORIZATION_VALUES, **change))
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_authorization(stage)


def test_launcher_identity(sandbox):
    _, attempt, request, _ = sandbox
    assert probe.validate_launcher(attempt, request, lambda: 100)["pid"] == os.getpid()
    dump(attempt / "launcher.json", dict(job_id=request["job_id"], pid=os.getpid(), creation_time=999, recorded_unix=time.time()))
    with pytest.raises(probe.RuntimeGateError, match="ledger mismatch"):
        probe.validate_launcher(attempt, request, lambda: 100)


def test_readonly_inspect_has_no_slot_or_render_files(sandbox):
    stage, attempt, _, _ = sandbox
    before = sorted(str(path) for path in stage.rglob("*"))
    result = probe.inspect(stage)
    assert result["status"] == "passed"
    assert result["template_preflight"]["files_written"] is False
    assert before == sorted(str(path) for path in stage.rglob("*"))
    assert not (attempt / "worker_claim.json").exists()


def test_readonly_render_with_no_attempt_parent(sandbox):
    stage, _, _, frozen = sandbox
    shutil = __import__("shutil")
    shutil.rmtree(stage / "runs")
    result = probe.render_scripts(stage / probe.ATTEMPT_ROOT_RELATIVE / "attempt_003", stage, frozen)
    assert result["configuration"]["job_id"] == "sizing-b001-a003"
    assert not (stage / "runs").exists()


def test_completed_underdefined_is_not_mesh_acceptance(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    states = mock_success(monkeypatch, attempt, request)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed", result
    assert result["diagnostic_completed"] is True
    assert result["control_fully_defined"] is False
    assert result["zero_mesh_verified"] is True
    assert result["mesh_generated"] is False and result["flow_iterations"] == 0
    assert result["inputs_reverified_after_cleanup"] and result["sources_reverified_after_cleanup"]
    assert all(row["owned_cleanup_verified"] is False for row in states[:-1])
    assert result["owned_cleanup_verified"] is True
    with pytest.raises(probe.RuntimeGateError):
        probe.run(attempt, stage_root=stage)


def test_cleanup_unverified_retains_lock(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    mock_success(monkeypatch, attempt, request, cleanup=False)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert result["owned_cleanup_verified"] is False
    assert (stage / ".ansys-task.lock").exists()


def test_cancel_before_launch_is_preserved(sandbox):
    stage, attempt, request, _ = sandbox
    dump(attempt / "cancel_request.json", dict(job_id=request["job_id"], operation="cancel_sizing_diagnostic", created_unix=time.time()))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cancelled"
    assert result["ansys_launched"] is False


def test_early_validation_failure_and_exclusive_claim(sandbox):
    stage, attempt, request, _ = sandbox
    request["profile"] = "invalid"
    dump(attempt / "request.json", request)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and not result["ansys_launched"]
    assert (attempt / "worker_claim.json").exists()


@pytest.mark.parametrize("mutation", ["nonzero", "missing_core", "false_fully_defined", "wrong_job", "wrong_physical", "bool_count", "missing_optional", "global_change"])
def test_receipt_fail_closed(sandbox, mutation):
    _, _, request, _ = sandbox
    value = native_receipt(request, "define")
    if mutation == "nonzero":
        value["mesh_counts"]["nodes"] = 1
    elif mutation == "missing_core":
        value["control"]["fields"]["ElementSize"] = dict(available=False, error="unavailable")
    elif mutation == "false_fully_defined":
        value["control_fully_defined"] = True
    elif mutation == "wrong_job":
        value["job_id"] = "other"
    elif mutation == "bool_count":
        value["mesh_counts"]["nodes"] = False
    elif mutation == "missing_optional":
        value["control"]["fields"].pop("Active")
    elif mutation == "global_change":
        value["global_readback"]["ElementSize"]["value"]["value"] = .2
    else:
        value["control"]["fields"]["ElementSize"]["value"]["value"] = 20
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_receipt(value, "define", request, dict(local_size_m=.002, growth_rate=1.3, global_size_m=.15, maximum_size_m=.2, minimum_size_m=.0003))


def test_post_cleanup_real_source_rehash_detects_change(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    mock_success(monkeypatch, attempt, request)
    original_tree = probe.OwnedProcessTree
    class MutatingClose(original_tree):
        def close(self):
            (stage / "sizing_diagnostic.py").write_text("changed during owned cleanup", encoding="utf-8")
            return True
    monkeypatch.setattr(probe, "OwnedProcessTree", MutatingClose)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed"
    assert result["owned_cleanup_verified"] is True
    assert result["sources_reverified_after_cleanup"] is False
    assert "frozen source" in result["input_recheck_error"]


def test_uncertain_launch_keeps_lock_without_retry(sandbox, monkeypatch):
    stage, attempt, _, _ = sandbox
    launches = []
    def uncertain(*a, **k):
        launches.append(True)
        raise probe.UncertainCall("native launch exceeded bounded budget")
    monkeypatch.setattr(probe.subprocess, "Popen", uncertain)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "cleanup_unverified"
    assert not result["owned_cleanup_verified"]
    assert (stage / ".ansys-task.lock").exists()
    assert launches == [True]


def test_work_deadline_prevents_new_native_launch(sandbox):
    stage, attempt, request, _ = sandbox
    request.update(created_unix=time.time() - 271)
    request.update(deadline_unix=request["created_unix"] + 300, work_deadline_unix=request["created_unix"] + 270)
    dump(attempt / "request.json", request)
    # A valid old ledger proves the worker identity; it does not extend its budget.
    dump(attempt / "launcher.json", dict(job_id=request["job_id"], pid=os.getpid(), creation_time=123.5, recorded_unix=request["created_unix"] + 1))
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "failed" and "270-second" in result["error"]
    assert result["ansys_launched"] is False


def test_existing_claim_prevents_second_worker(sandbox):
    stage, attempt, request, _ = sandbox
    dump(attempt / "worker_claim.json", dict(job_id=request["job_id"], pid=1))
    with pytest.raises(FileExistsError):
        probe.run(attempt, stage_root=stage)
    assert not (attempt / "probe_result.json").exists()


def test_fullydefined_independently_reported(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    mock_success(monkeypatch, attempt, request)
    launch = probe.subprocess.Popen
    def fullydefined(*a, **k):
        process = launch(*a, **k)
        for phase, name in probe.RECEIPTS.items():
            dump(attempt / name, native_receipt(request, phase, "FullyDefined"))
        return process
    monkeypatch.setattr(probe.subprocess, "Popen", fullydefined)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed"
    assert result["control_fully_defined"] is True
    assert all(row["control_fully_defined"] for row in result["phase_summaries"].values())
    assert result["mesh_generated"] is False and result["mesh_motion_verified"] is False


@pytest.mark.parametrize("error_flag", [True, None])
def test_native_messages_terminal_and_phase_summaries(sandbox, monkeypatch, error_flag):
    stage, attempt, request, _ = sandbox
    mock_success(monkeypatch, attempt, request)
    launch = probe.subprocess.Popen
    def with_messages(*a, **k):
        process = launch(*a, **k)
        for phase, name in probe.RECEIPTS.items():
            value = native_receipt(request, phase)
            if error_flag is True:
                value.update(messages=[dict(severity="Error", text="native sizing diagnostic")],
                             message_characters=len("Errornative sizing diagnostic"), native_errors_present=True)
            else:
                value.update(message_capture_complete=False, message_capture_error="native messages unavailable", native_errors_present=None)
            dump(attempt / name, value)
        return process
    monkeypatch.setattr(probe.subprocess, "Popen", with_messages)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed", result
    assert result["native_errors_present"] is error_flag
    assert result["message_capture_complete"] is (error_flag is True)
    assert all(row["native_errors_present"] is error_flag for row in result["phase_summaries"].values())
    assert result["control_fully_defined"] is False and result["zero_mesh_verified"] is True


@pytest.mark.parametrize("change", [dict(message_capture_complete=False), dict(native_errors_present=True),
    dict(message_characters=20001), dict(messages_truncated=True), dict(message_capture_error="failed")])
def test_messages_cannot_fake_complete_or_empty_success(sandbox, change):
    _, _, request, _ = sandbox
    value = native_receipt(request, "define")
    value.update(change)
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_receipt(value, "define", request)


def test_omit_local_growth_worker_raw_zero_with_global_unchanged(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    request["profile"] = "omit_local_growth"
    dump(attempt / "request.json", request)
    mock_success(monkeypatch, attempt, request)
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed", result
    assert result["local_growth_override_applied"] is False
    assert result["local_growth_readback_semantics"] == "raw_local_default_no_physical_or_inheritance_claim"
    for value in result["receipts"].values():
        assert value["control"]["fields"]["GrowthRate"]["value"] == 0.0
        assert value["global_readback"]["GrowthRate"]["value"] == 1.3
        assert value["local_growth_override_applied"] is False
    assert result["zero_mesh_verified"] and result["owned_cleanup_verified"]
    assert result["mesh_generated"] is False and result["flow_iterations"] == 0


@pytest.mark.parametrize("change", ["local_nonzero", "wrong_provenance", "override_step", "global_growth_change"])
def test_omit_local_growth_receipt_rejects_extra_interventions(sandbox, change):
    _, _, request, _ = sandbox
    request["profile"] = "omit_local_growth"
    value = native_receipt(request, "define")
    if change == "local_nonzero":
        value["control"]["fields"]["GrowthRate"]["value"] = 1.3
    elif change == "wrong_provenance":
        value["local_growth_override_applied"] = True
    elif change == "override_step":
        value["control"]["step"] = "set_growth_rate"
    else:
        value["global_readback"]["GrowthRate"]["value"] = 0.0
    with pytest.raises(probe.RuntimeGateError):
        probe.validate_receipt(value, "define", request, dict(local_size_m=.002, growth_rate=1.3, global_size_m=.15, maximum_size_m=.2, minimum_size_m=.0003))


def test_new_grant_keeps_batch_and_total_allowance(sandbox):
    stage, attempt, _, _ = sandbox
    assert probe.AUTHORIZATION_RELATIVE == "frozen/sizing_diagnostic_authorization_003.json"
    assert probe.PROFILES == ("baseline", "geometry_scope", "capture_off", "omit_local_growth", "background_all_controls")
    assert probe.AUTHORIZATION_VALUES["maximum_attempts"] == 3
    assert probe.AUTHORIZATION_VALUES["attempt_root_relative"] == "runs/mcp_sizing_diagnostic/batch_001"
    assert probe.attempt_identity(attempt, stage)[2] == "sizing-b001-a001"


def test_full_background_verified_only_after_final_cleanup_and_rehash(sandbox, monkeypatch):
    stage, attempt, request, _ = sandbox
    request["profile"] = "background_all_controls"
    dump(attempt / "request.json", request)
    states = mock_success(monkeypatch, attempt, request, state="FullyDefined")
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed", result
    assert result["full_background_definitions_verified"] is True
    assert result["scope"] == "background_all_required_controls"
    assert result["local_growth_override_applied"] is False
    assert result["message_capture_complete"] and result["native_errors_present"] is False
    assert result["inputs_reverified_after_cleanup"] and result["sources_reverified_after_cleanup"] and result["owned_cleanup_verified"]
    assert all(not row["full_background_definitions_verified"] for row in states[:-1])
    assert all(row["required_background_definitions_match"] for row in result["phase_summaries"].values())
    assert len(result["receipts"]["readback"]["background_controls"]) == 4
    assert result["mesh_generated"] is False and result["mesh_motion_verified"] is False and result["flow_iterations"] == 0


@pytest.mark.parametrize("mutation", ["underdefined", "base_snapshot_underdefined", "scope_change", "coordinate_state", "sphere_radius", "face_size", "native_error", "incomplete_messages", "cleanup_unverified", "source_changed"])
def test_full_background_gate_rejects_single_failed_prerequisite(sandbox, monkeypatch, mutation):
    stage, attempt, request, _ = sandbox
    request["profile"] = "background_all_controls"
    dump(attempt / "request.json", request)
    mock_success(monkeypatch, attempt, request, state="FullyDefined", cleanup=mutation != "cleanup_unverified")
    launch = probe.subprocess.Popen
    def mutate(*args, **kwargs):
        process = launch(*args, **kwargs)
        path = attempt / probe.RECEIPTS["refresh"]
        value = json.loads(path.read_text(encoding="utf-8"))
        if mutation == "underdefined":
            value["background_controls"][2]["object_state"] = "UnderDefined"
        elif mutation == "base_snapshot_underdefined":
            value["control"]["object_state"] = "UnderDefined"
            value["snapshots"][-1]["object_state"] = "UnderDefined"
            value["control_fully_defined"] = False
        elif mutation == "scope_change":
            value["named_scopes"]["inlet"], value["named_scopes"]["outlet"] = value["named_scopes"]["outlet"], value["named_scopes"]["inlet"]
        elif mutation == "coordinate_state":
            value["background_controls"][3]["coordinate_system"]["object_state"] = "UnderDefined"
        elif mutation == "sphere_radius":
            value["background_controls"][3]["sphere_radius"]["value"] = .7
        elif mutation == "face_size":
            value["background_controls"][2]["element_size"]["value"] = .05
        elif mutation == "native_error":
            value.update(messages=[dict(severity="Error", text="native message")], message_characters=len("Errornative message"), native_errors_present=True)
        elif mutation == "incomplete_messages":
            value.update(message_capture_complete=False, message_capture_error="unavailable", native_errors_present=None)
        dump(path, value)
        return process
    monkeypatch.setattr(probe.subprocess, "Popen", mutate)
    if mutation == "source_changed":
        original_tree = probe.OwnedProcessTree
        class MutatingClose(original_tree):
            def close(self):
                (stage / "sizing_diagnostic.py").write_text("changed after observations", encoding="utf-8")
                return True
        monkeypatch.setattr(probe, "OwnedProcessTree", MutatingClose)
    result = probe.run(attempt, stage_root=stage)
    assert result["full_background_definitions_verified"] is False, result
    if mutation not in ("cleanup_unverified", "source_changed"):
        assert result["status"] == "completed", result
        assert result["diagnostic_completed"] and result["zero_mesh_verified"]


@pytest.mark.parametrize("profile", ["baseline", "geometry_scope", "capture_off", "omit_local_growth"])
def test_old_profiles_never_claim_full_background_acceptance(sandbox, monkeypatch, profile):
    stage, attempt, request, _ = sandbox
    request["profile"] = profile
    dump(attempt / "request.json", request)
    mock_success(monkeypatch, attempt, request, state="FullyDefined")
    result = probe.run(attempt, stage_root=stage)
    assert result["status"] == "completed" and result["control_fully_defined"] is True
    assert result["full_background_definitions_verified"] is False
    assert result["scope"] == "background_tet_and_wall_base0_only"


@pytest.mark.parametrize("mutation", ["method_scope", "face_scope", "face_growth", "sphere_center", "coordinate_link", "wrong_type", "overlap_scope", "out_of_body", "coordinate_unit"])
def test_background_definition_match_checks_each_scope_size_and_coordinate(sandbox, mutation):
    _, _, request, frozen = sandbox
    request["profile"] = "background_all_controls"
    value = native_receipt(request, "define", "FullyDefined")
    settings = probe.domain_settings(frozen["contract"], Path("."))[0]
    assert probe.background_definitions_match(value, settings)
    rows = value["background_controls"]
    if mutation == "method_scope":
        rows[0]["actual_geometry_ids"] = [2]
    elif mutation == "face_scope":
        rows[2]["actual_geometry_ids"] = [9]
    elif mutation == "face_growth":
        rows[2]["local_growth_raw"]["value"] = 1.3
    elif mutation == "sphere_center":
        rows[3]["coordinate_system"]["origins"]["OriginX"]["value"] += .01
    elif mutation == "coordinate_link":
        rows[3]["sphere_center_object_id"] = 100
    elif mutation == "wrong_type":
        rows[3]["type"] = "ElementSize"
    elif mutation == "overlap_scope":
        value["named_scopes"]["inlet"] = [7]
    elif mutation == "out_of_body":
        value["named_scopes"]["inlet"] = [999]
    else:
        rows[3]["coordinate_system"]["origins"]["OriginX"]["unit"] = "unknown"
    assert probe.background_definitions_match(value, settings) is False

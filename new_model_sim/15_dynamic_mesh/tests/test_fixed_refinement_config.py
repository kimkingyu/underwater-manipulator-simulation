"""Offline configuration-only tests. No native process, filesystem outputs or APIs."""
from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
from fixed_refinement_config import (appended_spheres, trial_settings, validate_overlay,
    configuration_comparison, BASELINE_MESHES)
from mesh_prepare import domain_settings
from runtime import RuntimeGateError


@pytest.fixture
def contract():
    return json.loads((STAGE / "contract.json").read_text(encoding="utf-8"))


def test_only_one_sphere_appended_each_domain(contract, tmp_path):
    before = deepcopy(contract)
    baseline = domain_settings(contract, tmp_path)
    trial = trial_settings(contract, tmp_path)
    assert contract == before
    assert [len(x["spheres"]) for x in baseline] == [1, 0]
    assert [len(x["spheres"]) for x in trial] == [2, 1]
    for old, new in zip(baseline, trial):
        extra = new["spheres"].pop()
        assert extra == appended_spheres()[old["domain"]]
        assert new == old
    assert not list(tmp_path.iterdir())


def test_exact_authorized_metre_values(contract, tmp_path):
    background, arm = trial_settings(contract, tmp_path)
    assert background["spheres"][-1] == {
        "centre_m": [0.282807, 0.453505, 0.426884], "radius_m": 0.05, "size_m": 0.01}
    assert arm["spheres"][-1] == {
        "centre_m": [-0.0153238, -0.170113, -0.349268], "radius_m": 0.03, "size_m": 0.001}
    assert background["growth_rate"] == arm["growth_rate"] == 1.3
    assert background["element_method"] == arm["element_method"] == "AllTriAllTet"
    assert background["motion_authorized"] is arm["motion_authorized"] is False


@pytest.mark.parametrize("field,value", [
    ("growth_rate", 1.2), ("minimum_size_m", 0.0001),
    ("maximum_size_m", 0.1), ("global_size_m", 0.1),
    ("element_method", "Polyhedral"), ("motion_authorized", True),
])
def test_other_baseline_changes_rejected(contract, tmp_path, field, value):
    trial = trial_settings(contract, tmp_path)
    trial[0][field] = value
    with pytest.raises(RuntimeGateError):
        validate_overlay(contract, tmp_path, trial)


@pytest.mark.parametrize("case", ["missing_extra", "duplicate_extra", "replace_existing", "wrong_units", "wrong_centre", "reverse_domains", "extra_field", "bool_size"])
def test_scope_or_overlay_mutation_rejected(contract, tmp_path, case):
    trial = trial_settings(contract, tmp_path)
    if case == "missing_extra": trial[0]["spheres"].pop()
    elif case == "duplicate_extra": trial[0]["spheres"].append(deepcopy(trial[0]["spheres"][-1]))
    elif case == "replace_existing": trial[0]["spheres"].pop(0)
    elif case == "wrong_units": trial[0]["spheres"][-1]["radius_m"] = 50.0
    elif case == "wrong_centre": trial[1]["spheres"][-1]["centre_m"][0] += 0.001
    elif case == "reverse_domains": trial.reverse()
    elif case == "extra_field": trial[0]["CaptureCurvature"] = True
    elif case == "bool_size": trial[1]["spheres"][-1]["size_m"] = True
    with pytest.raises(RuntimeGateError):
        validate_overlay(contract, tmp_path, trial)


def test_independent_return_values_do_not_change_constants(contract, tmp_path):
    first = trial_settings(contract, tmp_path)
    first[0]["spheres"][-1]["radius_m"] = 2.0
    assert trial_settings(contract, tmp_path)[0]["spheres"][-1]["radius_m"] == 0.05
    extra = appended_spheres()
    extra["arm"]["centre_m"][0] = 2.0
    assert appended_spheres()["arm"]["centre_m"][0] == -0.0153238


def test_audit_is_not_native_readback_or_acceptance(contract, tmp_path):
    audit = configuration_comparison(contract, tmp_path)
    assert audit["native_readback_verified"] is False
    assert audit["construction_seconds_including_cleanup"] == 7200
    assert audit["per_domain_seconds"] == 3600
    assert audit["QA_seconds_including_cleanup"] == 300
    assert audit["automatic_retries"] == audit["initializations"] == audit["flow_iterations"] == audit["mesh_motion_steps"] == 0
    assert BASELINE_MESHES["background"]["relative"] == "runs/mesh/attempt_004/background.msh"
    assert BASELINE_MESHES["arm"]["sha256"] == "1dffa98e02b80a9813c37d43e746ced5cf501cd0032580a04a69726b419bd8d5"


def test_fixed_journal_generation_is_offline_and_records_overlay(contract, tmp_path):
    from unittest.mock import patch
    import mesh_prepare
    background = tmp_path / "background.scdoc"
    arm = tmp_path / "arm.scdoc"
    background.write_bytes(b"explicit offline background CAD fixture")
    arm.write_bytes(b"explicit offline arm CAD fixture")
    attempt = tmp_path / "fixed_trial"
    contract_before = (STAGE / "contract.json").read_bytes()
    with patch("subprocess.Popen") as launch:
        plan = mesh_prepare.generate(attempt, background, arm, stage_root=tmp_path, fixed_refinement=True)
        launch.assert_not_called()
    assert plan["status"] == "not_run"
    assert plan["valid"] is False
    assert [len(x["spheres"]) for x in plan["domains"]] == [2, 1]
    comparison = json.loads((attempt / "fixed_refinement_configuration.json").read_text(encoding="utf-8"))
    assert comparison["native_readback_verified"] is False
    assert comparison["domains"][0]["only_change"] == "one_appended_sphere"
    assert comparison["domains"][1]["trial"]["spheres"][-1]["size_m"] == 0.001
    journal = (attempt / "mesh_stage15.wbjn").read_text(encoding="utf-8")
    compile(journal, "offline_fixed_trial_journal", "exec")
    assert "__CONFIG_JSON__" not in journal
    assert (STAGE / "contract.json").read_bytes() == contract_before
    assert background.read_bytes() == b"explicit offline background CAD fixture"


@pytest.mark.parametrize("options", [
    {"fixed_refinement": "yes"}, {"fixed_refinement": 1},
    {"fixed_refinement": True, "growth_rate": 1.2},
    {"fixed_refinement": True, "background_size": 0.1},
    {"fixed_refinement": True, "cell_type": "polyhedral"},
])
def test_fixed_journal_rejects_overrides_before_output(tmp_path, options):
    import mesh_prepare
    background = tmp_path / "b.scdoc"
    arm = tmp_path / "a.scdoc"
    background.write_bytes(b"offline fixture")
    arm.write_bytes(b"offline fixture")
    attempt = tmp_path / "rejected"
    with pytest.raises(RuntimeGateError):
        mesh_prepare.generate(attempt, background, arm, stage_root=tmp_path, **options)
    assert not attempt.exists()


def native_sphere_fixture(config):
    # Explicit synthetic readback for offline validation, never native evidence.
    return {"sphere_readback": [dict(index=i, name="stage15_overlap_%d" % i,
        type="SphereOfInfluence", state="FullyDefined", geometry_ids=[7],
        coordinate_system_name="stage15_overlap_%d" % i,
        assigned_coordinate_system_name="stage15_overlap_%d" % i,
        coordinate_system_state="FullyDefined",
        centre=[str(x * 1000) + " [mm]" for x in sphere["centre_m"]],
        radius=str(sphere["radius_m"] * 1000) + " [mm]",
        element_size=str(sphere["size_m"]) + " [m]")
        for i, sphere in enumerate(config["spheres"])]}


def test_sphere_native_readback_handles_units_without_guessing(contract, tmp_path):
    from fixed_refinement_config import validate_native_spheres
    for config in trial_settings(contract, tmp_path):
        assert validate_native_spheres(config, native_sphere_fixture(config))


@pytest.mark.parametrize("field,value", [
    ("type", "ElementSize"), ("state", "UnderDefined"),
    ("coordinate_system_state", "UnderDefined"),
    ("assigned_coordinate_system_name", "unrelated"),
    ("geometry_ids", []), ("geometry_ids", [True]),
    ("radius", "50 [m]"), ("radius", "0.05"),
    ("element_size", "nan [m]"), ("centre", ["0 [m]"]),
])
def test_bad_sphere_native_readback_rejected(contract, tmp_path, field, value):
    from fixed_refinement_config import validate_native_spheres
    config = trial_settings(contract, tmp_path)[0]
    stats = native_sphere_fixture(config)
    stats["sphere_readback"][-1][field] = value
    with pytest.raises(RuntimeGateError):
        validate_native_spheres(config, stats)


def test_missing_sphere_native_readback_is_not_success(contract, tmp_path):
    from fixed_refinement_config import validate_native_spheres
    config = trial_settings(contract, tmp_path)[0]
    with pytest.raises(RuntimeGateError):
        validate_native_spheres(config, {})
    stats = native_sphere_fixture(config)
    stats["sphere_readback"].pop(0)
    with pytest.raises(RuntimeGateError):
        validate_native_spheres(config, stats)

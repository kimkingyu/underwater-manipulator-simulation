"""Prepare the separately authorized arm-only completion or independent reopen.

No external application is started. The background artifact is immutable and
must come from the actual passed background branch of build004.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cad_prepare as cad
from prepare_face_probe import verify_module_startup
from preflight import STAGE, WORKSPACE, load_contract, sha256, verify_inputs

AUTH = STAGE / "frozen/arm_completion_authorization_001.json"
BASE_REPORT = STAGE / "runs/cad_build/attempt_004/build_report.json"
PROTOCOL = STAGE / "frozen/measurement_protocol_v2.json"
CALIBRATION = STAGE / "runs/cad_calibration/attempt_002/calibration_report.json"
LOCAL_PROBE = STAGE / "runs/cad_face_probe/attempt_002/face_probe_report.json"


def validate_grant(grant):
    limits = {"maximum_arm_construction_attempts": 1, "maximum_arm_construction_seconds": 1800,
              "maximum_independent_reopen_attempts": 1, "maximum_independent_reopen_seconds": 1800}
    if grant.get("scope") != "arm_domain_completion_then_independent_reopen":
        raise ValueError("A new arm-completion authorization is required")
    if any(isinstance(grant.get(k), bool) or grant.get(k) != v for k, v in limits.items()):
        raise ValueError("Unexpected arm-completion execution limits")
    if any(grant.get(k) is not False for k in ("background_rebuild_allowed", "original_cad_changes_allowed",
                                              "acceptance_gate_relaxation_allowed", "automatic_retry_allowed")):
        raise ValueError("The grant must preserve geometry, background and acceptance gates")
    if grant.get("independent_reopen_requires_construction_pass") is not True:
        raise ValueError("Independent reopen must depend on a passed construction")
    build_id = grant.get("arm_build_attempt_id", "attempt_001")
    if build_id not in ("attempt_001", "attempt_002"):
        raise ValueError("No authorization for another arm-completion slot")
    if build_id == "attempt_002" and grant.get("naming_sampler_repair_rerun_explicitly_approved") is not True:
        raise ValueError("Naming-sampler corrective rerun requires a new explicit grant")


def record(path):
    path = cad.stage_path(path, must_exist=True)
    if not path.is_file():
        raise ValueError("A regular evidence/artifact file is required")
    return {"path": str(path), "sha256": sha256(path)}


def validate_background(base, source_sha, grant):
    if base.get("source_unchanged") is not True or base.get("source_sha256_after") != source_sha:
        raise ValueError("Background source geometry does not match")
    domain = base["domains"]["background"]
    if domain.get("state") != "passed":
        raise ValueError("Background construction branch did not pass")
    artifact = record(domain["scdoc"])
    if artifact["sha256"] != domain["scdoc_sha256"] or artifact["sha256"] != grant.get("background_sha256"):
        raise ValueError("Immutable background artifact hash mismatch")
    if Path(artifact["path"]) != (STAGE / grant["background_artifact"]).resolve():
        raise ValueError("Background artifact is outside the expressly authorized reference")
    return artifact


def validate_local_evidence(probe, base, source_sha):
    if (probe.get("mode") != "face_probe" or probe.get("source_unchanged") is not True
            or probe.get("input_integrity", {}).get("state") != "passed"
            or probe.get("source_sha256_after") != source_sha):
        raise ValueError("Actual local sampler evidence is missing or source-mismatched")
    expected = base["domains"]["arm"]["boolean_complement_diagnostics"]["geometry_acceptance"]["shapes"][0]
    actual = probe["material"]
    if expected["faces"] != actual["faces"]:
        raise ValueError("Probe must be compared to the MATERIAL object, not exterior water")
    scalar_errors = [abs(actual[k] - expected[k]) for k in ("volume_m3", "surface_area_m2")]
    scalar_errors += [abs(actual["bbox_m"][i][j] - expected["bbox_m"][i][j]) for i in (0, 1) for j in range(3)]
    if max(scalar_errors) > 1e-12:
        raise ValueError("Material fingerprint changed between reference and local probe")
    targets = probe.get("targeted_faces", [])
    rows = [r for r in probe.get("face_observations", []) if r.get("face_index_this_run") in targets]
    if not targets or len(rows) != len(targets) or len(set(targets)) != len(targets):
        raise ValueError("Incomplete local sampling target evidence")
    for row in rows:
        sampling = row.get("sampling", {})
        if (row.get("state") != "passed_for_this_face_sampling_only" or sampling.get("state") != "passed"
                or sampling.get("final_distinct_count", 0) < 3 or sampling.get("fallback_used") is not True):
            raise ValueError("A targeted local face did not establish fallback sampling")
    return {"raw_probe_state_retained": probe["state"], "source_object": "arm_material",
            "sampled_face_indices": targets, "material_fingerprint_matched": True,
            "local_observations_only_not_full_geometry_acceptance": True}


def generate(kind, authorization_path=AUTH):
    if kind not in ("build", "verify"):
        raise ValueError("Only build and verify are supported")
    authorization_path = cad.stage_path(authorization_path, must_exist=True)
    grant = json.loads(authorization_path.read_text(encoding="utf-8"))
    validate_grant(grant)
    contract = load_contract()
    protected = json.loads((STAGE / "frozen/input_manifest.json").read_text(encoding="utf-8"))
    if verify_inputs(protected)["status"] != "passed":
        raise ValueError("Protected inputs changed")
    directory = "cad_arm_completion" if kind == "build" else "cad_arm_completion_verify"
    build_id = grant.get("arm_build_attempt_id", "attempt_001")
    target = STAGE / "runs" / directory / (build_id if kind == "build" else "attempt_001")
    if target.exists():
        raise FileExistsError("This single authorized slot is already occupied: " + str(target))
    source = WORKSPACE / contract["source"]["step_relative_to_workspace"]
    source_sha = sha256(source)
    base = json.loads(BASE_REPORT.read_text(encoding="utf-8"))
    background = validate_background(base, source_sha, grant)
    probe = json.loads(LOCAL_PROBE.read_text(encoding="utf-8"))
    local_evidence = validate_local_evidence(probe, base, source_sha)
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    calibration = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    if (protocol.get("schema_version") != 2 or calibration.get("state") != "passed"
            or calibration.get("small_diagnostic_passed") is not True
            or calibration.get("source_sha256_after") != source_sha):
        raise ValueError("Frozen measurement/control evidence is not eligible")
    config = {
        "schema_version": 1, "mode": kind, "stage_root": str(STAGE), "attempt_dir": str(target),
        "source_path": str(source), "source_sha256": source_sha, "step_length_unit": cad.step_units(source),
        "frozen_sources": cad.frozen_sources(), "recipe_sha256": cad.APPROVED_SOURCE_HASHES[cad.RECIPE_PATH.name],
        "pose_rad": contract["source"]["pose_rad"],
        "joint_centres_m": contract["frame"]["joint_centres_CAD0_m"], "joint_axes": contract["frame"]["joint_axes_CAD0"],
        "j1_samples_rad": cad.sample_angles(9, *contract["motion"]["j1_range_rad"]),
        "padding_m": contract["domains"]["initial_padding_m"],
        "tank_lo_m": contract["domains"]["tank_lo_m"], "tank_hi_m": contract["domains"]["tank_hi_m"],
        "distal_shift_m": contract["source"]["distal_shift_before_pose_m"],
        "volume_diagnostics": {"surface_deviations_m": [1e-5, 2.5e-6, 6.25e-7], "maximum_facets": 250000,
                               "tessellation_is_diagnostic_only": True, "absolute_tolerance_m3": 1e-10, "relative_tolerance": 1e-8},
        "measurement_protocol": record(PROTOCOL), "calibration_reference": record(CALIBRATION),
        "additional_authorization": record(authorization_path),
        "background_reference_report": record(BASE_REPORT), "background_artifact_reference": background,
        "local_sampler_reference": record(LOCAL_PROBE), "local_sampler_evidence_scope": local_evidence,
        "script_path": str(target / ("sc_arm_completion.py" if kind == "build" else "sc_independent_verify.py")),
        "output_json": str(target / ("build_report.json" if kind == "build" else "verify_report.json")),
        "maximum_runtime_seconds": 1800,
    }
    if kind == "build":
        config["output_scdoc"] = {"background": background["path"], "arm": str(target / "arm_fluid.scdoc")}
        template = "sc_arm_completion.py.in"
    else:
        previous_path = STAGE / "runs/cad_arm_completion" / build_id / "build_report.json"
        previous = json.loads(previous_path.read_text(encoding="utf-8"))
        if (previous.get("mode") != "build" or previous.get("state") != "passed"
                or previous.get("source_unchanged") is not True or previous.get("source_sha256_after") != source_sha
                or previous.get("cad_construction", {}).get("constructed_domains") != ["arm"]
                or previous.get("cad_construction", {}).get("reused_domains") != ["background"]):
            raise ValueError("Independent reopen requires the actual passed arm-only construction")
        if sorted(previous.get("domains", {})) != ["arm", "background"]:
            raise ValueError("Two construction artifacts are required")
        for key in ("measurement_protocol", "calibration_reference", "additional_authorization"):
            if previous.get("config", {}).get(key) != config[key]:
                raise ValueError("Verify must inherit the exact build bindings: " + key)
        artifacts = []
        for name in ("background", "arm"):
            domain = previous["domains"][name]
            item = record(domain["scdoc"])
            if domain.get("state") != "passed" or item["sha256"] != domain["scdoc_sha256"]:
                raise ValueError("Construction artifact missing or changed: " + name)
            artifacts.append(item)
        config.update(build_report=str(previous_path), build_report_sha256=sha256(previous_path))
        config["frozen_sources"] += [record(previous_path)] + artifacts
        template = "sc_verify.py.in"
    source_paths = [Path(__file__), STAGE / "cad_prepare.py", STAGE / "prepare_face_probe.py"]
    source_paths += [STAGE / "cad" / name for name in ("sc_common.py.in", "sc_volume.py.in", "sc_domains.py.in", template)]
    config["frozen_sources"] += [record(path) for path in source_paths]
    config["frozen_sources"] += [config[key] for key in ("measurement_protocol", "calibration_reference", "additional_authorization",
                                                       "background_reference_report", "background_artifact_reference", "local_sampler_reference")]
    code = cad.render_template(template, config)
    startup = verify_module_startup(code)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.mkdir(exist_ok=False)
    Path(config["script_path"]).write_text(code, encoding="utf-8", newline="\n")
    result = {"state": "not_run", "external_software_started": False, "startup_check": startup,
              "script_path": config["script_path"], "output_json": config["output_json"],
              "script_sha256": sha256(config["script_path"]), "config": config}
    (target / "generation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("build", "verify"))
    parser.add_argument("--authorization", default=str(AUTH))
    args = parser.parse_args()
    result = generate(args.kind, args.authorization)
    print(json.dumps({key: result[key] for key in ("state", "external_software_started", "startup_check", "script_path", "output_json")}, ensure_ascii=False))

"""Prepare one local face-sampling probe after a separate explicit user grant.

This program only creates a script. It never starts SpaceClaim or a solver.
The previously exhausted build/small-control authorization is not accepted here.
"""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path

import cad_prepare
from preflight import STAGE, WORKSPACE, load_contract, sha256, verify_inputs


def validate_authorization(data):
    exact = {"scope": "single_face_sampling_probe", "maximum_attempts": 1,
             "maximum_runtime_seconds": 300, "maximum_faces_scanned": 32,
             "maximum_failed_faces_diagnosed": 3,
             "complete_fluid_domain_builds_allowed": 0}
    if any(data.get(key) != value or isinstance(data.get(key), bool) for key, value in exact.items() if isinstance(value, int)):
        raise ValueError("Face-probe authorization does not match the finite requested limits")
    if data.get("scope") != exact["scope"]:
        raise ValueError("An old or different authorization cannot run this probe")
    if data.get("flow_solves_allowed") is not False or data.get("original_cad_changes_allowed") is not False:
        raise ValueError("Local diagnosis may not change the source geometry or run flow")
    attempt_id = data.get("attempt_id", "attempt_001")
    if attempt_id not in ("attempt_001", "attempt_002"):
        raise ValueError("Unsupported local-probe slot")
    if attempt_id == "attempt_002" and data.get("startup_repair_rerun_explicitly_approved") is not True:
        raise ValueError("The corrective rerun needs a new explicit user grant")
    exact["attempt_id"] = attempt_id
    return exact


def verify_module_startup(source):
    """Execute declarations/constants only; never enter the CAD runtime function."""
    tree = ast.parse(source)
    runtime_calls = [node for node in tree.body
                     if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                     and isinstance(node.value.func, ast.Name) and node.value.func.id == "runtime_main"]
    if len(runtime_calls) != 1:
        raise ValueError("Expected exactly one removable CAD runtime entry")
    body = [node for node in tree.body if node not in runtime_calls]
    permitted = (ast.Import, ast.ImportFrom, ast.Assign, ast.FunctionDef, ast.ClassDef)
    if any(not isinstance(node, permitted) for node in body):
        raise ValueError("Unexpected top-level executable statement in generated CAD source")
    namespace = {"__name__": "_stage15_offline_declarations_only"}
    try:
        exec(compile(ast.Module(body=body, type_ignores=[]), "generated_cad_startup_only", "exec"), namespace)
    except KeyError as exc:
        raise ValueError("Generated CAD startup is missing configuration: " + str(exc)) from exc
    if not callable(namespace.get("runtime_main")) or not callable(namespace.get("mode_main")):
        raise ValueError("Generated CAD entry functions are missing")
    return {"status": "passed", "scope": "Python module constants/declarations only",
            "cad_runtime_called": False, "sdk_compatibility_proven": False}


def render(config):
    template_root = STAGE / "cad"
    common = (template_root / "sc_common.py.in").read_text(encoding="utf-8")
    marker = "'__STAGE15_CONFIG_JSON__'"
    if common.count(marker) != 1:
        raise ValueError("Ambiguous configuration marker")
    mode_source = (template_root / "sc_face_probe.py.in").read_text(encoding="utf-8")
    mode_tree = ast.parse(mode_source)
    forbidden = {"build_domain", "regenerate_domain_reference", "run_volume_calibration", "DocumentSave"}
    if any(isinstance(node, ast.Name) and node.id in forbidden for node in ast.walk(mode_tree)):
        raise ValueError("A local sampling probe must not construct/export full fluid domains")
    source = common.replace(marker, repr(json.dumps(config, ensure_ascii=True, sort_keys=True)))
    source += "\n" + cad_prepare.approved_recipe_code()
    source += "\n" + (template_root / "sc_domains.py.in").read_text(encoding="utf-8")
    source += "\n" + mode_source
    ast.parse(source, filename=config["script_path"])
    verify_module_startup(source)
    return source


def reference_objects(build_report):
    """Keep material and exterior-water comparisons distinct; face IDs are local."""
    arm = build_report["domains"]["arm"]
    sections = {
        "arm_material": (arm["boolean_complement_diagnostics"]["geometry_acceptance"],
                         "expected_vs_boolean_reconstructed_material"),
        "arm_exterior_water": (arm["imprint_geometry_check"],
                               "exterior_water_before_vs_after_imprint"),
    }
    result = {}
    for kind, (section, label) in sections.items():
        if section.get("label") != label:
            raise ValueError("Reference geometric object label mismatch: " + kind)
        direction = section["surface_fallback"]["a_to_b"]
        shape = section["shapes"][0]
        count = direction["source_face_count"]
        if isinstance(count, bool) or not isinstance(count, int) or count < 1 or shape["faces"] != count:
            raise ValueError("Reference face census is inconsistent: " + kind)
        incomplete = [row for row in direction["face_results"] if row.get("sample_count", 0) < 3]
        if not incomplete:
            raise ValueError("Reference has no sampling-blocked face: " + kind)
        result[kind] = {"label": label, "face_count": count,
                        "first_sampling_blocked_face_index": incomplete[0]["face_index"],
                        "shape_fingerprint": shape,
                        "face_index_is_not_persistent_identity": True}
    return result


def generate(authorization_path):
    auth_path = cad_prepare.stage_path(authorization_path, must_exist=True)
    grant = json.loads(auth_path.read_text(encoding="utf-8"))
    limits = validate_authorization(grant)
    contract = load_contract()
    manifest = json.loads((STAGE / "frozen/input_manifest.json").read_text(encoding="utf-8"))
    if verify_inputs(manifest)["status"] != "passed":
        raise ValueError("Protected source input changed")
    target = STAGE / "runs/cad_face_probe" / limits["attempt_id"]
    if target.exists():
        raise FileExistsError("The single face-probe slot already exists")
    source_step = WORKSPACE / contract["source"]["step_relative_to_workspace"]
    reference = STAGE / "runs/cad_build/attempt_004/build_report.json"
    if not reference.is_file():
        raise ValueError("Actual blocked build004 evidence is required")
    objects = reference_objects(json.loads(reference.read_text(encoding="utf-8")))
    templates = [STAGE / "cad" / name for name in ("sc_common.py.in", "sc_domains.py.in", "sc_face_probe.py.in")]
    frozen = cad_prepare.frozen_sources() + [{"path": str(path), "sha256": sha256(path)}
                                            for path in [auth_path, reference, Path(__file__)] + templates]
    config = {
        "schema_version": 1, "mode": "face_probe", "stage_root": str(STAGE), "attempt_dir": str(target),
        "source_path": str(source_step), "source_sha256": sha256(source_step),
        "frozen_sources": frozen,
        "recipe_sha256": cad_prepare.APPROVED_SOURCE_HASHES[cad_prepare.RECIPE_PATH.name],
        "pose_rad": contract["source"]["pose_rad"],
        "distal_shift_m": contract["source"]["distal_shift_before_pose_m"],
        "joint_centres_m": contract["frame"]["joint_centres_CAD0_m"],
        "joint_axes": contract["frame"]["joint_axes_CAD0"],
        "script_path": str(target / "sc_face_sampling_probe.py"),
        "output_json": str(target / "face_probe_report.json"),
        "authorization": {"path": str(auth_path), "sha256": sha256(auth_path)},
        "face_probe_limits": limits,
        "reference_report": {"path": str(reference), "sha256": sha256(reference)},
        "reference_object_kind": "arm_material",
        "reference_objects": objects,
        "reference_material_face_count": objects["arm_material"]["face_count"],
        "reference_first_blocked_face_index": objects["arm_material"]["first_sampling_blocked_face_index"],
        "reference_index_is_not_persistent_identity": True,
    }
    code = render(config)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.mkdir(exist_ok=False)
    Path(config["script_path"]).write_text(code, encoding="utf-8", newline="\n")
    result = {"state": "not_run", "external_software_started": False,
              "script_path": config["script_path"], "output_json": config["output_json"],
              "script_sha256": sha256(config["script_path"]), "config": config}
    (target / "generation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", required=True)
    result = generate(parser.parse_args().authorization)
    print(json.dumps({key: result[key] for key in ("state", "external_software_started", "script_path", "output_json")}, ensure_ascii=False))

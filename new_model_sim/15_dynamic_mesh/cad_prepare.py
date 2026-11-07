"""Generate isolated V242 scripts. This module never launches CAD or a solver."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import textwrap

STAGE_ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = STAGE_ROOT.parent.parent
TEMPLATE_ROOT = STAGE_ROOT / "cad"
CONTRACT_PATH = STAGE_ROOT / "contract.json"
RECIPE_PATH = WORKSPACE_ROOT / "new_model_sim/14_hydro_validation/cfd/qe/sc_build_fluid_domain_qe.py"
DOMAIN_REFERENCE_PATH = WORKSPACE_ROOT / "new_model_sim/05_overset_cad/sc_build_overset_domains.py"
POSE_REFERENCE_PATH = WORKSPACE_ROOT / "new_model_sim/14_hydro_validation/cfd/qe/sc_probe_pose_qe.py"
APPROVED_SOURCE_HASHES = {
    "sc_build_fluid_domain_qe.py": "966d0b95041b4a2e75281b94505e257ab8bd97ea009808e4582ca382ee7f3ab3",
    "sc_build_overset_domains.py": "4649850dba0662067cc7548916f03ceb5b7e3de6af6cffb7e23b5242f0e05109",
    "sc_probe_pose_qe.py": "0bf721a664681bbf5bd80f4d95aa021b49c77dd543be90d4d0ca9b1650c87c5d",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stage_path(value: str | Path, *, must_exist: bool = False) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = STAGE_ROOT / path
    resolved = path.resolve(strict=must_exist)
    root = STAGE_ROOT.resolve()
    if resolved == root or root not in resolved.parents:
        raise ValueError("Path must be strictly below stage15")
    # resolve() follows Windows junctions as well as ordinary symlinks.
    return resolved


def load_contract() -> dict:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    expected = {
        "frame.length_unit": (contract["frame"]["length_unit"], "m"),
        "frame.angle_unit": (contract["frame"]["angle_unit"], "rad"),
        "frame.joint_centres_CAD0_m": (contract["frame"]["joint_centres_CAD0_m"], [[-0.000004, -0.0366, -0.460702], [0.016686, -0.0666, -0.511202], [0.014796, -0.1226, -0.313202]]),
        "source.pose": (contract["source"]["pose"], "qe"),
        "source.seal_recipe_relative_to_workspace": (contract["source"]["seal_recipe_relative_to_workspace"], "new_model_sim/14_hydro_validation/cfd/qe/sc_build_fluid_domain_qe.py"),
        "source.new_physical_geometry_changes_allowed": (contract["source"]["new_physical_geometry_changes_allowed"], False),
        "motion.initial_q_rad": (contract["motion"]["initial_q_rad"], [0.225, -0.25, -0.2]),
        "motion.active_joint_indices_1based": (contract["motion"]["active_joint_indices_1based"], [1]),
        "domains.shared_topology_between_motion_groups_allowed": (contract["domains"]["shared_topology_between_motion_groups_allowed"], False),
        "domains.cell_zones": (contract["domains"]["cell_zones"], ["fluid_background", "fluid_arm"]),
        "domains.overset_boundary": (contract["domains"]["overset_boundary"], "overset_arm"),
        "domains.fixed_walls": (contract["domains"]["fixed_walls"], ["wall_auv", "wall_base0"]),
        "domains.moving_walls": (contract["domains"]["moving_walls"], ["wall_l1", "wall_l2", "wall_link3", "wall_pin4", "wall_jaw1", "wall_jaw2"]),
        "source.pose_rad": (contract["source"]["pose_rad"], [0.225, -0.25, -0.2]),
        "motion.j1_range_rad": (contract["motion"]["j1_range_rad"], [0.225, 0.275]),
        "frame.joint_axes_CAD0": (contract["frame"]["joint_axes_CAD0"], [[0, 0, -1], [-1, 0, 0], [1, 0, 0]]),
        "domains.fixed_parts": (contract["domains"]["fixed_parts"], ["hull", "base0"]),
        "domains.moving_parts": (contract["domains"]["moving_parts"], ["l1", "l2", "link3", "pin4", "jaw1", "jaw2"]),
        "source.keep_l2_cylinder_gap_fill": (contract["source"]["keep_l2_cylinder_gap_fill"], False),
        "source.distal_shift_before_pose_m": (contract["source"]["distal_shift_before_pose_m"], [0.0, 0.00001, 0.0]),
    }
    for name, (actual, required) in expected.items():
        if actual != required:
            raise ValueError("Contract requires a newly reviewed generator: " + name)
    return contract


def sample_angles(count: int, low: float, high: float) -> list[float]:
    if isinstance(count, bool) or not isinstance(count, int) or count < 9 or count > 65 or count % 2 == 0:
        raise ValueError("J1 sample count must be an odd integer in [9,65]")
    if not all(math.isfinite(value) for value in (low, high)) or [low, high] != [0.225, 0.275]:
        raise ValueError("Only the approved J1 range [0.225,0.275] is supported")
    # Odd uniform samples include both endpoints and q1=0.25 (quarter-cycle).
    return [low + (high - low) * i / (count - 1) for i in range(count - 1)] + [high]


def step_units(path: Path) -> dict:
    # STEP contains legacy non-UTF8 names. Unit tokens are ASCII; latin-1 is
    # a lossless byte mapping and does not rewrite or decode entity names.
    text = path.read_bytes().decode("latin-1")
    units = re.findall(r"#\d+\s*=\s*\([^;]*?LENGTH_UNIT\s*\([^;]*?;", text, re.S)
    if not units or any(not re.search(r"SI_UNIT\s*\(\s*\.MILLI\.\s*,\s*\.METRE\.\s*\)", row) for row in units):
        raise ValueError("STEP length-unit declaration is missing or differs from millimetres")
    return {"unit": "mm", "declaration_count": len(units), "kernel_output_unit": "m", "evidence": units}


def frozen_sources() -> list[dict]:
    paths = [CONTRACT_PATH, RECIPE_PATH, DOMAIN_REFERENCE_PATH, POSE_REFERENCE_PATH]
    rows = [{"path": str(path), "sha256": sha256(path)} for path in paths]
    for path, row in zip(paths, rows):
        expected = APPROVED_SOURCE_HASHES.get(path.name)
        if expected is not None and row["sha256"] != expected:
            raise ValueError("Approved reference hash differs: " + str(path))
    return rows


def unique_slice(source: str, start: str, end: str) -> str:
    if source.count(start) != 1 or source.count(end) != 1:
        raise ValueError("Recipe boundaries must match uniquely")
    left, right = source.index(start), source.index(end)
    if right <= left:
        raise ValueError("Recipe boundaries are reversed")
    return textwrap.dedent(source[left:right])


def approved_recipe_code() -> str:
    """Extract reviewed declarations/statement ranges; never import/exec the old file."""
    if sha256(RECIPE_PATH) != APPROVED_SOURCE_HASHES[RECIPE_PATH.name]:
        raise ValueError("Approved sealing recipe hash mismatch")
    source = RECIPE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = ("mm", "plateau_depth", "slab_box", "jaw_fillers", "find_j3_tangency")
    constants = ("W_J3", "EMBED", "R_J3", "DA", "DEPTHS", "H_MAX", "CLUSTERS", "JAW_MEMBERS", "DISTAL")
    sections = []
    for name in constants + functions:
        found = [node for node in tree.body if
                 (isinstance(node, ast.FunctionDef) and node.name == name) or
                 (isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == name for target in node.targets))]
        if len(found) != 1:
            raise ValueError("Recipe declaration not unique: " + name)
        sections.append(ast.get_source_segment(source, found[0]))
    seal = unique_slice(source, "    # (b) jaw-group fillers in the unshifted frame (same as j3_fluid_v2).",
                        "    # (c) l2 / base0-l1 cylinder gap, unshifted frame (none of the three parts is shifted).")
    extension = unique_slice(source, "    # Extended parts, then the v3 shift of the distal group.", "    olap = {}")
    body = "\n".join((seal, extension, "return ext\n"))
    sections.append("SHIFT_M = CFG['distal_shift_m']")
    sections.append("def approved_extended_parts(tools):\n" + textwrap.indent(body, "    "))
    result = "\n\n".join(sections) + "\n"
    ast.parse(result)
    banned = ("CylinderBody", "cyl_fill", "DocumentSave", "OUT_SCDOC", "V3_JSON", "V4_JSON", "policy")
    if any(token in result for token in banned):
        raise ValueError("Unexpected legacy output/cylinder-gap code entered recipe")
    return result


def render_template(name: str, config: dict) -> str:
    common = (TEMPLATE_ROOT / "sc_common.py.in").read_text(encoding="utf-8")
    marker = "'__STAGE15_CONFIG_JSON__'"
    if common.count(marker) != 1:
        raise ValueError("Configuration marker must appear exactly once")
    common = common.replace(marker, repr(json.dumps(config, ensure_ascii=True, sort_keys=True)))
    mode = (TEMPLATE_ROOT / name).read_text(encoding="utf-8")
    extra = ""
    if config["mode"] in ("build", "verify"):
        extra = approved_recipe_code() + "\n" + (TEMPLATE_ROOT / "sc_volume.py.in").read_text(encoding="utf-8")
        extra += "\n" + (TEMPLATE_ROOT / "sc_domains.py.in").read_text(encoding="utf-8")
    text = common + "\n" + extra + "\n" + mode
    ast.parse(text, filename=config["script_path"])
    if "__STAGE15_CONFIG_JSON__" in text:
        raise ValueError("Unexpanded configuration marker")
    return text


def bind_measurement_sources(config: dict, protocol_path, calibration_path, authorization_path) -> None:
    """Bind the actual finite user grant and passed control evidence, not a model assertion."""
    supplied = (protocol_path, calibration_path, authorization_path)
    if not all(supplied):
        raise ValueError("Measurement protocol, calibration reference and authorization are all required")
    paths = [stage_path(value, must_exist=True) for value in supplied]
    protocol, calibration, grant = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    if (protocol.get("schema_version") != 2
            or protocol.get("new_volume_consistency", {}).get("maximum_single_relative_accuracy") != 0.01
            or protocol.get("new_volume_consistency", {}).get("maximum_combined_relative_uncertainty") != 0.01):
        raise ValueError("Unexpected precision-aware measurement protocol")
    if (calibration.get("stage") != "15_dynamic_mesh" or calibration.get("mode") != "calibration"
            or calibration.get("state") != "passed" or calibration.get("small_diagnostic_passed") is not True
            or calibration.get("source_unchanged") is not True
            or calibration.get("source_sha256_before") != config["source_sha256"]
            or calibration.get("source_sha256_after") != config["source_sha256"]):
        raise ValueError("Calibration reference is not an actually passed matching control report")
    if (grant.get("maximum_small_diagnostic_attempts") != 2
            or grant.get("maximum_seconds_per_small_diagnostic") != 300
            or grant.get("maximum_additional_complete_cad_builds") != 1
            or grant.get("additional_complete_cad_build_id") != "cad_build/attempt_004"
            or grant.get("original_cad_geometry_changes_allowed") is not False):
        raise ValueError("Additional complete-build authorization is not the finite user grant")
    if config["mode"] == "build":
        expected = (STAGE_ROOT / "runs/cad_build/attempt_004").resolve()
        if Path(config["attempt_dir"]).resolve() != expected:
            raise ValueError("This extension authorizes only cad_build/attempt_004")
    for key, path in zip(("measurement_protocol", "calibration_reference", "additional_authorization"), paths):
        record = {"path": str(path), "sha256": sha256(path)}
        config[key] = record
        config["frozen_sources"].append(record)


def generate(mode: str, attempt: str | Path, *, samples: int = 9,
             padding: float | None = None, build_report: str | Path | None = None,
             measurement_protocol: str | Path | None = None,
             calibration_reference: str | Path | None = None,
             additional_authorization: str | Path | None = None) -> dict:
    if mode not in ("probe", "build", "verify"):
        raise ValueError("Unknown CAD mode")
    contract = load_contract()
    target = stage_path(attempt)
    if target.exists():
        raise FileExistsError("Attempt must be a new directory: " + str(target))
    source = (WORKSPACE_ROOT / contract["source"]["step_relative_to_workspace"]).resolve(strict=True)
    if source != (WORKSPACE_ROOT / "简化模型_导出/AUVM.step").resolve(strict=True):
        raise ValueError("Unexpected STEP source")
    angles = sample_angles(samples, *contract["motion"]["j1_range_rad"])
    pad = contract["domains"]["initial_padding_m"] if padding is None else padding
    if isinstance(pad, bool) or not math.isfinite(pad) or not (0.005 <= pad <= 0.2):
        raise ValueError("Padding must be a finite value in [0.005,0.2] m")
    config = {
        "schema_version": 1, "mode": mode, "stage_root": str(STAGE_ROOT), "attempt_dir": str(target),
        "source_path": str(source), "source_sha256": sha256(source), "step_length_unit": step_units(source),
        "frozen_sources": frozen_sources(), "recipe_sha256": APPROVED_SOURCE_HASHES[RECIPE_PATH.name],
        "pose_rad": contract["source"]["pose_rad"],
        "joint_centres_m": contract["frame"]["joint_centres_CAD0_m"],
        "joint_axes": contract["frame"]["joint_axes_CAD0"], "j1_samples_rad": angles,
        "padding_m": pad, "tank_lo_m": contract["domains"]["tank_lo_m"], "tank_hi_m": contract["domains"]["tank_hi_m"],
        "distal_shift_m": contract["source"]["distal_shift_before_pose_m"],
        "volume_diagnostics": {"surface_deviations_m": [1e-5, 2.5e-6, 6.25e-7],
                               "maximum_facets": 250000, "tessellation_is_diagnostic_only": True,
                               "absolute_tolerance_m3": 1e-10, "relative_tolerance": 1e-8},
        "script_path": str(target / ("sc_" + mode + "_stage15.py")),
        "output_json": str(target / (mode + "_report.json")),
    }
    if mode == "probe":
        template = "sc_probe.py.in"
    elif mode == "build":
        if target.parent.name == "cad_build" and target.name not in ("attempt_001", "attempt_002", "attempt_003", "attempt_004"):
            raise ValueError("No authorization for an additional complete CAD build beyond attempt004")
        if any((measurement_protocol, calibration_reference, additional_authorization)) or target.name == "attempt_004":
            bind_measurement_sources(config, measurement_protocol, calibration_reference, additional_authorization)
        config["output_scdoc"] = {name: str(target / (name + "_fluid.scdoc")) for name in ("background", "arm")}
        template = "sc_build.py.in"
    else:
        if build_report is None:
            raise ValueError("verify requires --build-report from an actual isolated build")
        path = stage_path(build_report, must_exist=True)
        build = json.loads(path.read_text(encoding="utf-8"))
        if (build.get("mode") != "build" or build.get("stage") != "15_dynamic_mesh"
                or build.get("state") != "passed" or build.get("source_unchanged") is not True
                or build.get("source_sha256_before") != config["source_sha256"]
                or build.get("source_sha256_after") != config["source_sha256"]
                or sorted(build.get("domains", {})) != ["arm", "background"]):
            raise ValueError("Build report has no eligible CAD artifacts or matching source hashes")
        artifacts = []
        for domain in ("background", "arm"):
            product = build["domains"][domain]
            if product.get("state") != "passed":
                raise ValueError("Build domain did not finish: " + domain)
            product_path = stage_path(product["scdoc"], must_exist=True)
            if product_path.suffix.lower() != ".scdoc" or not product_path.is_file() or sha256(product_path) != product["scdoc_sha256"]:
                raise ValueError("Build artifact path/hash mismatch: " + domain)
            artifacts.append({"path": str(product_path), "sha256": product["scdoc_sha256"]})
        if len({row["path"] for row in artifacts}) != 2:
            raise ValueError("Two domains must be different CAD files")
        config["build_report"] = str(path)
        config["build_report_sha256"] = sha256(path)
        config["frozen_sources"] += [{"path": str(path), "sha256": config["build_report_sha256"]}] + artifacts
        upstream = build.get("config", {})
        keys = ("measurement_protocol", "calibration_reference", "additional_authorization")
        if any(key in upstream for key in keys):
            references = []
            for key in keys:
                entry = upstream[key]
                ref_path = stage_path(entry["path"], must_exist=True)
                if sha256(ref_path) != entry["sha256"]:
                    raise ValueError("Upstream measurement source changed: " + key)
                references.append(ref_path)
            bind_measurement_sources(config, *references)
        template = "sc_verify.py.in"
    text = render_template(template, config)
    # Validate and render everything before creating the new output directory.
    target.parent.mkdir(parents=True, exist_ok=True)
    if stage_path(target) != target:
        raise ValueError("Attempt parent changed during preparation")
    target.mkdir(exist_ok=False)
    script = Path(config["script_path"])
    with script.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    manifest = target / "generation.json"
    prepared = {"state": "not_run", "generated": True, "external_software_started": False,
                "script_path": str(script), "output_json": config["output_json"],
                "script_sha256": sha256(script), "mode": mode, "config": config}
    with manifest.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(prepared, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return prepared


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    for mode in ("probe", "build", "verify"):
        child = sub.add_parser(mode)
        child.add_argument("--attempt", required=True, help="New directory under stage15; relative paths are stage15-relative")
        child.add_argument("--samples", type=int, default=9)
        if mode == "build":
            child.add_argument("--padding", type=float, default=None)
            child.add_argument("--measurement-protocol")
            child.add_argument("--calibration-reference")
            child.add_argument("--additional-authorization")
        if mode == "verify":
            child.add_argument("--build-report", required=True)
    args = parser.parse_args(argv)
    try:
        prepared = generate(args.mode, args.attempt, samples=args.samples, padding=getattr(args, "padding", None),
                            build_report=getattr(args, "build_report", None),
                            measurement_protocol=getattr(args, "measurement_protocol", None),
                            calibration_reference=getattr(args, "calibration_reference", None),
                            additional_authorization=getattr(args, "additional_authorization", None))
    except (ValueError, OSError, KeyError, json.JSONDecodeError) as exc:
        print(json.dumps({"state": "failed", "generated": False, "external_software_started": False,
                          "error": str(exc)}, ensure_ascii=True))
        return 2
    print(json.dumps({key: prepared[key] for key in ("state", "generated", "external_software_started", "script_path", "output_json", "script_sha256")}, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

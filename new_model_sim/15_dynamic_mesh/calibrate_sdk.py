"""Generate, but never run, one of the two explicitly authorized SDK controls."""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path

from preflight import STAGE, WORKSPACE, load_contract, sha256, verify_inputs

AUTH = STAGE / "frozen/diagnostic_authorization_001.json"


def authorization():
    data = json.loads(AUTH.read_text(encoding="utf-8"))
    if data.get("maximum_small_diagnostic_attempts") != 2 or data.get("maximum_seconds_per_small_diagnostic") != 300:
        raise ValueError("Unexpected authorized diagnostic limits")
    if data.get("original_cad_geometry_changes_allowed") is not False:
        raise ValueError("This diagnostic may not change the source CAD")
    return data


def generate(index: int):
    if isinstance(index, bool) or index not in (1, 2):
        raise ValueError("Only diagnostic attempts 1 and 2 are authorized")
    authorization()
    manifest = json.loads((STAGE / "frozen/input_manifest.json").read_text(encoding="utf-8"))
    if verify_inputs(manifest)["status"] != "passed":
        raise ValueError("Frozen source inputs changed")
    contract = load_contract()
    target = STAGE / "runs/cad_calibration" / ("attempt_%03d" % index)
    if target.exists():
        raise FileExistsError("Refusing diagnostic attempt overwrite: " + str(target))
    source = WORKSPACE / contract["source"]["step_relative_to_workspace"]
    templates = [STAGE / "cad" / name for name in ("sc_common.py.in", "sc_volume.py.in", "sc_calibration_probe.py.in")]
    frozen = [STAGE / "contract.json", AUTH, Path(__file__)] + templates
    cfg = {
        "schema_version": 1, "mode": "calibration", "stage_root": str(STAGE), "attempt_dir": str(target),
        "source_path": str(source), "source_sha256": sha256(source),
        "source_geometry_not_opened": True,
        "frozen_sources": [{"path": str(path), "sha256": sha256(path)} for path in frozen],
        "pose_rad": contract["source"]["pose_rad"],
        "joint_centres_m": contract["frame"]["joint_centres_CAD0_m"],
        "joint_axes": contract["frame"]["joint_axes_CAD0"],
        "script_path": str(target / "sc_sdk_calibration.py"),
        "output_json": str(target / "calibration_report.json"),
        "authorization_path": str(AUTH), "authorization_sha256": sha256(AUTH),
        "maximum_runtime_seconds": 300, "attempt_index": index,
        "calibration_scale": 0.1,
        "volume_diagnostics": {
            "surface_deviations_m": [5e-7, 1e-7, 2e-8],
            "maximum_facets": 250000,
            "absolute_tolerance_m3": 1e-10, "relative_tolerance": 1e-8,
            "scope": "analytic synthetic controls only; no full-domain accuracy claim",
        },
    }
    marker = "'__STAGE15_CONFIG_JSON__'"
    common = templates[0].read_text(encoding="utf-8")
    if common.count(marker) != 1:
        raise ValueError("Ambiguous configuration marker")
    code = common.replace(marker, repr(json.dumps(cfg, ensure_ascii=True, sort_keys=True)))
    code += "\n" + templates[1].read_text(encoding="utf-8")
    code += "\n" + templates[2].read_text(encoding="utf-8")
    ast.parse(code, filename=cfg["script_path"])
    if "DocumentOpen.Execute" in code:
        # The common helper defines a source-reader but this mode must never call
        # it. Audit the mode body separately, rather than deleting shared code.
        mode = ast.parse(templates[2].read_text(encoding="utf-8"))
        if any(isinstance(node, ast.Name) and node.id == "load_source_parts" for node in ast.walk(mode)):
            raise ValueError("Synthetic control attempted to load robot geometry")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.mkdir(exist_ok=False)
    Path(cfg["script_path"]).write_text(code, encoding="utf-8", newline="\n")
    result = {"state": "not_run", "external_software_started": False, "config": cfg,
              "script_path": cfg["script_path"], "output_json": cfg["output_json"],
              "script_sha256": sha256(cfg["script_path"]), "timeout_seconds": 300}
    (target / "generation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", type=int, choices=(1, 2), required=True)
    result = generate(parser.parse_args().attempt)
    print(json.dumps({key: result[key] for key in ("state", "external_software_started", "script_path", "output_json", "timeout_seconds")}, ensure_ascii=False))

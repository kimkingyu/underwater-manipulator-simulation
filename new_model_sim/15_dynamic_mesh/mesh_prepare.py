"""Generate, never execute, the stage15 two-domain Workbench meshing journal.

Use --collect after a separately authorized Workbench run. Process exit status is
not mesh evidence. --cell-type polyhedral records intent but performs no silent
conversion; static tetrahedral sampling is supported while motion stays blocked.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path
import sys

from runtime import (STAGE_ROOT, RuntimeGateError, create_attempt, file_record,
                     load_contract, snapshot_inputs, write_json_new)


TEMPLATES = STAGE_ROOT / "mesh"


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise RuntimeGateError(name + " must be finite and positive")
    return value


def domain_settings(contract, attempt, *, background_size=0.15, arm_size=0.02,
                    interface_size=0.01, contact_size=0.002, overlap_radius=0.65,
                    minimum_size=0.0003, growth_rate=1.3):
    numbers = dict(background_size=background_size, arm_size=arm_size,
                   interface_size=interface_size, contact_size=contact_size,
                   overlap_radius=overlap_radius, minimum_size=minimum_size,
                   growth_rate=growth_rate)
    numbers = {k: positive(v, k) for k, v in numbers.items()}
    if not 1.0 < numbers["growth_rate"] <= 2.0:
        raise RuntimeGateError("growth rate must be in (1, 2]")
    if numbers["minimum_size"] > min(numbers[k] for k in ("contact_size", "interface_size", "arm_size", "background_size")):
        raise RuntimeGateError("minimum size exceeds a requested local size")
    domains = contract["domains"]
    background = {"domain": "background", "cell_zone": "fluid_background",
                  "global_size_m": numbers["background_size"],
                  "maximum_size_m": numbers["background_size"] * 4 / 3,
                  "required_selections": list(domains["fixed_walls"]) + ["inlet", "outlet", "tank_wall"],
                  "face_controls": [{"name": "wall_base0", "size_m": numbers["contact_size"]},
                                    {"name": "wall_auv", "size_m": 0.04}],
                  "spheres": [{"centre_m": contract["motion"]["reference_point_m"],
                               "radius_m": numbers["overlap_radius"], "size_m": numbers["interface_size"]}]}
    arm = {"domain": "arm", "cell_zone": "fluid_arm", "global_size_m": numbers["arm_size"],
           "maximum_size_m": numbers["arm_size"],
           "required_selections": list(domains["moving_walls"]) + [domains["overset_boundary"]],
           "face_controls": [{"name": name, "size_m": numbers["contact_size"] if name == "wall_l1" else
                              (0.0015 if name in ("wall_pin4", "wall_jaw1", "wall_jaw2") else 0.0035)}
                             for name in domains["moving_walls"]] +
                            [{"name": domains["overset_boundary"], "size_m": numbers["interface_size"]}],
           "spheres": []}
    for item in (background, arm):
        item.update({"minimum_size_m": numbers["minimum_size"], "growth_rate": numbers["growth_rate"],
                     "stats_path": str(attempt / (item["domain"] + "_meshing_stats.json")),
                     "element_method": "AllTriAllTet", "compatibility": "unverified",
                     "motion_authorized": False})
    return [background, arm]


def generate(attempt, cad_background, cad_arm, *, contract_path=None, cell_type="tetrahedral",
             cad_verification=None, stage_root=STAGE_ROOT, fixed_refinement=False, **mesh_options):
    contract_path = Path(contract_path or STAGE_ROOT / "contract.json")
    contract = load_contract(contract_path)
    cad = [Path(cad_background).resolve(strict=True), Path(cad_arm).resolve(strict=True)]
    if cad[0] == cad[1] or any(p.suffix.lower() != ".scdoc" for p in cad):
        raise RuntimeGateError("provide two distinct .scdoc files, one per independent fluid domain")
    for path in cad:
        file_record(path)
    verification_record = file_record(cad_verification) if cad_verification else None
    if cell_type not in ("tetrahedral", "polyhedral"):
        raise RuntimeGateError("unsupported requested cell type")
    # The fixed trial is an internal exact policy, not a parameter sweep. Its
    # baseline sizes/options cannot be overridden through the legacy kwargs.
    if type(fixed_refinement) is not bool:
        raise RuntimeGateError("Fixed refinement flag must be a strict boolean")
    if fixed_refinement and (mesh_options or cell_type != "tetrahedral"):
        raise RuntimeGateError("Fixed refinement permits no baseline overrides or conversion")
    def settings(path):
        if fixed_refinement:
            from fixed_refinement_config import trial_settings
            return trial_settings(contract, path)
        return domain_settings(contract, path, **mesh_options)
    # Validate numerical settings before creating any output directory.
    settings(Path(attempt))
    attempt = create_attempt(attempt, stage_root=stage_root)
    configs = settings(attempt)
    if fixed_refinement:
        from fixed_refinement_config import configuration_comparison
        write_json_new(attempt / "fixed_refinement_configuration.json",
                       configuration_comparison(contract, attempt))
    meshing_template = (TEMPLATES / "meshing_stage15.py.in").read_text(encoding="utf-8")
    wb_template = (TEMPLATES / "wb_stage15.wbjn.in").read_text(encoding="utf-8")
    for item, source in zip(configs, cad):
        item["cad_path"] = str(source)
        item["meshing_script"] = meshing_template.replace("__CONFIG_JSON__", repr(json.dumps(item, ensure_ascii=True)))
    config = {"project": str(attempt / "stage15_two_domains.wbpj"),
              "trace": str(attempt / "workbench.trace.txt"),
              "completion": str(attempt / "workbench_completion.json"), "domains": configs}
    journal = attempt / "mesh_stage15.wbjn"
    with journal.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(wb_template.replace("__CONFIG_JSON__", repr(json.dumps(config, ensure_ascii=True))))
    snapshot_inputs(attempt, inputs=[contract_path] + cad + ([Path(cad_verification)] if cad_verification else []),
                    sources=[Path(__file__), STAGE_ROOT / "runtime.py", TEMPLATES / "meshing_stage15.py.in", TEMPLATES / "wb_stage15.wbjn.in"] +
                            ([STAGE_ROOT / "fixed_refinement_config.py"] if fixed_refinement else []),
                    settings={"domains": [{k: v for k, v in x.items() if k != "meshing_script"} for x in configs],
                              "cell_type_requested": cell_type})
    plan = {"status": "not_run", "valid": False, "journal_generated": True, "journal": file_record(journal),
            "attempt": str(attempt), "workbench_executed": False, "static_overset": "not_run",
            "motion_authorized": False, "generated_cell_type": "tetrahedral",
            "requested_cell_type": cell_type, "polyhedral_conversion": "not_run",
            "overset_cell_type_compatibility": "unverified", "cad": [file_record(p) for p in cad],
            "cad_verification": verification_record,
            "domains": [{k: v for k, v in x.items() if k != "meshing_script"} for x in configs],
            "workbench_command": [contract["software"]["python"],
                str(STAGE_ROOT.parent / "04_workbench/run_wb_fullenv.py"), str(journal), str(attempt / "workbench.trace.txt"),
                str(2 * contract["budget"]["mesh_system_timeout_s"])],
            "notes": ["Generated journal is not executed or API validated.",
                      "Contact size scopes wall_base0/wall_l1; background overlap uses the recorded sphere.",
                      "Finite CAD/contact checks and Fluent orphan/quality checks remain separate prerequisites."]}
    write_json_new(attempt / "mesh_plan.json", plan)
    return plan


def collect(attempt, mesh_background, mesh_arm, *, deadline_unix=None, cancel_check=None):
    def check_budget():
        if cancel_check is not None and cancel_check():
            raise RuntimeGateError('mesh inventory cancelled; partial evidence only')
        if deadline_unix is not None and time.time() >= deadline_unix:
            raise TimeoutError('mesh inventory exceeded its remaining total budget')
    def record(path):
        if deadline_unix is None and cancel_check is None:
            return file_record(path)
        path = Path(path).resolve(strict=True)
        check_budget()
        before = path.stat()
        hasher = hashlib.sha256()
        with path.open('rb') as stream:
            while True:
                check_budget()
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                hasher.update(chunk)
        check_budget()
        after = path.stat()
        if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
            raise RuntimeGateError('mesh inventory source changed while hashing: ' + str(path))
        return {'path': str(path), 'size': after.st_size, 'sha256': hasher.hexdigest()}
    attempt = Path(attempt).resolve(strict=True)
    plan = json.loads((attempt / "mesh_plan.json").read_text(encoding="utf-8"))
    report = {"status": "failed", "valid": False, "motion_authorized": False,
              "static_overset": "not_run", "overset_cell_type_compatibility": "unverified",
              "requested_cell_type": plan["requested_cell_type"], "polyhedral_conversion": "not_run", "meshes": [], "errors": [],
              "cad": plan["cad"], "cad_verification": plan.get("cad_verification"),
              "mesh_plan": file_record(attempt / "mesh_plan.json"), "journal": plan["journal"]}
    try:
        check_budget()
        if record(plan["journal"]["path"])["sha256"] != plan["journal"]["sha256"]:
            raise RuntimeGateError("generated journal changed before collection")
        frozen = json.loads((attempt / "inputs.json").read_text(encoding="utf-8"))
        for source in frozen["inputs"] + frozen["sources"]:
            if record(source["path"])["sha256"] != source["sha256"]:
                raise RuntimeGateError("frozen input changed: " + source["path"])
        report["workbench_completion"] = record(attempt / "workbench_completion.json")
        completion = json.loads((attempt / "workbench_completion.json").read_text(encoding="utf-8-sig"))
        if completion.get("status") != "completed_pending_mesh_inventory" or len(completion.get("systems", [])) != 2:
            raise RuntimeGateError("Workbench completion/statistics missing or unsuccessful; exit 0 is insufficient")
        paths = [Path(mesh_background).resolve(strict=True), Path(mesh_arm).resolve(strict=True)]
        if paths[0] == paths[1] or os.path.samefile(paths[0], paths[1]):
            raise RuntimeGateError("background and arm cannot use the same mesh")
        for domain, path in zip(("background", "arm"), paths):
            if attempt not in path.parents:
                raise RuntimeGateError("exported mesh must belong to this new attempt")
            if not (path.name.endswith(".msh") or path.name.endswith(".msh.h5")):
                raise RuntimeGateError("expected Fluent mesh file")
            if not path.is_file() or path.stat().st_size <= 0:
                raise RuntimeGateError("exported mesh is empty or not a regular file")
            stats = json.loads((attempt / (domain + "_meshing_stats.json")).read_text(encoding="utf-8-sig"))
            if stats.get("domain") != domain or stats.get("mesh_generated") is not True or stats.get("status") == "failed":
                raise RuntimeGateError("missing successful meshing evidence for " + domain)
            for key in ("nodes", "elements"):
                if isinstance(stats.get(key), bool) or not isinstance(stats.get(key), int) or stats[key] <= 0:
                    raise RuntimeGateError("missing/invalid mesh " + key + " for " + domain)
            row = {"domain": domain, "file": record(path), "nodes": stats["nodes"], "elements": stats["elements"],
                   "quality": stats.get("quality", {}), "statistics": record(attempt / (domain + "_meshing_stats.json"))}
            report["meshes"].append(row)
            orthogonal = stats.get("quality", {}).get("orthogonal_quality", {})
            metric = orthogonal.get("minimum")
            if (orthogonal.get("status") == "unavailable" or isinstance(metric, bool) or
                    not isinstance(metric, (int, float)) or not math.isfinite(metric)):
                report["errors"].append(domain + ": missing finite native minimum orthogonal quality")
            elif metric < 0.01:
                report["errors"].append(domain + ": minimum orthogonal quality below 0.01; no motion")
        report["status"] = "failed" if report["errors"] else "mesh_exported_pending_fluent_static_gate"
    except Exception as exc:
        report["errors"].append(type(exc).__name__ + ": " + str(exc))
    write_json_new(attempt / "mesh_manifest.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--contract")
    parser.add_argument("--cad-background")
    parser.add_argument("--cad-arm")
    parser.add_argument("--cad-verification", help="actual independent CAD verify report; not a generated manifest")
    parser.add_argument("--cell-type", choices=("tetrahedral", "polyhedral"), default="tetrahedral")
    parser.add_argument("--collect", action="store_true")
    parser.add_argument("--mesh-background")
    parser.add_argument("--mesh-arm")
    for name, default in (("background-size", .15), ("arm-size", .02), ("interface-size", .01),
                          ("contact-size", .002), ("overlap-radius", .65), ("minimum-size", .0003)):
        parser.add_argument("--" + name + "-m", type=float, default=default, dest=name.replace("-", "_"))
    parser.add_argument("--growth-rate", type=float, default=1.3)
    args = parser.parse_args(argv)
    try:
        if args.collect:
            if not args.mesh_background or not args.mesh_arm:
                parser.error("--collect requires both actual mesh paths")
            result = collect(args.attempt, args.mesh_background, args.mesh_arm)
        else:
            if not args.cad_background or not args.cad_arm:
                parser.error("generation requires two explicit saved .scdoc files")
            opts = {key: getattr(args, key) for key in ("background_size", "arm_size", "interface_size", "contact_size", "overlap_radius", "minimum_size", "growth_rate")}
            result = generate(args.attempt, args.cad_background, args.cad_arm, contract_path=args.contract,
                              cell_type=args.cell_type, cad_verification=args.cad_verification, **opts)
        print(json.dumps({k: result[k] for k in ("status", "valid", "motion_authorized")}, ensure_ascii=False))
        return 2 if result["status"] == "failed" else 0
    except Exception as exc:
        print(type(exc).__name__ + ": " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

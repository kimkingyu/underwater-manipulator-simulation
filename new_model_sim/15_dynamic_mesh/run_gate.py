"""Fail-closed stage15 static overset assembly and non-advancing API probes.

No motion/initialization/solve operation is implemented here. --api-only reads
installed Python API source, without launching Fluent. --gate probe may load an
explicit case and inspect live APIs, but it never validates motion semantics.
"""
from __future__ import annotations

import argparse
import ast
import inspect
import json
import math
from pathlib import Path
import re
import sys
import traceback

from runtime import (STAGE_ROOT, RuntimeGateError, create_attempt, file_record,
                     launch, load_contract, snapshot_inputs, write_json_new)

FLOAT = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
KINDS = ("solve", "receptor", "donor", "orphan", "dead")
REQUIRED_API = ("file.read_mesh", "mesh.modify_zones.append_mesh", "mesh.modify_zones.list_zones",
                "tui.mesh.check", "tui.mesh.quality", "tui.define.overset_interfaces.options.expert",
                "tui.define.overset_interfaces.options.verbosity", "tui.define.overset_interfaces.create",
                "tui.define.overset_interfaces.intersect", "tui.define.overset_interfaces.mark_cells",
                "tui.define.overset_interfaces.list", "tui.define.overset_interfaces.check")


def _audit_adapter(text):
    """A deliberately narrow adapter, not a guessed audit dictionary schema."""
    import audit
    result = audit.parse_solver_text(text)
    if not isinstance(result, dict) or not isinstance(result.get("fatal_errors"), list):
        raise RuntimeGateError("audit.parse_solver_text contract unrecognized: expected explicit fatal_errors list")
    if not all(isinstance(item, str) for item in result["fatal_errors"]):
        raise RuntimeGateError("audit fatal_errors must contain strings")
    return {"raw": result, "fatal_errors": result["fatal_errors"],
            "adapter": "stage15-fatal-errors-v1; counts independently extracted with native source lines"}


def native_evidence(text):
    """Extract observations, preserving all counts, rather than defaulting to zero."""
    if not isinstance(text, str) or not text.strip():
        raise RuntimeGateError("missing native transcript")
    observed = {kind: [] for kind in KINDS}
    orphan_observations, negative, minimum_volume, orthogonal, cells = [], [], [], [], []
    for number, line in enumerate(text.splitlines(), 1):
        source = {"line": number, "text": line.strip()}
        for match in re.finditer(r"\bMarked\s+([0-9][0-9,]*)\s+(?:overset\s+)?(solve|receptor|donor|orphan|dead)\s+cells?\b", line, re.I):
            kind = match.group(2).lower()
            observed[kind].append(dict(source, value=int(match.group(1).replace(",", ""))))
        for match in re.finditer(r"\b([0-9][0-9,]*)\s+(?:overset\s+)?orphan\s+cells?\b", line, re.I):
            orphan_observations.append(dict(source, value=int(match.group(1).replace(",", ""))))
        for match in re.finditer(r"\b([0-9][0-9,]*)\s+(?:cells?\s+with\s+negative\s+volumes?|negative[- ]volume\s+cells?)\b", line, re.I):
            negative.append(dict(source, value=int(match.group(1).replace(",", ""))))
        for match in re.finditer(r"\bnegative[- ]volume\s+cells?\s*[:=]\s*([0-9][0-9,]*)", line, re.I):
            negative.append(dict(source, value=int(match.group(1).replace(",", ""))))
        for match in re.finditer(r"\bminimum\s+(?:cell\s+)?volume\s*(?:\(m3\))?\s*[:=]\s*(" + FLOAT + r"|nan|[+-]?inf)", line, re.I):
            minimum_volume.append(dict(source, value=float(match.group(1))))
        for match in re.finditer(r"\bminimum\s+orthogonal\s+quality\s*[:=]\s*(" + FLOAT + ")", line, re.I):
            orthogonal.append(dict(source, value=float(match.group(1))))
        for match in re.finditer(r"^\s*([0-9][0-9,]*)\s+cells\s*,?\s+(?:[0-9]+\s+(?:cell\s+)?zones|zone)", line, re.I):
            cells.append(dict(source, value=int(match.group(1).replace(",", ""))))
    derived = None
    if not negative and minimum_volume and all(x["value"] > 0 and math.isfinite(x["value"]) for x in minimum_volume):
        derived = {"value": 0, "derived": True, "source": "native all-cell minimum volume is strictly positive",
                   "minimum_volume_observations": minimum_volume}
    return {"classifications": observed, "orphan_observations": orphan_observations,
            "negative_volume_observations": negative, "negative_volume_derived": derived,
            "minimum_volume_observations": minimum_volume, "orthogonal_quality_observations": orthogonal,
            "cell_count_observations": cells}


def validate_static_evidence(text, *, threshold=0.01):
    evidence = native_evidence(text)
    audit = _audit_adapter(text)
    errors = list(audit["fatal_errors"])
    errors.extend(line.strip() for line in text.splitlines() if re.search(
        r"mesh check failed|invalid command|not a valid command|connectivity (?:error|failed)", line, re.I))
    import audit as audit_module
    classes = evidence["classifications"]
    for kind in KINDS:
        if not classes[kind]:
            errors.append("missing explicit native marked " + kind + " cell count")
    for kind in ("solve", "receptor", "donor"):
        if classes[kind] and any(row["value"] <= 0 for row in classes[kind]):
            errors.append("no valid positive " + kind + " classification")
    for row in evidence["orphan_observations"]:
        try:
            audit_module.require_zero_orphans(row["value"], row["text"])
        except audit_module.GateError as exc:
            errors.append(str(exc))
    if not evidence["orphan_observations"]:
        errors.append("missing explicit native zero orphan evidence")
    negative = evidence["negative_volume_observations"]
    if negative:
        if any(x["value"] != 0 for x in negative):
            errors.append("negative-volume cells are present")
    elif evidence["negative_volume_derived"] is None:
        errors.append("negative-volume count and strictly-positive global minimum-volume proof both missing")
    values = evidence["orthogonal_quality_observations"]
    if not values:
        errors.append("missing native minimum orthogonal quality")
    elif any(not math.isfinite(x["value"]) or not threshold <= x["value"] <= 1 for x in values):
        errors.append("native minimum orthogonal quality below threshold or invalid")
    if any(not math.isfinite(x["value"]) or x["value"] <= 0 for x in evidence["minimum_volume_observations"]):
        errors.append("native nonpositive or nonfinite minimum cell volume")
    return {"status": "failed" if errors else "passed", "valid": not errors,
            "errors": list(dict.fromkeys(errors)), "native": evidence, "audit": audit}


def zone_mapping(roster, contract):
    expected = contract["domains"]
    fluid = roster.get("fluid")
    if not isinstance(fluid, list) or len(fluid) != 2:
        raise RuntimeGateError("exactly two fluid cell zones are required")
    mapped = {}
    for canonical in expected["cell_zones"]:
        # Meshing's documented-in-local-evidence FFF prefix is allowed, not arbitrary substring guessing.
        matches = [name for name in fluid if re.fullmatch(r"(?:fff(?:-\d+)?_)?" + re.escape(canonical), name, re.I)]
        if len(matches) != 1:
            raise RuntimeGateError("ambiguous or missing exact fluid mapping: " + canonical)
        mapped[canonical] = matches[0]
    if len(set(mapped.values())) != 2:
        raise RuntimeGateError("fluid mappings are not independent")
    walls = roster.get("wall")
    required = set(expected["fixed_walls"] + expected["moving_walls"] + ["tank_wall"])
    if not isinstance(walls, list) or set(walls) != required:
        raise RuntimeGateError("wall roster mismatch: expected " + repr(sorted(required)) + "; observed " + repr(walls))
    if roster.get("overset") != [expected["overset_boundary"]]:
        raise RuntimeGateError("one exact overset_arm boundary is required")
    if roster.get("velocity_inlet") != ["inlet"] or roster.get("pressure_outlet") != ["outlet"]:
        raise RuntimeGateError("background inlet/outlet boundary types or names are wrong")
    if roster.get("solid"):
        raise RuntimeGateError("unexpected solid cell zones")
    return {"canonical_to_actual": mapped, "roster": roster, "native_zone_ids": None}


def attach_zone_ids(mapping, native_text):
    required = set(mapping["canonical_to_actual"].values())
    for kind in ("wall", "overset", "velocity_inlet", "pressure_outlet"):
        required.update(mapping["roster"][kind])
    ids = {}
    for line in native_text.splitlines():
        tokens = line.replace("(", " ").replace(")", " ").split()
        if not tokens or not tokens[0].isdigit():
            continue
        matches = required.intersection(tokens[1:])
        for name in matches:
            value = int(tokens[0])
            if name in ids and ids[name] != value:
                raise RuntimeGateError("conflicting native zone IDs for " + name)
            ids[name] = value
    if set(ids) != required or len(set(ids.values())) != len(ids):
        raise RuntimeGateError("native list-zones must provide unique IDs for all required zones; missing " + repr(sorted(required - set(ids))))
    mapping["native_zone_ids"] = ids
    return mapping


def resolve(obj, path):
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


class Steps:
    def __init__(self):
        self.rows = []

    def call(self, name, fn, *args, **kwargs):
        row = {"step": name, "status": "failed", "validation": "not_performed", "args": list(args), "kwargs": kwargs}
        self.rows.append(row)
        try:
            result = fn(*args, **kwargs)
            row["status"] = "executed"
            row["return_is_none"] = result is None
            row["result"] = result if isinstance(result, (str, int, float, list, dict, bool)) or result is None else repr(result)
            # None/empty string means no returned evidence, never a passed check.
            return result
        except Exception as exc:
            row["exception"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
            raise


def read_roster(solver):
    c = solver.setup.cell_zone_conditions
    b = solver.setup.boundary_conditions
    return {"fluid": sorted(c.fluid.keys()), "solid": sorted(c.solid.keys()),
            "wall": sorted(b.wall.keys()), "overset": sorted(b.overset.keys()),
            "velocity_inlet": sorted(b.velocity_inlet.keys()), "pressure_outlet": sorted(b.pressure_outlet.keys())}


def _transcript_text(path):
    if not Path(path).is_file() or Path(path).stat().st_size == 0:
        raise RuntimeGateError("required native transcript was not written: " + str(path))
    return Path(path).read_bytes().decode("utf-8", errors="replace")


def static_session(solver, attempt, background, arm, contract, *, save_diagnostic=True):
    """Operate only on the supplied owned session. Evidence comes from native transcript."""
    attempt = Path(attempt)
    steps = Steps()
    report = {"status": "failed", "valid": False, "steps": steps.rows, "motion_authorized": False,
              "initializations": 0, "flow_iterations": 0, "mesh_time_steps": 0,
              "cell_type_compatibility": "unverified", "base_candidate": None, "errors": []}
    trn = attempt / "static.native.trn"
    if trn.exists():
        raise FileExistsError("native transcript must be new")
    transcript_active = False
    loaded = False
    try:
        for path in REQUIRED_API:
            if not callable(resolve(solver, path)):
                raise RuntimeGateError("required native API unavailable: " + path)
        steps.call("start_native_transcript", solver.tui.file.start_transcript, str(trn))
        transcript_active = True
        steps.call("read_background", solver.file.read_mesh, file_name=str(Path(background).resolve()))
        loaded = True
        steps.call("append_arm", solver.mesh.modify_zones.append_mesh, file_name=Path(arm).resolve().as_posix())
        report["zone_mapping"] = zone_mapping(read_roster(solver), contract)
        steps.call("list_exact_zone_ids", solver.mesh.modify_zones.list_zones)
        # A pre-interface mesh-check predictably flags unassigned overset zones.
        # Only quality is sampled here; connectivity check is performed after creation.
        steps.call("mesh_quality_before", solver.tui.mesh.quality)
        overset = solver.tui.define.overset_interfaces
        steps.call("overset_verbosity", overset.options.verbosity, 2)
        steps.call("enable_expert_for_documented_intersect", overset.options.expert, "yes")
        actual = report["zone_mapping"]["canonical_to_actual"]
        interface = contract["domains"]["overset_interface"]
        steps.call("create_exact_interface", overset.create, interface,
                   actual["fluid_background"], "()", actual["fluid_arm"], "()")
        # Local tui_242 docs require expert before intersect; never retry intersect_all.
        steps.call("intersect_single_interface", overset.intersect, interface)
        for kind in KINDS:
            steps.call("native_mark_" + kind, overset.mark_cells, kind)
        steps.call("native_list_interface", overset.list)
        steps.call("native_check_interface", overset.check)
        steps.call("mesh_check_after", solver.tui.mesh.check)
        steps.call("mesh_quality_after", solver.tui.mesh.quality)
    except Exception as exc:
        report["errors"].append(type(exc).__name__ + ": " + str(exc))
        report["traceback"] = traceback.format_exc()
    finally:
        if transcript_active:
            try:
                steps.call("stop_native_transcript", solver.tui.file.stop_transcript)
            except Exception as exc:
                report["errors"].append("transcript stop failed: " + repr(exc))
    try:
        text = _transcript_text(trn)
        report["transcript"] = file_record(trn)
        report["evidence"] = validate_static_evidence(text, threshold=contract["gates"]["minimum_orthogonal_quality"])
        report["errors"].extend(report["evidence"]["errors"])
        if not re.search(r"Interface Name:\s*" + re.escape(contract["domains"]["overset_interface"]) + r"\b", text):
            report["errors"].append("native interface list did not confirm the requested interface")
        if "zone_mapping" in report:
            attach_zone_ids(report["zone_mapping"], text)
        else:
            report["errors"].append("no exact zone mapping")
        report["cell_types_observed"] = sorted(set(re.findall(r"\b(tetrahedral|hexahedral|polyhedral|prismatic|pyramid)\s+cells\b", text, re.I)))
    except Exception as exc:
        report["errors"].append("native evidence validation failed: " + repr(exc))
    gate_pass = not report["errors"]
    if loaded and (gate_pass or save_diagnostic):
        case = attempt / ("candidate_uninitialized.cas.h5" if gate_pass else "FAILED_DIAGNOSTIC_DO_NOT_USE_AS_BASE.cas.h5")
        persistence = attempt / "persistence.native.trn"
        active = False
        try:
            if case.exists() or persistence.exists():
                raise FileExistsError("refusing case/transcript overwrite")
            steps.call("start_persistence_transcript", solver.tui.file.start_transcript, str(persistence))
            active = True
            steps.call("write_candidate" if gate_pass else "write_failed_diagnostic", solver.file.write,
                       file_type="case", file_name=str(case))
            steps.call("stop_persistence_transcript", solver.tui.file.stop_transcript)
            active = False
            persistence_text = _transcript_text(persistence)
            errors = _audit_adapter(persistence_text)["fatal_errors"]
            if errors:
                raise RuntimeGateError("case persistence emitted native errors: " + repr(errors))
            report["saved_case"] = file_record(case)
            report["saved_case_role"] = "candidate_pending_owned_session_close" if gate_pass else "failed_diagnostic_only"
            if gate_pass:
                report["base_candidate"] = str(case)
            write_json_new(attempt / "case_role.json", {"case": str(case), "role": report["saved_case_role"],
                                                       "valid_base": False, "motion_authorized": False})
        except Exception as exc:
            report["errors"].append("case persistence failed: " + repr(exc))
        finally:
            if active:
                try:
                    solver.tui.file.stop_transcript()
                except Exception as exc:
                    report["errors"].append("persistence transcript close failed: " + repr(exc))
    if not report["errors"] and report["base_candidate"]:
        report["status"] = "candidate_pending_session_close"
    report["errors"] = list(dict.fromkeys(report["errors"]))
    return report


def local_api_inventory(contract):
    root = Path(contract["software"]["python"]).parent / "Lib/site-packages/ansys/fluent/core"
    paths = {"tui_242": root / "solver/tui_242.py", "launcher": root / "launcher/launcher.py",
             "standalone_launcher": root / "launcher/standalone_launcher.py",
             "settings_setup_242": root / "solver/settings_242/setup.py"}
    result = {"status": "not_run", "valid": False, "external_processes_started": 0,
              "live_api_verified": False, "motion_api": "blocked", "time_step_executed": False,
              "reference_point_verified": False, "files": {}, "declarations": {}}
    for label, path in paths.items():
        if not path.is_file():
            result["files"][label] = {"status": "missing", "path": str(path)}
            continue
        result["files"][label] = file_record(path)
        tree = ast.parse(path.read_text(encoding="utf-8"))
        wanted = {"overset_interfaces", "dynamic_mesh", "preview_mesh_motion", "mesh_motion", "update_mesh",
                  "launch_fluent", "connect_to_fluent", "setup"}
        result["declarations"][label] = [{"name": node.name, "line": node.lineno, "doc": ast.get_docstring(node),
                                            "children": [x.name for x in node.body if isinstance(x, (ast.ClassDef, ast.FunctionDef))]}
                                           for node in ast.walk(tree) if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in wanted]
    result["notes"] = ["Installed generated definitions are availability clues, not a real Fluent API test.",
                        "intersect requires overset options expert=yes in local 24.2 documentation.",
                        "PyFluent Windows standalone launcher contains an implicit retry; runtime bypasses it.",
                        "Installed settings_242.setup has no generated dynamic_mesh child; do not invent setters.",
                        "tui.solve.mesh_motion and tui.define.dynamic_mesh.zones.list are source-declared, not runtime validated.",
                        "No initialization, iteration, or mesh-time advancement performed."]
    return result


def live_api_probe(solver, case, attempt):
    result = {"status": "blocked", "valid": False, "motion_api": "blocked", "mesh_time_steps": 0,
              "initializations": 0, "flow_iterations": 0, "reference_point_verified": False,
              "blocking_reason": "Real one-step motion/time/reference-point semantics have not been executed and audited.",
              "apis": {}}
    path = Path(attempt) / "api_probe.native.trn"
    solver.tui.file.start_transcript(str(path))
    try:
        if case is not None:
            solver.file.read_case(file_name=str(Path(case).resolve()))
            result["loaded_case"] = file_record(case)
        for name in ("tui.define.dynamic_mesh", "tui.define.dynamic_mesh.zones", "tui.solve", "tui.solve.mesh_motion",
                     "setup.general", "setup.dynamic_mesh", "solution.run_calculation"):
            try:
                obj = resolve(solver, name)
                item = {"public_members": sorted(n for n in dir(obj) if not n.startswith("_")),
                        "doc": inspect.getdoc(obj), "executed": False}
                if hasattr(obj, "get_state") and not name.startswith("tui."):
                    state = obj.get_state()
                    item["state"] = state
                    item["state_is_none"] = state is None
                    # An absent setting is not a known reference point.
                result["apis"][name] = item
            except Exception as exc:
                result["apis"][name] = {"status": "unavailable", "error": repr(exc), "executed": False}
        try:
            value = solver.tui.define.dynamic_mesh.zones.list()
            result["native_dynamic_zone_listing"] = {"status": "executed", "return_is_none": value is None,
                                                      "validation": "not_performed"}
        except Exception as exc:
            result["native_dynamic_zone_listing"] = {"status": "unavailable", "error": repr(exc)}
    finally:
        solver.tui.file.stop_transcript()
    result["transcript"] = file_record(path)
    result["native_audit"] = _audit_adapter(_transcript_text(path))
    if result["native_audit"]["fatal_errors"]:
        result["status"] = "failed"
    return result


class PrelaunchBlocked(RuntimeGateError):
    pass


def prelaunch_guard(contract, *, stage_root=STAGE_ROOT):
    """No CLI bypass: frozen hashes, software, clients and licensing are rechecked."""
    import preflight
    manifest_path = Path(stage_root) / "frozen/input_manifest.json"
    if contract != load_contract(Path(stage_root) / "contract.json"):
        return {"status": "blocked", "blockers": ["execution contract differs from frozen stage15 contract"],
                "environment": {"status": "not_run"}}
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    frozen = preflight.verify_inputs(manifest)
    # Do not even probe licensing when the protected inputs have changed.
    if frozen.get("status") != "passed":
        return {"status": "blocked", "frozen_inputs": frozen, "environment": {"status": "not_run"},
                "blockers": ["frozen stage15 input hashes did not verify"]}
    environment = preflight.inspect_environment(contract, root=Path(stage_root), license_probe=True)
    return {"status": "passed" if environment.get("status") == "passed" else "blocked",
            "frozen_inputs": frozen, "frozen_manifest": file_record(manifest_path), "environment": environment,
            "blockers": environment.get("blockers", ["environment schema unknown"]) }


def _checked_record(record):
    if not isinstance(record, dict) or not isinstance(record.get("path"), str):
        raise RuntimeGateError("missing artifact path/hash record")
    actual = file_record(record["path"])
    if actual["sha256"] != record.get("sha256"):
        raise RuntimeGateError("artifact hash mismatch: " + actual["path"])
    return actual


def upstream_chain(path, background, arm, contract, *, stage_root=STAGE_ROOT):
    """Adapt the actual stage15 CAD verify and meshing schema; never infer passes."""
    result = {"status": "blocked", "cad_contact": "not_run", "cad_domains": "not_run",
              "reason": "Missing verified CAD/contact/mesh provenance chain.", "evidence": [],
              "meshes": {"background": file_record(background), "arm": file_record(arm)}}
    if not path:
        return result
    try:
        manifest_record = file_record(path)
        manifest = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        result["evidence"].append(manifest_record)
        if manifest.get("status") != "mesh_exported_pending_fluent_static_gate" or manifest.get("errors") != []:
            raise RuntimeGateError("mesh manifest has missing/failed export statistics")
        rows = manifest.get("meshes", [])
        if len(rows) != 2 or {row.get("domain") for row in rows} != {"background", "arm"}:
            raise RuntimeGateError("mesh manifest must contain exactly background and arm")
        for row in rows:
            actual = _checked_record(row["file"])
            if actual != result["meshes"][row["domain"]]:
                raise RuntimeGateError("static mesh does not match upstream export: " + row["domain"])
            for key in ("nodes", "elements"):
                if isinstance(row.get(key), bool) or not isinstance(row.get(key), int) or row[key] <= 0:
                    raise RuntimeGateError("missing upstream mesh " + key)
            stats_record = _checked_record(row["statistics"])
            stats = json.loads(Path(stats_record["path"]).read_text(encoding="utf-8-sig"))
            if stats.get("domain") != row["domain"] or stats.get("mesh_generated") is not True or stats.get("status") != "generated_pending_export_and_fluent_checks":
                raise RuntimeGateError("actual Meshing statistics do not confirm generation")
            if any(stats.get(k) != row[k] for k in ("nodes", "elements")):
                raise RuntimeGateError("mesh statistics and manifest counts differ")
            minimum = stats.get("quality", {}).get("orthogonal_quality", {}).get("minimum")
            if isinstance(minimum, bool) or not isinstance(minimum, (int, float)) or not math.isfinite(minimum) or minimum < contract["gates"]["minimum_orthogonal_quality"]:
                raise RuntimeGateError("actual upstream mesh quality missing or below threshold")
            result["evidence"] += [actual, stats_record]
        plan_record = _checked_record(manifest["mesh_plan"])
        plan = json.loads(Path(plan_record["path"]).read_text(encoding="utf-8-sig"))
        journal_record = _checked_record(manifest["journal"])
        if plan.get("journal") != manifest["journal"]:
            raise RuntimeGateError("journal differs from frozen meshing plan")
        completion_record = _checked_record(manifest["workbench_completion"])
        completion = json.loads(Path(completion_record["path"]).read_text(encoding="utf-8-sig"))
        if completion.get("status") != "completed_pending_mesh_inventory" or {x.get("domain") for x in completion.get("systems", [])} != {"background", "arm"}:
            raise RuntimeGateError("actual Workbench completion does not confirm two systems")
        result["evidence"] += [plan_record, journal_record, completion_record]
        verification_record = _checked_record(manifest.get("cad_verification"))
        if plan.get("cad_verification") != manifest.get("cad_verification"):
            raise RuntimeGateError("CAD verification changed since meshing plan")
        verified = json.loads(Path(verification_record["path"]).read_text(encoding="utf-8-sig"))
        if verified.get("schema_version") != 1 or verified.get("stage") != "15_dynamic_mesh" or verified.get("mode") != "verify" or verified.get("state") != "passed":
            raise RuntimeGateError("actual independent CAD verify report did not pass")
        domains = verified.get("cad_domains", {})
        if domains.get("state") != "passed" or domains.get("independent_reopen_completed") is not True or domains.get("physical_face_ownership_checked") is not True:
            raise RuntimeGateError("independent saved-domain reopen/ownership evidence is missing")
        contact = verified.get("cad_contact", {})
        if contact.get("state") != "passed" or any(contact.get(k) != 0 for k in ("kernel_failures", "cross_group_penetrations", "unresolved_zero_clearance")):
            raise RuntimeGateError("actual contact classification has unresolved or missing evidence")
        if verified.get("source_unchanged") is not True or verified.get("source_sha256_before") != verified.get("source_sha256_after") or verified.get("input_integrity", {}).get("state") != "passed":
            raise RuntimeGateError("CAD source/recipe unchanged evidence missing")
        source_record = file_record(STAGE_ROOT.parent.parent / contract["source"]["step_relative_to_workspace"])
        if source_record["sha256"] != verified.get("source_sha256_after"):
            raise RuntimeGateError("CAD verify source differs from protected STEP")
        build_record = _checked_record({"path": verified["build_report"], "sha256": verified["build_report_sha256"]})
        built = json.loads(Path(build_record["path"]).read_text(encoding="utf-8-sig"))
        if built.get("mode") != "build" or built.get("state") != "passed" or built.get("source_unchanged") is not True or built.get("source_sha256_after") != source_record["sha256"]:
            raise RuntimeGateError("linked real CAD build did not pass unchanged-source checks")
        cad_records = manifest.get("cad", [])
        if cad_records != plan.get("cad") or len(cad_records) != 2:
            raise RuntimeGateError("CAD->mesh input chain differs")
        for domain, cad in zip(("background", "arm"), cad_records):
            actual = _checked_record(cad)
            artifact = _checked_record(verified["artifacts"][domain])
            if actual != artifact:
                raise RuntimeGateError("scdoc meshed is not independently verified: " + domain)
            if built.get("domains", {}).get(domain, {}).get("state") != "passed":
                raise RuntimeGateError("missing built CAD domain: " + domain)
            result["evidence"].append(actual)
        result["evidence"] += [verification_record, build_record]
        root = Path(stage_root).resolve()
        if any(root not in Path(record["path"]).resolve().parents for record in result["evidence"]):
            raise RuntimeGateError("upstream generated evidence must stay inside the new stage15 directory")
        result["evidence"].append(source_record)
        result.update(status="passed", cad_contact="passed", cad_domains="passed",
                      reason="Actual independently verified CAD and exported mesh hashes match this diagnostic.",
                      scope="upstream_artifact_validation_not_motion", motion_authorized=False)
    except Exception as exc:
        result["reason"] = type(exc).__name__ + ": " + str(exc)
    return result


def demote_failed_case(attempt, report):
    """A late cleanup/hash/persistence failure must not leave a plausible base."""
    attempt = Path(attempt)
    diagnostic = attempt / "FAILED_DIAGNOSTIC_DO_NOT_USE_AS_BASE.cas.h5"
    for name in ("candidate_uninitialized.cas.h5", "base_uninitialized.cas.h5"):
        source = attempt / name
        if source.is_file():
            if diagnostic.exists():
                raise FileExistsError("cannot overwrite an existing failed diagnostic")
            source.rename(diagnostic)
    if diagnostic.is_file():
        report["diagnostic_case"] = file_record(diagnostic)
        write_json_new(attempt / "failure_case_role.json", {"status": "failed", "valid_base": False,
            "motion_authorized": False, "case": report["diagnostic_case"], "role": "failed_diagnostic_only"})


def run(args, *, launcher=launch, guard=prelaunch_guard, upstream_verifier=upstream_chain, stage_root=STAGE_ROOT):
    contract_path = Path(args.contract or STAGE_ROOT / "contract.json")
    contract = load_contract(contract_path)
    inputs = [contract_path]
    if args.gate == "static" and not args.api_only:
        if not args.mesh_background or not args.mesh_arm:
            raise RuntimeGateError("static gate requires two actual mesh paths")
        if Path(args.mesh_background).resolve() == Path(args.mesh_arm).resolve():
            raise RuntimeGateError("two independent meshes required")
        inputs += [Path(args.mesh_background), Path(args.mesh_arm)]
    if args.case:
        inputs.append(Path(args.case))
    if getattr(args, "upstream_evidence", None):
        inputs.append(Path(args.upstream_evidence))
    for path in inputs:
        file_record(path)
    attempt = create_attempt(args.attempt, stage_root=stage_root)
    frozen = snapshot_inputs(attempt, inputs=inputs, sources=[Path(__file__), STAGE_ROOT / "runtime.py", STAGE_ROOT / "audit.py", STAGE_ROOT / "preflight.py"],
                             settings={"gate": args.gate, "api_only": args.api_only, "processors": args.processors,
                                       "initializations_authorized": False, "iterations_authorized": False})
    report = {"status": "not_run", "valid": False, "gate": args.gate, "attempt": str(attempt),
              "stages": {name: "not_run" for name in contract["required_separate_statuses"]},
              "motion_authorized": False}
    try:
        if args.api_only:
            report["api_inventory"] = local_api_inventory(contract)
        elif args.gate == "motion":
            report.update(status="blocked", blocking_reason="Motion gate is NotImplemented until a real single-step probe confirms clock, reference point and update command.")
            report["stages"]["motion_api"] = "blocked"
        else:
            report["prelaunch"] = guard(contract, stage_root=stage_root)
            if report["prelaunch"].get("status") != "passed":
                report["stages"]["environment"] = "blocked"
                raise PrelaunchBlocked("protected input/environment/license/session guard did not pass")
            report["stages"]["environment"] = "passed"
            if args.gate == "static":
                report["upstream"] = upstream_verifier(getattr(args, "upstream_evidence", None),
                                                      args.mesh_background, args.mesh_arm, contract)
            with launcher(attempt, contract, processors=args.processors, stage_root=stage_root) as (solver, runtime_report):
                if args.gate == "static":
                    report["static"] = static_session(solver, attempt, args.mesh_background, args.mesh_arm, contract,
                                                      save_diagnostic=not args.no_diagnostic_case)
                else:
                    report["probe"] = live_api_probe(solver, args.case, attempt)
            for item in frozen["inputs"] + frozen["sources"]:
                if file_record(item["path"])["sha256"] != item["sha256"]:
                    raise RuntimeGateError("frozen input/source changed during execution: " + item["path"])
            if report["prelaunch"].get("frozen_manifest"):
                _checked_record(report["prelaunch"]["frozen_manifest"])
            for item in report["prelaunch"].get("frozen_inputs", {}).get("files", []):
                actual = file_record(STAGE_ROOT.parent.parent / item["path"])
                if actual["sha256"] != item["expected_sha256"]:
                    raise RuntimeGateError("protected stage15 input changed while Fluent ran")
            if args.gate == "static":
                for artifact in report["upstream"].get("evidence", []):
                    _checked_record(artifact)
                static = report["static"]
                if static["status"] == "candidate_pending_session_close" and report["upstream"].get("status") != "passed":
                    report.update(status="blocked", valid=False,
                                  blocking_reason="Static diagnostic passed, but CAD/contact/mesh upstream evidence is not verified; candidate only.")
                    report["stages"]["static_overset"] = "passed"
                    report["stages"]["mesh_quality"] = "passed"
                    report["candidate_only"] = file_record(static["base_candidate"])
                elif static["status"] == "candidate_pending_session_close":
                    source = Path(static["base_candidate"])
                    destination = attempt / "base_uninitialized.cas.h5"
                    if destination.exists():
                        raise FileExistsError("refusing base overwrite")
                    source.rename(destination)
                    report["base_case"] = file_record(destination)
                    write_json_new(attempt / "base_acceptance.json", {"status": "passed", "valid_static_base": True,
                                   "case": report["base_case"], "initialized": False, "flow_iterations": 0,
                                   "motion_authorized": False, "evidence": static["transcript"],
                                   "upstream": report["upstream"], "frozen_inputs": report["prelaunch"].get("frozen_manifest")})
                    report.update(status="passed", valid=True, validation_scope="static_base_only_no_motion_or_flow")
                    report["stages"]["cad_contact"] = "passed"
                    report["stages"]["cad_domains"] = "passed"
                    report["stages"]["static_overset"] = "passed"
                    report["stages"]["mesh_quality"] = "passed"
                else:
                    report["status"] = "failed"
                    report["stages"]["static_overset"] = "failed"
            else:
                report["status"] = report["probe"]["status"]
                report["stages"]["motion_api"] = "blocked"
    except PrelaunchBlocked as exc:
        report.update(status="blocked", valid=False, blocking_reason=str(exc))
    except Exception as exc:
        report.update(status="failed", valid=False, exception={"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()})
        if args.gate == "static":
            report["stages"]["static_overset"] = "failed"
    if report["status"] == "failed":
        try:
            demote_failed_case(attempt, report)
        except Exception as exc:
            report["diagnostic_marking_error"] = repr(exc)
    write_json_new(attempt / "gate_report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", choices=("static", "probe", "motion"), required=True)
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--contract")
    parser.add_argument("--mesh-background")
    parser.add_argument("--mesh-arm")
    parser.add_argument("--case")
    parser.add_argument("--upstream-evidence", help="actual mesh_manifest.json linking independent CAD verification")
    parser.add_argument("--processors", type=int, default=1)
    parser.add_argument("--api-only", action="store_true")
    parser.add_argument("--no-diagnostic-case", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run(args)
    except Exception as exc:
        print(type(exc).__name__ + ": " + str(exc), file=sys.stderr)
        return 2
    print(json.dumps({k: result[k] for k in ("status", "valid", "motion_authorized", "attempt")}, ensure_ascii=False))
    return 0 if result["status"] in ("passed", "not_run") else 2


if __name__ == "__main__":
    raise SystemExit(main())

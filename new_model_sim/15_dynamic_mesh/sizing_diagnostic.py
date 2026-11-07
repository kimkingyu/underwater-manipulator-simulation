"""Fixed-batch field diagnostics. Completion is not mesh or full-control acceptance."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
import psutil
import definition_probe as baseline
import preflight
import run_mesh_job
from mesh_prepare import domain_settings
from runtime import AnsysTaskLock, OwnedProcessTree, RuntimeGateError

STAGE = Path(__file__).resolve().parent
ATTEMPT_ROOT_RELATIVE = "runs/mcp_sizing_diagnostic/batch_001"
AUTHORIZATION_RELATIVE = "frozen/sizing_diagnostic_authorization_003.json"
PROFILES = ("baseline", "geometry_scope", "capture_off", "omit_local_growth", "background_all_controls")
UNSET_LOCAL_GROWTH_PROFILES = ("omit_local_growth", "background_all_controls")
BASELINE_AUTHORIZATION_SHA256 = "51dbdd6023ad9d9f0929d38fefcb4e86a6d906ec25ac18b7c00c2126ccbfbd1f"
SOURCE_RELATIVES = (
    "stage15_mcp.py", "mcp_bridge.py", "definition_probe.py", "sizing_diagnostic.py",
    "sizing_mcp_bridge.py", "runtime.py", "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "mesh/wb_definition_probe.wbjn.in", "mesh/meshing_definition_probe.py.in",
    "mesh/wb_sizing_diagnostic.wbjn.in", "mesh/meshing_sizing_diagnostic.py.in",
)
AUTHORIZATION_VALUES = {
    "schema_version": 1, "scope": "stage15_mcp_sizing_diagnostic", "operation": "sizing_diagnostic",
    "maximum_attempts": 3, "maximum_total_seconds": 300, "cleanup_reserve_seconds": 30,
    "attempt_root_relative": ATTEMPT_ROOT_RELATIVE, "job_id_prefix": "sizing-b001-a",
    "allowed_profiles": list(PROFILES),
    "baseline_authorization_relative": "frozen/mcp_definition_probe_authorization_001.json",
    "baseline_authorization_sha256": BASELINE_AUTHORIZATION_SHA256,
    "full_mesh_allowed": False, "fluent_allowed": False, "automatic_retry_allowed": False,
    "original_geometry_modification_allowed": False, "default_theta_modification_allowed": False,
    "license_service_modification_allowed": False,
}
AUTHORIZATION_METADATA = ("authorization_source", "recorded_local", "batch_policy")
SAFE_DEFAULTS = dict(mesh_generated=False, flow_iterations=0, mesh_motion_verified=False,
                     diagnostic_completed=False, zero_mesh_verified=False, control_fully_defined=False,
                     full_background_definitions_verified=False)
RECEIPTS = {phase: "sizing_" + phase + "_stats.json" for phase in ("define", "refresh", "readback")}
OBSERVED_FIELDS = {"Type", "ElementSize", "GrowthRate", "Location", "ScopingMethod", "NamedSelection", "Active",
                   "Suppressed", "Behavior", "CaptureCurvature", "CurvatureNormalAngle", "CaptureProximity",
                   "LocalMinimumSize", "ProximityMinimumSize", "ProximityGapFactor", "ProximitySizeSources", "DefeatureSize"}
checked_path = baseline.checked_path
load_json = baseline.load_json
digest = baseline.digest
write_new = baseline.write_new
write_state = baseline.write_state
bounded_call = baseline.bounded_call
UncertainCall = baseline.UncertainCall
ProbeCancelled = baseline.ProbeCancelled


def source_paths(stage_root=STAGE):
    stage_root = Path(stage_root)
    return [stage_root / name for name in SOURCE_RELATIVES] + [stage_root.parent / "04_workbench/run_wb_fullenv.py"]


def attempt_identity(attempt, stage_root=STAGE, *, must_exist=True):
    root = checked_path(stage_root, stage_root)
    if must_exist:
        attempt = checked_path(attempt, root)
    else:
        raw = Path(attempt)
        if ".." in raw.parts:
            raise RuntimeGateError("parent traversal forbidden")
        attempt = Path(os.path.abspath(raw))
        ancestor = attempt
        while not ancestor.exists() and not ancestor.is_symlink():
            ancestor = ancestor.parent
        checked_path(ancestor, root)
    for index in range(1, 4):
        if attempt == root / ATTEMPT_ROOT_RELATIVE / ("attempt_%03d" % index):
            return attempt, index, "sizing-b001-a%03d" % index
    raise RuntimeGateError("only fixed batch_001 attempt_001..003 allowed")


def validate_authorization(stage_root=STAGE):
    path = Path(stage_root) / AUTHORIZATION_RELATIVE
    value = load_json(path, stage_root, maximum=16384)
    if not isinstance(value, dict) or not set(AUTHORIZATION_VALUES).issubset(value) or set(value) - set(AUTHORIZATION_VALUES) - set(AUTHORIZATION_METADATA):
        raise RuntimeGateError("unexpected sizing authorization fields")
    for name, expected in AUTHORIZATION_VALUES.items():
        if type(value[name]) is not type(expected) or value[name] != expected:
            raise RuntimeGateError("sizing authorization mismatch: " + name)
    for name in set(value).intersection(AUTHORIZATION_METADATA):
        if not isinstance(value[name], str) or not 1 <= len(value[name]) <= 4096:
            raise RuntimeGateError("invalid bounded authorization metadata")
    old_path = Path(stage_root) / value["baseline_authorization_relative"]
    if digest(old_path, stage_root) != BASELINE_AUTHORIZATION_SHA256:
        raise RuntimeGateError("baseline authorization changed")
    return {"path": str(checked_path(path, stage_root)), "sha256": digest(path, stage_root)}


def validate_frozen(stage_root=STAGE):
    result = baseline.validate_frozen(stage_root)
    old = result["authorization"]
    if old["sha256"] != BASELINE_AUTHORIZATION_SHA256:
        raise RuntimeGateError("baseline grant is not the approved immutable grant")
    result["baseline_authorization"] = old
    result["authorization"] = validate_authorization(stage_root)
    return result


def validate_sources(rows, stage_root=STAGE):
    expected = {str(checked_path(path, Path(stage_root).parent)).casefold() for path in source_paths(stage_root)}
    if not isinstance(rows, list) or len(rows) != 14 or len(expected) != 14:
        raise RuntimeGateError("exactly fourteen frozen sources required")
    seen = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"path", "size", "sha256"}:
            raise RuntimeGateError("invalid sizing source record")
        path = checked_path(row["path"], Path(stage_root).parent)
        key = str(path).casefold()
        if (key not in expected or key in seen or type(row["size"]) is not int or
                not 0 <= row["size"] <= 2 * 1024 * 1024 or not baseline._hash(row["sha256"]) or
                path.stat().st_size != row["size"] or digest(path, Path(stage_root).parent) != row["sha256"]):
            raise RuntimeGateError("unapproved, duplicate or changed frozen source")
        seen.add(key)
    return rows


def validate_request(attempt, stage_root=STAGE):
    attempt, index, job_id = attempt_identity(attempt, stage_root)
    value = load_json(attempt / "request.json", attempt, maximum=65536)
    fields = {"schema_version", "operation", "job_id", "attempt_index", "profile", "hypothesis",
              "created_unix", "deadline_unix", "work_deadline_unix", "authorization", "sources"}
    if (not isinstance(value, dict) or set(value) != fields or type(value["schema_version"]) is not int or
            value["schema_version"] != 1 or value["operation"] != "sizing_diagnostic" or
            value["job_id"] != job_id or type(value["attempt_index"]) is not int or value["attempt_index"] != index or
            value["profile"] not in PROFILES or not isinstance(value["hypothesis"], str) or
            not value["hypothesis"].strip() or len(value["hypothesis"]) > 2000):
        raise RuntimeGateError("invalid fixed sizing diagnostic request")
    for name in ("created_unix", "deadline_unix", "work_deadline_unix"):
        if type(value[name]) not in (int, float) or not math.isfinite(value[name]):
            raise RuntimeGateError("invalid finite diagnostic deadline")
    created = value["created_unix"]
    if (value["deadline_unix"] != created + 300 or value["work_deadline_unix"] != created + 270 or
            created > time.time() + 1 or value["deadline_unix"] <= time.time()):
        raise RuntimeGateError("diagnostic total/work budget mismatch or expired")
    if value["authorization"] != validate_authorization(stage_root):
        raise RuntimeGateError("request must bind the new sizing authorization")
    validate_sources(value["sources"], stage_root)
    return value


def validate_launcher(attempt, request, remaining):
    end = time.monotonic() + min(20, remaining())
    while time.monotonic() < end:
        path = checked_path(attempt / "launcher.json", attempt, must_exist=False)
        if path.exists():
            row = load_json(path, attempt, maximum=8192)
            stamp = bounded_call(lambda: psutil.Process(os.getpid()).create_time(), min(2, remaining()))
            if (not isinstance(row, dict) or set(row) != {"job_id", "pid", "creation_time", "recorded_unix"} or
                    row["job_id"] != request["job_id"] or type(row["pid"]) is not int or row["pid"] != os.getpid() or
                    type(row["creation_time"]) not in (int, float) or row["creation_time"] != stamp or
                    type(row["recorded_unix"]) not in (int, float) or not math.isfinite(row["recorded_unix"]) or
                    not request["created_unix"] <= row["recorded_unix"] <= min(time.time() + 1, request["created_unix"] + 15)):
                raise RuntimeGateError("worker launcher PID/create-time ledger mismatch")
            return row
        time.sleep(min(0.05, max(0, end - time.monotonic())))
    raise RuntimeGateError("launcher ownership ledger absent after twenty-second wait")


def check_cancel(attempt, request):
    path = checked_path(attempt / "cancel_request.json", attempt, must_exist=False)
    if path.exists():
        value = load_json(path, attempt, maximum=8192)
        if (not isinstance(value, dict) or set(value) != {"job_id", "operation", "created_unix"} or
                value["job_id"] != request["job_id"] or value["operation"] != "cancel_sizing_diagnostic" or
                type(value["created_unix"]) not in (int, float) or not math.isfinite(value["created_unix"]) or
                not request["created_unix"] <= value["created_unix"] <= time.time() + 1):
            raise RuntimeGateError("invalid fixed cancellation request")
        raise ProbeCancelled("sizing diagnostic cancellation requested")


def audit_template(text, *, journal=False):
    tree = ast.parse(text)
    if re.search(r"generatemesh|\bfluent\b|\bsolver\b|\bsolve\b|\biterate\b|\bconvert(?:mesh|tomesh)\w*|InternalObject|InternalValue|Property\.Value", text, re.I):
        raise RuntimeGateError("forbidden native diagnostic path")
    saves = updates = edits = sends = refreshes = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store) and isinstance(node.value, ast.Name) and node.value.id == "C":
            raise RuntimeGateError("configuration mutation forbidden")
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            modules = [item.name for item in node.names] if isinstance(node, ast.Import) else [node.module]
            if any(item not in {"System", "System.IO", "System.Text", "json", "codecs", "os", "traceback"} for item in modules):
                raise RuntimeGateError("unapproved diagnostic import")
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Name):
            if func.id in {"eval", "exec", "compile", "__import__", "Open", "RunScript"}:
                raise RuntimeGateError("dynamic execution forbidden")
            if func.id == "Save":
                args = {key.arg: key.value for key in node.keywords}
                if (not journal or node.args or set(args) != {"FilePath", "Overwrite"} or
                        ast.dump(args["FilePath"]) != ast.dump(ast.parse('C["project"]', mode="eval").body) or
                        not isinstance(args["Overwrite"], ast.Constant) or args["Overwrite"].value is not False):
                    raise RuntimeGateError("only one fixed new-project non-overwriting Save allowed")
                saves += 1
        if not isinstance(func, ast.Attribute):
            continue
        if func.attr in {"system", "popen", "Popen", "Start", "execv", "spawnv", "Save"}:
            raise RuntimeGateError("unapproved native process/save")
        receiver = ast.unparse(func.value)
        if func.attr == "Update":
            if (not journal or receiver != "geometry_component" or node.args or len(node.keywords) != 1 or
                    node.keywords[0].arg != "AllDependencies" or not isinstance(node.keywords[0].value, ast.Constant) or
                    node.keywords[0].value.value is not False):
                raise RuntimeGateError("only isolated Geometry.Update allowed")
            updates += 1
        if func.attr == "Refresh":
            allowed_receiver = "mesh_component" if journal else "ExtAPI.DataModel.Tree"
            if receiver != allowed_receiver or node.args or node.keywords:
                raise RuntimeGateError("only the fixed mesh component/native tree Refresh allowed")
            refreshes += 1
        if func.attr in {"Edit", "SendCommand", "Exit"}:
            if not journal or receiver != "mesh_container":
                raise RuntimeGateError("unapproved editor operation")
            edits += func.attr == "Edit"
            sends += func.attr == "SendCommand"
    if journal and (saves, updates, edits, sends, refreshes) != (1, 1, 1, 2, 1):
        raise RuntimeGateError("journal requires one geometry update/save/mesh refresh/edit and two commands")
    if not journal and (saves or updates or edits or sends or refreshes != 1):
        raise RuntimeGateError("observation script requires exactly one tree Refresh site")
    return True


def render_scripts(attempt, stage_root, frozen, profile="baseline", job_id=None):
    attempt, index, fixed_id = attempt_identity(attempt, stage_root, must_exist=False)
    if profile not in PROFILES or job_id not in (None, fixed_id):
        raise RuntimeGateError("unapproved profile/job id")
    settings = domain_settings(frozen["contract"], attempt)[0]
    settings.update(job_id=fixed_id, profile=profile,
                    scope="background_all_required_controls" if profile == "background_all_controls" else "background_tet_and_wall_base0_only",
                    local_size_m=next(row["size_m"] for row in settings["face_controls"] if row["name"] == "wall_base0"))
    for phase, name in RECEIPTS.items():
        settings[phase + "_path"] = str(attempt / name)
    template = checked_path(Path(stage_root) / "mesh/meshing_sizing_diagnostic.py.in", stage_root).read_text(encoding="utf-8")
    journal_template = checked_path(Path(stage_root) / "mesh/wb_sizing_diagnostic.wbjn.in", stage_root).read_text(encoding="utf-8")
    if template.count("__CONFIG_JSON__") != 1 or journal_template.count("__CONFIG_JSON__") != 1:
        raise RuntimeGateError("exactly one template placeholder required")
    audit_template(template)
    audit_template(journal_template, journal=True)
    scripts = {}
    for phase in ("define", "readback"):
        config = dict(settings, phase=phase)
        script = template.replace("__CONFIG_JSON__", repr(json.dumps(config, ensure_ascii=True, allow_nan=False)))
        audit_template(script)
        scripts[phase + "_script"] = script
    config = dict(scripts, job_id=fixed_id, profile=profile, scope=settings["scope"], cad_path=frozen["cad"]["background"]["path"],
                  project=str(attempt / "diagnostic.wbpj"), trace=str(attempt / "workbench.trace.txt"),
                  completion=str(attempt / "workbench_completion.json"))
    config.update({phase + "_path": settings[phase + "_path"] for phase in RECEIPTS})
    journal = journal_template.replace("__CONFIG_JSON__", repr(json.dumps(config, ensure_ascii=True, allow_nan=False)))
    audit_template(journal, journal=True)
    return dict(scripts, journal_script=journal, configuration=config, settings=settings)


def prepare_scripts(attempt, stage_root, frozen, request):
    rendered = render_scripts(attempt, stage_root, frozen, request["profile"], request["job_id"])
    for name, key in (("meshing_define.py", "define_script"), ("meshing_readback.py", "readback_script"), ("sizing_diagnostic.wbjn", "journal_script")):
        path = checked_path(attempt / name, attempt, must_exist=False)
        with path.open("x", encoding="utf-8") as stream:
            stream.write(rendered[key])
    return attempt / "sizing_diagnostic.wbjn"


def native_zero(value):
    return (isinstance(value, dict) and set(value) == {"nodes", "elements"} and
            all(type(value[name]) is int and value[name] == 0 for name in ("nodes", "elements")))


def quantity_matches(value, expected):
    factors = {"m": 1.0, "mm": 0.001, "cm": 0.01, "um": 0.000001, "µm": 0.000001}
    return (isinstance(value, dict) and type(value.get("value")) in (int, float) and
            math.isfinite(value["value"]) and value.get("unit") in factors and
            math.isclose(value["value"] * factors[value["unit"]], expected, rel_tol=1e-7, abs_tol=1e-10))


def background_definitions_match(value, settings):
    """Assess captured definitions, not discretization or motion; raw quantities stay untouched."""
    rows = value.get("background_controls")
    expected_names = {"stage15_background_tet", "stage15_wall_base0", "stage15_wall_auv", "stage15_overlap_0"}
    if (not isinstance(rows, list) or len(rows) != 4 or any(not isinstance(row, dict) for row in rows) or
            {row.get("name") for row in rows} != expected_names):
        raise RuntimeGateError("exact four background control observations required")
    named = value.get("named_scopes")
    body_ids = value.get("body_geometry_ids")
    face_ids = value.get("body_face_ids")
    def ids_valid(ids, *, nonempty=True):
        return (isinstance(ids, list) and (bool(ids) or not nonempty) and
                all(type(item) is int for item in ids) and len(ids) == len(set(ids)))
    if (not isinstance(named, dict) or set(named) != set(settings["required_selections"]) or len(named) != 5 or
            not ids_valid(body_ids) or len(body_ids) != 1 or not ids_valid(face_ids)):
        raise RuntimeGateError("complete body and five named scope observations required")
    matched = value.get("control_fully_defined") is True and value.get("tet_object_state") == "FullyDefined"
    used = set()
    for ids in named.values():
        if not ids_valid(ids):
            raise RuntimeGateError("invalid native named scope observation")
        matched = matched and set(ids).issubset(face_ids) and not bool(used.intersection(ids))
        used.update(ids)
    configs = {"stage15_" + row["name"]: row for row in settings["face_controls"]}
    sphere = settings["spheres"][0]
    for row in rows:
        try:
            state = row["object_state"]
            expected = row["expected_geometry_ids"]
            actual = row["actual_geometry_ids"]
            if not isinstance(state, str) or not state or not ids_valid(expected) or not ids_valid(actual, nonempty=False):
                raise RuntimeGateError("missing native background state or scope")
            matched = matched and state == "FullyDefined" and sorted(actual) == sorted(expected)
            if row["name"] == "stage15_background_tet":
                matched = matched and row["kind"] == "tet_method" and sorted(expected) == sorted(body_ids) and row["method"].rsplit(".", 1)[-1] == "AllTriAllTet"
            elif row["name"] in configs:
                config = configs[row["name"]]
                growth = row["local_growth_raw"]
                matched = (matched and row["kind"] == "face_size" and row["source_named_selection"] == config["name"] and
                           sorted(expected) == sorted(named[config["name"]]) and row["type"].rsplit(".", 1)[-1] == "ElementSize" and
                           row["expected_element_size_m"] == config["size_m"] and quantity_matches(row["element_size"], config["size_m"]) and
                           row["local_growth_override_applied"] is False and growth.get("available") is True and
                           type(growth.get("value")) in (int, float) and growth["value"] == 0.0)
            else:
                coordinate = row["coordinate_system"]
                origins = coordinate["origins"]
                matched = (matched and row["kind"] == "sphere" and sorted(expected) == sorted(body_ids) and
                           row["type"].rsplit(".", 1)[-1] == "SphereOfInfluence" and
                           row["expected_element_size_m"] == sphere["size_m"] and quantity_matches(row["element_size"], sphere["size_m"]) and
                           row["expected_sphere_radius_m"] == sphere["radius_m"] and quantity_matches(row["sphere_radius"], sphere["radius_m"]) and
                           row["expected_sphere_center_m"] == sphere["centre_m"] and coordinate["name"] == row["name"] and
                           coordinate["object_state"] == "FullyDefined" and type(coordinate["object_id"]) is int and
                           type(row["sphere_center_object_id"]) is int and coordinate["object_id"] == row["sphere_center_object_id"] and
                           set(origins) == {"OriginX", "OriginY", "OriginZ"} and
                           all(quantity_matches(origins[key], number) for key, number in zip(("OriginX", "OriginY", "OriginZ"), sphere["centre_m"])))
        except (KeyError, TypeError, AttributeError, IndexError) as exc:
            raise RuntimeGateError("incomplete native background control observation") from exc
    base_ids = value["control"]["fields"]["Location"]["value"]
    return bool(matched and sorted(base_ids) == sorted(named["wall_base0"]))


def validate_messages(value):
    rows = value.get("messages")
    complete = value.get("message_capture_complete")
    truncated = value.get("messages_truncated")
    error = value.get("message_capture_error")
    present = value.get("native_errors_present")
    used = value.get("message_characters")
    if (not isinstance(rows, list) or len(rows) > 100 or type(complete) is not bool or type(truncated) is not bool or
            type(used) is not int or not 0 <= used <= 20000 or
            not (error is None or isinstance(error, str) and 1 <= len(error) <= 2000) or
            not (present is None or type(present) is bool)):
        raise RuntimeGateError("invalid bounded native message capture")
    for row in rows:
        if (not isinstance(row, dict) or set(row) != {"severity", "text"} or
                not isinstance(row["severity"], str) or not isinstance(row["text"], str)):
            raise RuntimeGateError("invalid native message row")
    if used != sum(len(row["severity"]) + len(row["text"]) for row in rows):
        raise RuntimeGateError("native message character budget mismatch")
    if complete:
        detected = any(row["severity"].lower().rsplit(".", 1)[-1] not in ("info", "information", "warning") for row in rows)
        if truncated or error is not None or type(present) is not bool or present != detected:
            raise RuntimeGateError("native message completeness/error semantics mismatch")
    elif present is False or not (truncated or error is not None):
        raise RuntimeGateError("incomplete capture cannot claim no native errors")


def validate_receipt(value, phase, request, settings=None):
    if (not isinstance(value, dict) or type(value.get("schema_version")) is not int or value.get("schema_version") != 1 or value.get("job_id") != request["job_id"] or
            value.get("profile") != request["profile"] or value.get("phase") != phase or value.get("status") != "completed" or
            value.get("diagnostic_completed") is not True or value.get("zero_mesh_verified") is not True or
            not native_zero(value.get("mesh_counts")) or
            value.get("mesh_generated") is not False or type(value.get("flow_iterations")) is not int or
            value["flow_iterations"] != 0 or value.get("mesh_motion_verified") is not False or
            value.get("scope") != ("background_all_required_controls" if request["profile"] == "background_all_controls" else "background_tet_and_wall_base0_only")):
        raise RuntimeGateError("invalid native zero-mesh sizing receipt: " + phase)
    validate_messages(value)
    rows = value.get("snapshots")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 10 or value.get("control") != rows[-1]:
        raise RuntimeGateError("missing bounded native snapshots")
    for row in rows:
        if (not isinstance(row, dict) or not isinstance(row.get("object_state"), str) or not row["object_state"] or
                row.get("name") != "stage15_wall_base0" or not native_zero(row.get("mesh_counts"))):
            raise RuntimeGateError("native snapshot/count/state missing")
        observed = row.get("fields")
        if not isinstance(observed, dict) or set(observed) != OBSERVED_FIELDS:
            raise RuntimeGateError("exact bounded public field set required")
        for observation in observed.values():
            if not isinstance(observation, dict) or type(observation.get("available")) is not bool:
                raise RuntimeGateError("field availability must be explicit")
            if observation["available"]:
                if set(observation) != {"available", "value"}:
                    raise RuntimeGateError("invalid available field receipt")
            elif (set(observation) != {"available", "error"} or not isinstance(observation["error"], str) or
                    not observation["error"] or len(observation["error"]) > 2000):
                raise RuntimeGateError("unavailable field must retain bounded native error")
    control = value["control"]
    fields = control.get("fields", {})
    for name in ("Location", "Type", "ElementSize", "GrowthRate"):
        if not isinstance(fields.get(name), dict) or fields[name].get("available") is not True or "value" not in fields[name]:
            raise RuntimeGateError("core sizing field unavailable: " + name)
    ids = fields["Location"]["value"]
    size = fields["ElementSize"]["value"]
    growth = fields["GrowthRate"]["value"]
    if (not isinstance(ids, list) or not ids or any(type(item) is not int for item in ids) or len(set(ids)) != len(ids) or
            not isinstance(size, dict) or type(size.get("value")) not in (int, float) or not math.isfinite(size["value"]) or
            not isinstance(size.get("unit"), str) or not size["unit"] or
            type(growth) not in (int, float) or not math.isfinite(growth) or
            not isinstance(fields["Type"]["value"], str) or fields["Type"]["value"].rsplit(".", 1)[-1] != "ElementSize"):
        raise RuntimeGateError("invalid core sizing readback")
    override_applied = request["profile"] not in UNSET_LOCAL_GROWTH_PROFILES
    if type(value.get("local_growth_override_applied")) is not bool or value["local_growth_override_applied"] is not override_applied:
        raise RuntimeGateError("local growth override provenance mismatch")
    expected_growth = (settings["growth_rate"] if settings is not None else 1.3) if override_applied else 0.0
    if growth != expected_growth:
        raise RuntimeGateError("local growth raw readback differs from profile expectation")
    if phase == "define":
        steps = [row.get("step") for row in rows]
        expected_step = "set_growth_rate" if override_applied else "leave_local_growth_unset"
        opposite_step = "leave_local_growth_unset" if override_applied else "set_growth_rate"
        if expected_step not in steps or opposite_step in steps:
            raise RuntimeGateError("local growth intervention step mismatch")
    global_fields = value.get("global_readback")
    global_names = {"ElementSize", "MaximumSize", "MinimumSize", "GrowthRate", "CaptureCurvature", "CaptureProximity"}
    if not isinstance(global_fields, dict) or set(global_fields) != global_names:
        raise RuntimeGateError("global readback required")
    for observation in global_fields.values():
        if not isinstance(observation, dict) or observation.get("available") is not True or "value" not in observation:
            raise RuntimeGateError("global readback unavailable")
    if global_fields["CaptureCurvature"]["value"] is not False or global_fields["CaptureProximity"]["value"] is not False:
        raise RuntimeGateError("global capture settings changed")
    if settings is not None:
        factors = {"m": 1.0, "mm": 0.001, "cm": 0.01, "um": 0.000001, "µm": 0.000001}
        if size["unit"] not in factors or not math.isclose(size["value"] * factors[size["unit"]], settings["local_size_m"], rel_tol=1e-7, abs_tol=1e-10) or growth != expected_growth:
            raise RuntimeGateError("physical local sizing/growth changed")
        for name, key in (("ElementSize", "global_size_m"), ("MaximumSize", "maximum_size_m"), ("MinimumSize", "minimum_size_m")):
            actual = global_fields[name]["value"]
            if (not isinstance(actual, dict) or type(actual.get("value")) not in (int, float) or
                    not math.isfinite(actual["value"]) or actual.get("unit") not in factors or
                    not math.isclose(actual["value"] * factors[actual["unit"]], settings[key], rel_tol=1e-7, abs_tol=1e-10)):
                raise RuntimeGateError("physical global setting changed: " + name)
        if global_fields["GrowthRate"]["value"] != settings["growth_rate"]:
            raise RuntimeGateError("global growth rate changed")
    if type(value.get("control_fully_defined")) is not bool or value["control_fully_defined"] != (control["object_state"] == "FullyDefined"):
        raise RuntimeGateError("control definition semantics mismatch")
    if request["profile"] == "background_all_controls" and settings is not None:
        background_definitions_match(value, settings)
    return value


def observe_receipts(attempt, request, *, required=False, settings=None):
    result = {}
    for phase, name in RECEIPTS.items():
        path = checked_path(attempt / name, attempt, must_exist=False)
        if path.exists():
            result[phase] = validate_receipt(load_json(path, attempt, maximum=262144), phase, request, settings)
        elif required:
            raise RuntimeGateError("missing native sizing receipt: " + phase)
    completion = None
    path = checked_path(attempt / "workbench_completion.json", attempt, must_exist=False)
    if path.exists():
        completion = load_json(path, attempt, maximum=65536)
        if (completion.get("job_id") != request["job_id"] or completion.get("profile") != request["profile"] or
                completion.get("status") != "completed" or completion.get("journal_completed") is not True or
                completion.get("diagnostic_completed") is not True or completion.get("mesh_generated") is not False or
                completion.get("flow_iterations") != 0 or completion.get("mesh_motion_verified") is not False):
            raise RuntimeGateError("Workbench diagnostic did not complete")
    elif required:
        raise RuntimeGateError("missing Workbench completion")
    if required and len({tuple(result[phase]["control"]["fields"]["Location"]["value"]) for phase in result}) != 1:
        raise RuntimeGateError("actual geometry scope changed across observations")
    return result, completion


def inspect(stage_root=STAGE):
    result = dict(SAFE_DEFAULTS, operation="sizing_diagnostic", status="blocked", blockers=[], ansys_launched=False)
    try:
        contract = preflight.load_contract(Path(stage_root) / "contract.json")
        environment = preflight.inspect_environment(contract, root=stage_root)
        result["environment"] = environment
        result["blockers"].extend(environment.get("blockers", []))
        frozen = validate_frozen(stage_root)
        result.update({key: value for key, value in frozen.items() if key != "contract"})
        audits = []
        for profile in PROFILES:
            rendered = render_scripts(Path(stage_root) / ATTEMPT_ROOT_RELATIVE / "attempt_001", stage_root, frozen, profile)
            audits.append({"profile": profile, "journal_sha256": hashlib.sha256(rendered["journal_script"].encode("utf-8")).hexdigest()})
        result["template_preflight"] = dict(status="passed", rendered_in_memory=True, files_written=False, profiles=audits)
        if checked_path(Path(stage_root) / ".ansys-task.lock", stage_root, must_exist=False).exists():
            result["blockers"].append("ANSYS task lock exists")
        if environment.get("status") != "passed" and not result["blockers"]:
            result["blockers"].append("environment did not pass")
        result["status"] = "blocked" if result["blockers"] else "passed"
    except Exception as exc:
        result["blockers"].append(type(exc).__name__ + ": " + str(exc))
    return result


def run(attempt, *, stage_root=STAGE):
    attempt, index, job_id = attempt_identity(attempt, stage_root)
    if not attempt.is_dir() or checked_path(attempt / "probe_result.json", attempt, must_exist=False).exists():
        raise RuntimeGateError("requires fresh bridge-reserved attempt; rerun forbidden")
    write_new(attempt / "worker_claim.json", dict(job_id=job_id, pid=os.getpid(), created_unix=time.time()), attempt)
    began = time.monotonic()
    deadline = began + 300
    work_deadline = began + 270
    request = frozen = process = tree = lock = None
    uncertain = False
    report = dict(SAFE_DEFAULTS, operation="sizing_diagnostic", job_id=job_id, attempt_index=index,
                  profile=None, status="running", phase="validating", error=None,
                  owned_cleanup_verified=False, ansys_launched=False,
                  maximum_total_seconds=300, cleanup_reserve_seconds=30,
                  scope="background_tet_and_wall_base0_only", full_mesh_allowed=False, fluent_allowed=False,
                  receipt_paths={phase: str(attempt / name) for phase, name in RECEIPTS.items()}, receipts={},
                  native_errors_present=None, message_capture_complete=False, messages_truncated=False,
                  local_growth_override_applied=None)

    def remaining():
        return max(0, min(deadline - time.monotonic(), request["deadline_unix"] - time.time() if request else 300))

    def working():
        if request:
            check_cancel(attempt, request)
        if time.monotonic() >= work_deadline or (request and time.time() >= request["work_deadline_unix"]):
            raise TimeoutError("270-second work limit reached; owned cleanup begins")

    def state(phase):
        report.update(phase=phase, elapsed_seconds=max(time.monotonic() - began, 300 - remaining()))
        write_state(attempt, report)

    def recheck(timeout):
        # Always hash real files again; never reuse the pre-launch result as final evidence.
        def verify():
            actual = validate_frozen(stage_root)
            validate_sources(request["sources"], stage_root)
            if actual["authorization"] != request["authorization"]:
                raise RuntimeGateError("sizing authorization changed")
            return actual
        return bounded_call(verify, timeout)

    try:
        request = validate_request(attempt, stage_root)
        deadline = began + max(0, request["deadline_unix"] - time.time())
        work_deadline = deadline - 30
        report.update({key: request[key] for key in ("profile", "created_unix", "deadline_unix", "work_deadline_unix", "hypothesis", "sources", "authorization")})
        report["scope"] = "background_all_required_controls" if request["profile"] == "background_all_controls" else "background_tet_and_wall_base0_only"
        report["local_growth_override_applied"] = request["profile"] not in UNSET_LOCAL_GROWTH_PROFILES
        report["local_growth_readback_semantics"] = "explicit_local_override" if report["local_growth_override_applied"] else "raw_local_default_no_physical_or_inheritance_claim"
        validate_launcher(attempt, request, remaining)
        working()
        frozen = recheck(min(15, remaining() - 30))
        report.update(protected_inputs_before=frozen["protected_inputs"], cad=frozen["cad"], baseline_authorization=frozen["baseline_authorization"])
        state("preflight")
        environment = bounded_call(lambda: preflight.inspect_environment(frozen["contract"], root=stage_root), min(40, remaining() - 30))
        if environment.get("status") != "passed":
            raise RuntimeGateError("Workbench preflight blocked: " + repr(environment.get("blockers")))
        working()
        lock = AnsysTaskLock(attempt, stage_root=stage_root)
        lock.__enter__()
        journal = prepare_scripts(attempt, stage_root, frozen, request)
        env = bounded_call(lambda: run_mesh_job.child_environment(attempt, frozen["contract"]), min(10, remaining() - 30))
        for name in ("workbench.stdout.log", "workbench.stderr.log", "workbench.trace.txt", "workbench_completion.json", "workbench_completion.json.tmp", "diagnostic.wbpj") + tuple(RECEIPTS.values()) + tuple(name + ".tmp" for name in RECEIPTS.values()):
            if checked_path(attempt / name, attempt, must_exist=False).exists():
                raise RuntimeGateError("refusing existing diagnostic output: " + name)
        recheck(min(10, remaining() - 30))
        working()
        command = [frozen["contract"]["software"]["workbench"], "-I", "-X", "-R", str(journal)]
        state("starting_workbench")
        with (attempt / "workbench.stdout.log").open("xb") as stdout, (attempt / "workbench.stderr.log").open("xb") as stderr:
            try:
                process = bounded_call(lambda: subprocess.Popen(command, cwd=str(attempt), env=env, stdout=stdout, stderr=stderr, shell=False), min(10, remaining() - 30))
            except UncertainCall:
                uncertain = True
                raise
            report.update(ansys_launched=True, launcher_pid=process.pid)
            try:
                tree = bounded_call(lambda: OwnedProcessTree(process, psutil), min(3, remaining() - 30))
            except Exception:
                uncertain = True
                raise
            if tree.errors:
                raise RuntimeGateError("owned process tree cannot be established: " + repr(tree.errors))
            state("native_observations")
            while True:
                working()
                bounded_call(tree.capture, min(2, remaining() - 30))
                receipts, completion = observe_receipts(attempt, request)
                report["observed_phases"] = list(receipts)
                report["receipts"] = receipts
                if shutil.disk_usage(attempt).free < frozen["contract"]["budget"]["minimum_free_disk_gib"] * 1024 ** 3:
                    raise RuntimeGateError("disk reserve exhausted")
                code = bounded_call(process.poll, min(2, remaining() - 30))
                if code is not None:
                    report["exit_code"] = code
                    if code != 0:
                        raise RuntimeGateError("Workbench exit code " + str(code))
                    break
                time.sleep(min(0.1, max(0, work_deadline - time.monotonic())))
        settings = domain_settings(frozen["contract"], attempt)[0]
        settings["local_size_m"] = next(row["size_m"] for row in settings["face_controls"] if row["name"] == "wall_base0")
        receipts, completion = observe_receipts(attempt, request, required=True, settings=settings)
        if request["profile"] == "background_all_controls":
            report["background_definition_matches"] = {phase: background_definitions_match(value, settings) for phase, value in receipts.items()}
            first = receipts["define"]
            report["background_scopes_stable"] = all(
                value["named_scopes"] == first["named_scopes"] and value["body_geometry_ids"] == first["body_geometry_ids"] and
                value["body_face_ids"] == first["body_face_ids"] for value in receipts.values())
        report.update(receipts=receipts, receipt_paths={phase: str(attempt / name) for phase, name in RECEIPTS.items()},
                      completion=completion, control_fully_defined=receipts["readback"]["control_fully_defined"],
                      diagnostic_completed=True, zero_mesh_verified=True)
        working()
        report["protected_inputs_after"] = recheck(min(10, remaining() - 30))["protected_inputs"]
        report["status"] = "completed"
    except ProbeCancelled as exc:
        report.update(status="cancelled", error=str(exc))
    except Exception as exc:
        if isinstance(exc, UncertainCall) and process is not None:
            uncertain = True
        report.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
    finally:
        try:
            state("cleanup")
        except Exception as exc:
            report.update(status="failed", state_write_error=str(exc))
        cleanup = not uncertain
        if tree is not None:
            try:
                if remaining() < 15:
                    raise UncertainCall("insufficient owned-cleanup reserve")
                cleanup = bool(bounded_call(tree.close, min(20, remaining() - 1))) and cleanup
            except Exception as exc:
                cleanup = False
                report["cleanup_error"] = type(exc).__name__ + ": " + str(exc)
            report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
            report["ownership_errors"] = list(tree.errors)
        elif process is not None:
            cleanup = False
        if request is not None:
            try:
                after = recheck(min(5, remaining() - 1))
                report["protected_inputs_after"] = after["protected_inputs"]
                report["sources_reverified_after_cleanup"] = True
                report["inputs_reverified_after_cleanup"] = True
            except Exception as exc:
                report.update(status="failed", input_recheck_error=type(exc).__name__ + ": " + str(exc),
                              sources_reverified_after_cleanup=False, inputs_reverified_after_cleanup=False)
        if lock is not None and lock.acquired:
            if not cleanup:
                lock.retain("sizing diagnostic ownership or cleanup unverified; no automatic retry")
            try:
                lock.__exit__(None, None, None)
            except Exception as exc:
                cleanup = False
                lock.retain(str(exc))
                report["lock_error"] = str(exc)
        if not cleanup:
            report["status"] = "cleanup_unverified"
        if remaining() <= 0:
            report.update(status="cleanup_unverified" if not cleanup else "failed", deadline_exceeded=True)
        report["phase_summaries"] = {
            phase: {"path": report["receipt_paths"][phase],
                    "status": report["receipts"].get(phase, {}).get("status", "not_observed"),
                    "zero_mesh_verified": report["receipts"].get(phase, {}).get("zero_mesh_verified", False),
                    "control_fully_defined": report["receipts"].get(phase, {}).get("control_fully_defined", False),
                    "required_background_definitions_match": report.get("background_definition_matches", {}).get(phase, False),
                    "local_growth_override_applied": report["receipts"].get(phase, {}).get("local_growth_override_applied"),
                    "native_errors_present": report["receipts"].get(phase, {}).get("native_errors_present"),
                    "message_capture_complete": report["receipts"].get(phase, {}).get("message_capture_complete", False),
                    "messages_truncated": report["receipts"].get(phase, {}).get("messages_truncated", False)}
            for phase in RECEIPTS}
        flags = [value.get("native_errors_present") for value in report["receipts"].values()]
        report["native_errors_present"] = True if any(flag is True for flag in flags) else (False if len(flags) == 3 and all(flag is False for flag in flags) else None)
        report["message_capture_complete"] = len(flags) == 3 and all(value.get("message_capture_complete") is True for value in report["receipts"].values())
        report["messages_truncated"] = any(value.get("messages_truncated") is True for value in report["receipts"].values())
        report["full_background_definitions_verified"] = bool(
            request is not None and request["profile"] == "background_all_controls" and report["status"] == "completed" and
            report.get("diagnostic_completed") is True and report.get("zero_mesh_verified") is True and cleanup and
            report.get("inputs_reverified_after_cleanup") is True and report.get("sources_reverified_after_cleanup") is True and
            report.get("message_capture_complete") is True and report.get("native_errors_present") is False and
            report.get("background_scopes_stable") is True and len(report.get("background_definition_matches", {})) == 3 and
            all(report["background_definition_matches"].values()))
        report.update(owned_cleanup_verified=cleanup, phase="finished", elapsed_seconds=max(time.monotonic() - began, 300 - remaining()))
        try:
            write_state(attempt, report)
        finally:
            write_new(attempt / "probe_result.json", report, attempt)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--run", action="store_true")
    parser.add_argument("--attempt", type=Path)
    args = parser.parse_args(argv)
    if args.preflight and args.attempt is not None:
        parser.error("--preflight does not accept --attempt")
    if args.run and args.attempt is None:
        parser.error("--run requires the bridge-reserved --attempt")
    result = inspect() if args.preflight else run(args.attempt)
    print(json.dumps(result, ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if result["status"] in ("passed", "completed") else 2


if __name__ == "__main__":
    raise SystemExit(main())

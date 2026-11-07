"""Fixed, single-use Workbench definition probe; never a mesh/solver runner."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import queue
import re
import shutil
import stat
import subprocess
import sys
import threading
import time
from pathlib import Path

# The read-only CLI must not populate bytecode caches in imported stage files.
sys.dont_write_bytecode = True

import psutil
import preflight
import run_mesh_job
from mesh_prepare import domain_settings
from runtime import AnsysTaskLock, OwnedProcessTree, RuntimeGateError

STAGE = Path(__file__).resolve().parent
JOB_ID = "definition-probe-001"
LAUNCHER_WAIT_SECONDS = 20
ATTEMPT_RELATIVE = "runs/mcp_definition_probe/attempt_001"
AUTHORIZATION_RELATIVE = "frozen/mcp_definition_probe_authorization_001.json"
BACKGROUND_RELATIVE = "runs/cad_build/attempt_004/background_fluid.scdoc"
ARM_RELATIVE = "runs/cad_arm_completion/attempt_002/arm_fluid.scdoc"
VERIFICATION_RELATIVE = "runs/cad_arm_completion_verify/attempt_001/verify_report.json"
SOURCE_RELATIVES = (
    "definition_probe.py", "mcp_bridge.py", "stage15_mcp.py", "runtime.py",
    "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "mesh/wb_definition_probe.wbjn.in", "mesh/meshing_definition_probe.py.in",
)
AUTHORIZATION_VALUES = {
    "schema_version": 1, "scope": "stage15_mcp_definition_probe",
    "operation": "definition_probe", "maximum_attempts": 1,
    "maximum_total_seconds": 300, "cleanup_reserve_seconds": 30,
    "attempt_relative": ATTEMPT_RELATIVE,
    "background_cad_relative": BACKGROUND_RELATIVE, "arm_cad_relative": ARM_RELATIVE,
    "cad_verification_relative": VERIFICATION_RELATIVE,
    "full_mesh_allowed": False, "fluent_allowed": False, "automatic_retry_allowed": False,
    "old_fluent_mcp_modification_allowed": False, "original_geometry_modification_allowed": False,
    "default_theta_modification_allowed": False, "license_service_modification_allowed": False,
    "original_mesh_attempt_allowance_increased": False, "new_service_name": "ansys-stage15",
}
AUTHORIZATION_METADATA = ("authorization_source", "recorded_local", "completion_scope")
AUTHORIZATION_HASHES = ("background_sha256", "arm_sha256", "cad_verification_sha256",
                        "input_manifest_sha256", "contract_sha256")
SAFE_DEFAULTS = {"mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False}


class ProbeCancelled(RuntimeGateError):
    pass


class UncertainCall(RuntimeGateError):
    """A deadline-limited native call did not return; retain the ownership lock."""


def checked_path(path, root, *, must_exist=True):
    """Reject traversal and every symlink/reparse ancestor, including junctions."""
    root = Path(os.path.abspath(root))
    raw = Path(path)
    if ".." in raw.parts:
        raise RuntimeGateError("parent traversal is forbidden")
    path = Path(os.path.abspath(raw))
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise RuntimeGateError("path outside fixed root: " + str(path)) from exc
    for part in (path, *path.parents):
        try:
            info = part.lstat()
        except FileNotFoundError:
            if part == path and not must_exist:
                continue
            raise
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise RuntimeGateError("symlink/junction/reparse path is forbidden: " + str(part))
    if path.resolve() != path:
        raise RuntimeGateError("path resolves through an alias")
    return path


def load_json(path, root, *, maximum=64 * 1024 * 1024):
    path = checked_path(path, root)
    if not path.is_file() or path.stat().st_size > maximum:
        raise RuntimeGateError("missing/oversize JSON: " + str(path))
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise RuntimeGateError("duplicate JSON key: " + key)
            value[key] = item
        return value
    def reject_constant(value):
        raise RuntimeGateError("nonfinite JSON constant: " + value)
    value = json.loads(path.read_text(encoding="utf-8-sig"), object_pairs_hook=unique, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise RuntimeGateError("JSON object required: " + str(path))
    return value


def digest(path, root):
    path = checked_path(path, root)
    if not path.is_file():
        raise RuntimeGateError("regular input file required")
    return preflight.sha256(path)


def _hash(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def source_paths(stage_root):
    stage_root = Path(stage_root)
    return [stage_root / name for name in SOURCE_RELATIVES] + [stage_root.parent / "04_workbench/run_wb_fullenv.py"]


def validate_frozen(stage_root=STAGE):
    """Read actual protected inputs and CAD; no metadata-only acceptance."""
    stage_root = checked_path(stage_root, stage_root)
    workspace = stage_root.parent.parent
    auth_path = stage_root / AUTHORIZATION_RELATIVE
    auth = load_json(auth_path, stage_root, maximum=16384)
    if set(auth) != set(AUTHORIZATION_VALUES) | set(AUTHORIZATION_HASHES) | set(AUTHORIZATION_METADATA):
        raise RuntimeGateError("unexpected authorization keys")
    for key in AUTHORIZATION_METADATA:
        if not isinstance(auth[key], str) or not 1 <= len(auth[key]) <= 4096:
            raise RuntimeGateError("invalid bounded authorization metadata: " + key)
    for key, expected in AUTHORIZATION_VALUES.items():
        if type(auth[key]) is not type(expected) or auth[key] != expected:
            raise RuntimeGateError("authorization scope mismatch: " + key)
    if not all(_hash(auth[key]) for key in AUTHORIZATION_HASHES):
        raise RuntimeGateError("invalid authorization hash")
    contract_path = stage_root / "contract.json"
    if digest(contract_path, workspace) != auth["contract_sha256"]:
        raise RuntimeGateError("contract changed")
    contract = preflight.load_contract(contract_path)
    manifest_path = stage_root / "frozen/input_manifest.json"
    if digest(manifest_path, workspace) != auth["input_manifest_sha256"]:
        raise RuntimeGateError("frozen input manifest changed")
    manifest = load_json(manifest_path, workspace)
    expected = set(contract["preserve_inputs"]) | {contract_path.relative_to(workspace).as_posix()}
    rows = manifest.get("files", [])
    if (len(expected) != 8 or len(rows) != 8 or
            {row.get("path") for row in rows} != expected):
        raise RuntimeGateError("exactly eight contract-bound protected inputs required")
    for row in rows:
        if not _hash(row.get("sha256")) or digest(workspace / row["path"], workspace) != row["sha256"]:
            raise RuntimeGateError("protected input changed: " + str(row.get("path")))
    protected = preflight.verify_inputs(manifest, workspace=workspace)
    if protected["status"] != "passed":
        raise RuntimeGateError("protected input verification failed")
    cad = {}
    for name, relative in (("background", BACKGROUND_RELATIVE), ("arm", ARM_RELATIVE),
                           ("cad_verification", VERIFICATION_RELATIVE)):
        path = stage_root / relative
        actual = digest(path, stage_root)
        if actual != auth[name + "_sha256"]:
            raise RuntimeGateError("CAD/verification changed: " + name)
        cad[name] = {"path": str(path), "sha256": actual}
    receipt = load_json(stage_root / VERIFICATION_RELATIVE, stage_root)
    gate = receipt.get("cad_domains", {})
    if (receipt.get("mode") != "verify" or receipt.get("state") != "passed" or
            receipt.get("source_unchanged") is not True or gate.get("state") != "passed" or
            gate.get("independent_reopen_completed") is not True or
            gate.get("physical_face_ownership_checked") is not True or
            set(receipt.get("artifacts", {})) != {"background", "arm"}):
        raise RuntimeGateError("independent CAD acceptance is not passed")
    for name in ("background", "arm"):
        row = receipt["artifacts"][name]
        if checked_path(row["path"], stage_root) != Path(cad[name]["path"]) or row.get("sha256") != cad[name]["sha256"]:
            raise RuntimeGateError("independent CAD artifact mismatch: " + name)
    return {"contract": contract, "authorization": {"path": str(auth_path), "sha256": digest(auth_path, stage_root)},
            "protected_inputs": protected, "cad": cad}


def inspect(stage_root=STAGE):
    result = dict(SAFE_DEFAULTS, job_id=JOB_ID, status="blocked", blockers=[], ansys_launched=False)
    try:
        # Real environment is inspected even if an authorization is absent.
        contract = preflight.load_contract(Path(stage_root) / "contract.json")
        environment = preflight.inspect_environment(contract, root=stage_root)
        result["environment"] = environment
        result["blockers"].extend(environment.get("blockers", []))
        frozen = validate_frozen(stage_root)
        result.update({key: value for key, value in frozen.items() if key != "contract"})
        rendered = render_scripts(Path(stage_root) / ATTEMPT_RELATIVE, stage_root, frozen)
        result["template_preflight"] = {
            "status": "passed", "rendered_in_memory": True, "files_written": False,
            "meshing_script_sha256": hashlib.sha256(rendered["meshing_script"].encode("utf-8")).hexdigest(),
            "journal_sha256": hashlib.sha256(rendered["journal_script"].encode("utf-8")).hexdigest(),
        }
        lock = checked_path(Path(stage_root) / ".ansys-task.lock", stage_root, must_exist=False)
        if lock.exists():
            result["blockers"].append("ANSYS task lock exists; ownership must be resolved")
        if environment.get("status") != "passed" and not result["blockers"]:
            result["blockers"].append("environment did not pass")
        result["status"] = "blocked" if result["blockers"] else "passed"
    except Exception as exc:
        result["blockers"].append(type(exc).__name__ + ": " + str(exc))
    return result


def write_new(path, value, attempt):
    path = checked_path(path, attempt, must_exist=False)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())


def write_state(attempt, value):
    target = checked_path(attempt / "job_state.json", attempt, must_exist=False)
    temporary = checked_path(attempt / "job_state.worker.tmp", attempt, must_exist=False)
    # Never follow a previously left temporary file.
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def bounded_call(function, timeout):
    """Bound native/environment/cleanup APIs; an uncertain launch is never retried."""
    if timeout <= 0:
        raise UncertainCall("no remaining budget for native call")
    output = queue.Queue(maxsize=1)
    def execute():
        try:
            output.put((True, function()))
        except BaseException as exc:
            output.put((False, exc))
    thread = threading.Thread(target=execute, daemon=True)
    thread.start()
    try:
        ok, value = output.get(timeout=timeout)
    except queue.Empty as exc:
        raise UncertainCall("native call exceeded remaining bounded budget") from exc
    if not ok:
        raise value
    return value


def validate_request(attempt, stage_root):
    request = load_json(attempt / "request.json", attempt, maximum=65536)
    expected = {"schema_version", "operation", "job_id", "created_unix", "deadline_unix",
                "work_deadline_unix", "authorization", "sources"}
    if (set(request) != expected or type(request.get("schema_version")) is not int or
            request["schema_version"] != 1 or request.get("operation") != "definition_probe" or
            request.get("job_id") != JOB_ID):
        raise RuntimeGateError("invalid fixed probe request")
    for name in ("created_unix", "deadline_unix", "work_deadline_unix"):
        if type(request[name]) not in (int, float) or not math.isfinite(request[name]):
            raise RuntimeGateError("invalid finite deadline")
    created = request["created_unix"]
    if (request["deadline_unix"] != created + 300 or request["work_deadline_unix"] != created + 270 or
            created > time.time() + 1 or request["deadline_unix"] <= time.time()):
        raise RuntimeGateError("request total/work deadline mismatch or expired")
    validate_sources(request["sources"], stage_root)
    return request


def validate_sources(rows, stage_root):
    expected = {str(path.absolute()).casefold(): path for path in source_paths(stage_root)}
    if not isinstance(rows, list) or len(rows) != len(expected):
        raise RuntimeGateError("exact frozen source set required")
    seen = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"path", "sha256", "size"}:
            raise RuntimeGateError("invalid source record")
        path = checked_path(row["path"], Path(stage_root).parent)
        key = str(path).casefold()
        if key not in expected or key in seen or type(row["size"]) is not int or not 0 <= row["size"] <= 2 * 1024 * 1024:
            raise RuntimeGateError("unapproved/duplicate/oversize source path")
        seen.add(key)
        if path.stat().st_size != row["size"] or not _hash(row["sha256"]) or digest(path, Path(stage_root).parent) != row["sha256"]:
            raise RuntimeGateError("frozen source changed: " + str(path))


def validate_launcher(attempt, request, remaining):
    started = time.monotonic()
    end = started + min(LAUNCHER_WAIT_SECONDS, remaining())
    while time.monotonic() < end:
        path = checked_path(attempt / "launcher.json", attempt, must_exist=False)
        if path.exists():
            ledger = load_json(path, attempt, maximum=8192)
            stamp = bounded_call(lambda: psutil.Process(os.getpid()).create_time(), min(2, remaining()))
            if (set(ledger) != {"job_id", "pid", "creation_time", "recorded_unix"} or
                    ledger.get("job_id") != JOB_ID or type(ledger.get("pid")) is not int or
                    ledger["pid"] != os.getpid() or ledger.get("creation_time") != stamp or
                    type(ledger.get("recorded_unix")) not in (int, float) or
                    not math.isfinite(ledger["recorded_unix"]) or
                    not request["created_unix"] <= ledger["recorded_unix"] <= time.time() + 1):
                raise RuntimeGateError("worker PID/creation-time launcher ledger mismatch")
            return ledger
        time.sleep(min(0.05, max(0, end - time.monotonic())))
    raise RuntimeGateError("launcher ownership ledger absent after bounded %d-second wait" % LAUNCHER_WAIT_SECONDS)


def check_cancel(attempt):
    path = checked_path(attempt / "cancel_request.json", attempt, must_exist=False)
    if path.exists():
        value = load_json(path, attempt, maximum=4096)
        if value.get("job_id") != JOB_ID or value.get("operation") != "cancel_definition_probe":
            raise RuntimeGateError("invalid cancellation sentinel")
        raise ProbeCancelled("definition probe cancellation requested")


def audit_template(text, *, journal=False):
    """Deny generation/solver paths and restrict all update/edit operations by AST."""
    tree = ast.parse(text)
    banned = re.compile(r"generatemesh|\bfluent\b|\bsolver\b|\bsolve\b|\biterate\b|\bconvert(?:mesh|tomesh)\w*", re.I)
    if banned.search(text):
        raise RuntimeGateError("forbidden generation/solver path in definition template")
    save_calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store) and isinstance(node.value, ast.Name) and node.value.id == "C":
            raise RuntimeGateError("template configuration mutation forbidden")
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            modules = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module]
            if any(name not in {"System", "System.IO", "System.Text", "json", "codecs", "os", "traceback"} for name in modules):
                raise RuntimeGateError("unapproved native template import")
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr in {"system", "popen", "Popen", "Start", "execv", "spawnv", "Save"}:
            raise RuntimeGateError("unapproved launch/save operation in definition template")
        if isinstance(func, ast.Name) and func.id == "Save":
            arguments = {item.arg: item.value for item in node.keywords}
            path = arguments.get("FilePath")
            overwrite = arguments.get("Overwrite")
            if (not journal or node.args or len(node.keywords) != 2 or set(arguments) != {"FilePath", "Overwrite"} or
                    not isinstance(path, ast.Subscript) or not isinstance(path.value, ast.Name) or path.value.id != "C" or
                    not isinstance(path.slice, ast.Constant) or path.slice.value != "project" or
                    not isinstance(overwrite, ast.Constant) or overwrite.value is not False):
                raise RuntimeGateError("only the fixed new-attempt project Save with Overwrite=False is allowed")
            save_calls.append(node)
        if isinstance(func, ast.Name) and func.id in {"Open", "RunScript"}:
            raise RuntimeGateError("unapproved project/script operation")
        if isinstance(func, ast.Name) and func.id in {"eval", "exec", "compile", "__import__"}:
            raise RuntimeGateError("dynamic execution forbidden")
        if isinstance(func, ast.Attribute) and func.attr in {"Update", "Refresh", "Edit", "SendCommand"}:
            receiver = func.value.id if isinstance(func.value, ast.Name) else None
            allowed = {"Update": "geometry_component", "Refresh": "mesh_component",
                       "Edit": "mesh_container", "SendCommand": "mesh_container"}
            if not journal or receiver != allowed[func.attr]:
                raise RuntimeGateError("unapproved component operation")
        if isinstance(func, ast.Attribute) and func.attr == "Update":
            if any(key.arg == "AllDependencies" and not (isinstance(key.value, ast.Constant) and key.value.value is False)
                   for key in node.keywords):
                raise RuntimeGateError("dependency-chain updates forbidden")
    if len(save_calls) > 1:
        raise RuntimeGateError("only one new-attempt project Save is permitted")
    return True


def render_scripts(attempt, stage_root, frozen):
    """Render and audit in memory, including before the fixed slot exists."""
    stage_root = checked_path(stage_root, stage_root)
    raw = Path(attempt)
    attempt = Path(os.path.abspath(raw))
    if ".." in raw.parts or attempt != stage_root / ATTEMPT_RELATIVE:
        raise RuntimeGateError("script rendering requires the fixed attempt path")
    project = attempt / "stage15_definition_probe.wbpj"
    settings = domain_settings(frozen["contract"], attempt)[0]
    meshing_template = checked_path(stage_root / "mesh/meshing_definition_probe.py.in", stage_root).read_text(encoding="utf-8")
    journal_template = checked_path(stage_root / "mesh/wb_definition_probe.wbjn.in", stage_root).read_text(encoding="utf-8")
    if meshing_template.count("__CONFIG_JSON__") != 1 or journal_template.count("__CONFIG_JSON__") != 1:
        raise RuntimeGateError("each definition template must contain exactly one configuration placeholder")
    audit_template(meshing_template)
    audit_template(journal_template, journal=True)
    script = meshing_template.replace("__CONFIG_JSON__", repr(json.dumps(settings, ensure_ascii=True, allow_nan=False)))
    config = {"cad_path": frozen["cad"]["background"]["path"], "meshing_script": script,
              "project": str(project), "trace": str(attempt / "workbench.trace.txt"),
              "completion": str(attempt / "workbench_completion.json"),
              "stats_path": str(attempt / "background_meshing_stats.json")}
    journal = journal_template.replace("__CONFIG_JSON__", repr(json.dumps(config, ensure_ascii=True, allow_nan=False)))
    audit_template(script)
    audit_template(journal, journal=True)
    return {"meshing_script": script, "journal_script": journal, "configuration": config, "settings": settings}


def prepare_scripts(attempt, stage_root, frozen):
    project = checked_path(attempt / "stage15_definition_probe.wbpj", attempt, must_exist=False)
    if project.exists():
        raise RuntimeGateError("refusing existing definition probe project")
    rendered = render_scripts(attempt, stage_root, frozen)
    for name, value in (("meshing_definition_probe.py", rendered["meshing_script"]),
                        ("wb_definition_probe.wbjn", rendered["journal_script"])):
        path = checked_path(attempt / name, attempt, must_exist=False)
        with path.open("x", encoding="utf-8") as stream:
            stream.write(value)
    return attempt / "wb_definition_probe.wbjn"


def validate_definition_stats(stats, settings):
    for phase in ("mesh_counts_before", "mesh_counts_after"):
        counts = stats.get(phase)
        if (not isinstance(counts, dict) or set(counts) != {"nodes", "elements"} or
                any(type(counts[key]) is not int or counts[key] != 0 for key in ("nodes", "elements"))):
            raise RuntimeGateError("native mesh counts must be complete nonnegative integer zeros: " + phase)
    selections = stats.get("named_selections", {})
    if set(selections) != set(settings["required_selections"]):
        raise RuntimeGateError("required named selections missing from readback")
    faces = stats.get("body_face_ids", [])
    bodies = stats.get("body_geometry_ids", [])
    if not faces or len(bodies) != 1 or len(set(faces)) != len(faces):
        raise RuntimeGateError("body/face readback is incomplete")
    used = set()
    for ids in selections.values():
        if not ids or len(set(ids)) != len(ids) or not set(ids).issubset(faces) or used.intersection(ids):
            raise RuntimeGateError("required selection is not unique nonempty body-face scope")
        used.update(ids)
    def same_number(actual, expected):
        return type(actual) in (int, float) and math.isfinite(actual) and abs(actual - expected) <= max(1e-10, abs(expected) * 1e-7)
    for key in ("global_size_m", "maximum_size_m", "minimum_size_m", "growth_rate"):
        if not same_number(stats.get("global_readback", {}).get(key), settings[key]):
            raise RuntimeGateError("global size readback mismatch")
    expected = {"stage15_background_tet": ("tet_method", bodies, {"method": "AllTriAllTet"})}
    for item in settings["face_controls"]:
        expected["stage15_" + item["name"]] = ("face_size", selections[item["name"]],
            {"type": "ElementSize", "size_m": item["size_m"], "growth_rate": 0.0,
             "local_growth_override_applied": False, "global_growth_rate_requested": settings["growth_rate"]})
    for index, sphere in enumerate(settings["spheres"]):
        expected["stage15_overlap_%d" % index] = ("sphere", bodies,
            {"type": "SphereOfInfluence", "size_m": sphere["size_m"], "radius_m": sphere["radius_m"],
             "centre_m": sphere["centre_m"], "coordinate_object_state": "FullyDefined"})
    rows = stats.get("controls", [])
    if len(rows) != len(expected) or {row.get("name") for row in rows} != set(expected):
        raise RuntimeGateError("required tet/face/sphere controls are incomplete")
    for row in rows:
        kind, ids, values = expected[row["name"]]
        if (row.get("kind") != kind or row.get("object_state") != "FullyDefined" or
                sorted(row.get("actual_geometry_ids", [])) != sorted(ids) or
                sorted(row.get("expected_geometry_ids", [])) != sorted(ids)):
            raise RuntimeGateError("control scope/state readback mismatch")
        for key, expected_value in values.items():
            actual = row.get("values", {}).get(key)
            if isinstance(expected_value, list):
                equal = isinstance(actual, list) and len(actual) == len(expected_value) and all(same_number(a, b) for a, b in zip(actual, expected_value))
            elif type(expected_value) in (int, float):
                equal = same_number(actual, expected_value)
            elif type(expected_value) is bool:
                equal = actual is expected_value
            else:
                equal = actual == expected_value
            if not equal:
                raise RuntimeGateError("control value failed readback: " + key)


def observe_receipts(attempt, *, required=False, settings=None):
    stats_path = checked_path(attempt / "background_meshing_stats.json", attempt, must_exist=False)
    completion_path = checked_path(attempt / "workbench_completion.json", attempt, must_exist=False)
    stats = load_json(stats_path, attempt, maximum=1024 * 1024) if stats_path.exists() else None
    completion = load_json(completion_path, attempt, maximum=1024 * 1024) if completion_path.exists() else None
    for name, receipt in (("Meshing", stats), ("Workbench", completion)):
        if receipt is not None and (receipt.get("status") == "failed" or receipt.get("exception")):
            raise RuntimeGateError(name + " failure receipt: " + str(receipt.get("exception", "failed")))
    if stats is not None and stats.get("mesh_generated") is not False:
        raise RuntimeGateError("probe receipt must explicitly forbid generated mesh")
    if required:
        if not stats or not completion:
            raise RuntimeGateError("completed exit lacks required definition receipts")
        if (stats.get("status") != "definitions_ready" or stats.get("script_entered") is not True or
                stats.get("definitions_ready") is not True or stats.get("message_capture_complete") is not True or
                type(stats.get("flow_iterations")) is not int or stats["flow_iterations"] != 0 or stats.get("mesh_motion_verified") is not False or
                completion.get("status") != "journal_completed" or completion.get("journal_completed") is not True or
                completion.get("definitions_ready") is not True or completion.get("mesh_generated") is not False or
                type(completion.get("flow_iterations")) is not int or completion["flow_iterations"] != 0 or
                completion.get("mesh_motion_verified") is not False):
            raise RuntimeGateError("definition receipts are incomplete or not passed")
        if settings is None:
            raise RuntimeGateError("contract settings are required for definition acceptance")
        validate_definition_stats(stats, settings)
        trace = checked_path(attempt / "workbench.trace.txt", attempt)
        if trace.stat().st_size > 1024 * 1024:
            raise RuntimeGateError("oversize Workbench trace")
        lines = trace.read_text(encoding="utf-8-sig").splitlines()
        expected_phases = ["journal-entered", "geometry-loaded", "meshing-script-dispatched", "definitions-ready", "journal-completed"]
        if lines != expected_phases:
            raise RuntimeGateError("journal phase evidence incomplete")
        if not isinstance(stats.get("messages"), list):
            raise RuntimeGateError("native message inventory is missing")
        for message in stats["messages"]:
            severity = str(message.get("severity", "")).lower().rsplit(".", 1)[-1]
            if severity not in {"info", "information", "warning"} or not isinstance(message.get("text"), str):
                raise RuntimeGateError("native error/unknown message severity")
    return stats, completion


def run(attempt, *, stage_root=STAGE):
    stage_root = checked_path(stage_root, stage_root)
    attempt = checked_path(attempt, stage_root)
    if attempt != stage_root / ATTEMPT_RELATIVE or not attempt.is_dir():
        raise RuntimeGateError("only the fixed pre-reserved attempt is allowed")
    # A failed/partial first worker consumes the same slot: no retry or overwrite.
    terminal = checked_path(attempt / "probe_result.json", attempt, must_exist=False)
    if terminal.exists():
        raise RuntimeGateError("terminal probe result already exists; no rerun permitted")
    write_new(attempt / "worker_claim.json", {"job_id": JOB_ID, "pid": os.getpid(), "created_unix": time.time()}, attempt)
    began = time.monotonic()
    report = dict(SAFE_DEFAULTS, job_id=JOB_ID, status="running", phase="validating", error=None,
                  owned_cleanup_verified=True, ansys_launched=False)
    process = tree = lock = None
    request = frozen = None
    uncertain = False
    deadline = time.monotonic() + 300
    work_deadline = deadline - 30
    def remaining():
        return max(0, min(deadline - time.monotonic(), request["deadline_unix"] - time.time() if request else 300))
    def working():
        check_cancel(attempt)
        if time.monotonic() >= work_deadline or (request and time.time() >= request["work_deadline_unix"]):
            raise TimeoutError("270-second work deadline reached; beginning owned cleanup")
    def elapsed():
        return max(time.monotonic() - began, 300 - remaining())
    def state(phase):
        report.update(phase=phase, elapsed_seconds=elapsed(), worker_elapsed_seconds=time.monotonic() - began)
        write_state(attempt, report)
    try:
        request = validate_request(attempt, stage_root)
        deadline = began + max(0, request["deadline_unix"] - time.time())
        work_deadline = deadline - 30
        report.update({key: request[key] for key in ("created_unix", "deadline_unix", "work_deadline_unix")})
        report["maximum_total_seconds"] = 300
        report["cleanup_reserve_seconds"] = 30
        validate_launcher(attempt, request, remaining)
        working()
        frozen = bounded_call(lambda: validate_frozen(stage_root), min(15, remaining() - 30))
        if request["authorization"] != frozen["authorization"]:
            raise RuntimeGateError("request authorization path/hash mismatch")
        report["protected_inputs_before"] = frozen["protected_inputs"]
        report["cad"] = frozen["cad"]
        report["authorization"] = frozen["authorization"]
        state("preflight")
        environment = bounded_call(lambda: preflight.inspect_environment(frozen["contract"], root=stage_root), min(40, remaining() - 30))
        if environment.get("status") != "passed":
            raise RuntimeGateError("Workbench preflight blocked: " + repr(environment.get("blockers")))
        working()
        lock = AnsysTaskLock(attempt, stage_root=stage_root)
        lock.__enter__()
        journal = prepare_scripts(attempt, stage_root, frozen)
        working()
        env = bounded_call(lambda: run_mesh_job.child_environment(attempt, frozen["contract"]), min(10, remaining() - 30))
        command = [frozen["contract"]["software"]["workbench"], "-I", "-X", "-R", str(journal)]
        state("starting_workbench")
        for name in ("workbench.stdout.log", "workbench.stderr.log", "workbench.trace.txt", "background_meshing_stats.json", "workbench_completion.json",
                     "background_meshing_stats.json.tmp", "workbench_completion.json.tmp", "stage15_definition_probe.wbpj"):
            output = checked_path(attempt / name, attempt, must_exist=False)
            if output.exists():
                raise RuntimeGateError("refusing existing probe output: " + name)
        validate_sources(request["sources"], stage_root)
        latest = bounded_call(lambda: validate_frozen(stage_root), min(10, remaining() - 30))
        if latest["authorization"] != request["authorization"]:
            raise RuntimeGateError("authorization changed immediately before launch")
        working()
        with (attempt / "workbench.stdout.log").open("xb") as stdout, (attempt / "workbench.stderr.log").open("xb") as stderr:
            # A blocked Popen has uncertain ownership; keep the lock, never retry.
            try:
                process = bounded_call(lambda: subprocess.Popen(command, cwd=str(attempt), env=env,
                                       stdout=stdout, stderr=stderr, shell=False), min(10, remaining() - 30))
            except UncertainCall:
                uncertain = True
                raise
            report["ansys_launched"] = True
            report["launcher_pid"] = process.pid
            try:
                tree = bounded_call(lambda: OwnedProcessTree(process, psutil), min(3, remaining() - 30))
            except Exception:
                uncertain = True
                raise
            if tree.errors:
                raise RuntimeGateError("cannot establish owned process tree: " + repr(tree.errors))
            state("meshing_definitions")
            while True:
                working()
                bounded_call(tree.capture, min(2, remaining() - 30))
                stats, completion = observe_receipts(attempt)
                if stats and stats.get("script_entered") is True and report["phase"] != "definitions_ready":
                    state("definitions_ready" if stats.get("definitions_ready") is True else "script_entered")
                if shutil.disk_usage(attempt).free < frozen["contract"]["budget"]["minimum_free_disk_gib"] * 1024 ** 3:
                    raise RuntimeGateError("disk below configured 20-GiB reserve")
                code = bounded_call(process.poll, min(2, remaining() - 30))
                if code is not None:
                    report["exit_code"] = code
                    if code != 0:
                        raise RuntimeGateError("Workbench launcher exit code " + str(code))
                    break
                time.sleep(min(0.1, max(0, work_deadline - time.monotonic())))
        stats, completion = observe_receipts(attempt, required=True, settings=domain_settings(frozen["contract"], attempt)[0])
        report["definition_receipt"] = stats
        report["completion"] = completion
        validate_sources(request["sources"], stage_root)
        after = bounded_call(lambda: validate_frozen(stage_root), min(10, remaining() - 30))
        working()
        if after["authorization"] != frozen["authorization"]:
            raise RuntimeGateError("authorization changed during probe")
        report["protected_inputs_after"] = after["protected_inputs"]
        report["status"] = "passed"
    except ProbeCancelled as exc:
        report.update(status="cancelled", error=str(exc))
    except Exception as exc:
        if isinstance(exc, UncertainCall) and process is not None:
            uncertain = True
        report.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
    finally:
        report["phase"] = "cleanup"
        try:
            state("cleanup")
        except Exception as exc:
            report.update(status="failed", state_write_error=str(exc))
        cleanup = not uncertain
        if tree is not None:
            try:
                # Existing close uses 8+5s waits; reserve an additional margin.
                if remaining() < 15:
                    raise UncertainCall("insufficient cleanup reserve for 8+5-second owned close")
                cleanup = bool(bounded_call(tree.close, min(20, remaining() - 1))) and cleanup
            except Exception as exc:
                cleanup = False
                report["cleanup_error"] = type(exc).__name__ + ": " + str(exc)
            report["owned_processes"] = [{"pid": pid, "creation_time": stamp} for pid, stamp in tree.members.items()]
            report["ownership_errors"] = list(tree.errors)
        elif process is not None:
            cleanup = False
        if frozen is not None:
            try:
                after = bounded_call(lambda: validate_frozen(stage_root), min(5, remaining() - 1))
                report["protected_inputs_after"] = after["protected_inputs"]
                if after["authorization"] != frozen["authorization"]:
                    raise RuntimeGateError("authorization changed during cleanup")
                if request is not None:
                    validate_sources(request["sources"], stage_root)
            except Exception as exc:
                report.update(status="failed", input_recheck_error=type(exc).__name__ + ": " + str(exc))
        if lock is not None and lock.acquired:
            if not cleanup:
                lock.retain("definition-probe process ownership or cleanup is unverified")
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
        report.update(owned_cleanup_verified=cleanup, phase="finished", elapsed_seconds=elapsed(), worker_elapsed_seconds=time.monotonic() - began)
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
    try:
        if args.preflight:
            if args.attempt is not None:
                raise RuntimeGateError("--attempt is not accepted for read-only preflight")
            report = inspect()
        else:
            if args.attempt is None:
                raise RuntimeGateError("fixed --attempt is required")
            report = run(args.attempt)
    except Exception as exc:
        report = dict(SAFE_DEFAULTS, job_id=JOB_ID, status="failed", error=type(exc).__name__ + ": " + str(exc))
    print(json.dumps(report, ensure_ascii=True, allow_nan=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())

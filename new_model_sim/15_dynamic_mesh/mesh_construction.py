"""One explicit fourth two-domain mesh construction; never launch a CFD solver."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import re
import sys
import time
from pathlib import Path

# Read-only --preflight must not even create __pycache__ directories.
sys.dont_write_bytecode = True
import psutil
import definition_probe as probe
import mesh_prepare
import preflight
import run_mesh_job
from runtime import RuntimeGateError

STAGE = Path(__file__).resolve().parent
JOB_ID = "mesh-construction-004"
OPERATION = "mesh_construction"
ATTEMPT_RELATIVE = "runs/mcp_mesh_construction/attempt_001"
MESH_ATTEMPT_RELATIVE = "runs/mesh/attempt_004"
AUTHORIZATION_RELATIVE = "frozen/mesh_construction_authorization_001.json"
LAUNCHER_WAIT_SECONDS = 20
SOURCE_RELATIVES = (
    "stage15_mcp.py", "mcp_bridge.py", "sizing_mcp_bridge.py", "mesh_mcp_bridge.py",
    "mesh_construction.py", "runtime.py", "preflight.py", "mesh_prepare.py",
    "run_mesh_job.py", "definition_probe.py", "mesh/meshing_stage15.py.in", "mesh/wb_stage15.wbjn.in",
)
AUTHORIZATION_VALUES = {
    "schema_version": 1, "scope": "stage15_mcp_mesh_construction", "operation": OPERATION,
    "job_id": JOB_ID, "maximum_additional_attempts": 1, "original_full_mesh_attempts_used": 3,
    "maximum_total_seconds": 7200, "cleanup_reserve_seconds": 30, "maximum_domain_seconds": 3600,
    "job_attempt_relative": ATTEMPT_RELATIVE, "mesh_attempt_relative": MESH_ATTEMPT_RELATIVE,
    "domains": ["background", "arm"],
    "baseline_authorization_relative": "frozen/mcp_definition_probe_authorization_001.json",
    "baseline_authorization_sha256": "51dbdd6023ad9d9f0929d38fefcb4e86a6d906ec25ac18b7c00c2126ccbfbd1f",
    "mesh_generation_allowed": True, "mesh_export_allowed": True, "fluent_allowed": False,
    "CFD_allowed": False, "automatic_retry_allowed": False,
    "original_geometry_modification_allowed": False, "default_theta_modification_allowed": False,
    "license_service_modification_allowed": False,
}
AUTHORIZATION_METADATA = {"authorization_source", "recorded_local", "execution_policy"}
SAFE_DEFAULTS = {
    "job_id": JOB_ID, "operation": OPERATION, "mesh_generated": False,
    "mesh_exports_written": False, "mesh_inventory_status": "not_run", "mesh_quality_verified": False,
    "mesh_motion_verified": False, "static_overset": "not_run", "motion": "not_run",
    "flow": "not_run", "flow_iterations": 0, "solver_launched": False,
    "owned_cleanup_verified": False, "inputs_reverified_after_cleanup": False,
    "sources_reverified_after_cleanup": False, "mesh_attempt_relative": MESH_ATTEMPT_RELATIVE,
}


def source_paths(stage_root=STAGE):
    root = Path(stage_root)
    return [root / name for name in SOURCE_RELATIVES] + [root.parent / "04_workbench/run_wb_fullenv.py"]


def validate_authorization(stage_root=STAGE):
    root = Path(stage_root)
    path = root / AUTHORIZATION_RELATIVE
    value = probe.load_json(path, root, maximum=16384)
    if not set(AUTHORIZATION_VALUES) <= set(value) or set(value) - set(AUTHORIZATION_VALUES) - AUTHORIZATION_METADATA:
        raise RuntimeGateError("unexpected mesh construction authorization keys")
    for key, expected in AUTHORIZATION_VALUES.items():
        if type(value[key]) is not type(expected) or value[key] != expected:
            raise RuntimeGateError("mesh construction authorization mismatch: " + key)
    for key in AUTHORIZATION_METADATA & set(value):
        if not isinstance(value[key], str) or not 1 <= len(value[key]) <= 4096 or any(ord(c) < 32 for c in value[key]):
            raise RuntimeGateError("invalid bounded authorization metadata: " + key)
    baseline = root / value["baseline_authorization_relative"]
    if probe.digest(baseline, root) != value["baseline_authorization_sha256"]:
        raise RuntimeGateError("baseline CAD/input binding authorization changed")
    return {"path": str(path), "sha256": probe.digest(path, root)}


def validate_sources(rows, stage_root=STAGE):
    root = Path(stage_root)
    expected = {str(p.absolute()).casefold() for p in source_paths(root)}
    if not isinstance(rows, list) or len(rows) != 13 or len(expected) != 13:
        raise RuntimeGateError("exact 13 frozen mesh source records required")
    seen = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"path", "size", "sha256"}:
            raise RuntimeGateError("invalid mesh source record")
        path = probe.checked_path(row["path"], root.parent)
        key = str(path).casefold()
        if key not in expected or key in seen or type(row["size"]) is not int or not 0 <= row["size"] <= 2 * 1024 * 1024:
            raise RuntimeGateError("unapproved/duplicate/oversize mesh source")
        seen.add(key)
        if path.stat().st_size != row["size"] or not probe._hash(row["sha256"]) or probe.digest(path, root.parent) != row["sha256"]:
            raise RuntimeGateError("mesh source changed: " + str(path))
    return rows


def validate_request(attempt, stage_root=STAGE):
    attempt = probe.checked_path(attempt, stage_root)
    if attempt != Path(stage_root) / ATTEMPT_RELATIVE:
        raise RuntimeGateError("mesh worker requires the fixed outer attempt")
    request = probe.load_json(attempt / "request.json", attempt, maximum=65536)
    keys = {"schema_version", "operation", "job_id", "created_unix", "deadline_unix",
            "work_deadline_unix", "authorization", "sources"}
    if (set(request) != keys or type(request.get("schema_version")) is not int or
            request["schema_version"] != 1 or request.get("operation") != OPERATION or request.get("job_id") != JOB_ID):
        raise RuntimeGateError("invalid fixed mesh construction request")
    for key in ("created_unix", "deadline_unix", "work_deadline_unix"):
        if type(request[key]) not in (int, float) or not math.isfinite(request[key]):
            raise RuntimeGateError("invalid finite mesh deadline")
    created = request["created_unix"]
    if (request["deadline_unix"] != created + 7200 or request["work_deadline_unix"] != created + 7170 or
            created > time.time() + 1 or request["deadline_unix"] <= time.time()):
        raise RuntimeGateError("mesh total/work deadline mismatch or expired")
    if request["authorization"] != validate_authorization(stage_root):
        raise RuntimeGateError("request does not bind the new mesh authorization")
    validate_sources(request["sources"], stage_root)
    return request


def validate_launcher(attempt, request, remaining):
    end = time.monotonic() + min(LAUNCHER_WAIT_SECONDS, remaining())
    while time.monotonic() < end:
        path = probe.checked_path(attempt / "launcher.json", attempt, must_exist=False)
        if path.exists():
            ledger = probe.load_json(path, attempt, maximum=8192)
            stamp = probe.bounded_call(lambda: psutil.Process(os.getpid()).create_time(), min(2, remaining()))
            if (set(ledger) != {"job_id", "pid", "creation_time", "recorded_unix"} or
                    ledger.get("job_id") != JOB_ID or type(ledger.get("pid")) is not int or ledger["pid"] != os.getpid() or
                    type(ledger.get("creation_time")) not in (int, float) or ledger["creation_time"] != stamp or
                    type(ledger.get("recorded_unix")) not in (int, float) or not math.isfinite(ledger["recorded_unix"]) or
                    not request["created_unix"] <= ledger["recorded_unix"] <= time.time() + 1):
                raise RuntimeGateError("mesh worker PID/creation-time launcher ledger mismatch")
            return ledger
        time.sleep(min(.05, max(0, end - time.monotonic())))
    raise RuntimeGateError("mesh launcher ownership ledger absent after bounded 20-second wait")


def check_cancel(attempt):
    path = probe.checked_path(Path(attempt) / "cancel_request.json", attempt, must_exist=False)
    if not path.exists():
        return False
    value = probe.load_json(path, attempt, maximum=4096)
    if (set(value) != {"job_id", "operation", "created_unix"} or value.get("job_id") != JOB_ID or
            value.get("operation") != "cancel_mesh_construction" or type(value.get("created_unix")) not in (int, float) or
            not math.isfinite(value["created_unix"]) or value["created_unix"] > time.time() + 1):
        raise RuntimeGateError("invalid mesh cancellation request")
    return True


def audit_templates(stage_root, frozen):
    """Render only in memory, and inspect executable call sites, not comments."""
    root = Path(stage_root)
    meshing = (root / "mesh/meshing_stage15.py.in").read_text(encoding="utf-8")
    journal = (root / "mesh/wb_stage15.wbjn.in").read_text(encoding="utf-8")
    config = mesh_prepare.domain_settings(frozen["contract"], root / MESH_ATTEMPT_RELATIVE)
    if [row["domain"] for row in config] != ["background", "arm"] or any(row["growth_rate"] != 1.3 for row in config):
        raise RuntimeGateError("fixed two-domain settings changed")
    for text in (meshing, journal):
        tree = ast.parse(text)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                modules = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ""]
                if any(m.split('.')[0] in {"subprocess", "ansys", "socket", "requests"} for m in modules):
                    raise RuntimeGateError("unexpected solver/network/process import in mesh template")
            if isinstance(node, ast.Call):
                name = node.func.attr if isinstance(node.func, ast.Attribute) else (node.func.id if isinstance(node.func, ast.Name) else "")
                if name.lower() in {"launch_fluent", "initialize", "iterate", "solve", "popen", "system", "exec", "eval", "run_calculation"}:
                    raise RuntimeGateError("solver or arbitrary process call forbidden in mesh template")
    mesh_tree = ast.parse(meshing)
    generates = [n for n in ast.walk(mesh_tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'GenerateMesh']
    if len(generates) != 1:
        raise RuntimeGateError("one explicit GenerateMesh call per fixed domain required")
    # The A/B-proven fix must not regress to local face GrowthRate assignment.
    face = next(n for n in mesh_tree.body if isinstance(n, ast.FunctionDef) and n.name == 'configure_face_size')
    for node in ast.walk(face):
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(t, ast.Attribute) and t.attr == 'GrowthRate' for t in targets):
                raise RuntimeGateError("local face GrowthRate setter is forbidden")
    for item in config:
        rendered = meshing.replace('__CONFIG_JSON__', repr(json.dumps(item, ensure_ascii=True)))
        compile(rendered, '<mesh construction in memory>', 'exec')
    compile(journal.replace('__CONFIG_JSON__', repr(json.dumps({'domains': config}))), '<mesh journal in memory>', 'exec')
    return {"status": "passed", "rendered_in_memory": True, "files_written": False,
            "domains": [r['domain'] for r in config], "meshing_sha256": hashlib.sha256(meshing.encode()).hexdigest(),
            "journal_sha256": hashlib.sha256(journal.encode()).hexdigest()}


def inspect(stage_root=STAGE):
    report = dict(SAFE_DEFAULTS, status="blocked", blockers=[], ansys_launched=False)
    try:
        root = probe.checked_path(stage_root, stage_root)
        contract = preflight.load_contract(root / 'contract.json')
        # Existing preflight uses read-only lmstat/TCP readiness, not feature checkout or a solver.
        environment = probe.bounded_call(lambda: preflight.inspect_environment(contract, root=root), 25)
        report['environment'] = environment
        report['blockers'].extend(environment.get('blockers', []))
        report['authorization'] = validate_authorization(root)
        frozen = probe.bounded_call(lambda: probe.validate_frozen(root), 15)
        report['protected_inputs'] = frozen['protected_inputs']
        report['cad'] = frozen['cad']
        report['template_preflight'] = audit_templates(root, frozen)
        rows = [{'path': str(p), 'size': p.stat().st_size, 'sha256': probe.digest(p, root.parent)} for p in source_paths(root)]
        report['sources'] = validate_sources(rows, root)
        if (root / '.ansys-task.lock').exists():
            report['blockers'].append('ANSYS task lock exists; ownership must be resolved')
        if (root / MESH_ATTEMPT_RELATIVE).exists():
            report['blockers'].append('fourth mesh attempt already exists; no reuse or automatic retry')
        if environment.get('status') != 'passed' and not report['blockers']:
            report['blockers'].append('environment did not pass')
        report['status'] = 'blocked' if report['blockers'] else 'passed'
    except Exception as exc:
        report['blockers'].append(type(exc).__name__ + ': ' + str(exc))
    return report


def validate_exports(attempt, completion):
    """Validate the journal's non-recursive native export provenance without hashing large files."""
    attempt = Path(attempt)
    systems = completion.get('systems', [])
    if (completion.get('mesh_exports_written') is not True or
            [row.get('domain') for row in systems] != ['background', 'arm']):
        raise RuntimeGateError('two explicit completed mesh exports required')
    project_files = attempt / 'stage15_two_domains_files' / 'dp0'
    sources, targets = [], []
    for domain, row in zip(('background', 'arm'), systems):
        name = row.get('system_name')
        if (not isinstance(name, str) or len(name) > 64 or
                re.fullmatch(r'[A-Za-z0-9_-]+(?: [A-Za-z0-9_-]+)*', name) is None):
            raise RuntimeGateError('unsafe Workbench system name')
        names = list(dict.fromkeys([name, name.replace(' ', '-')]))
        directories = [probe.checked_path(project_files / part, attempt, must_exist=False) for part in names]
        existing = [path for path in directories if path.is_dir()]
        if len(existing) != 1:
            raise RuntimeGateError('missing or ambiguous per-system export directories')
        mech = probe.checked_path(existing[0] / 'MECH', attempt)
        exported = row.get('mesh_export', {})
        if set(exported) != {'source_path', 'path', 'size'} or type(exported['size']) is not int or exported['size'] <= 0:
            raise RuntimeGateError('invalid native export receipt')
        source = probe.checked_path(exported['source_path'], mech)
        target = probe.checked_path(exported['path'], attempt)
        extension = '.msh.h5' if source.name.lower().endswith('.msh.h5') else '.msh'
        if source.parent != mech or not source.name.lower().endswith(extension) or target != attempt / (domain + extension):
            raise RuntimeGateError('mesh source/target is not in its fixed domain directory')
        candidates = [p for p in mech.iterdir() if p.name.lower().endswith(('.msh', '.msh.h5'))]
        if len(candidates) != 1 or candidates[0] != source:
            raise RuntimeGateError('missing or ambiguous native mesh export')
        if not source.is_file() or not target.is_file() or source.stat().st_size != exported['size'] or target.stat().st_size != exported['size']:
            raise RuntimeGateError('empty/missing/changed mesh export copy')
        if any(os.path.samefile(source, p) for p in sources) or any(os.path.samefile(target, p) for p in targets):
            raise RuntimeGateError('same mesh file cannot represent both domains')
        sources.append(source)
        targets.append(target)
    return targets


def run(attempt, *, stage_root=STAGE):
    root = Path(stage_root)
    attempt = Path(attempt)
    report = dict(SAFE_DEFAULTS, status='failed', receipt_paths={}, meshes=[])
    claimed = False
    request = None
    # A failed pre-claim validation must never overwrite another worker's state/result.
    try:
        request = probe.bounded_call(lambda: validate_request(attempt, root), 15)
        end = time.monotonic() + max(0, request['deadline_unix'] - time.time())
        def remaining(cleanup=False):
            return max(0, min(end - time.monotonic(), request['deadline_unix'] - time.time()) - (0 if cleanup else 30))
        def call(function, maximum=10, *, cleanup=False):
            return probe.bounded_call(function, min(maximum, remaining(cleanup)))
        ledger = validate_launcher(attempt, request, remaining)
        probe.write_new(attempt / 'worker_claim.json', {'job_id': JOB_ID, 'pid': ledger['pid'],
                        'creation_time': ledger['creation_time'], 'claimed_unix': time.time()}, attempt)
        claimed = True
        def elapsed():
            return max(0.0, time.monotonic() - (end - 7200), time.time() - request['created_unix'])
        def state(**changes):
            report.update(changes)
            report['elapsed_seconds'] = elapsed()
            # Running is never falsely advertised as already cleaned.
            if changes.get('status') in ('running', 'cleaning'):
                report['owned_cleanup_verified'] = False
            probe.write_state(attempt, report)
        def check():
            if check_cancel(attempt):
                raise probe.ProbeCancelled('mesh construction cancellation requested')
            if remaining() <= 0:
                raise TimeoutError('mesh construction work deadline reached')
        state(status='running', phase='preflight', created_unix=request['created_unix'],
              deadline_unix=request['deadline_unix'], work_deadline_unix=request['work_deadline_unix'])
        check()
        frozen = call(lambda: probe.validate_frozen(root), 15)
        call(lambda: audit_templates(root, frozen))
        environment = call(lambda: preflight.inspect_environment(frozen['contract'], root=root), 25)
        if environment.get('status') != 'passed':
            raise RuntimeGateError('mesh environment preflight blocked: ' + repr(environment.get('blockers')))
        mesh_attempt = root / MESH_ATTEMPT_RELATIVE
        probe.checked_path(mesh_attempt.parent, root)
        if mesh_attempt.exists():
            raise RuntimeGateError('fourth mesh attempt already exists; never reuse')
        check()
        state(status='running', phase='generate_journal')
        call(lambda: mesh_prepare.generate(mesh_attempt, frozen['cad']['background']['path'], frozen['cad']['arm']['path'],
             cad_verification=frozen['cad']['cad_verification']['path'], contract_path=root / 'contract.json', stage_root=root), 15)
        check()
        def progress(value):
            state(**value)
        code = run_mesh_job.run(mesh_attempt, total_deadline_unix=request['deadline_unix'],
                                cancel_check=lambda: check_cancel(attempt), on_progress=progress)
        execution = call(lambda: probe.load_json(mesh_attempt / 'workbench_execution.json', mesh_attempt), cleanup=True)
        report['owned_cleanup_verified'] = execution.get('owned_cleanup_verified') is True
        report['receipt_paths']['execution'] = str(mesh_attempt / 'workbench_execution.json')
        completion_path = mesh_attempt / 'workbench_completion.json'
        completion = call(lambda: probe.load_json(completion_path, mesh_attempt), cleanup=True) if completion_path.exists() else {}
        report['receipt_paths']['completion'] = str(completion_path)
        if (mesh_attempt / 'export_debug.json').is_file():
            report['receipt_paths']['export_debug'] = str(mesh_attempt / 'export_debug.json')
        report['export_debug'] = completion.get('export_debug', [])
        # Preserve partial actual generation statistics even if export/execution failed.
        stats = []
        for domain in ('background', 'arm'):
            path = mesh_attempt / (domain + '_meshing_stats.json')
            if path.exists():
                item = call(lambda path=path: probe.load_json(path, mesh_attempt), cleanup=True)
                stats.append(item)
                report['meshes'].append({'domain': domain, 'mesh_generated': item.get('mesh_generated') is True,
                    'actual_nodes': item.get('nodes'), 'actual_elements': item.get('elements'),
                    'native_quality': item.get('quality', {}), 'statistics_path': str(path)})
        report['mesh_generated'] = len(stats) == 2 and all(s.get('mesh_generated') is True for s in stats)
        paths = None
        try:
            paths = call(lambda: validate_exports(mesh_attempt, completion), cleanup=True)
            report['mesh_exports_written'] = True
            report['receipt_paths']['meshes'] = [str(p) for p in paths]
        except Exception as exc:
            report['export_inventory_error'] = type(exc).__name__ + ': ' + str(exc)
        if not report['owned_cleanup_verified']:
            report['status'] = 'cleanup_unverified'
            raise RuntimeGateError('Workbench process cleanup unverified; shared lock retained')
        if code != 0:
            report['status'] = execution.get('status', 'failed')
            raise RuntimeGateError(execution.get('error', 'mesh execution failed'))
        if paths is None:
            raise RuntimeGateError(report['export_inventory_error'])
        check()
        manifest = call(lambda: mesh_prepare.collect(mesh_attempt, *paths,
            deadline_unix=request['work_deadline_unix'], cancel_check=lambda: check_cancel(attempt)), max(0, remaining()))
        check()
        report['receipt_paths']['mesh_manifest'] = str(mesh_attempt / 'mesh_manifest.json')
        report['mesh_inventory_status'] = manifest['status']
        report['mesh_quality_verified'] = (manifest['status'] == 'mesh_exported_pending_fluent_static_gate' and
            len(manifest.get('meshes', [])) == 2 and not manifest.get('errors') and
            all(row.get('quality', {}).get('orthogonal_quality', {}).get('source') ==
                'Ansys Meshing MeshMetricType.OrthogonalQuality' for row in manifest['meshes']))
        report['meshes'] = manifest.get('meshes', report['meshes'])
        # Quality-only failures do not erase real generation/export evidence. Structural
        # inventory errors (changed journal, missing counts, etc.) still fail execution evidence.
        quality_errors = {domain + ': ' + detail for domain in ('background', 'arm') for detail in (
            'missing finite native minimum orthogonal quality', 'minimum orthogonal quality below 0.01; no motion')}
        if any(error not in quality_errors for error in manifest.get('errors', [])):
            raise RuntimeGateError('mesh inventory integrity failed: ' + repr(manifest['errors']))
        # completed means the permitted execution/export flow completed, not physical acceptance.
        report['status'] = 'completed'
    except Exception as exc:
        if not claimed:
            print(json.dumps(dict(report, error=type(exc).__name__ + ': ' + str(exc))))
            return 2
        if isinstance(exc, probe.ProbeCancelled):
            report['status'] = 'cancelled'
        elif report['status'] not in ('cancelled', 'cleanup_unverified'):
            report['status'] = 'failed'
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        if not (root / MESH_ATTEMPT_RELATIVE / 'workbench_execution.json').exists():
            # Environment/cancel/shared-lock rejection may happen before the launch
            # region. Its logs are opened before Popen; no logs and no structured
            # launch marker prove this worker never entered that region. Do not
            # touch a possibly unrelated shared lock to make this determination.
            launch_logs = root / MESH_ATTEMPT_RELATIVE
            launch_attempted = (report.get('launch_attempted') is True or
                (launch_logs / 'workbench.stdout.log').exists() or (launch_logs / 'workbench.stderr.log').exists())
            report['launch_attempted'] = launch_attempted
            report['owned_cleanup_verified'] = not launch_attempted
            if launch_attempted:
                report['status'] = 'cleanup_unverified'
    finally:
        if claimed:
            # Real post-cleanup source, CAD and eight-input hashes, still inside the 7200s budget.
            try:
                frozen_after = call(lambda: probe.validate_frozen(root), 10, cleanup=True)
                if call(lambda: validate_authorization(root), 3, cleanup=True) != request['authorization']:
                    raise RuntimeGateError('new mesh authorization changed during execution')
                report['inputs_reverified_after_cleanup'] = frozen_after['protected_inputs'].get('status') == 'passed'
            except Exception as exc:
                report['input_reverification_error'] = type(exc).__name__ + ': ' + str(exc)
            try:
                call(lambda: validate_sources(request['sources'], root), 5, cleanup=True)
                report['sources_reverified_after_cleanup'] = True
            except Exception as exc:
                report['source_reverification_error'] = type(exc).__name__ + ': ' + str(exc)
            if not (report['inputs_reverified_after_cleanup'] and report['sources_reverified_after_cleanup']):
                if report['status'] != 'cleanup_unverified':
                    report['status'] = 'failed'
            report['finished_unix'] = time.time()
            report['elapsed_seconds'] = elapsed()
            report['receipt_paths']['result'] = str(attempt / 'probe_result.json')
            probe.write_new(attempt / 'probe_result.json', report, attempt)
            probe.write_state(attempt, report)
    return 0 if report['status'] == 'completed' else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--preflight', action='store_true')
    mode.add_argument('--run', action='store_true')
    parser.add_argument('--attempt', type=Path)
    args = parser.parse_args(argv)
    if args.preflight:
        if args.attempt is not None:
            parser.error('--preflight does not accept an output attempt')
        report = inspect()
        print(json.dumps(report, ensure_ascii=False, allow_nan=False))
        return 0 if report['status'] == 'passed' else 2
    if args.attempt is None:
        parser.error('--run requires the bridge-owned fixed --attempt')
    return run(args.attempt)


if __name__ == '__main__':
    raise SystemExit(main())

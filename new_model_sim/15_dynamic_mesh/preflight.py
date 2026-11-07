"""Read-only input/environment inspection; never launches CAD, meshing or Fluent.

The only writes are explicitly selected stage15 JSON receipts. lmutil -lmstat is
read-only and does not modify the license manager or check out a solver license.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import shutil
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path

STAGE = Path(__file__).resolve().parent
WORKSPACE = STAGE.parent.parent


class PreflightError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def within(path, root):
    result = Path(path).resolve()
    if not result.is_relative_to(Path(root).resolve()):
        raise PreflightError("path outside authorized root: " + str(result))
    return result


def write_new_json(path, value, root=STAGE):
    path = within(path, root)
    if not path.parent.is_dir():
        raise PreflightError("output parent must already exist: " + str(path.parent))
    encoded = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(encoded)
    return path


def load_contract(path=STAGE / "contract.json"):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if value.get("stage") != "15_dynamic_mesh" or value.get("schema_version") != 1:
        raise PreflightError("unexpected stage/contract schema")
    if value.get("default_theta_replacement_allowed") is not False:
        raise PreflightError("default theta replacement must remain disallowed")
    return value


def snapshot_inputs(contract, contract_path=STAGE / "contract.json", workspace=WORKSPACE):
    files = []
    paths = list(contract["preserve_inputs"])
    contract_path = within(contract_path, workspace)
    paths.append(contract_path.relative_to(Path(workspace).resolve()).as_posix())
    for relative in sorted(set(paths)):
        path = within(Path(workspace) / relative, workspace)
        if not path.is_file():
            raise PreflightError("missing required input: " + relative)
        files.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256(path)})
    return {
        "schema_version": 1,
        "created_local": datetime.now().astimezone().isoformat(),
        "purpose": "immutable inputs only, not a solver or scientific validation",
        "files": files,
    }


def verify_inputs(manifest, workspace=WORKSPACE):
    rows = manifest.get("files")
    if not isinstance(rows, list) or not rows:
        raise PreflightError("empty or missing frozen input list")
    results = []
    seen = set()
    for row in rows:
        relative = row["path"]
        if relative in seen:
            raise PreflightError("duplicate input path: " + relative)
        seen.add(relative)
        path = within(Path(workspace) / relative, workspace)
        expected = row.get("sha256", "")
        if re.fullmatch(r"[0-9a-f]{64}", expected) is None:
            raise PreflightError("invalid stored hash for " + relative)
        actual = sha256(path) if path.is_file() else None
        results.append({"path": relative, "expected_sha256": expected,
                        "actual_sha256": actual, "passed": actual == expected})
    return {"status": "passed" if all(r["passed"] for r in results) else "failed", "files": results}


def inspect_processes():
    try:
        import psutil
        active = []
        skipped = 0
        prefixes = ("fluent", "cortex", "spaceclaim", "runwb2", "ansyswbu", "ansysmeshing")
        for process in psutil.process_iter(["pid", "name", "create_time"]):
            try:
                info = process.info
                if str(info.get("name", "")).lower().startswith(prefixes):
                    active.append({k: info.get(k) for k in ("pid", "name", "create_time")})
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                skipped += 1
        return {"status": "passed", "active_ansys_clients": active, "skipped_processes": skipped,
                "processes_stopped": False}
    except Exception as exc:
        return {"status": "failed", "error": type(exc).__name__ + ": " + str(exc)}


def parse_license_status(returncode, text):
    server_up = re.search(r"license server\s+UP\b", text, re.I) is not None
    vendor_up = re.search(r"ansyslmd\s*:\s*UP\b", text, re.I) is not None
    return {"status": "passed" if returncode == 0 and server_up and vendor_up else "blocked",
            "returncode": returncode, "license_server_up": server_up,
            "ansyslmd_up": vendor_up,
            "feature_availability_and_checkout": "not_verified"}


def inspect_license(contract):
    root = Path(contract["software"]["ansys_root"])
    candidates = [root / "licensingclient/winx64/lmutil.exe", root / "fluent/ntbin/win64/lmutil.exe"]
    executable = next((p for p in candidates if p.is_file()), None)
    result = {"configuration": contract["software"]["license_server"], "service_changes": False,
              "solver_launched": False}
    if executable is None:
        return dict(result, status="blocked", reason="lmutil not found")
    target = contract["software"]["license_server"]
    try:
        port, host = target.split("@", 1)
        with socket.create_connection((host, int(port)), timeout=3):
            result["tcp_connect"] = True
    except Exception as exc:
        result["tcp_connect"] = False
        result["tcp_error"] = type(exc).__name__ + ": " + str(exc)
    command = [str(executable), "lmstat", "-c", target]
    try:
        completed = subprocess.run(command, capture_output=True, timeout=20)
        output = (completed.stdout + completed.stderr).decode("utf-8", errors="replace")
        result.update(parse_license_status(completed.returncode, output))
        result["utility"] = str(executable)
        result["read_only_status_output"] = output[:12000]
    except Exception as exc:
        result.update(status="blocked", reason=type(exc).__name__ + ": " + str(exc))
    return result


def inspect_environment(contract, root=STAGE, license_probe=True):
    software = {name: {"path": contract["software"][name],
                       "exists": Path(contract["software"][name]).is_file()}
                for name in ("python", "fluent", "spaceclaim", "workbench", "matlab")}
    modules = {}
    for name in ("numpy", "scipy", "h5py", "pytest", "psutil", "ansys.fluent.core"):
        try:
            modules[name] = importlib.util.find_spec(name) is not None
        except (ImportError, ModuleNotFoundError, ValueError):
            modules[name] = False
    usage = shutil.disk_usage(root)
    free_gib = usage.free / (1024 ** 3)
    processes = inspect_processes()
    license_status = inspect_license(contract) if license_probe else {
        "status": "not_run", "feature_availability_and_checkout": "not_verified"}
    blockers = []
    if sys.version_info[:2] != (3, 10):
        blockers.append("run with configured ANSYS Python 3.10")
    if not all(row["exists"] for row in software.values()):
        blockers.append("required software executable missing")
    if not all(modules.values()):
        blockers.append("required Python module missing")
    if free_gib < contract["budget"]["minimum_free_disk_gib"]:
        blockers.append("insufficient free disk for configured budget")
    if processes["status"] != "passed":
        blockers.append("process inspection unavailable")
    elif processes["active_ansys_clients"]:
        blockers.append("existing ANSYS client session requires serialization; do not terminate it")
    if license_status["status"] != "passed":
        blockers.append("license server/vendor status not verified ready")
    return {"status": "passed" if not blockers else "blocked", "blockers": blockers,
            "python_executable": sys.executable, "python_version": sys.version,
            "software": software, "modules": modules,
            "disk": {"free_gib": free_gib, "minimum_free_gib": contract["budget"]["minimum_free_disk_gib"]},
            "process_inspection": processes, "licensing": license_status,
            "actual_solver_api_compatibility": "not_verified",
            "cad_launched": False, "fluent_launched": False, "global_environment_changed": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=STAGE / "contract.json")
    parser.add_argument("--manifest", type=Path, default=STAGE / "frozen/input_manifest.json")
    parser.add_argument("--freeze", action="store_true", help="create a new input-only manifest")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    contract_path = within(args.contract, STAGE)
    manifest_path = within(args.manifest, STAGE)
    report_path = within(args.report, STAGE)
    if report_path.exists():
        raise PreflightError("refusing to overwrite receipt: " + str(report_path))
    contract = load_contract(contract_path)
    if args.freeze:
        write_new_json(manifest_path, snapshot_inputs(contract, contract_path), STAGE)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    inputs = verify_inputs(manifest)
    environment = inspect_environment(contract)
    report = {"schema_version": 1, "created_local": datetime.now().astimezone().isoformat(),
              "status": "passed" if inputs["status"] == "passed" and environment["status"] == "passed" else "blocked",
              "scope": "input integrity and software/environment preflight only",
              "manifest": str(manifest_path), "manifest_sha256": sha256(manifest_path),
              "script_sha256": sha256(__file__), "inputs": inputs, "environment": environment,
              "solver_geometry_and_loads_verified": False}
    write_new_json(report_path, report, STAGE)
    print(json.dumps({"status": report["status"], "report": str(report_path),
                      "input_count": len(inputs["files"]), "blockers": environment["blockers"]}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())

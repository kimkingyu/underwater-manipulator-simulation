"""Fixed-scope MCP job bridge. No MCP dependency and no ANSYS launch on import.

Only one persistent definition-probe slot exists. The solver/mesher execution is
owned by definition_probe.py, not by the lifetime of a stdio connection.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import time

STAGE = Path(__file__).resolve().parent
ANSYS_PYTHON = Path(r"D:\Program Files\ANSYS Inc\v242\commonfiles\CPython\3_10\winx64\Release\python\python.exe")
JOB_ID = "definition-probe-001"
ATTEMPT = "runs/mcp_definition_probe/attempt_001"
AUTHORIZATION = "frozen/mcp_definition_probe_authorization_001.json"
START_CONFIRMATION = "RUN_STAGE15_DEFINITION_PROBE_ONCE"
CANCEL_CONFIRMATION = "CANCEL_STAGE15_DEFINITION_PROBE"
IDENTITY_TIMEOUT_SECONDS = 15
LAUNCHER_WAIT_SECONDS = 20
SOURCES = (
    "stage15_mcp.py", "mcp_bridge.py", "definition_probe.py", "runtime.py",
    "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "mesh/wb_definition_probe.wbjn.in", "mesh/meshing_definition_probe.py.in",
)
LOGS = {
    "worker": "worker.log", "workbench_stdout": "workbench.stdout.log",
    "workbench_stderr": "workbench.stderr.log", "trace": "workbench.trace.txt",
    "meshing": "background_meshing_stats.json", "completion": "workbench_completion.json",
}
FIXED_AUTH = {
    "schema_version": 1, "scope": "stage15_mcp_definition_probe",
    "operation": "definition_probe", "maximum_attempts": 1,
    "maximum_total_seconds": 300, "cleanup_reserve_seconds": 30,
    "attempt_relative": ATTEMPT,
    "background_cad_relative": "runs/cad_build/attempt_004/background_fluid.scdoc",
    "arm_cad_relative": "runs/cad_arm_completion/attempt_002/arm_fluid.scdoc",
    "cad_verification_relative": "runs/cad_arm_completion_verify/attempt_001/verify_report.json",
    "full_mesh_allowed": False, "fluent_allowed": False, "automatic_retry_allowed": False,
}
# Read only one PID supplied by our own Popen or immutable launcher ledger.
PID_INSPECTION = """import json,sys,psutil
pid=int(sys.argv[1])
try:
 p=psutil.Process(pid)
 print(json.dumps({'pid':pid,'exists':p.is_running(),'creation_time':p.create_time()}))
except psutil.NoSuchProcess:
 print(json.dumps({'pid':pid,'exists':False}))
"""


class BridgeError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def redact(text):
    # Do not return credential-bearing lines even when native software wrote them.
    return re.sub(r"(?im)^.*(?:(?:password|passwd|api[_-]?key|authorization|secret|token)[\"']?\s*[:=]|bearer\s+\S+).*$",
                  "[redacted credential-bearing line]", str(text))


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")


def read_json(path, limit=2 * 1024 * 1024):
    if not path.is_file() or path.stat().st_size > limit:
        raise BridgeError("Missing or oversized JSON: " + path.name)
    with path.open("rb") as stream:
        raw = stream.read(limit + 1)
    if len(raw) > limit:
        raise BridgeError("JSON grew beyond the permitted limit")
    value = json.loads(raw.decode("utf-8-sig"))
    if not isinstance(value, dict):
        raise BridgeError("Expected a JSON object: " + path.name)
    return value


class Stage15Bridge:
    # Fixed per-instance policy permits a separately authorized diagnostic slot
    # without mutating globals or invalidating the legacy job's read-only routes.
    JOB_ID = JOB_ID
    ATTEMPT = ATTEMPT
    AUTHORIZATION = AUTHORIZATION
    START_CONFIRMATION = START_CONFIRMATION
    CANCEL_CONFIRMATION = CANCEL_CONFIRMATION
    SOURCES = SOURCES
    LOGS = LOGS
    WORKER_SCRIPT = "definition_probe.py"
    OPERATION = "definition_probe"
    CANCEL_OPERATION = "cancel_definition_probe"
    TOTAL_SECONDS = 300
    CLEANUP_RESERVE_SECONDS = 30

    def __init__(self, stage_root=STAGE):
        # Internal injection is for offline tests; no MCP tool accepts this value.
        self.root = Path(stage_root).absolute()
        if self.root.resolve(strict=True) != self.root:
            raise BridgeError("Stage root must not be redirected")
        self.safe("contract.json")

    def safe(self, relative):
        part = Path(relative)
        if part.is_absolute() or ".." in part.parts or ":" in str(relative):
            raise BridgeError("Only fixed stage-relative paths are permitted")
        raw = self.root / part
        for node in (raw, *raw.parents):
            try:
                info = node.lstat()
            except FileNotFoundError:
                info = None
            if info and (stat.S_ISLNK(info.st_mode) or
                         getattr(info, "st_file_attributes", 0) & 0x400):
                raise BridgeError("Symlink/junction/reparse-point path rejected")
            if node == self.root:
                break
        resolved = raw.resolve()
        if resolved != raw or (resolved != self.root and self.root not in resolved.parents):
            raise BridgeError("Path escaped stage15")
        return raw

    def _json(self, relative, **kwargs):
        return read_json(self.safe(relative), **kwargs)

    def _python(self):
        configured = Path(self._json("contract.json")["software"]["python"])
        if configured.resolve() != ANSYS_PYTHON.resolve():
            raise BridgeError("The fixed ANSYS Python interpreter changed")
        if not ANSYS_PYTHON.is_file():
            raise BridgeError("Configured ANSYS Python is missing")
        return str(ANSYS_PYTHON)

    def authorize(self):
        grant = self._json(self.AUTHORIZATION)
        for name, expected in FIXED_AUTH.items():
            if type(grant.get(name)) is not type(expected) or grant[name] != expected:
                raise BridgeError("Authorization scope mismatch: " + name)
        pairs = {
            "background_sha256": FIXED_AUTH["background_cad_relative"],
            "arm_sha256": FIXED_AUTH["arm_cad_relative"],
            "cad_verification_sha256": FIXED_AUTH["cad_verification_relative"],
            "input_manifest_sha256": "frozen/input_manifest.json",
            "contract_sha256": "contract.json",
        }
        for key, relative in pairs.items():
            expected = grant.get(key)
            if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
                raise BridgeError("Required input hash missing: " + key)
            if sha256(self.safe(relative)) != expected:
                raise BridgeError("Authorized input changed: " + relative)
        return grant

    def _environment(self):
        env = dict(os.environ)
        # Python path injection and user site packages must not change the worker.
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        env.update(PYTHONIOENCODING="utf-8", PYTHONUTF8="1", PYTHONNOUSERSITE="1",
                   PYTHONDONTWRITEBYTECODE="1")
        return env

    def _inspect_pid(self, pid):
        if type(pid) is not int or pid <= 0:
            raise BridgeError("Invalid owned PID")
        result = subprocess.run([self._python(), "-B", "-c", PID_INSPECTION, str(pid)],
                                stdin=subprocess.DEVNULL, capture_output=True, timeout=IDENTITY_TIMEOUT_SECONDS,
                                env=self._environment(), cwd=str(self.root), shell=False)
        if result.returncode != 0:
            raise BridgeError("Read-only worker identity inspection failed")
        value = json.loads(result.stdout.decode("utf-8-sig"))
        if value.get("pid") != pid or type(value.get("exists")) is not bool:
            raise BridgeError("Malformed process inspection")
        if value["exists"]:
            stamp = value.get("creation_time")
            if type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp <= 0:
                raise BridgeError("Unverified process creation time")
        return value

    def _sources(self):
        paths = [self.safe(name) for name in self.SOURCES]
        # Fixed, previously used environment builder; never caller-supplied.
        helper = self.root.parent / "04_workbench/run_wb_fullenv.py"
        if helper.resolve(strict=True) != helper.absolute() or helper.is_symlink():
            raise BridgeError("Workbench environment builder redirected")
        paths.append(helper)
        return [{"path": str(path), "size": path.stat().st_size, "sha256": sha256(path)}
                for path in paths]

    def status(self):
        path = self.safe("reports/stage15_status.json")
        baseline = read_json(path) if path.exists() else {}
        used = self.safe(self.ATTEMPT).exists()
        result = {
            "status": "available", "service": "ansys-stage15", "allowed_root": str(self.root),
            "scope": "single background definition probe only; not mesh or CFD validation",
            "stage_status": baseline.get("overall_status", "unknown"),
            "gates": baseline.get("gates", {}),
            "mesh_attempts_used": len(baseline.get("mesh_attempts", [])),
            "definition_probe_slot_reserved": used, "remaining_definition_probes": 0 if used else 1,
            "authorization_present": self.safe(self.AUTHORIZATION).is_file(),
            "task_lock_present": self.safe(".ansys-task.lock").exists(),
            "full_mesh_authorized": False, "solver_authorized": False,
            "mesh_motion_verified": False, "actual_flow_steps": 0,
            "start_confirmation": self.START_CONFIRMATION, "cancel_confirmation": self.CANCEL_CONFIRMATION,
        }
        if used:
            result["job"] = self.job_status(self.JOB_ID)
        return result

    def preflight(self):
        self.authorize()
        worker = self.safe(self.WORKER_SCRIPT)
        if not worker.is_file():
            raise BridgeError("Definition probe worker missing")
        completed = subprocess.run([self._python(), "-B", str(worker), "--preflight"],
                                   stdin=subprocess.DEVNULL, capture_output=True, timeout=45,
                                   env=self._environment(), cwd=str(self.root), shell=False)
        if len(completed.stdout) > 2 * 1024 * 1024:
            raise BridgeError("Oversized preflight result")
        try:
            report = json.loads(completed.stdout.decode("utf-8-sig"))
        except (ValueError, UnicodeError):
            raise BridgeError("Preflight did not return a JSON receipt: " +
                              redact(completed.stderr.decode("utf-8", errors="replace")[-3000:]))
        if not isinstance(report, dict):
            raise BridgeError("Preflight result is not an object")
        if completed.returncode != 0 and report.get("status") == "passed":
            raise BridgeError("Preflight exit status contradicts its receipt")
        report["worker_exit_code"] = completed.returncode
        report["slot_reserved"] = self.safe(self.ATTEMPT).exists()
        return report

    def make_request(self, created, sources):
        return {"schema_version": 1, "operation": self.OPERATION, "job_id": self.JOB_ID,
                "created_unix": created, "deadline_unix": created + self.TOTAL_SECONDS,
                "work_deadline_unix": created + self.TOTAL_SECONDS - self.CLEANUP_RESERVE_SECONDS,
                "authorization": {"path": str(self.safe(self.AUTHORIZATION)),
                                  "sha256": sha256(self.safe(self.AUTHORIZATION))}, "sources": sources}
    def archive_sources(self, request):
        archive = self.safe(self.ATTEMPT + "/source_snapshot")
        archive.mkdir(exist_ok=False)
        manifest = []
        for row in request["sources"]:
            source = Path(row["path"])
            if self.root in source.parents:
                relative = source.relative_to(self.root)
            elif source == self.root.parent / "04_workbench/run_wb_fullenv.py":
                relative = Path("external") / source.name
            else:
                raise BridgeError("Unexpected archival source")
            destination = self.safe(self.ATTEMPT + "/source_snapshot/" + relative.as_posix())
            destination.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as original, destination.open("xb") as saved:
                shutil.copyfileobj(original, saved)
            if sha256(destination) != row["sha256"]:
                raise BridgeError("Source changed during create-only archival")
            manifest.append({"original_path": str(source), "archive_relative": relative.as_posix(),
                             "sha256": row["sha256"], "size": row["size"]})
        write_new(self.safe(self.ATTEMPT + "/source_snapshot/archive_manifest.json"), {
            "job_id": self.JOB_ID, "purpose": "immutable source evidence for this job only", "files": manifest})
    def preserve_sources(self, request):
        """Optional create-only per-slot archival hook; legacy artifacts stay untouched."""
        return None
    def start(self, confirmation):
        if confirmation != self.START_CONFIRMATION:
            raise BridgeError("Explicit definition-probe confirmation required")
        attempt = self.safe(self.ATTEMPT)
        if attempt.exists():
            return {"status": "already_reserved", "new_worker_started": False,
                    "remaining_definition_probes": 0, "job": self.job_status(self.JOB_ID)}
        self.authorize()
        if self.safe(".ansys-task.lock").exists():
            raise BridgeError("Existing ANSYS ownership lock: no new task permitted")
        python = self._python()
        sources = self._sources()
        self.safe(Path(self.ATTEMPT).parent).mkdir(parents=True, exist_ok=True)
        # mkdir is also the cross-process reservation. No failed slot is recycled.
        try:
            attempt.mkdir(exist_ok=False)
        except FileExistsError:
            return {"status": "already_reserved", "new_worker_started": False,
                    "remaining_definition_probes": 0, "job": self.job_status(self.JOB_ID)}
        created = time.time()
        process = None
        try:
            request = self.make_request(created, sources)
            write_new(self.safe(self.ATTEMPT + "/request.json"), request)
            self.preserve_sources(request)
            write_new(self.safe(self.ATTEMPT + "/job_state.json"), {
                "job_id": self.JOB_ID, "status": "reserved", "phase": "worker_start_pending",
                "created_unix": created, "deadline_unix": request["deadline_unix"],
                "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False,
                "owned_cleanup_verified": False})
            options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}
            with self.safe(self.ATTEMPT + "/worker.log").open("xb") as log:
                process = subprocess.Popen(
                    [python, "-B", str(self.safe(self.WORKER_SCRIPT)), "--run", "--attempt", str(attempt)],
                    cwd=str(self.root), env=self._environment(), stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, shell=False, close_fds=True, **options)
            identity = self._inspect_pid(process.pid)
            if not identity["exists"]:
                raise BridgeError("Worker exited before ownership was recorded")
            write_new(self.safe(self.ATTEMPT + "/launcher.json"), {
                "job_id": self.JOB_ID, "pid": process.pid, "creation_time": identity["creation_time"],
                "recorded_unix": time.time()})
            return {"status": "accepted", "job_id": self.JOB_ID, "new_worker_started": True,
                    "deadline_unix": request["deadline_unix"], "total_budget_seconds": self.TOTAL_SECONDS,
                    "remaining_definition_probes": 0, "mesh_generated": False, "flow_iterations": 0}
        except Exception as exc:
            # No broad process termination here. A worker without a matching
            # launcher ledger must refuse to launch any ANSYS client itself.
            failure = {"job_id": self.JOB_ID, "status": "failed" if process is None else "cleanup_unverified",
                       "phase": "worker_launch_failed", "error": redact(type(exc).__name__ + ": " + str(exc)),
                       "owned_cleanup_verified": process is None,
                       "mesh_generated": False, "flow_iterations": 0, "remaining_definition_probes": 0}
            write_new(self.safe(self.ATTEMPT + "/launch_failure.json"), failure)
            if process is not None:
                try:
                    write_new(self.safe(self.ATTEMPT + "/cancel_request.json"), {
                        "job_id": self.JOB_ID, "operation": self.CANCEL_OPERATION, "created_unix": time.time()})
                except FileExistsError:
                    pass
            return failure

    def _job_path(self, job_id):
        if job_id != self.JOB_ID:
            raise BridgeError("Unknown job_id; arbitrary paths/PIDs are not accepted")
        path = self.safe(self.ATTEMPT)
        if not path.is_dir():
            raise BridgeError("Definition probe has not been reserved")
        return path

    def job_status(self, job_id):
        self._job_path(job_id)
        result_path = self.safe(self.ATTEMPT + "/probe_result.json")
        result = read_json(result_path) if result_path.exists() else None
        state_path = self.safe(self.ATTEMPT + "/job_state.json")
        state = read_json(state_path) if state_path.exists() else {
            "job_id": self.JOB_ID, "status": "reserved", "phase": "reservation_incomplete"}
        if state.get("job_id") != job_id or (result is not None and result.get("job_id") != job_id):
            raise BridgeError("Job receipt identity mismatch")
        failure_path = self.safe(self.ATTEMPT + "/launch_failure.json")
        if result is None and failure_path.exists():
            state = read_json(failure_path)
            if state.get("job_id") != job_id:
                raise BridgeError("Launch failure identity mismatch")
        no_worker_launched = (state.get("phase") == "worker_launch_failed" and
                              state.get("owned_cleanup_verified") is True)
        answer = dict(state)
        answer.update(job_id=self.JOB_ID, remaining_definition_probes=0,
                      mesh_generated=False, flow_iterations=0, mesh_motion_verified=False)
        if result is not None:
            answer["result"] = result
            answer["status"] = result.get("status", "cleanup_unverified")
            answer["owned_cleanup_verified"] = result.get("owned_cleanup_verified") is True
            if answer["status"] == "passed" and not answer["owned_cleanup_verified"]:
                answer["status"] = "cleanup_unverified"
            elif answer["status"] == "passed":
                expected_flags = {"mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False}
                if any(type(result.get(key)) is not type(value) or result.get(key) != value
                       for key, value in expected_flags.items()):
                    answer.update(status="failed", phase="final_scope_evidence_missing_or_contradictory")
                for key in expected_flags:
                    answer[key] = result.get(key)
                definition = result.get("definition_receipt", {})
                for checkpoint in ("mesh_counts_before", "mesh_counts_after"):
                    counts = definition.get(checkpoint, {}) if isinstance(definition, dict) else {}
                    if not isinstance(counts, dict) or any(type(counts.get(key)) is not int or counts[key] != 0
                                                          for key in ("nodes", "elements")):
                        answer.update(status="failed", phase="native_zero_mesh_counts_missing_or_nonzero")
        launcher_path = self.safe(self.ATTEMPT + "/launcher.json")
        if launcher_path.exists():
            owner = read_json(launcher_path)
            if owner.get("job_id") != job_id:
                raise BridgeError("Launcher identity mismatch")
            try:
                live = self._inspect_pid(owner["pid"])
                alive = live["exists"] and live.get("creation_time") == owner.get("creation_time")
                answer["worker_alive_verified"] = alive
                answer["worker_pid_reused"] = live["exists"] and not alive
                if not alive and result is None:
                    answer.update(status="cleanup_unverified", phase="worker_lost_without_final_receipt",
                                  owned_cleanup_verified=False)
            except Exception as exc:
                answer["worker_identity_error"] = redact(str(exc))
                answer["worker_alive_verified"] = None
        elif result is None and not no_worker_launched and time.time() - float(state.get("created_unix", 0)) > LAUNCHER_WAIT_SECONDS:
            answer.update(status="cleanup_unverified", phase="launcher_ledger_missing",
                          owned_cleanup_verified=False)
        answer["task_lock_present"] = self.safe(".ansys-task.lock").exists()
        if result is None and time.time() > float(state.get("deadline_unix", float("inf"))):
            answer.update(status="cleanup_unverified", phase="deadline_exceeded_without_final_receipt",
                          owned_cleanup_verified=False)
        return answer

    def log_path(self, stream):
        return self.safe(self.ATTEMPT + "/" + self.LOGS[stream])
    def job_log(self, job_id, stream="trace", limit=100):
        self._job_path(job_id)
        if stream not in self.LOGS or type(limit) is not int or not 1 <= limit <= 200:
            raise BridgeError("Log stream must be allow-listed and limit must be 1..200")
        path = self.log_path(stream)
        if not path.is_file():
            return {"job_id": job_id, "stream": stream, "status": "not_available", "text": ""}
        size = path.stat().st_size
        with path.open("rb") as handle:
            handle.seek(max(0, size - 65536))
            raw = handle.read(65536)
        encoding = "utf-16" if size <= 65536 and raw[:2] in (b"\xff\xfe", b"\xfe\xff") else "utf-8-sig"
        decoded = raw.decode(encoding, errors="replace")
        if size > 65536:
            # The omitted beginning might contain a credential label; never
            # expose the tail of an unclassifiable partial first line.
            decoded = decoded.split("\n", 1)[1] if "\n" in decoded else ""
        text = "\n".join(decoded.splitlines()[-limit:])
        text = redact(text)[-20000:]
        return {"job_id": job_id, "stream": stream, "status": "available", "text": text,
                "bounded": True, "max_lines": limit, "max_characters": 20000, "file_bytes": size}

    def cancel(self, job_id, confirmation):
        if confirmation != self.CANCEL_CONFIRMATION:
            raise BridgeError("Explicit cancellation confirmation required")
        self._job_path(job_id)
        current = self.job_status(job_id)
        if current.get("result") is not None:
            return {"status": "already_finished", "job_id": job_id, "job": current}
        if current.get("worker_alive_verified") is not True:
            return {"status": "cleanup_unverified", "phase": "worker_unavailable_no_signal",
                    "job_id": job_id, "cleanup_verified": False, "requires_operator_review": True,
                    "lock_must_be_preserved": True, "retry_permitted": False,
                    "note": "No process signalled. Preserve the slot and lock until ownership/cleanup is verified.",
                    "job": current}
        try:
            write_new(self.safe(self.ATTEMPT + "/cancel_request.json"), {
                "job_id": job_id, "operation": self.CANCEL_OPERATION, "created_unix": time.time()})
        except FileExistsError:
            existing = self._json(self.ATTEMPT + "/cancel_request.json")
            if existing.get("job_id") != job_id or existing.get("operation") != self.CANCEL_OPERATION:
                raise BridgeError("Existing cancellation request identity mismatch")
        return {"status": "cancellation_requested", "job_id": job_id,
                "cleanup_verified": False, "note": "Wait for worker final cleanup evidence; no arbitrary PID kill."}

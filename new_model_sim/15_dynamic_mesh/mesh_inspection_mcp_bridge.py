"""Two separately consumed read-only Fluent inspection slots; never a mesh grant."""
from __future__ import annotations

from pathlib import Path
import json
import subprocess
import queue
import threading
import time
import uuid
from mcp_bridge import Stage15Bridge, BridgeError, WorkerLaunchUncertain, STAGE, sha256, write_new

GRANT = "frozen/mesh_inspection_authorization_002.json"
MANIFEST = "frozen/mesh_inspection_sources_002.json"
START_TOKEN = "RUN_STAGE15_READ_ONLY_MESH_INSPECTION_ONCE"
CANCEL_TOKEN = "CANCEL_STAGE15_READ_ONLY_MESH_INSPECTION"
SOURCES = ("stage15_mcp.py", "mcp_bridge.py", "sizing_mcp_bridge.py", "mesh_mcp_bridge.py",
           "mesh_inspection_mcp_bridge.py", "mesh_inspection.py", "mesh_inspection_evidence.py", "mesh_inspection_connection.py",
           "definition_probe.py", "runtime.py", "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
           "frozen/mesh_inspection_native_api_basis_002.json")
MESHES = {
    "background": {"path": "runs/mesh/attempt_004/background.msh", "size": 380257330,
                   "sha256": "cbbc3d5b3ad88caffe9691307b6df74b5d51af94d9eaffd5e68051d38f1eefb2",
                   "fluid": "fluid_background", "walls": ["wall_auv", "wall_base0", "tank_wall"]},
    "arm": {"path": "runs/mesh/attempt_004/arm.msh", "size": 38443395,
            "sha256": "1dffa98e02b80a9813c37d43e746ced5cf501cd0032580a04a69726b419bd8d5",
            "fluid": "fluid_arm", "walls": ["wall_jaw1", "wall_jaw2", "wall_l1", "wall_l2", "wall_link3", "wall_pin4"]},
}
AUTH_VALUES = {
    "schema_version": 1, "authorization_revision": 2,
    "baseline_authorization_relative": "frozen/mesh_inspection_authorization_001.json",
    "baseline_authorization_sha256": "8dca86e3752484b6470b8ff1495963a80a5aa98bf270b6ce088b261c54382eda",
    "additional_attempts_authorized": 0,
    "scope": "stage15_mcp_read_only_mesh_inspection", "operation": "mesh_inspection",
    "domains": ["background", "arm"], "maximum_attempts": 2, "maximum_attempts_per_domain": 1,
    "maximum_total_seconds_per_domain": 300, "cleanup_reserve_seconds": 30,
    "native_start_timeout_seconds": 45, "mesh_generation_allowed": False, "mesh_write_allowed": False,
    "solver_read_only_allowed": True, "initialization_allowed": False, "iterations_allowed": False,
    "time_advance_allowed": False, "UDF_allowed": False, "motion_allowed": False,
    "overset_assembly_allowed": False, "automatic_retry_allowed": False, "CFD_allowed": False,
    "source_manifest_relative": MANIFEST, "meshes": MESHES,
}


class InspectionSlot(Stage15Bridge):
    AUTHORIZATION = GRANT
    START_CONFIRMATION = START_TOKEN
    CANCEL_CONFIRMATION = CANCEL_TOKEN
    SOURCES = SOURCES
    WORKER_SCRIPT = "mesh_inspection.py"
    OPERATION = "mesh_inspection"
    CANCEL_OPERATION = "cancel_mesh_inspection"
    LOGS = {"worker": "worker.log", "trace": "native.transcript.txt", "native": "native_result.json",
            "execution": "native.stdout.log", "native_stderr": "native.stderr.log", "plan": "command_audit.json"}

    def __init__(self, domain, stage_root=STAGE):
        if domain not in MESHES:
            raise BridgeError("Only background or arm is authorized")
        super().__init__(stage_root)
        self.domain = domain
        self.JOB_ID = "mesh-inspection-001-" + domain
        self.ATTEMPT = "runs/mcp_mesh_inspection/batch_001/" + domain

    def authorize(self):
        grant = self._json(GRANT)
        if set(grant) != set(AUTH_VALUES) | {"authorization_source", "recorded_local", "execution_policy", "source_manifest_sha256"}:
            raise BridgeError("Unexpected mesh inspection grant fields")
        for key, value in AUTH_VALUES.items():
            if type(grant.get(key)) is not type(value) or grant[key] != value:
                raise BridgeError("Inspection grant scope mismatch: " + key)
        for key in ("authorization_source", "recorded_local", "execution_policy"):
            if not isinstance(grant[key], str) or not 1 <= len(grant[key]) <= 4000:
                raise BridgeError("Missing authorization provenance")
        if sha256(self.safe(grant["baseline_authorization_relative"])) != grant["baseline_authorization_sha256"]:
            raise BridgeError("Original read-only grant binding changed; no budget reset allowed")
        if sha256(self.safe(MANIFEST)) != grant["source_manifest_sha256"]:
            raise BridgeError("Inspection source manifest changed")
        expected = self._sources()
        if self._json(MANIFEST) != {"schema_version": 1, "files": expected}:
            raise BridgeError("Inspection frozen source/size/hash changed")
        for row in MESHES.values():
            path = self.safe(row["path"])
            if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]:
                raise BridgeError("Existing mesh size/hash changed: " + row["path"])
        return grant

    def make_request(self, created, sources):
        value = super().make_request(created, sources)
        value.update(domain=self.domain, mesh=MESHES[self.domain])
        return value

    def preserve_sources(self, request):
        self.archive_sources(request)

    def preflight(self):
        # No worker/solver launch, no slot creation; all actual inputs are read.
        self.authorize()
        completed = subprocess.run([self._python(), "-B", str(self.safe(self.WORKER_SCRIPT)),
                                    "--preflight", "--domain", self.domain], stdin=subprocess.DEVNULL,
                                   capture_output=True, timeout=45, cwd=str(self.root), env=self._environment(), shell=False)
        if len(completed.stdout) > 2 * 1024 * 1024:
            raise BridgeError("Oversized inspection preflight")
        report = json.loads(completed.stdout.decode("utf-8-sig"))
        if not isinstance(report, dict) or completed.returncode != 0:
            raise BridgeError("Inspection read-only preflight failed")
        return report

    WORKER_LAUNCH_TIMEOUT = 10

    def launch_worker(self, command, **options):
        output = queue.Queue(maxsize=1)
        def launch():
            try:
                output.put((True, super(InspectionSlot, self).launch_worker(command, **options)))
            except BaseException as exc:
                output.put((False, exc))
        threading.Thread(target=launch, daemon=True).start()
        try:
            ok, value = output.get(timeout=self.WORKER_LAUNCH_TIMEOUT)
        except queue.Empty as exc:
            raise WorkerLaunchUncertain("Worker Popen exceeded 10 seconds; retain slot and dispatch lock") from exc
        if not ok:
            raise value
        return value

    def start(self, confirmation):
        if confirmation != START_TOKEN:
            raise BridgeError("Explicit read-only mesh inspection confirmation required")
        dispatch = self.safe(".mesh-inspection-dispatch.lock")
        token = None
        if not self.safe(self.ATTEMPT).exists():
            if self.safe(".sizing-dispatch.lock").exists():
                raise BridgeError("Sizing dispatch is pending")
            self.preflight()
            token = uuid.uuid4().hex
            try:
                write_new(dispatch, {"job_id": self.JOB_ID, "token": token, "created_unix": time.time()})
            except FileExistsError:
                return {"status": "dispatch_pending", "new_worker_started": False, "domain": self.domain,
                        "lock_must_be_preserved": True, "CFD_authorized": False}
        value = None
        try:
            value = super().start(confirmation)
            value.pop("remaining_definition_probes", None)
            value.update(domain=self.domain, CFD_authorized=False, mesh_generation_authorized=False)
            return value
        finally:
            # An uncertain worker can never be adopted/retried, even if no PID
            # was returned. It also has no launcher ledger to launch Fluent.
            release = (value is not None and (value.get("status") in ("accepted", "already_reserved") or
                       value.get("owned_cleanup_verified") is True)) or not self.safe(self.ATTEMPT).exists()
            if token is not None and release and dispatch.exists() and self._json(dispatch.name).get("token") == token:
                dispatch.unlink()

    def status(self):
        used = self.safe(self.ATTEMPT).exists()
        result = {"domain": self.domain, "job_id": self.JOB_ID, "slot_reserved": used,
                  "remaining_inspections": 0 if used else 1, "maximum_seconds_including_cleanup": 300,
                  "dispatch_lock_present": self.safe(".mesh-inspection-dispatch.lock").exists(),
                  "start_confirmation": START_TOKEN, "cancel_confirmation": CANCEL_TOKEN,
                  "mesh_generation_authorized": False, "CFD_authorized": False}
        if used:
            result["job"] = self.job_status(self.JOB_ID)
        return result

    def job_status(self, job_id):
        value = super().job_status(job_id)
        value.pop("remaining_definition_probes", None)
        value.update(domain=self.domain, mesh_generated=False, flow_iterations=0,
                     mesh_motion_verified=False, CFD_authorized=False)
        request_path = self.safe(self.ATTEMPT + "/request.json")
        if request_path.exists():
            saved_request = self._json(self.ATTEMPT + "/request.json")
            value["slot_authorization"] = saved_request.get("authorization")
            value["slot_source_count"] = len(saved_request.get("sources", []))
            try:
                value["slot_sources_match_current_code"] = saved_request.get("sources") == self._sources()
            except (OSError, BridgeError):
                value["slot_sources_match_current_code"] = False
            value["historical_receipt_not_rebound"] = saved_request.get("authorization", {}).get("path") != str(self.safe(GRANT))
        result = value.get("result")
        if isinstance(result, dict):
            from mesh_inspection_evidence import validate_native_evidence, parse_native_output
            if result.get("operation") != self.OPERATION or result.get("domain") != self.domain:
                raise BridgeError("Inspection receipt identity mismatch")
            if (result.get("mesh_generated") is not False or type(result.get("flow_iterations")) is not int or
                    result.get("flow_iterations") != 0 or result.get("mesh_motion_verified") is not False):
                value.update(status="failed", phase="inspection_scope_contradiction")
            native = result.get("native", {})
            artifacts_bound = False
            try:
                transcript = self.safe(self.ATTEMPT + "/native.transcript.txt")
                artifacts_bound = (isinstance(native, dict) and transcript.stat().st_size <= 2 * 1024 * 1024 and
                    self._json(self.ATTEMPT + "/native_result.json") == native and
                    native.get("job_id") == job_id and native.get("domain") == self.domain and
                    native.get("transcript_sha256") == sha256(transcript) and
                    native.get("metrics") == parse_native_output(transcript.read_text(encoding="utf-8", errors="replace")) and
                    result.get("solver_launched") is True and bool(result.get("owned_process_identity")))
            except (OSError, ValueError, BridgeError):
                pass
            evidence = validate_native_evidence(native if artifacts_bound else {}, self.domain)
            evidence["native_artifacts_bound"] = artifacts_bound
            value["inspection"] = evidence
            if result.get("status") == "completed" and not all(result.get(key) is True for key in
                    ("owned_cleanup_verified", "inputs_reverified_after_cleanup", "sources_reverified_after_cleanup")):
                value.update(status="failed", phase="inspection_final_evidence_missing")
            # A native mock/test receipt is never accepted as a real diagnostic.
            if result.get("status") == "completed" and result.get("native", {}).get("provenance") != "live_fluent_read_only":
                value.update(status="failed", phase="native_provenance_missing")
        return value


def inspection_job(job_id, stage_root=STAGE):
    for domain in MESHES:
        slot = InspectionSlot(domain, stage_root)
        if slot.JOB_ID == job_id:
            return slot
    raise BridgeError("Unknown inspection job ID")

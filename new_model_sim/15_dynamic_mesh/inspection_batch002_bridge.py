"""New explicit grant: exactly connection/background/arm slots, not old retries."""
from __future__ import annotations
import json
from pathlib import Path
import subprocess
import time
import uuid

from mcp_bridge import STAGE, Stage15Bridge, BridgeError, sha256, read_json, write_new
from mesh_inspection_mcp_bridge import InspectionSlot, MESHES

BATCH = "inspection-batch002"
SLOTS = ("connection", "background", "arm")
GRANT = "frozen/mesh_inspection_transcript_repair_authorization_001.json"
MANIFEST = "frozen/mesh_inspection_transcript_repair_sources_001.json"
SUPPLEMENT = "connection_supplement_001"
SUPPLEMENT_JOB = "inspection-b002-connection-supplement-001"
SUPPLEMENT_ATTEMPT = "runs/mcp_mesh_inspection/batch_002/connection_supplement_001"
SUPPLEMENT_START = "RUN_STAGE15_BATCH002_SUPPLEMENT_CONNECTION_ONCE"
SUPPLEMENT_CANCEL = "CANCEL_STAGE15_BATCH002_SUPPLEMENT_CONNECTION"
STDIN_VALIDATION = "connection_stdin_validation_001"
STDIN_VALIDATION_JOB = "inspection-b002-stdin-validation-001"
STDIN_VALIDATION_ATTEMPT = "runs/mcp_mesh_inspection/batch_002/connection_stdin_validation_001"
STDIN_VALIDATION_START = "RUN_STAGE15_STDIN_CONNECTION_VALIDATION_ONCE"
STDIN_VALIDATION_CANCEL = "CANCEL_STAGE15_STDIN_CONNECTION_VALIDATION"
EXECUTION_SLOTS = SLOTS + (SUPPLEMENT, STDIN_VALIDATION)
START = "RUN_STAGE15_BATCH002_READ_ONLY_ONCE"
CANCEL = "CANCEL_STAGE15_BATCH002_READ_ONLY"
SOURCES = ("stage15_mcp.py", "mcp_bridge.py", "sizing_mcp_bridge.py", "mesh_mcp_bridge.py",
    "mesh_inspection_mcp_bridge.py", "mesh_inspection_evidence.py", "mesh_inspection_connection.py",
    "inspection_batch002_bridge.py", "inspection_batch002.py", "inspection_batch002_connection.py",
    "definition_probe.py", "runtime.py", "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "frozen/mesh_inspection_transcript_repair_api_basis_001.json", "contract.json", "frozen/input_manifest.json")
POLICY = {"schema_version": 1, "binding_revision": 7,
    "baseline_batch_authorization_relative": "frozen/mesh_inspection_stdin_validation_authorization_001.json",
    "baseline_batch_authorization_sha256": "2a79bbcc90697d3c04c48e117f2e45883faaa255fd355b017d4f815fc54cea45",
    "additional_attempts_authorized": 0, "additional_connection_attempts": 0, "additional_QA_attempts": 0,
    "stdin_validation_retired": True, "original_stdin_validation_attempts": 1,
    "transcript_target_policy": "fixed_fresh_attempt_file_absolute_forward_slashes_unquoted",
    "stdin_validation_slot": STDIN_VALIDATION, "stdin_validation_job_id": STDIN_VALIDATION_JOB,
    "stdin_validation_attempt_relative": STDIN_VALIDATION_ATTEMPT,
    "original_additional_connection_attempts": 1, "supplement_connection_retired": True,
    "native_stdin": "private_PIPE_no_writes_until_cleanup",
    "supplement_slot": SUPPLEMENT, "supplement_job_id": SUPPLEMENT_JOB,
    "supplement_attempt_relative": SUPPLEMENT_ATTEMPT, "connection_gate_job_id": STDIN_VALIDATION_JOB,
    "old_connection_retired": True, "remaining_original_QA_slots": ["background", "arm"],
    "batch_id": BATCH, "operation": "read_only_connection_then_mesh_inspection",
    "slots": list(SLOTS), "maximum_attempts": 3, "maximum_attempts_per_slot": 1,
    "maximum_total_seconds_per_slot": 300, "cleanup_reserve_seconds": 30,
    "native_start_timeout_seconds": 45, "transport_ready_max_seconds": 20,
    "health_call_max_seconds": 2, "maximum_health_polls": 200,
    "completed_unavailable_health_polling_allowed": True, "maximum_simultaneous_jobs": 1,
    "connection_gate_required": True, "remoting_server_address": "127.0.0.1",
    "server_info_host_aliases": ["127.0.0.1", "localhost"],
    "alias_resolution": "ascii_casefold_no_dns_to_127.0.0.1",
    "loopback_endpoint_required": True, "automatic_retry_allowed": False,
    "mesh_generation_allowed": False, "mesh_write_allowed": False, "initialization_allowed": False,
    "iterations_allowed": False, "motion_allowed": False, "time_advance_allowed": False,
    "UDF_allowed": False, "overset_assembly_allowed": False, "CFD_allowed": False,
    "mesh_inputs": MESHES, "source_manifest_relative": MANIFEST,
    "old_slots_reusable": False}


class Batch002Slot(InspectionSlot):
    AUTHORIZATION = GRANT
    SOURCES = SOURCES
    START_CONFIRMATION = START
    CANCEL_CONFIRMATION = CANCEL
    CANCEL_OPERATION = "cancel_inspection_batch002"
    WORKER_SCRIPT = "inspection_batch002.py"
    OPERATION = "mesh_inspection"  # original bridge transport; batch_id is mandatory below
    LOGS = dict(InspectionSlot.LOGS, completion="readiness.json")

    def __init__(self, slot, stage_root=STAGE):
        if slot not in SLOTS:
            raise BridgeError("Only connection/background/arm batch002 slots are allowed")
        Stage15Bridge.__init__(self, stage_root)
        self.slot = slot
        self.domain = slot
        self.JOB_ID = "inspection-b002-" + slot
        self.ATTEMPT = "runs/mcp_mesh_inspection/batch_002/" + slot
        self.is_connection = slot == "connection"

    def authorize(self):
        grant = self._json(GRANT)
        metadata = {"authorization_source", "recorded_local", "execution_policy", "source_manifest_sha256"}
        if set(grant) != set(POLICY) | metadata:
            raise BridgeError("Batch002 grant fields differ from fixed policy")
        for key, expected in POLICY.items():
            if type(grant.get(key)) is not type(expected) or grant[key] != expected:
                raise BridgeError("Batch002 grant mismatch: " + key)
        for key in metadata - {"source_manifest_sha256"}:
            if not isinstance(grant[key], str) or not 1 <= len(grant[key]) <= 4000:
                raise BridgeError("Missing batch002 user provenance")
        if sha256(self.safe(grant["baseline_batch_authorization_relative"])) != grant["baseline_batch_authorization_sha256"]:
            raise BridgeError("Original batch002 three-slot authorization binding changed")
        if sha256(self.safe(MANIFEST)) != grant["source_manifest_sha256"] or self._json(MANIFEST) != {
                "schema_version": 1, "files": self._sources()}:
            raise BridgeError("Batch002 frozen source/hash changed")
        for row in MESHES.values():
            path = self.safe(row["path"])
            if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]:
                raise BridgeError("Batch002 original mesh changed")
        return grant

    def make_request(self, created, sources):
        value = Stage15Bridge.make_request(self, created, sources)
        value.update(batch_id=BATCH, slot=self.slot, mesh=None if self.is_connection else MESHES[self.slot])
        return value

    def preserve_sources(self, request):
        import shutil
        self.archive_sources(request)
        rows = []
        for relative in (GRANT, MANIFEST):
            source = self.safe(relative)
            target = self.safe(self.ATTEMPT + "/source_snapshot/execution_binding/" + Path(relative).name)
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as original, target.open("xb") as saved:
                shutil.copyfileobj(original, saved)
            digest = sha256(source)
            if sha256(target) != digest:
                raise BridgeError("Batch002 execution binding changed during archival")
            rows.append({"original": relative, "archive_relative": "execution_binding/" + source.name, "sha256": digest})
        write_new(self.safe(self.ATTEMPT + "/source_snapshot/execution_binding.json"), {"job_id": self.JOB_ID, "files": rows})

    def preflight(self):
        self.authorize()
        output = subprocess.run([self._python(), "-B", str(self.safe(self.WORKER_SCRIPT)), "--preflight", "--slot", self.slot],
            stdin=subprocess.DEVNULL, capture_output=True, timeout=45, cwd=str(self.root), env=self._environment(), shell=False)
        if len(output.stdout) > 2 * 1024 * 1024:
            raise BridgeError("Batch002 preflight oversized")
        result = json.loads(output.stdout.decode("utf-8-sig"))
        if output.returncode != 0 or not isinstance(result, dict):
            raise BridgeError("Batch002 preflight failed")
        if not self.is_connection and not self.safe(self.ATTEMPT).exists():
            self.connection_gate()
        return result

    def connection_gate(self):
        """Fail closed on exact current batch, native artifacts and cleanup; no launches."""
        conn = StdinValidationSlot(self.root)
        conn.authorize()
        request = conn._json(conn.ATTEMPT + "/request.json")
        final = conn._json(conn.ATTEMPT + "/probe_result.json")
        native = conn._json(conn.ATTEMPT + "/native_result.json")
        readiness = conn._json(conn.ATTEMPT + "/readiness.json")
        from inspection_batch002_connection import readiness_passed
        if not readiness_passed(readiness):
            raise BridgeError("Batch002 connection gate: native health/solver identity not verified")
        if (request.get("batch_id") != BATCH or request.get("slot") != conn.slot or
                request.get("job_id") != conn.JOB_ID or request.get("operation") != self.OPERATION or
                request.get("mesh") is not None or request.get("sources") != conn._sources() or
                request.get("authorization") != {"path": str(conn.safe(GRANT)), "sha256": sha256(conn.safe(GRANT))}):
            raise BridgeError("Batch002 connection gate request/source/grant mismatch")
        import math
        created = request.get("created_unix")
        if (type(created) not in (int, float) or not math.isfinite(created) or created > time.time() + 1 or
                request.get("deadline_unix") != created + 300 or request.get("work_deadline_unix") != created + 270 or
                any(final.get(k) != request.get(k) for k in ("created_unix", "deadline_unix", "work_deadline_unix"))):
            raise BridgeError("Batch002 connection gate original deadline binding missing")
        if (final.get("job_id") != conn.JOB_ID or final.get("batch_id") != BATCH or final.get("slot") != conn.slot or
                final.get("status") != "completed" or final.get("native") != native or
                any(final.get(k) is not True for k in ("owned_cleanup_verified", "inputs_reverified_after_cleanup", "sources_reverified_after_cleanup", "serial_dispatch_released")) or
                final.get("mesh_motion_verified") is not False or final.get("solver_launched") is not True or final.get("mesh_generated") is not False or
                type(final.get("flow_iterations")) is not int or final["flow_iterations"] != 0 or
                type(final.get("native_adapter_exit_code")) is not int or final.get("native_adapter_exit_code") != 0 or
                final.get("ownership_errors") != [] or
                type(final.get("elapsed_seconds")) not in (int, float) or not 0 < final["elapsed_seconds"] <= 300 or
                created + final["elapsed_seconds"] > time.time() + 1):
            raise BridgeError("Batch002 connection gate final scope/cleanup evidence missing")
        from preflight import verify_inputs
        actual_inputs = verify_inputs(self._json("frozen/input_manifest.json"), workspace=self.root.parent.parent)
        if (actual_inputs.get("status") != "passed" or len(actual_inputs.get("files", [])) != 8 or
                final.get("protected_inputs_before") != actual_inputs or final.get("protected_inputs_after") != actual_inputs):
            raise BridgeError("Batch002 connection gate protected input evidence changed/incomplete")
        if (native.get("provenance") != "live_fluent_batch002" or native.get("batch_id") != BATCH or
                native.get("slot") != conn.slot or native.get("job_id") != conn.JOB_ID or
                native.get("readiness") != readiness or native.get("readiness_sha256") != sha256(conn.safe(conn.ATTEMPT + "/readiness.json")) or
                native.get("launch_uncertain") is not False or native.get("commands") != {} or
                native.get("mesh_generated") is not False or type(native.get("flow_iterations")) is not int or
                native.get("flow_iterations") != 0 or native.get("errors") != {} or "mesh_evidence" in native):
            raise BridgeError("Batch002 connection gate native evidence is not authentic fixed-scope evidence")
        stdin = native.get("stdin_lifecycle", {})
        if (not isinstance(stdin, dict) or stdin.get("mode") != "private_subprocess_PIPE" or
                type(stdin.get("bytes_written")) is not int or stdin["bytes_written"] != 0 or
                any(stdin.get(k) is not True for k in ("close_requested_in_cleanup", "pipe_returned", "writer_closed")) or
                stdin.get("late_return_after_close_request") is not False or
                native.get("stdin_close_phase") != "after_bounded_session_exit_request"):
            raise BridgeError("Stdin validation gate requires the existing native PIPE lifecycle evidence")
        transcript = conn.safe(conn.ATTEMPT + "/native.transcript.txt")
        audit = conn._json(conn.ATTEMPT + "/command_audit.json")
        if (not transcript.is_file() or transcript.stat().st_size > 2 * 1024 * 1024 or
                native.get("transcript_sha256") != sha256(transcript) or audit.get("batch_id") != BATCH or
                audit.get("job_id") != conn.JOB_ID or audit.get("slot") != conn.slot or
                audit.get("free_form_commands_accepted") is not False):
            raise BridgeError("Batch002 connection native transcript/audit binding missing")
        # Tie native adapter and actual Fluent process to launch ledger/owned tree.
        launcher = conn._json(conn.ATTEMPT + "/launcher.json")
        claim = conn._json(conn.ATTEMPT + "/native_claim.json")
        fluent = conn._json(conn.ATTEMPT + "/fluent_process.json")
        pairs = {(r.get("pid"), r.get("creation_time")) for r in final.get("owned_processes", [])}
        if (launcher.get("job_id") != conn.JOB_ID or claim.get("job_id") != conn.JOB_ID or fluent.get("job_id") != conn.JOB_ID or
                claim.get("parent_pid") != launcher.get("pid") or claim.get("parent_creation_time") != launcher.get("creation_time") or
                fluent.get("parent_pid") != claim.get("pid") or
                (fluent.get("pid"), fluent.get("creation_time")) not in pairs or
                (claim.get("pid"), claim.get("creation_time")) not in pairs or
                fluent.get("executable") != self._json("contract.json")["software"]["fluent"].replace("/", "\\") or
                not any(row.get("pid") == fluent.get("pid") and row.get("creation_time") == fluent.get("creation_time") and
                        row.get("executable") == fluent.get("executable") for row in final.get("owned_process_identity", []))):
            raise BridgeError("Batch002 connection gate native ownership chain missing")
        manifest = conn._json(conn.ATTEMPT + "/source_snapshot/archive_manifest.json")
        files = manifest.get("files", [])
        if manifest.get("job_id") != conn.JOB_ID or len(files) != len(request["sources"]):
            raise BridgeError("Batch002 connection gate source archive incomplete")
        for row, source in zip(files, request["sources"]):
            if (row.get("original_path") != source["path"] or row.get("sha256") != source["sha256"] or
                    row.get("size") != source["size"] or
                    sha256(conn.safe(conn.ATTEMPT + "/source_snapshot/" + row["archive_relative"])) != source["sha256"]):
                raise BridgeError("Batch002 connection gate archived source changed")
        bindings = conn._json(conn.ATTEMPT + "/source_snapshot/execution_binding.json")
        expected_bindings = [{"original": rel, "archive_relative": "execution_binding/" + Path(rel).name,
                              "sha256": sha256(conn.safe(rel))} for rel in (GRANT, MANIFEST)]
        if bindings != {"job_id": conn.JOB_ID, "files": expected_bindings}:
            raise BridgeError("Batch002 execution binding archive incomplete")
        for row in expected_bindings:
            if sha256(conn.safe(conn.ATTEMPT + "/source_snapshot/" + row["archive_relative"])) != row["sha256"]:
                raise BridgeError("Batch002 execution binding archive changed")
        # A cleanup receipt alone cannot override a retained ownership lock.
        if self.safe(".ansys-task.lock").exists():
            lock = self._json(".ansys-task.lock")
            if lock.get("attempt") != str(self.safe(self.ATTEMPT)):
                raise BridgeError("Existing ANSYS task lock blocks connection gate")
        return {"status": "passed", "batch_id": BATCH, "job_id": conn.JOB_ID,
                "connection_result_sha256": sha256(conn.safe(conn.ATTEMPT + "/probe_result.json")),
                "readiness_sha256": sha256(conn.safe(conn.ATTEMPT + "/readiness.json"))}

    def start(self, confirmation):
        if confirmation != self.START_CONFIRMATION:
            raise BridgeError("Explicit fixed-slot batch002 confirmation required")
        # Consumed slots stay consumed even if current sources/connection change.
        if self.safe(self.ATTEMPT).exists():
            return {"status": "already_reserved", "new_worker_started": False, "job": self.job_status(self.JOB_ID)}
        if (self.slot == "connection" or self.slot == SUPPLEMENT and POLICY["supplement_connection_retired"] or
                self.slot == STDIN_VALIDATION and POLICY["stdin_validation_retired"]):
            raise BridgeError("Connection slot permanently retired; no new native allowance or directory recreation")
        self.authorize()
        if self.safe(".ansys-task.lock").exists() or self.safe(".sizing-dispatch.lock").exists():
            raise BridgeError("Existing ownership/dispatch lock; no new batch002 slot consumed")
        if not self.is_connection:
            self.connection_gate()
        self.preflight()
        token = uuid.uuid4().hex
        dispatch = self.safe(".mesh-inspection-dispatch.lock")
        try:
            write_new(dispatch, {"token": token, "job_id": self.JOB_ID, "created_unix": time.time()})
        except FileExistsError:
            raise BridgeError("Inspection dispatch already pending; no slot consumed")
        result = None
        try:
            # Recheck gate under dispatch reservation to reject changed evidence.
            if not self.is_connection:
                self.connection_gate()
            result = Stage15Bridge.start(self, confirmation)
            result.pop("remaining_definition_probes", None)
            result.update(batch_id=BATCH, slot=self.slot, CFD_authorized=False)
            return result
        finally:
            # Accepted work keeps the serial dispatch lock through final owned
            # cleanup. The worker releases its own lock; a second domain cannot
            # race the first worker's task-lock acquisition and consume a slot.
            release = (result is not None and result.get("owned_cleanup_verified") is True) or not self.safe(self.ATTEMPT).exists()
            if release and dispatch.exists() and self._json(dispatch.name).get("token") == token:
                dispatch.unlink()

    def status(self):
        used = self.safe(self.ATTEMPT).exists()
        result = {"batch_id": BATCH, "slot": self.slot, "job_id": self.JOB_ID,
            "slot_reserved": used, "remaining_attempts": 0 if (used or self.slot == "connection" or self.slot == SUPPLEMENT and POLICY["supplement_connection_retired"] or
                self.slot == STDIN_VALIDATION and POLICY["stdin_validation_retired"]) else 1,
            "maximum_seconds_including_cleanup": 300, "start_confirmation": self.START_CONFIRMATION, "cancel_confirmation": self.CANCEL_CONFIRMATION,
            "supplemental_connection": self.slot == SUPPLEMENT, "QA_attempts_added": 0,
            "connection_gate_required": not self.is_connection, "CFD_authorized": False}
        if used:
            result["job"] = self.job_status(self.JOB_ID)
        return result

    def job_status(self, job_id):
        value = Stage15Bridge.job_status(self, job_id)
        value.pop("remaining_definition_probes", None)
        value.update(batch_id=BATCH, slot=self.slot, CFD_authorized=False)
        final = value.get("result")
        if isinstance(final, dict):
            if final.get("batch_id") != BATCH or final.get("slot") != self.slot:
                raise BridgeError("Batch002 historical result identity mismatch")
            request = self._json(self.ATTEMPT + "/request.json")
            value["slot_authorization"] = request.get("authorization")
            value["historical_source_count"] = len(request.get("sources", []))
            if self.is_connection:
                try:
                    if self.JOB_ID != STDIN_VALIDATION_JOB:
                        raise BridgeError("Retired connection history cannot stand in for the stdin validation gate")
                    value["connection_gate"] = self.connection_gate()
                except Exception:
                    value["connection_gate"] = {"status": "unverified"}
            else:
                from mesh_inspection_evidence import validate_native_evidence, parse_native_output
                from inspection_batch002_connection import readiness_passed
                native = final.get("native", {})
                bound = False
                try:
                    transcript = self.safe(self.ATTEMPT + "/native.transcript.txt")
                    ready = self._json(self.ATTEMPT + "/readiness.json")
                    bound = (isinstance(native, dict) and self._json(self.ATTEMPT + "/native_result.json") == native and
                        native.get("provenance") == "live_fluent_batch002" and native.get("job_id") == job_id and
                        native.get("batch_id") == BATCH and native.get("slot") == self.slot and readiness_passed(ready) and
                        native.get("readiness") == ready and native.get("readiness_sha256") == sha256(self.safe(self.ATTEMPT + "/readiness.json")) and
                        transcript.stat().st_size <= 2 * 1024 * 1024 and native.get("transcript_sha256") == sha256(transcript) and
                        native.get("mesh_evidence", {}).get("metrics") == parse_native_output(transcript.read_text(encoding="utf-8", errors="replace")) and
                        final.get("solver_launched") is True and bool(final.get("owned_process_identity")))
                except (OSError, ValueError, BridgeError):
                    pass
                value["inspection"] = validate_native_evidence(native.get("mesh_evidence", {}) if bound else {}, self.slot)
                value["inspection"]["native_artifacts_bound"] = bound
            if (final.get("mesh_generated") is not False or type(final.get("flow_iterations")) is not int or
                    final.get("flow_iterations") != 0 or final.get("mesh_motion_verified") is not False):
                value["status"] = "failed"
            if final.get("status") == "completed" and any(final.get(k) is not True for k in
                    ("owned_cleanup_verified", "inputs_reverified_after_cleanup", "sources_reverified_after_cleanup")):
                value["status"] = "failed"
        return value


class SupplementalConnectionSlot(Batch002Slot):
    """Exactly one separately user-authorized connection, no new QA attempts."""
    START_CONFIRMATION = SUPPLEMENT_START
    CANCEL_CONFIRMATION = SUPPLEMENT_CANCEL
    CANCEL_OPERATION = "cancel_inspection_batch002_supplement"

    def __init__(self, stage_root=STAGE):
        Stage15Bridge.__init__(self, stage_root)
        self.slot = self.domain = SUPPLEMENT
        self.is_connection = True
        self.JOB_ID = SUPPLEMENT_JOB
        self.ATTEMPT = SUPPLEMENT_ATTEMPT


class StdinValidationSlot(SupplementalConnectionSlot):
    """Exactly one new user-approved stdin connection validation, no QA additions."""
    START_CONFIRMATION = STDIN_VALIDATION_START
    CANCEL_CONFIRMATION = STDIN_VALIDATION_CANCEL
    CANCEL_OPERATION = "cancel_stdin_connection_validation"

    def __init__(self, stage_root=STAGE):
        Stage15Bridge.__init__(self, stage_root)
        self.slot = self.domain = STDIN_VALIDATION
        self.is_connection = True
        self.JOB_ID = STDIN_VALIDATION_JOB
        self.ATTEMPT = STDIN_VALIDATION_ATTEMPT


def execution_slot(slot, stage_root=STAGE):
    if slot == STDIN_VALIDATION:
        return StdinValidationSlot(stage_root)
    if slot == SUPPLEMENT:
        return SupplementalConnectionSlot(stage_root)
    return Batch002Slot(slot, stage_root)


def batch_job(job_id, stage_root=STAGE):
    for slot in EXECUTION_SLOTS:
        item = execution_slot(slot, stage_root)
        if item.JOB_ID == job_id:
            return item
    raise BridgeError("Unknown fixed batch002 job ID")

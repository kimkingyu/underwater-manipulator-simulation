"""Three separately identified sizing-diagnostic slots; no automatic retries.

The user authorized further diagnostics without per-run questions. Each batch
still has a finite frozen budget, and each explicit attempt is idempotent.
"""
from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import time
import uuid

from mcp_bridge import Stage15Bridge, BridgeError, STAGE, read_json, sha256, write_new

BATCH_ROOT = "runs/mcp_sizing_diagnostic/batch_001"
GRANT = "frozen/sizing_diagnostic_authorization_003.json"
PREFIX = "sizing-b001-a"
PROFILES = ("baseline", "geometry_scope", "capture_off", "omit_local_growth", "background_all_controls")
START_TOKEN = "RUN_STAGE15_SIZING_DIAGNOSTIC"
CANCEL_TOKEN = "CANCEL_STAGE15_SIZING_DIAGNOSTIC"
SOURCE_RELATIVES = (
    "stage15_mcp.py", "mcp_bridge.py", "definition_probe.py", "sizing_diagnostic.py",
    "sizing_mcp_bridge.py", "runtime.py", "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "mesh/wb_definition_probe.wbjn.in", "mesh/meshing_definition_probe.py.in",
    "mesh/wb_sizing_diagnostic.wbjn.in", "mesh/meshing_sizing_diagnostic.py.in",
)
AUTH_VALUES = {
    "schema_version": 1, "scope": "stage15_mcp_sizing_diagnostic", "operation": "sizing_diagnostic",
    "maximum_attempts": 3, "maximum_total_seconds": 300, "cleanup_reserve_seconds": 30,
    "attempt_root_relative": BATCH_ROOT, "job_id_prefix": PREFIX,
    "allowed_profiles": list(PROFILES),
    "baseline_authorization_relative": "frozen/mcp_definition_probe_authorization_001.json",
    "baseline_authorization_sha256": "51dbdd6023ad9d9f0929d38fefcb4e86a6d906ec25ac18b7c00c2126ccbfbd1f",
    "full_mesh_allowed": False, "fluent_allowed": False, "automatic_retry_allowed": False,
    "original_geometry_modification_allowed": False, "default_theta_modification_allowed": False,
    "license_service_modification_allowed": False,
}
AUTH_METADATA = {"authorization_source", "recorded_local", "batch_policy"}
TERMINAL = {"completed", "failed", "cancelled"}


def parse_job_id(job_id):
    match = re.fullmatch(r"sizing-b001-a(00[123])", str(job_id))
    if not match:
        raise BridgeError("Unknown sizing job; no arbitrary IDs/paths accepted")
    return int(match.group(1))


class SizingSlot(Stage15Bridge):
    AUTHORIZATION = GRANT
    START_CONFIRMATION = START_TOKEN
    CANCEL_CONFIRMATION = CANCEL_TOKEN
    SOURCES = SOURCE_RELATIVES
    WORKER_SCRIPT = "sizing_diagnostic.py"
    OPERATION = "sizing_diagnostic"
    CANCEL_OPERATION = "cancel_sizing_diagnostic"
    LOGS = {
        "worker": "worker.log", "workbench_stdout": "workbench.stdout.log",
        "workbench_stderr": "workbench.stderr.log", "trace": "workbench.trace.txt",
        "define": "sizing_define_stats.json", "refresh": "sizing_refresh_stats.json",
        "readback": "sizing_readback_stats.json", "completion": "workbench_completion.json",
    }

    def __init__(self, attempt_index, profile="baseline", hypothesis="Read existing diagnostic", stage_root=STAGE):
        if type(attempt_index) is not int or not 1 <= attempt_index <= 3:
            raise BridgeError("Only batch001 attempt indices 1..3 are authorized")
        if profile not in PROFILES:
            raise BridgeError("Diagnostic profile is not allow-listed")
        if not isinstance(hypothesis, str) or not hypothesis.strip() or len(hypothesis) > 2000:
            raise BridgeError("A bounded nonempty diagnostic hypothesis is required")
        self.attempt_index = attempt_index
        self.profile = profile
        self.hypothesis = hypothesis
        self.JOB_ID = PREFIX + "%03d" % attempt_index
        self.ATTEMPT = BATCH_ROOT + "/attempt_%03d" % attempt_index
        super().__init__(stage_root)

    def authorize(self):
        grant = self._json(GRANT)
        if set(grant) - set(AUTH_VALUES) - AUTH_METADATA:
            raise BridgeError("Unexpected diagnostic authorization field")
        for key, expected in AUTH_VALUES.items():
            if type(grant.get(key)) is not type(expected) or grant[key] != expected:
                raise BridgeError("Diagnostic grant mismatch: " + key)
        for key in AUTH_METADATA & set(grant):
            if not isinstance(grant[key], str) or len(grant[key]) > 4000:
                raise BridgeError("Invalid grant metadata")
        baseline = self.safe(AUTH_VALUES["baseline_authorization_relative"])
        if sha256(baseline) != AUTH_VALUES["baseline_authorization_sha256"]:
            raise BridgeError("Baseline CAD authorization binding changed")
        # This verifies immutable input/CAD hashes, not the legacy spent slot.
        Stage15Bridge(self.root).authorize()
        return grant

    def make_request(self, created, sources):
        request = super().make_request(created, sources)
        request.update(attempt_index=self.attempt_index, profile=self.profile, hypothesis=self.hypothesis)
        return request

    def preserve_sources(self, request):
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
            "job_id": self.JOB_ID, "purpose": "immutable source evidence for this diagnostic only", "files": manifest})
    def _preceding_jobs_finished(self):
        for index in range(1, 4):
            slot = SizingSlot(index, stage_root=self.root)
            directory = slot.safe(slot.ATTEMPT)
            if not directory.exists():
                if index < self.attempt_index:
                    raise BridgeError("Use explicit sequential diagnostic slots; earlier attempt missing")
                continue
            if index == self.attempt_index:
                continue
            final_path = slot.safe(slot.ATTEMPT + "/probe_result.json")
            failure_path = slot.safe(slot.ATTEMPT + "/launch_failure.json")
            if final_path.exists():
                final = read_json(final_path)
            elif failure_path.exists():
                final = read_json(failure_path)
            else:
                raise BridgeError("Another diagnostic slot is reserved/running or lacks final evidence")
            if (final.get("job_id") != slot.JOB_ID or final.get("status") not in TERMINAL or
                    final.get("owned_cleanup_verified") is not True):
                raise BridgeError("Earlier diagnostic cleanup is not verified; do not remove its lock")
            launcher_path = slot.safe(slot.ATTEMPT + "/launcher.json")
            if launcher_path.exists():
                launcher = read_json(launcher_path)
                if launcher.get("job_id") != slot.JOB_ID:
                    raise BridgeError("Earlier launcher identity mismatch")
                live = self._inspect_pid(launcher["pid"])
                if live["exists"] and live.get("creation_time") == launcher.get("creation_time"):
                    raise BridgeError("Earlier diagnostic worker has not exited yet")

    def start(self, confirmation):
        if confirmation != START_TOKEN:
            raise BridgeError("Explicit sizing-diagnostic confirmation required")
        self.authorize()
        existing = self.safe(self.ATTEMPT)
        if existing.exists():
            request_path = self.safe(self.ATTEMPT + "/request.json")
            if request_path.exists():
                request = read_json(request_path)
                if request.get("job_id") != self.JOB_ID or request.get("profile") != self.profile:
                    raise BridgeError("This immutable slot already belongs to another diagnostic profile")
            return self._renamed(super().start(confirmation))
        # Serialize reservation-to-launch across different explicit slots. The
        # shared ANSYS task lock additionally protects all legacy/native callers.
        dispatch = self.safe(".sizing-dispatch.lock")
        token = uuid.uuid4().hex
        try:
            write_new(dispatch, {"token": token, "pid": os.getpid(), "job_id": self.JOB_ID,
                                 "created_unix": time.time()})
        except FileExistsError:
            raise BridgeError("Another diagnostic launch or uncleared dispatch lock exists")
        try:
            self._preceding_jobs_finished()
            return self._renamed(super().start(confirmation))
        finally:
            current = read_json(dispatch)
            if current.get("token") != token:
                raise BridgeError("Diagnostic dispatch lock ownership changed; lock retained")
            dispatch.unlink()

    @staticmethod
    def _renamed(result):
        result = dict(result)
        result.pop("remaining_definition_probes", None)
        result["current_slot_reusable"] = False
        result["full_mesh_authorized"] = False
        result["solver_authorized"] = False
        return result

    @staticmethod
    def verified_background_claim(final):
        if not isinstance(final, dict) or final.get("profile") != "background_all_controls" or final.get("status") != "completed":
            return False
        required = ("full_background_definitions_verified", "diagnostic_completed", "zero_mesh_verified",
                    "owned_cleanup_verified", "sources_reverified_after_cleanup", "inputs_reverified_after_cleanup",
                    "message_capture_complete", "background_scopes_stable")
        if any(final.get(key) is not True for key in required) or final.get("native_errors_present") is not False:
            return False
        matches = final.get("background_definition_matches", {})
        receipts = final.get("receipts", {})
        phases = {"define", "refresh", "readback"}
        if set(matches) != phases or any(item is not True for item in matches.values()) or set(receipts) != phases:
            return False
        expected = {"stage15_background_tet", "stage15_wall_base0", "stage15_wall_auv", "stage15_overlap_0"}
        for receipt in receipts.values():
            if receipt.get("message_capture_complete") is not True or receipt.get("native_errors_present") is not False:
                return False
            controls = receipt.get("background_controls", [])
            if len(controls) != 4 or any(not isinstance(row, dict) for row in controls) or {row.get("name") for row in controls} != expected:
                return False
            for row in controls:
                actual, wanted = row.get("actual_geometry_ids"), row.get("expected_geometry_ids")
                if (row.get("object_state") != "FullyDefined" or not isinstance(actual, list) or not actual or
                        not isinstance(wanted, list) or sorted(actual) != sorted(wanted)):
                    return False
        return True
    def job_status(self, job_id):
        result = self._renamed(super().job_status(job_id))
        final = result.get("result")
        if final is not None and final.get("status") == "completed":
            expected = {"diagnostic_completed": True, "zero_mesh_verified": True,
                        "owned_cleanup_verified": True, "mesh_generated": False,
                        "flow_iterations": 0, "mesh_motion_verified": False}
            if any(type(final.get(key)) is not type(value) or final.get(key) != value
                   for key, value in expected.items()) or type(final.get("control_fully_defined")) is not bool:
                result.update(status="failed", phase="diagnostic_final_evidence_missing_or_contradictory")
            receipts = final.get("receipts", {})
            if not isinstance(receipts, dict) or set(receipts) != {"define", "refresh", "readback"}:
                result.update(status="failed", phase="diagnostic_phase_evidence_missing")
            else:
                for phase, value in receipts.items():
                    counts = value.get("mesh_counts", {}) if isinstance(value, dict) else {}
                    if (not isinstance(value, dict) or value.get("status") != "completed" or
                            value.get("zero_mesh_verified") is not True or not isinstance(counts, dict) or
                            any(type(counts.get(key)) is not int or counts[key] != 0 for key in ("nodes", "elements"))):
                        result.update(status="failed", phase="diagnostic_native_zero_counts_missing_or_nonzero")
                last = receipts.get("readback", {})
                if last.get("control_fully_defined") is not final.get("control_fully_defined"):
                    result.update(status="failed", phase="diagnostic_control_state_contradiction")
            result["control_fully_defined"] = final.get("control_fully_defined")
            result["diagnostic_completed"] = final.get("diagnostic_completed")
            result["zero_mesh_verified"] = final.get("zero_mesh_verified")
        # Keep status useful and bounded. Full snapshots remain in fixed job logs.
        receipts = (final or result).get("receipts", {})
        observations = {}
        if isinstance(receipts, dict):
            for phase, value in receipts.items():
                if phase not in {"define", "refresh", "readback"} or not isinstance(value, dict):
                    continue
                control = value.get("control", {})
                observations[phase] = {
                    "status": value.get("status"), "mesh_counts": value.get("mesh_counts"),
                    "control_state": control.get("object_state"), "fields": control.get("fields", {}),
                    "step_states": [{"step": row.get("step"), "object_state": row.get("object_state")}
                                    for row in value.get("snapshots", [])[:10]],
                    "native_errors_present": value.get("native_errors_present"),
                    "message_capture_complete": value.get("message_capture_complete"),
                    "messages_excerpt": value.get("messages", [])[:8],
                    "background_controls": [{
                        "name": row.get("name"), "kind": row.get("kind"), "object_state": row.get("object_state"),
                        "scope_count": len(row.get("actual_geometry_ids", [])),
                        "scope_matches": row.get("actual_geometry_ids") == row.get("expected_geometry_ids"),
                        "element_size": row.get("element_size"), "sphere_radius": row.get("sphere_radius"),
                        "coordinate_state": row.get("coordinate_system", {}).get("object_state"),
                        "local_growth_override_applied": row.get("local_growth_override_applied"),
                    } for row in value.get("background_controls", [])[:4] if isinstance(row, dict)],
                }
        result["observations"] = observations
        result.pop("receipts", None)
        result.pop("sources", None)
        for key in ("protected_inputs_before", "protected_inputs_after"):
            evidence = result.get(key)
            if isinstance(evidence, dict):
                result[key] = {"status": evidence.get("status"), "file_count": len(evidence.get("files", []))}
        if isinstance(final, dict):
            keys = ("job_id", "status", "profile", "error", "elapsed_seconds", "diagnostic_completed",
                    "control_fully_defined", "zero_mesh_verified", "owned_cleanup_verified",
                    "sources_reverified_after_cleanup", "inputs_reverified_after_cleanup",
                    "receipt_paths", "phase_summaries", "native_errors_present",
                    "full_background_definitions_verified", "background_definition_matches", "background_scopes_stable")
            result["result"] = {key: final.get(key) for key in keys if key in final}
        result["full_background_definitions_verified"] = result.get("status") == "completed" and self.verified_background_claim(final)
        if isinstance(final, dict) and final.get("full_background_definitions_verified") is True and not result["full_background_definitions_verified"]:
            result.update(status="failed", phase="background_definition_claim_inconsistent")
        result["mesh_quality_verified"] = False
        return result


class SizingBudget:
    def __init__(self, stage_root=STAGE):
        self.root = Path(stage_root).absolute()
        self.reader = Stage15Bridge(self.root)

    def status(self):
        grant_present = self.reader.safe(GRANT).is_file()
        jobs = []
        for index in range(1, 4):
            slot = SizingSlot(index, stage_root=self.root)
            if not slot.safe(slot.ATTEMPT).exists():
                continue
            value = {"job_id": slot.JOB_ID, "attempt_index": index, "status": "reserved"}
            for name in ("probe_result.json", "launch_failure.json", "job_state.json"):
                path = slot.safe(slot.ATTEMPT + "/" + name)
                if path.exists():
                    record = read_json(path)
                    if record.get("job_id") != slot.JOB_ID:
                        raise BridgeError("Diagnostic ledger identity mismatch")
                    value.update({key: record.get(key) for key in
                                  ("status", "phase", "profile", "diagnostic_completed",
                                   "control_fully_defined", "owned_cleanup_verified")})
                    break
            jobs.append(value)
        return {"batch": "batch_001", "grant_present": grant_present,
                "maximum_attempts": 3, "reserved_attempts": len(jobs),
                "unreserved_slots": 3 - len(jobs), "per_attempt_total_seconds": 300,
                "profiles": list(PROFILES), "jobs": jobs,
                "dispatch_lock_present": self.reader.safe(".sizing-dispatch.lock").exists(),
                "full_mesh_authorized": False, "solver_authorized": False,
                "note": "Unreserved slots are not readiness evidence; check the frozen grant and preflight.",
                "start_confirmation": START_TOKEN, "cancel_confirmation": CANCEL_TOKEN}

    def preflight(self):
        result = SizingSlot(1, stage_root=self.root).preflight()
        result["diagnostic_budget"] = self.status()
        return result

    def start(self, attempt_index, profile, hypothesis, confirmation):
        slot = SizingSlot(attempt_index, profile, hypothesis, self.root)
        result = slot.start(confirmation)
        result["diagnostic_budget"] = self.status()
        return result

    def job(self, job_id):
        return SizingSlot(parse_job_id(job_id), stage_root=self.root)

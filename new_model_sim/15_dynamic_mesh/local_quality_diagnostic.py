"""Two independently approved bounded local diagnostics; no repair/rebuild/CFD.

MCP accepts only a fixed domain and confirmation. CLI execution is restricted to
reserved fixed paths; import and --preflight do not launch Fluent.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

from mcp_bridge import STAGE, Stage15Bridge, BridgeError, read_json, sha256, write_new
from inspection_batch002_bridge import Batch002Slot, MESHES, SOURCES as EXISTING_SOURCES

BATCH_ID = "local-quality-batch001"
PROVENANCE = "live_fluent_local_quality_batch001"
GRANT = "frozen/local_quality_authorization_001.json"
MANIFEST = "frozen/local_quality_sources_001.json"
START = "RUN_STAGE15_LOCAL_QUALITY_DIAGNOSTIC_ONCE"
CANCEL = "CANCEL_STAGE15_LOCAL_QUALITY_DIAGNOSTIC"
SOURCES = tuple(EXISTING_SOURCES) + ("local_quality_diagnostic.py", "local_quality_evidence.py", "frozen/local_quality_api_basis_001.json")
POLICY = {"schema_version": 1, "batch_id": BATCH_ID, "operation": "existing_mesh_local_quality_read_only",
    "domains": ["background", "arm"], "maximum_attempts": 2, "maximum_attempts_per_domain": 1,
    "maximum_total_seconds_per_slot": 300, "cleanup_reserve_seconds": 30, "native_start_timeout_seconds": 45,
    "transport_ready_max_seconds": 20, "health_call_max_seconds": 2, "normal_exit_request_seconds": 8,
    "normal_exit_wait_seconds": 12, "maximum_simultaneous_jobs": 1, "threshold": 0.01, "maximum_histogram_bins": 6,
    "maximum_worst_records": 20, "maximum_diagnostic_json_bytes": 16384, "maximum_transcript_bytes": 2097152,
    "strict_threshold_count_requires_verified_native_semantics": True, "top20_array_fallback_allowed": False,
    "histogram_report_settings_only": True, "mesh_write_allowed": False, "mesh_generation_allowed": False,
    "repair_or_marking_allowed": False, "initialization_allowed": False, "iterations_allowed": False,
    "motion_allowed": False, "overset_assembly_allowed": False, "CFD_allowed": False,
    "automatic_retry_allowed": False, "old_slots_reusable": False, "additional_mesh_constructions": 0,
    "mesh_inputs": MESHES, "source_manifest_relative": MANIFEST}


class LocalQualitySlot(Batch002Slot):
    AUTHORIZATION = GRANT
    SOURCE_MANIFEST = MANIFEST
    SOURCES = SOURCES
    WORKER_SCRIPT = "local_quality_diagnostic.py"
    START_CONFIRMATION = START
    CANCEL_CONFIRMATION = CANCEL
    CANCEL_OPERATION = "cancel_local_quality_diagnostic"
    OPERATION = "mesh_inspection"  # existing fixed process transport, distinct batch/path/job below

    def __init__(self, domain, stage_root=STAGE):
        if domain not in ("background", "arm"):
            raise BridgeError("Only the two fixed local-quality domains are allowed")
        Stage15Bridge.__init__(self, stage_root)
        self.slot = self.domain = domain
        self.is_connection = False
        self.JOB_ID = "local-quality-001-" + domain
        self.ATTEMPT = "runs/mcp_local_quality/batch_001/" + domain

    def authorize(self):
        value = self._json(GRANT)
        metadata = {"authorization_source", "recorded_local", "execution_policy", "source_manifest_sha256"}
        if set(value) != set(POLICY) | metadata:
            raise BridgeError("Local-quality authorization fields differ")
        for key, expected in POLICY.items():
            if type(value.get(key)) is not type(expected) or value[key] != expected:
                raise BridgeError("Local-quality fixed scope mismatch: " + key)
        if any(type(value[k]) is not str or not 1 <= len(value[k]) <= 4000 for k in metadata - {"source_manifest_sha256"}):
            raise BridgeError("Local-quality actual user provenance missing")
        if sha256(self.safe(MANIFEST)) != value["source_manifest_sha256"] or self._json(MANIFEST) != {"schema_version": 1, "files": self._sources()}:
            raise BridgeError("Local-quality source binding changed")
        for row in MESHES.values():
            path = self.safe(row["path"])
            if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]:
                raise BridgeError("Original diagnostic mesh changed")
        from preflight import verify_inputs
        inputs = verify_inputs(self._json("frozen/input_manifest.json"), workspace=self.root.parent.parent)
        if inputs["status"] != "passed" or len(inputs["files"]) != 8:
            raise BridgeError("Eight protected inputs must remain unchanged")
        return value

    def make_request(self, created, sources):
        value = Stage15Bridge.make_request(self, created, sources)
        value.update(batch_id=BATCH_ID, slot=self.slot, mesh=MESHES[self.slot])
        return value

    def preserve_sources(self, request):
        self.archive_sources(request)
        rows = []
        for relative in (GRANT, MANIFEST):
            source = self.safe(relative)
            target = self.safe(self.ATTEMPT + "/source_snapshot/execution_binding/" + Path(relative).name)
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as original, target.open("xb") as saved:
                shutil.copyfileobj(original, saved)
            if sha256(target) != sha256(source): raise BridgeError("Local execution binding changed")
            rows.append({"original": relative, "archive_relative": "execution_binding/" + Path(relative).name, "sha256": sha256(source)})
        write_new(self.safe(self.ATTEMPT + "/source_snapshot/execution_binding.json"), {"job_id": self.JOB_ID, "files": rows})

    def connection_gate(self):
        # Each new domain authorization includes its own single connect/read/query/exit.
        # No retired connection/QA preflight, probe, proof or old quality pass is used.
        return {"status": "independent_session_readiness_required", "batch_id": BATCH_ID,
                "old_connection_or_QA_reused": False}

    def preflight(self):
        self.authorize()
        output = subprocess.run([self._python(), "-B", str(self.safe(self.WORKER_SCRIPT)), "--preflight", "--domain", self.domain],
            stdin=subprocess.DEVNULL, capture_output=True, timeout=45, cwd=str(self.root), env=self._environment(), shell=False)
        if output.returncode != 0 or len(output.stdout) > 65536:
            raise BridgeError("Fixed read-only local diagnostic preflight failed")
        value = json.loads(output.stdout.decode("utf-8-sig"))
        if not isinstance(value, dict) or value.get("status") != "passed":
            raise BridgeError("Local diagnostic environment not ready")
        return value

    def start(self, confirmation):
        value = Batch002Slot.start(self, confirmation)
        value.update(batch_id=BATCH_ID, local_quality_diagnostic=True, mesh_repair_authorized=False)
        return value

    def status(self):
        used = self.safe(self.ATTEMPT).exists()
        value = {"batch_id": BATCH_ID, "domain": self.domain, "job_id": self.JOB_ID, "slot_reserved": used,
            "remaining_attempts": 0 if used else 1, "maximum_seconds_including_cleanup": 300,
            "start_confirmation": START, "cancel_confirmation": CANCEL, "mesh_repair_authorized": False,
            "mesh_generation_authorized": False, "CFD_authorized": False}
        if used: value["job"] = self.job_status(self.JOB_ID)
        return value

    def job_status(self, job_id):
        value = Stage15Bridge.job_status(self, job_id)
        value.pop("remaining_definition_probes", None)
        value.update(batch_id=BATCH_ID, domain=self.domain, mesh_repair_authorized=False, CFD_authorized=False)
        final = value.get("result")
        if isinstance(final, dict):
            if final.get("job_id") != self.JOB_ID or final.get("batch_id") != BATCH_ID or final.get("slot") != self.slot:
                raise BridgeError("Local diagnostic result identity mismatch")
            try:
                from local_quality_evidence import extract_diagnostic, MAX_REPORT_BYTES
                from inspection_full_record import full_record_passed, verify_full_record_artifacts
                from inspection_batch002_connection import readiness_passed
                transcript = self.safe(self.ATTEMPT + "/native.transcript.txt")
                native = self._json(self.ATTEMPT + "/native_result.json")
                if not 0 < transcript.stat().st_size <= 2097152:
                    raise BridgeError("Local transcript unavailable/oversized")
                verify_full_record_artifacts(self, native, final, classified=True)
                readiness = self._json(self.ATTEMPT + "/readiness.json")
                actual = extract_diagnostic(transcript.read_text(encoding="utf-8", errors="replace"), self.domain, native.get("local_histogram"))
                bound = (native == final.get("native") and native.get("provenance") == PROVENANCE and
                    native.get("job_id") == job_id and native.get("batch_id") == BATCH_ID and
                    native.get("transcript_sha256") == sha256(transcript) and readiness_passed(readiness) and native.get("readiness") == readiness and
                    native.get("readiness_sha256") == sha256(self.safe(self.ATTEMPT + "/readiness.json")) and final.get("native_adapter_exit_code") == 0 and
                    full_record_passed(native) and actual == native.get("local_diagnostic") == final.get("local_diagnostic") and
                    len(json.dumps(actual)) <= MAX_REPORT_BYTES and final.get("owned_cleanup_verified") is True and
                    final.get("inputs_reverified_after_cleanup") is True and final.get("sources_reverified_after_cleanup") is True and
                    final.get("serial_dispatch_released") is True)
                if not bound: raise BridgeError("Local native diagnostic binding incomplete")
                value.update(diagnostic=actual, native_artifacts_bound=True)
            except Exception:
                value.update(diagnostic={"status": "unavailable", "reason": "native_artifacts_not_verified"}, native_artifacts_bound=False)
        return value


def local_job(job_id, stage_root=STAGE):
    for domain in ("background", "arm"):
        slot = LocalQualitySlot(domain, stage_root)
        if slot.JOB_ID == job_id: return slot
    raise BridgeError("Unknown fixed local-quality job")


def local_slot_for_attempt(attempt, stage_root=STAGE):
    from definition_probe import checked_path
    try:
        path = checked_path(attempt, stage_root)
    except OSError as exc:
        raise BridgeError("Local execution requires an existing reserved fixed attempt") from exc
    for domain in ("background", "arm"):
        slot = LocalQualitySlot(domain, stage_root)
        if path == slot.safe(slot.ATTEMPT): return slot, path
    raise BridgeError("Only the two reserved new local-quality attempts are allowed")


def validate_local_request(slot, attempt):
    value = read_json(attempt / "request.json", 65536)
    keys = {"schema_version", "operation", "job_id", "created_unix", "deadline_unix", "work_deadline_unix",
            "authorization", "sources", "batch_id", "slot", "mesh"}
    if (set(value) != keys or type(value.get("schema_version")) is not int or value["schema_version"] != 1 or
            value.get("operation") != slot.OPERATION or value.get("job_id") != slot.JOB_ID or
            value.get("batch_id") != BATCH_ID or value.get("slot") != slot.domain or value.get("mesh") != MESHES[slot.domain]):
        raise BridgeError("Local request fixed identity mismatch")
    created = value.get("created_unix")
    if (any(type(value.get(k)) not in (int, float) or not math.isfinite(value[k]) for k in ("created_unix", "deadline_unix", "work_deadline_unix")) or
            created > time.time() + 1 or value["deadline_unix"] != created + 300 or value["work_deadline_unix"] != created + 270 or
            value["deadline_unix"] <= time.time()):
        raise BridgeError("Local request original 300-second deadline invalid")
    if value["sources"] != slot._sources() or value["authorization"] != {"path": str(slot.safe(GRANT)), "sha256": sha256(slot.safe(GRANT))}:
        raise BridgeError("Local request source/authorization changed")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--run", action="store_true")
    mode.add_argument("--native", action="store_true")
    parser.add_argument("--domain", choices=("background", "arm"))
    parser.add_argument("--attempt", type=Path)
    args = parser.parse_args(argv)
    if args.preflight:
        if args.domain is None or args.attempt is not None: parser.error("Preflight accepts fixed domain only")
        slot = LocalQualitySlot(args.domain)
        slot.authorize()
        from definition_probe import validate_frozen
        from preflight import inspect_processes
        validate_frozen()
        processes = inspect_processes()
        if processes.get("status") != "passed" or processes.get("active_ansys_clients"):
            raise BridgeError("Existing native client blocks independent local diagnostic")
        import psutil
        if any(str(p.info.get("name", "")).casefold() in ("cx2420.exe", "fl2420.exe") for p in psutil.process_iter(["name"])):
            raise BridgeError("Existing Fluent cortex/solver blocks local diagnostic")
        result = {"status": "passed", "batch_id": BATCH_ID, "domain": args.domain, "slot_reserved": slot.safe(slot.ATTEMPT).exists(),
            "maximum_seconds_including_cleanup": 300, "quality_pass_required_to_collect": False,
            "threshold_count": "unavailable_without_verified_strict_native_semantics", "worst_records": "native_minimum_only_if_emitted",
            "maximum_histogram_bins": 6, "native_started": False}
    else:
        if args.attempt is None or args.domain is not None: parser.error("Execution requires the fixed reserved attempt only")
        from inspection_batch002 import run, native_collect
        result = native_collect(args.attempt, local_quality=True) if args.native else run(args.attempt, local_quality=True)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if args.native or result.get("status") in ("passed", "completed") else 1


if __name__ == "__main__":
    raise SystemExit(main())

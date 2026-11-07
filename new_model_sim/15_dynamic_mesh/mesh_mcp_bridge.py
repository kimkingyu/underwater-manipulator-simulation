"""Single newly authorized two-domain mesh construction, separate from diagnostics."""
from __future__ import annotations

import math
from pathlib import Path

from mcp_bridge import Stage15Bridge, BridgeError, STAGE, read_json, sha256

JOB_ID = "mesh-construction-004"
JOB_ATTEMPT = "runs/mcp_mesh_construction/attempt_001"
MESH_ATTEMPT = "runs/mesh/attempt_004"
GRANT = "frozen/mesh_construction_authorization_001.json"
START_TOKEN = "RUN_STAGE15_FULL_MESH_CONSTRUCTION_ONCE"
CANCEL_TOKEN = "CANCEL_STAGE15_FULL_MESH_CONSTRUCTION"
SOURCE_RELATIVES = (
    "stage15_mcp.py", "mcp_bridge.py", "sizing_mcp_bridge.py", "mesh_mcp_bridge.py",
    "mesh_construction.py", "runtime.py", "preflight.py", "mesh_prepare.py", "run_mesh_job.py",
    "definition_probe.py", "mesh/meshing_stage15.py.in", "mesh/wb_stage15.wbjn.in",
)
AUTH_VALUES = {
    "schema_version": 1, "scope": "stage15_mcp_mesh_construction", "operation": "mesh_construction",
    "job_id": JOB_ID, "maximum_additional_attempts": 1, "original_full_mesh_attempts_used": 3,
    "maximum_total_seconds": 7200, "cleanup_reserve_seconds": 30, "maximum_domain_seconds": 3600,
    "job_attempt_relative": JOB_ATTEMPT, "mesh_attempt_relative": MESH_ATTEMPT,
    "domains": ["background", "arm"],
    "baseline_authorization_relative": "frozen/mcp_definition_probe_authorization_001.json",
    "baseline_authorization_sha256": "51dbdd6023ad9d9f0929d38fefcb4e86a6d906ec25ac18b7c00c2126ccbfbd1f",
    "mesh_generation_allowed": True, "mesh_export_allowed": True,
    "fluent_allowed": False, "CFD_allowed": False, "automatic_retry_allowed": False,
    "original_geometry_modification_allowed": False, "default_theta_modification_allowed": False,
    "license_service_modification_allowed": False,
}
AUTH_METADATA = {"authorization_source", "recorded_local", "execution_policy"}


class MeshSlot(Stage15Bridge):
    JOB_ID = JOB_ID
    ATTEMPT = JOB_ATTEMPT
    AUTHORIZATION = GRANT
    START_CONFIRMATION = START_TOKEN
    CANCEL_CONFIRMATION = CANCEL_TOKEN
    SOURCES = SOURCE_RELATIVES
    WORKER_SCRIPT = "mesh_construction.py"
    OPERATION = "mesh_construction"
    CANCEL_OPERATION = "cancel_mesh_construction"
    TOTAL_SECONDS = 7200
    CLEANUP_RESERVE_SECONDS = 30
    LOGS = {
        "worker": "worker.log", "workbench_stdout": "workbench.stdout.log",
        "workbench_stderr": "workbench.stderr.log", "trace": "workbench.trace.txt",
        "mesh_background": "background_meshing_stats.json", "mesh_arm": "arm_meshing_stats.json",
        "completion": "workbench_completion.json", "execution": "workbench_execution.json",
        "plan": "mesh_plan.json",
    }

    def authorize(self):
        grant = self._json(GRANT)
        if set(grant) - set(AUTH_VALUES) - AUTH_METADATA:
            raise BridgeError("Unexpected mesh construction authorization field")
        for key, expected in AUTH_VALUES.items():
            if type(grant.get(key)) is not type(expected) or grant[key] != expected:
                raise BridgeError("Mesh construction authorization mismatch: " + key)
        for key in AUTH_METADATA & set(grant):
            if not isinstance(grant[key], str) or len(grant[key]) > 4000:
                raise BridgeError("Invalid mesh authorization metadata")
        if sha256(self.safe(AUTH_VALUES["baseline_authorization_relative"])) != AUTH_VALUES["baseline_authorization_sha256"]:
            raise BridgeError("Baseline CAD authorization binding changed")
        Stage15Bridge(self.root).authorize()
        reference = self._json("runs/mcp_sizing_diagnostic/batch_001/attempt_003/probe_result.json")
        if (reference.get("job_id") != "sizing-b001-a003" or reference.get("status") != "completed" or
                reference.get("full_background_definitions_verified") is not True or
                reference.get("owned_cleanup_verified") is not True or
                reference.get("inputs_reverified_after_cleanup") is not True or
                reference.get("sources_reverified_after_cleanup") is not True):
            raise BridgeError("Successful real full-background definition prerequisite is missing")
        return grant

    def preserve_sources(self, request):
        return self.archive_sources(request)

    def start(self, confirmation):
        if confirmation != START_TOKEN:
            raise BridgeError("Explicit full-mesh construction confirmation required")
        if not self.safe(self.ATTEMPT).exists():
            self.authorize()
            if self.safe(MESH_ATTEMPT).exists():
                raise BridgeError("Mesh attempt004 already exists; never overwrite or silently rerun it")
            if self.safe(".sizing-dispatch.lock").exists():
                raise BridgeError("Sizing launch/ownership is still pending")
        result = dict(super().start(confirmation))
        result.pop("remaining_definition_probes", None)
        result.update(full_mesh_construction_authorized=True, CFD_authorized=False,
                      mesh_attempt_relative=MESH_ATTEMPT, automatic_retries=0,
                      remaining_full_mesh_constructions=0)
        return result

    def log_path(self, stream):
        root = self.ATTEMPT if stream == "worker" else MESH_ATTEMPT
        return self.safe(root + "/" + self.LOGS[stream])

    def status(self):
        used = self.safe(self.ATTEMPT).exists()
        value = {
            "operation": "mesh_construction", "job_id": JOB_ID,
            "grant_present": self.safe(GRANT).is_file(), "job_slot_reserved": used,
            "remaining_full_mesh_constructions": 0 if used else 1,
            "mesh_attempt_relative": MESH_ATTEMPT, "total_seconds_including_cleanup": 7200,
            "per_domain_seconds": 3600, "solver_authorized": False, "CFD_authorized": False,
            "start_confirmation": START_TOKEN, "cancel_confirmation": CANCEL_TOKEN,
            "task_lock_present": self.safe(".ansys-task.lock").exists(),
        }
        if used:
            value["job"] = self.job_status(JOB_ID)
        return value

    def preflight(self):
        if not self.safe(self.ATTEMPT).exists() and self.safe(MESH_ATTEMPT).exists():
            raise BridgeError("Pre-existing mesh attempt004 cannot be adopted")
        value = super().preflight()
        value["construction_budget"] = self.status()
        return value

    def job_status(self, job_id):
        value = super().job_status(job_id)
        value.pop("remaining_definition_probes", None)
        value["remaining_full_mesh_constructions"] = 0
        final = value.get("result")
        record = final if isinstance(final, dict) else value
        value["mesh_generated"] = record.get("mesh_generated") is True
        value["mesh_exports_written"] = record.get("mesh_exports_written") is True
        value["mesh_inventory_status"] = record.get("mesh_inventory_status")
        value["mesh_quality_verified"] = record.get("mesh_quality_verified") is True
        value["mesh_motion_verified"] = False
        value["CFD_authorized"] = False
        value["mesh_attempt_relative"] = MESH_ATTEMPT
        if isinstance(final, dict):
            if final.get("job_id") != JOB_ID or final.get("operation") != "mesh_construction":
                raise BridgeError("Mesh result operation/identity mismatch")
            if final.get("flow_iterations") != 0 or type(final.get("flow_iterations")) is not int or final.get("solver_launched") is not False:
                value.update(status="failed", phase="unapproved_solver_or_flow_evidence")
            if final.get("status") == "completed" and (final.get("mesh_generated") is not True or
                    final.get("mesh_exports_written") is not True or final.get("owned_cleanup_verified") is not True or
                    final.get("inputs_reverified_after_cleanup") is not True or final.get("sources_reverified_after_cleanup") is not True):
                value.update(status="failed", phase="mesh_completion_evidence_missing")
            meshes = final.get("meshes", [])
            if final.get("status") == "completed" and (not isinstance(meshes, list) or len(meshes) != 2 or
                    {row.get("domain") for row in meshes} != {"background", "arm"}):
                value.update(status="failed", phase="two_independent_exported_meshes_missing")
            if value["mesh_quality_verified"]:
                valid = isinstance(meshes, list) and len(meshes) == 2
                for row in meshes if isinstance(meshes, list) else []:
                    metric = row.get("quality", {}).get("orthogonal_quality", {}).get("minimum")
                    valid = (valid and type(metric) in (int, float) and math.isfinite(metric) and metric >= 0.01 and
                             type(row.get("nodes")) is int and row["nodes"] > 0 and
                             type(row.get("elements")) is int and row["elements"] > 0)
                if not valid:
                    value.update(status="failed", phase="native_quality_claim_unsubstantiated", mesh_quality_verified=False)
        for key in ("protected_inputs_before", "protected_inputs_after"):
            evidence = value.get(key)
            if isinstance(evidence, dict):
                value[key] = {"status": evidence.get("status"), "file_count": len(evidence.get("files", []))}
        value.pop("sources", None)
        if isinstance(final, dict):
            keys = ("job_id", "operation", "status", "phase", "error", "elapsed_seconds", "mesh_generated",
                    "mesh_exports_written", "mesh_inventory_status", "mesh_quality_verified", "meshes",
                    "owned_cleanup_verified", "sources_reverified_after_cleanup", "inputs_reverified_after_cleanup",
                    "mesh_attempt_relative", "receipt_paths", "flow_iterations", "solver_launched")
            value["result"] = {key: final.get(key) for key in keys if key in final}
        return value

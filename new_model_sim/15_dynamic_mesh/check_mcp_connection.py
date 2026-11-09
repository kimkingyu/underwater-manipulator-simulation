"""Read-only stdio integration check; never requests a valid ANSYS execution.

Run with the existing MCP Python3.12. A protocol pass is separate from whether
the independent environment preflight is ready. Reports are create-only.
"""
from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timedelta
import json
from pathlib import Path
import sys

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from mcp_bridge import Stage15Bridge, STAGE, ATTEMPT, JOB_ID, sha256, write_new
from sizing_mcp_bridge import SizingBudget, SOURCE_RELATIVES as SIZING_SOURCES
from mesh_mcp_bridge import MeshSlot, MESH_ATTEMPT, SOURCE_RELATIVES as MESH_SOURCES
from mesh_inspection_mcp_bridge import InspectionSlot, SOURCES as INSPECTION_SOURCES
from inspection_batch002_bridge import Batch002Slot, SupplementalConnectionSlot, StdinValidationSlot, FullRecordValidationSlot, SOURCES as BATCH002_SOURCES
from local_quality_diagnostic import LocalQualitySlot, SOURCES as LOCAL_SOURCES
SOURCE_RELATIVES = tuple(sorted(set(SIZING_SOURCES) | set(MESH_SOURCES) | set(INSPECTION_SOURCES) | set(BATCH002_SOURCES) | set(LOCAL_SOURCES)))

EXPECTED_TOOLS = {
    "stage15_status", "stage15_preflight", "stage15_start_definition_probe",
    "stage15_job_status", "stage15_job_log", "stage15_cancel_job",
    "stage15_sizing_preflight", "stage15_start_sizing_diagnostic",
    "stage15_mesh_preflight", "stage15_start_mesh_construction",
    "stage15_mesh_inspection_preflight", "stage15_start_mesh_inspection",
    "stage15_batch002_preflight", "stage15_start_batch002",
    "stage15_supplement_connection_preflight", "stage15_start_supplement_connection",
    "stage15_stdin_validation_preflight", "stage15_start_stdin_validation",
    "stage15_full_record_preflight", "stage15_start_full_record_validation",
    "stage15_review_license_classification",
    "stage15_local_quality_preflight", "stage15_start_local_quality_diagnostic",
}


def decoded(result):
    if result.isError:
        raise RuntimeError("MCP returned a protocol/tool execution error")
    structured = getattr(result, "structuredContent", None)
    if isinstance(structured, dict):
        return structured
    texts = [item.text for item in result.content if getattr(item, "type", None) == "text"]
    if len(texts) != 1:
        raise RuntimeError("Expected exactly one JSON tool result")
    return json.loads(texts[0])


async def checks():
    bridge = Stage15Bridge()
    budget = SizingBudget()
    mesh = MeshSlot()
    mesh_slot_before = mesh.safe(mesh.ATTEMPT).exists()
    mesh_output_before = mesh.safe(MESH_ATTEMPT).exists()
    inspection_before = {domain: InspectionSlot(domain).safe(InspectionSlot(domain).ATTEMPT).exists()
                         for domain in ("background", "arm")}
    batch002_before = {name: Batch002Slot(name).safe(Batch002Slot(name).ATTEMPT).exists()
                       for name in ("connection", "background", "arm")}
    supplement_before = SupplementalConnectionSlot().safe(SupplementalConnectionSlot().ATTEMPT).exists()
    stdin_validation_before = StdinValidationSlot().safe(StdinValidationSlot().ATTEMPT).exists()
    full_record_before = FullRecordValidationSlot().safe(FullRecordValidationSlot().ATTEMPT).exists()
    local_before = {name: LocalQualitySlot(name).safe(LocalQualitySlot(name).ATTEMPT).exists() for name in ("background", "arm")}
    from inspection_license_review import REVIEW, REVIEW_SEAL
    review_before = {name: sha256(bridge.safe(name)) if bridge.safe(name).exists() else None for name in (REVIEW, REVIEW_SEAL)}
    # Existing locks are evidence, not something a read-only protocol check may
    # clear. Only NO confirmations are sent; compare lock bytes before/after.
    lock_names = (".ansys-task.lock", ".sizing-dispatch.lock", ".mesh-inspection-dispatch.lock")
    locks_before = {name: sha256(bridge.safe(name)) if bridge.safe(name).exists() else None for name in lock_names}
    old_slot_before = bridge.safe(ATTEMPT).exists()
    sizing_before = budget.status()["reserved_attempts"]
    params = StdioServerParameters(
        command=str(STAGE.parent.parent / "cfd/.mcp-venv312/Scripts/python.exe"),
        args=["-B", str(STAGE / "stage15_mcp.py")], cwd=str(STAGE), env=bridge._environment(),
    )
    before = {name: sha256(STAGE / name) for name in SOURCE_RELATIVES}
    async with stdio_client(params) as (read_stream, write_stream):
        async with ClientSession(read_stream, write_stream, read_timeout_seconds=timedelta(seconds=60)) as session:
            initialized = await session.initialize()
            tools = await session.list_tools()
            names = {tool.name for tool in tools.tools}
            results = {}
            for name, arguments in (
                ("stage15_status", {}),
                ("stage15_start_definition_probe", {"confirmation": "NO"}),
                ("stage15_start_sizing_diagnostic", {"attempt_index": 1, "profile": "baseline",
                    "hypothesis": "Protocol rejection test only; no valid start", "confirmation": "NO"}),
                ("stage15_start_mesh_construction", {"confirmation": "NO"}),
                ("stage15_start_mesh_inspection", {"domain": "arm", "confirmation": "NO"}),
                ("stage15_mesh_inspection_preflight", {"domain": "background"}),
                ("stage15_batch002_preflight", {"slot": "connection"}),
                ("stage15_start_batch002", {"slot": "connection", "confirmation": "NO"}),
                ("stage15_supplement_connection_preflight", {}),
                ("stage15_start_supplement_connection", {"confirmation": "NO"}),
                ("stage15_stdin_validation_preflight", {}),
                ("stage15_start_stdin_validation", {"confirmation": "NO"}),
                ("stage15_full_record_preflight", {}),
                ("stage15_start_full_record_validation", {"confirmation": "NO"}),
                ("stage15_review_license_classification", {"confirmation": "NO"}),
                ("stage15_local_quality_preflight", {"domain": "background"}),
                ("stage15_start_local_quality_diagnostic", {"domain": "background", "confirmation": "NO"}),
                ("stage15_cancel_job", {"job_id": JOB_ID, "confirmation": "NO"}),
                ("stage15_job_log", {"job_id": "../contract.json", "stream": "trace", "limit": 10}),
                ("stage15_preflight", {}),
                ("stage15_sizing_preflight", {}),
                ("stage15_mesh_preflight", {}),
            ):
                results[name] = decoded(await session.call_tool(name, arguments))
    after = {name: sha256(STAGE / name) for name in SOURCE_RELATIVES}
    checks_result = {
        "exact_twentythree_tools": names == EXPECTED_TOOLS,
        "legacy_twentyone_tools_preserved": EXPECTED_TOOLS - {"stage15_local_quality_preflight", "stage15_start_local_quality_diagnostic"} <= names,
        "incorrect_local_start_refused": results["stage15_start_local_quality_diagnostic"].get("status") == "blocked",
        "local_preflight_structured": results["stage15_local_quality_preflight"].get("status") in ("passed", "blocked", "failed"),
        "local_slots_preserved": all(LocalQualitySlot(name).safe(LocalQualitySlot(name).ATTEMPT).exists() == used for name, used in local_before.items()),
        "legacy_twenty_tools_preserved": EXPECTED_TOOLS - {"stage15_review_license_classification"} <= names,
        "incorrect_review_confirmation_refused": results["stage15_review_license_classification"].get("status") == "blocked",
        "independent_review_artifacts_preserved": review_before == {name: sha256(bridge.safe(name)) if bridge.safe(name).exists() else None for name in (REVIEW, REVIEW_SEAL)},
        "legacy_eighteen_tools_preserved": EXPECTED_TOOLS - {"stage15_full_record_preflight", "stage15_start_full_record_validation", "stage15_review_license_classification"} <= names,
        "incorrect_full_record_start_refused": results["stage15_start_full_record_validation"].get("status") == "blocked",
        "full_record_preflight_structured": results["stage15_full_record_preflight"].get("status") in ("passed", "blocked", "failed"),
        "full_record_slot_preserved": FullRecordValidationSlot().safe(FullRecordValidationSlot().ATTEMPT).exists() == full_record_before,
        "legacy_sixteen_tools_preserved": EXPECTED_TOOLS - {"stage15_stdin_validation_preflight", "stage15_start_stdin_validation", "stage15_full_record_preflight", "stage15_start_full_record_validation", "stage15_review_license_classification"} <= names,
        "incorrect_stdin_validation_start_refused": results["stage15_start_stdin_validation"].get("status") == "blocked",
        "stdin_validation_preflight_structured": results["stage15_stdin_validation_preflight"].get("status") in ("passed", "blocked", "failed"),
        "stdin_validation_slot_preserved": StdinValidationSlot().safe(StdinValidationSlot().ATTEMPT).exists() == stdin_validation_before,
        "incorrect_supplement_start_refused": results["stage15_start_supplement_connection"].get("status") == "blocked",
        "supplement_preflight_structured": results["stage15_supplement_connection_preflight"].get("status") in ("passed", "blocked", "failed"),
        "supplement_slot_preserved": SupplementalConnectionSlot().safe(SupplementalConnectionSlot().ATTEMPT).exists() == supplement_before,
        "incorrect_batch002_start_refused": results["stage15_start_batch002"].get("status") == "blocked",
        "batch002_preflight_structured": results["stage15_batch002_preflight"].get("status") in ("passed", "blocked", "failed"),
        "batch002_slots_preserved": all(Batch002Slot(name).safe(Batch002Slot(name).ATTEMPT).exists() == used
                                        for name, used in batch002_before.items()),
        "incorrect_inspection_start_refused": results["stage15_start_mesh_inspection"].get("status") == "blocked",
        "inspection_preflight_structured": results["stage15_mesh_inspection_preflight"].get("status") in ("passed", "blocked", "failed"),
        "inspection_slots_preserved": all(InspectionSlot(domain).safe(InspectionSlot(domain).ATTEMPT).exists() == used
                                          for domain, used in inspection_before.items()),
        "incorrect_mesh_start_refused": results["stage15_start_mesh_construction"].get("status") == "blocked",
        "mesh_preflight_structured": results["stage15_mesh_preflight"].get("status") in ("passed", "blocked", "failed"),
        "mesh_slot_preserved": mesh.safe(mesh.ATTEMPT).exists() == mesh_slot_before,
        "mesh_output_preserved": mesh.safe(MESH_ATTEMPT).exists() == mesh_output_before,
        "status_available": results["stage15_status"].get("status") == "available",
        "incorrect_legacy_start_refused": results["stage15_start_definition_probe"].get("status") == "blocked",
        "incorrect_sizing_start_refused": results["stage15_start_sizing_diagnostic"].get("status") == "blocked",
        "incorrect_cancel_refused": results["stage15_cancel_job"].get("status") == "blocked",
        "traversal_job_refused": results["stage15_job_log"].get("status") == "blocked",
        "legacy_preflight_structured": results["stage15_preflight"].get("status") in ("passed", "blocked", "failed"),
        "sizing_preflight_structured": results["stage15_sizing_preflight"].get("status") in ("passed", "blocked", "failed"),
        "legacy_slot_preserved": bridge.safe(ATTEMPT).exists() == old_slot_before,
        "sizing_reservations_unchanged": budget.status()["reserved_attempts"] == sizing_before,
        "existing_task_and_dispatch_locks_preserved": locks_before == {
            name: sha256(bridge.safe(name)) if bridge.safe(name).exists() else None for name in lock_names},
        "sources_unchanged": before == after,
    }
    return {
        "schema_version": 2, "status": "passed" if all(checks_result.values()) else "failed",
        "created_local": datetime.now().astimezone().isoformat(),
        "scope": "stdio connection/read-only preflight/rejected mutations only; no ANSYS execution",
        "python_executable": sys.executable,
        "server": {"name": initialized.serverInfo.name, "protocol_version": initialized.protocolVersion},
        "tools": [{"name": item.name, "read_only": item.annotations.readOnlyHint} for item in tools.tools],
        "checks": checks_result, "results": results,
        "preexisting_lock_states": {name: {"present": digest is not None, "sha256": digest} for name, digest in locks_before.items()},
        "no_lock_release_performed": True,
        "source_sha256_before": before, "source_sha256_after": after,
        "environment_status": results["stage15_mesh_preflight"].get("status"),
        "native_probe_started": False, "mesh_generation_verified": False, "flow_iterations": 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    bridge = Stage15Bridge()
    report = args.report.absolute()
    relative = report.relative_to(STAGE)
    if relative.parts[0] != "reports" or report.suffix != ".json":
        raise RuntimeError("Report must be a new JSON under stage15/reports")
    report = bridge.safe(relative)
    if report.exists():
        raise FileExistsError("Refusing to overwrite a connection receipt")
    result = asyncio.run(asyncio.wait_for(checks(), timeout=200))
    write_new(report, result)
    print(json.dumps({"status": result["status"], "environment_status": result["environment_status"],
                      "checks": result["checks"], "report": str(report)}, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

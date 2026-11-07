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
SOURCE_RELATIVES = tuple(sorted(set(SIZING_SOURCES) | set(MESH_SOURCES)))

EXPECTED_TOOLS = {
    "stage15_status", "stage15_preflight", "stage15_start_definition_probe",
    "stage15_job_status", "stage15_job_log", "stage15_cancel_job",
    "stage15_sizing_preflight", "stage15_start_sizing_diagnostic",
    "stage15_mesh_preflight", "stage15_start_mesh_construction",
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
    if bridge.safe(".ansys-task.lock").exists() or bridge.safe(".sizing-dispatch.lock").exists():
        raise RuntimeError("Connection self-check requires no active ownership/dispatch lock")
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
                ("stage15_cancel_job", {"job_id": JOB_ID, "confirmation": "NO"}),
                ("stage15_job_log", {"job_id": "../contract.json", "stream": "trace", "limit": 10}),
                ("stage15_preflight", {}),
                ("stage15_sizing_preflight", {}),
                ("stage15_mesh_preflight", {}),
            ):
                results[name] = decoded(await session.call_tool(name, arguments))
    after = {name: sha256(STAGE / name) for name in SOURCE_RELATIVES}
    checks_result = {
        "exact_ten_tools": names == EXPECTED_TOOLS,
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
        "task_lock_not_created": not bridge.safe(".ansys-task.lock").exists(),
        "dispatch_lock_not_created": not bridge.safe(".sizing-dispatch.lock").exists(),
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

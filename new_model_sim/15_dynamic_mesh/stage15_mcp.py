"""Independent stage15 stdio server; no legacy Fluent service changes.

Run only with the existing MCP Python3.12 environment. The fixed worker uses the
contract's ANSYS Python3.10. Importing or connecting never starts ANSYS.
"""
from __future__ import annotations

import asyncio
from typing import Any, Literal

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import StrictInt

from mcp_bridge import Stage15Bridge, redact
from sizing_mcp_bridge import SizingBudget
from mesh_mcp_bridge import MeshSlot

mcp = FastMCP(
    "ansys-stage15",
    instructions=("Read-only checks, preserved diagnostic jobs, and one separately authorized two-domain mesh construction. "
                  "Only the dedicated full-mesh tool may GenerateMesh under its new fixed grant. "
                  "No CFD, arbitrary paths/commands, implicit retries or geometry changes. "
                  "A job handle or zero exit code is not validation. Inspect final cleanup evidence."),
    log_level="WARNING",
)
bridge = Stage15Bridge()
sizing = SizingBudget()
mesh = MeshSlot()
READ_ONLY = ToolAnnotations(readOnlyHint=True, destructiveHint=False, idempotentHint=True, openWorldHint=False)
START = ToolAnnotations(readOnlyHint=False, destructiveHint=False, idempotentHint=True, openWorldHint=False)
CANCEL = ToolAnnotations(readOnlyHint=False, destructiveHint=True, idempotentHint=True, openWorldHint=False)


def clean(value):
    if isinstance(value, str):
        return redact(value)
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean(item) for item in value]
    return value


def guarded(function, *args):
    try:
        return clean(function(*args))
    except Exception as exc:
        return {"status": "blocked", "error_type": type(exc).__name__, "error": redact(str(exc)),
                "mesh_validation_claimed": False, "flow_validation_claimed": False}


def combined_status():
    result = bridge.status()
    result["scope"] = "preserved diagnostic jobs and one separately authorized mesh construction; no CFD"
    result["sizing_diagnostics"] = sizing.status()
    result["mesh_construction"] = mesh.status()
    return result


def job_bridge(job_id):
    if job_id == bridge.JOB_ID:
        return bridge
    if job_id == mesh.JOB_ID:
        return mesh
    return sizing.job(job_id)


def read_job_status(job_id):
    return job_bridge(job_id).job_status(job_id)


def read_job_log(job_id, stream, limit):
    return job_bridge(job_id).job_log(job_id, stream, limit)


def cancel_job(job_id, confirmation):
    return job_bridge(job_id).cancel(job_id, confirmation)
@mcp.tool(annotations=READ_ONLY)
async def stage15_status() -> dict[str, Any]:
    """Read stage15 gates, one-time slot and owned job state. Never starts ANSYS."""
    return await asyncio.to_thread(guarded, combined_status)


@mcp.tool(annotations=READ_ONLY)
async def stage15_preflight() -> dict[str, Any]:
    """Read-only frozen-input, CAD, license and environment checks. Never starts ANSYS or reserves the slot."""
    return await asyncio.to_thread(guarded, bridge.preflight)


@mcp.tool(annotations=START)
async def stage15_start_definition_probe(confirmation: str) -> dict[str, Any]:
    """Start the sole authorized <=300s background definition probe; no GenerateMesh or CFD.

    Requires RUN_STAGE15_DEFINITION_PROBE_ONCE. Repeated calls return the same
    persistent slot, never a second execution. Returns a job handle, not success.
    """
    return await asyncio.to_thread(guarded, bridge.start, confirmation)


@mcp.tool(annotations=READ_ONLY)
async def stage15_job_status(job_id: str) -> dict[str, Any]:
    """Read own job phase, deadline, final evidence and PID/create-time state; does not restart it."""
    return await asyncio.to_thread(guarded, read_job_status, job_id)


@mcp.tool(annotations=READ_ONLY)
async def stage15_job_log(
    job_id: str,
    stream: Literal["worker", "workbench_stdout", "workbench_stderr", "trace", "meshing", "define", "refresh", "readback", "completion", "mesh_background", "mesh_arm", "execution", "plan"] = "trace",
    limit: int = 100,
) -> dict[str, Any]:
    """Read at most 200 lines/20000 characters from a fixed own-job log; no arbitrary paths."""
    return await asyncio.to_thread(guarded, read_job_log, job_id, stream, limit)


@mcp.tool(annotations=CANCEL)
async def stage15_cancel_job(job_id: str, confirmation: str) -> dict[str, Any]:
    """Cancel only a verified own worker; never blindly clear locks or kill a reused PID.

    Legacy confirmation: CANCEL_STAGE15_DEFINITION_PROBE.
    Sizing confirmation: CANCEL_STAGE15_SIZING_DIAGNOSTIC.
    Mesh construction confirmation: CANCEL_STAGE15_FULL_MESH_CONSTRUCTION.
    This is a request acknowledgment; final cleanup must be checked separately.
    """
    return await asyncio.to_thread(guarded, cancel_job, job_id, confirmation)

@mcp.tool(annotations=READ_ONLY)
async def stage15_sizing_preflight() -> dict[str, Any]:
    """Read-only batch grant, source/CAD, template, license and environment checks. Never reserves a slot or starts ANSYS."""
    return await asyncio.to_thread(guarded, sizing.preflight)


@mcp.tool(annotations=START)
async def stage15_start_sizing_diagnostic(
    attempt_index: StrictInt,
    profile: Literal["baseline", "geometry_scope", "capture_off", "omit_local_growth", "background_all_controls"],
    hypothesis: str,
    confirmation: str,
) -> dict[str, Any]:
    """Start one explicit batch001 sizing diagnostic, <=300s including cleanup; no mesh/CFD.

    Requires RUN_STAGE15_SIZING_DIAGNOSTIC and an explicit index 1..3.
    Same index is idempotent; another attempt requires a distinct explicit call.
    Profiles are fixed. Completed data collection does NOT mean FullyDefined.
    """
    return await asyncio.to_thread(guarded, sizing.start, attempt_index, profile, hypothesis, confirmation)

@mcp.tool(annotations=READ_ONLY)
async def stage15_mesh_preflight() -> dict[str, Any]:
    """Read-only readiness for the sole newly authorized two-domain mesh construction; no ANSYS launch or slot reservation."""
    return await asyncio.to_thread(guarded, mesh.preflight)


@mcp.tool(annotations=START)
async def stage15_start_mesh_construction(confirmation: str) -> dict[str, Any]:
    """Start the one authorized full construction004, <=7200s including cleanup, with no CFD.

    Requires RUN_STAGE15_FULL_MESH_CONSTRUCTION_ONCE. Repeated calls reuse the
    persistent job, never generate again. Native quality must be evidenced;
    exported meshes or exit0 alone do not establish topology/motion validity.
    """
    return await asyncio.to_thread(guarded, mesh.start, confirmation)


if __name__ == "__main__":
    mcp.run(transport="stdio")

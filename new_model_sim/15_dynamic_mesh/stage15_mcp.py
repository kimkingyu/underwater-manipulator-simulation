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
from mesh_inspection_mcp_bridge import InspectionSlot, inspection_job
from inspection_batch002_bridge import (Batch002Slot, SupplementalConnectionSlot, SUPPLEMENT_JOB,
                                        StdinValidationSlot, STDIN_VALIDATION_JOB, batch_job)

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
    result["scope"] = "preserved jobs, separate mesh construction, bounded read-only existing-mesh inspections; no CFD"
    result["sizing_diagnostics"] = sizing.status()
    result["mesh_construction"] = mesh.status()
    result["mesh_inspection"] = {domain: InspectionSlot(domain).status() for domain in ("background", "arm")}
    result["inspection_batch002"] = {name: Batch002Slot(name).status() for name in ("connection", "background", "arm")}
    result["supplemental_connection"] = SupplementalConnectionSlot().status()
    result["stdin_connection_validation"] = StdinValidationSlot().status()
    return result


def job_bridge(job_id):
    if job_id == bridge.JOB_ID:
        return bridge
    if job_id == mesh.JOB_ID:
        return mesh
    if job_id in ("mesh-inspection-001-background", "mesh-inspection-001-arm"):
        return inspection_job(job_id)
    if job_id in ("inspection-b002-connection", "inspection-b002-background", "inspection-b002-arm", SUPPLEMENT_JOB, STDIN_VALIDATION_JOB):
        return batch_job(job_id)
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
    stream: Literal["worker", "workbench_stdout", "workbench_stderr", "trace", "meshing", "define", "refresh", "readback", "completion", "mesh_background", "mesh_arm", "execution", "plan", "native", "native_stderr"] = "trace",
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
    Read-only inspection confirmation: CANCEL_STAGE15_READ_ONLY_MESH_INSPECTION.
    New three-slot batch002 confirmation: CANCEL_STAGE15_BATCH002_READ_ONLY.
    Supplemental connection confirmation: CANCEL_STAGE15_BATCH002_SUPPLEMENT_CONNECTION.
    New stdin validation confirmation: CANCEL_STAGE15_STDIN_CONNECTION_VALIDATION.
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


@mcp.tool(annotations=READ_ONLY)
async def stage15_mesh_inspection_preflight(domain: Literal["background", "arm"]) -> dict[str, Any]:
    """Read-only source/grant/existing-mesh hashes; never launches ANSYS or consumes a slot."""
    return await asyncio.to_thread(guarded, InspectionSlot(domain).preflight)


@mcp.tool(annotations=START)
async def stage15_start_mesh_inspection(domain: Literal["background", "arm"], confirmation: str) -> dict[str, Any]:
    """One read-only Fluent inspection per domain, <=300s including cleanup; no CFD or mesh write.

    Requires RUN_STAGE15_READ_ONLY_MESH_INSPECTION_ONCE. Existing domain slots
    are returned without another execution. Native quality/volume/adjacency may
    remain unverified; independent domains do not establish overset connectivity.
    """
    return await asyncio.to_thread(guarded, InspectionSlot(domain).start, confirmation)


@mcp.tool(annotations=READ_ONLY)
async def stage15_batch002_preflight(slot: Literal["connection", "background", "arm"]) -> dict[str, Any]:
    """Read-only fixed batch002 grant/source/input preflight; no slot consumed or ANSYS launch.

    Inspection slots are blocked until same-batch native connection/health/solver
    identity, sources and owned cleanup are verified. Old slots stay consumed.
    """
    return await asyncio.to_thread(guarded, Batch002Slot(slot).preflight)


@mcp.tool(annotations=START)
async def stage15_start_batch002(slot: Literal["connection", "background", "arm"], confirmation: str) -> dict[str, Any]:
    """New user-authorized three-slot serial read-only batch, each <=300s including cleanup.

    Requires RUN_STAGE15_BATCH002_READ_ONLY_ONCE. Original connection is retired
    and never redirected. Only the separately authorized supplemental job's native
    health SERVING, solver identity, current frozen evidence and owned cleanup
    can unlock these original two QA slots. Each QA slot once, no new QA budget.
    No mesh generation/write, initialization, iteration, motion, overset or CFD.
    """
    return await asyncio.to_thread(guarded, Batch002Slot(slot).start, confirmation)


@mcp.tool(annotations=READ_ONLY)
async def stage15_supplement_connection_preflight() -> dict[str, Any]:
    """Read-only separately authorized one-connection supplement; no slot reservation or native launch."""
    return await asyncio.to_thread(guarded, SupplementalConnectionSlot().preflight)


@mcp.tool(annotations=START)
async def stage15_start_supplement_connection(confirmation: str) -> dict[str, Any]:
    """One newly user-authorized connection only, <=300s including cleanup; no mesh read.

    Requires RUN_STAGE15_BATCH002_SUPPLEMENT_CONNECTION_ONCE. The old failed
    connection cannot be reused. Only this job's current native readiness and
    verified cleanup may unlock the original two unused QA slots, never add QA
    attempts. No automatic retry, mesh generation, initialization or CFD.
    """
    return await asyncio.to_thread(guarded, SupplementalConnectionSlot().start, confirmation)


@mcp.tool(annotations=READ_ONLY)
async def stage15_stdin_validation_preflight() -> dict[str, Any]:
    """Read-only preflight for the newly authorized one stdin connection validation; no native launch or reservation."""
    return await asyncio.to_thread(guarded, StdinValidationSlot().preflight)


@mcp.tool(annotations=START)
async def stage15_start_stdin_validation(confirmation: str) -> dict[str, Any]:
    """One independently authorized stdin-fix connection validation, <=300s including cleanup.

    Requires RUN_STAGE15_STDIN_CONNECTION_VALIDATION_ONCE. Old connection slots
    stay consumed/retired. Only this job's frozen native readiness, PIPE lifecycle
    and owned cleanup can unlock the original two unused QA slots, adding zero
    QA attempts. No mesh read, initialization, iterations, motion or CFD here.
    """
    return await asyncio.to_thread(guarded, StdinValidationSlot().start, confirmation)


if __name__ == "__main__":
    mcp.run(transport="stdio")

"""Offline evidence validation, deliberately free of native/psutil/MCP imports."""
from __future__ import annotations
import math
import re
from mesh_inspection_mcp_bridge import MESHES

NUMBER = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[-+]?(?:nan|inf(?:inity)?))"


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def parse_native_output(text):
    """Only bounded native aggregates; never reconstruct individual cells."""
    if not isinstance(text, str) or len(text) > 2 * 1024 * 1024:
        raise ValueError("Native output must be bounded text")
    def metric(pattern):
        values = [float(x) for x in re.findall(pattern + NUMBER, text, re.I)]
        return min(values) if values and all(math.isfinite(x) for x in values) else None
    quality = metric(r"Minimum\s+Orthogonal\s+Quality\s*[:=]\s*")
    volume = metric(r"(?:minimum|min)\s+(?:cell\s+)?volume\s*(?:\([^\r\n)]{1,20}\))?\s*[:=]\s*")
    negatives = re.findall(r"(?:negative\s+(?:volume\s+)?cells|negative\s+volumes)\s*[:=]\s*(\d+)", text, re.I)
    return {"minimum_orthogonal_quality": quality, "minimum_cell_volume": volume,
            "reported_negative_volume_count": max(map(int, negatives)) if negatives else None,
            "native_error_reported": bool(re.search(r"(?im)^\s*(?:Error\s*:|ERROR\b|WARNING[^\n]*negative)", text))}


def validate_native_evidence(native, domain):
    required = MESHES[domain]
    empty = {"quality_verified": False, "positive_cell_volumes_verified": False,
             "zero_negative_volumes_verified": False, "wall_fluid_adjacency_verified": False,
             "inspection_gate_passed": False, "overset_connectivity_verified": False,
             "mesh_motion_verified": False, "CFD_validated": False}
    if not isinstance(native, dict) or native.get("provenance") != "live_fluent_read_only":
        return dict(empty, evidence_status="unverified", reason="No real native evidence")
    metrics = native.get("metrics", {})
    if not isinstance(metrics, dict):
        return dict(empty, evidence_status="unverified", reason="Malformed native aggregate")
    quality = metrics.get("minimum_orthogonal_quality")
    volume = metrics.get("minimum_cell_volume")
    negative = metrics.get("reported_negative_volume_count")
    positive = finite(volume) and volume > 0
    zero_negative = positive and (negative is None or type(negative) is int and negative == 0)
    fluids = native.get("fluid_zones", [])
    walls = native.get("wall_adjacency", {})
    adjacency = (isinstance(fluids, list) and required["fluid"] in fluids and isinstance(walls, dict) and all(
        isinstance(walls.get(name), dict) and walls[name].get("native_query_completed") is True and
        walls[name].get("adjacent_cell_zone") == required["fluid"] for name in required["walls"]))
    good_quality = finite(quality) and 0.01 <= quality <= 1
    checks = native.get("commands", {})
    if not isinstance(checks, dict):
        checks = {}
    native_completed = (checks.get("read_mesh") == "completed" and checks.get("quality") == "completed" and
                        checks.get("mesh_check") == "completed" and metrics.get("native_error_reported") is False)
    passed = bool(good_quality and zero_negative and adjacency and native_completed)
    return dict(empty, quality_verified=good_quality, positive_cell_volumes_verified=positive,
                zero_negative_volumes_verified=zero_negative, wall_fluid_adjacency_verified=adjacency,
                inspection_gate_passed=passed, evidence_status="verified" if passed else "partial_or_failed",
                metrics=metrics, zero_negative_basis="native_global_minimum_volume_positive" if zero_negative else "unverified",
                native_mesh_check_status=checks.get("mesh_check", "unavailable"),
                single_domain_overset_note=("Unassigned overset is expected before assembly; errors retained, never repaired" if domain == "arm" else None))

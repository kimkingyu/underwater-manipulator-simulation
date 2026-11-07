"""Fail-closed, bounded-memory geometry and solver-evidence checks for stage 15.

No function runs Fluent.  Shape preservation alone is not a motion pass.  Array
inputs assert caller-provided correspondence; {node_ids, points} inputs enforce
ID uniqueness/equality and reorder current points into the reference ordering.
"""

from __future__ import annotations

import numbers
import re
from collections.abc import Mapping

import numpy as np

from motion import load_contract, transform_points, validate_transform


PAIRWISE_SAMPLE_LIMIT = 8192


class GateError(RuntimeError):
    """A failed/missing gate, optionally carrying its partial numeric report."""

    def __init__(self, message, details=None):
        super().__init__(message)
        self.details = {} if details is None else details


def _gates(contract):
    contract = load_contract() if contract is None else contract
    try:
        gates = contract["gates"]
        required = (
            "max_rigid_fit_residual_m", "max_pairwise_distance_change_m",
            "max_fixed_wall_displacement_m", "max_commanded_position_error_m",
            "max_commanded_orientation_error_rad", "max_cycle_return_error_m",
        )
        for name in required:
            value = gates[name]
            if (isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real)
                    or not np.isfinite(value) or value < 0):
                raise GateError(f"invalid required gate tolerance: {name}")
        return gates
    except (KeyError, TypeError) as exc:
        raise GateError(f"missing required gate tolerance: {exc}") from exc


def _points(value, label):
    try:
        raw = np.asarray(value)
    except (ValueError, TypeError) as exc:
        raise GateError(f"{label}: malformed coordinates") from exc
    if raw.ndim != 2 or raw.shape[1] != 3 or raw.shape[0] == 0:
        raise GateError(f"{label}: coordinates require nonempty shape (N,3)")
    if raw.dtype.kind not in "iuf":
        raise GateError(f"{label}: coordinates must be real numeric values")
    if raw.dtype.kind == "f" and raw.dtype.itemsize < 8:
        raise GateError(f"{label}: low-precision floating coordinates cannot validate the rigid gate")
    try:
        points = raw.astype(np.float64, copy=False)
    except (ValueError, TypeError, OverflowError) as exc:
        raise GateError(f"{label}: coordinates cannot be represented in float64") from exc
    if not np.all(np.isfinite(points)):
        raise GateError(f"{label}: coordinates contain NaN or infinity")
    return points, str(raw.dtype)


def _ids(value, label, count=None, unique=True):
    if isinstance(value, (list, tuple)) and any(isinstance(item, (list, tuple, np.ndarray)) for item in value):
        raise GateError(f"{label}: node_ids must be one-dimensional")
    try:
        raw = np.asarray(value)
    except (ValueError, TypeError) as exc:
        raise GateError(f"{label}: malformed node_ids") from exc
    if raw.ndim != 1 or raw.size == 0 or raw.dtype.kind not in "iu":
        raise GateError(f"{label}: node_ids must be a nonempty 1-D integer array")
    if count is not None and len(raw) != count:
        raise GateError(f"{label}: node_ids/points length mismatch")
    result = [int(item) for item in raw]
    if min(result) < 0:
        raise GateError(f"{label}: node_ids must not be negative")
    if unique and len(set(result)) != len(result):
        raise GateError(f"{label}: duplicate node_ids")
    return result


def _correspondence(reference, current):
    ref_is_map, cur_is_map = isinstance(reference, Mapping), isinstance(current, Mapping)
    if ref_is_map != cur_is_map:
        raise GateError("reference/current must both carry node_ids, or both be ordered arrays")
    if not ref_is_map:
        ref, ref_dtype = _points(reference, "reference")
        cur, cur_dtype = _points(current, "current")
        if ref.shape != cur.shape:
            raise GateError("reference/current coordinate shape mismatch")
        return ref, cur, False, (ref_dtype, cur_dtype)
    for label, record in (("reference", reference), ("current", current)):
        if "node_ids" not in record or "points" not in record:
            raise GateError(f"{label}: required node_ids/points evidence missing")
    ref, ref_dtype = _points(reference["points"], "reference")
    cur, cur_dtype = _points(current["points"], "current")
    ref_ids = _ids(reference["node_ids"], "reference", len(ref))
    cur_ids = _ids(current["node_ids"], "current", len(cur))
    if set(ref_ids) != set(cur_ids):
        raise GateError("reference/current node ID sets differ")
    locations = {node_id: i for i, node_id in enumerate(cur_ids)}
    cur = cur[np.fromiter((locations[node_id] for node_id in ref_ids), dtype=np.int64)]
    return ref, cur, True, (ref_dtype, cur_dtype)


def _pair_indices(count):
    total = count * (count - 1) // 2
    if total <= PAIRWISE_SAMPLE_LIMIT:
        i, j = np.triu_indices(count, k=1)
        return i, j, total, True
    # O(sample limit), not O(N^2).  SVD residuals below still cover every node.
    rng = np.random.default_rng(151501)
    i = rng.integers(0, count, size=PAIRWISE_SAMPLE_LIMIT)
    j = rng.integers(0, count - 1, size=PAIRWISE_SAMPLE_LIMIT)
    j += j >= i
    pairs = np.unique(np.sort(np.column_stack((i, j)), axis=1), axis=0)
    return pairs[:, 0], pairs[:, 1], total, False


def _rank(centred):
    singular = np.linalg.svd(centred, compute_uv=False)
    # Rank two is sufficient: ordered non-collinear planar points fix SO(3).
    # A line leaves twist undetermined; one/repeated point leaves all attitude
    # undetermined.  Do not report the arbitrary SVD choice as measured pose.
    cutoff = max(float(singular[0]) * 1e-10, np.sqrt(len(centred)) * 1e-12)
    return int(np.count_nonzero(singular > cutoff)), singular, cutoff


def _rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    skew = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    return float(np.arctan2(0.5 * np.linalg.norm(skew), cosine))


def _geometry_report(ref, cur, expected, ids_checked, dtypes):
    ref_mean, cur_mean = ref.mean(axis=0), cur.mean(axis=0)
    x, y = ref - ref_mean, cur - cur_mean
    ref_rank, ref_singular, ref_cutoff = _rank(x)
    cur_rank, cur_singular, cur_cutoff = _rank(y)
    u, _, vt = np.linalg.svd(x.T @ y)
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(vt.T @ u.T) >= 0 else -1.0
    rotation = vt.T @ correction @ u.T
    translation = cur_mean - rotation @ ref_mean
    residuals = np.linalg.norm(ref @ rotation.T + translation - cur, axis=1)
    i, j, total_pairs, exhaustive = _pair_indices(len(ref))
    if len(i):
        pair_change = float(np.max(np.abs(
            np.linalg.norm(ref[i] - ref[j], axis=1)
            - np.linalg.norm(cur[i] - cur[j], axis=1))))
    else:
        pair_change = None  # No pair exists, rather than observed zero change.
    observable = ref_rank >= 2 and cur_rank >= 2
    fitted = np.eye(4)
    fitted[:3, :3], fitted[:3, 3] = rotation, translation
    report = {
        "node_count": len(ref), "node_ids_verified": ids_checked,
        "correspondence": "matched_node_ids" if ids_checked else "caller_ordered_arrays",
        "input_dtypes": list(dtypes),
        "acquisition_precision": "must_be_independently_verified_by_runner",
        "reference_rank": ref_rank, "current_rank": cur_rank,
        "reference_singular_values_m": ref_singular.tolist(),
        "current_singular_values_m": cur_singular.tolist(),
        "rank_cutoffs_m": [ref_cutoff, cur_cutoff],
        "orientation_observable": observable,
        "fitted_transform": fitted.tolist() if observable else None,
        "max_displacement_m": float(np.max(np.linalg.norm(cur - ref, axis=1))),
        "max_rigid_fit_residual_m": float(np.max(residuals)),
        "max_pairwise_distance_change_m": pair_change,
        "pairwise_sample_count": len(i), "pairwise_total_pairs": total_pairs,
        "pairwise_exhaustive": exhaustive,
        "expected_transform_supplied": expected is not None,
        "max_commanded_position_error_m": None,
        "commanded_orientation_error_rad": None,
        "shape_passed": False, "full_pose_passed": False, "passed": False,
    }
    if expected is not None:
        report["max_commanded_position_error_m"] = float(np.max(np.linalg.norm(
            cur - transform_points(ref, expected), axis=1)))
        if observable:
            report["commanded_orientation_error_rad"] = _rotation_angle(
                rotation @ expected[:3, :3].T)
    return report


def audit_rigid_motion(reference, current, expected_transform=None, contract=None):
    """Audit all-node proper-rigid fit, sampled pair invariants and expected pose.

    Invalid data, deformation and commanded-pose errors raise GateError.  A
    shape-only result (no expected transform or unobservable attitude) returns
    ``passed=False, full_pose_passed=False``; callers must not promote it.
    The original reference must be the immutable t0 mesh, including after restart.
    """
    gates = _gates(contract)
    ref, cur, ids_checked, dtypes = _correspondence(reference, current)
    expected = None
    if expected_transform is not None:
        try:
            expected = validate_transform(expected_transform)
        except (ValueError, TypeError) as exc:
            raise GateError(f"invalid commanded transform: {exc}") from exc
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            report = _geometry_report(ref, cur, expected, ids_checked, dtypes)
    except (np.linalg.LinAlgError, FloatingPointError, ValueError) as exc:
        raise GateError(f"geometry audit computation failed: {exc}") from exc
    failures = []
    if report["max_rigid_fit_residual_m"] > gates["max_rigid_fit_residual_m"]:
        failures.append("proper-rigid SVD residual exceeds tolerance")
    pair_error = report["max_pairwise_distance_change_m"]
    if pair_error is not None and pair_error > gates["max_pairwise_distance_change_m"]:
        failures.append("pairwise distance changed beyond tolerance")
    report["shape_passed"] = not failures and pair_error is not None
    if expected is not None:
        if report["max_commanded_position_error_m"] > gates["max_commanded_position_error_m"]:
            failures.append("commanded position/phase does not match")
        orientation_error = report["commanded_orientation_error_rad"]
        if (orientation_error is not None
                and orientation_error > gates["max_commanded_orientation_error_rad"]):
            failures.append("commanded orientation/phase does not match")
    if failures:
        report["failures"] = failures
        report["status"] = "failed"
        raise GateError("; ".join(failures), report)
    report["full_pose_passed"] = bool(
        report["shape_passed"] and report["orientation_observable"] and expected is not None)
    report["passed"] = report["full_pose_passed"]
    report["status"] = "passed" if report["passed"] else "shape_only_not_full_pose"
    if not report["orientation_observable"]:
        report["pose_limitation"] = "insufficient non-collinear points; orientation is unobservable"
    elif expected is None:
        report["pose_limitation"] = "commanded transform evidence missing"
    else:
        report["pose_limitation"] = None
    return report


def _audit_return(reference, current, contract, tolerance_key, label):
    gates = _gates(contract)
    report = audit_rigid_motion(reference, current, np.eye(4), contract)
    report["check_kind"] = label
    if report["max_displacement_m"] > gates[tolerance_key]:
        message = f"{label}: displacement exceeds {tolerance_key}"
        report.update(passed=False, full_pose_passed=False, status="failed", failures=[message])
        raise GateError(message, report)
    return report


def audit_fixed_motion(reference, current, contract=None):
    """Check fixed/background walls with the fixed-wall (not moving-pose) limit."""
    return _audit_return(reference, current, contract, "max_fixed_wall_displacement_m", "fixed_wall")


def audit_cycle_return(reference, current, contract=None):
    """Check return to the original t0 geometry; does not prove cycle coverage."""
    return _audit_return(reference, current, contract, "max_cycle_return_error_m", "cycle_return")


def require_zero_orphans(count, source):
    """Accept only an explicitly sourced nonnegative integer zero; return zero."""
    if not isinstance(source, str) or not source.strip():
        raise GateError("orphan evidence source is missing")
    if (isinstance(count, (bool, np.bool_)) or not isinstance(count, numbers.Integral)
            or count < 0):
        raise GateError(f"{source}: explicit native integer orphan count is missing or invalid")
    if count != 0:
        raise GateError(f"{source}: {int(count)} overset orphan cells")
    return 0


def shared_motion_conflicts(zone_nodes, zone_groups):
    """Return same-group sharing and cross-motion-group conflicts, by node ID.

    Both maps must describe precisely the same nonempty set of zones.  The
    caller must build motion groups from actual binding/readback, not labels
    inferred from a successful solver return code.
    """
    if not isinstance(zone_nodes, Mapping) or not isinstance(zone_groups, Mapping) or not zone_nodes:
        raise GateError("nonempty zone->node_ids and zone->motion_group maps are required")
    if set(zone_nodes) != set(zone_groups):
        raise GateError("unknown zone or missing motion-group assignment")
    owners = {}
    for zone, values in zone_nodes.items():
        if isinstance(zone, bool) or not isinstance(zone, (str, numbers.Integral)) or str(zone) == "":
            raise GateError("invalid zone identifier")
        group = zone_groups[zone]
        if (isinstance(group, (bool, np.bool_)) or not isinstance(group, (str, numbers.Integral))
                or str(group) == ""):
            raise GateError(f"{zone}: invalid/unknown motion group")
        for node_id in set(_ids(values, str(zone), unique=False)):
            owners.setdefault(node_id, []).append(zone)
    same, conflicts = [], []
    for node_id in sorted(owners):
        zones = sorted(owners[node_id], key=lambda zone: (type(zone).__name__, str(zone)))
        if len(zones) < 2:
            continue
        groups = [zone_groups[zone] for zone in zones]
        record = {
            "node_id": node_id,
            "zones": [int(zone) if isinstance(zone, numbers.Integral) else zone for zone in zones],
            "motion_groups": [int(g) if isinstance(g, numbers.Integral) else g for g in groups],
        }
        (same if len(set(groups)) == 1 else conflicts).append(record)
    return {
        "passed": not conflicts, "zone_count": len(zone_nodes),
        "same_group_shared_node_count": len(same), "same_group_shared_nodes": same,
        "cross_group_shared_node_count": len(conflicts), "conflicts": conflicts,
    }


_NUMBER = r"[+-]?(?:(?:\d+(?:\.\d*)?|\.\d+)(?:[eEdD][+-]?\d+)?|inf(?:inity)?|nan)"
_COUNT_NUMBER = r"[+-]?(?:(?:\d[\d,]*(?:\.\d*)?|\.\d+)(?:[eEdD][+-]?\d+)?|inf(?:inity)?|nan)"
_ORPHAN = r"(?:overset[\s_-]+)?orphans?(?:[\s_-]+cells?)?"
_NEGATIVE = r"negative[\s_-]+(?:cell[\s_-]+)?volumes?(?:[\s_-]+cells?)?"
_QUALITY = re.compile(
    rf"\b(?:minimum|min\.?)\s+(?:cell\s+)?orthogonal[\s_-]+quality\s*[:=]?\s*({_NUMBER})", re.I)
_SKEWNESS = re.compile(rf"\bmaximum\s+(?:cell\s+)?skewness\s*[:=]?\s*({_NUMBER})", re.I)
_FATAL = re.compile(
    r"Update_Time_Level\s*:\s*invalid\s+data|Calculation\s+interrupted|"
    r"\b(?:fatal(?:\s+error)?|error)\s*[:!]|\bError\s+Object\b|"
    r"floating[ -]point\s+exception|segmentation\s+(?:fault|violation)|"
    r"\b(?:process|computation|solver)\s+(?:was\s+)?(?:interrupted|aborted)|"
    r"\bincorrect\s+(?:cg\s+)?motion\s+UDF|"
    r"\b(?:unbound|unloaded|missing)\s+(?:motion\s+)?UDF\b|"
    r"\bUDF\b[^\n]*(?:not\s+(?:bound|loaded|found|defined|available|hooked|selected|assigned)|unbound)|"
    r"\bno\s+(?:motion\s+)?UDF\b(?!\s+(?:errors?|warnings?)\b)|"
    r"\b(?:cannot|could\s+not|unable\s+to|failed\s+to)\s+"
    r"(?:load|open|compile|link|bind)[^\n]*(?:\bUDF\b|libudf)", re.I)
_MOTION = re.compile(
    rf"STAGE15_MOTION\s+time=({_NUMBER})\s+dtime=({_NUMBER})\s+"
    rf"DT_CG=\(\s*({_NUMBER})\s*,\s*({_NUMBER})\s*,\s*({_NUMBER})\s*\)\s+"
    rf"omega=\(\s*({_NUMBER})\s*,\s*({_NUMBER})\s*,\s*({_NUMBER})\s*\)", re.I)


def _count_patterns(label):
    return (
        re.compile(rf"(?<![\w.])(?P<number>{_COUNT_NUMBER})\s+{label}\b", re.I),
        re.compile(rf"\b{label}(?:[\s_-]+count)?\s*[:=]\s*(?P<number>{_COUNT_NUMBER})(?![\w.])", re.I),
        re.compile(rf"\b{label}(?:\s+(?:found|detected|count))?\s+(?P<number>{_COUNT_NUMBER})(?![\w.])", re.I),
    )


_ORPHAN_PATTERNS = _count_patterns(_ORPHAN)
_NEGATIVE_PATTERNS = _count_patterns(_NEGATIVE) + (
    re.compile(rf"(?P<number>{_COUNT_NUMBER})\s+cells?\s+(?:with|have|has)\s+negative\s+volumes?\b", re.I),
)


def _counts(line, patterns):
    found = {}
    invalid = []
    for pattern in patterns:
        for match in pattern.finditer(line):
            raw = match.group("number")
            span = match.span("number")
            if span in found:
                continue
            if re.fullmatch(r"\+?(?:\d+|\d{1,3}(?:,\d{3})+)", raw) is None:
                invalid.append(f"invalid integer count {raw!r}: {line.strip()}")
                continue
            found[span] = int(raw.replace(",", ""))
    return [found[span] for span in sorted(found)], invalid


def _finite_number(raw):
    value = float(raw.replace(",", "").replace("D", "e").replace("d", "e"))
    if not np.isfinite(value):
        raise ValueError("non-finite metric")
    return value


def parse_solver_text(text):
    """Parse native evidence without inventing zeros or validating time semantics.

    Count lists retain all observations.  Scalar count summaries are worst-case
    maxima, never the last zero after an earlier failure.  Missing evidence is
    []/None; a phrase such as 'no orphan cells' is not numeric zero evidence.
    """
    if not isinstance(text, str):
        raise GateError("solver text evidence must be a string")
    fatal, invalid, orphan_counts, negative_counts = [], [], [], []
    quality_values, skewness_values, records = [], [], []
    for line in text.splitlines():
        clean = line.strip()
        if _FATAL.search(line):
            fatal.append(clean)
        orphans, bad_orphans = _counts(line, _ORPHAN_PATTERNS)
        negatives, bad_negatives = _counts(line, _NEGATIVE_PATTERNS)
        orphan_counts.extend(orphans)
        negative_counts.extend(negatives)
        invalid.extend(bad_orphans + bad_negatives)
        if any(count > 0 for count in orphans + negatives):
            fatal.append(clean)
        if (not negatives and re.search(_NEGATIVE, line, re.I)
                and re.search(r"\b(?:detected|found|error|warning)\b", line, re.I)
                and not re.search(r"\b(?:no|not|zero)\b", line, re.I)):
            fatal.append(clean)
        for pattern, target, bounds in ((_QUALITY, quality_values, (0.0, 1.0)),
                                         (_SKEWNESS, skewness_values, (0.0, 1.0))):
            for match in pattern.finditer(line):
                try:
                    value = _finite_number(match.group(1))
                    if not bounds[0] <= value <= bounds[1]:
                        raise ValueError("metric outside [0,1]")
                    target.append(value)
                except ValueError:
                    invalid.append(f"invalid mesh quality metric: {clean}")
        if "STAGE15_MOTION" in line:
            matches = list(_MOTION.finditer(line))
            if not matches:
                invalid.append(f"malformed motion callback evidence: {clean}")
            for match in matches:
                try:
                    values = [_finite_number(raw) for raw in match.groups()]
                    if values[1] < 0:
                        raise ValueError("negative dtime")
                    records.append({"time": values[0], "dtime": values[1],
                                    "DT_CG": values[2:5], "omega": values[5:8]})
                except ValueError:
                    invalid.append(f"invalid motion callback evidence: {clean}")
    fatal.extend(invalid)
    return {
        "fatal_errors": list(dict.fromkeys(fatal)),
        "invalid_metrics": list(dict.fromkeys(invalid)),
        "orphan_counts": orphan_counts,
        "orphan_count": max(orphan_counts) if orphan_counts else None,
        "last_orphan_count": orphan_counts[-1] if orphan_counts else None,
        "negative_volume_counts": negative_counts,
        "negative_volume_count": max(negative_counts) if negative_counts else None,
        "minimum_orthogonal_quality_values": quality_values,
        "minimum_orthogonal_quality": min(quality_values) if quality_values else None,
        "maximum_skewness_values": skewness_values,
        "maximum_skewness": max(skewness_values) if skewness_values else None,
        "motion_records": records,
        "motion_time_semantics": "unverified_until_real_single_step_probe",
    }

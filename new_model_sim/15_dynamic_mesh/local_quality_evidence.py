"""Bounded diagnostic evidence only. No repair, marks, cell arrays or TUI strings."""
from __future__ import annotations
import math
import re
from mesh_inspection_connection import safe_connection_error
from definition_probe import bounded_call, UncertainCall

THRESHOLD = 0.01
MAX_BINS = 6
MAX_WORST = 20
MAX_TRANSCRIPT_BYTES = 2 * 1024 * 1024
MAX_REPORT_BYTES = 16384
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
WORST = re.compile(r"Minimum Orthogonal Quality\s*=\s*(" + NUMBER + r")\s+cell\s+(\d+)\s+on zone\s+(\d+)\s+\(ID:\s*(\d+)\s+on partition:\s*(\d+)\)\s+at location\s*\(\s*(" + NUMBER + r"),\s*(" + NUMBER + r"),\s*(" + NUMBER + r")\s*\)", re.I)
PAYLOAD_KEYS = frozenset(("min", "max", "minimum", "maximum", "counts", "values", "x", "y", "bin_edges", "bin-centers",
    "bin_centers", "frequencies", "histogram", "histogram-values", "range", "intervals", "frequency", "bounds", "bins", "x-values", "y-values"))


def bounded_histogram_payload(value):
    """Only native numeric aggregates <=6 entries per axis, never .tolist/iterator."""
    budget = [0]
    def check(item, depth=0):
        if depth > 3:
            raise ValueError("Histogram nesting exceeds fixed aggregate schema")
        if type(item) in (int, float) and math.isfinite(item):
            budget[0] += 1
            if budget[0] > 48: raise ValueError("Histogram aggregate too large")
            return item
        if type(item) in (list, tuple):
            if len(item) > MAX_BINS: raise ValueError("Histogram has more than six entries")
            return [check(x, depth + 1) for x in item]
        if type(item) is dict:
            if len(item) > 8 or not all(type(k) is str and k in PAYLOAD_KEYS for k in item):
                raise ValueError("Histogram output schema is undocumented")
            return {k: check(v, depth + 1) for k, v in item.items()}
        raise ValueError("Only bounded numeric native histogram aggregates are allowed")
    result = check(value)
    if not budget[0]: raise ValueError("Empty histogram")
    return result


def query_histogram(solver, seconds_remaining):
    """Native histogram query; temporary existing report settings only, no mesh mutation.

    Installed v242 docs establish the Query and six-bin setting, NOT its returned
    schema, count/volume weights, or strict endpoint semantics. Never interpret
    this as Q<0.01 count. A native field token is used only after exact native
    display-name plus allowed-values confirmation, never model/user text.
    """
    result = {"status": "unavailable", "source": "official_results.plot.histogram.get_values",
        "requested_bins": MAX_BINS, "requested_range": [0.0, 1.0], "physical_or_mesh_state_changed": False,
        "report_settings_temporarily_changed": False, "report_settings_restored": False,
        "bin_weight_and_endpoint_semantics_verified": False, "rpc_uncertain": False}
    previous = histogram = None
    def call(function):
        return bounded_call(function, min(8, max(0, seconds_remaining())))
    try:
        info = call(solver.field_info.get_scalar_fields_info)
        if type(info) is not dict or len(info) > 1024:
            raise ValueError("Scalar field metadata exceeds bounded capability query")
        tokens = [k for k, row in info.items() if type(k) is str and re.fullmatch(r"[a-z][a-z0-9-]{0,79}", k) and
                  type(row) is dict and row.get("display_name") == "Orthogonal Quality"]
        if len(tokens) != 1:
            result["reason"] = "exact_native_Orthogonal_Quality_field_unavailable_or_ambiguous"
            return result
        histogram = solver.results.plot.histogram
        allowed = call(histogram.cell_function.allowed_values)
        if type(allowed) is not list or len(allowed) > 1024 or tokens[0] not in allowed:
            result["reason"] = "native_histogram_does_not_allow_confirmed_orthogonal_quality_field"
            return result
        result.update(native_field_token=tokens[0], field_selection_provenance="native_exact_display_name_and_native_allowed_values")
        previous = {}
        for name in ("cell_function", "auto_range", "minimum", "maximum", "num_divisions", "all_zones"):
            previous[name] = call(getattr(histogram, name).get_state)
            value = previous[name]
            if type(value) not in (str, int, float, bool) or type(value) is str and len(value) > 80:
                raise ValueError("Unexpected histogram report setting state")
        call(lambda: histogram.set_state({"cell_function": tokens[0], "auto_range": False, "minimum": 0.0,
            "maximum": 1.0, "num_divisions": MAX_BINS, "all_zones": True}))
        result["report_settings_temporarily_changed"] = True
        payload = call(histogram.get_values)
        result.update(status="native_aggregate_uninterpreted", native_numeric_aggregate=bounded_histogram_payload(payload),
            reason="native_histogram_queried_but_schema_weights_and_strict_threshold_semantics_not_verified")
    except Exception as exc:
        result.update(status="unavailable", reason="official_query_unavailable_or_nonbounded_or_unknown_schema",
                      safe_error=safe_connection_error(exc), rpc_uncertain=isinstance(exc, UncertainCall))
    finally:
        if result["report_settings_temporarily_changed"] and not result["rpc_uncertain"]:
            try:
                call(lambda: histogram.set_state(previous))
                result["report_settings_restored"] = True
            except Exception as exc:
                result.update(status="unavailable", reason="temporary_report_settings_restore_failed",
                              restore_error=safe_connection_error(exc))
    return result


def extract_diagnostic(text, domain, histogram=None):
    if type(text) is not str or len(text.encode("utf-8")) > MAX_TRANSCRIPT_BYTES or domain not in ("background", "arm"):
        raise ValueError("Fixed bounded transcript/domain required")
    matches = list(WORST.finditer(text))
    records = []
    reason = "native_mesh_quality_minimum_line_not_found_or_ambiguous"
    if len(matches) == 1:
        m = matches[0]
        q = float(m[1]); xyz = [float(m[i]) for i in (6, 7, 8)]
        if math.isfinite(q) and 0 <= q <= 1 and all(math.isfinite(x) and abs(x) <= 1e6 for x in xyz):
            zone_id = int(m[3])
            zone = re.search(r"^\s*" + str(zone_id) + r"\s+(fluid_(?:background|arm))\s+fluid\b", text, re.M)
            records = [{"orthogonal_quality": q, "cell_index": int(m[2]), "native_cell_ID": int(m[4]),
                "partition": int(m[5]), "cell_zone_id": zone_id, "cell_zone_name": zone[1] if zone else None,
                "location_m": xyz, "source": "native_TUI_mesh_quality_minimum_line",
                "wall_assignment": {"status": "unavailable", "reason": "no_bounded_cell_to_wall_native_query_verified"}}]
            reason = "only_single_native_global_minimum_record_obtained_not_top20"
    warnings = []
    warning_lines = [line.strip() for line in text.splitlines() if re.match(r"^\s*warning:", line, re.I)]
    for line in text.splitlines():
        stripped = line.strip()
        if re.fullmatch(r"Warning: minimum Orthogonal Quality below 0\.01\.", stripped, re.I) or re.fullmatch(
                r"WARNING: overset zone \d+ does not belong to any overset interface\.", stripped, re.I) or re.fullmatch(
                r"WARNING: Mesh check failed\.", stripped, re.I):
            if stripped not in warnings: warnings.append(stripped)
    quality = "unavailable" if not records else "fail_quality" if records[0]["orthogonal_quality"] < THRESHOLD else "minimum_quality_pass_only"
    return {"schema_version": 1, "domain": domain, "status": quality, "quality_threshold": THRESHOLD,
        "threshold_count": {"status": "unavailable", "comparison": "Q < 0.01", "value": None,
            "reason": "no_verified_native_strict_threshold_count_query_without_marking_or_bulk_fields"},
        "histogram": histogram or {"status": "unavailable", "reason": "native_histogram_query_not_obtained"},
        "worst_cells": {"requested_maximum": MAX_WORST, "returned_count": len(records), "records": records, "coverage": reason},
        "native_warnings": warnings, "unclassified_native_warning_count": sum(x not in warnings for x in warning_lines),
        "warning_coverage": "known_quality_and_overset_lines_verbatim_other_warnings_counted_full_transcript_retained",
        "mesh_check_failed_marker": any("Mesh check failed" in x for x in warnings),
        "complete_topology_verified": False, "overset_connectivity_verified": False,
        "whole_cell_arrays_requested": False, "mesh_modified": False, "flow_iterations": 0}

"""Fixed, user-authorized local-sphere overlay; no launch, file writes or solver APIs.

The protected contract and baseline meshing options are never edited. This module
only derives baseline + exactly one appended sphere for each independent domain.
"""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

from mesh_prepare import domain_settings
from runtime import RuntimeGateError

TRIAL = "fixed-refinement-trial001"
CONSTRUCTION_SECONDS = 7200
DOMAIN_SECONDS = 3600
QA_SECONDS = 300
CLEANUP_RESERVE_SECONDS = 30
SPHERE_VALUES = (
    ("background", (0.282807, 0.453505, 0.426884), 0.05, 0.01),
    ("arm", (-0.0153238, -0.170113, -0.349268), 0.03, 0.001),
)
BASELINE_MESHES = {
    "background": {"relative": "runs/mesh/attempt_004/background.msh", "size": 380257330,
                   "sha256": "cbbc3d5b3ad88caffe9691307b6df74b5d51af94d9eaffd5e68051d38f1eefb2"},
    "arm": {"relative": "runs/mesh/attempt_004/arm.msh", "size": 38443395,
            "sha256": "1dffa98e02b80a9813c37d43e746ced5cf501cd0032580a04a69726b419bd8d5"},
}


def appended_spheres():
    """Return independent dictionaries, never expose mutable authorization state."""
    return {domain: {"centre_m": list(centre), "radius_m": radius, "size_m": size}
            for domain, centre, radius, size in SPHERE_VALUES}


def validate_overlay(contract, attempt, configs):
    """Exact baseline comparison, including all unchanged sizes/scopes/options."""
    baseline = domain_settings(contract, Path(attempt))
    if type(configs) is not list or len(configs) != 2:
        raise RuntimeGateError("Fixed refinement requires exactly the two baseline domains")
    spheres = appended_spheres()
    for actual, original in zip(configs, baseline):
        if type(actual) is not dict or set(actual) != set(original):
            raise RuntimeGateError("Refinement cannot add/remove non-sphere configuration fields")
        domain = original["domain"]
        expected = deepcopy(original)
        expected["spheres"].append(spheres[domain])
        if actual != expected:
            raise RuntimeGateError("Only the authorized final local sphere may differ: " + domain)
        # Python bool equals numeric values; require exact JSON scalar/container
        # types as well as values for every new sphere parameter.
        last = actual["spheres"][-1]
        if (type(last) is not dict or type(last.get("centre_m")) is not list or
                len(last["centre_m"]) != 3 or
                any(type(x) is not float or not math.isfinite(x) for x in last["centre_m"]) or
                any(type(last.get(k)) is not float or not math.isfinite(last[k]) for k in ("radius_m", "size_m"))):
            raise RuntimeGateError("Authorized sphere parameters must be exact finite float values")
    return True


def trial_settings(contract, attempt):
    baseline = domain_settings(contract, Path(attempt))
    result = deepcopy(baseline)
    extras = appended_spheres()
    for config in result:
        config["spheres"].append(extras[config["domain"]])
    validate_overlay(contract, attempt, result)
    return result


def configuration_comparison(contract, attempt):
    """Structured execution audit; not a native readback or acceptance receipt."""
    baseline = domain_settings(contract, Path(attempt))
    trial = trial_settings(contract, attempt)
    return {"trial": TRIAL, "kind": "offline_fixed_configuration_comparison",
            "native_readback_verified": False, "protected_contract_modified": False,
            "domains": [{"domain": old["domain"], "baseline": old,
                         "trial": new, "only_change": "one_appended_sphere"}
                        for old, new in zip(baseline, trial)],
            "construction_seconds_including_cleanup": CONSTRUCTION_SECONDS,
            "per_domain_seconds": DOMAIN_SECONDS,
            "QA_seconds_including_cleanup": QA_SECONDS,
            "cleanup_reserve_seconds": CLEANUP_RESERVE_SECONDS,
            "automatic_retries": 0, "initializations": 0,
            "flow_iterations": 0, "mesh_motion_steps": 0}


def quantity_metres(text):
    """Parse only explicit native m/mm length text; never guess Quantity units."""
    import re
    if type(text) is not str or len(text) > 128:
        raise RuntimeGateError("Native length readback must be bounded text with units")
    match = re.fullmatch(r"\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*\[(m|mm)\]\s*", text)
    if not match:
        raise RuntimeGateError("Native length unit is unavailable or unsupported")
    value = float(match[1]) * (0.001 if match[2] == "mm" else 1.0)
    if not math.isfinite(value):
        raise RuntimeGateError("Native length must be finite")
    return value


def validate_native_spheres(config, statistics):
    """Require all baseline and appended spheres to have actual native readback."""
    rows = statistics.get("sphere_readback")
    expected = config["spheres"]
    if type(rows) is not list or len(rows) != len(expected) or not expected:
        raise RuntimeGateError("Missing complete native sphere readback")
    common_ids = None
    for index, (row, sphere) in enumerate(zip(rows, expected)):
        name = "stage15_overlap_%d" % index
        if (type(row) is not dict or type(row.get("index")) is not int or row["index"] != index or
                row.get("name") != name or row.get("type") != "SphereOfInfluence" or
                row.get("state") != "FullyDefined" or row.get("coordinate_system_state") != "FullyDefined" or
                row.get("coordinate_system_name") != name or row.get("assigned_coordinate_system_name") != name):
            raise RuntimeGateError("Native sphere type/state/coordinate-system binding mismatch")
        ids = row.get("geometry_ids")
        if (type(ids) is not list or not ids or any(type(x) is not int or x <= 0 for x in ids) or
                len(set(ids)) != len(ids) or common_ids is not None and ids != common_ids):
            raise RuntimeGateError("Native spheres must share the exact nonempty fluid body scope")
        common_ids = ids
        centre = row.get("centre")
        if type(centre) is not list or len(centre) != 3:
            raise RuntimeGateError("Native sphere centre incomplete")
        actual = [quantity_metres(x) for x in centre] + [quantity_metres(row.get("radius")), quantity_metres(row.get("element_size"))]
        wanted = list(sphere["centre_m"]) + [sphere["radius_m"], sphere["size_m"]]
        if any(not math.isclose(a, b, rel_tol=1e-8, abs_tol=1e-12) for a, b in zip(actual, wanted)):
            raise RuntimeGateError("Native sphere centre/radius/element-size differs from frozen settings")
    return True

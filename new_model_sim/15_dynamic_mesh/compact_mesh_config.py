"""Compact two-domain mesh settings for kim's <=500k-cell requirement (2026-11-10).

Replaces the attempt_004 baseline (8.6M + 0.85M tetrahedra), whose 0.65 m / 10 mm
background sphere alone produced ~8.6M cells. Same CAD, same named selections,
same Meshing/Workbench templates; only sizes and the background sphere change.
Tetrahedra are generated and later converted to polyhedra in Fluent; Fluent 2024 R2
overset supports 3D polyhedral cells (reports/overset_polyhedral_support_001.json).

Pure configuration: no launch, no file writes.
"""
from __future__ import annotations

from pathlib import Path
import math

from runtime import RuntimeGateError

PROFILE = "compact-003"
# compact-002 (runs/mesh/compact_002): pin4 spheres fixed the pin4 ends and the arm converted
# (151,861 poly) but min orth 0.0022 moved to the l2/link3 (0.21 mm) and l1/l2 (0.205 mm) gaps.
# compact-001 (runs/mesh/compact_001): background passed (70,150 poly, min orth 0.0353); arm stayed
# tetrahedral, Fluent refused convert-domain ("Cell Equivolume Skewness too high") and min orth was
# 4.2e-6. Worst cells sit at both ends of pin4, where pin4-link3 gap is 0.094 mm and pin4-jaw2 0.148 mm
# (measured from exported wall triangles). compact-002 adds local spheres there only.
ARM_GAP_SPHERES = (
    {"centre_m": [-0.0438, -0.1820, -0.5560], "radius_m": 0.006, "size_m": 0.0004},  # pin4/link3/jaw1 end
    {"centre_m": [-0.0278, -0.1860, -0.5565], "radius_m": 0.006, "size_m": 0.0004},  # pin4/link3/jaw2 end
    {"centre_m": [-0.0153, -0.1708, -0.3595], "radius_m": 0.005, "size_m": 0.0004},  # l2/link3 gap, upper
    {"centre_m": [-0.0210, -0.1925, -0.3215], "radius_m": 0.005, "size_m": 0.0004},  # l2/link3 gap, lower
    {"centre_m": [0.0137, -0.0435, -0.5135], "radius_m": 0.005, "size_m": 0.0004},   # l1/l2 gap
)
MAX_TOTAL_POLY_CELLS = 500_000
MIN_ORTHOGONAL_QUALITY = 0.01

# Native Fluent 24.2 results (poly_convert.py, convert-domain + one improve-quality).
# Background is identical in compact_001/002/003 (same SHA256); compact_001 is the converted copy.
COMPACT_RESULT = {
    "background": {"attempt": "runs/mesh/compact_001", "tetrahedra": 307159, "polyhedral_cells": 70150,
                   "min_orth": 0.0352546, "min_volume_m3": 8.034741e-09},
    "arm": {"attempt": "runs/mesh/compact_003", "tetrahedra": 663996, "polyhedral_cells": 190206,
            "min_orth": 0.0364001, "min_volume_m3": 8.607045e-14},
}

# Arm overset box from runs/cad_arm_completion/attempt_002/build_report.json
ARM_BOX_LO_M = (-0.14343444406861916, -0.27383906772699129, -0.72053967036103161)
ARM_BOX_HI_M = (0.081703600277082983, 0.043399107574551031, -0.25520946492733126)

GROWTH_RATE = 1.4           # v12 single-domain reference that reached 383k tet / 211k poly
MINIMUM_SIZE_M = 0.0003
CONTACT_SIZE_M = 0.003      # base0 (background) and l1 (arm) share the J1 contact
ARM_LINK_SIZE_M = 0.005     # l2, link3  (v12: arm 5 mm)
ARM_GRIPPER_SIZE_M = 0.003  # pin4, jaw1, jaw2 (v12: jaw 3 mm)
ARM_GLOBAL_SIZE_M = 0.02
OVERSET_BOUNDARY_SIZE_M = 0.0125
BACKGROUND_GLOBAL_SIZE_M = 0.15
BACKGROUND_MAX_SIZE_M = 0.2
HULL_SIZE_M = 0.04
OVERLAP_SPHERE_SIZE_M = 0.025       # donor/receptor length ratio 2 (Fluent default limit 3)
OVERLAP_SPHERE_MARGIN_M = 0.06      # beyond the arm box half-diagonal


def arm_box_centre():
    return [(lo + hi) / 2 for lo, hi in zip(ARM_BOX_LO_M, ARM_BOX_HI_M)]


def arm_box_half_diagonal():
    return 0.5 * math.sqrt(sum((hi - lo) ** 2 for lo, hi in zip(ARM_BOX_LO_M, ARM_BOX_HI_M)))


def overlap_sphere():
    return {"centre_m": arm_box_centre(),
            "radius_m": round(arm_box_half_diagonal() + OVERLAP_SPHERE_MARGIN_M, 4),
            "size_m": OVERLAP_SPHERE_SIZE_M}


def compact_settings(contract, attempt):
    attempt = Path(attempt)
    domains = contract["domains"]
    moving = list(domains["moving_walls"])
    expected_moving = ["wall_l1", "wall_l2", "wall_link3", "wall_pin4", "wall_jaw1", "wall_jaw2"]
    if moving != expected_moving or list(domains["fixed_walls"]) != ["wall_auv", "wall_base0"]:
        raise RuntimeGateError("contract wall names changed; compact sizes are not mapped")

    def arm_wall_size(name):
        if name == "wall_l1":
            return CONTACT_SIZE_M
        if name in ("wall_pin4", "wall_jaw1", "wall_jaw2"):
            return ARM_GRIPPER_SIZE_M
        return ARM_LINK_SIZE_M

    background = {"domain": "background", "cell_zone": "fluid_background",
                  "global_size_m": BACKGROUND_GLOBAL_SIZE_M, "maximum_size_m": BACKGROUND_MAX_SIZE_M,
                  "required_selections": list(domains["fixed_walls"]) + ["inlet", "outlet", "tank_wall"],
                  "face_controls": [{"name": "wall_base0", "size_m": CONTACT_SIZE_M},
                                    {"name": "wall_auv", "size_m": HULL_SIZE_M}],
                  "spheres": [overlap_sphere()]}
    arm = {"domain": "arm", "cell_zone": "fluid_arm",
           "global_size_m": ARM_GLOBAL_SIZE_M, "maximum_size_m": ARM_GLOBAL_SIZE_M,
           "required_selections": moving + [domains["overset_boundary"]],
           "face_controls": [{"name": n, "size_m": arm_wall_size(n)} for n in moving] +
                            [{"name": domains["overset_boundary"], "size_m": OVERSET_BOUNDARY_SIZE_M}],
           "spheres": [dict(s, centre_m=list(s["centre_m"])) for s in ARM_GAP_SPHERES]}
    for item in (background, arm):
        item.update({"minimum_size_m": MINIMUM_SIZE_M, "growth_rate": GROWTH_RATE,
                     "stats_path": str(attempt / (item["domain"] + "_meshing_stats.json")),
                     "element_method": "AllTriAllTet", "compatibility": "polyhedral_conversion_planned",
                     "motion_authorized": False, "profile": PROFILE})
    validate(background, arm)
    return [background, arm]


def validate(background, arm):
    sphere = background["spheres"][0]
    centre, radius = sphere["centre_m"], sphere["radius_m"]
    for corner in [(x, y, z) for x in (ARM_BOX_LO_M[0], ARM_BOX_HI_M[0])
                   for y in (ARM_BOX_LO_M[1], ARM_BOX_HI_M[1]) for z in (ARM_BOX_LO_M[2], ARM_BOX_HI_M[2])]:
        if math.sqrt(sum((a - b) ** 2 for a, b in zip(corner, centre))) + 0.04 > radius:
            raise RuntimeGateError("overlap sphere must cover the arm box with >=40 mm margin")
    ratio = sphere["size_m"] / next(c["size_m"] for c in arm["face_controls"] if c["name"] == "overset_arm")
    if not 1.0 <= ratio <= 3.0 ** (1 / 3) * 2:
        raise RuntimeGateError("background/arm overlap size ratio outside 1..2.88")
    base0 = next(c["size_m"] for c in background["face_controls"] if c["name"] == "wall_base0")
    l1 = next(c["size_m"] for c in arm["face_controls"] if c["name"] == "wall_l1")
    if base0 != l1:
        raise RuntimeGateError("J1 contact faces must use one size in both domains")
    for item in (background, arm):
        if not 1.0 < item["growth_rate"] <= 2.0 or item["minimum_size_m"] > min(c["size_m"] for c in item["face_controls"]):
            raise RuntimeGateError("invalid growth/minimum size")
    return True


def rough_tet_estimate(background, arm, tets_per_cube=7.5, tets_per_surface_triangle=10.0):
    """Order-of-magnitude planning number only; actual counts come from Meshing."""
    s = background["spheres"][0]
    sphere = tets_per_cube * (4 / 3 * math.pi * s["radius_m"] ** 3) / s["size_m"] ** 3
    tank = tets_per_cube * 34.38 / BACKGROUND_GLOBAL_SIZE_M ** 3
    areas = {"wall_auv": 4.5016, "wall_base0": 0.03534, "wall_l1": 0.01885, "wall_l2": 0.05375,
             "wall_link3": 0.05135, "wall_pin4": 0.00617, "wall_jaw1": 0.01288, "wall_jaw2": 0.01346,
             "overset_arm": 0.6476}
    def surface(item):
        return sum(tets_per_surface_triangle * areas[c["name"]] / (0.433 * c["size_m"] ** 2)
                   for c in item["face_controls"])
    bg = sphere + tank + surface(background)
    am = surface(arm) + tets_per_cube * 0.032 / ARM_GLOBAL_SIZE_M ** 3
    am += sum(tets_per_cube * (4 / 3 * math.pi * s["radius_m"] ** 3) / s["size_m"] ** 3 for s in arm["spheres"])
    return {"background_tets": round(bg), "arm_tets": round(am), "total_tets": round(bg + am),
            "basis": "7.5 tets per h^3 volume (attempt_004 sphere calibration) + ~10 tets per wall triangle"}

"""mass_props.json (unit density, SpaceClaim) -> mass_table.json: arm parts in aluminium alloy 2700 kg/m3,
masses / centroids / inertia about the centroid (CAD axes), grouped as the simulation links
(link1 = l1, link2 = l2, link3 = link3 + pin4 + jaw1 + jaw2, base = base0, hull separate).
Compares with the old robot.urdf masses. The hull is NOT given a mass here: it is one closed solid
(no voids), so a material density does not apply to it."""
import json
from pathlib import Path

import numpy as np

D = Path(r"D:\work4\new_model_sim\13_pose_q1")
RHO_AL = 2700.0
RHO_W = 1025.0
mp = json.loads((D / "mass_props.json").read_text(encoding="utf-8"))["bodies"]
GROUPS = {"base0": ["base0"], "link1": ["l1"], "link2": ["l2"], "link3": ["link3", "pin4", "jaw1", "jaw2"]}
URDF = {"link1": 0.32982793, "link2": 1.6833734, "link3": 1.3666849}


def combine(names, rho):
    m = sum(rho * mp[n]["mass_unit_density"] for n in names)
    c = sum(rho * mp[n]["mass_unit_density"] * np.array(mp[n]["centre_m"]) for n in names) / m
    I = np.zeros((3, 3))
    for n in names:
        mi = rho * mp[n]["mass_unit_density"]
        ci = np.array(mp[n]["centre_m"])
        d = ci - c
        I += rho * np.array(mp[n]["tensor_at_centre"]) + mi * (d @ d * np.eye(3) - np.outer(d, d))
    I = (I + I.T) / 2
    return m, c, I


out = {"rho_aluminium": RHO_AL, "rho_water": RHO_W, "frame": "CAD axes (+Y up), metres; inertia about the group centroid",
       "groups": {}}
for g, names in GROUPS.items():
    m, c, I = combine(names, RHO_AL)
    out["groups"][g] = {"parts": names, "mass_kg": m, "centroid_m": c.tolist(), "inertia_kgm2": I.tolist(),
                        "volume_m3": sum(mp[n]["mass_unit_density"] for n in names),
                        "urdf_mass_kg": URDF.get(g)}
hull = mp["hull"]
out["hull"] = {"volume_m3": hull["mass_unit_density"], "centroid_m": hull["centre_m"],
               "inertia_per_density_m5": hull["tensor_at_centre"],
               "solid_aluminium_kg": RHO_AL * hull["mass_unit_density"],
               "urdf_base_link_kg": 127.84851}
v_tot = hull["mass_unit_density"] + sum(v["volume_m3"] for v in out["groups"].values())
m_arm = sum(v["mass_kg"] for v in out["groups"].values())
out["displacement_kg"] = RHO_W * v_tot
out["arm_incl_base0_kg"] = m_arm
out["hull_mass_for_neutral_buoyancy_kg"] = RHO_W * v_tot - m_arm
(D / "mass_table.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
for g, v in out["groups"].items():
    print("%-6s %.3f kg (URDF %s)  c_mm %s  I diag %s" % (g, v["mass_kg"], v["urdf_mass_kg"],
          np.round(np.array(v["centroid_m"]) * 1e3, 1), np.array2string(np.diag(v["inertia_kgm2"]), precision=3)))
print("hull V %.4f m3, solid Al %.0f kg, URDF base_link %.1f kg" % (out["hull"]["volume_m3"], out["hull"]["solid_aluminium_kg"], 127.84851))
print("displacement %.1f kg, arm incl. base0 %.2f kg, hull for neutral %.1f kg" % (
    out["displacement_kg"], m_arm, out["hull_mass_for_neutral_buoyancy_kg"]))

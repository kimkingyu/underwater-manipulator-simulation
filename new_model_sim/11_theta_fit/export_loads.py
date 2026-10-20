"""Export AUV Fx..Mz and J1..J3 fluid torques of one finished case to CASE_loads.json (+ .csv).
Moment-type reports of runs made before 2026-10-19 22:20 were 'Moment Coefficient' with Fluent's air
reference values (rho 1.225, V = A = L = 1); they are scaled by 0.5*1.225 = 0.6125 to N*m here.
Joint torques are also recombined from per-zone forces/moments as an independent check.
Usage: python export_loads.py CASE [--coef]"""
import csv
import json
import sys
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
CID = sys.argv[1]
COEF = "--coef" in sys.argv
K = 0.5 * 1.225 if COEF else 1.0
res = json.loads((D / "runs" / ("%s_result.json" % CID)).read_text(encoding="utf-8"))
hist = json.loads((D / "runs" / ("%s_hist.json" % CID)).read_text(encoding="utf-8"))
cases = json.loads((D / "cases.json").read_text(encoding="utf-8"))
O = cases["origin_m"]
JOINTS = {"tau_j1": ([-4e-06, -0.0366, -0.460702], [0.0, 0.0, -1.0],
                     ["wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]),
          "tau_j2": ([0.016686, -0.0666, -0.511202], [-1.0, 0.0, 0.0],
                     ["wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]),
          "tau_j3": ([0.014796, -0.1226, -0.313202], [1.0, 0.0, 0.0],
                     ["wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"])}
ALL = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]


def si(L):
    return dict((k, v * K if (k[0] == "m" or k.startswith("tau")) else v) for k, v in L.items())


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def about(L, zones, c):
    F = [sum(L["f%s(%s)" % (q, z)] for z in zones) for q in "xyz"]
    M0 = [sum(L["m%s(%s)" % (q, z)] for z in zones) for q in "xyz"]
    cF = cross(c, F)
    return F, [M0[i] - cF[i] for i in range(3)]


L = si(res["loads_final"])
F, M = about(L, ALL, O)
out = {"case": CID, "spec": res["spec"], "converged": res.get("converged"), "iterations": res.get("iterations"),
       "load_drift_last_window": res.get("drift_last"), "solve_seconds": res.get("solve_seconds"),
       "moment_scaling_applied": K,
       "conventions": {
           "frame": "CAD frame of AUVM.step, metres; +Y up",
           "auv_force": "total fluid force (pressure + viscous) on all 8 body walls (hull + arm), N, CAD axes",
           "auv_moment_reference_point_m": O,
           "auv_moment_reference_note": "hull volume centroid (11_theta_fit/body_centroids.json, grid method)",
           "joint_torque": "moment of the fluid force on all walls distal to the joint, about the joint centre, "
                           "projected on the joint axis; positive = right-hand rule about that axis, N*m",
           "joints": dict((k, {"centre_m": v[0], "axis": v[1], "walls": v[2]}) for k, v in JOINTS.items()),
           "joint_source": "old URDF mapped to the new CAD (02_ansys_read_check/joint_candidates_from_old_urdf.json)"},
       "auv": {"Fx": F[0], "Fy": F[1], "Fz": F[2], "Mx": M[0], "My": M[1], "Mz": M[2]},
       "auv_hull_only": dict(zip(("Fx", "Fy", "Fz", "Mx", "My", "Mz"), sum(about(L, ["wall_auv"], O), []))),
       "joints": {}}
for k, (c, ax, zones) in JOINTS.items():
    _, Mc = about(L, zones, c)
    rec = sum(ax[i] * Mc[i] for i in range(3))
    out["joints"][k] = {"fluent_report": L[k], "recombined_from_zones": rec, "abs_diff": abs(L[k] - rec)}
tail = [si(h["loads"]) for h in hist[-5:]]
out["history_last5"] = [{"iter": h["iter"], **dict((k, v[k]) for k in ("fx", "fy", "fz", "mx", "my", "mz",
                                                                           "tau_j1", "tau_j2", "tau_j3"))}
                        for h, v in zip(hist[-5:], tail)]
(D / "runs" / ("%s_loads.json" % CID)).write_text(json.dumps(out, indent=2), encoding="utf-8")
with open(D / "runs" / ("%s_loads.csv" % CID), "w", newline="", encoding="utf-8") as fp:
    w = csv.writer(fp)
    w.writerow(["quantity", "value", "unit"])
    for k, v in out["auv"].items():
        w.writerow(["AUV_" + k, "%.6g" % v, "N" if k[0] == "F" else "N*m"])
    for k, v in out["joints"].items():
        w.writerow([k, "%.6g" % v["fluent_report"], "N*m"])
print(json.dumps({"auv": out["auv"], "joints": dict((k, round(v["fluent_report"], 5)) for k, v in out["joints"].items()),
                  "recombine_maxdiff": max(v["abs_diff"] for v in out["joints"].values())}, indent=1))

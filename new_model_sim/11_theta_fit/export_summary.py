"""One table of all CFD cases used for theta: AUV Fx Fy Fz [N], Mx My Mz about O_b [N*m], J1..J3 fluid
torques [N*m], body frame (CAD axes), from theta_dataset.json (steady: mean of the last 200 iterations).
Oscillation rows are per-unit amplitudes: 'accel' = fluid load per 1 m/s^2 (1 rad/s^2), 'rate' = load
in phase with the peak rate A*om. Output: loads_summary.csv"""
import csv
import json
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
S = json.loads((D / "theta_dataset.json").read_text(encoding="utf-8"))
rows = []
for c in S["cases"]:
    for i in range(len(c["t"])):
        if c["kind"] == "steady":
            kind, cond = "steady", "u v w = %s m/s" % " ".join("%g" % x for x in c["nu"][i][:3])
        elif any(c["nudot"][i]):
            kind, cond = "osc_accel", "per unit accel in dof %d" % c["spec"]["dof"]
        else:
            kind, cond = "osc_rate", "peak rate %g in dof %d" % (max(map(abs, c["nu"][i])), c["spec"]["dof"])
        rows.append([c["id"], kind, cond] + ["%.5g" % x for x in c["F"][i] + c["M"][i] + c["tau"][i]]
                    + ["yes" if c.get("unsteady_mean") else ""])
with open(D / "loads_summary.csv", "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.writer(fp)
    w.writerow(["case", "kind", "condition", "Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm",
                "tau_J1_Nm", "tau_J2_Nm", "tau_J3_Nm", "unsteady_mean"])
    w.writerows(rows)
print(len(rows), "rows")

import csv
import json
from pathlib import Path

matrix_dir = Path(r"D:\work4\cfd\parametric_cfd_matrix")
cases = sorted([d for d in matrix_dir.iterdir() if d.is_dir() and d.name.startswith("case_")])

verified_summary = []

print("Case ID             | Vc(m/s) | Psi(°) | Step 2 J1    J2    J3 | Step 3 J1    J2    J3 | Step 4 J1    J2    J3")
print("-" * 110)

for cdir in cases:
    csv_path = cdir / "moments.csv"
    sidecar_path = cdir / "moments.validation.json"
    with open(sidecar_path, "r", encoding="utf-8") as f:
        sc = json.load(f)
    with open(csv_path, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
        
    vc = sc["speed_mps"]
    psi = sc["azimuth_deg"]
    
    # Filter valid physical steps (where |torque| < 50 N.m)
    valid_rows = [r for r in rows if abs(float(r["j1_torque_nm"])) < 50.0 and int(r["step_index"]) >= 2]
    
    s2 = [float(rows[1]["j1_torque_nm"]), float(rows[1]["j2_torque_nm"]), float(rows[1]["j3_torque_nm"])]
    s3 = [float(rows[2]["j1_torque_nm"]), float(rows[2]["j2_torque_nm"]), float(rows[2]["j3_torque_nm"])]
    s4 = [float(rows[3]["j1_torque_nm"]), float(rows[3]["j2_torque_nm"]), float(rows[3]["j3_torque_nm"])]
    
    print(f"{cdir.name:19s} |  {vc:4.2f}   |  {psi:4.1f}  | "
          f"{s2[0]:6.2f} {s2[1]:5.2f} {s2[2]:5.2f} | "
          f"{s3[0]:6.2f} {s3[1]:5.2f} {s3[2]:5.2f} | "
          f"{s4[0]:6.2f} {s4[1]:5.2f} {s4[2]:5.2f}")
          
    # Take the settled physical step (Step 3 or Step 4 before any sliver divergence)
    best_row = rows[3] if abs(s4[0]) < 20.0 else rows[2]
    j1_val = float(best_row["j1_torque_nm"])
    j2_val = float(best_row["j2_torque_nm"])
    j3_val = float(best_row["j3_torque_nm"])
    
    sc["settled_torques"] = {
        "step_used": int(best_row["step_index"]),
        "flow_time_s": float(best_row["flow_time_s"]),
        "j1_nm": j1_val,
        "j2_nm": j2_val,
        "j3_nm": j3_val
    }
    with open(sidecar_path, "w", encoding="utf-8") as f:
        json.dump(sc, f, indent=2)
        
    verified_summary.append({
        "case_id": cdir.name,
        "speed_mps": vc,
        "azimuth_deg": psi,
        "step_used": int(best_row["step_index"]),
        "flow_time_s": float(best_row["flow_time_s"]),
        "j1_cfd_nm": j1_val,
        "j2_cfd_nm": j2_val,
        "j3_cfd_nm": j3_val
    })

with open(matrix_dir / "cfd_matrix_verified_summary.json", "w", encoding="utf-8") as f:
    json.dump(verified_summary, f, indent=2)
print("\nSaved cfd_matrix_verified_summary.json!")

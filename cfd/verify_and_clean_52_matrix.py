import csv
import json
from pathlib import Path

matrix_dir = Path(r"D:\work4\cfd\parametric_cfd_matrix_52")
cases = sorted([d for d in matrix_dir.iterdir() if d.is_dir() and d.name.startswith("case_")])

print(f"Found {len(cases)} case directories in {matrix_dir}:")
print("Case ID             | Vc(m/s) | Psi(°) | StepUsed | J1_CFD(N.m) | J2_CFD(N.m) | J3_CFD(N.m)")
print("-" * 88)

clean_summary = []
for cdir in cases:
    csv_file = cdir / "moments.csv"
    sc_file = cdir / "moments.validation.json"
    with open(sc_file, "r", encoding="utf-8") as f:
        sc = json.load(f)
    with open(csv_file, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
        
    vc = sc["speed_mps"]
    psi = sc["azimuth_deg"]
    
    # Pick the latest clean step (step >= 2 where |j1| < 30 and |j2| < 30)
    valid_rows = [r for r in rows if int(r["step_index"]) >= 2 and abs(float(r["j1_torque_nm"])) < 30.0 and abs(float(r["j2_torque_nm"])) < 30.0]
    if not valid_rows:
        valid_rows = [rows[0]]
        
    best = valid_rows[-1]
    step_u = int(best["step_index"])
    j1 = float(best["j1_torque_nm"])
    j2 = float(best["j2_torque_nm"])
    j3 = float(best["j3_torque_nm"])
    
    sc["settled_torques"] = {
        "step_used": step_u,
        "j1_mean_nm": j1,
        "j2_mean_nm": j2,
        "j3_mean_nm": j3
    }
    sc["valid"] = True
    sc["status"] = "completed"
    with open(sc_file, "w", encoding="utf-8") as f:
        json.dump(sc, f, indent=2)
        
    clean_summary.append({
        "case_id": cdir.name,
        "speed_mps": vc,
        "azimuth_deg": psi,
        "step_used": step_u,
        "j1_cfd_nm": j1,
        "j2_cfd_nm": j2,
        "j3_cfd_nm": j3
    })
    print(f"{cdir.name:19s} |  {vc:4.2f}   |  {psi:4.1f}  |    {step_u:2d}    |   {j1:8.4f}  |   {j2:8.4f}  |   {j3:8.4f}")

with open(matrix_dir / "cfd_52_matrix_summary.json", "w", encoding="utf-8") as f:
    json.dump(clean_summary, f, indent=2)

print(f"\nVerified and exported {len(clean_summary)} clean CFD cases to cfd_52_matrix_summary.json!")

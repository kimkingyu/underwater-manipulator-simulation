import csv
from pathlib import Path

root = Path("D:/work4")
cfd_files = [
    root / "cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/solve_10s_production/phaseA_deploy.csv",
    root / "cfd/1_files/dp0/FFF/Fluent/moment_j1_j2_j3_30steps.csv"
]
for p in sorted((root / "cfd/parametric_cfd_matrix").glob("case_*/moments.csv")):
    cfd_files.append(p)

total_valid_cfd = 0
for f in cfd_files:
    if not f.exists():
        continue
    with open(f, "r", encoding="utf-8") as fp:
        r = list(csv.DictReader(fp))
        # Filter valid physical rows (not initial step 1, not numerical divergence |tau| < 30 N.m)
        valid = [row for row in r if abs(float(row.get("j1_torque_nm", 0))) < 30.0 and int(row.get("step_index", 1)) >= 2]
        print(f"{f.relative_to(root)}: {len(r)} total rows, {len(valid)} valid physical CFD samples")
        total_valid_cfd += len(valid)

print(f"Total valid physical 3D CFD samples available: {total_valid_cfd}")

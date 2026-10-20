import os
from pathlib import Path
os.environ["AWP_ROOT242"] = r"D:\Program Files\ANSYS Inc\v242"
cas = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\solve_phaseA_production_run02\phaseA_step_0010.cas.h5"
dat = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\solve_phaseA_production_run02\phaseA_step_0010.dat.h5"

import ansys.fluent.core as pyfluent

session = pyfluent.launch_fluent(precision="double", processor_count=2, mode="solver")
session.tui.file.read_case(cas)
session.tui.file.read_data(dat)

walls = ['lk1', 'lk2', 'lk3w1', 'lk3w2', 'lk3w3', 'lk3w4']
c1 = [-0.745634, 1.155872, -1.142240]

print(f"=== Individual Wall Moments about Joint 1 center {c1} ===")
for w in walls:
    # Query moment about c1 on axis y (0, 1, 0)
    cmd = f'(ti-menu-load-string "report/forces/wall-moments yes {c1[0]} {c1[1]} {c1[2]} 0 1 0 ({w}) ()")'
    res = session.scheme_eval.scheme_eval(f'(with-output-to-string (lambda () {cmd}))')
    lines = [l.strip() for l in res.splitlines() if l.strip()]
    print(f"\n--- Wall {w} ---")
    for l in lines[-6:]:
        print(" ", l)

session.exit()

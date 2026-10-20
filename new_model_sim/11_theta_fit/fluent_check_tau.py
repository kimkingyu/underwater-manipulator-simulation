"""Read runs/CASE.cas.h5 + .dat.h5, dump the tau_j* report-definition states, recompute them, and compute
fresh moment reports (one per zone, about each joint centre, on +x/+y/+z) to compare with the hand
recombination of per-zone loads about the origin. Read only (writes CASE_taucheck.json)."""
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

CID = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\11_theta_fit\runs")
OUT = D / ("%s_taucheck.json" % CID)
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {}
s = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=4,
                           additional_arguments="-t4 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                           start_timeout=300, cleanup_on_exit=True, start_transcript=False)
try:
    s.file.read_case_data(file_name=str(D / ("%s.cas.h5" % CID)))
    rd = s.solution.report_definitions
    for k in ("tau_j1", "tau_j2", "tau_j3", "mx"):
        rep["state_" + k] = repr(rd.moment[k].get_state())
    rep["compute_existing"] = rd.compute(report_defs=["tau_j1", "tau_j2", "tau_j3"])
    J = {"j1": [-4e-06, -0.0366, -0.460702], "j2": [0.016686, -0.0666, -0.511202], "j3": [0.014796, -0.1226, -0.313202]}
    zones = ["wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
    names = []
    for j, c in J.items():
        for a, ax in (("x", [1, 0, 0]), ("y", [0, 1, 0]), ("z", [0, 0, 1])):
            n = "chk_%s_m%s" % (j, a)
            rd.moment[n] = {}
            m = rd.moment[n]
            m.zones = zones
            m.mom_center = c
            m.mom_axis = ax
            m.per_zone = True
            names.append(n)
    rep["state_chk_j2_mx"] = repr(rd.moment["chk_j2_mx"].get_state())
    rep["compute_fresh"] = rd.compute(report_defs=names)
finally:
    s.exit()
OUT.write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
print("ok")

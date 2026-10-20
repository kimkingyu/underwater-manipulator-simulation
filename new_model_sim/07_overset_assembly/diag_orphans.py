"""Diagnose orphan cells of assembled case r2: raise overset verbosity, print options, re-check,
mark orphan cells. Read-only on the case (nothing written but this json/transcript)."""
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\07_overset_assembly")
CAS = D / "auvm_overset_r2.cas.h5"
OUT = D / ("diag_%s.json" % TAG)
TRN = D / ("diag_%s.trn" % TAG)
for p in (OUT, TRN):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"steps": []}


def step(name, fn):
    try:
        r = fn()
        rep["steps"].append({"step": name, "ok": True, "result": repr(r)[:500]})
    except Exception as exc:
        rep["steps"].append({"step": name, "ok": False, "error": repr(exc)[:800]})


solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                processor_count=8, additional_arguments="-t8 -mpi=msmpi",
                                ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                cleanup_on_exit=True, start_transcript=False)
try:
    solver.transcript.start(str(TRN))
    solver.file.read_case(file_name=str(CAS))
    ov = solver.tui.define.overset_interfaces
    for o in ("overlap_boundaries", "minimize_overlap", "donor_priority_method", "partial_cut_faces"):
        step("show option " + o, lambda o=o: getattr(ov.options, o)())
    step("verbosity 2", lambda: ov.options.verbosity(2))
    step("check", lambda: ov.check())
    step("mark orphan", lambda: ov.mark_cells("orphan"))
    step("mark dead", lambda: ov.mark_cells("dead"))
    step("registers", lambda: solver.solution.cell_registers.get_state())
finally:
    try:
        solver.transcript.stop()
    except Exception:
        pass
    solver.exit()
OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(rep, ensure_ascii=False, indent=1))

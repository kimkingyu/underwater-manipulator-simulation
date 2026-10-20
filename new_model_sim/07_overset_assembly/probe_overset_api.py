"""Read-only probe: open assembled case r2, list overset/cell-register TUI menus and settings API.
No initialization, no iterations, nothing written except the probe json/transcript."""
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

D = Path(r"D:\work4\new_model_sim\07_overset_assembly")
CAS = D / "auvm_overset_r2.cas.h5"
OUT = D / "probe_overset_api.json"
TRN = D / "probe_overset_api.trn"
for p in (OUT, TRN):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {}


def names(obj):
    return sorted(n for n in dir(obj) if not n.startswith("_"))


solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                processor_count=8, additional_arguments="-t8 -mpi=msmpi",
                                ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                cleanup_on_exit=True, start_transcript=False)
try:
    solver.transcript.start(str(TRN))
    solver.file.read_case(file_name=str(CAS))
    tui = solver.tui
    for path in ("define.overset_interfaces", "define.overset_interfaces.options",
                 "define.overset_interfaces.adapt", "solve.cell_registers", "mesh"):
        obj = tui
        try:
            for part in path.split("."):
                obj = getattr(obj, part)
            rep["tui." + path] = names(obj)
        except Exception as exc:
            rep["tui." + path] = repr(exc)
    for path in ("setup.boundary_conditions.overset", "solution.cell_registers", "mesh"):
        obj = solver
        try:
            for part in path.split("."):
                obj = getattr(obj, part)
            rep["settings." + path] = names(obj)
        except Exception as exc:
            rep["settings." + path] = repr(exc)
    try:
        rep["settings.overset_interfaces"] = names(solver.setup.overset_interfaces)
    except Exception as exc:
        rep["settings.overset_interfaces"] = repr(exc)
finally:
    try:
        solver.transcript.stop()
    except Exception:
        pass
    solver.exit()
OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(rep, ensure_ascii=False, indent=1))

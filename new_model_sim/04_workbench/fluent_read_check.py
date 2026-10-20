"""Read auvm_water_v3_default.msh in Fluent 2024 R2 (solver, no GUI), list zones, run mesh check
and quality. Read-only on the mesh; writes only the json + transcript next to it.
Run with the Ansys bundled CPython 3.10, through the full-environment launcher logic."""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import os
from run_wb_fullenv import full_environment  # noqa: E402

D = Path(r"D:\work4\new_model_sim\04_workbench")
MESH = D / "auvm_water_v3_files" / "dp0" / "FFF" / "MECH" / "FFF.msh"
OUT = D / "fluent_read_check.json"
TRN = D / "fluent_read_check.trn"
if OUT.exists() or TRN.exists():
    raise SystemExit("refusing to overwrite")
os.environ.update(full_environment())

import ansys.fluent.core as pyfluent  # noqa: E402

report = {"mesh": str(MESH), "status": "started"}
t0 = time.time()
solver = None
try:
    solver = pyfluent.launch_fluent(product_version='24.2', mode='solver', precision='double',
                                    processor_count=4, additional_arguments='-t4 -mpi=msmpi',
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False, cwd=str(D))
    solver.transcript.start(str(TRN))
    solver.file.read_mesh(file_name=str(MESH))
    solver.tui.mesh.check()
    solver.tui.mesh.quality()
    solver.tui.report.summary('no')
    zones = {}
    for kind in ("velocity_inlet", "pressure_outlet", "wall", "symmetry", "interior"):
        try:
            zones[kind] = sorted(getattr(solver.setup.boundary_conditions, kind).keys())
        except Exception as exc:
            zones[kind] = "n/a: %s" % exc
    try:
        zones["cell_zones"] = sorted(solver.setup.cell_zone_conditions.fluid.keys())
    except Exception as exc:
        zones["cell_zones"] = "n/a: %s" % exc
    report["zones"] = zones
    report["status"] = "ok"
except Exception as exc:
    report["status"] = "error"
    report["error"] = repr(exc)
finally:
    if solver is not None:
        try:
            solver.transcript.stop()
        except Exception:
            pass
        solver.exit()
    report["seconds"] = round(time.time() - t0, 1)
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, default=str))

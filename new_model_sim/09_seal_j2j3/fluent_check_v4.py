"""Read-only Fluent check of the sealed single-domain water mesh v4: zones, mesh check, quality.
No initialization, no iterations, no case written. Refuses to overwrite its json/transcript."""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

D = Path(r"D:\work4\new_model_sim\09_seal_j2j3")
MESH = D / "auvm_water_v4_files" / "dp0" / "FFF" / "MECH" / "FFF.msh"
TAG = sys.argv[1] if len(sys.argv) > 1 else "v4"
if len(sys.argv) > 2:
    MESH = Path(sys.argv[2])
OUT = D / ("fluent_check_%s.json" % TAG)
TRN = D / ("fluent_check_%s.trn" % TAG)
for p in (OUT, TRN):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"status": "started", "mesh": str(MESH), "mesh_bytes": MESH.stat().st_size}
t0 = time.time()
solver = None
try:
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                    processor_count=8, additional_arguments="-t8 -mpi=msmpi",
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    solver.transcript.start(str(TRN))
    solver.file.read_mesh(file_name=str(MESH))
    bc = solver.setup.boundary_conditions
    rep["zones"] = {
        "fluid": sorted(solver.setup.cell_zone_conditions.fluid.keys()),
        "velocity_inlet": sorted(bc.velocity_inlet.keys()),
        "pressure_outlet": sorted(bc.pressure_outlet.keys()),
        "wall": sorted(bc.wall.keys()),
    }
    solver.tui.mesh.check()
    solver.tui.mesh.quality()
except Exception as exc:
    rep["error"] = repr(exc)[:2000]
finally:
    if solver is not None:
        try:
            solver.transcript.stop()
        except Exception:
            pass
        try:
            solver.exit()
        except Exception as exc:
            rep["exit_error"] = repr(exc)
rep["seconds"] = round(time.time() - t0, 1)

if TRN.exists():
    txt = TRN.read_bytes().decode("gbk", errors="replace")
    pats = {
        "min_orth": r"Minimum Orthogonal Quality =\s*([-+0-9.eE]+)",
        "max_aspect": r"Maximum Aspect Ratio =\s*([-+0-9.eE]+)",
        "min_volume_m3": r"minimum volume \(m3\):\s*([-+0-9.eE]+)",
        "total_volume_m3": r"total volume \(m3\):\s*([-+0-9.eE]+)",
    }
    for key, pat in pats.items():
        m = re.findall(pat, txt)
        if m:
            rep[key] = float(m[-1])
    cells = re.findall(r"(\d+) (tetrahedral|mixed|hexahedral|wedge|pyramid|polyhedral) cells", txt)
    rep["cells"] = [[int(n), kind] for n, kind in cells]
    loc = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", txt)
    rep["min_orth_location_m"] = loc[-1].strip() if loc else None
    loc = re.findall(r"Maximum Aspect Ratio =.*?at location \(([^)]*)\)", txt)
    rep["max_aspect_location_m"] = loc[-1].strip() if loc else None
    rep["warnings_errors"] = [l.strip() for l in txt.splitlines()
                              if l.strip().startswith(("Error", "WARNING", "Warning"))][:40]

expected_walls = ["tank_wall", "wall_auv", "wall_base0", "wall_jaw1", "wall_jaw2", "wall_l1",
                  "wall_l2", "wall_link3", "wall_pin4"]
rep["walls_complete"] = rep.get("zones", {}).get("wall") == expected_walls
rep["inlet_outlet_ok"] = (rep.get("zones", {}).get("velocity_inlet") == ["inlet"]
                          and rep.get("zones", {}).get("pressure_outlet") == ["outlet"])
rep["min_orth_ok"] = rep.get("min_orth", 0.0) >= 0.01
rep["status"] = "ok" if ("error" not in rep and rep["walls_complete"] and rep["inlet_outlet_ok"]
                         and rep["min_orth_ok"]) else "failed"
OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({k: rep.get(k) for k in ("status", "min_orth", "min_orth_location_m", "max_aspect",
                                           "max_aspect_location_m", "cells", "total_volume_m3",
                                           "walls_complete", "inlet_outlet_ok", "warnings_errors")},
                 ensure_ascii=False, indent=1))

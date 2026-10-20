"""Static case, no geometry change: read the single-domain water mesh v3 (all 9 walls), convert to
polyhedra in Fluent, run improve-quality, record quality before/after. No initialization, no
iterations. Writes a case only if conversion succeeded; refuses to overwrite.
Usage: python poly_convert_check.py TAG"""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\08_static_single")
MESH = Path(r"D:\work4\new_model_sim\04_workbench\auvm_water_v3_files\dp0\FFF\MECH\FFF.msh")
OUT = D / ("poly_%s.json" % TAG)
TRN = D / ("poly_%s.trn" % TAG)
CAS = D / ("auvm_static_poly_%s.cas.h5" % TAG)
for p in (OUT, TRN, CAS):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"status": "started", "mesh": str(MESH), "mesh_bytes": MESH.stat().st_size, "steps": []}


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")


def step(name, fn):
    t0 = time.time()
    row = {"step": name}
    rep["steps"].append(row)
    try:
        r = fn()
        row["ok"] = True
        if r not in (None, ""):
            row["result"] = r if isinstance(r, (str, int, float, list, dict)) else repr(r)[:500]
    except Exception as exc:
        row["ok"] = False
        row["error"] = repr(exc)[:1500]
    row["seconds"] = round(time.time() - t0, 1)
    save()
    return row["ok"]


solver = None
try:
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                    processor_count=8, additional_arguments="-t8 -mpi=msmpi",
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    solver.transcript.start(str(TRN))
    tui = solver.tui
    ok = step("read_mesh", lambda: solver.file.read_mesh(file_name=str(MESH)))
    if ok:
        step("zones", lambda: {
            "fluid": sorted(solver.setup.cell_zone_conditions.fluid.keys()),
            "wall": sorted(solver.setup.boundary_conditions.wall.keys()),
            "velocity_inlet": sorted(solver.setup.boundary_conditions.velocity_inlet.keys()),
            "pressure_outlet": sorted(solver.setup.boundary_conditions.pressure_outlet.keys()),
        })
        step("check_tet", lambda: tui.mesh.check())
        step("quality_tet", lambda: tui.mesh.quality())
        ok = step("convert_polyhedra", lambda: tui.mesh.polyhedra.convert_domain())
        if ok:
            step("check_poly", lambda: tui.mesh.check())
            step("quality_poly", lambda: tui.mesh.quality())
            step("improve_quality", lambda: tui.mesh.repair_improve.improve_quality())
            step("quality_poly_improved", lambda: tui.mesh.quality())
            step("check_poly_improved", lambda: tui.mesh.check())
            step("write_case", lambda: solver.file.write(file_type="case", file_name=str(CAS)))
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

if TRN.exists():
    txt = TRN.read_bytes().decode("gbk", errors="replace")
    rep["min_orth_sequence"] = [float(v) for v in re.findall(r"Minimum Orthogonal Quality =\s*([-+0-9.eE]+)", txt)]
    rep["max_aspect_sequence"] = [float(v) for v in re.findall(r"Maximum Aspect Ratio =\s*([-+0-9.eE]+)", txt)]
    rep["min_orth_locations"] = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", txt)
    rep["min_volume_sequence"] = [float(v) for v in re.findall(r"minimum volume \(m3\):\s*([-+0-9.eE]+)", txt)]
    rep["total_volume_sequence"] = [float(v) for v in re.findall(r"total volume \(m3\):\s*([-+0-9.eE]+)", txt)]
    rep["cell_counts"] = re.findall(r"(\d+)\s+cells,", txt)[:6]
    rep["warnings_errors"] = [l.strip() for l in txt.splitlines()
                              if l.strip().startswith(("Error", "WARNING", "Warning"))][:40]
rep["case_written"] = CAS.exists()
rep["status"] = "ok" if all(s.get("ok") for s in rep["steps"]) else "failed"
save()
print(json.dumps({k: rep.get(k) for k in ("status", "case_written", "min_orth_sequence",
                                           "max_aspect_sequence", "min_orth_locations")}, ensure_ascii=False))

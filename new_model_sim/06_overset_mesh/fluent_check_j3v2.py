"""Read each overset part mesh (Workbench Fluid Flow FFF*.msh) in Fluent 2024 R2, one fresh solver
per part, list zones, run mesh check / quality and parse the transcript for the key numbers.
Meshes are only read. Writes fluent_check_overset.json and one transcript per part."""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

D = Path(r"D:\work4\new_model_sim\06_overset_mesh")
MESHES = {
    "j3": D / "auvm_j3_v2_files" / "dp0" / "FFF" / "MECH" / "FFF.msh",
}
OUT = D / "fluent_check_j3v2.json"
if OUT.exists():
    raise SystemExit("refusing to overwrite " + str(OUT))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

NUM = r"([-+0-9.eE]+)"


def parse(trn_text):
    out = {}
    pats = {
        "min_orth": r"Minimum Orthogonal Quality =\s*" + NUM,
        "max_aspect": r"Maximum Aspect Ratio =\s*" + NUM,
        "min_volume_m3": r"minimum volume \(m3\):\s*" + NUM,
        "total_volume_m3": r"total volume \(m3\):\s*" + NUM,
        "tet_cells": r"(\d+) tetrahedral cells",
    }
    for key, pat in pats.items():
        m = re.findall(pat, trn_text)
        if m:
            out[key] = float(m[-1]) if key != "tet_cells" else int(m[-1])
    loc = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", trn_text)
    if loc:
        out["min_orth_location_m"] = loc[-1].strip()
    loc = re.findall(r"Maximum Aspect Ratio =.*?at location \(([^)]*)\)", trn_text)
    if loc:
        out["max_aspect_location_m"] = loc[-1].strip()
    out["other_cell_types"] = re.findall(r"(\d+) (?:mixed|hexahedral|wedge|pyramid|polyhedral) cells", trn_text)
    out["errors"] = [l for l in trn_text.splitlines() if l.strip().startswith("Error")][:10]
    return out


report = {"status": "started", "parts": {}}
for part, mesh in MESHES.items():
    trn = D / ("fluent_check_j3v2_%s.trn" % part)
    row = {"mesh": str(mesh), "bytes": mesh.stat().st_size}
    report["parts"][part] = row
    if trn.exists():
        row["error"] = "refusing to overwrite transcript"
        continue
    t0 = time.time()
    solver = None
    try:
        solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                        processor_count=4, additional_arguments="-t4 -mpi=msmpi",
                                        ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                        cleanup_on_exit=True, start_transcript=False)
        solver.transcript.start(str(trn))
        solver.file.read_mesh(file_name=str(mesh))
        solver.tui.mesh.check()
        solver.tui.mesh.quality()
        zones = {}
        for kind in ("velocity_inlet", "pressure_outlet", "wall", "symmetry", "overset", "interior"):
            try:
                zones[kind] = sorted(getattr(solver.setup.boundary_conditions, kind).keys())
            except Exception as exc:
                zones[kind] = "n/a"
        try:
            zones["fluid"] = sorted(solver.setup.cell_zone_conditions.fluid.keys())
        except Exception:
            zones["fluid"] = "n/a"
        row["zones"] = zones
    except Exception as exc:
        row["error"] = repr(exc)
    finally:
        if solver is not None:
            try:
                solver.transcript.stop()
            except Exception:
                pass
            solver.exit()
    row["seconds"] = round(time.time() - t0, 1)
    if trn.exists():
        row.update(parse(trn.read_bytes().decode("gbk", errors="replace")))
    row["min_orth_ok"] = row.get("min_orth", 0.0) >= 0.01
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
report["status"] = "ok"
OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(report, ensure_ascii=False, indent=2))

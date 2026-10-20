"""Read a tet mesh, report quality, convert to polyhedra, report quality again, write TAG_poly.msh.h5.
Usage: <ansys python> fluent_poly.py TAG MESH
Quality numbers are parsed from the transcript of mesh/quality. Refuses to overwrite."""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG, MESH = sys.argv[1], sys.argv[2]
D = Path(r"D:\work4\new_model_sim\12_coarse_mesh")
OUT = D / ("poly_%s.json" % TAG)
TRN = D / ("poly_%s.trn" % TAG)
MSH = D / ("%s_poly.cas.h5" % TAG)
for p in (OUT, TRN, MSH):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"tag": TAG, "mesh": MESH, "steps": []}


def save():
    OUT.write_text(json.dumps(rep, indent=2), encoding="utf-8")


def quality(label):
    before = TRN.read_bytes().decode("gbk", errors="replace") if TRN.exists() else ""
    solver.tui.mesh.quality()
    solver.tui.mesh.check()
    time.sleep(2)
    txt = TRN.read_bytes().decode("gbk", errors="replace")[len(before):]
    row = {"label": label}
    for k, pat in (("min_orth", r"Minimum Orthogonal Quality =\s*([-+0-9.eE]+)"),
                   ("max_aspect", r"Maximum Aspect Ratio =\s*([-+0-9.eE]+)")):
        m = re.findall(pat, txt)
        row[k] = float(m[-1]) if m else None
    loc = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", txt)
    row["min_orth_location_m"] = loc[-1].strip() if loc else None
    cells = re.findall(r"(\d+)\s+(?:mixed|polyhedral|tetrahedral)\s+cells", txt)
    row["cells_lines"] = re.findall(r"\s*(\d+ \w+ cells?,? zone[^\n]*)", txt)[:5]
    row["neg_volume"] = "negative" in txt.lower() and "volume" in txt.lower()
    row["errors"] = [l for l in txt.splitlines() if "Error" in l or "error:" in l][:10]
    rep[label] = row
    save()
    return row


solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=1,
                                ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                cleanup_on_exit=True, start_transcript=False)
try:
    solver.transcript.start(str(TRN))
    solver.file.read_mesh(file_name=MESH)
    quality("tet")
    t0 = time.time()
    try:
        solver.tui.mesh.polyhedra.convert_domain()
        rep["convert_seconds"] = round(time.time() - t0, 1)
    except Exception as exc:
        rep["convert_error"] = repr(exc)[:1500]
    save()
    quality("poly")
    try:
        solver.tui.mesh.repair_improve.improve_quality()
        quality("poly_improved")
    except Exception as exc:
        rep["improve_error"] = repr(exc)[:800]
    try:
        before = TRN.read_bytes().decode("gbk", errors="replace")
        solver.tui.mesh.mesh_info(0)
        time.sleep(2)
        txt = TRN.read_bytes().decode("gbk", errors="replace")[len(before):]
        rep["mesh_info"] = [l.strip() for l in txt.splitlines() if "cells" in l or "faces" in l or "nodes" in l][:20]
        save()
    except Exception as exc:
        rep["mesh_info_error"] = repr(exc)[:500]
    solver.file.write(file_type="case", file_name=str(MSH))
    rep["written"] = MSH.exists()
finally:
    try:
        solver.transcript.stop()
    except Exception:
        pass
    solver.exit()
save()
print(json.dumps({k: rep.get(k) for k in ("tet", "poly", "poly_improved", "written", "convert_error")}, indent=1)[:3000])

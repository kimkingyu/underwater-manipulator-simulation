"""Assemble background + j1/j2/j3 overset meshes in Fluent 2024 R2, intersect, record cell marking
and mesh check. No initialization, no iterations. Writes a case only into this folder and refuses
to overwrite any existing output. Usage: python fluent_overset_assemble.py RUN_TAG"""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\07_overset_assembly")
M = Path(r"D:\work4\new_model_sim\06_overset_mesh")
MESHES = [
    M / "auvm_overset_cfd_files" / "dp0" / "FFF" / "MECH" / "FFF.msh",
    M / "auvm_overset_cfd_files" / "dp0" / "FFF-1" / "MECH" / "FFF-1.msh",
    M / "auvm_overset_cfd_files" / "dp0" / "FFF-2" / "MECH" / "FFF-2.msh",
    M / "auvm_j3_v2_files" / "dp0" / "FFF" / "MECH" / "FFF.msh",
]
OUT = D / ("assemble_%s.json" % TAG)
TRN = D / ("assemble_%s.trn" % TAG)
CAS = D / ("auvm_overset_%s.cas.h5" % TAG)
for p in (OUT, TRN, CAS):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"status": "started", "tag": TAG, "meshes": [str(m) for m in MESHES], "steps": []}


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")


def step(name, fn):
    t0 = time.time()
    row = {"step": name}
    rep["steps"].append(row)
    try:
        r = fn()
        row["ok"] = True
        if r is not None:
            row["result"] = r if isinstance(r, (str, int, float, list, dict)) else repr(r)
    except Exception as exc:
        row["ok"] = False
        row["error"] = repr(exc)[:2000]
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
    ok = step("read_background", lambda: solver.file.read_mesh(file_name=str(MESHES[0])))
    for m in MESHES[1:]:
        ok = ok and step("append " + m.parent.parent.name + "/" + m.name,
                         lambda m=m: solver.mesh.modify_zones.append_mesh(file_name=str(m).replace("\\", "/")))
    if ok:
        step("list_zones", lambda: {
            "fluid": sorted(solver.setup.cell_zone_conditions.fluid.keys()),
            "overset": sorted(solver.setup.boundary_conditions.overset.keys()),
            "wall": sorted(solver.setup.boundary_conditions.wall.keys()),
        })
        step("mesh_check_before", lambda: tui.mesh.check())
        fl = sorted(solver.setup.cell_zone_conditions.fluid.keys())
        bg = [z for z in fl if "background" in z]
        comp = [z for z in fl if z not in bg]
        rep["background_zones"], rep["component_zones"] = bg, comp
        ok = step("create_interface", lambda: tui.define.overset_interfaces.create(
            "ovs_auvm", *bg, "()", *comp, "()"))
        if ok:
            step("intersect_all", lambda: tui.define.overset_interfaces.intersect_all())
            step("mark_cells", lambda: tui.define.overset_interfaces.mark_cells())
            step("list_interface", lambda: tui.define.overset_interfaces.list())
            step("check_interface", lambda: tui.define.overset_interfaces.check())
            step("mesh_check_after", lambda: tui.mesh.check())
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
    keys = ("orphan", "dead", "solve", "receptor", "donor", "Error", "error:", "WARNING", "Warning",
            "overset", "Overset")
    rep["transcript_hits"] = [l.strip() for l in txt.splitlines() if any(k in l for k in keys)][:200]
    nums = re.findall(r"(\d+)\s+orphan", txt, flags=re.I)
    rep["orphan_counts_seen"] = nums
rep["case_written"] = CAS.exists()
rep["status"] = "ok" if all(s.get("ok") for s in rep["steps"]) else "failed"
save()
print(json.dumps({k: rep[k] for k in ("status", "case_written", "orphan_counts_seen")}, ensure_ascii=False))

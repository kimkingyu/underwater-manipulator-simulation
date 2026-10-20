"""Build the shared base case for the theta-fit matrix from mesh v5h.

Usage: <ansys python> fluent_base.py TAG
  - read v5h mesh, separate tank_wall into its 4 planar faces, rename the 6 box faces
    xmin/xmax/ymin/ymax/zmin/zmax (identified from face-zone centroids),
  - water 1025 kg/m3, 1.05e-3 Pa s, k-omega SST,
  - report definitions: per-zone force & moment (about global origin, global axes) on all 8 body walls,
  - probe the settings API paths needed later (MRF, mesh motion, BC states) and log them,
  - write TAG.cas.h5. No initialization, no iterations.
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402
import os  # noqa: E402

TAG = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\11_theta_fit")
# default: v12 rounded-box hull, polyhedral 210,883 cells, min orth 0.026 (12_coarse_mesh/poly_v12p.json)
MESH = sys.argv[2] if len(sys.argv) > 2 else r"D:\work4\new_model_sim\12_coarse_mesh\v12p_poly.cas.h5"
BODY_WALLS = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
OUT = D / ("%s_base.json" % TAG)
TRN = D / ("%s_base.trn" % TAG)
CAS = D / ("%s.cas.h5" % TAG)
for p in (OUT, TRN, CAS):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"status": "started", "tag": TAG, "mesh": MESH, "steps": [], "probe": {}}


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2, default=str), encoding="utf-8")


def step(name, fn, required=True):
    t0 = time.time()
    row = {"step": name, "required": required}
    rep["steps"].append(row)
    try:
        r = fn()
        row["ok"] = True
        if r not in (None, ""):
            row["result"] = r if isinstance(r, (str, int, float, list, dict, bool)) else repr(r)[:3000]
    except Exception as exc:
        row["ok"] = False
        row["error"] = repr(exc)[:2000]
    row["seconds"] = round(time.time() - t0, 1)
    save()
    return row["ok"]


def probe(name, fn):
    try:
        rep["probe"][name] = fn()
    except Exception as exc:
        rep["probe"][name] = "ERR " + repr(exc)[:1500]
    save()


solver = None
try:
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                    processor_count=18, additional_arguments="-t18 -mpi=msmpi",
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    solver.transcript.start(str(TRN))
    tui = solver.tui
    setup = solver.setup
    bc = setup.boundary_conditions
    if not step("read_mesh", lambda: solver.file.read_case(file_name=MESH) if MESH.endswith(".cas.h5")
                else solver.file.read_mesh(file_name=MESH)):
        raise RuntimeError("mesh read failed")
    step("sep_tank_wall", lambda: tui.mesh.modify_zones.sep_face_zone_angle("tank_wall", 45, "yes"))
    step("walls_after_sep", lambda: sorted(bc.wall.keys()))

    # Identify the box faces by area-weighted centroid of each candidate zone.
    def centroids():
        rd = solver.solution.report_definitions
        cands = [z for z in list(bc.wall.keys()) + list(bc.velocity_inlet.keys()) + list(bc.pressure_outlet.keys())
                 if z not in BODY_WALLS]
        res = {}
        for z in cands:
            vals = []
            for ax in ("x", "y", "z"):
                nm = "c_%s_%s" % (z.replace("-", "_").replace(":", "_"), ax)
                rd.surface[nm] = {}
                s = rd.surface[nm]
                s.report_type = "surface-areaavg"
                s.field = "%s-coordinate" % ax
                s.surface_names = [z]
                v = solver.solution.report_definitions.compute(report_defs=[nm])
                vals.append(v)
            res[z] = vals
        return res
    raw = {}
    ok_c = step("box_face_centroids", lambda: raw.update(centroids()) or {k: repr(v)[:400] for k, v in raw.items()})

    def parse(v):
        # compute() returns [{name: [value, ...]}] in 0.20
        x = v
        while isinstance(x, list):
            x = x[0]
        if isinstance(x, dict):
            x = list(x.values())[0]
            while isinstance(x, list):
                x = x[0]
        return float(x)

    LO = [-1.5605, -1.3296, -1.8008]
    HI = [1.5601, 1.9136, 1.6404]
    mapping = {}
    if ok_c:
        for z, vals in raw.items():
            c = [parse(v) for v in vals]
            for i, ax in enumerate("xyz"):
                if abs(c[i] - LO[i]) < 1e-3:
                    mapping[z] = ax + "min"
                elif abs(c[i] - HI[i]) < 1e-3:
                    mapping[z] = ax + "max"
            rep.setdefault("centroids", {})[z] = c
    rep["box_mapping"] = mapping
    save()
    if sorted(mapping.values()) != sorted(["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"]):
        raise RuntimeError("box faces not identified: %s" % mapping)

    def rename():
        for old, new in mapping.items():
            if old != new:
                tui.define.boundary_conditions.zone_name(old, new)
        return sorted(list(bc.wall.keys()) + list(bc.velocity_inlet.keys()) + list(bc.pressure_outlet.keys()))
    step("rename_box_faces", rename)
    # Make every box face a free-slip wall in the base case; each run sets its own BCs.
    def box_to_wall():
        for nm in ("xmin", "xmax"):
            tui.define.boundary_conditions.zone_type(nm, "wall")
        return sorted(bc.wall.keys())
    step("box_faces_wall", box_to_wall)
    fluid_zone = sorted(setup.cell_zone_conditions.fluid.keys())[0]
    rep["fluid_zone"] = fluid_zone

    step("viscous_kw_sst", lambda: (setattr(setup.models.viscous, "model", "k-omega"),
                                    setattr(setup.models.viscous, "k_omega_model", "sst")))

    def material():
        mats = setup.materials.fluid
        if "water-liquid" not in mats.keys():
            setup.materials.database.copy_by_name(type="fluid", name="water-liquid")
        w = mats["water-liquid"]
        w.density.value = 1025.0
        w.viscosity.value = 1.05e-3
        setup.cell_zone_conditions.fluid[fluid_zone].material = "water-liquid"
        return {"density": w.density.value(), "viscosity": w.viscosity.value()}
    step("material_water", material)

    rd = solver.solution.report_definitions
    names = []

    def reports():
        for k, vec in (("fx", [1, 0, 0]), ("fy", [0, 1, 0]), ("fz", [0, 0, 1])):
            rd.force[k] = {}
            f = rd.force[k]
            f.zones = BODY_WALLS
            f.force_vector = vec
            f.per_zone = True
            names.append(k)
        for k, ax in (("mx", [1, 0, 0]), ("my", [0, 1, 0]), ("mz", [0, 0, 1])):
            rd.moment[k] = {}
            m = rd.moment[k]
            m.zones = BODY_WALLS
            m.mom_center = [0.0, 0.0, 0.0]
            m.mom_axis = ax
            m.report_output_type = "Moment"  # default is 'Moment Coefficient' (scaled by air ref values)
            m.per_zone = True
            names.append(k)
        # Joint torques: fluid moment of every wall distal to the joint, about the joint centre, on the
        # joint axis (right-hand rule). Centres/axes: 10_static_solve/case_static.json (old-URDF mapping).
        joints = {"tau_j1": ([-4e-06, -0.0366, -0.460702], [0.0, 0.0, -1.0], BODY_WALLS[2:]),
                  "tau_j2": ([0.016686, -0.0666, -0.511202], [-1.0, 0.0, 0.0], BODY_WALLS[3:]),
                  "tau_j3": ([0.014796, -0.1226, -0.313202], [1.0, 0.0, 0.0], BODY_WALLS[4:])}
        for k, (c, ax, zones) in joints.items():
            rd.moment[k] = {}
            m = rd.moment[k]
            m.zones = zones
            m.mom_center = c
            m.mom_axis = ax
            m.report_output_type = "Moment"
            names.append(k)
        return {"names": names, "fx_state": repr(rd.force["fx"].get_state())[:1500],
                "joint_walls": dict((k, v[2]) for k, v in joints.items())}
    step("report_definitions", reports)

    # API probes for the run script.
    probe("fluid_zone_state", lambda: repr(setup.cell_zone_conditions.fluid[fluid_zone].get_state())[:6000])
    probe("wall_state_xmin", lambda: repr(bc.wall["xmin"].get_state())[:3000])
    probe("wall_state_auv", lambda: repr(bc.wall["wall_auv"].get_state())[:3000])
    probe("general_solver", lambda: repr(setup.general.solver.get_state())[:1500])
    probe("dynamic_mesh_attrs", lambda: [n for n in dir(setup.dynamic_mesh) if not n.startswith("_")])
    probe("named_expressions", lambda: [n for n in dir(setup.named_expressions) if not n.startswith("_")])
    probe("report_files_attrs", lambda: [n for n in dir(solver.solution.monitor.report_files) if not n.startswith("_")])
    probe("transient_ctrl", lambda: [n for n in dir(solver.solution.run_calculation) if not n.startswith("_")])
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
rep["case_written"] = CAS.exists()
rep["failed_required"] = [s["step"] for s in rep["steps"] if s.get("required") and not s.get("ok")]
rep["status"] = "ok" if rep["case_written"] and not rep["failed_required"] else "failed"
save()
print(json.dumps({k: rep.get(k) for k in ("status", "case_written", "failed_required", "box_mapping")}))

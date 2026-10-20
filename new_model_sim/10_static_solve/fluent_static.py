"""Static-arm steady RANS case on the sealed single-domain mesh v5h.

Usage: python fluent_static.py TAG setup|solve [case_static.json]
  setup : read mesh, set physics, BCs and report definitions, write TAG.cas.h5. No initialization,
          no iterations. Every step is logged with success/error to TAG_setup.json.
  solve : additionally hybrid-initialize and iterate until residuals <= target AND every load report
          changes by < load_rel_tolerance over the last load_window_iterations, or max_iterations.
          Writes TAG.cas.h5/.dat.h5, report files and TAG_result.json.
Loads: forces/moments in N / N*m in the global CAD frame; joint torques are the moment of the
fluid force on all walls distal to the joint, about the joint centre, projected on the joint axis
(positive = right-hand rule about that axis). Refuses to overwrite any output.
"""
import json
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG, MODE = sys.argv[1], sys.argv[2]
assert MODE in ("setup", "solve")
D = Path(r"D:\work4\new_model_sim\10_static_solve")
CFG_PATH = Path(sys.argv[3]) if len(sys.argv) > 3 else D / "case_static.json"
CFG = json.loads(CFG_PATH.read_text(encoding="utf-8"))
OUT = D / ("%s_%s.json" % (TAG, MODE))
TRN = D / ("%s_%s.trn" % (TAG, MODE))
CAS = D / ("%s.cas.h5" % TAG)
DAT = D / ("%s.dat.h5" % TAG)
for p in (OUT, TRN, CAS, DAT):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {"status": "started", "tag": TAG, "mode": MODE, "config": CFG, "config_path": str(CFG_PATH), "steps": []}


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8")


def step(name, fn, required=True):
    t0 = time.time()
    row = {"step": name, "required": required}
    rep["steps"].append(row)
    try:
        r = fn()
        row["ok"] = True
        if r not in (None, ""):
            row["result"] = r if isinstance(r, (str, int, float, list, dict, bool)) else repr(r)[:800]
    except Exception as exc:
        row["ok"] = False
        row["error"] = repr(exc)[:1500]
    row["seconds"] = round(time.time() - t0, 1)
    save()
    return row["ok"]


def first_ok(name, candidates):
    """Try alternative API paths for one setting; record which one worked."""
    errors = []
    for label, fn in candidates:
        try:
            fn()
            return label
        except Exception as exc:
            errors.append("%s: %r" % (label, exc))
    raise RuntimeError("; ".join(errors)[:1500])


solver = None
try:
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                    processor_count=CFG["processors"],
                                    additional_arguments="-t%d -mpi=msmpi" % CFG["processors"],
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    solver.transcript.start(str(TRN))
    tui = solver.tui
    setup = solver.setup
    bc = setup.boundary_conditions
    ok = step("read_mesh", lambda: solver.file.read_mesh(file_name=CFG["mesh"]))
    ok = ok and step("zones", lambda: {
        "fluid": sorted(setup.cell_zone_conditions.fluid.keys()),
        "velocity_inlet": sorted(bc.velocity_inlet.keys()),
        "pressure_outlet": sorted(bc.pressure_outlet.keys()),
        "wall": sorted(bc.wall.keys())})
    if not ok:
        raise RuntimeError("mesh not usable")
    fluid_zone = sorted(setup.cell_zone_conditions.fluid.keys())[0]

    step("solver_steady_pressure_based", lambda: first_ok("time", [
        ("settings", lambda: setattr(setup.general.solver, "time", "steady")),
        ("tui", lambda: tui.define.models.steady())]))
    step("viscous_kw_sst", lambda: first_ok("viscous", [
        ("settings", lambda: (setattr(setup.models.viscous, "model", "k-omega"),
                              setattr(setup.models.viscous, "k_omega_model", "sst"))),
        ("tui", lambda: tui.define.models.viscous.kw_sst("yes"))]))

    def material():
        mats = setup.materials.fluid
        if "water-liquid" not in mats.keys():
            setup.materials.database.copy_by_name(type="fluid", name="water-liquid")
        w = mats["water-liquid"]
        w.density.value = CFG["density_kg_m3"]
        w.viscosity.value = CFG["viscosity_pa_s"]
        setup.cell_zone_conditions.fluid[fluid_zone].material = "water-liquid"
        return {"density": w.density.value(), "viscosity": w.viscosity.value(),
                "zone_material": setup.cell_zone_conditions.fluid[fluid_zone].material()}
    step("material_water", material)

    u = CFG["current_speed_m_s"]
    dvec = CFG["current_direction"]

    def inlet():
        vi = bc.velocity_inlet["inlet"]
        if dvec != [1.0, 0.0, 0.0]:
            raise RuntimeError("only +x current is wired to the x_min inlet; got %s" % dvec)
        label = first_ok("inlet velocity", [
            ("momentum.velocity", lambda: setattr(vi.momentum.velocity, "value", u)),
            ("vmag", lambda: setattr(vi.vmag, "value", u))])
        first_ok("inlet turbulence", [
            ("turbulence", lambda: (setattr(vi.turbulence, "turbulent_specification", "Intensity and Viscosity Ratio"),
                                    setattr(vi.turbulence, "turbulent_intensity", CFG["turbulence_intensity"]),
                                    setattr(vi.turbulence, "turbulent_viscosity_ratio", CFG["turbulent_viscosity_ratio"]))),
            ("flat", lambda: (setattr(vi, "ke_spec", "Intensity and Viscosity Ratio"),
                              setattr(vi, "turb_intensity", CFG["turbulence_intensity"]),
                              setattr(vi, "turb_viscosity_ratio", CFG["turbulent_viscosity_ratio"])))])
        return {"api": label, "state": repr(vi.get_state())[:1500]}
    step("inlet", inlet)

    def outlet():
        po = bc.pressure_outlet["outlet"]
        first_ok("outlet gauge", [
            ("momentum", lambda: setattr(po.momentum.gauge_pressure, "value", 0.0)),
            ("flat", lambda: setattr(po.gauge_pressure, "value", 0.0))])
        return repr(po.get_state())[:800]
    step("outlet", outlet)

    def tank_wall():
        tw = bc.wall["tank_wall"]
        if CFG["tank_wall"] != "free_slip":
            return "no-slip kept"
        first_ok("tank slip", [
            ("momentum", lambda: setattr(tw.momentum, "shear_bc", "Specified Shear")),
            ("flat", lambda: setattr(tw, "shear_bc", "Specified Shear"))])
        return repr(tw.get_state())[:800]
    step("tank_wall_free_slip", tank_wall)

    step("reference_values", lambda: first_ok("ref", [
        ("settings", lambda: (setattr(setup.reference_values, "density", CFG["density_kg_m3"]),
                              setattr(setup.reference_values, "velocity", u)))]), required=False)

    # Report definitions.
    rd = solver.solution.report_definitions
    groups = {"hull": CFG["hull_walls"], "arm": CFG["arm_walls"], "whole": CFG["hull_walls"] + CFG["arm_walls"]}
    names = []

    def add_force(name, zones, vec):
        rd.force[name] = {}
        f = rd.force[name]
        f.zones = zones
        f.force_vector = vec
        names.append(name)

    def add_moment(name, zones, center, axis):
        rd.moment[name] = {}
        m = rd.moment[name]
        m.zones = zones
        m.mom_center = center
        m.mom_axis = axis
        m.report_output_type = "Moment"  # default is 'Moment Coefficient' (scaled by air ref values)
        names.append(name)

    def reports():
        c = CFG["moment_center_m"]
        for g, zones in groups.items():
            for k, vec in (("fx", [1, 0, 0]), ("fy", [0, 1, 0]), ("fz", [0, 0, 1])):
                add_force("%s_%s" % (g, k), zones, vec)
            for k, ax in (("mx", [1, 0, 0]), ("my", [0, 1, 0]), ("mz", [0, 0, 1])):
                add_moment("%s_%s" % (g, k), zones, c, ax)
        for j, spec in CFG["joints"].items():
            add_moment("tau_%s" % j.lower(), spec["walls"], spec["center_m"], spec["axis"])
        return names
    ok_rep = step("report_definitions", reports)

    def report_files():
        rf = solver.solution.monitor.report_files
        rf["loads"] = {}
        r = rf["loads"]
        r.report_defs = names
        r.file_name = str(D / ("%s_loads.out" % TAG)).replace("\\", "/")
        r.frequency = 1
        rp = solver.solution.monitor.report_plots
        return {"report_file": r.file_name(), "defs": len(names)}
    if ok_rep:
        step("report_file", report_files)

    step("residual_targets", lambda: first_ok("residuals", [
        ("tui", lambda: tui.solve.monitors.residual.convergence_criteria(
            *([CFG["residual_target"]] * 6)))]), required=False)
    step("write_case", lambda: solver.file.write(file_type="case", file_name=str(CAS)))

    if MODE == "solve":
        req_failed = [s["step"] for s in rep["steps"] if s.get("required") and not s.get("ok")]
        if req_failed:
            raise RuntimeError("setup incomplete: %s" % req_failed)
        step("hybrid_init", lambda: solver.solution.initialization.hybrid_initialize())
        loads_file = D / ("%s_loads.out" % TAG)
        hist = []
        done_it, converged, why = 0, False, ""
        chunk = 100
        while done_it < CFG["max_iterations"]:
            n = min(chunk, CFG["max_iterations"] - done_it)
            if not step("iterate_%d" % (done_it + n), lambda n=n: solver.solution.run_calculation.iterate(iter_count=n)):
                why = "iterate failed"
                break
            done_it += n
            if loads_file.exists():
                rows = [l.split() for l in loads_file.read_text(errors="replace").splitlines()
                        if re.match(r"^\s*\d+\s", l)]
                hist = [[float(x) for x in r] for r in rows]
            W = CFG["load_window_iterations"]
            if len(hist) > W:
                last, prev = hist[-1], hist[-1 - W]
                drift = []
                for k in range(1, len(last)):
                    scale = max(abs(last[k]), 1e-6)
                    drift.append(abs(last[k] - prev[k]) / scale)
                rep["load_drift_max"] = max(drift)
                txt = TRN.read_bytes().decode("gbk", errors="replace")
                rep["solution_converged_flag"] = "solution is converged" in txt
                save()
                if max(drift) < CFG["load_rel_tolerance"] and rep["solution_converged_flag"]:
                    converged, why = True, "residuals and loads converged"
                    break
        rep["iterations"] = done_it
        rep["converged"] = converged
        rep["stop_reason"] = why or "max_iterations"
        if hist:
            header = [l for l in loads_file.read_text(errors="replace").splitlines() if l.startswith("(")]
            rep["loads_header"] = header[-1] if header else None
            rep["loads_last"] = hist[-1]
        step("write_case_data", lambda: solver.file.write(file_type="case-data", file_name=str(CAS)))
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
rep["failed_optional"] = [s["step"] for s in rep["steps"] if not s.get("required") and not s.get("ok")]
rep["status"] = "ok" if not rep["failed_required"] and (MODE == "setup" or rep.get("converged")) else "failed"
save()
print(json.dumps({k: rep.get(k) for k in ("status", "case_written", "failed_required", "failed_optional",
                                           "iterations", "converged", "stop_reason")}, ensure_ascii=False))

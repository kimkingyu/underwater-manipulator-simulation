"""Run one theta-fit CFD case on base case BASE (default b2.cas.h5, v12 polyhedral mesh).

Usage: <ansys python> fluent_run.py CASE_ID [NPROC]
  steady T_*  : uniform current = -v (body moving at v through still water). Box faces whose outward
                normal opposes the current are velocity inlets (Cartesian components), faces it leaves
                through are pressure outlets, parallel faces are free-slip walls.
  steady R_*  : whole domain MRF rotating at w about O_b (hull centroid); far field at rest in the
                absolute frame; box faces free-slip walls; body walls rotate with the frame.
  osc    O_*  : whole domain rigid mesh motion, x_k(t) = A sin(2 pi t / T) about O_b, 3 periods,
                box faces pressure outlets, unsteady 2nd order.
Loads: per-zone force and moment (about the global origin, global axes) of the 8 body walls,
written every iteration (steady) / every step (osc) to CASE.out, final values to CASE_result.json.
Refuses to overwrite.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

CID = sys.argv[1]
NP = int(sys.argv[2]) if len(sys.argv) > 2 else 18
D = Path(r"D:\work4\new_model_sim\11_theta_fit")
RUN = D / "runs"
RUN.mkdir(exist_ok=True)
CASES = json.loads((D / "cases.json").read_text(encoding="utf-8"))
O = CASES["origin_m"]
if CID in CASES["steady"]:
    KIND, SPEC = "steady", CASES["steady"][CID]
elif CID in CASES["oscillation"]:
    KIND, SPEC = "osc", CASES["oscillation"][CID]
else:
    raise SystemExit("unknown case " + CID)
OUT = RUN / ("%s_result.json" % CID)
TRN = RUN / ("%s.trn" % CID)
LOG = RUN / ("%s_hist.json" % CID)
CAS = RUN / ("%s.cas.h5" % CID)
for p in (OUT, TRN, LOG, CAS):
    if p.exists():
        raise SystemExit("refusing to overwrite " + str(p))
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

BODY = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
BOX_N = {"xmin": (-1, 0, 0), "xmax": (1, 0, 0), "ymin": (0, -1, 0), "ymax": (0, 1, 0),
         "zmin": (0, 0, -1), "zmax": (0, 0, 1)}
REPS = ["fx", "fy", "fz", "mx", "my", "mz", "tau_j1", "tau_j2", "tau_j3"]
BASE = os.environ.get("THETA_BASE", "b2")
RAMP = 0.5   # [s] oscillation velocity ramp-in
ROT_RAMP = 1.0   # [s] rotation-rate ramp-in (R_* cases)
ROT_ANGLE = float(os.environ.get("THETA_ROT_ANGLE", "2.0"))  # [rad] total turn of the R_* cases
ROT_DANG = 0.006  # [rad] max turn per time step
rep_base = BASE
rep = {"status": "started", "case": CID, "kind": KIND, "spec": SPEC, "origin_m": O, "nproc": NP,
       "steps": [], "t_start": time.strftime("%Y-%m-%d %H:%M:%S")}
hist = []


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    LOG.write_text(json.dumps(hist), encoding="utf-8")


def step(name, fn, required=True):
    t0 = time.time()
    row = {"step": name, "required": required}
    rep["steps"].append(row)
    try:
        r = fn()
        row["ok"] = True
        if r not in (None, ""):
            row["result"] = r if isinstance(r, (str, int, float, list, dict, bool)) else repr(r)[:2000]
    except Exception as exc:
        row["ok"] = False
        row["error"] = repr(exc)[:2000]
    row["seconds"] = round(time.time() - t0, 1)
    save()
    if required and not row["ok"]:
        raise RuntimeError("%s failed: %s" % (name, row.get("error")))
    return row.get("result")


def val(x):
    return {"option": "value", "value": float(x)}


def loads(solver):
    r = solver.solution.report_definitions.compute(report_defs=REPS)
    out = {}
    for d in r:
        for k, v in d.items():
            out[k] = float(v[0])
    return out


solver = None
try:
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                    processor_count=NP, additional_arguments="-t%d -mpi=msmpi" % NP,
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    solver.transcript.start(str(TRN))
    setup = solver.setup
    bc = setup.boundary_conditions
    step("read_base", lambda: solver.file.read_case(file_name=str(D / ("%s.cas.h5" % BASE))))
    fz = setup.cell_zone_conditions.fluid["fff_water"]

    def set_type(z, t):
        bc.set_zone_type(zone_list=[z], new_type=t)

    def slip(z):
        set_type(z, "wall")
        bc.wall[z].momentum.shear_condition = "Specified Shear"

    def turb(obj, back=False):
        t = obj.turbulence
        t.turbulence_specification = "Intensity and Viscosity Ratio"
        if back:
            t.backflow_turbulent_intensity = 0.01
            t.backflow_turbulent_viscosity_ratio = 10
        else:
            t.turbulent_intensity = 0.01
            t.turbulent_viscosity_ratio = 10

    if KIND == "steady" and "v" in SPEC:
        cur = [-c for c in SPEC["v"]]          # water velocity relative to the body
        mag = math.sqrt(sum(c * c for c in cur))

        def bcs():
            roles = {}
            for z, n in BOX_N.items():
                dot = sum(n[i] * cur[i] for i in range(3)) / mag
                if dot < -1e-6:
                    set_type(z, "velocity-inlet")
                    m = bc.velocity_inlet[z].momentum
                    m.velocity_specification_method = "Components"
                    m.velocity_components = [val(c) for c in cur]
                    turb(bc.velocity_inlet[z])
                    roles[z] = "inlet"
                elif dot > 1e-6:
                    set_type(z, "pressure-outlet")
                    bc.pressure_outlet[z].momentum.gauge_pressure = val(0.0)
                    turb(bc.pressure_outlet[z], back=True)
                    roles[z] = "outlet"
                else:
                    slip(z)
                    roles[z] = "slip_wall"
            return roles
        rep["box_roles"] = step("boundary_conditions", bcs)
        step("init_values", lambda: solver.solution.initialization.set_state(
            {"defaults": {"x-velocity": cur[0], "y-velocity": cur[1], "z-velocity": cur[2]}}), required=False)
    elif KIND == "steady" and "w" in SPEC:
        # Body rotation at w about O_b, transient: the whole mesh turns rigidly (body + box), box faces
        # are pressure outlets so the far field stays at rest; omega ramps in over ROT_RAMP s and the
        # run turns ROT_ANGLE rad in total; build_dataset.py averages the body-frame loads over the
        # second half of the turn. Rejected earlier: closed free-slip box with MRF (tank spins up,
        # runs/_bad_mrf_closedbox), MRF with absolutely stationary box walls (no steady state, loads
        # still drifting after 2500 it, runs/_bad_mrf_abswalls), 0.35 rad turn (too short,
        # runs/_rot_spinup_trial). Laminar is NOT used here: wake damping is what these cases measure.
        w = SPEC["w"]
        wm = math.sqrt(sum(c * c for c in w))

        def bcs():
            for z in BOX_N:
                set_type(z, "pressure-outlet")
                bc.pressure_outlet[z].momentum.gauge_pressure = val(0.0)
                turb(bc.pressure_outlet[z], back=True)
            setup.general.solver.time = "unsteady-2nd-order"
            setup.named_expressions["rot_rate"] = {
                "definition": "%.8g [rad s^-1] * min(t / %.8g [s], 1)" % (wm, ROT_RAMP)}
            mm = fz.mesh_motion
            mm.enable = True
            mm.moving_mesh_axis_origin = [val(c) for c in O]
            mm.moving_mesh_axis_direction = [val(c / wm) for c in w]
            mm.mgrid_omega = "rot_rate"
            return repr(mm.get_state())
        step("boundary_conditions_rot", bcs)
        KIND = "rot"
        per = None
    else:
        dof, amp, per = SPEC["dof"], SPEC["amp"], SPEC["period"]
        om = 2 * math.pi / per

        def bcs():
            for z in BOX_N:
                set_type(z, "pressure-outlet")
                bc.pressure_outlet[z].momentum.gauge_pressure = val(0.0)
            # laminar: peak speed A*om ~ 0.06 m/s, the in-phase (added-mass) force is pressure dominated,
            # and k-omega SST diverged twice in the 1 mm arm gaps starting from rest
            # (runs/_osc_diverged_*). Damping comes from the steady SST cases, not from these runs.
            setup.models.viscous.model = "laminar"
            setup.general.solver.time = "unsteady-2nd-order"
            ne = setup.named_expressions
            # velocity A*om*cos(om t) ramped in over the first RAMP seconds (a step start from rest
            # blew up in the 1 mm gap at l1 on the first try); build_dataset.py integrates the same law
            u = "m s^-1" if dof < 3 else "rad s^-1"     # Fluent requires rad for the rotation speed
            ne["osc_rate"] = {"definition": "%.8g [%s] * cos(%.8g [s^-1] * t) * min(t / %.8g [s], 1)"
                                            % (amp * om, u, om, RAMP)}
            mm = fz.mesh_motion
            mm.enable = True
            mm.moving_mesh_axis_origin = [val(c) for c in O]
            # Fluent 24.2 settings API: an expression is passed as a plain string value (probe_osc.py)
            if dof < 3:
                vel = [0.0, 0.0, 0.0]
                vel[dof] = "osc_rate"
                mm.moving_mesh_velocity = vel
            else:
                ax = [0.0, 0.0, 0.0]
                ax[dof - 3] = 1.0
                mm.moving_mesh_axis_direction = [val(c) for c in ax]
                mm.mgrid_omega = "osc_rate"
            return repr(mm.get_state())
        step("boundary_conditions_osc", bcs)

    step("write_case", lambda: solver.file.write(file_type="case", file_name=str(CAS)))
    t_solve = time.time()
    if KIND == "steady":
        step("hybrid_init", lambda: solver.solution.initialization.hybrid_initialize())
        chunk, max_it, win, tol = 25, 2500, 200, 0.005
        it = 0
        converged = False
        while it < max_it:
            step("iterate_%d" % (it + chunk), lambda: solver.solution.run_calculation.iterate(iter_count=chunk))
            it += chunk
            L = loads(solver)
            hist.append({"iter": it, "loads": L})
            save()
            # windowed means (win iterations each) of the last two windows: a load that settles into a
            # small periodic oscillation (wake shedding) converges in the mean instead of never passing
            nw = win // chunk
            if len(hist) < 2 * nw:
                continue
            mean = lambda hs: dict((k, sum(h["loads"][k] for h in hs) / len(hs)) for k in hs[0]["loads"])
            L, prev = mean(hist[-nw:]), mean(hist[-2 * nw:-nw])
            rep["loads_mean_last_window"] = L
            # drift over the window, each quantity scaled by the largest magnitude of its kind
            # (forces / moments about the origin / joint torques are scaled separately)
            kind = lambda k: "t" if k.startswith("tau") else k[0]
            scale = {}
            for k in L:
                scale[kind(k)] = max(scale.get(kind(k), 0.0), abs(L[k]))
            drifts = dict((k, abs(L[k] - prev[k]) / (scale[kind(k)] or 1e-9)) for k in L)
            drift = max(drifts.values())
            rep["drift_last"] = drift
            rep["drift_by_kind"] = dict((g, max(v for k, v in drifts.items() if kind(k) == g)) for g in scale)
            rep["iterations"] = it
            if drift < tol and it >= 400:
                converged = True
                break
        rep["converged"] = converged
    else:
        if KIND == "rot":
            # turn angle = wm*(T_end - ROT_RAMP/2); at most ROT_DANG rad per step at full rate
            # (dt = 0.03 s at 0.8 rad/s moved the arm tip 1.4 cm per step across the sub-mm cells of
            # the 1 mm arm gaps and blew up at step 3: runs/_rot_diverged_dt003)
            t_end = ROT_ANGLE / wm + ROT_RAMP / 2
            dt = ROT_DANG / wm
            nsteps = int(math.ceil(t_end / dt))
            rep["rot_angle_rad"], rep["rot_ramp_s"], rep["rot_dt"] = ROT_ANGLE, ROT_RAMP, dt
        else:
            dt = per / 80.0
            nsteps = 3 * 80
            rep["osc_ramp_s"], rep["osc_dt"] = RAMP, dt

        def tc():
            t = solver.solution.run_calculation.transient_controls
            t.time_step_size = dt
            t.max_iter_per_time_step = 30
            return repr(t.get_state())[:1000]
        step("transient_controls", tc)

        # Water at rest initially. The zero-velocity standard init leaves k, omega at defaults that
        # give nu_t/nu = 1e5 everywhere (first try diverged); seed a quiet free stream instead:
        # k from 1 % intensity at 0.1 m/s, omega for nu_t/nu = 10.
        k0 = 1.5 * (0.01 * 0.1) ** 2
        w0 = k0 / (10 * 1.05e-3 / 1025.0)

        def init_rest():
            ini = solver.solution.initialization
            ini.initialization_type = "standard"
            d = {"x-velocity": 0.0, "y-velocity": 0.0, "z-velocity": 0.0, "pressure": 0.0}
            if KIND != "osc":
                d.update({"k": k0, "omega": w0})
            ini.defaults = d
            ini.standard_initialize()
            return repr(ini.defaults.get_state())[:800]
        if KIND == "rot":
            # SST from a zero-velocity standard init blew up at step 3 (runs/_rot_diverged_stdinit);
            # the hybrid-initialised 0.35 rad trial ran 60 steps cleanly (runs/_rot_spinup_trial)
            step("hybrid_init", lambda: solver.solution.initialization.hybrid_initialize())
        else:
            step("standard_init_rest", init_rest)
        for k in range(nsteps):
            step("ts_%d" % (k + 1), lambda: solver.solution.run_calculation.dual_time_iterate(
                time_step_count=1, max_iter_per_step=30), required=True)
            rep["steps"].pop()  # keep the log short
            L = loads(solver)
            hist.append({"t": (k + 1) * dt, "loads": L})
            if any(not math.isfinite(v) or abs(v) > 1e6 for v in L.values()):
                rep["diverged_at_step"] = k + 1
                save()
                raise RuntimeError("non-finite / exploding loads at time step %d" % (k + 1))
            if (k + 1) % 10 == 0:
                save()
        rep["converged"] = True
        rep["time_steps"] = nsteps
    rep["solve_seconds"] = round(time.time() - t_solve, 1)
    rep["loads_final"] = hist[-1]["loads"] if hist else None
    if KIND == "steady" and hist:
        # mean over the last 200 iterations (convergence is judged on these window means)
        tail = [h for h in hist if h["iter"] > hist[-1]["iter"] - 200]
        rep["loads_mean_last_window"] = dict((k, sum(h["loads"][k] for h in tail) / len(tail))
                                             for k in tail[0]["loads"])
    step("write_data", lambda: solver.file.write(file_type="data", file_name=str(RUN / ("%s.dat.h5" % CID))),
         required=False)
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
rep["t_end"] = time.strftime("%Y-%m-%d %H:%M:%S")
rep["failed_required"] = [s["step"] for s in rep["steps"] if s.get("required") and not s.get("ok")]
rep["status"] = "ok" if not rep["failed_required"] and rep.get("converged") else "failed"
save()
print(json.dumps({k: rep.get(k) for k in ("status", "case", "iterations", "drift_last", "converged",
                                           "solve_seconds", "failed_required")}))

"""Probe settings-API shapes on base case b1 (no iterations, nothing written except the json).
Usage: <ansys python> fluent_probe.py TAG
"""
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

TAG = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\11_theta_fit")
OUT = D / ("%s_probe.json" % TAG)
if OUT.exists():
    raise SystemExit("refusing to overwrite")
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

rep = {}


def save():
    OUT.write_text(json.dumps(rep, ensure_ascii=False, indent=2, default=str), encoding="utf-8")


def probe(name, fn):
    try:
        r = fn()
        rep[name] = r if isinstance(r, (str, int, float, list, dict, bool, type(None))) else repr(r)[:6000]
    except Exception as exc:
        rep[name] = "ERR " + repr(exc)[:2000]
    save()


solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                                processor_count=4, additional_arguments="-t4 -mpi=msmpi",
                                ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                cleanup_on_exit=True, start_transcript=False)
try:
    solver.file.read_case(file_name=str(D / "b1.cas.h5"))
    setup = solver.setup
    bc = setup.boundary_conditions
    tui = solver.tui
    fz = setup.cell_zone_conditions.fluid["fff_water"]

    probe("frame_enable", lambda: setattr(fz.reference_frame, "frame_motion", True))
    probe("frame_state", lambda: repr(fz.reference_frame.get_state()))
    probe("frame_children", lambda: [n for n in fz.reference_frame.child_names])
    probe("frame_disable", lambda: setattr(fz.reference_frame, "frame_motion", False))
    probe("mesh_enable", lambda: setattr(fz.mesh_motion, "enable", True))
    probe("mesh_state", lambda: repr(fz.mesh_motion.get_state()))
    probe("mesh_children", lambda: [n for n in fz.mesh_motion.child_names])
    probe("mesh_expr_try", lambda: fz.mesh_motion.set_state(
        {"mesh_translational_velocity": [{"option": "value", "value": 0}, {"option": "value", "value": 0},
                                         {"option": "value", "value": 0}]}))
    probe("mesh_attrs_full", lambda: repr(fz.mesh_motion.get_state()))
    probe("mesh_disable", lambda: setattr(fz.mesh_motion, "enable", False))

    probe("to_vi", lambda: tui.define.boundary_conditions.zone_type("xmin", "velocity-inlet"))
    probe("vi_state", lambda: repr(bc.velocity_inlet["xmin"].get_state()))
    probe("vi_momentum_children", lambda: list(bc.velocity_inlet["xmin"].momentum.child_names))
    probe("vi_method_allowed", lambda: bc.velocity_inlet["xmin"].momentum.velocity_specification_method.allowed_values())
    probe("vi_set_components", lambda: setattr(bc.velocity_inlet["xmin"].momentum,
                                               "velocity_specification_method", "Components"))
    probe("vi_state_comp", lambda: repr(bc.velocity_inlet["xmin"].get_state()))
    probe("vi_frame_allowed", lambda: bc.velocity_inlet["xmin"].momentum.reference_frame.allowed_values())
    probe("to_po", lambda: tui.define.boundary_conditions.zone_type("xmax", "pressure-outlet"))
    probe("po_state", lambda: repr(bc.pressure_outlet["xmax"].get_state()))
    probe("wall_shear_allowed", lambda: bc.wall["ymin"].momentum.shear_condition.allowed_values())
    probe("methods", lambda: repr(solver.solution.methods.get_state())[:4000])
    probe("transient_controls", lambda: repr(solver.solution.run_calculation.transient_controls.get_state())[:2000])
    probe("general_time_allowed", lambda: setup.general.solver.time.allowed_values())
    probe("report_def_compute_force",
          lambda: repr(solver.solution.report_definitions.compute(report_defs=["fx"]))[:2000])
finally:
    solver.exit()
save()
print("done")

"""Probe the oscillation-case settings one by one on b5 (no solve, nothing written). Log: runs/probe_osc.json"""
import json
import os
import sys
import traceback
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

out = []


def t(name, fn):
    try:
        r = fn()
        out.append({"step": name, "ok": True, "result": repr(r)[:3000]})
    except Exception:
        out.append({"step": name, "ok": False, "error": traceback.format_exc()[-3000:]})
    (D / "runs" / "probe_osc.json").write_text(json.dumps(out, indent=1), encoding="utf-8")


def val(x):
    return {"option": "value", "value": float(x)}


s = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=4,
                           additional_arguments="-t4 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                           start_timeout=300, cleanup_on_exit=True)
try:
    s.file.read_case(file_name=str(D / "b5.cas.h5"))
    setup = s.setup
    fz = setup.cell_zone_conditions.fluid["fff_water"]
    t("time_unsteady", lambda: setattr(setup.general.solver, "time", "unsteady-2nd-order"))
    t("ne_create", lambda: setup.named_expressions.__setitem__(
        "osc_rate", {"definition": "0.0628 [m s^-1] * cos(3.1416 [s^-1] * t)"}))
    t("ne_state", lambda: setup.named_expressions["osc_rate"].get_state())
    mm = fz.mesh_motion
    t("mm_state0", lambda: mm.get_state())
    t("mm_enable", lambda: setattr(mm, "enable", True))
    t("mm_origin", lambda: setattr(mm, "moving_mesh_axis_origin", [val(0), val(0.29386), val(-0.0663)]))
    t("mm_vel_expr_list", lambda: setattr(mm, "moving_mesh_velocity",
                                          [{"option": "expression", "expression": "osc_rate"}, val(0), val(0)]))
    t("mm_state1", lambda: mm.get_state())
    t("mm_vel0_child", lambda: mm.moving_mesh_velocity[0].get_state())
    t("mm_vel_child_type", lambda: type(mm.moving_mesh_velocity).__mro__)
    t("mm_vel_set_child_value", lambda: mm.moving_mesh_velocity[0].set_state(
        {"option": "expression", "expression": "osc_rate"}))
    t("mm_vel_str_expr", lambda: setattr(mm, "moving_mesh_velocity", ["osc_rate", 0.0, 0.0]))
    t("mm_state2", lambda: mm.get_state())
    t("mm_omega_expr", lambda: setattr(mm, "mgrid_omega", {"option": "expression", "expression": "osc_rate"}))
    t("mm_omega_str", lambda: setattr(mm, "mgrid_omega", "osc_rate"))
    t("mm_state3", lambda: mm.get_state())
finally:
    s.exit()
print(json.dumps([(o["step"], o["ok"]) for o in out]))

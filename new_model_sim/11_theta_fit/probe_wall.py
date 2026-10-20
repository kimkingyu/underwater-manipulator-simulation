"""Probe wall-motion API (absolute stationary box walls around an MRF zone) on b5. 4 cores, no solve.
Log: runs/probe_wall.json"""
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
        out.append({"step": name, "ok": True, "result": repr(r)[:4000]})
    except Exception:
        out.append({"step": name, "ok": False, "error": traceback.format_exc()[-1500:]})
    (D / "runs" / "probe_wall.json").write_text(json.dumps(out, indent=1), encoding="utf-8")


s = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=4,
                           additional_arguments="-t4 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                           start_timeout=300, cleanup_on_exit=True)
try:
    s.file.read_case(file_name=str(D / "b5.cas.h5"))
    bc = s.setup.boundary_conditions
    fz = s.setup.cell_zone_conditions.fluid["fff_water"]
    t("zones", lambda: (list(bc.wall.keys()), list(bc.velocity_inlet.keys()), list(bc.pressure_outlet.keys())))
    z = "xmin"
    t("set_wall", lambda: bc.set_zone_type(zone_list=[z], new_type="wall"))
    t("wall_mom0", lambda: bc.wall[z].momentum.get_state())
    t("mrf_on", lambda: (setattr(fz.reference_frame, "frame_motion", True), fz.reference_frame.get_state()))
    t("wall_mom1", lambda: bc.wall[z].momentum.get_state())
    t("motion_moving", lambda: setattr(bc.wall[z].momentum, "wall_motion", "Moving Wall"))
    t("wall_mom2", lambda: bc.wall[z].momentum.get_state())
    t("allowed_motion", lambda: bc.wall[z].momentum.wall_motion.allowed_values())
    t("relative_false", lambda: setattr(bc.wall[z].momentum, "relative", False))
    t("rotating_true", lambda: setattr(bc.wall[z].momentum, "rotating", True))
    t("speed0", lambda: setattr(bc.wall[z].momentum, "rotation_speed", {"option": "value", "value": 0.0}))
    t("wall_mom3", lambda: bc.wall[z].momentum.get_state())
    t("body_wall_mom", lambda: bc.wall["wall_auv"].momentum.get_state())
finally:
    s.exit()
print(json.dumps([(o["step"], o["ok"]) for o in out]))

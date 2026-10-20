"""Probe standard-initialization defaults API on b5 (4 cores, no solve, nothing written). Log: runs/probe_init.json"""
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
        out.append({"step": name, "ok": False, "error": traceback.format_exc()[-2000:]})
    (D / "runs" / "probe_init.json").write_text(json.dumps(out, indent=1), encoding="utf-8")


s = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=4,
                           additional_arguments="-t4 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                           start_timeout=300, cleanup_on_exit=True)
try:
    s.file.read_case(file_name=str(D / "b5.cas.h5"))
    ini = s.solution.initialization
    t("defaults_state0", lambda: ini.defaults.get_state())
    t("init_type", lambda: setattr(ini, "initialization_type", "standard"))
    k0 = 1.5 * (0.01 * 0.1) ** 2
    w0 = k0 / (10 * 1.05e-3 / 1025.0)
    t("set_defaults", lambda: setattr(ini, "defaults", {"x-velocity": 0.0, "y-velocity": 0.0, "z-velocity": 0.0,
                                                        "pressure": 0.0, "k": k0, "omega": w0}))
    t("defaults_state1", lambda: ini.defaults.get_state())
    t("standard_initialize", lambda: ini.standard_initialize())
    t("nut_ratio_max", lambda: s.solution.report_definitions.volume.__setitem__(
        "nutr", {"report_type": "volume-max", "field": "viscosity-ratio", "cell_zones": ["fff_water"]}))
    t("nut_ratio_val", lambda: s.solution.report_definitions.compute(report_defs=["nutr"]))
finally:
    s.exit()
print(json.dumps([(o["step"], o["ok"]) for o in out]))

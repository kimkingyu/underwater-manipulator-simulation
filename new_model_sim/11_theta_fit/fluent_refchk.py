"""Read runs/T_px_08 case+data; report reference values and what fx / mx / tau_j1 actually return as
coefficient vs dimensional values. Read only; writes runs/T_px_08_refchk.json."""
import json
import os
import sys

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

os.environ.update(full_environment())
import ansys.fluent.core as pyfluent  # noqa: E402

out = {}


def tryit(name, fn):
    try:
        out[name] = fn()
    except Exception as exc:
        out[name] = "ERR " + repr(exc)[:600]


s = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=4,
                           additional_arguments="-t4 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                           start_timeout=300, cleanup_on_exit=True, start_transcript=False)
try:
    s.file.read_case_data(file_name=r"D:\work4\new_model_sim\11_theta_fit\runs\T_px_08.cas.h5")
    rd = s.solution.report_definitions
    tryit("ref", lambda: repr(s.setup.reference_values.get_state()))
    tryit("fx_children", lambda: list(rd.force["fx"].child_names))
    tryit("mx_children", lambda: list(rd.moment["mx"].child_names))
    tryit("mx_type_allowed", lambda: rd.moment["mx"].report_output_type.allowed_values())
    tryit("before", lambda: rd.compute(report_defs=["fx", "mx", "tau_j1"]))
    tryit("set_mx", lambda: setattr(rd.moment["mx"], "report_output_type", "Moment"))
    tryit("set_tau", lambda: setattr(rd.moment["tau_j1"], "report_output_type", "Moment"))
    tryit("after", lambda: rd.compute(report_defs=["fx", "mx", "tau_j1"]))
    # independent dimensional check: area-weighted wall force via the force report with all-1 ref values
    tryit("ref_set_1", lambda: (setattr(s.setup.reference_values, "area", 1.0),
                                setattr(s.setup.reference_values, "density", 2.0),
                                setattr(s.setup.reference_values, "velocity", 1.0),
                                setattr(s.setup.reference_values, "length", 1.0)) and None)
    tryit("ref_after_1", lambda: repr(s.setup.reference_values.get_state()))
    tryit("after_ref1", lambda: rd.compute(report_defs=["fx", "mx", "tau_j1"]))
finally:
    s.exit()
open(r"D:\work4\new_model_sim\11_theta_fit\runs\T_px_08_refchk.json", "w").write(json.dumps(out, indent=1, default=str))
print("ok")

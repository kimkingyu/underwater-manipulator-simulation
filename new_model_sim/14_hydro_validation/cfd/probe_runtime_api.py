"""Real Fluent API compatibility probe: geometry/reports only, no flow solution or loads."""
import argparse
import json
import math
from pathlib import Path

from prepare_mesh_base import Preparation, refuse_existing
from run_case import HERE, STAGE, MASS_FLOW_REPORT_TYPE, flatten_report, sha256

BASE = STAGE.parent / "11_theta_fit/b5.cas.h5"
OUT = HERE / "runtime_api_probe.json"
TRN = HERE / "runtime_api_probe.trn"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verification-reason", default="initial real API compatibility probe before blind CFD")
    args = parser.parse_args()
    refuse_existing((OUT, TRN))
    if not BASE.is_file():
        raise RuntimeError("missing q0 base case")
    initial_hash = sha256(BASE)
    p = Preparation(OUT, TRN)
    p.rep.update({"purpose": "actual 24.2 API checks before blind CFD; no solution/load computation",
                  "base_case": str(BASE), "base_sha256_before": initial_hash,
                  "flow_loads_computed": False, "case_written": False,
                  "allowed_compute_definitions": ["runtime_geom_" + axis for axis in "xyz"],
                  "probe_script_sha256": sha256(Path(__file__)), "verification_reason": args.verification_reason})
    previous = HERE / "runtime_api_probe_attempt1.json"
    if previous.exists():
        p.rep["previous_failed_attempt_retained"] = previous.name
        p.rep["repair_reason"] = "Actual Fluent 24.2 rejected mass-flow and reported allowed enum flux-massflow; use that verified enum in both runner and probe"
    previous_pass = HERE / "runtime_api_probe_pre_v3_pass.json"
    if previous_pass.exists():
        p.rep["previous_pass_retained"] = previous_pass.name
    try:
        p.start()
        solver = p.solver
        p.step("read_q0_uninitialized_case", lambda: solver.file.read_case(file_name=str(BASE)))
        setup, rd = solver.setup, solver.solution.report_definitions
        def sst():
            setup.models.viscous.model = "k-omega"
            setup.models.viscous.k_omega_model = "sst"
            state = {"model": setup.models.viscous.model(), "k_omega_model": setup.models.viscous.k_omega_model()}
            if state != {"model": "k-omega", "k_omega_model": "sst"}:
                raise RuntimeError("SST settings readback mismatch: " + str(state))
            p.rep["sst_settings"] = state
        p.step("set_and_read_SST", sst)
        def laminar():
            setup.models.viscous.model = "laminar"
            model = setup.models.viscous.model()
            if model != "laminar":
                raise RuntimeError("laminar settings readback mismatch")
            p.rep["laminar_model"] = model
        p.step("set_and_read_laminar", laminar)
        def mass_report():
            name = "runtime_mass_xmin"
            rd.flux[name] = {"report_type": MASS_FLOW_REPORT_TYPE, "boundaries": ["xmin"]}
            state = rd.flux[name].get_state()
            if state.get("report_type") != MASS_FLOW_REPORT_TYPE or state.get("boundaries") != ["xmin"]:
                raise RuntimeError("mass-flow definition readback mismatch: " + repr(state))
            p.rep["mass_flow_definition_state"] = state
            p.rep["mass_flow_computed"] = False
        p.step("mass_flow_report_fields_without_compute", mass_report)
        def geometry_reports():
            states = {}
            for axis in "xyz":
                name = "runtime_geom_" + axis
                rd.surface[name] = {"report_type": "surface-areaavg", "field": axis + "-coordinate", "surface_names": ["wall_auv"]}
                state = rd.surface[name].get_state()
                if state.get("report_type") != "surface-areaavg" or state.get("field") != axis + "-coordinate" or state.get("surface_names") != ["wall_auv"]:
                    raise RuntimeError("geometry report field readback mismatch: " + repr(state))
                states[name] = state
            p.rep["geometry_report_definition_states"] = states
        p.step("geometry_surface_report_fields", geometry_reports)
        def centroid():
            names = p.rep["allowed_compute_definitions"]
            values = flatten_report(rd.compute(report_defs=names))
            if set(values) != set(names) or any(not math.isfinite(v) for v in values.values()):
                raise RuntimeError("incomplete or nonfinite geometric centroid")
            p.rep["hull_surface_area_centroid_m"] = [values[name] for name in names]
            p.rep["computed_definitions"] = names
            p.rep["centroid_note"] = "geometry API test only; does not replace the original Ob"
        p.step("compute_mesh_geometric_centroid_only", centroid)
        p.step("restore_SST_in_memory_only", sst)
        if sha256(BASE) != initial_hash:
            raise RuntimeError("source q0 base changed unexpectedly")
        p.rep["base_source_unchanged"] = True
        p.rep["status"] = "api_compatible_no_initialization_no_solve"
    except Exception as exc:
        p.rep["status"], p.rep["error"] = "failed", repr(exc)
        raise
    finally:
        try:
            p.close()
        finally:
            p.save()
    print(json.dumps({k: p.rep.get(k) for k in ("status", "failed_required", "hull_surface_area_centroid_m", "flow_initialized", "flow_solved", "flow_loads_computed", "base_source_unchanged")}))


if __name__ == "__main__":
    main()

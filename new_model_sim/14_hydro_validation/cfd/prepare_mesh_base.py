"""qe mesh conversion / empty base only. No initialization, iterations, or flow data."""
import argparse
import json
import math
import os
import re
import sys
import time
from pathlib import Path

from run_case import BODY, DEFAULT_CONFIG, HERE, STAGE, load_config, posed_joints, sha256, write_json

QE = HERE / "qe"
TET = QE / "auvm_water_qe_files/dp0/FFF/MECH/FFF.msh"
POLY = QE / "qe_poly_e5.cas.h5"
POLY_REPORT = QE / "poly_qe_e5.json"
BASE = HERE / "bqe.cas.h5"


def poly_artifacts(variant):
    if variant == "qe_v3":
        return QE / "qe_poly_v3.cas.h5", QE / "poly_qe_v3.json", QE / "poly_qe_v3.trn"
    if variant in ("qe", "qe_v2"):
        return POLY, POLY_REPORT, QE / "poly_qe_e5.trn"
    raise RuntimeError("unknown poly artifact variant")


def refuse_existing(paths):
    for p in paths:
        if p.exists():
            raise RuntimeError("refusing to overwrite: " + str(p))


def mesh_inputs(variant="qe"):
    if variant not in ("qe", "qe_v2", "qe_v3"):
        raise RuntimeError("unknown mesh preparation variant")
    if variant == "qe_v3":
        from prepare_qe import verify_parent_quality_disposition
        verify_parent_quality_disposition()
    tet = QE / ("auvm_water_%s_files/dp0/FFF/MECH/FFF.msh" % variant)
    report_path = QE / "fluid_domain_qe.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    scdoc = QE / "fluid_domain_qe.scdoc"
    if report.get("status") != "ok" or not scdoc.is_file() or report.get("scdoc_sha256") != sha256(scdoc):
        raise RuntimeError("water geometry has no verified completed report")
    probe = json.loads((QE / "probe_pose_qe.json").read_text(encoding="utf-8"))
    if probe.get("passed") is not True or probe.get("status") != "ok":
        raise RuntimeError("exact STEP interference probe not passed")
    trace = (QE / ("wb_mesh_%s_trace.txt" % variant)).read_text(encoding="utf-8-sig")
    messages = (QE / ("mesh_%s_messages.txt" % variant)).read_text(encoding="utf-8-sig")
    if not tet.is_file() or tet.stat().st_size == 0 or "END" not in trace or any(x in trace for x in ("ERROR", "FATAL")):
        raise RuntimeError("tet mesh/Workbench trace not complete")
    if any(x in messages for x in ("FAILED", "INSIDE ERROR", "GenerateMesh raised", "[Error]")):
        raise RuntimeError("Meshing reported required failure")
    result = re.search(r"RESULT elements (\d+) nodes (\d+) skew max ([\d.eE+-]+) avg ([\d.eE+-]+)", messages)
    if not result or int(result.group(1)) <= 0:
        raise RuntimeError("no verified mesh element/quality record")
    return {"mesh_variant": variant, "tet_mesh": str(tet), "tet_sha256": sha256(tet), "elements": int(result.group(1)),
            "nodes": int(result.group(2)), "skew_max": float(result.group(3)), "skew_average": float(result.group(4))}


def launch():
    sys.path.insert(0, str(STAGE.parent / "04_workbench"))
    from run_wb_fullenv import full_environment
    os.environ.update(full_environment())
    import ansys.fluent.core as pyfluent
    return pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=1,
                                 ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                 cleanup_on_exit=True, start_transcript=False, cwd=str(QE))


class Preparation:
    def __init__(self, report_path, transcript_path):
        self.report_path, self.transcript_path = report_path, transcript_path
        self.rep = {"status": "started", "flow_initialized": False, "flow_solved": False, "processor_count": 1,
                    "steps": [], "failed_required": []}
        self.solver = None

    def save(self):
        write_json(self.report_path, self.rep)

    def step(self, name, action):
        row = {"name": name, "ok": False, "required": True}
        self.rep["steps"].append(row)
        start = time.monotonic()
        try:
            result = action()
            row["ok"] = True
            return result
        except Exception as exc:
            row["error"] = repr(exc)
            self.rep["failed_required"].append(name)
            raise
        finally:
            row["seconds"] = round(time.monotonic() - start, 3)
            self.save()

    def start(self):
        self.save()
        self.solver = self.step("launch_one_rank_prepare_only", launch)
        self.step("transcript_start", lambda: self.solver.transcript.start(str(self.transcript_path)))

    def close(self):
        if self.solver is not None:
            try:
                self.solver.transcript.stop()
            finally:
                self.solver.exit()
                self.solver = None

    def quality(self, label):
        path = self.transcript_path
        before = path.stat().st_size if path.exists() else 0
        self.solver.tui.mesh.quality()
        self.solver.tui.mesh.check()
        # Transcript delivery is asynchronous. Bounded wait only for this explicit output.
        text = ""
        for _ in range(10):
            text = path.read_bytes()[before:].decode("utf-8", errors="replace")
            if "Minimum Orthogonal Quality" in text and "Maximum Aspect Ratio" in text and re.search(r"Checking mesh[\s\S]*?Done\.", text):
                break
            time.sleep(.2)
        row = {"label": label}
        for key, pattern in (("min_orth", r"Minimum Orthogonal Quality =\s*([-+0-9.eE]+)"),
                             ("max_aspect", r"Maximum Aspect Ratio =\s*([-+0-9.eE]+)")):
            values = re.findall(pattern, text)
            if not values or not math.isfinite(float(values[-1])):
                raise RuntimeError("missing/nonfinite mesh quality metric: " + key)
            row[key] = float(values[-1])
        volumes = re.findall(r"minimum volume\s*\(m3\)\s*:\s*([-+0-9.eE]+)", text, re.I)
        row["minimum_volume_m3"] = float(volumes[-1]) if volumes else None
        row["mesh_check_complete"] = bool(re.search(r"Checking mesh[\s\S]*?Done\.", text))
        locations = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", text)
        row["min_orth_location_m"] = locations[-1].strip() if locations else None
        row["negative_volume"] = bool(re.search(r"negative\s+(?:cell\s+)?volumes?", text, re.I)) or (row["minimum_volume_m3"] is not None and row["minimum_volume_m3"] <= 0)
        row["errors"] = [line.strip() for line in text.splitlines() if re.search(r"\bError\b|\bfailed\b", line, re.I)]
        self.rep[label] = row
        self.save()
        if row["min_orth"] <= 0 or row["negative_volume"] or row["errors"] or not row["mesh_check_complete"] or row["minimum_volume_m3"] is None or not math.isfinite(row["minimum_volume_m3"]):
            raise RuntimeError("invalid/incomplete mesh quality/check: " + str(row))
        return row


def poly(variant="qe", repair_reason=None):
    inputs = mesh_inputs(variant)
    previous_path = QE / ("poly_qe_e5.json" if variant == "qe_v3" else "poly_qe.json")
    if variant == "qe_v3":
        if not repair_reason or not repair_reason.strip():
            raise RuntimeError("qe_v3 requires the explicit parent quality disposition reason")
    elif previous_path.exists():
        previous = json.loads(previous_path.read_text(encoding="utf-8"))
        if not repair_reason or not repair_reason.strip() or previous.get("status") != "failed" or previous.get("inputs", {}).get("tet_sha256") != inputs["tet_sha256"]:
            raise RuntimeError("one explicit reason and the unchanged failed input are required to complete e5 preprocessing")
    poly_case, out, trn = poly_artifacts(variant)
    refuse_existing((out, trn, poly_case))
    p = Preparation(out, trn)
    p.rep["inputs"] = inputs
    p.rep["repair_reason"] = repair_reason
    p.rep["previous_attempt_retained"] = previous_path.name if previous_path.exists() else None
    p.rep["workflow_reference"] = "12_coarse_mesh/fluent_poly.py: convert followed by exactly one mesh-only improve_quality call"
    p.rep["maximum_quality_improvement_calls"] = 1
    if variant == "qe_v3":
        p.rep["parent_quality_disposition"] = "cfd/parent_quality_disposition.json"
        p.rep["parent_quality_disposition_sha256"] = sha256(HERE / "parent_quality_disposition.json")
        p.rep["maximum_parent_authorized_alternative_attempts"] = 1
    try:
        p.start()
        solver = p.solver
        p.step("read_tet", lambda: solver.file.read_mesh(file_name=inputs["tet_mesh"]))
        p.step("tet_quality", lambda: p.quality("tet"))
        p.step("convert_polyhedra", solver.tui.mesh.polyhedra.convert_domain)
        p.step("poly_quality", lambda: p.quality("poly"))
        p.step("standard_e5_mesh_only_improve_quality_once", solver.tui.mesh.repair_improve.improve_quality)
        quality = p.step("poly_improved_quality", lambda: p.quality("poly_improved"))
        p.rep["minimum_orthogonal_quality_gate"] = 0.01
        def quality_gate():
            if quality["min_orth"] < .01:
                raise RuntimeError("poly quality below unchanged e5 preparation gate after one standard improvement; stop")
        p.step("final_poly_quality_gate", quality_gate)
        p.step("mesh_info", lambda: solver.tui.mesh.mesh_info(0))
        p.step("write_poly_case", lambda: solver.file.write(file_type="case", file_name=str(poly_case)))
        if not poly_case.is_file() or not poly_case.stat().st_size:
            raise RuntimeError("poly output missing/empty")
        for _ in range(10):
            info = trn.read_text(encoding="utf-8", errors="replace")
            counts = re.findall(r"(\d+)\s+polyhedra(?:l)?\s+cells", info)
            if counts:
                break
            time.sleep(.2)
        if not counts or int(counts[-1]) <= 0:
            raise RuntimeError("conversion has no verified positive polyhedral cell count")
        p.rep["polyhedral_cells"] = int(counts[-1])
        p.rep["mesh_info_lines"] = [line.strip() for line in info.splitlines() if re.search(r"\d+ (?:polyhedra|polygonal|nodes)", line)][-20:]
        p.rep["poly_case_sha256"] = sha256(poly_case)
        p.rep["status"] = "ready_uninitialized"
    except Exception as exc:
        p.rep["status"], p.rep["error"] = "failed", repr(exc)
        raise
    finally:
        p.close()
        p.save()


def base(variant="qe"):
    poly_case, report_path, _ = poly_artifacts(variant)
    poly_report = json.loads(report_path.read_text(encoding="utf-8"))
    if poly_report.get("status") != "ready_uninitialized" or poly_report.get("poly_case_sha256") != sha256(poly_case) or poly_report.get("failed_required") or poly_report.get("inputs", {}).get("mesh_variant") != variant or poly_report.get("poly_improved", {}).get("min_orth", 0) < 0.01:
        raise RuntimeError("poly case is not verified ready")
    if variant == "qe_v3":
        from prepare_qe import verify_parent_quality_disposition
        verify_parent_quality_disposition()
    out, trn = HERE / "bqe_base.json", HERE / "bqe_base.trn"
    refuse_existing((out, trn, BASE))
    cfg = load_config(DEFAULT_CONFIG)
    joints = posed_joints(cfg, cfg["poses"]["qe"]["q"])
    p = Preparation(out, trn)
    p.rep.update({"pose_id": "qe", "q": cfg["poses"]["qe"]["q"], "mesh_variant": variant, "mesh_sha256": sha256(poly_case),
                  "origin_m": cfg["origin_m"], "posed_joint_reports": joints,
                  "physical_policy_source": "11_theta_fit/fluent_base.py"})
    try:
        p.start()
        solver = p.solver
        p.step("read_poly", lambda: solver.file.read_case(file_name=str(poly_case)))
        bc, setup, rd = solver.setup.boundary_conditions, solver.setup, solver.solution.report_definitions
        p.step("separate_tank_wall", lambda: solver.tui.mesh.modify_zones.sep_face_zone_angle("tank_wall", 45, "yes"))
        def centroids_and_rename():
            candidates = [x for x in list(bc.wall.keys()) + list(bc.velocity_inlet.keys()) + list(bc.pressure_outlet.keys()) if x not in BODY]
            lo, hi = [-1.5605, -1.3296, -1.8008], [1.5601, 1.9136, 1.6404]
            mapping, centres = {}, {}
            from run_case import flatten_report
            for n, zone in enumerate(candidates):
                centre = []
                for axis in "xyz":
                    name = "prepare_centroid_%d_%s" % (n, axis)
                    rd.surface[name] = {"report_type": "surface-areaavg", "field": axis + "-coordinate", "surface_names": [zone]}
                    values = flatten_report(rd.compute(report_defs=[name]))
                    centre.append(values[name])
                centres[zone] = centre
                matches = [axis + side for i, axis in enumerate("xyz") for side, limits in (("min", lo), ("max", hi)) if abs(centre[i] - limits[i]) < .001]
                if len(matches) != 1:
                    raise RuntimeError("ambiguous outer face centroid: " + zone)
                mapping[zone] = matches[0]
            if sorted(mapping.values()) != sorted([a + side for a in "xyz" for side in ("min", "max")]):
                raise RuntimeError("not all six box faces identified")
            for old, new in mapping.items():
                if old != new:
                    solver.tui.define.boundary_conditions.zone_name(old, new)
            p.rep["box_centroids"], p.rep["box_mapping"] = centres, mapping
        p.step("identify_and_rename_box_faces", centroids_and_rename)
        def physics():
            for zone in (a + side for a in "xyz" for side in ("min", "max")):
                bc.set_zone_type(zone_list=[zone], new_type="wall")
                bc.wall[zone].momentum.shear_condition = "Specified Shear"
            fluids = list(setup.cell_zone_conditions.fluid.keys())
            if len(fluids) != 1 or set(BODY) - set(bc.wall.keys()):
                raise RuntimeError("fluid/body topology mismatch")
            setup.models.viscous.model = "k-omega"
            setup.models.viscous.k_omega_model = "sst"
            if "water-liquid" not in setup.materials.fluid.keys():
                setup.materials.database.copy_by_name(type="fluid", name="water-liquid")
            water = setup.materials.fluid["water-liquid"]
            water.density.value = 1025.
            water.viscosity.value = .00105
            setup.cell_zone_conditions.fluid[fluids[0]].material = "water-liquid"
            p.rep["fluid_zone"] = fluids[0]
            p.rep["material"] = {"density_kg_m3": water.density.value(), "dynamic_viscosity_Pa_s": water.viscosity.value()}
        p.step("material_models_and_boundaries", physics)
        def reports():
            for key, axis in (("fx", [1, 0, 0]), ("fy", [0, 1, 0]), ("fz", [0, 0, 1])):
                rd.force[key] = {"zones": BODY, "force_vector": axis, "per_zone": True}
            for key, axis in (("mx", [1, 0, 0]), ("my", [0, 1, 0]), ("mz", [0, 0, 1])):
                rd.moment[key] = {"zones": BODY, "mom_center": [0, 0, 0], "mom_axis": axis, "report_output_type": "Moment", "per_zone": True}
            for joint in joints:
                rd.moment[joint["name"]] = {"zones": joint["zones"], "mom_center": joint["centre_m"], "mom_axis": joint["axis"], "report_output_type": "Moment"}
            p.rep["report_states"] = {j["name"]: rd.moment[j["name"]].get_state() for j in joints}
        p.step("57_channel_reports_with_posed_joints", reports)
        p.step("base_mesh_quality", lambda: p.quality("base_quality"))
        p.step("write_uninitialized_base", lambda: solver.file.write(file_type="case", file_name=str(BASE)))
        if not BASE.is_file() or not BASE.stat().st_size:
            raise RuntimeError("base output missing/empty")
        p.rep["base_case_sha256"], p.rep["status"] = sha256(BASE), "ready_uninitialized"
    except Exception as exc:
        p.rep["status"], p.rep["error"] = "failed", repr(exc)
        raise
    finally:
        p.close()
        p.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=["poly", "base"])
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--mesh-variant", choices=["qe", "qe_v2", "qe_v3"], default="qe")
    parser.add_argument("--repair-reason")
    args = parser.parse_args()
    if args.check:
        tet = QE / ("auvm_water_%s_files/dp0/FFF/MECH/FFF.msh" % args.mesh_variant)
        poly_case, _, _ = poly_artifacts(args.mesh_variant)
        print(json.dumps({"solver_launched": False, "stage": args.stage, "mesh_variant": args.mesh_variant, "tet_exists": tet.is_file(),
                          "poly_exists": poly_case.is_file(), "base_exists": BASE.is_file()}))
        return
    if args.stage == "poly":
        poly(args.mesh_variant, args.repair_reason)
    else:
        base(args.mesh_variant)


if __name__ == "__main__":
    main()

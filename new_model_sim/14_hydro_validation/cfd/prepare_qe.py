"""Generate isolated qe CAD/mesh inputs; never initialize or solve a flow field."""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
QE = HERE / "qe"
Q = [0.225, -0.25, -0.20]


def write_new(path, text):
    if path.exists():
        raise RuntimeError("refusing to overwrite " + str(path))
    path.write_text(text, encoding="utf-8")


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError("template anchor not unique: " + old[:80])
    return text.replace(old, new)


def probe():
    QE.mkdir(exist_ok=True)
    src = ROOT / "13_pose_q1/sc_probe_pose_qc.py"
    s = src.read_text(encoding="utf-8")
    s = s.replace(str(ROOT / "13_pose_q1"), str(QE)).replace("probe_pose_qc.json", "probe_pose_qe.json")
    s = replace_once(s, "Q = [0.0, -0.80, 0.0]", "Q = " + repr(Q))
    s = s.replace("[0.45, -0.50, 0.40]", repr(Q)).replace("probe_pose_qc.json", "probe_pose_qe.json").replace("For every pair within 3 mm:", "For all 28 part pairs:")
    s = replace_once(s, "try:\n    save()\n    DocumentOpen.Execute(SOURCE)",
                     "import os\nif os.path.exists(OUT_JSON):\n    raise Exception('refusing to overwrite probe')\n\ntry:\n    save()\n    DocumentOpen.Execute(SOURCE)")
    s = replace_once(s, "        base[name_for(float(sh.Volume))] = sh", "        name = name_for(float(sh.Volume))\n        if name is None or name in base:\n            raise Exception('unrecognized or duplicate STEP part')\n        base[name] = sh")
    s = replace_once(s, "            if row.get('dist_mm', 0.0) > 3.0:\n                continue\n", "")
    s = replace_once(s, "    report['parts'] = sorted(base)", 
                     "    if sorted(base) != sorted(GROUP):\n        raise Exception('STEP part census differs')\n    report['parts'] = sorted(base)")
    s = replace_once(s, "    census(posed, 'q1')\n    report['status'] = 'ok'", """    census(posed, 'qe')
    failures = [r for rows in (report['q0'], report['qe']) for r in rows
                if 'dist_error' in r or not isinstance(r.get('intersect_mm3'), (int, float))]
    clashes = [r for r in report['qe'] if GROUP[r['a']] != GROUP[r['b']]]
    report['cross_group_intersections_mm3'] = clashes
    report['gate'] = {'max_cross_group_overlap_mm3': 1.0, 'kernel_tolerance_m': 1e-7,
                      'same_rigid_group_overlap_is_reference_only': True,
                      'method': 'exact B-rep closest separation and Boolean intersection for all 28 part pairs'}
    report['passed'] = not failures and all(r['intersect_mm3'] <= 1.0 for r in clashes)
    report['status'] = 'ok' if report['passed'] else 'rejected'
""")
    compile(s, "generated_exact_STEP_probe", "exec")
    write_new(QE / "sc_probe_pose_qe.py", s)
    print("generated isolated exact STEP probe")


def build():
    p = json.loads((QE / "probe_pose_qe.json").read_text(encoding="utf-8"))
    if p.get("status") != "ok" or p.get("passed") is not True or p.get("q") != Q:
        raise RuntimeError("qe exact CAD probe has not passed")
    src = ROOT / "13_pose_q1/make_build_pose.py"
    s = src.read_text(encoding="utf-8")
    s = replace_once(s, 'r"D = r\'D:\\work4\\new_model_sim\\13_pose_q1\'"',
                     repr("D = r'%s'" % str(QE)))
    s = replace_once(s, 'out = ROOT / "13_pose_q1" / ("sc_build_fluid_domain_%s.py" % TAG)',
                     'out = Path(%r) / ("sc_build_fluid_domain_%%s.py" %% TAG)' % str(QE))
    # Execute the unchanged generator logic only AFTER both write destinations were redirected.
    old = sys.argv
    sys.argv = [str(src), "qe", "0.225", "-0.25", "-0.20", "0"]
    try:
        exec(compile(s, "isolated_make_build_pose", "exec"), {"__name__": "__main__"})
    finally:
        sys.argv = old
    generated = QE / "sc_build_fluid_domain_qe.py"
    content = generated.read_text(encoding="utf-8")
    compile(content, "generated_qe_water_builder", "exec")
    if "D = r'%s'" % str(QE) not in content:
        raise RuntimeError("isolated builder output-root verification failed")
    record = {"pose": Q, "generator_source": str(src), "generator_sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
              "output_root": str(QE), "exact_probe": "probe_pose_qe.json", "flow_solved": False,
              "seal_strategy": {"q0": "v5 cylinder-gap fill retained", "qe": "same qb/qc v5 seal-before-pose policy; cylinder-gap fill dropped because joint2 opens it", "j3_wedge_and_jaw_seams": "retained before pose", "distal_shift_before_pose_m": [0, 1e-5, 0]},
              "local_fragment_repair": "not applied; only one bounded repair permitted after recorded failure"}
    write_new(QE / "generation_report.json", json.dumps(record, indent=2))


def mesh():
    report = json.loads((QE / "fluid_domain_qe.json").read_text(encoding="utf-8"))
    if report.get("status") != "ok" or not (QE / "fluid_domain_qe.scdoc").is_file():
        raise RuntimeError("qe water geometry not ready")
    s = (ROOT / "13_pose_q1/wb_mesh_qc.wbjn").read_text(encoding="utf-8")
    s = s.replace(str(ROOT / "13_pose_q1"), str(QE)).replace("_qc", "_qe").replace("Pose qc", "Pose qe").replace("pose qc", "pose qe")
    soi = [[1000 * x for x in c] for c in report["soi_centres_m"]]
    s, n = re.subn(r"SOI = \[.*?\]\nSIZES", "SOI = " + repr(soi) + "\nSIZES", s, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("SOI template anchor missing")
    # Missing gap faces is a geometry error, not a silent loss of near-joint resolution.
    s = replace_once(s, '    w("settings %%s" %% ([x for x in r if "FAILED" in x] or "all set"))',
                     '    w("settings %%s" %% ([x for x in r if "FAILED" in x] or "all set"))\n    if any("FAILED" in x for x in r):\n        raise Exception("required mesh settings failed")')
    write_new(QE / "wb_mesh_qe.wbjn", s)
    print("generated qe e5 journal with posed SOI centres")


def mesh_diagnostic():
    messages = (QE / "mesh_qe_messages.txt").read_text(encoding="utf-8-sig")
    if "[Error]" not in messages:
        raise RuntimeError("a recorded mesh failure is required before diagnosis")
    target = QE / "wb_mesh_qe_diag.wbjn"
    if target.exists():
        raise RuntimeError("one diagnostic generation only; previous evidence retained")
    source = ROOT / "13_pose_q1/make_mesh_diag.py"
    text = source.read_text(encoding="utf-8")
    text = replace_once(text, 'D = Path(r"D:\\work4\\new_model_sim\\13_pose_q1")', "D = Path(%r)" % str(QE))
    old = sys.argv
    sys.argv = [str(source), "qe"]
    try:
        exec(compile(text, "isolated_qe_mesh_diagnostic", "exec"), {"__name__": "__main__"})
    finally:
        sys.argv = old
    if not target.is_file():
        raise RuntimeError("isolated diagnostic journal was not generated")


def mesh_repair():
    diagnostic = QE / "mesh_qe_diag_messages.txt"
    text = diagnostic.read_text(encoding="utf-8-sig")
    found = re.findall(r"id (\d+) GeoFace ns (\w+) area_mm2 ([\d.eE+-]+) centroid_mm \[([^\]]+)\]", text)
    faces = {(int(i), name, float(area), tuple(float(x.strip()) for x in xyz.split(","))) for i, name, area, xyz in found}
    if "[Error]" not in text or len(faces) != 1:
        raise RuntimeError("bounded repair requires exactly one evidenced failed face")
    face_id, name, area, centre = next(iter(faces))
    if name != "wall_pin4" or not 0 < area < 10 or len(centre) != 3:
        raise RuntimeError("failure outside the allowed bounded pin4 sizing repair")
    target = QE / "wb_mesh_qe_v2.wbjn"
    plan_path = QE / "mesh_repair_plan.json"
    if target.exists() or plan_path.exists():
        raise RuntimeError("the single local mesh repair has already been prepared")
    source = (QE / "wb_mesh_qe.wbjn").read_text(encoding="utf-8")
    for old, new in (("auvm_water_qe.wbpj", "auvm_water_qe_v2.wbpj"), ("auvm_water_qe_files", "auvm_water_qe_v2_files"),
                     ("wb_mesh_qe_trace", "wb_mesh_qe_v2_trace"), ("mesh_qe_messages", "mesh_qe_v2_messages")):
        source = source.replace(old, new)
    anchor = '    if "gap_faces" in ns:\n        ns["gap_faces"].Delete()\n'
    refinement = '''    geo = ExtAPI.DataModel.GeoData
    FAILED_CENTRE_MM = __CENTRE__
    FAILED_AREA_MM2 = __AREA__
    local_faces, target_matches = [], []
    for fid in ns["wall_pin4"].Location.Ids:
        ent = geo.GeoEntityById(fid)
        c = [v * 1000 for v in ent.Centroid]
        area = ent.Area * 1e6
        dist = sum((c[k] - FAILED_CENTRE_MM[k]) ** 2 for k in range(3)) ** 0.5
        if dist < 0.1 and abs(area - FAILED_AREA_MM2) < 0.01:
            target_matches.append(fid)
        if dist < 1.0 and 0 < area < 10.0:
            local_faces.append(fid)
            w("repair pin4 face id %%d area_mm2 %%.6f dist_mm %%.6f" %% (fid, area, dist))
    if len(target_matches) != 1 or not 1 <= len(local_faces) <= 5:
        raise Exception("bounded repair must find one failed face and 1-5 local pin4 faces")
    ssel = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    ssel.Ids = local_faces
    size = mesh.AddSizing()
    size.Location = ssel
    size.Name = "size_pin4_evidenced_local_repair"
    r.append(size.Name + ":" + put(size, "ElementSize", Quantity("0.3 [mm]")))
    r.append(size.Name + ":" + put(size, "GrowthRate", 1.4))
'''.replace("__CENTRE__", repr(centre)).replace("__AREA__", repr(area))
    source = replace_once(source, anchor, anchor + refinement)
    plan = {"status": "prepared_single_bounded_sizing_repair", "reason": "observed surface mesh intersections at a diagnosed pin4 face",
            "diagnostic": diagnostic.name, "diagnostic_sha256": hashlib.sha256(diagnostic.read_bytes()).hexdigest(),
            "failed_face": {"id_in_diagnostic": face_id, "named_selection": name, "area_mm2": area, "centroid_mm": centre},
            "bounds": {"selection_named_selection": "wall_pin4", "centroid_radius_mm": 1.0, "face_area_max_mm2": 10.0,
                       "maximum_selected_faces": 5, "new_element_size_mm": 0.3, "growth_rate": 1.4},
            "geometry_changed": False, "physical_case_design_changed": False, "previous_project_retained": "auvm_water_qe.wbpj",
            "repair_project": "auvm_water_qe_v2.wbpj", "maximum_repair_attempts": 1}
    write_new(target, source)
    write_new(plan_path, json.dumps(plan, indent=2))
    print("generated one evidence-bounded local pin4 sizing repair", face_id, area, centre)


def verify_parent_quality_disposition():
    from run_case import safe_stage_path, sha256
    path = HERE / "parent_quality_disposition.json"
    plan = json.loads(path.read_text(encoding="utf-8"))
    required = {"status": "parent_approved_bounded_alternative_before_new_loads", "alternative_mesh_id": "qe_v3",
                "minimum_orthogonal_quality_gate": 0.01, "maximum_mesh_generate_calls": 1,
                "maximum_poly_conversion_calls": 1, "maximum_standard_quality_improvement_calls": 1,
                "automatic_retries": 0, "stop_if_gate_fails": True, "flow_initialization_allowed": False,
                "flow_iterations_allowed": False, "new_CFD_loads_allowed": False}
    if any(plan.get(k) != v for k, v in required.items()):
        raise RuntimeError("parent quality disposition differs from the bounded authorization")
    change = {"scope": "water body sphere of influence", "centre_m": [0.317916, 0.463444, 0.415393],
              "radius_m": 0.020, "target_element_size_m": 0.002}
    if plan.get("only_mesh_change") != change or (HERE / "runs").exists():
        raise RuntimeError("v3 geometry inputs changed or flow-run artifacts already exist")
    for item in plan["sources"]:
        source = safe_stage_path(HERE.parent, item["path"])
        if not source.is_file() or sha256(source) != item["sha256"]:
            raise RuntimeError("parent disposition source hash mismatch: " + item["path"])
    evidence = plan["evidence"]
    if sha256(safe_stage_path(HERE.parent, evidence["report"])) != evidence["report_sha256"]:
        raise RuntimeError("original failed-quality evidence changed")
    return plan


def mesh_v3():
    plan = verify_parent_quality_disposition()
    original = QE / "wb_mesh_qe_v2.wbjn"
    source = original.read_text(encoding="utf-8")
    for old, new in (("auvm_water_qe_v2.wbpj", "auvm_water_qe_v3.wbpj"), ("auvm_water_qe_v2_files", "auvm_water_qe_v3_files"),
                     ("wb_mesh_qe_v2_trace", "wb_mesh_qe_v3_trace"), ("mesh_qe_v2_messages", "mesh_qe_v3_messages")):
        source = source.replace(old, new)
    anchor = '    w("settings %%s" %% ([x for x in r if "FAILED" in x] or "all set"))'
    addition = '''    # Parent-approved sole alternative: same geometry/e5/pin4 controls, one local water-body SOI.
    cs_quality = m.CoordinateSystems.AddCoordinateSystem()
    cs_quality.Name = "cs_parent_quality_v3"
    cs_quality.OriginX = Quantity("317.916 [mm]")
    cs_quality.OriginY = Quantity("463.444 [mm]")
    cs_quality.OriginZ = Quantity("415.393 [mm]")
    size_quality = mesh.AddSizing()
    size_quality.Location = bsel
    size_quality.Type = SizingType.SphereOfInfluence
    size_quality.SphereCenter = cs_quality
    size_quality.SphereRadius = Quantity("20 [mm]")
    size_quality.ElementSize = Quantity("2 [mm]")
    size_quality.Name = "soi_parent_quality_v3"
    w("parent quality SOI centre_mm [317.916, 463.444, 415.393] radius_mm 20 size_mm 2 scope water_body")
'''
    source = replace_once(source, anchor, addition + anchor)
    target = QE / "wb_mesh_qe_v3.wbjn"
    write_new(target, source)
    record = {"mesh_variant": "qe_v3", "parent_disposition": "cfd/parent_quality_disposition.json",
              "parent_disposition_sha256": hashlib.sha256((HERE / "parent_quality_disposition.json").read_bytes()).hexdigest(),
              "source_journal_sha256": hashlib.sha256(original.read_bytes()).hexdigest(),
              "generated_journal_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
              "only_mesh_change": plan["only_mesh_change"], "geometry_changed": False,
              "pin4_v2_refinement_retained": True, "flow_initialized": False, "flow_solved": False}
    write_new(QE / "qe_v3_generation.json", json.dumps(record, indent=2))
    print("generated sole parent-approved qe_v3 alternative journal")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["probe", "build", "mesh", "mesh-diagnostic", "mesh-repair", "mesh-v3"])
    args = parser.parse_args()
    {"probe": probe, "build": build, "mesh": mesh, "mesh-diagnostic": mesh_diagnostic, "mesh-repair": mesh_repair, "mesh-v3": mesh_v3}[args.stage]()

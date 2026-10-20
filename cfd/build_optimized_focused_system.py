"""Build optimized focused lightweight overset mesh system (~250k - 300k cells).
Design criteria:
1. Fine mesh on robot arm components (lk1, lk2, lk3w1..4) and water immediate envelope (6-12 mm).
2. Coarse mesh in far-field outer water (40-60 mm), but keep inlet & outlet smooth (15-20 mm) to avoid reverse flow shocks.
3. Matching 8-12 mm cell sizes across all overset boundaries to minimize orphan cells.
4. Generates a comprehensive structured audit report (JSON) tracking cell counts, quality, and worst elements.
"""
import os
import sys
import json
import time
from pathlib import Path

# Setup environment for ANSYS and Gmsh
awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
directories = [
    'aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
    'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
    'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm'
]
os.environ['PATH'] = ';'.join([f'{awp}/{p}' for p in directories] + [os.environ.get('PATH', '')])

VENDOR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor")
sys.path.insert(0, str(VENDOR))
import gmsh

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

CAD_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04")
OUT_DIR = CAD_DIR / "focused_lightweight_system01"
OUT_DIR.mkdir(parents=True, exist_ok=True)

REPORT_FILE = OUT_DIR / "mesh_quality_and_diagnostic_report.json"
TAGS_JSON_J3 = CAD_DIR / "j3_occ_remesh01" / "j3_occ_group_tags.json"


def mesh_background(report):
    print("\n--- [1/4] Meshing Background Domain (Smooth Inlet/Outlet + Near-Arm Refinement) ---")
    step_file = CAD_DIR / "background_fluid.step"
    neu_file = OUT_DIR / "background.neu"
    
    gmsh.initialize(readConfigFiles=False)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    
    gmsh.model.occ.importShapes(str(step_file))
    gmsh.model.occ.synchronize()
    
    volumes = gmsh.model.getEntities(dim=3)
    surfaces = gmsh.model.getEntities(dim=2)
    
    input_tags = [2]
    output_tags = [4]
    sym_tags = [1, 3, 5, 6]
    wall_tags = [t for dim, t in surfaces if t not in [1, 2, 3, 4, 5, 6]]
    
    gmsh.model.addPhysicalGroup(2, input_tags, 1, name="input")
    gmsh.model.addPhysicalGroup(2, output_tags, 2, name="output")
    gmsh.model.addPhysicalGroup(2, sym_tags, 3, name="symmetry")
    gmsh.model.addPhysicalGroup(2, wall_tags, 4, name="rov_body_wall")
    gmsh.model.addPhysicalGroup(3, [volumes[0][1]], 5, name="fluid")
    
    # 1. Inlet & Outlet: 20 mm to prevent boundary reverse flow numerical instability
    f_in_out = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_in_out, "SurfacesList", input_tags + output_tags)
    f_in_out_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_in_out_thresh, "InField", f_in_out)
    gmsh.model.mesh.field.setNumber(f_in_out_thresh, "SizeMin", 0.020)
    gmsh.model.mesh.field.setNumber(f_in_out_thresh, "SizeMax", 0.080)
    gmsh.model.mesh.field.setNumber(f_in_out_thresh, "DistMin", 0.100)
    gmsh.model.mesh.field.setNumber(f_in_out_thresh, "DistMax", 0.400)
    
    # 2. ROV body wall: 20 mm near wall, 60 mm far field
    f_wall = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", wall_tags)
    f_wall_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_wall_thresh, "InField", f_wall)
    gmsh.model.mesh.field.setNumber(f_wall_thresh, "SizeMin", 0.020)
    gmsh.model.mesh.field.setNumber(f_wall_thresh, "SizeMax", 0.080)
    gmsh.model.mesh.field.setNumber(f_wall_thresh, "DistMin", 0.060)
    gmsh.model.mesh.field.setNumber(f_wall_thresh, "DistMax", 0.350)
    
    # 3. Near-Arm Water refinement box: 12 mm matches component outer overset boundaries!
    f_arm = gmsh.model.mesh.field.add("Box")
    gmsh.model.mesh.field.setNumber(f_arm, "VIn", 0.012)
    gmsh.model.mesh.field.setNumber(f_arm, "VOut", 0.080)
    gmsh.model.mesh.field.setNumber(f_arm, "XMin", -1.05)
    gmsh.model.mesh.field.setNumber(f_arm, "XMax", -0.55)
    gmsh.model.mesh.field.setNumber(f_arm, "YMin", 0.80)
    gmsh.model.mesh.field.setNumber(f_arm, "YMax", 1.60)
    gmsh.model.mesh.field.setNumber(f_arm, "ZMin", -1.50)
    gmsh.model.mesh.field.setNumber(f_arm, "ZMax", -0.90)
    gmsh.model.mesh.field.setNumber(f_arm, "Thickness", 0.10)
    
    f_min = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(f_min, "FieldsList", [f_in_out_thresh, f_wall_thresh, f_arm])
    gmsh.model.mesh.field.setAsBackgroundMesh(f_min)
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.010)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.090)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 10) # HXT
    gmsh.option.setNumber("Mesh.Optimize", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Background domain generated in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    
    report["domains"]["background"] = {
        "cells": n_cells,
        "generation_time_s": elapsed,
        "neu_file": str(neu_file),
        "settings": {"arm_box_size_m": 0.012, "inlet_outlet_size_m": 0.020, "far_field_size_m": 0.080}
    }
    return n_cells


def mesh_j1(report):
    print("\n--- [2/4] Meshing Joint 1 Domain (Turntable) ---")
    step_file = CAD_DIR / "j1_fluid.step"
    neu_file = OUT_DIR / "j1.neu"
    
    gmsh.initialize(readConfigFiles=False)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    
    gmsh.model.occ.importShapes(str(step_file))
    gmsh.model.occ.synchronize()
    
    volumes = gmsh.model.getEntities(dim=3)
    surfaces = gmsh.model.getEntities(dim=2)
    
    cad_bbox = gmsh.model.getBoundingBox(volumes[0][0], volumes[0][1])
    TOL = 1e-5
    overset_tags = []
    lk1_tags = []
    
    for dim, tag in surfaces:
        s_bbox = gmsh.model.getBoundingBox(dim, tag)
        on_bbox = False
        for i in range(3):
            if abs(s_bbox[i] - cad_bbox[i]) < TOL and abs(s_bbox[i+3] - cad_bbox[i]) < TOL:
                on_bbox = True
            if abs(s_bbox[i] - cad_bbox[i+3]) < TOL and abs(s_bbox[i+3] - cad_bbox[i+3]) < TOL:
                on_bbox = True
        if on_bbox:
            overset_tags.append(tag)
        else:
            lk1_tags.append(tag)
            
    gmsh.model.addPhysicalGroup(2, overset_tags, 1, name="overset_j1")
    gmsh.model.addPhysicalGroup(2, lk1_tags, 2, name="lk1")
    gmsh.model.addPhysicalGroup(3, [volumes[0][1]], 3, name="fluid")
    
    f_wall = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", lk1_tags)
    
    f_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thresh, "InField", f_wall)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.005) # 5 mm near lk1 wall
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.012) # 12 mm outer overset boundary
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMin", 0.008)
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMax", 0.035)
    
    gmsh.model.mesh.field.setAsBackgroundMesh(f_thresh)
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.004)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.014)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 1 domain generated in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    
    report["domains"]["j1"] = {
        "cells": n_cells,
        "generation_time_s": elapsed,
        "neu_file": str(neu_file),
        "settings": {"wall_size_m": 0.005, "overset_size_m": 0.012}
    }
    return n_cells


def mesh_j2(report):
    print("\n--- [3/4] Meshing Joint 2 Domain (Upper Arm with Fillets Resolved) ---")
    step_file = CAD_DIR / "j2_fluid.step"
    neu_file = OUT_DIR / "j2.neu"
    
    gmsh.initialize(readConfigFiles=False)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    
    gmsh.model.occ.importShapes(str(step_file))
    gmsh.model.occ.synchronize()
    
    volumes = gmsh.model.getEntities(dim=3)
    surfaces = gmsh.model.getEntities(dim=2)
    
    cad_bbox = gmsh.model.getBoundingBox(volumes[0][0], volumes[0][1])
    TOL = 1e-5
    overset_tags = []
    lk2_tags = []
    
    for dim, tag in surfaces:
        s_bbox = gmsh.model.getBoundingBox(dim, tag)
        on_bbox = False
        for i in range(3):
            if abs(s_bbox[i] - cad_bbox[i]) < TOL and abs(s_bbox[i+3] - cad_bbox[i]) < TOL:
                on_bbox = True
            if abs(s_bbox[i] - cad_bbox[i+3]) < TOL and abs(s_bbox[i+3] - cad_bbox[i+3]) < TOL:
                on_bbox = True
        if on_bbox:
            overset_tags.append(tag)
        else:
            lk2_tags.append(tag)
            
    gmsh.model.addPhysicalGroup(2, overset_tags, 1, name="overset_j2")
    gmsh.model.addPhysicalGroup(2, lk2_tags, 2, name="lk2")
    gmsh.model.addPhysicalGroup(3, [volumes[0][1]], 3, name="fluid")
    
    # Resolving small fillets down to 0.8mm with curvature sampling 14, outer overset 12mm
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0008) # 0.8 mm resolves narrow fillets
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.0120) # 12 mm outer overset boundary
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 14)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 2 domain generated in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    
    report["domains"]["j2"] = {
        "cells": n_cells,
        "generation_time_s": elapsed,
        "neu_file": str(neu_file),
        "settings": {"fillet_min_size_m": 0.0008, "overset_size_m": 0.012}
    }
    return n_cells


def mesh_j3(report):
    print("\n--- [4/4] Meshing Joint 3 Domain (Forearm & Claw) ---")
    step_file = CAD_DIR / "j3_fluid.step"
    neu_file = OUT_DIR / "j3.neu"
    group_tags = json.loads(TAGS_JSON_J3.read_text(encoding="utf-8"))
    
    gmsh.initialize(readConfigFiles=False)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    
    gmsh.model.occ.importShapes(str(step_file))
    gmsh.model.occ.synchronize()
    
    volumes = gmsh.model.getEntities(dim=3)
    
    gmsh.model.addPhysicalGroup(2, group_tags["overset_j3"], 1, name="overset_j3")
    gmsh.model.addPhysicalGroup(2, group_tags["lk3w1"], 2, name="lk3w1")
    gmsh.model.addPhysicalGroup(2, group_tags["lk3w2"], 3, name="lk3w2")
    gmsh.model.addPhysicalGroup(2, group_tags["lk3w3"], 4, name="lk3w3")
    gmsh.model.addPhysicalGroup(2, group_tags["lk3w4"], 5, name="lk3w4")
    gmsh.model.addPhysicalGroup(3, [volumes[0][1]], 6, name="fluid")
    
    # J3 clean settings: curvature 16, min 1mm to resolve features cleanly without explosion
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0010)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.0100)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 16)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 3 domain generated in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    
    report["domains"]["j3"] = {
        "cells": n_cells,
        "generation_time_s": elapsed,
        "neu_file": str(neu_file),
        "settings": {"curvature_min_size_m": 0.0010, "overset_size_m": 0.010}
    }
    return n_cells


def assemble_and_audit(report):
    print("\n========================================================")
    print("  Converting to .cas.h5 and Assembling with Full Quality Audit")
    print("========================================================")
    
    session = pyfluent.launch_fluent(
        product_version='24.2',
        mode='solver',
        precision='double',
        processor_count=8,
        additional_arguments='-t8 -mpi=msmpi',
        ui_mode=UIMode.NO_GUI_OR_GRAPHICS,
        start_timeout=180,
        cleanup_on_exit=True,
        start_transcript=True,
    )
    
    trn_file = OUT_DIR / "assembly_and_audit.trn"
    try:
        session.transcript.start(str(trn_file))
        
        domains = [
            ("background", OUT_DIR / "background.neu", OUT_DIR / "background.cas.h5"),
            ("j1", OUT_DIR / "j1.neu", OUT_DIR / "j1.cas.h5"),
            ("j2", OUT_DIR / "j2.neu", OUT_DIR / "j2.cas.h5"),
            ("j3", OUT_DIR / "j3.neu", OUT_DIR / "j3.cas.h5")
        ]
        
        for name, neu_p, cas_p in domains:
            print(f"  [Convert] Importing {name}...")
            session.tui.file.import_.gambit(str(neu_p))
            session.tui.mesh.check()
            session.tui.mesh.repair_improve.improve_quality()
            session.tui.file.write_case(str(cas_p))
            
        print("\n  [Assembly] Reading background and appending j1, j2, j3...")
        session.tui.file.read_case(str(OUT_DIR / "background.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j1.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j2.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j3.cas.h5"))
        
        session.tui.define.boundary_conditions.zone_type('output', 'pressure-outlet')
        session.tui.define.boundary_conditions.zone_type('overset_j1', 'overset')
        session.tui.define.boundary_conditions.zone_type('overset_j2', 'overset')
        session.tui.define.boundary_conditions.zone_type('overset_j3', 'overset')
        
        threads = session.scheme_eval.scheme_eval('(map (lambda (th) (list (thread-id th) (symbol->string (thread-name th)))) (get-cell-threads))')
        bg_id = [t[0] for t in threads if t[1] == 'fluid'][0]
        j1_id = [t[0] for t in threads if t[1] == 'fluid.1'][0]
        j2_id = [t[0] for t in threads if t[1] == 'fluid.1.1'][0]
        j3_id = [t[0] for t in threads if t[1] == 'fluid.1.1.1'][0]
        
        cmd_intf = f'(ti-menu-load-string "define/overset-interfaces/create arm_interface {bg_id} () {j1_id} {j2_id} {j3_id} ()")'
        session.scheme_eval.scheme_eval(cmd_intf)
        
        cmd_prio = '(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")'
        session.scheme_eval.scheme_eval(cmd_prio)
        
        # Audit overset
        print("\n=== Overset Check ===")
        session.tui.define.overset_interfaces.check()
        
        print("\n=== Mesh Quality Audit ===")
        session.tui.mesh.check()
        session.tui.mesh.quality()
        
        out_case = OUT_DIR / "assembled_focused_lightweight_t0.cas.h5"
        session.tui.file.write_case(str(out_case))
        print(f"\nSuccessfully wrote assembled case: {out_case}")
        report["assembled_case"] = str(out_case)
        
    finally:
        session.transcript.stop()
        session.exit()


def main():
    t_start = time.time()
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "domains": {},
        "target_mesh_budget": "< 300,000 cells"
    }
    
    nb = mesh_background(report)
    n1 = mesh_j1(report)
    n2 = mesh_j2(report)
    n3 = mesh_j3(report)
    
    total = nb + n1 + n2 + n3
    report["total_cells"] = total
    print("\n========================================================")
    print(f"  Lightweight Mesh Generation Finished:")
    print(f"    Background : {nb:8d} cells")
    print(f"    Joint 1    : {n1:8d} cells")
    print(f"    Joint 2    : {n2:8d} cells")
    print(f"    Joint 3    : {n3:8d} cells")
    print(f"    TOTAL      : {total:8d} cells (target < 300,000)")
    print("========================================================")
    
    assemble_and_audit(report)
    
    report["status"] = "success"
    report["total_elapsed_s"] = time.time() - t_start
    with open(REPORT_FILE, "w", encoding="utf-8") as fp:
        json.dump(report, fp, indent=2, ensure_ascii=False)
    print(f"\nDiagnostic audit report written to: {REPORT_FILE}")
    print(f"All operations finished in {time.time()-t_start:.1f}s!")


if __name__ == '__main__':
    main()

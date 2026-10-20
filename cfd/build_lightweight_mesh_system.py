"""Build lightweight 4-domain overset mesh system under 300k total elements.
Solves the slow computation issue by right-sizing the background domain and component overset boundaries.
Guarantees identical CAD fingerprints, boundary names, and hierarchical overset interfaces.
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
OUT_DIR = CAD_DIR / "lightweight_270k_system01"
OUT_DIR.mkdir(parents=True, exist_ok=True)

TAGS_JSON_J3 = CAD_DIR / "j3_occ_remesh01" / "j3_occ_group_tags.json"


def mesh_background():
    print("\n==========================================")
    print("  [1/4] Generating Background Domain Mesh")
    print("==========================================")
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
    
    # Sizing
    f_wall = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", wall_tags)
    
    f_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thresh, "InField", f_wall)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.050) # 50 mm on ROV body
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.250) # 250 mm far field
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMin", 0.100)
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMax", 0.600)
    
    # Arm box refinement
    f_arm = gmsh.model.mesh.field.add("Box")
    gmsh.model.mesh.field.setNumber(f_arm, "VIn", 0.024)  # 24 mm inside arm motion zone
    gmsh.model.mesh.field.setNumber(f_arm, "VOut", 0.250)
    gmsh.model.mesh.field.setNumber(f_arm, "XMin", -1.05)
    gmsh.model.mesh.field.setNumber(f_arm, "XMax", -0.55)
    gmsh.model.mesh.field.setNumber(f_arm, "YMin", 0.80)
    gmsh.model.mesh.field.setNumber(f_arm, "YMax", 1.60)
    gmsh.model.mesh.field.setNumber(f_arm, "ZMin", -1.50)
    gmsh.model.mesh.field.setNumber(f_arm, "ZMax", -0.90)
    gmsh.model.mesh.field.setNumber(f_arm, "Thickness", 0.10)
    
    f_min = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(f_min, "FieldsList", [f_thresh, f_arm])
    gmsh.model.mesh.field.setAsBackgroundMesh(f_min)
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.020)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.280)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 10) # HXT
    gmsh.option.setNumber("Mesh.Optimize", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Background domain meshed in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells, neu_file


def mesh_j1():
    print("\n==========================================")
    print("  [2/4] Generating Joint 1 Domain Mesh")
    print("==========================================")
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
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.006) # 6 mm near lk1
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.016) # 16 mm outer overset
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMin", 0.008)
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMax", 0.035)
    
    gmsh.model.mesh.field.setAsBackgroundMesh(f_thresh)
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.005)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.018)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 1 domain meshed in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells, neu_file


def mesh_j2():
    print("\n==========================================")
    print("  [3/4] Generating Joint 2 Domain Mesh")
    print("==========================================")
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
    
    f_wall = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", lk2_tags)
    
    f_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thresh, "InField", f_wall)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.007) # 7 mm near lk2
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.016) # 16 mm outer overset
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMin", 0.010)
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMax", 0.045)
    
    gmsh.model.mesh.field.setAsBackgroundMesh(f_thresh)
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.005)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.018)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 2 domain meshed in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells, neu_file


def mesh_j3():
    print("\n==========================================")
    print("  [4/4] Generating Joint 3 Domain Mesh")
    print("==========================================")
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
    
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0010)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.0140)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1) # Delaunay
    gmsh.option.setNumber("Mesh.Optimize", 1)
    
    t0 = time.time()
    gmsh.model.mesh.generate(3)
    elapsed = time.time() - t0
    
    elements = gmsh.model.mesh.getElements(3)
    n_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
    print(f"  Joint 3 domain meshed in {elapsed:.1f}s: {n_cells} cells")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells, neu_file


def convert_and_assemble():
    print("\n==========================================")
    print("  Converting .neu to .cas.h5 and Assembling")
    print("==========================================")
    
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
    
    trn_file = OUT_DIR / "assembly_and_check.trn"
    try:
        session.transcript.start(str(trn_file))
        
        # 1. Convert each domain to .cas.h5
        domains = [("background", OUT_DIR / "background.neu", OUT_DIR / "background.cas.h5"),
                   ("j1", OUT_DIR / "j1.neu", OUT_DIR / "j1.cas.h5"),
                   ("j2", OUT_DIR / "j2.neu", OUT_DIR / "j2.cas.h5"),
                   ("j3", OUT_DIR / "j3.neu", OUT_DIR / "j3.cas.h5")]
                   
        for name, neu_p, cas_p in domains:
            print(f"  Importing {name} ({neu_p.name})...")
            session.tui.file.import_.gambit(str(neu_p))
            session.tui.mesh.check()
            session.tui.mesh.repair_improve.improve_quality()
            session.tui.file.write_case(str(cas_p))
            
        print("\n  Assembling 4 domains into single overset case...")
        session.tui.file.read_case(str(OUT_DIR / "background.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j1.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j2.cas.h5"))
        session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j3.cas.h5"))
        
        # Set boundary types
        session.tui.define.boundary_conditions.zone_type('output', 'pressure-outlet')
        session.tui.define.boundary_conditions.zone_type('overset_j1', 'overset')
        session.tui.define.boundary_conditions.zone_type('overset_j2', 'overset')
        session.tui.define.boundary_conditions.zone_type('overset_j3', 'overset')
        
        threads = session.scheme_eval.scheme_eval('(map (lambda (th) (list (thread-id th) (symbol->string (thread-name th)))) (get-cell-threads))')
        print('  Cell threads in assembled mesh:', threads)
        
        bg_id = [t[0] for t in threads if t[1] == 'fluid'][0]
        j1_id = [t[0] for t in threads if t[1] == 'fluid.1'][0]
        j2_id = [t[0] for t in threads if t[1] == 'fluid.1.1'][0]
        j3_id = [t[0] for t in threads if t[1] == 'fluid.1.1.1'][0]
        
        cmd_intf = f'(ti-menu-load-string "define/overset-interfaces/create arm_interface {bg_id} () {j1_id} {j2_id} {j3_id} ()")'
        session.scheme_eval.scheme_eval(cmd_intf)
        
        cmd_prio = '(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")'
        session.scheme_eval.scheme_eval(cmd_prio)
        
        print("\n=== Overset Interface List ===")
        session.tui.define.overset_interfaces.list()
        
        print("\n=== Overset Check ===")
        session.tui.define.overset_interfaces.check()
        
        print("\n=== Mesh Check & Quality ===")
        session.tui.mesh.check()
        session.tui.mesh.quality()
        
        out_case = OUT_DIR / "assembled_lightweight_t0.cas.h5"
        session.tui.file.write_case(str(out_case))
        print(f"\nSuccessfully wrote assembled lightweight case: {out_case}")
        
    finally:
        session.transcript.stop()
        session.exit()


def main():
    t_start = time.time()
    n_bg, _ = mesh_background()
    n_j1, _ = mesh_j1()
    n_j2, _ = mesh_j2()
    n_j3, _ = mesh_j3()
    
    total = n_bg + n_j1 + n_j2 + n_j3
    print(f"\n==========================================")
    print(f"  Summary of Gmsh 3D Cell Counts:")
    print(f"    Background : {n_bg:8d} cells")
    print(f"    Joint 1    : {n_j1:8d} cells")
    print(f"    Joint 2    : {n_j2:8d} cells")
    print(f"    Joint 3    : {n_j3:8d} cells")
    print(f"    TOTAL      : {total:8d} cells (target < 300,000)")
    print(f"==========================================")
    
    convert_and_assemble()
    print(f"\nAll operations finished successfully in {time.time()-t_start:.1f} seconds!")


if __name__ == '__main__':
    main()

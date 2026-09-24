import os
import sys
import json
import time
from pathlib import Path

VENDOR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor")
sys.path.insert(0, str(VENDOR))
import gmsh

CAD_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04")
OUT_DIR = CAD_DIR / "lightweight_270k_system01"
OUT_DIR.mkdir(parents=True, exist_ok=True)
TAGS_JSON_J3 = CAD_DIR / "j3_occ_remesh01" / "j3_occ_group_tags.json"


def mesh_background():
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
    
    f_wall = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", wall_tags)
    
    f_thresh = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thresh, "InField", f_wall)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.050) # 50 mm on ROV body
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.250) # 250 mm far field
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMin", 0.100)
    gmsh.model.mesh.field.setNumber(f_thresh, "DistMax", 0.600)
    
    # Arm box refinement (24 mm inside)
    f_arm = gmsh.model.mesh.field.add("Box")
    gmsh.model.mesh.field.setNumber(f_arm, "VIn", 0.024)
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
    print(f"  Background domain: {n_cells} cells ({elapsed:.1f}s)")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells


def mesh_j1():
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
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.006)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.016)
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
    print(f"  Joint 1 domain   : {n_cells} cells ({elapsed:.1f}s)")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells


def mesh_j2():
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
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMin", 0.007)
    gmsh.model.mesh.field.setNumber(f_thresh, "SizeMax", 0.016)
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
    print(f"  Joint 2 domain   : {n_cells} cells ({elapsed:.1f}s)")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells


def mesh_j3():
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
    print(f"  Joint 3 domain   : {n_cells} cells ({elapsed:.1f}s)")
    
    gmsh.option.setNumber("Mesh.Format", 49)
    gmsh.write(str(neu_file))
    gmsh.finalize()
    return n_cells


if __name__ == '__main__':
    print("Generating lightweight 4-domain Gmsh meshes...")
    t0 = time.time()
    nb = mesh_background()
    n1 = mesh_j1()
    n2 = mesh_j2()
    n3 = mesh_j3()
    total = nb + n1 + n2 + n3
    print(f"\nFinished all 4 domains in {time.time()-t0:.1f}s!")
    print(f"Total elements = {total} (Target < 300,000)")

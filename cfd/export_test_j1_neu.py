import sys
from pathlib import Path

VENDOR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor")
sys.path.insert(0, str(VENDOR))
import gmsh

cad_step = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_fluid.step")
out_neu = Path(r"D:\work4\cfd\test_j1.neu")

gmsh.initialize(readConfigFiles=False)
gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "M")

gmsh.model.occ.importShapes(str(cad_step))
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

wall_tags = lk1_tags
f_wall = gmsh.model.mesh.field.add("Distance")
gmsh.model.mesh.field.setNumbers(f_wall, "SurfacesList", wall_tags)

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

gmsh.model.mesh.generate(3)

gmsh.option.setNumber("Mesh.Format", 49) # Gambit Neutral
gmsh.write(str(out_neu))
gmsh.finalize()

print("Generated test_j1.neu successfully!")

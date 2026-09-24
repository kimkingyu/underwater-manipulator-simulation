import sys
import json
import time
from pathlib import Path

VENDOR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor")
sys.path.insert(0, str(VENDOR))
import gmsh

cad_step = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j3_fluid.step")
tags_json = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j3_occ_remesh01\j3_occ_group_tags.json")
group_tags = json.loads(tags_json.read_text(encoding="utf-8"))

gmsh.initialize(readConfigFiles=False)
gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "M")

gmsh.model.occ.importShapes(str(cad_step))
gmsh.model.occ.synchronize()

to_defeature = [328, 329, 330, 659, 661, 662, 331, 332, 333, 334, 655, 656, 657, 658]
for s in to_defeature:
    try:
        gmsh.model.occ.defeature([1], [s], removeVolume=False)
    except:
        pass
gmsh.model.occ.synchronize()

volumes = gmsh.model.getEntities(dim=3)
all_surfs = set(tag for dim, tag in gmsh.model.getEntities(dim=2))

g_overset = [t for t in group_tags["overset_j3"] if t in all_surfs]
g_w1 = [t for t in group_tags["lk3w1"] if t in all_surfs]
g_w2 = [t for t in group_tags["lk3w2"] if t in all_surfs]
g_w3 = [t for t in group_tags["lk3w3"] if t in all_surfs]
g_w4 = [t for t in group_tags["lk3w4"] if t in all_surfs]

assigned = set(g_overset + g_w1 + g_w2 + g_w3 + g_w4)
unassigned = all_surfs - assigned
if unassigned:
    g_w4.extend(list(unassigned))

gmsh.model.addPhysicalGroup(2, g_overset, 1, name="overset_j3")
gmsh.model.addPhysicalGroup(2, g_w1, 2, name="lk3w1")
gmsh.model.addPhysicalGroup(2, g_w2, 3, name="lk3w2")
gmsh.model.addPhysicalGroup(2, g_w3, 4, name="lk3w3")
gmsh.model.addPhysicalGroup(2, g_w4, 5, name="lk3w4")
gmsh.model.addPhysicalGroup(3, [volumes[0][1]], 6, name="fluid")

# Clean mesh settings
gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0010)
gmsh.option.setNumber("Mesh.MeshSizeMax", 0.0100)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 16)
gmsh.option.setNumber("Mesh.Algorithm", 6)
gmsh.option.setNumber("Mesh.Algorithm3D", 1) # Delaunay
gmsh.option.setNumber("Mesh.Optimize", 1)
gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)

t0 = time.time()
gmsh.model.mesh.generate(3)
elapsed = time.time() - t0

elements = gmsh.model.mesh.getElements(3)
total_cells = len(elements[1][0]) if len(elements[1]) > 0 else 0
print(f"J3 defeatured mesh: total 3D cells = {total_cells} in {elapsed:.1f}s")

neu_file = Path(r"D:\work4\cfd\test_j3_defeatured.neu")
gmsh.option.setNumber("Mesh.Format", 49)
gmsh.write(str(neu_file))
gmsh.finalize()
print(f"Written {neu_file} successfully.")

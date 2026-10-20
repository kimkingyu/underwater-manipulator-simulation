import sys
VENDOR = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor"
sys.path.insert(0, VENDOR)
import gmsh

CAD_DIR = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04"

gmsh.initialize(readConfigFiles=False)
gmsh.option.setString("Geometry.OCCTargetUnit", "M")

# The authorative 4.STEP contains the arm solids!
step_arm = r"D:\work4\机器人\L\4.STEP"
gmsh.model.occ.importShapes(step_arm)
gmsh.model.occ.synchronize()

vols = gmsh.model.getEntities(dim=3)
print(f"Total solids in 4.STEP: {len(vols)}")
for dim, tag in vols:
    mass = gmsh.model.occ.getMass(dim, tag)
    com = gmsh.model.occ.getCenterOfMass(dim, tag)
    bbox = gmsh.model.getBoundingBox(dim, tag)
    print(f"Solid {tag:2d}: volume = {mass:.6e} m3 ({mass*1e3:.3f} L), COM = [{com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f}]")

gmsh.finalize()

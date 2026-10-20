import sys
VENDOR = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor"
sys.path.insert(0, VENDOR)
import gmsh

cad_step = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j3_fluid.step"

gmsh.initialize(readConfigFiles=False)
gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "M")
gmsh.model.occ.importShapes(cad_step)
gmsh.model.occ.synchronize()

print("Distance between Sphere 85 and nearby faces:")
for f in [84, 86, 87, 88, 89, 90, 91, 92, 93]:
    d = gmsh.model.occ.getDistance(2, 85, 2, f)
    print(f"  Dist(Sphere 85, Face {f}) = {d[0]*1000:.4f} mm")

gmsh.finalize()

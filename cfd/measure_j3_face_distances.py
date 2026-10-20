import sys
from pathlib import Path

VENDOR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j1_occ_remesh01\vendor")
sys.path.insert(0, str(VENDOR))
import gmsh

cad_step = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\j3_fluid.step")

gmsh.initialize(readConfigFiles=False)
gmsh.option.setNumber("General.Terminal", 0)
gmsh.option.setString("Geometry.OCCTargetUnit", "M")

gmsh.model.occ.importShapes(str(cad_step))
gmsh.model.occ.synchronize()

# Check distance between faces around (-0.748391, 1.23832, -1.25050)
faces = [14, 29, 85, 91, 112, 257, 328, 329, 338, 341, 548, 662]
for f1 in [328, 329]:
    for f2 in faces:
        if f1 != f2:
            d = gmsh.model.occ.getDistance(2, f1, 2, f2)
            if d[0] < 0.001:
                print(f"Dist(Face {f1}, Face {f2}) = {d[0]*1000:.4f} mm")

gmsh.finalize()

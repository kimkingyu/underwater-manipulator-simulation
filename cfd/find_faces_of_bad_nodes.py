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

nodes = {
    661: (-0.74563601, 1.23832240, -1.26574012),
    367: (-0.74763414, 1.23832084, -1.26574012),
    40: (-0.74743223, 1.23832074, -1.26574012),
    19924: (-0.74672313, 1.23832155, -1.26477652)
}

surfaces = gmsh.model.getEntities(dim=2)
for nid, pt in nodes.items():
    print(f"\nSurfaces touching Node {nid} {pt}:")
    for dim, tag in surfaces:
        bbox = gmsh.model.getBoundingBox(dim, tag)
        if (bbox[0] - 1e-4 <= pt[0] <= bbox[3] + 1e-4 and
            bbox[1] - 1e-4 <= pt[1] <= bbox[4] + 1e-4 and
            bbox[2] - 1e-4 <= pt[2] <= bbox[5] + 1e-4):
            typ = gmsh.model.getType(dim, tag)
            area = gmsh.model.occ.getMass(dim, tag)
            print(f"  Face {tag:3d}: {typ:<12} area={area:.4e}")

gmsh.finalize()

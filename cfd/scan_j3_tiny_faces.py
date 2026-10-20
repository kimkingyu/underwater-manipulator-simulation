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

surfaces = gmsh.model.getEntities(dim=2)
tiny_faces = []
for dim, tag in surfaces:
    bbox = gmsh.model.getBoundingBox(dim, tag)
    dx = bbox[3] - bbox[0]
    dy = bbox[4] - bbox[1]
    dz = bbox[5] - bbox[2]
    area = gmsh.model.occ.getMass(dim, tag)
    dims = sorted([dx, dy, dz])
    # dims are in mm or meters? Let's check bbox
    # If bbox coords are ~ -750, then coords are in mm!
    if dims[0] < 0.05 or area < 0.01: # min dimension < 50 microns or area < 0.01 mm^2
        typ = gmsh.model.getType(dim, tag)
        tiny_faces.append((tag, typ, area, dims))

print(f"Total faces: {len(surfaces)}")
print(f"Tiny sliver faces (< 0.05mm min dimension or area < 0.01 mm2): {len(tiny_faces)}")
for tag, typ, area, dims in tiny_faces[:25]:
    print(f"  Face {tag:3d}: type={typ:<15} area={area:.4e} mm2, dims=[{dims[0]:.4e}, {dims[1]:.4e}, {dims[2]:.4e}] mm")

gmsh.finalize()

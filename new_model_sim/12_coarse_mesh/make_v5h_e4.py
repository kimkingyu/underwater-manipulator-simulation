"""wb_mesh_v5h_e4.wbjn = wb_mesh_v5h_e3.wbjn with finer arm (3.5 mm) and jaw/pin (2 mm) face sizes;
everything else (real hull, soft 40 mm hull sizing, growth 1.4, gap 1 mm, J3 spheres 0.4 mm) unchanged.
Target ~300k polyhedra after Fluent conversion."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")
s = (M / "wb_mesh_v5h_e3.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_mesh_v5h_e3_trace", "wb_mesh_v5h_e4_trace"), ("auvm_water_v5h_e3.wbpj", "auvm_water_v5h_e4.wbpj"),
             ("mesh_v5h_e3_messages.txt", "mesh_v5h_e4_messages.txt"),
             ("auvm_water_v5h_e3_files", "auvm_water_v5h_e4_files"), ('"V5H_E3_hull_soft40"', '"V5H_E4_arm3.5_jaw2"')):
    assert a in s, a
    s = s.replace(a, b)
jaw = 'r.append(n + ":" + put(c, "ElementSize", Quantity("3 [mm]")))'
arm = 'r.append(n + ":" + put(c, "ElementSize", Quantity("5 [mm]")))'
assert s.count(jaw) == 1 and s.count(arm) == 1
s = s.replace(jaw, jaw.replace("3 [mm]", "2 [mm]")).replace(arm, arm.replace("5 [mm]", "3.5 [mm]"))
(M / "wb_mesh_v5h_e4.wbjn").write_text(s, encoding="utf-8")
print("ok")

"""wb_mesh_v5h_e5.wbjn = e4 with jaw/pin faces 1.5 mm (arm 3.5 mm unchanged)."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")
s = (M / "wb_mesh_v5h_e4.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_mesh_v5h_e4_trace", "wb_mesh_v5h_e5_trace"), ("auvm_water_v5h_e4.wbpj", "auvm_water_v5h_e5.wbpj"),
             ("mesh_v5h_e4_messages.txt", "mesh_v5h_e5_messages.txt"),
             ("auvm_water_v5h_e4_files", "auvm_water_v5h_e5_files"),
             ('"V5H_E4_arm3.5_jaw2"', '"V5H_E5_arm3.5_jaw1.5"'), ('Quantity("2 [mm]")', 'Quantity("1.5 [mm]")')):
    assert a in s, a
    s = s.replace(a, b)
(M / "wb_mesh_v5h_e5.wbjn").write_text(s, encoding="utf-8")
print("ok")

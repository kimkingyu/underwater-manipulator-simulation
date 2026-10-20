"""v13 gap-face script and meshing journal, derived from the verified v12 ones; the three jaw sizings
become one wall_jaw sizing (3 mm)."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")

g = (M / "sc_gap_faces_v12.py").read_text(encoding="utf-8")
for a, b in (("fluid_domain_v12.scdoc", "fluid_domain_v13.scdoc"), ("fluid_domain_v12g.scdoc", "fluid_domain_v13g.scdoc"),
             ("gap_faces_v12.json", "gap_faces_v13.json")):
    assert a in g, a
    g = g.replace(a, b)
(M / "sc_gap_faces_v13.py").write_text(g, encoding="utf-8")

s = (M / "wb_mesh_v12.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_mesh_v12_trace", "wb_mesh_v13_trace"), ("auvm_water_v12.wbpj", "auvm_water_v13.wbpj"),
             ("fluid_domain_v12g.scdoc", "fluid_domain_v13g.scdoc"), ("mesh_v12_messages", "mesh_v13_messages"),
             ('"mesh_water_v12"', '"mesh_water_v13"')):
    assert a in s, a
    s = s.replace(a, b)
old = '''SIZES = [("wall_jaw1", "3 [mm]"), ("wall_jaw2", "3 [mm]"), ("wall_pin4", "3 [mm]"),'''
assert old in s
s = s.replace(old, '''SIZES = [("wall_jaw", "3 [mm]"),''')
(M / "wb_mesh_v13.wbjn").write_text(s, encoding="utf-8")
print("ok")

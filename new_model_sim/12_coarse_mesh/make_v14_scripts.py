"""v14 gap-face script and meshing journal from the v13 ones (only file names change)."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")
g = (M / "sc_gap_faces_v13.py").read_text(encoding="utf-8")
for a, b in (("fluid_domain_v13.scdoc", "fluid_domain_v14.scdoc"), ("fluid_domain_v13g.scdoc", "fluid_domain_v14g.scdoc"),
             ("gap_faces_v13.json", "gap_faces_v14.json")):
    assert a in g, a
    g = g.replace(a, b)
(M / "sc_gap_faces_v14.py").write_text(g, encoding="utf-8")
s = (M / "wb_mesh_v13.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_mesh_v13_trace", "wb_mesh_v14_trace"), ("auvm_water_v13.wbpj", "auvm_water_v14.wbpj"),
             ("fluid_domain_v13g.scdoc", "fluid_domain_v14g.scdoc"), ("mesh_v13_messages", "mesh_v14_messages"),
             ('"mesh_water_v13"', '"mesh_water_v14"')):
    assert a in s, a
    s = s.replace(a, b)
(M / "wb_mesh_v14.wbjn").write_text(s, encoding="utf-8")
print("ok")

"""Derive the v12 gap-face script and the v12 meshing journal from the verified v11/E3 ones.
v12 mesh = wb_mesh_v11.wbjn geometry pipeline + E3 sizes (growth 1.4, arm 5, jaw 3, gap 1 mm, J3 spheres
0.4 mm r 4 mm, curvature off, min 0.3, max 200, ES 150) + gap sizing scoped by face ids and the helper
named selection deleted before the msh export (the v5g fix)."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")

g = (M / "sc_gap_faces_v11.py").read_text(encoding="utf-8")
for a, b in (("fluid_domain_v11.scdoc", "fluid_domain_v12.scdoc"), ("fluid_domain_v11g.scdoc", "fluid_domain_v12g.scdoc"),
             ("gap_faces_v11.json", "gap_faces_v12.json")):
    assert a in g, a
    g = g.replace(a, b)
(M / "sc_gap_faces_v12.py").write_text(g, encoding="utf-8")

s = (M / "wb_mesh_v11.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_mesh_v11_trace", "wb_mesh_v12_trace"), ("auvm_water_v11.wbpj", "auvm_water_v12.wbpj"),
             ("fluid_domain_v11g.scdoc", "fluid_domain_v12g.scdoc"), ("mesh_v11_messages", "mesh_v12_messages"),
             ('"mesh_water_v11"', '"mesh_water_v12"')):
    assert a in s, a
    s = s.replace(a, b)

old_sizes = '''SIZES = [("wall_jaw1", "1 [mm]"), ("wall_jaw2", "1 [mm]"), ("wall_pin4", "1 [mm]"),
         ("wall_link3", "2 [mm]"), ("wall_l2", "2 [mm]"), ("wall_l1", "2 [mm]"),
         ("gap_faces", "0.3 [mm]")]'''
assert old_sizes in s
s = s.replace(old_sizes, '''SIZES = [("wall_jaw1", "3 [mm]"), ("wall_jaw2", "3 [mm]"), ("wall_pin4", "3 [mm]"),
         ("wall_link3", "5 [mm]"), ("wall_l2", "5 [mm]"), ("wall_l1", "5 [mm]"),
         ("gap_faces", "1 [mm]")]''')

assert 'sz.ElementSize = Quantity("0.2 [mm]")' in s
s = s.replace('sz.ElementSize = Quantity("0.2 [mm]")', 'sz.ElementSize = Quantity("0.4 [mm]")')

old_glob = '''    put("MaximumSize", Quantity("300 [mm]"))
    put("ElementSize", Quantity("150 [mm]"))'''
assert old_glob in s
s = s.replace(old_glob, '''    put("MaximumSize", Quantity("200 [mm]"))
    put("ElementSize", Quantity("150 [mm]"))
    put("CaptureCurvature", False)
    put("MinimumSize", Quantity("0.3 [mm]"))
    put("GrowthRate", 1.4)
    put("GrowthRateSF", 1.4)
    for c in mesh.Children:
        if c.GetType().Name == "Sizing":
            c.GrowthRate = 1.4
    # v5g fix: scope the gap sizing by face ids and drop the overlapping helper named selection,
    # otherwise the Fluent mesh write fails.
    gsz = [c for c in mesh.Children if c.GetType().Name == "Sizing" and c.Name == "size_gap_faces"][0]
    gsel = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    gsel.Ids = list(ns["gap_faces"].Location.Ids)
    out.write("gap face ids %%s\\n" %% list(gsel.Ids))
    gsz.Location = gsel
    ns["gap_faces"].Delete()''')
(M / "wb_mesh_v12.wbjn").write_text(s, encoding="utf-8")
print("ok")

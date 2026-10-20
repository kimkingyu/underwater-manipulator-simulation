"""wb_mesh_v11.wbjn = wb_mesh_v5g.wbjn retargeted to fluid_domain_v11g.scdoc, plus the v5h J3 spheres of
influence and global ElementSize 150 / MaximumSize 300 mm, plus a skewness dump. Arm sizings unchanged."""
from pathlib import Path

ROOT = Path("D:/work4/new_model_sim")
s = (ROOT / "09_seal_j2j3" / "wb_mesh_v5g.wbjn").read_text(encoding="utf-8")
M = r"D:\work4\new_model_sim\12_coarse_mesh"
reps = [(r'D = r"D:\work4\new_model_sim\09_seal_j2j3"', 'D = r"%s"' % M),
        ("wb_mesh_v5g_trace", "wb_mesh_v11_trace"), ("auvm_water_v5g.wbpj", "auvm_water_v11.wbpj"),
        ("fluid_domain_v5g.scdoc", "fluid_domain_v11g.scdoc"), ("mesh_v5g_messages", "mesh_v11_messages"),
        ('"mesh_water_v5g"', '"mesh_water_v11"')]
for a, b in reps:
    assert a in s, a
    s = s.replace(a, b)

old = "    n_before = len(list(ExtAPI.Application.Messages))"
new = '''    ZS = [-344.0, -351.0, -358.0, -365.0, -372.0, -379.0, -386.0, -392.0]
    body = m.Geometry.GetChildren(DataModelObjectCategory.Body, True)[0]
    bsel = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    bsel.Ids = [body.GetGeoBody().Id]
    for i, z in enumerate(ZS):
        cs = m.CoordinateSystems.AddCoordinateSystem()
        cs.Name = "soi_j3gap_%%d" %% i
        cs.OriginX = Quantity("0.483 [mm]")
        cs.OriginY = Quantity("-122.603 [mm]")
        cs.OriginZ = Quantity("%%s [mm]" %% z)
        sz = mesh.AddSizing()
        sz.Location = bsel
        sz.Type = SizingType.SphereOfInfluence
        sz.SphereCenter = cs
        sz.SphereRadius = Quantity("4 [mm]")
        sz.ElementSize = Quantity("0.2 [mm]")
        sz.Name = "soi_j3gap_%%d" %% i
    put("MaximumSize", Quantity("300 [mm]"))
    put("ElementSize", Quantity("150 [mm]"))
''' + old
assert old in s
s = s.replace(old, new, 1)

old2 = '    out.write("mesh_elements %%s\\n" %% mesh.Elements)'
new2 = old2 + '''
    try:
        mesh.MeshMetric = MeshMetricType.Skewness
        out.write("skewness min %%s max %%s avg %%s\\n" %% (mesh.Minimum, mesh.Maximum, mesh.Average))
    except Exception, e:
        out.write("metric error %%s\\n" %% e)'''
assert old2 in s, "mesh_elements line"
s = s.replace(old2, new2, 1)
(ROOT / "12_coarse_mesh" / "wb_mesh_v11.wbjn").write_text(s, encoding="utf-8")
print("ok")

from pathlib import Path

ROOT = Path("D:/work4/new_model_sim")
s = (ROOT / "09_seal_j2j3" / "wb_mesh_v5g_export.wbjn").read_text(encoding="utf-8")
reps = [(r'D = r"D:\work4\new_model_sim\09_seal_j2j3"', r'D = r"D:\work4\new_model_sim\12_coarse_mesh"'),
        ("wb_mesh_v5g_export_trace", "wb_export_v11e3b_trace"), ("auvm_water_v5g.wbpj", "auvm_water_v11e3.wbpj"),
        ("mesh_v5g_export_messages", "mesh_v11e3b_messages"), ('"1 open v5g"', '"1 open v11e3"')]
for a, b in reps:
    assert a in s, a
    s = s.replace(a, b)
a = '    out.write("mesh_elements %%s\\n" %% mesh.Elements)'
assert a in s
s = s.replace(a, a + '''
    try:
        mesh.MeshMetric = MeshMetricType.Skewness
        out.write("skewness max %%s avg %%s\\n" %% (mesh.Maximum, mesh.Average))
    except Exception, e:
        out.write("metric err %%s\\n" %% e)''')
(ROOT / "12_coarse_mesh" / "wb_export_v11e3b.wbjn").write_text(s, encoding="utf-8")
print("ok")

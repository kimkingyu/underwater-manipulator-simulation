"""wb_mesh_qd_v2.wbjn = wb_mesh_qd.wbjn + a 0.3 mm face sizing on the pin4 sliver faces around the one that failed in qd
(wb_mesh_qd_diag: id 523, wall_pin4, 0.741 mm2, centroid (-9.996, -305.151, -445.609) mm). The face is
found by centroid (< 1 mm) and area (< 2 mm2) among the wall_pin4 faces. New project auvm_water_qd_v2."""
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\13_pose_q1")
s = (D / "wb_mesh_qd.wbjn").read_text(encoding="utf-8")


def rep(a, b, n=1):
    global s
    assert s.count(a) == n, (a, s.count(a))
    s = s.replace(a, b)


rep("auvm_water_qd.wbpj", "auvm_water_qd_v2.wbpj", 2)   # comment line 2 + PROJECT
rep("auvm_water_qd_files", "auvm_water_qd_v2_files")
rep("wb_mesh_qd_trace", "wb_mesh_qd_v2_trace")
rep("mesh_qd_messages", "mesh_qd_v2_messages")
rep('''    if "gap_faces" in ns:
        ns["gap_faces"].Delete()
''', '''    if "gap_faces" in ns:
        ns["gap_faces"].Delete()
    geo = ExtAPI.DataModel.GeoData
    SLIVER_C = (-9.996, -305.151, -445.609)
    sl = []
    for i in ns["wall_pin4"].Location.Ids:
        ent = geo.GeoEntityById(i)
        c = [v * 1000 for v in ent.Centroid]
        dd = sum((c[k] - SLIVER_C[k]) ** 2 for k in range(3)) ** 0.5
        if dd < 1.0 and ent.Area * 1e6 < 2.0:
            sl.append(i)
            w("sliver face id %%d area_mm2 %%.4f dist_mm %%.4f" %% (i, ent.Area * 1e6, dd))
    if not 1 <= len(sl) <= 5:
        raise Exception("expected 1-5 pin4 sliver faces near the failed one, found %%d" %% len(sl))
    ssel = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    ssel.Ids = sl
    ssz = mesh.AddSizing()
    ssz.Location = ssel
    ssz.Name = "size_pin4_sliver"
    r.append(ssz.Name + ":" + put(ssz, "ElementSize", Quantity("0.3 [mm]")))
    r.append(ssz.Name + ":" + put(ssz, "GrowthRate", 1.4))
''')
out = D / "wb_mesh_qd_v2.wbjn"
assert not out.exists()
out.write_text(s, encoding="utf-8")
print("wrote", out)

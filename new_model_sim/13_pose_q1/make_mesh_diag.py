"""wb_mesh_TAG_diag.wbjn = wb_mesh_TAG.wbjn into a separate project, logging the failing geometry entities
(id, type, named selection, area, centroid) of every error message; no FFF.msh export.
Usage: python make_mesh_diag.py TAG"""
import sys
from pathlib import Path

T = sys.argv[1]
D = Path(r"D:\work4\new_model_sim\13_pose_q1")
s = (D / ("wb_mesh_%s.wbjn" % T)).read_text(encoding="utf-8")


def rep(a, b):
    global s
    assert s.count(a) >= 1, a
    s = s.replace(a, b)


rep("auvm_water_%s.wbpj" % T, "auvm_water_%s_diag.wbpj" % T)
rep("auvm_water_%s_files" % T, "auvm_water_%s_diag_files" % T)
rep("wb_mesh_%s_trace" % T, "wb_mesh_%s_diag_trace" % T)
rep("mesh_%s_messages" % T, "mesh_%s_diag_messages" % T)
old = '''    for msg in ExtAPI.Application.Messages:
        if msg.Severity.ToString() != "Warning":
            w("  [%%s] %%s" %% (msg.Severity, msg.DisplayString))'''
new = '''    geo = ExtAPI.DataModel.GeoData
    face_ns = {}
    for x in m.NamedSelections.Children:
        for i in x.Location.Ids:
            face_ns[i] = x.Name
    for i in gsel.Ids:
        face_ns[i] = "gap_faces"
    for msg in ExtAPI.Application.Messages:
        if msg.Severity.ToString() != "Warning":
            w("  [%%s] %%s" %% (msg.Severity, msg.DisplayString))
            try:
                ids = list(msg.Location.Ids) if msg.Location is not None else []
            except Exception, e:
                ids = []
                w("     loc err %%s" %% e)
            for i in ids[:40]:
                ent = geo.GeoEntityById(i)
                try:
                    area = ent.Area * 1e6
                except Exception:
                    area = -1.0
                w("     id %%d %%s ns %%s area_mm2 %%.4f centroid_mm %%s" %% (i, ent.Type, face_ns.get(i, "-"), area,
                                                                         [round(v * 1000, 3) for v in ent.Centroid]))'''
rep(old, new)
cut_a = '    step("5 mkdir MECH"'
cut_b = '    step("7 save"'
i, j = s.index(cut_a), s.index(cut_b)
s = s[:i] + s[j:]
assert "update mesh component" not in s and "mkdir MECH" not in s
out = D / ("wb_mesh_%s_diag.wbjn" % T)
out.write_text(s, encoding="utf-8")
print("wrote", out)

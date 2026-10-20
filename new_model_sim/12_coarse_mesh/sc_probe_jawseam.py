# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: in fluid_domain_v13.scdoc, list water faces and edges within 3 mm of the worst cell
# P = (-10.3, -151.2, -536.8) mm: type, area/length, bbox, distance; plus face-pair dihedral hints.
import codecs
import json
import traceback

SCDOC = r'D:\work4\new_model_sim\12_coarse_mesh\fluid_domain_v13.scdoc'
OUT = r'D:\work4\new_model_sim\12_coarse_mesh\jawseam_probe.json'
P = (-0.0103, -0.1512, -0.5368)
R = 0.003
rep = {'status': 'started', 'point_m': P, 'radius_m': R, 'faces': [], 'edges': []}


def save():
    s = codecs.open(OUT, 'w', 'utf-8')
    s.write(json.dumps(rep, indent=2))
    s.close()


def bb(x):
    b = x.GetBoundingBox(Matrix.Identity)
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z, b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def near(b):
    d2 = 0.0
    for k in range(3):
        v = P[k]
        if v < b[k]:
            d2 += (b[k] - v) ** 2
        elif v > b[k + 3]:
            d2 += (v - b[k + 3]) ** 2
    return d2 ** 0.5


try:
    DocumentOpen.Execute(SCDOC)
    water = [b for b in GetRootPart().GetAllBodies()][0]
    shape = water.Shape
    for i, f in enumerate(shape.Faces):
        b = bb(f)
        d = near(b)
        if d <= R:
            row = {'i': i, 'kind': f.Geometry.GetType().Name, 'area_mm2': round(float(f.Area) * 1e6, 5),
                   'bbox_mm': [round(v * 1000, 3) for v in b], 'bbox_dist_mm': round(d * 1000, 3)}
            try:
                g = f.Geometry
                if row['kind'] == 'Plane':
                    n = g.Frame.DirZ
                    row['normal'] = [round(n.X, 4), round(n.Y, 4), round(n.Z, 4)]
                elif row['kind'] == 'Cylinder':
                    row['radius_mm'] = round(float(g.Radius) * 1000, 4)
                    a = g.Axis.Direction
                    row['axis'] = [round(a.X, 4), round(a.Y, 4), round(a.Z, 4)]
            except Exception:
                pass
            rep['faces'].append(row)
    for i, e in enumerate(shape.Edges):
        b = bb(e)
        d = near(b)
        if d <= R:
            rep['edges'].append({'i': i, 'kind': e.Geometry.GetType().Name if e.Geometry else None,
                                 'len_mm': round(float(e.Length) * 1000, 4),
                                 'bbox_mm': [round(v * 1000, 3) for v in b]})
    rep['faces'].sort(key=lambda r: r['area_mm2'])
    rep['edges'].sort(key=lambda r: r['len_mm'])
    rep['status'] = 'ok'
except Exception:
    rep['status'] = 'error'
    rep['traceback'] = traceback.format_exc()
finally:
    save()

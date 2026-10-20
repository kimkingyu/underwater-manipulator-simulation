# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: list water faces in fluid_domain_v6.scdoc (and v5 for comparison) within R of a point
# reported by Meshing (face 623 could not be meshed), with group, type, area, bbox, edge lengths.
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point

DOCS = {'v6': r'D:\work4\new_model_sim\09_seal_j2j3\fluid_domain_v6.scdoc',
        'v5': r'D:\work4\new_model_sim\09_seal_j2j3\fluid_domain_v5.scdoc'}
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\probe_face_v6.json'
P = (0.01905, -0.155635, -0.361932)
R = 4e-3
report = {'status': 'started', 'point_m': P, 'radius_m': R}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def save():
    stream = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        stream.close()


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    p = Point.Create(*P)
    for key, path in DOCS.items():
        DocumentOpen.Execute(path)
        root = GetRootPart()
        face_group = {}
        for g in root.Groups:
            for x in g.Members:
                if hasattr(x, 'Shape') and x.Shape.GetType().Name == 'Face':
                    face_group[x.Shape.GetHashCode()] = u(g.Name)
        water = list(root.GetAllBodies())[0]
        rows = []
        for idx, df in enumerate(list(water.Faces)):
            f = df.Shape
            b = f.GetBoundingBox(Matrix.Identity)
            lo = (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)
            hi = (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)
            if not all(lo[k] - R <= P[k] <= hi[k] + R for k in range(3)):
                continue
            hit = f.ProjectPoint(p)
            q = getattr(hit, 'Point', hit)
            d = ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5
            if d > R:
                continue
            rows.append({'face_index': idx, 'group': face_group.get(f.GetHashCode(), '?'),
                         'type': f.Geometry.GetType().Name, 'dist_mm': round(d * 1e3, 4),
                         'area_mm2': round(float(f.Area) * 1e6, 4),
                         'lo_mm': [round(v * 1e3, 3) for v in lo], 'hi_mm': [round(v * 1e3, 3) for v in hi],
                         'edges_mm': sorted(round(float(e.Length) * 1e3, 4) for e in f.Edges)})
        report[key] = sorted(rows, key=lambda r: r['dist_mm'])
        save()
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

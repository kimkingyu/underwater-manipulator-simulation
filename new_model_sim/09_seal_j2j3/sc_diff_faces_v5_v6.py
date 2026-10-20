# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: compare water faces of fluid_domain_v5.scdoc and fluid_domain_v6.scdoc by
# (group, type, rounded area, rounded bbox) and list faces present only in v6 or only in v5,
# with edge lengths and the minimum interior angle proxy (shortest edge / bbox diagonal).
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix

DOCS = [('v5', r'D:\work4\new_model_sim\09_seal_j2j3\fluid_domain_v5.scdoc'),
        ('v6', r'D:\work4\new_model_sim\09_seal_j2j3\fluid_domain_v6.scdoc')]
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\diff_faces_v5_v6.json'
report = {'status': 'started'}


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
    sets = {}
    rows_by = {}
    for key, path in DOCS:
        DocumentOpen.Execute(path)
        root = GetRootPart()
        face_group = {}
        for g in root.Groups:
            for x in g.Members:
                if hasattr(x, 'Shape') and x.Shape.GetType().Name == 'Face':
                    face_group[x.Shape.GetHashCode()] = u(g.Name)
        water = list(root.GetAllBodies())[0]
        rows = {}
        for idx, df in enumerate(list(water.Faces)):
            f = df.Shape
            b = f.GetBoundingBox(Matrix.Identity)
            lo = [round(v * 1e3, 2) for v in (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)]
            hi = [round(v * 1e3, 2) for v in (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)]
            grp = face_group.get(f.GetHashCode(), '?')
            sig = (grp, f.Geometry.GetType().Name, round(float(f.Area) * 1e6, 2), tuple(lo), tuple(hi))
            edges = sorted(round(float(e.Length) * 1e3, 4) for e in f.Edges)
            rows[sig] = {'face_index': idx, 'group': grp, 'type': sig[1], 'area_mm2': sig[2],
                         'lo_mm': lo, 'hi_mm': hi, 'edges_mm': edges,
                         'centre_mm': [round(0.5 * (lo[k] + hi[k]), 2) for k in range(3)]}
        rows_by[key] = rows
        sets[key] = set(rows)
    report['only_v6'] = [rows_by['v6'][s] for s in sorted(sets['v6'] - sets['v5'])]
    report['only_v5'] = [rows_by['v5'][s] for s in sorted(sets['v5'] - sets['v6'])]
    report['counts'] = {'v5': len(sets['v5']), 'v6': len(sets['v6'])}
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

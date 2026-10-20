# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: in fluid_domain_v5.scdoc, list the water faces lying in the planes x = 14.8 mm and
# x = 15.0 mm (the 0.2 mm parallel gaps at J2 l1/l2 and J3 l2/link3), with group, area and bbox.
# Then save a COPY with one extra named selection 'gap_faces' holding exactly those faces
# (fluid_domain_v5g.scdoc). The v5 document itself is not saved.
import codecs
import json
import os
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242 import IDocObject
from SpaceClaim.Api.V242.Geometry import Matrix

SCDOC = r'D:\work4\new_model_sim\12_coarse_mesh\fluid_domain_v12.scdoc'
OUT_SCDOC = r'D:\work4\new_model_sim\12_coarse_mesh\fluid_domain_v12g.scdoc'
OUT_JSON = r'D:\work4\new_model_sim\12_coarse_mesh\gap_faces_v12.json'
PLANES_X = (0.0148, 0.0150)
TOL = 2e-6
report = {'status': 'started', 'scdoc': SCDOC, 'planes_x_m': PLANES_X}


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


if os.path.exists(OUT_JSON) or os.path.exists(OUT_SCDOC):
    raise SystemExit('Refusing to overwrite')
try:
    DocumentOpen.Execute(SCDOC)
    root = GetRootPart()
    face_group = {}
    for g in root.Groups:
        for x in g.Members:
            if hasattr(x, 'Shape') and x.Shape.GetType().Name == 'Face':
                face_group[x.Shape.GetHashCode()] = u(g.Name)
    water = list(root.GetAllBodies())[0]
    rows, picked = [], []
    for idx, df in enumerate(list(water.Faces)):
        f = df.Shape
        if f.Geometry.GetType().Name != 'Plane':
            continue
        b = f.GetBoundingBox(Matrix.Identity)
        if b.MaxCorner.X - b.MinCorner.X > TOL:
            continue
        x = 0.5 * (b.MaxCorner.X + b.MinCorner.X)
        if not any(abs(x - px) <= TOL for px in PLANES_X):
            continue
        rows.append({'face_index': idx, 'group': face_group.get(f.GetHashCode(), '?'), 'x_mm': round(x * 1e3, 4),
                     'area_mm2': round(float(f.Area) * 1e6, 3),
                     'lo_mm': [round(v * 1e3, 3) for v in (b.MinCorner.Y, b.MinCorner.Z)],
                     'hi_mm': [round(v * 1e3, 3) for v in (b.MaxCorner.Y, b.MaxCorner.Z)]})
        picked.append(df)
    report['faces'] = rows
    report['total_area_mm2'] = sum(r['area_mm2'] for r in rows)
    save()
    if not picked:
        raise Exception('No gap faces found')
    members = List[IDocObject]()
    for df in picked:
        members.Add(df)
    Group.Create(root, 'gap_faces', members)
    DocumentSave.Execute(OUT_SCDOC)
    report['saved_copy'] = OUT_SCDOC
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

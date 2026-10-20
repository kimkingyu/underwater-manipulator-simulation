# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: are the off-l1 samples of faces 254/263 really on the trimmed face?
# Records the face bbox, the edges' z-range, and for each sample the water/solid state 0.1 mm
# on both sides along the surface normal plus the distance to base0. Saves only the json.
import codecs
import json
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector, PointUV

SCDOC = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2.scdoc'
SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2_verify_l1_b.json'
FACES = (254, 263)
N = 6
H = 1e-4
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
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


def world_copy(design_body):
    shape = design_body.GetMaster().Shape.Copy()
    shape.Transform(design_body.GetTransformToMaster().Inverse)
    return shape


def name_for(volume):
    if volume > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda row: abs(row[0] - volume))
    return best[1] if abs(best[0] - volume) / best[0] < 2e-3 else None


def as_point(hit):
    return getattr(hit, 'Point', hit)


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


def box_mm(obj):
    b = obj.GetBoundingBox(Matrix.Identity)
    return [round(v * 1000, 3) for v in (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z,
                                         b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)]


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    report['l1_bbox_mm'] = box_mm(tools['l1'])
    report['base0_bbox_mm'] = box_mm(tools['base0'])
    DocumentOpen.Execute(SCDOC)
    water = list(GetRootPart().GetAllBodies())[0].Shape
    faces = list(water.Faces)
    out = []
    for idx in FACES:
        face = faces[idx]
        row = {'face_index': idx, 'bbox_mm': box_mm(face), 'edges': [], 'samples': []}
        for e in face.Edges:
            row['edges'].append({'type': e.Geometry.GetType().Name, 'length_mm': float(e.Length) * 1000,
                                 'bbox_mm': box_mm(e)})
        box = face.BoxUV
        ru, rv = box.RangeU, box.RangeV
        row['uv_range'] = [ru.Start, ru.End, rv.Start, rv.End]
        for i in range(N):
            for j in range(N):
                uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N,
                                    rv.Start + (rv.End - rv.Start) * (j + 0.5) / N)
                if not face.ContainsParam(uv):
                    continue
                ev = face.Geometry.Evaluate(uv)
                p = ev.Point
                n = ev.Normal
                plus = p + n.UnitVector * H if hasattr(n, 'UnitVector') else Point.Create(p.X + n.X * H, p.Y + n.Y * H, p.Z + n.Z * H)
                minus = Point.Create(2 * p.X - plus.X, 2 * p.Y - plus.Y, 2 * p.Z - plus.Z)
                row['samples'].append({'z_mm': round(p.Z * 1000, 3),
                                       'water_plus': bool(water.ContainsPoint(plus)),
                                       'water_minus': bool(water.ContainsPoint(minus)),
                                       'to_l1_m': dist(as_point(tools['l1'].ProjectPoint(p)), p),
                                       'to_base0_m': dist(as_point(tools['base0'].ProjectPoint(p)), p)})
        out.append(row)
    report['faces'] = out
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

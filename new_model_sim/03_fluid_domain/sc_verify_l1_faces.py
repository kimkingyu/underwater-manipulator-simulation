# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only follow-up for faces 254 and 263 of fluid_domain_v2.scdoc (wall_l1): for every UV sample
# that ContainsParam accepted, record distance to the water face itself (is the sample really on the
# trimmed face?) and distance to l1 and to every other part. Nothing is saved except the json.
import codecs
import json
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector, PointUV

SCDOC = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2.scdoc'
SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2_verify_l1.json'
FACES = (254, 263)
N = 6
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


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    l1 = tools['l1']
    report['l1_faces'] = [{'type': f.Geometry.GetType().Name, 'area_mm2': float(f.Area) * 1e6} for f in l1.Faces]
    DocumentOpen.Execute(SCDOC)
    water = list(GetRootPart().GetAllBodies())[0]
    faces = list(water.Faces)
    out = []
    for idx in FACES:
        face = faces[idx].Shape
        row = {'face_index': idx, 'type': face.Geometry.GetType().Name, 'area_mm2': float(face.Area) * 1e6,
               'reversed': bool(face.IsReversed), 'loops': len(list(face.Loops)), 'samples': []}
        box = face.BoxUV
        ru, rv = box.RangeU, box.RangeV
        for i in range(N):
            for j in range(N):
                uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N,
                                    rv.Start + (rv.End - rv.Start) * (j + 0.5) / N)
                if not face.ContainsParam(uv):
                    continue
                p = face.Geometry.Evaluate(uv).Point
                on_face = dist(as_point(face.ProjectPoint(p).Point), p)
                d_parts = dict((n, dist(as_point(s.ProjectPoint(p)), p)) for n, s in tools.items())
                row['samples'].append({'p_mm': [round(p.X * 1000, 3), round(p.Y * 1000, 3), round(p.Z * 1000, 3)],
                                       'to_face_m': on_face, 'to_l1_m': d_parts['l1'],
                                       'inside_l1': bool(l1.ContainsPoint(p)),
                                       'nearest': min(d_parts, key=lambda n: d_parts[n])})
        out.append(row)
    report['faces'] = out
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

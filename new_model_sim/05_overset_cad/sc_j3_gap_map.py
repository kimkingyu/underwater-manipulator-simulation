# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only map of thin water layers in the j3 overset fluid (j3_fluid.scdoc, unchanged):
# every wall face is sampled on a UV grid; from each sample a ray goes into the water along the
# face normal and the local water thickness is the first distance at which the ray re-enters the
# j3 material union (link3 U pin4 U jaw1 U jaw2, rebuilt from AUVM.step exactly as the builder did).
# Samples with thickness <= 0.2 mm are reported with owner, opposite part and location.
import codecs
import json
import os
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, PointUV

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
D = r'D:\work4\new_model_sim\05_overset_cad'
SCDOC = os.path.join(D, 'j3_fluid.scdoc')
OUT_JSON = os.path.join(D, 'j3_gap_map.json')
MEMBERS = ('link3', 'pin4', 'jaw1', 'jaw2')
N = 12
STEPS = (2e-5, 5e-5, 1e-4, 1.5e-4, 2e-4)
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'samples_per_dir': N, 'ray_steps_m': STEPS}


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


def offset(p, n, h):
    return Point.Create(p.X + n[0] * h, p.Y + n[1] * h, p.Z + n[2] * h)


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    ordered = sorted(MEMBERS, key=lambda n: -tools[n].Volume)
    material = tools[ordered[0]].Copy()
    for n in ordered[1:]:
        lst = List[Body]()
        lst.Add(tools[n].Copy())
        material.Unite(lst, 1e-7)
    report['material_volume_m3'] = float(material.Volume)
    DocumentOpen.Execute(SCDOC)
    root = GetRootPart()
    face_group = {}
    for g in root.Groups:
        for x in g.Members:
            if hasattr(x, 'Shape') and x.Shape.GetType().Name == 'Face':
                face_group[x.Shape.GetHashCode()] = u(g.Name)
    water = list(root.GetAllBodies())[0]
    thin = []
    checked = 0
    for idx, dface in enumerate(list(water.Faces)):
        group = face_group.get(dface.Shape.GetHashCode(), '?')
        if not group.startswith('wall_'):
            continue
        face = dface.Shape
        box = face.BoxUV
        ru, rv = box.RangeU, box.RangeV
        for i in range(N):
            for j in range(N):
                uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N,
                                    rv.Start + (rv.End - rv.Start) * (j + 0.5) / N)
                if not face.ContainsParam(uv):
                    continue
                ev = face.Geometry.Evaluate(uv)
                p = ev.Point
                nv = ev.Normal
                n = [nv.X, nv.Y, nv.Z]
                # orient n into the water
                if not water.Shape.ContainsPoint(offset(p, n, 5e-6)):
                    n = [-n[0], -n[1], -n[2]]
                    if not water.Shape.ContainsPoint(offset(p, n, 5e-6)):
                        continue
                checked += 1
                for h in STEPS:
                    q = offset(p, n, h)
                    if material.ContainsPoint(q):
                        d_parts = dict((m, dist(as_point(tools[m].ProjectPoint(q)), q)) for m in MEMBERS)
                        opposite = min(d_parts, key=lambda m: d_parts[m])
                        thin.append({'face_index': idx, 'group': group, 'thickness_le_m': h,
                                     'opposite_part': opposite,
                                     'p_mm': [round(p.X * 1000, 3), round(p.Y * 1000, 3), round(p.Z * 1000, 3)],
                                     'face_type': face.Geometry.GetType().Name,
                                     'face_area_mm2': float(face.Area) * 1e6})
                        break
        if idx % 50 == 0:
            report['progress_face'] = idx
            report['samples_checked'] = checked
            report['thin'] = thin
            save()
    report['samples_checked'] = checked
    report['thin'] = thin
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

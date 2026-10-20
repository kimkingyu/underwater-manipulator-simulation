# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: how do l2 and link3 meet at J3? Measures overlap volume, the closest faces,
# and whether any planar faces of the two bodies are coplanar and overlapping (the classic
# cause of ACIS "General Failure" in subtract/unite). Also tries small separations along
# the J3 axis to see which gap makes the Boolean succeed. Saves nothing but the json.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\j3_contact.json'
J3_CENTER = (0.014796, -0.1226, -0.313202)
J3_AXIS = (1.0, 0.0, 0.0)
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


def plane_faces(shape):
    rows = []
    for face in shape.Faces:
        geom = face.Geometry
        if geom.GetType().Name != 'Plane':
            continue
        frame = geom.Frame
        normal = frame.DirZ
        origin = frame.Origin
        offset = origin.X * normal.X + origin.Y * normal.Y + origin.Z * normal.Z
        if face.IsReversed:
            normal = -normal
            offset = -offset
        box = face.GetBoundingBox(Matrix.Identity)
        rows.append({'normal': [normal.X, normal.Y, normal.Z], 'offset': offset, 'area': float(face.Area),
                     'bbox': [box.MinCorner.X, box.MinCorner.Y, box.MinCorner.Z,
                              box.MaxCorner.X, box.MaxCorner.Y, box.MaxCorner.Z]})
    return rows


def try_boolean(a, b, label):
    """Subtract b from a copy of a, and unite them. Returns dict of outcomes."""
    out = {'label': label}
    for op in ('Subtract', 'Unite', 'Intersect'):
        try:
            base = a.Copy()
            lst = List[Body]()
            lst.Add(b.Copy())
            getattr(base, op)(lst, 1e-5)
            out[op] = float(base.Volume)
        except Exception as exc:
            out[op] = 'FAIL: ' + u(exc)
    return out


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    l2, link3 = tools['l2'], tools['link3']
    report['volumes'] = {'l2': float(l2.Volume), 'link3': float(link3.Volume)}
    report['as_is'] = try_boolean(l2, link3, 'l2 vs link3 as exported')
    save()
    # Coplanar overlapping planar faces between l2 and link3.
    pl2, pl3 = plane_faces(l2), plane_faces(link3)
    coplanar = []
    for a in pl2:
        for b in pl3:
            dot = sum(a['normal'][i] * b['normal'][i] for i in range(3))
            if abs(abs(dot) - 1.0) > 1e-6:
                continue
            same_plane = abs(a['offset'] - (b['offset'] if dot > 0 else -b['offset'])) < 1e-6
            if not same_plane:
                continue
            overlap = all(min(a['bbox'][i + 3], b['bbox'][i + 3]) - max(a['bbox'][i], b['bbox'][i]) > -1e-9
                          for i in range(3))
            if overlap:
                coplanar.append({'normal': a['normal'], 'offset_m': a['offset'],
                                 'facing': 'opposite (touching)' if dot < 0 else 'same direction',
                                 'l2_area_m2': a['area'], 'link3_area_m2': b['area']})
    report['coplanar_planar_faces'] = coplanar
    save()
    # Minimum distance between bodies (sampled through face projection of vertices).
    # Separation sweep: move link3 along the J3 axis (both directions) and retry.
    sweep = []
    for gap in (1e-6, 1e-5, 5e-5, 1e-4, 2e-4):
        for sign in (1.0, -1.0):
            moved = link3.Copy()
            moved.Transform(Matrix.CreateTranslation(Vector.Create(J3_AXIS[0], J3_AXIS[1], J3_AXIS[2]) * (sign * gap)))
            res = try_boolean(l2, moved, 'link3 shifted %+.0e m along J3 axis' % (sign * gap))
            sweep.append(res)
            save()
    report['axis_shift_sweep'] = sweep
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only contact survey of every body pair in AUVM.step:
#   - closest separation distance and points
#   - collision class (None / Touch / Intersect / Contained / ContainedTouch)
#   - overlap volume (Intersect), and whether a pairwise Unite and Subtract succeed
#   - coplanar, opposite-facing, overlapping planar faces (face-to-face contacts)
# Saves nothing except the json report.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\all_contacts.json'
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'pairs': []}


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


def box_list(box):
    return [box.MinCorner.X, box.MinCorner.Y, box.MinCorner.Z, box.MaxCorner.X, box.MaxCorner.Y, box.MaxCorner.Z]


def plane_faces(shape):
    rows = []
    for face in shape.Faces:
        geom = face.Geometry
        if geom.GetType().Name != 'Plane':
            continue
        normal = geom.Frame.DirZ
        origin = geom.Frame.Origin
        offset = origin.X * normal.X + origin.Y * normal.Y + origin.Z * normal.Z
        if face.IsReversed:
            normal = -normal
            offset = -offset
        rows.append({'n': [normal.X, normal.Y, normal.Z], 'd': offset, 'area': float(face.Area),
                     'box': box_list(face.GetBoundingBox(Matrix.Identity))})
    return rows


def coplanar_contacts(pa, pb):
    out = []
    for a in pa:
        for b in pb:
            dot = sum(a['n'][i] * b['n'][i] for i in range(3))
            if dot > -1.0 + 1e-6:
                continue  # need opposite normals: outward faces pressed together
            if abs(a['d'] + b['d']) > 1e-6:
                continue
            gap = [min(a['box'][i + 3], b['box'][i + 3]) - max(a['box'][i], b['box'][i]) for i in range(3)]
            if all(g > -1e-9 for g in gap):
                out.append({'normal_a': [round(v, 6) for v in a['n']], 'plane_offset_m': a['d'],
                            'area_a_m2': a['area'], 'area_b_m2': b['area'],
                            'box_overlap_extent_m': gap})
    return out


def boolean(a, b, op):
    try:
        base = a.Copy()
        lst = List[Body]()
        lst.Add(b.Copy())
        getattr(base, op)(lst, 1e-5)
        return float(base.Volume)
    except Exception as exc:
        return 'FAIL: ' + u(exc)


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    names = ['hull', 'base0', 'l1', 'l2', 'link3', 'pin4', 'jaw1', 'jaw2']
    planes = dict((n, plane_faces(tools[n])) for n in names)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            row = {'a': a, 'b': b}
            try:
                sep = tools[a].GetClosestSeparation(tools[b])
                row['distance_m'] = float(sep.Distance)
                row['point_a'] = [sep.PointA.X, sep.PointA.Y, sep.PointA.Z]
            except Exception as exc:
                row['distance_error'] = u(exc)
            if row.get('distance_m', 0.0) > 1e-3:
                row['class'] = 'apart'
                report['pairs'].append(row)
                save()
                continue
            try:
                row['collision'] = u(tools[a].GetCollision(tools[b]))
            except Exception as exc:
                row['collision_error'] = u(exc)
            row['intersect_m3'] = boolean(tools[a], tools[b], 'Intersect')
            row['unite'] = boolean(tools[a], tools[b], 'Unite')
            row['subtract_b_from_a'] = boolean(tools[a], tools[b], 'Subtract')
            row['coplanar_contacts'] = coplanar_contacts(planes[a], planes[b])
            report['pairs'].append(row)
            save()
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

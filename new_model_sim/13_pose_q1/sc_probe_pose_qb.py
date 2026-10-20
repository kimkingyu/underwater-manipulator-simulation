# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only probe: AUVM.step parts posed at q = [0.45, -0.50, 0.40] (joint axes/centres of
# 11_theta_fit/matlab/uvms_geometry.m, CAD frame). For every pair within 3 mm: closest separation,
# intersection volume. Same at q = 0 for reference. Writes probe_pose_qb.json only.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Line, Direction

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\13_pose_q1\probe_pose_qb.json'
Q = [0.45, -0.50, -0.40]
C = [(-4e-06, -0.0366, -0.460702), (0.016686, -0.0666, -0.511202), (0.014796, -0.1226, -0.313202)]
A = [(0, 0, -1), (-1, 0, 0), (1, 0, 0)]
GROUP = {'hull': 0, 'base0': 0, 'l1': 1, 'l2': 2, 'link3': 3, 'pin4': 3, 'jaw1': 3, 'jaw2': 3}
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'q': Q}


def u(v):
    try:
        return unicode(v)
    except Exception:
        return repr(v)


def save():
    s = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        s.write(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        s.close()


def world_copy(db):
    sh = db.GetMaster().Shape.Copy()
    sh.Transform(db.GetTransformToMaster().Inverse)
    return sh


def name_for(v):
    if v > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda r: abs(r[0] - v))
    return best[1] if abs(best[0] - v) / best[0] < 2e-3 else None


def pose_matrix(group, q):
    # serial chain: distal joint first, all rotations about the q=0 axes
    m = Matrix.Identity
    for j in range(group, 0, -1):
        rot = Matrix.CreateRotation(Line.Create(Point.Create(*C[j - 1]), Direction.Create(*A[j - 1])), q[j - 1])
        m = rot * m
    return m


def inter_mm3(a, b):
    c = a.Copy()
    lst = List[Body]()
    lst.Add(b.Copy())
    try:
        c.Intersect(lst, 1e-7)
    except Exception as exc:
        return 'FAIL ' + u(exc)
    return float(c.Volume) * 1e9 if c.Volume > 0 else 0.0


def census(tools, tag):
    names = sorted(tools)
    rows = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            row = {'a': a, 'b': b}
            try:
                sep = tools[a].GetClosestSeparation(tools[b])
                row['dist_mm'] = float(sep.Distance) * 1e3
                row['pa_mm'] = [round(sep.PointA.X * 1e3, 3), round(sep.PointA.Y * 1e3, 3), round(sep.PointA.Z * 1e3, 3)]
            except Exception as exc:
                row['dist_error'] = u(exc)
            if row.get('dist_mm', 0.0) > 3.0:
                continue
            row['intersect_mm3'] = inter_mm3(tools[a], tools[b])
            rows.append(row)
    report[tag] = rows
    save()


try:
    save()
    DocumentOpen.Execute(SOURCE)
    base = {}
    for db in GetRootPart().GetAllBodies():
        sh = world_copy(db)
        base[name_for(float(sh.Volume))] = sh
    report['parts'] = sorted(base)
    census(base, 'q0')
    posed = {}
    for n, sh in base.items():
        c = sh.Copy()
        c.Transform(pose_matrix(GROUP[n], Q))
        posed[n] = c
    report['posed_volume_drift'] = dict((n, float(posed[n].Volume) / float(base[n].Volume) - 1.0) for n in base)
    census(posed, 'q1')
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

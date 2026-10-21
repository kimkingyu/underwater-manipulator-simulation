# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: collision-free joint ranges of the new CAD (AUVM.step parts, no seals), joint axes/centres of
# uvms_geometry.m. Collision = intersection > V_HIT between parts of different groups
# (0 hull+base0, 1 l1, 2 l2, 3 link3+pin4+jaw1+jaw2).
#   (a) single joint, the other two at 0: march 0 -> +-1.57 in 0.02 rad, bisect the first hit to 0.005 rad
#   (b) J2 x J3 grid at J1 = 0, step 0.0785 rad over [-1.57, 1.57]
#   (c) J1 at {-1.57, -0.785, 0.785, 1.57}: the J2 x J3 grid again at step 0.157 rad (hull clearance check)
# RESUME: reads joint_limits.json, finishes the J1 = +0.785 / +1.57 grids (rows missing), per-pose
# errors recorded as -1. Writes joint_limits.json only.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Line, Direction

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\13_pose_q1\joint_limits.json'
C = [(-4e-06, -0.0366, -0.460702), (0.016686, -0.0666, -0.511202), (0.014796, -0.1226, -0.313202)]
A = [(0, 0, -1), (-1, 0, 0), (1, 0, 0)]
GROUP = {'hull': 0, 'base0': 0, 'l1': 1, 'l2': 2, 'link3': 3, 'pin4': 3, 'jaw1': 3, 'jaw2': 3}
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
QMAX = 1.57
STEP = 0.02
TOL_Q = 0.005
V_HIT = 0.5e-9      # m3 (0.5 mm3)
report = json.loads(codecs.open(OUT_JSON, 'r', 'utf-8').read())
report['status'] = 'resume_started'


def u(v):
    try:
        return unicode(v)
    except Exception:
        return repr(v)


def save():
    s = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        s.write(json.dumps(report, ensure_ascii=False, indent=1))
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
    m = Matrix.Identity
    for j in range(group, 0, -1):
        m = Matrix.CreateRotation(Line.Create(Point.Create(*C[j - 1]), Direction.Create(*A[j - 1])), q[j - 1]) * m
    return m


def boxes_overlap(a, b):
    return all(a[0][k] <= b[1][k] and b[0][k] <= a[1][k] for k in range(3))


def bbox(sh):
    b = sh.GetBoundingBox(Matrix.Identity)
    return ([b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z], [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z])


def hits(q):
    posed, box = {}, {}
    for n, sh in BASE.items():
        c = sh.Copy()
        c.Transform(pose_matrix(GROUP[n], q))
        posed[n] = c
        box[n] = bbox(c)
    out = {}
    for a, b in PAIRS:
        if not boxes_overlap(box[a], box[b]):
            continue
        c = posed[a].Copy()
        lst = List[Body]()
        lst.Add(posed[b].Copy())
        try:
            c.Intersect(lst, 1e-7)
            v = float(c.Volume) if c.Volume > 0 else 0.0
        except Exception:
            v = 0.0      # empty intersection raises in some builds
        if v > V_HIT:
            out[a + '/' + b] = round(v * 1e9, 2)
    return out



def grid(j1, step, key):
    n = int(round(2 * QMAX / step))
    vals = [-QMAX + i * 2 * QMAX / n for i in range(n + 1)]
    g = report.get(key) or {'q1': j1, 'q2_values': vals, 'q3_values': vals, 'free': [], 'pairs': {}}
    report[key] = g
    for q2 in vals[len(g['free']):]:
        row = []
        for q3 in vals:
            try:
                h = hits([j1, q2, q3])
            except Exception as exc:
                report.setdefault('pose_errors', []).append([j1, q2, q3, u(exc)[:200]])
                row.append(-1)
                continue
            row.append(0 if h else 1)
            for p in h:
                g['pairs'][p] = g['pairs'].get(p, 0) + 1
        g['free'].append(row)
        save()


try:
    save()
    DocumentOpen.Execute(SOURCE)
    BASE = {}
    for db in GetRootPart().GetAllBodies():
        sh = world_copy(db)
        BASE[name_for(float(sh.Volume))] = sh
    names = sorted(BASE)
    PAIRS = [(names[i], names[k]) for i in range(len(names)) for k in range(i + 1, len(names))
             if GROUP[names[i]] != GROUP[names[k]]]
    for j1 in (0.785, 1.57):
        grid(j1, 0.157, 'grid_j1_%+.3f' % j1)
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    try:
        report['traceback'] = u(traceback.format_exc())
    except Exception:
        report['traceback'] = 'format_exc failed'
finally:
    save()

# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only probe around given points (AUVM.step, unshifted frame): faces of every arm body whose box
# lies within R of a point (with plane normal / cylinder axis+radius), and the distance from each
# point to every body. Points come from Fluent's worst cells. Writes probe_points_v5g.json only.
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\probe_points_v5g.json'
POINTS = {'min_orth_v5g': (-2.44198e-03, -1.22435e-01, -3.41637e-01),
          'max_aspect_v5g': (-6.32298e-04, -1.22523e-01, -3.52565e-01)}
R = 3e-3
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'points_m': POINTS, 'radius_m': R}


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


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    out = {}
    for key, xyz in POINTS.items():
        p = Point.Create(*xyz)
        row = {'inside': [n for n, s in tools.items() if s.ContainsPoint(p)], 'body_dist_mm': {}, 'faces': []}
        for n, s in tools.items():
            hit = s.ProjectPoint(p)
            row['body_dist_mm'][n] = round(dist(getattr(hit, 'Point', hit), p) * 1e3, 5)
        for n, s in tools.items():
            for f in s.Faces:
                b = f.GetBoundingBox(Matrix.Identity)
                lo = (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)
                hi = (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)
                if not all(lo[k] - R <= xyz[k] <= hi[k] + R for k in range(3)):
                    continue
                hit = f.ProjectPoint(p)
                d = dist(getattr(hit, 'Point', hit), p)
                if d > R:
                    continue
                g = f.Geometry
                fr = {'body': n, 'type': g.GetType().Name, 'dist_mm': round(d * 1e3, 5),
                      'area_mm2': round(float(f.Area) * 1e6, 4),
                      'lo_mm': [round(v * 1e3, 4) for v in lo], 'hi_mm': [round(v * 1e3, 4) for v in hi]}
                if fr['type'] == 'Plane':
                    nz = g.Frame.DirZ
                    fr['normal'] = [round(nz.X, 5), round(nz.Y, 5), round(nz.Z, 5)]
                    o = g.Frame.Origin
                    fr['origin_mm'] = [round(o.X * 1e3, 4), round(o.Y * 1e3, 4), round(o.Z * 1e3, 4)]
                if fr['type'] in ('Cylinder', 'Cone'):
                    o, dd = g.Axis.Origin, g.Axis.Direction
                    fr['axis_origin_mm'] = [round(o.X * 1e3, 4), round(o.Y * 1e3, 4), round(o.Z * 1e3, 4)]
                    fr['axis_dir'] = [round(dd.X, 5), round(dd.Y, 5), round(dd.Z, 5)]
                    if fr['type'] == 'Cylinder':
                        fr['radius_mm'] = float(g.Radius) * 1e3
                row['faces'].append(fr)
        row['faces'].sort(key=lambda r: r['dist_mm'])
        out[key] = row
    report['probe'] = out
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

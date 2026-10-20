# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only map of the l2 R28 / base0-l1 R30 cylinder gap (parallel axes along Z) in AUVM.step
# (l2/base0/l1 are not shifted in v3/v4). Along z, on lines parallel to the axes at lateral offset s
# from the axis-connecting plane, sample the water gap = dist(p, l2) + dist(p, nearest of base0/l1)
# at the mid point between the two nominal cylinders. Writes probe_cyl_gap_j3.json only.
import codecs
import json
import math
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\probe_cyl_gap_j3.json'
BIG_AXIS = (-0.0020, -0.1526)
L2_AXIS = (0.0028, -0.0946)
R_BIG, R_L2 = 0.030, 0.028
Z0, Z1, DZ = -0.4000, -0.3300, 2.5e-4
S_LIST = (0.0, 2e-3, -2e-3, 5e-3, -5e-3)
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'big_axis_m': BIG_AXIS, 'l2_axis_m': L2_AXIS, 'dz_m': DZ, 's_m': S_LIST}


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


def dist_to(body, p):
    hit = body.ProjectPoint(p)
    q = getattr(hit, 'Point', hit)
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    dx, dy = L2_AXIS[0] - BIG_AXIS[0], L2_AXIS[1] - BIG_AXIS[1]
    D = math.hypot(dx, dy)
    ux, uy = dx / D, dy / D
    vx, vy = -uy, ux
    report['axis_distance_m'] = D
    report['nominal_gap_m'] = D - R_BIG - R_L2
    report['u'] = [ux, uy]
    report['v'] = [vx, vy]
    lines = {}
    for s in S_LIST:
        nb = math.sqrt(R_BIG ** 2 - s ** 2)
        nl = D - math.sqrt(R_L2 ** 2 - s ** 2)
        n_mid = 0.5 * (nb + nl)
        rows = []
        z = Z0
        while z <= Z1 + 1e-12:
            p = Point.Create(BIG_AXIS[0] + n_mid * ux + s * vx, BIG_AXIS[1] + n_mid * uy + s * vy, z)
            inside = [n for n in ('l2', 'link3', 'base0', 'l1') if tools[n].ContainsPoint(p)]
            d_l2 = dist_to(tools['l2'], p)
            d_b0 = dist_to(tools['link3'], p)
            d_l1 = d_b0
            big = 'link3'
            rows.append([round(z * 1e3, 3), round((d_l2 + min(d_b0, d_l1)) * 1e3, 5), big,
                         round(d_l2 * 1e3, 5), round(min(d_b0, d_l1) * 1e3, 5), inside])
            z += DZ
        lines['%+.1f' % (s * 1e3)] = {'nominal_gap_mm': (nl - nb) * 1e3, 'n_mid_mm': n_mid * 1e3,
                                      'rows_z_gap_big_dl2_dbig_inside': rows}
        report['lines'] = lines
        save()
    faces = {}
    for n in ('l2', 'link3'):
        rows = []
        for f in tools[n].Faces:
            g = f.Geometry
            b = f.GetBoundingBox(Matrix.Identity)
            lo = (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)
            hi = (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)
            if hi[2] < Z0 or lo[2] > Z1:
                continue
            cx = BIG_AXIS[0] + 0.0300 * ux
            cy = BIG_AXIS[1] + 0.0300 * uy
            if not (lo[0] - 0.006 <= cx <= hi[0] + 0.006 and lo[1] - 0.006 <= cy <= hi[1] + 0.006):
                continue
            row = {'type': g.GetType().Name, 'lo_mm': [round(v * 1e3, 4) for v in lo],
                   'hi_mm': [round(v * 1e3, 4) for v in hi], 'area_mm2': round(float(f.Area) * 1e6, 3)}
            if row['type'] == 'Cylinder':
                o, d = g.Axis.Origin, g.Axis.Direction
                row['radius_mm'] = round(float(g.Radius) * 1e3, 5)
                row['axis_origin_mm'] = [round(o.X * 1e3, 4), round(o.Y * 1e3, 4)]
                row['axis_dir'] = [round(d.X, 4), round(d.Y, 4), round(d.Z, 4)]
            if row['type'] == 'Torus':
                try:
                    row['major_mm'] = round(float(g.MajorRadius) * 1e3, 4)
                    row['minor_mm'] = round(float(g.MinorRadius) * 1e3, 4)
                except Exception:
                    pass
            if row['type'] == 'Plane':
                nz = g.Frame.DirZ
                row['normal'] = [round(nz.X, 4), round(nz.Y, 4), round(nz.Z, 4)]
            rows.append(row)
        faces[n] = sorted(rows, key=lambda r: -r['hi_mm'][2])
    report['faces_near_gap'] = faces
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

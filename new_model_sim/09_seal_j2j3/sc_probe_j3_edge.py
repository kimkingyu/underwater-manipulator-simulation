# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only probe of the J3 tangency region in AUVM.step (unshifted frame):
# point membership in l2 / link3 on a grid across the tangent line, and the link3 / l2 faces whose
# boxes touch the line. Writes probe_j3_edge.json only.
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\probe_j3_edge.json'
Y_T, Z_P = -0.1226, -0.3412
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'y_t': Y_T, 'z_p': Z_P}


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


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    grid = []
    for x in (-0.020, -0.005, 0.010):
        for dy in (-6e-3, -3e-3, -1e-3, -2e-4, 2e-4, 1e-3, 3e-3, 6e-3):
            for dz in (-1e-3, -1e-4, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3):
                p = Point.Create(x, Y_T + dy, Z_P + dz)
                grid.append({'x_mm': x * 1e3, 'dy_mm': dy * 1e3, 'dz_mm': dz * 1e3,
                             'in': [n for n in ('l2', 'link3') if tools[n].ContainsPoint(p)]})
    report['grid'] = grid
    faces = {}
    for n in ('l2', 'link3'):
        rows = []
        for f in tools[n].Faces:
            b = f.GetBoundingBox(Matrix.Identity)
            lo = (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)
            hi = (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)
            if lo[1] - 7e-3 <= Y_T <= hi[1] + 7e-3 and lo[2] - 4e-3 <= Z_P <= hi[2] + 4e-3 and hi[0] > -0.03 and lo[0] < 0.02:
                g = f.Geometry
                row = {'type': g.GetType().Name, 'area_mm2': float(f.Area) * 1e6,
                       'lo_mm': [round(v * 1e3, 4) for v in lo], 'hi_mm': [round(v * 1e3, 4) for v in hi]}
                if row['type'] == 'Plane':
                    nz = g.Frame.DirZ
                    row['normal'] = [round(nz.X, 5), round(nz.Y, 5), round(nz.Z, 5)]
                if row['type'] == 'Cylinder':
                    row['radius_mm'] = float(g.Radius) * 1e3
                    o, d = g.Axis.Origin, g.Axis.Direction
                    row['axis_origin_mm'] = [round(o.X * 1e3, 4), round(o.Y * 1e3, 4), round(o.Z * 1e3, 4)]
                    row['axis_dir'] = [round(d.X, 5), round(d.Y, 5), round(d.Z, 5)]
                rows.append(row)
        faces[n] = rows
    report['faces_near_line'] = faces
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

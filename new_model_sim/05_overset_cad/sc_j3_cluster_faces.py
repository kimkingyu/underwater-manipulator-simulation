# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: geometry parameters of pin4 / jaw1 / jaw2 faces inside the thin-gap cluster box
# (x -26..8, y -162..-143, z -561..-534 mm): planes (normal, offset), cylinders (axis, radius),
# tori (axis, major/minor radius), cones; plus face bbox and area. Saves only the json.
import codecs
import json
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\05_overset_cad\j3_cluster_faces.json'
BOX = (-0.026, -0.162, -0.561, 0.008, -0.143, -0.534)
PARTS = ('pin4', 'jaw1', 'jaw2')
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'box_m': BOX}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def r6(v):
    return round(v * 1000.0, 4)


def world_copy(design_body):
    shape = design_body.GetMaster().Shape.Copy()
    shape.Transform(design_body.GetTransformToMaster().Inverse)
    return shape


def name_for(volume):
    if volume > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda row: abs(row[0] - volume))
    return best[1] if abs(best[0] - volume) / best[0] < 2e-3 else None


def vec(d):
    return [round(d.X, 6), round(d.Y, 6), round(d.Z, 6)]


try:
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    for part in PARTS:
        rows = []
        bb = tools[part].GetBoundingBox(Matrix.Identity)
        report[part + '_bbox_mm'] = [r6(bb.MinCorner.X), r6(bb.MinCorner.Y), r6(bb.MinCorner.Z),
                                     r6(bb.MaxCorner.X), r6(bb.MaxCorner.Y), r6(bb.MaxCorner.Z)]
        for i, f in enumerate(tools[part].Faces):
            b = f.GetBoundingBox(Matrix.Identity)
            lo = [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z]
            hi = [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]
            if any(hi[k] < BOX[k] or lo[k] > BOX[k + 3] for k in range(3)):
                continue
            g = f.Geometry
            t = g.GetType().Name
            row = {'i': i, 'type': t, 'area_mm2': round(float(f.Area) * 1e6, 4),
                   'bbox_mm': [r6(v) for v in lo + hi], 'reversed': bool(f.IsReversed)}
            fr = getattr(g, 'Frame', None)
            if fr is not None:
                row['origin_mm'] = [r6(fr.Origin.X), r6(fr.Origin.Y), r6(fr.Origin.Z)]
                row['axis'] = vec(fr.DirZ)
            for attr in ('Radius', 'MajorRadius', 'MinorRadius', 'HalfAngle'):
                if hasattr(g, attr):
                    try:
                        val = float(getattr(g, attr))
                        row[attr] = r6(val) if attr != 'HalfAngle' else round(val, 6)
                    except Exception:
                        pass
            rows.append(row)
        report[part] = rows
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    stream = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        stream.write(json.dumps(report, ensure_ascii=False, indent=1))
    finally:
        stream.close()

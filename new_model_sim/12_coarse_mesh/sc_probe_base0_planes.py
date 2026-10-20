# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: planar faces of base0 (world frame) with |normal.Y| > 0.9: their y level, area, bbox.
# Also every base0 face whose bbox crosses y in [-0.012, 0.0].
import codecs
import json
import traceback

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT = r'D:\work4\new_model_sim\12_coarse_mesh\base0_planes.json'
rep = {'status': 'started', 'y_planes': [], 'near_hull_bottom': []}


def save():
    s = codecs.open(OUT, 'w', 'utf-8')
    s.write(json.dumps(rep, indent=2))
    s.close()


try:
    DocumentOpen.Execute(SOURCE)
    for db in GetRootPart().GetAllBodies():
        shape = db.GetMaster().Shape.Copy()
        shape.Transform(db.GetTransformToMaster().Inverse)
        if abs(float(shape.Volume) - 0.0006491) / 0.0006491 > 2e-3:
            continue
        for f in shape.Faces:
            b = f.GetBoundingBox(Matrix.Identity)
            lo = [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z]
            hi = [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]
            kind = f.Geometry.GetType().Name
            row = {'kind': kind, 'area_mm2': float(f.Area) * 1e6,
                   'bbox_mm': [round(v * 1000, 3) for v in lo + hi]}
            if kind == 'Plane':
                n = f.Geometry.Frame.DirZ
                if abs(n.Y) > 0.9:
                    row['y_mm'] = round(lo[1] * 1000, 4)
                    rep['y_planes'].append(row)
            if lo[1] <= 0.0 and hi[1] >= -0.012:
                rep['near_hull_bottom'].append(row)
    rep['y_planes'].sort(key=lambda r: r['y_mm'])
    rep['status'] = 'ok'
except Exception:
    rep['status'] = 'error'
    rep['traceback'] = traceback.format_exc()
finally:
    save()

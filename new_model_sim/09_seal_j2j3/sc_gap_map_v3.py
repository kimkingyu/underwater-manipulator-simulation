# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only map of thin water layers around the arm in the single-domain water v3
# (fluid_domain_v3.scdoc, unchanged). Each wall face whose box touches the arm region is sampled on
# a UV grid; from each sample a ray goes into the water along the face normal and the local water
# thickness is the first step at which the ray enters any part other than the face owner.
# Parts are rebuilt from AUVM.step with the same +10 um Y shift of the distal group as v3.
# Writes only gap_map_v3.json.
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point, PointUV, Vector

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
SCDOC = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v3.scdoc'
OUT_JSON = r'D:\work4\new_model_sim\09_seal_j2j3\gap_map_v3.json'
N = 12
STEPS = (5e-6, 1e-5, 2e-5, 5e-5, 1e-4, 1.5e-4, 2e-4)
REGION_LO = (-0.08, -0.20, -0.67)
REGION_HI = (0.08, 0.05, -0.27)
SHIFT_M = (0.0, 1e-5, 0.0)
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
OWNER = {'wall_auv': 'hull', 'wall_base0': 'base0', 'wall_l1': 'l1', 'wall_l2': 'l2',
         'wall_link3': 'link3', 'wall_pin4': 'pin4', 'wall_jaw1': 'jaw1', 'wall_jaw2': 'jaw2'}
report = {'status': 'started', 'samples_per_dir': N, 'ray_steps_m': STEPS, 'scdoc': SCDOC,
          'region_lo_m': REGION_LO, 'region_hi_m': REGION_HI}


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


def offset(p, n, h):
    return Point.Create(p.X + n[0] * h, p.Y + n[1] * h, p.Z + n[2] * h)


def in_region(face):
    b = face.GetBoundingBox(Matrix.Identity)
    lo = (b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z)
    hi = (b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z)
    return all(hi[k] >= REGION_LO[k] and lo[k] <= REGION_HI[k] for k in range(3))


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite')
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    move = Matrix.CreateTranslation(Vector.Create(*SHIFT_M))
    for name in DISTAL:
        tools[name].Transform(move)
    report['tools'] = sorted(tools)
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
    faces_scanned = 0
    for idx, dface in enumerate(list(water.Faces)):
        group = face_group.get(dface.Shape.GetHashCode(), '?')
        if group not in OWNER:
            continue
        face = dface.Shape
        if not in_region(face):
            continue
        faces_scanned += 1
        owner = OWNER[group]
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
                if not water.Shape.ContainsPoint(offset(p, n, 2e-6)):
                    n = [-n[0], -n[1], -n[2]]
                    if not water.Shape.ContainsPoint(offset(p, n, 2e-6)):
                        continue
                checked += 1
                for h in STEPS:
                    q = offset(p, n, h)
                    hit = [m for m, s in tools.items() if m != owner and s.ContainsPoint(q)]
                    if hit:
                        thin.append({'face_index': idx, 'group': group, 'owner': owner,
                                     'opposite_part': hit[0], 'thickness_le_m': h,
                                     'p_mm': [round(p.X * 1000, 3), round(p.Y * 1000, 3), round(p.Z * 1000, 3)],
                                     'n': [round(v, 4) for v in n],
                                     'face_type': face.Geometry.GetType().Name,
                                     'face_area_mm2': float(face.Area) * 1e6})
                        break
        if faces_scanned % 25 == 0:
            report['progress_face'] = idx
            report['faces_scanned'] = faces_scanned
            report['samples_checked'] = checked
            report['thin'] = thin
            save()
    report['faces_scanned'] = faces_scanned
    report['samples_checked'] = checked
    report['thin'] = thin
    pairs = {}
    for r in thin:
        key = '-'.join(sorted([r['owner'], r['opposite_part']]))
        pairs.setdefault(key, []).append(r['p_mm'])
    report['pair_summary'] = dict((k, {'samples': len(v),
                                       'lo_mm': [min(p[i] for p in v) for i in range(3)],
                                       'hi_mm': [max(p[i] for p in v) for i in range(3)]})
                                  for k, v in pairs.items())
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

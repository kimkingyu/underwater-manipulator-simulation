# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# j3 overset fluid v2 with the internal contact seams of the rigid j3 group sealed.
# Approved by kim: unite the j3 group and fill the zero-angle wedges where a part's edge fillet /
# round meets the flat contact plane of its neighbour. Nothing outside the seam bands changes.
#
# Seam filler, per (cluster box, contact plane Y0, part X, side s):
#   section profile A(d) = volume(X ∩ slab[Y0+s*d, Y0+s*(d+0.02mm)] ∩ cluster box) / 0.02mm
#   if A grows with depth (a fillet/chamfer curling away from the plane), h = start of the plateau
#   filler = X's slab at depth [h, h+h/4] stacked back to the plane (5 exact copies)
# i.e. X's full section is extended flat to the plane over the fillet height only.
# The filler is part of X for naming (wall owner = nearest of X ∪ its fillers).
# AUVM.step and the v1 j3 files are only read; writes j3_fluid_v2.scdoc + j3_v2_manifest.json.
import codecs
import json
import math
import os
import traceback
from System.Collections.Generic import List
from System.IO import File
from System.Security.Cryptography import SHA256
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, PointUV, Vector
from SpaceClaim.Api.V242 import IDocObject, DesignBody

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
D = r'D:\work4\new_model_sim\05_overset_cad'
MANIFEST_V1 = os.path.join(D, 'overset_manifest.json')
OUT_SCDOC = os.path.join(D, 'j3_fluid_v2.scdoc')
OUT_JSON = os.path.join(D, 'j3_v2_manifest.json')
MEMBERS = ['link3', 'pin4', 'jaw1', 'jaw2']
TOL_BOOL = 1e-7
TOL_OUTER = 1e-7
TOL_OWNER = 1e-5
N_SAMPLE = 6
DA = 2e-5
DEPTHS = [2e-5, 5e-5, 1e-4, 1.5e-4, 2e-4, 3e-4, 4e-4, 5e-4, 6e-4, 8e-4, 1e-3, 1.2e-3, 1.5e-3, 2e-3, 2.2e-3, 2.5e-3, 3e-3]
H_MAX = 2.2e-3
# Cluster boxes (x, z in metres) from j3_gap_map.json thin samples, with 1.5 mm margin.
CLUSTERS = [
    {'name': 'hub_jaw1_jaw2', 'x': (-0.0081, 0.0069), 'z': (-0.5595, -0.5406), 'planes': [-0.1451, -0.1601]},
    {'name': 'pin4_jaw1', 'x': (-0.0115, 0.0067), 'z': (-0.5593, -0.5356), 'planes': [-0.1501, -0.1551]},
    {'name': 'jaw1_lip', 'x': (-0.0247, -0.0124), 'z': (-0.5544, -0.5461), 'planes': [-0.1451, -0.1601]},
]
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
report = {'status': 'started', 'source': SOURCE, 'clusters': CLUSTERS, 'depths_m': DEPTHS,
          'policy': 'Seal zero-angle seam wedges inside the rigid j3 group only (approved by kim).'}


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


def sha256(path):
    algo = SHA256.Create()
    stream = File.OpenRead(path)
    try:
        return ''.join('%02x' % int(v) for v in algo.ComputeHash(stream))
    finally:
        stream.Dispose()
        algo.Dispose()


def world_copy(design_body):
    shape = design_body.GetMaster().Shape.Copy()
    shape.Transform(design_body.GetTransformToMaster().Inverse)
    return shape


def name_for(volume):
    if volume > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda row: abs(row[0] - volume))
    return best[1] if abs(best[0] - volume) / best[0] < 2e-3 else None


def bbox(obj):
    b = obj.GetBoundingBox(Matrix.Identity)
    return ([b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z], [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z])


def make_box(lo, hi):
    created = list(BlockBody.Create(Point.Create(*lo), Point.Create(*hi),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    shape = world_copy(created)
    created.Delete()
    return shape


def lst(*bodies):
    out = List[Body]()
    for b in bodies:
        out.Add(b.Copy())
    return out


def intersect(a, b):
    c = a.Copy()
    try:
        c.Intersect(lst(b), TOL_BOOL)
    except Exception:
        return None
    if c.Volume <= 0:
        return None
    return c


def slab_box(cl, y0, s, d0, d1):
    ya, yb = y0 + s * d0, y0 + s * d1
    return make_box((cl['x'][0], min(ya, yb), cl['z'][0]), (cl['x'][1], max(ya, yb), cl['z'][1]))


def as_point(hit):
    return getattr(hit, 'Point', hit)


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


def samples(face):
    box = face.BoxUV
    ru, rv = box.RangeU, box.RangeV
    pts = []
    for i in range(N_SAMPLE):
        for j in range(N_SAMPLE):
            uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N_SAMPLE,
                                rv.Start + (rv.End - rv.Start) * (j + 0.5) / N_SAMPLE)
            if face.ContainsParam(uv):
                pts.append(face.Geometry.Evaluate(uv).Point)
    if not pts:
        b = face.GetBoundingBox(Matrix.Identity)
        pts.append(as_point(face.ProjectPoint(b.Center)))
    return pts


def plateau_depth(areas):
    """First depth after growth where the section stops growing (rel. 0.2 %) for 2 samples."""
    if areas[-1] <= 0 or max(areas) <= areas[0] * 1.002 + 1e-12:
        return None
    for i in range(1, len(DEPTHS) - 2):
        if DEPTHS[i] > H_MAX:
            break
        if areas[i] <= areas[0] * 1.002:
            continue
        if areas[i + 1] <= areas[i] * 1.002 and areas[i + 2] <= areas[i] * 1.002:
            return DEPTHS[i]
    return None


try:
    if os.path.exists(OUT_JSON) or os.path.exists(OUT_SCDOC):
        raise Exception('Refusing to overwrite v2 outputs')
    save()
    stream = codecs.open(MANIFEST_V1, 'r', 'utf-8')
    v1 = json.load(stream)['parts']['j3']
    stream.close()
    lo, hi = v1['box_lo_m'], v1['box_hi_m']
    report['box_lo_m'], report['box_hi_m'] = lo, hi
    report['source_sha256_before'] = sha256(SOURCE)
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    doc = Document.Create()
    root = doc.MainPart

    # 1. Seam fillers.
    fillers = dict((m, []) for m in MEMBERS)
    log = []
    for cl in CLUSTERS:
        for y0 in cl['planes']:
            for m in MEMBERS:
                for s in (-1.0, 1.0):
                    areas = []
                    for d in DEPTHS:
                        piece = intersect(tools[m], slab_box(cl, y0, s, d, d + DA))
                        areas.append(float(piece.Volume) / DA if piece is not None else 0.0)
                    row = {'cluster': cl['name'], 'plane_y_m': y0, 'part': m, 'side': s,
                           'areas_mm2': [round(a * 1e6, 4) for a in areas]}
                    h = plateau_depth(areas)
                    row['h_m'] = h
                    if h is not None:
                        df = h / 4.0
                        ref = intersect(tools[m], slab_box(cl, y0, s, h, h + df))
                        if ref is None:
                            row['error'] = 'empty reference slab'
                        else:
                            filler = ref.Copy()
                            for k in range(1, 5):
                                c = ref.Copy()
                                c.Transform(Matrix.CreateTranslation(Vector.Create(0, -s * k * df, 0)))
                                filler.Unite(lst(c), TOL_BOOL)
                            added = filler.Copy()
                            added.Subtract(lst(tools[m]), TOL_BOOL)
                            row['filler_volume_mm3'] = float(filler.Volume) * 1e9
                            row['added_to_part_mm3'] = float(added.Volume) * 1e9
                            fl, fh = bbox(filler)
                            row['filler_bbox_mm'] = [round(v * 1000, 3) for v in fl + fh]
                            if added.Volume > 1e-15:
                                fillers[m].append(filler)
                    log.append(row)
                    report['filler_log'] = log
                    save()

    # 2. Extended parts (part ∪ its fillers) and the group material.
    ext = {}
    for m in MEMBERS:
        e = tools[m].Copy()
        for f in fillers[m]:
            e.Unite(lst(f), TOL_BOOL)
        ext[m] = e
    report['extended_parts'] = dict((m, {'part_mm3': float(tools[m].Volume) * 1e9,
                                         'extended_mm3': float(ext[m].Volume) * 1e9,
                                         'fillers': len(fillers[m])}) for m in MEMBERS)
    ordered = sorted(MEMBERS, key=lambda n: -ext[n].Volume)
    material = ext[ordered[0]].Copy()
    for m in ordered[1:]:
        material.Unite(lst(ext[m]), TOL_BOOL)
    v1_material = tools[ordered[0]].Copy()
    for m in ordered[1:]:
        v1_material.Unite(lst(tools[m]), TOL_BOOL)
    report['material_mm3'] = {'v1': float(v1_material.Volume) * 1e9, 'v2': float(material.Volume) * 1e9}
    save()

    # 3. Fluid.
    fluid = make_box(lo, hi)
    v_box = float(fluid.Volume)
    fluid.Subtract(lst(material), TOL_BOOL)
    pieces = list(fluid.SeparatePieces())
    outer, rows = [], []
    for p in pieces:
        pl, ph = bbox(p)
        touches = all(abs(pl[k] - lo[k]) <= TOL_OUTER and abs(ph[k] - hi[k]) <= TOL_OUTER for k in range(3))
        rows.append({'volume_m3': float(p.Volume), 'reaches_all_box_sides': touches})
        if touches:
            outer.append(p)
    report['pieces'] = rows
    if len(outer) != 1:
        raise Exception('Expected one outer fluid piece, found %d' % len(outer))
    main = outer[0]
    for m in ordered:
        main.Imprint(ext[m].Copy())
    report['fluid'] = {'volume_m3': float(main.Volume), 'v1_volume_m3': v1['fluid']['volume_m3'],
                       'faces': len(list(main.Faces)), 'v1_faces': v1['fluid']['faces'],
                       'closed': bool(main.IsClosed), 'manifold': bool(main.IsManifold),
                       'box_minus_material_m3': v_box - float(material.Volume)}
    save()
    if not main.IsClosed or not main.IsManifold:
        raise Exception('Fluid not closed/manifold')

    # 4. Names.
    body = DesignBody.Create(root, 'fluid_j3', main)
    groups, failures, close = {}, [], []
    max_owner = 0.0
    for idx, dface in enumerate(list(body.Faces)):
        face = dface.Shape
        fl, fh = bbox(face)
        on_outer = any(fh[k] - fl[k] <= TOL_OUTER and (abs(fl[k] - lo[k]) <= TOL_OUTER or abs(fl[k] - hi[k]) <= TOL_OUTER)
                       for k in range(3))
        if on_outer:
            groups.setdefault('overset_j3', []).append(dface)
            continue
        pts = samples(face)
        worst = dict((n, max(dist(as_point(ext[n].ProjectPoint(p)), p) for p in pts)) for n in MEMBERS)
        ranked = sorted((d, n) for n, d in worst.items())
        if ranked[0][0] > TOL_OWNER:
            failures.append({'face_index': idx, 'nearest': ranked[0][1], 'max_dist_m': ranked[0][0],
                             'area_mm2': float(face.Area) * 1e6})
            continue
        if ranked[1][0] <= TOL_OWNER:
            close.append({'face_index': idx, 'owner': ranked[0][1], 'other': ranked[1][1],
                          'd_owner_m': ranked[0][0], 'd_other_m': ranked[1][0], 'area_mm2': float(face.Area) * 1e6})
        max_owner = max(max_owner, ranked[0][0])
        groups.setdefault(WALL_NAME[ranked[0][1]], []).append(dface)
    report['owner_max_dist_m'] = max_owner
    report['unassigned_faces'] = failures
    report['close_second_owner'] = close
    report['named_selections'] = dict((n, {'faces': len(f), 'area_m2': sum(float(x.Shape.Area) for x in f)})
                                      for n, f in groups.items())
    report['named_selections_v1'] = v1['named_selections']
    report['short_edges_lt_0p1mm'] = sorted(round(float(e.Length) * 1e6, 3) for e in main.Edges if e.Length < 1e-4)
    report['tiny_faces_lt_0p1mm2'] = sorted(round(float(f.Area) * 1e6, 5) for f in main.Faces if f.Area < 1e-7)
    save()
    expected = sorted(list(WALL_NAME.values()) + ['overset_j3'])
    if failures or sorted(groups) != expected:
        raise Exception('Face ownership incomplete: failures=%d found=%s' % (len(failures), sorted(groups)))
    for n in sorted(groups):
        members = List[IDocObject]()
        for f in groups[n]:
            members.Add(f)
        Group.Create(root, n, members)
    DocumentSave.Execute(OUT_SCDOC)
    report['scdoc'] = OUT_SCDOC
    report['scdoc_sha256'] = sha256(OUT_SCDOC)
    report['source_unchanged'] = sha256(SOURCE) == report['source_sha256_before']
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

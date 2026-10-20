# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Overset fluid CAD for the new model (AUVM.step, read only, NO pose/shape changes).
# Layout follows cfd/wetted_arm_rebuild/articulated_mesh_v2/build_overset_fluid_cad.py:
#   background = tank box  - (hull U base0)          names: inlet / outlet / tank_wall / wall_auv / wall_base0
#   j1         = box(l1 + 50 mm)              - l1     names: overset_j1 / wall_l1
#   j2         = box(l2 + 50 mm)              - l2     names: overset_j2 / wall_l2
#   j3         = box(link3 group + 50 mm)     - (link3 U pin4 U jaw1 U jaw2)
#                                                      names: overset_j3 / wall_link3 / wall_pin4 / wall_jaw1 / wall_jaw2
# Same-motion bodies are united first, then subtracted once. The fluid is imprinted with its own
# bodies so a merged face spanning two parts is split; each wall face is owned by multi-point
# sampling (max distance over samples). Every part is saved in its own NEW document.
import codecs
import json
import os
import traceback
from System.Collections.Generic import List
from System.IO import File
from System.Security.Cryptography import SHA256
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, PointUV
from SpaceClaim.Api.V242 import IDocObject, DesignBody

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_DIR = r'D:\work4\new_model_sim\05_overset_cad'
MANIFEST = os.path.join(OUT_DIR, 'overset_manifest.json')
TANK_LO = (-1.5605, -1.3296, -1.8008)
TANK_HI = (1.5601, 1.9136, 1.6404)
PADDING = 0.05
TOL_BOOL = float(os.environ.get('OVERSET_TOL_BOOL', '1e-7'))
TOL_OUTER = 1e-7
TOL_OWNER = 1e-5
N_SAMPLE = 6
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'hull': 'wall_auv', 'base0': 'wall_base0', 'l1': 'wall_l1', 'l2': 'wall_l2',
             'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
PARTS = [('background', ['hull', 'base0']), ('j1', ['l1']), ('j2', ['l2']),
         ('j3', ['link3', 'pin4', 'jaw1', 'jaw2'])]
report = {'status': 'started', 'source': SOURCE, 'boolean_tol_m': TOL_BOOL, 'owner_tol_m': TOL_OWNER,
          'component_padding_m': PADDING, 'tank_lo_m': TANK_LO, 'tank_hi_m': TANK_HI,
          'geometry_modified': False, 'parts': {}}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def save():
    stream = codecs.open(MANIFEST, 'w', 'utf-8')
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


def outer_side(face, lo, hi):
    fl, fh = bbox(face)
    sides = []
    for k in range(3):
        if fh[k] - fl[k] <= TOL_OUTER:
            for upper, value in ((False, lo[k]), (True, hi[k])):
                if abs(fl[k] - value) <= TOL_OUTER:
                    sides.append((k, upper))
    if len(sides) > 1:
        raise Exception('Degenerate outer face')
    return sides[0] if sides else None


def build_part(part_name, members, tools):
    part = {'members': members, 'valid_for_meshing': False}
    report['parts'][part_name] = part
    save()
    if part_name == 'background':
        lo, hi = list(TANK_LO), list(TANK_HI)
    else:
        boxes = [bbox(tools[n]) for n in members]
        lo = [min(b[0][k] for b in boxes) - PADDING for k in range(3)]
        hi = [max(b[1][k] for b in boxes) + PADDING for k in range(3)]
    part['box_lo_m'], part['box_hi_m'] = lo, hi
    doc = Document.Create()
    root = doc.MainPart
    created = list(BlockBody.Create(Point.Create(*lo), Point.Create(*hi),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    fluid = world_copy(created)
    created.Delete()
    v_box = float(fluid.Volume)
    part['box_volume_m3'] = v_box
    ordered = sorted(members, key=lambda n: -tools[n].Volume)
    material = tools[ordered[0]].Copy()
    unions = []
    for n in ordered[1:]:
        lst = List[Body]()
        lst.Add(tools[n].Copy())
        material.Unite(lst, TOL_BOOL)
        unions.append({'added': n, 'union_volume_m3': float(material.Volume)})
    part['unions'] = unions
    part['material_volume_m3'] = float(material.Volume)
    part['sum_member_volumes_m3'] = sum(float(tools[n].Volume) for n in members)
    save()
    lst = List[Body]()
    lst.Add(material.Copy())
    fluid.Subtract(lst, TOL_BOOL)
    pieces = list(fluid.SeparatePieces())
    outer, rows = [], []
    for p in pieces:
        pl, ph = bbox(p)
        touches = all(abs(pl[k] - lo[k]) <= TOL_OUTER and abs(ph[k] - hi[k]) <= TOL_OUTER for k in range(3))
        rows.append({'volume_m3': float(p.Volume), 'reaches_all_box_sides': touches})
        if touches:
            outer.append(p)
    part['pieces'] = sorted(rows, key=lambda r: -r['volume_m3'])
    if len(outer) != 1:
        raise Exception('Expected exactly one outer fluid piece, found %d' % len(outer))
    main = outer[0]
    faces0 = len(list(main.Faces))
    for n in ordered:
        main.Imprint(tools[n].Copy())
    part['imprint_faces'] = [faces0, len(list(main.Faces))]
    part['fluid'] = {'volume_m3': float(main.Volume), 'faces': len(list(main.Faces)),
                     'closed': bool(main.IsClosed), 'manifold': bool(main.IsManifold),
                     'box_minus_material_m3': v_box - float(material.Volume),
                     'sealed_cavities_m3': sum(r['volume_m3'] for r in rows if not r['reaches_all_box_sides'])}
    save()
    if not main.IsClosed or not main.IsManifold:
        raise Exception('Fluid not closed/manifold')
    body = DesignBody.Create(root, 'fluid_' + part_name, main)
    groups, failures, close = {}, [], []
    max_owner = 0.0
    for idx, dface in enumerate(list(body.Faces)):
        face = dface.Shape
        side = outer_side(face, lo, hi)
        if side is not None:
            if part_name != 'background':
                name = 'overset_' + part_name
            elif side[0] == 0:
                name = 'outlet' if side[1] else 'inlet'
            else:
                name = 'tank_wall'
            groups.setdefault(name, []).append(dface)
            continue
        pts = samples(face)
        worst = dict((n, max(dist(as_point(tools[n].ProjectPoint(p)), p) for p in pts)) for n in members)
        ranked = sorted((d, n) for n, d in worst.items())
        if ranked[0][0] > TOL_OWNER:
            failures.append({'face_index': idx, 'nearest': ranked[0][1], 'max_dist_m': ranked[0][0],
                             'area_mm2': float(face.Area) * 1e6})
            continue
        if len(ranked) > 1 and ranked[1][0] <= TOL_OWNER:
            close.append({'face_index': idx, 'owner': ranked[0][1], 'd_owner_m': ranked[0][0],
                          'other': ranked[1][1], 'd_other_m': ranked[1][0], 'area_mm2': float(face.Area) * 1e6})
        max_owner = max(max_owner, ranked[0][0])
        groups.setdefault(WALL_NAME[ranked[0][1]], []).append(dface)
    expected = [WALL_NAME[n] for n in members] + (['inlet', 'outlet', 'tank_wall'] if part_name == 'background'
                                                  else ['overset_' + part_name])
    part['owner_max_dist_m'] = max_owner
    part['unassigned_faces'] = failures
    part['close_second_owner'] = close
    part['named_selections'] = dict((n, {'faces': len(f), 'area_m2': sum(float(x.Shape.Area) for x in f)})
                                    for n, f in groups.items())
    part['expected_names'] = sorted(expected)
    part['wall_vs_part_area'] = dict((n, {'wall_m2': part['named_selections'].get(WALL_NAME[n], {}).get('area_m2', 0.0),
                                          'part_m2': float(tools[n].SurfaceArea)}) for n in members)
    save()
    if failures or sorted(groups) != sorted(expected):
        raise Exception('Face ownership incomplete: failures=%d found=%s' % (len(failures), sorted(groups)))
    if sum(len(f) for f in groups.values()) != len(list(body.Faces)):
        raise Exception('Face count did not reconcile')
    for n in sorted(groups):
        lst = List[IDocObject]()
        for f in groups[n]:
            lst.Add(f)
        Group.Create(root, n, lst)
    path = os.path.join(OUT_DIR, part_name + '_fluid.scdoc')
    if os.path.exists(path):
        raise Exception('Refusing to overwrite ' + path)
    DocumentSave.Execute(path)
    part['scdoc'] = path
    part['scdoc_sha256'] = sha256(path)
    part['valid_for_meshing'] = True
    save()


try:
    if os.path.exists(MANIFEST):
        raise Exception('Refusing to overwrite manifest')
    save()
    report['source_sha256_before'] = sha256(SOURCE)
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        n = name_for(float(shape.Volume))
        if n is None or n in tools:
            raise Exception('Unrecognised/duplicate body volume %.9f' % float(shape.Volume))
        if not shape.IsClosed or not shape.IsManifold:
            raise Exception('Source body not closed/manifold: ' + n)
        tools[n] = shape
    if sorted(tools) != sorted(WALL_NAME):
        raise Exception('Unexpected bodies %s' % sorted(tools))
    report['bodies'] = dict((n, {'volume_m3': float(s.Volume), 'area_m2': float(s.SurfaceArea),
                                 'bbox_m': bbox(s)}) for n, s in tools.items())
    save()
    for part_name, members in PARTS:
        try:
            build_part(part_name, members, tools)
        except Exception:
            report['parts'].setdefault(part_name, {})['valid_for_meshing'] = False
            report['parts'][part_name]['error'] = u(traceback.format_exc())
            save()
    report['source_sha256_after'] = sha256(SOURCE)
    report['source_unchanged'] = report['source_sha256_after'] == report['source_sha256_before']
    report['valid_for_meshing'] = report['source_unchanged'] and all(
        report['parts'].get(p, {}).get('valid_for_meshing') for p, _ in PARTS)
    report['status'] = 'ok' if report['valid_for_meshing'] else 'failed'
except Exception:
    report['status'] = 'error'
    report['error'] = u(traceback.format_exc())
finally:
    save()

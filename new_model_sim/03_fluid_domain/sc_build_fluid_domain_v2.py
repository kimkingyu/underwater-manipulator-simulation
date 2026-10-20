# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Water domain v2 for the new model (AUVM.step, read only).
# Change vs. the failed trial: the distal rigid group (link3, pin4, jaw1, jaw2) is translated
# +10 um along +Y on in-memory copies, which removes the l2/link3 line contact at J3 that makes
# the ACIS Boolean fail (see regularize_sweep.json / line_contact_check.json).
# Writes only OUT_SCDOC and OUT_JSON, in a NEW document; the STEP document is never saved.
#
# Frame (new model): +X = flow direction (inlet at x_min), +Y = up, Z = transverse.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector
from SpaceClaim.Api.V242 import IDocObject, DesignBody

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_DIR = r'D:\work4\new_model_sim\03_fluid_domain'
OUT_SCDOC = OUT_DIR + r'\fluid_domain_v2.scdoc'
OUT_JSON = OUT_DIR + r'\fluid_domain_v2.json'
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
TOL_BOOL = 1e-5
TOL_OUTER = 1e-7
TOL_FACE = 1e-4
SHIFT_M = (0.0, 1e-5, 0.0)
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'hull': 'wall_auv', 'base0': 'wall_base0', 'l1': 'wall_l1', 'l2': 'wall_l2',
             'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
manifest = {'status': 'started', 'source': SOURCE, 'tank_lo_m': LO, 'tank_hi_m': HI,
            'boolean_tol_m': TOL_BOOL, 'face_owner_tol_m': TOL_FACE,
            'regularisation': {'bodies': DISTAL, 'translation_m': SHIFT_M,
                               'reason': 'l2/link3 line contact at J3 (y=-0.1226, z=-0.3412) breaks Boolean'},
            'frame': '+X flow (inlet at x_min), +Y up'}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def save_manifest():
    stream = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        stream.write(json.dumps(manifest, ensure_ascii=False, indent=2))
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


def bbox(obj):
    b = obj.GetBoundingBox(Matrix.Identity)
    return ([b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z], [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z])


def as_point(hit):
    return getattr(hit, 'Point', hit)


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


def mm(values):
    return [round(v * 1000.0, 3) for v in values]


try:
    save_manifest()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        name = name_for(float(shape.Volume))
        if name is None or name in tools:
            raise Exception('Unrecognised or duplicate body volume %.9f' % float(shape.Volume))
        tools[name] = shape
    if sorted(tools) != sorted(WALL_NAME):
        raise Exception('Expected hull + 7 arm bodies, got %s' % sorted(tools))
    move = Matrix.CreateTranslation(Vector.Create(*SHIFT_M))
    for name in DISTAL:
        tools[name].Transform(move)
    manifest['tools'] = dict((n, {'volume_m3': float(s.Volume), 'area_m2': float(s.SurfaceArea),
                                  'faces': len(list(s.Faces)), 'shifted': n in DISTAL})
                             for n, s in tools.items())
    save_manifest()

    # New document: the opened STEP document is left unsaved and untouched.
    doc = Document.Create()
    root = doc.MainPart
    created = list(BlockBody.Create(Point.Create(*LO), Point.Create(*HI),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    fluid = world_copy(created)
    created.Delete()
    v_box = float(fluid.Volume)
    manifest['box_volume_m3'] = v_box
    ledger = []
    v_prev = v_box
    for name in sorted(tools, key=lambda n: -tools[n].Volume):
        lst = List[Body]()
        lst.Add(tools[name].Copy())
        fluid.Subtract(lst, TOL_BOOL)
        v_now = float(fluid.Volume)
        ledger.append({'tool': name, 'tool_volume_m3': float(tools[name].Volume), 'cut_m3': v_prev - v_now})
        v_prev = v_now
        manifest['subtraction_ledger'] = ledger
        save_manifest()

    pieces = list(fluid.SeparatePieces())
    outer = []
    rows = []
    for p in pieces:
        lo, hi = bbox(p)
        touches = all(abs(lo[k] - LO[k]) <= TOL_OUTER and abs(hi[k] - HI[k]) <= TOL_OUTER for k in range(3))
        rows.append({'volume_m3': float(p.Volume), 'reaches_all_box_sides': touches,
                     'bbox_mm': mm(lo) + mm(hi)})
        if touches:
            outer.append(p)
    manifest['pieces'] = sorted(rows, key=lambda r: -r['volume_m3'])
    if len(outer) != 1:
        raise Exception('Expected exactly one outer water piece, found %d' % len(outer))
    main = outer[0]
    manifest['water'] = {'volume_m3': float(main.Volume), 'faces': len(list(main.Faces)),
                         'edges': len(list(main.Edges)), 'closed': bool(main.IsClosed),
                         'manifold': bool(main.IsManifold)}
    if not main.IsClosed or not main.IsManifold:
        raise Exception('Water body not closed/manifold')
    sum_tools = sum(float(s.Volume) for s in tools.values())
    manifest['volume_check'] = {'box_minus_tools_m3': v_box - sum_tools,
                                'sealed_cavities_m3': sum(r['volume_m3'] for r in rows if not r['reaches_all_box_sides']),
                                'water_minus_expected_m3': float(main.Volume) - (v_box - sum_tools)}
    save_manifest()

    # Face grouping on the document body.
    water = DesignBody.Create(root, 'water', main)
    groups = {'inlet': [], 'outlet': [], 'tank_wall': []}
    for n in WALL_NAME.values():
        groups[n] = []
    unassigned, ambiguous, tiny = [], [], []
    for idx, dface in enumerate(list(water.Faces)):
        face = dface.Shape
        lo, hi = bbox(face)
        if hi[0] - lo[0] <= TOL_OUTER and abs(lo[0] - LO[0]) <= TOL_OUTER:
            groups['inlet'].append(dface)
            continue
        if hi[0] - lo[0] <= TOL_OUTER and abs(lo[0] - HI[0]) <= TOL_OUTER:
            groups['outlet'].append(dface)
            continue
        if any(hi[k] - lo[k] <= TOL_OUTER and (abs(lo[k] - LO[k]) <= TOL_OUTER or abs(lo[k] - HI[k]) <= TOL_OUTER)
               for k in (1, 2)):
            groups['tank_wall'].append(dface)
            continue
        centre = Point.Create((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
        probe = as_point(face.ProjectPoint(centre))
        dists = sorted((dist(as_point(s.ProjectPoint(probe)), probe), n) for n, s in tools.items())
        area = float(face.Area)
        if area < 1e-6:
            tiny.append({'face_index': idx, 'area_mm2': area * 1e6, 'owner': dists[0][1],
                         'centre_mm': mm([probe.X, probe.Y, probe.Z])})
        if dists[0][0] >= TOL_FACE:
            unassigned.append({'face_index': idx, 'nearest': dists[0][1], 'distance_m': dists[0][0],
                               'centre_mm': mm([probe.X, probe.Y, probe.Z])})
            continue
        if dists[1][0] < TOL_FACE:
            ambiguous.append({'face_index': idx, 'chosen': dists[0][1], 'd_chosen_m': dists[0][0],
                              'other': dists[1][1], 'd_other_m': dists[1][0], 'area_mm2': area * 1e6,
                              'centre_mm': mm([probe.X, probe.Y, probe.Z])})
        groups[WALL_NAME[dists[0][1]]].append(dface)
    manifest['unassigned_faces'] = unassigned
    manifest['ambiguous_faces'] = ambiguous
    manifest['tiny_faces_lt_1mm2'] = tiny
    short = []
    for e in main.Edges:
        if e.Length < 1e-4:
            lo, hi = bbox(e)
            short.append({'length_um': float(e.Length) * 1e6, 'centre_mm': mm([(lo[k] + hi[k]) / 2 for k in range(3)])})
    manifest['short_edges_lt_0p1mm'] = sorted(short, key=lambda r: r['length_um'])
    manifest['named_selections'] = {}
    for name in sorted(groups):
        faces = groups[name]
        manifest['named_selections'][name] = {'faces': len(faces),
                                              'area_m2': sum(float(f.Shape.Area) for f in faces)}
        if faces:
            members = List[IDocObject]()
            for f in faces:
                members.Add(f)
            Group.Create(root, name, members)
    manifest['faces_grouped'] = sum(len(v) for v in groups.values())
    manifest['wetted_vs_part_area'] = dict(
        (n, {'wall_area_m2': manifest['named_selections'][WALL_NAME[n]]['area_m2'],
             'part_area_m2': float(tools[n].SurfaceArea)}) for n in tools)
    save_manifest()
    if unassigned:
        raise Exception('%d faces could not be assigned' % len(unassigned))
    DocumentSave.Execute(OUT_SCDOC)
    manifest['saved_scdoc'] = OUT_SCDOC
    manifest['status'] = 'ok'
except Exception:
    manifest['status'] = 'error'
    manifest['traceback'] = u(traceback.format_exc())
finally:
    save_manifest()

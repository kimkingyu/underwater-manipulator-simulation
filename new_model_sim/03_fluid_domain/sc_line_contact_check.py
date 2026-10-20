# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only check of the l2/link3 line contact near (x~0, y=-0.1226, z=-0.3412):
#  1. which quadrant around the line each body occupies (ContainsPoint probes);
#  2. for each candidate shift of the distal group (link3, pin4, jaw1, jaw2), build the fluid and
#     test whether the l2/link3 near-contact becomes a solid neck (good) or a thin fluid slit (bad):
#     closest points between l2-owned and link3-owned fluid faces near the line, midpoint inside fluid?
#  3. small-feature census of the fluid (tiny faces, short edges) and the enclosed pockets.
#  4. Body members related to volume / mass properties.
# AUVM.step is only read; saves nothing but the json.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\line_contact_check.json'
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
TOL = 1e-5
Y0, Z0 = -0.1226, -0.3412
REGION = (-0.06, Y0 - 0.006, Z0 - 0.006, 0.06, Y0 + 0.006, Z0 + 0.006)
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
CANDIDATES = [('-Y', (0.0, -1.0, 0.0), 5e-5), ('+Y', (0.0, 1.0, 0.0), 1e-5), ('-Z', (0.0, 0.0, -1.0), 1e-5)]
report = {'status': 'started', 'line_y': Y0, 'line_z': Z0, 'region': REGION}


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


def as_point(hit):
    return getattr(hit, 'Point', hit)


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


def bbox(face):
    b = face.GetBoundingBox(Matrix.Identity)
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z, b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def in_region(box):
    return all(box[i] <= REGION[i + 3] and box[i + 3] >= REGION[i] for i in range(3))


def owner(face, tool_map):
    b = face.GetBoundingBox(Matrix.Identity)
    p = as_point(face.ProjectPoint(b.Center))
    best = (1e9, None)
    for name, shape in tool_map.items():
        d = dist(as_point(shape.ProjectPoint(p)), p)
        if d < best[0]:
            best = (d, name)
    return best


def face_info(face):
    geom = face.Geometry
    row = {'type': geom.GetType().Name, 'area_mm2': float(face.Area) * 1e6,
           'bbox_mm': [round(v * 1000, 3) for v in bbox(face)]}
    if row['type'] == 'Plane':
        n = geom.Frame.DirZ
        if face.IsReversed:
            n = -n
        row['normal'] = [round(n.X, 6), round(n.Y, 6), round(n.Z, 6)]
    return row


def build(tool_map):
    created = list(BlockBody.Create(Point.Create(*LO), Point.Create(*HI),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    fluid = world_copy(created)
    created.Delete()
    for name in sorted(tool_map, key=lambda n: -tool_map[n].Volume):
        lst = List[Body]()
        lst.Add(tool_map[name].Copy())
        fluid.Subtract(lst, TOL)
    return sorted(list(fluid.SeparatePieces()), key=lambda p: -p.Volume)


try:
    save()
    report['body_members'] = sorted(n for n in dir(Body) if any(k in n.lower() for k in ('volume', 'mass', 'area', 'accura', 'propert')))
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape

    # 1. Quadrants around the line at several x stations.
    quads = []
    for x in (-0.02, -0.01, -0.002, 0.005, 0.01, 0.02):
        for dy in (-5e-4, 5e-4):
            for dz in (-5e-4, 5e-4):
                p = Point.Create(x, Y0 + dy, Z0 + dz)
                inside = [n for n in ('l2', 'link3') if tools[n].ContainsPoint(p)]
                quads.append({'x': x, 'dy': dy, 'dz': dz, 'inside': inside})
    report['quadrants'] = quads
    report['faces_near_line'] = dict((n, [face_info(f) for f in tools[n].Faces if in_region(bbox(f))])
                                     for n in ('l2', 'link3'))
    save()

    # 2./3. Candidate builds.
    results = []
    for label, vec, delta in CANDIDATES:
        move = Matrix.CreateTranslation(Vector.Create(vec[0], vec[1], vec[2]) * delta)
        moved = dict(tools)
        for n in DISTAL:
            s = tools[n].Copy()
            s.Transform(move)
            moved[n] = s
        row = {'dir': label, 'delta_m': delta}
        try:
            pieces = build(moved)
        except Exception as exc:
            row['build'] = 'FAIL: ' + u(exc)
            results.append(row)
            report['candidates'] = results
            save()
            continue
        main = pieces[0]
        row['pieces'] = [{'volume_m3': float(p.Volume), 'bbox_mm': [round(v * 1000, 1) for v in
                          [p.GetBoundingBox(Matrix.Identity).MinCorner.X, p.GetBoundingBox(Matrix.Identity).MinCorner.Y,
                           p.GetBoundingBox(Matrix.Identity).MinCorner.Z, p.GetBoundingBox(Matrix.Identity).MaxCorner.X,
                           p.GetBoundingBox(Matrix.Identity).MaxCorner.Y, p.GetBoundingBox(Matrix.Identity).MaxCorner.Z]]}
                         for p in pieces]
        near = []
        for f in main.Faces:
            if not in_region(bbox(f)):
                continue
            d, who = owner(f, moved)
            near.append((who, d, f))
        row['near_faces'] = [dict(face_info(f), owner=who, owner_dist_m=d) for who, d, f in near]
        pairs = []
        for wa, da, fa in near:
            if wa != 'l2':
                continue
            for wb, db_, fb in near:
                if wb != 'link3':
                    continue
                sep = fa.GetClosestSeparation(fb)
                d = float(sep.Distance)
                if d > 1e-3:
                    continue
                pa, pb = sep.PointA, sep.PointB
                mid = Point.Create((pa.X + pb.X) / 2, (pa.Y + pb.Y) / 2, (pa.Z + pb.Z) / 2)
                pairs.append({'distance_m': d, 'mid_mm': [round(mid.X * 1000, 4), round(mid.Y * 1000, 4), round(mid.Z * 1000, 4)],
                              'mid_in_fluid': bool(main.ContainsPoint(mid)) if d > 1e-9 else None})
        row['l2_link3_close_pairs'] = pairs
        row['tiny_faces_lt_1mm2'] = sum(1 for f in main.Faces if f.Area < 1e-6)
        row['short_edges_lt_0p1mm'] = sorted(round(float(e.Length) * 1e6, 3) for e in main.Edges if e.Length < 1e-4)
        row['main'] = {'volume_m3': float(main.Volume), 'faces': len(list(main.Faces)),
                       'closed': bool(main.IsClosed), 'manifold': bool(main.IsManifold)}
        results.append(row)
        report['candidates'] = results
        save()
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

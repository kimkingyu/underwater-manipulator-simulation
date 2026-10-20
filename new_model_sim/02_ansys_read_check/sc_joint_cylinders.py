# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: cylindrical faces (bores, shafts, pins) of the small arm bodies, in world coordinates.
# Env: NM_INPUT (CAD file), NM_OUTPUT (json). The source file is never saved.
#
# Occurrence bodies expose no faces, so faces are read from the master body and mapped to world
# space with the occurrence transform. The transform direction is chosen by checking that the
# mapped master bounding box reproduces the occurrence's world bounding box.
import json
import codecs
import traceback
import System

src = System.Environment.GetEnvironmentVariable('NM_INPUT')
out = System.Environment.GetEnvironmentVariable('NM_OUTPUT')
result = {'input': src, 'status': 'started', 'bodies': [], 'errors': []}


def write():
    stream = codecs.open(out, 'w', 'utf-8')
    try:
        stream.write(json.dumps(result, ensure_ascii=True, indent=2))
    finally:
        stream.close()


def text(value):
    try:
        return unicode(value)
    except Exception:
        return u'?'


def xyz(p):
    return [p.X, p.Y, p.Z]


def walk(part, path):
    rows = []
    for body in part.Bodies:
        rows.append((body, path))
    for comp in part.Components:
        label = text(comp.Name)
        if not label:
            try:
                label = text(comp.Template.Name)
            except Exception:
                label = u''
        rows += walk(comp.Content, path + [label or u'(unnamed)'])
    return rows


def box(shape):
    b = shape.GetBoundingBox(Matrix.Identity)
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z,
            b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def mapped_box(b, m):
    pts = []
    for x in (b[0], b[3]):
        for y in (b[1], b[4]):
            for z in (b[2], b[5]):
                pts.append(m * Point.Create(x, y, z))
    return [min([p.X for p in pts]), min([p.Y for p in pts]), min([p.Z for p in pts]),
            max([p.X for p in pts]), max([p.Y for p in pts]), max([p.Z for p in pts])]


def box_error(a, b):
    return max([abs(a[i] - b[i]) for i in range(6)])


def cylinder_row(geom, area, to_world):
    axis = geom.Axis
    origin = axis.Origin
    direction = axis.Direction
    if to_world is not None:
        origin = to_world * origin
        direction = to_world * direction
    return {'origin_m': xyz(origin), 'direction': [direction.X, direction.Y, direction.Z],
            'radius_m': float(geom.Radius), 'area_m2': float(area)}


try:
    write()
    DocumentOpen.Execute(src)
    root = GetRootPart()
    for body, path in walk(root, [text(root.Name)]):
        world = body.Shape
        volume = float(world.Volume)
        if volume > 0.01:
            continue  # the hull; only the arm parts are needed
        row = {'owner': path, 'name': text(body.Name), 'volume_m3': volume,
               'bbox_m': box(world), 'cylinders': [], 'face_types': {}}
        master = body.Master.Shape
        to_world = None
        try:
            t = body.TransformToMaster
            scored = []
            for name, m in (('inverse', t.Inverse), ('direct', t)):
                scored.append((box_error(mapped_box(box(master), m), row['bbox_m']), name, m))
            scored.sort(key=lambda s: s[0])
            row['transform_used'] = scored[0][1]
            row['transform_bbox_error_m'] = scored[0][0]
            row['transform_other_error_m'] = scored[1][0]
            # Curved-body boxes are not always tight, so allow 0.5 mm; the wrong direction is off by cm.
            if scored[0][0] < 5e-4:
                to_world = scored[0][2]
        except Exception as exc:
            row['transform_error'] = text(exc)
        if to_world is not None:
            row['method'] = 'master_faces_mapped'
            faces = [(f.Geometry, f.Area) for f in master.Faces]
        else:
            # Fallback: occurrence faces, whose shapes are already in occurrence (world) space.
            row['method'] = 'occurrence_faces_unverified'
            faces = [(f.Shape.Geometry, f.Shape.Area) for f in body.Faces]
        for geom, area in faces:
            try:
                kind = geom.GetType().Name
                row['face_types'][kind] = row['face_types'].get(kind, 0) + 1
                if kind == 'Cylinder':
                    row['cylinders'].append(cylinder_row(geom, area, to_world))
            except Exception as exc:
                result['errors'].append(text(exc))
        result['bodies'].append(row)
    result['status'] = 'ok'
except Exception:
    result['status'] = 'error'
    result['error'] = traceback.format_exc()
finally:
    write()

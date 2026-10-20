# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Independent re-check of the four overset fluid scdoc files in a fresh session (saves nothing):
# body count / closed / manifold / volume vs manifest, named selections vs manifest, every face in
# exactly one group, and each wall face re-owned by multi-point sampling against the source parts.
import codecs
import json
import os
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, PointUV

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
D = r'D:\work4\new_model_sim\05_overset_cad'
MANIFEST = os.path.join(D, 'overset_manifest.json')
OUT_JSON = os.path.join(D, 'overset_verify.json')
TOL = 1e-5
N = 6
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'hull': 'wall_auv', 'base0': 'wall_base0', 'l1': 'wall_l1', 'l2': 'wall_l2',
             'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
report = {'status': 'started', 'parts': {}}


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


def samples(face):
    box = face.BoxUV
    ru, rv = box.RangeU, box.RangeV
    pts = []
    for i in range(N):
        for j in range(N):
            uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N,
                                rv.Start + (rv.End - rv.Start) * (j + 0.5) / N)
            if face.ContainsParam(uv):
                pts.append(face.Geometry.Evaluate(uv).Point)
    if not pts:
        b = face.GetBoundingBox(Matrix.Identity)
        pts.append(as_point(face.ProjectPoint(b.Center)))
    return pts


try:
    if os.path.exists(OUT_JSON):
        raise Exception('Refusing to overwrite verify report')
    save()
    stream = codecs.open(MANIFEST, 'r', 'utf-8')
    manifest = json.load(stream)
    stream.close()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    inv = dict((w, n) for n, w in WALL_NAME.items())
    for part_name in ('background', 'j1', 'j2', 'j3'):
        m = manifest['parts'][part_name]
        row = {}
        report['parts'][part_name] = row
        DocumentOpen.Execute(m['scdoc'])
        root = GetRootPart()
        bodies = list(root.GetAllBodies())
        row['bodies'] = len(bodies)
        b = bodies[0]
        row['volume_m3'] = float(b.Shape.Volume)
        row['volume_matches_manifest'] = abs(row['volume_m3'] - m['fluid']['volume_m3']) < 1e-12
        row['closed'] = bool(b.Shape.IsClosed)
        row['manifold'] = bool(b.Shape.IsManifold)
        row['faces'] = len(list(b.Faces))
        face_group = {}
        groups = {}
        for g in root.Groups:
            fs = [x for x in g.Members if hasattr(x, 'Shape') and x.Shape.GetType().Name == 'Face']
            groups[u(g.Name)] = {'faces': len(fs), 'area_m2': sum(float(x.Shape.Area) for x in fs)}
            for x in fs:
                face_group.setdefault(x.Shape.GetHashCode(), []).append(u(g.Name))
        row['groups'] = groups
        mism = []
        for n, v in m['named_selections'].items():
            g = groups.get(n)
            if g is None or g['faces'] != v['faces'] or abs(g['area_m2'] - v['area_m2']) > 1e-12:
                mism.append(n)
        if sorted(groups) != sorted(m['named_selections']):
            mism.append('name set differs')
        row['group_mismatch'] = mism
        problems = []
        not_one = 0
        max_owner = 0.0
        for idx, dface in enumerate(list(b.Faces)):
            names = face_group.get(dface.Shape.GetHashCode(), [])
            if len(names) != 1:
                not_one += 1
                continue
            if not names[0].startswith('wall_'):
                continue
            pts = samples(dface.Shape)
            worst = dict((n, max(dist(as_point(tools[n].ProjectPoint(p)), p) for p in pts)) for n in m['members'])
            owner = min(worst, key=lambda n: worst[n])
            max_owner = max(max_owner, worst[owner])
            if inv[names[0]] != owner or worst[owner] > TOL:
                problems.append({'face_index': idx, 'group': names[0], 'owner': owner, 'd_m': worst[owner]})
        row['faces_not_in_one_group'] = not_one
        row['owner_max_dist_m'] = max_owner
        row['owner_problems'] = problems
        row['ok'] = (row['bodies'] == 1 and row['closed'] and row['manifold'] and row['volume_matches_manifest']
                     and not mism and not_one == 0 and not problems)
        save()
    report['all_ok'] = all(r['ok'] for r in report['parts'].values())
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

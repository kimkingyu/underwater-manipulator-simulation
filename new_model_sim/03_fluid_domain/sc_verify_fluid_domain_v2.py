# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Independent re-check of fluid_domain_v2.scdoc in a fresh session (nothing is saved):
#  - body count, closed/manifold, volume, face count
#  - named selections: names, face counts, areas
#  - multi-sample owner check: for every wall face, sample points over the trimmed face and take,
#    for each part, the MAX distance over samples; the true owner must have max < TOL and it must be
#    the group the face is in. Parts are rebuilt from AUVM.step + the same +10 um shift.
import codecs
import json
import traceback
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector, PointUV

SCDOC = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2.scdoc'
SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v2_verify.json'
TOL = 1e-5
N = 6
SHIFT_M = (0.0, 1e-5, 0.0)
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'hull': 'wall_auv', 'base0': 'wall_base0', 'l1': 'wall_l1', 'l2': 'wall_l2',
             'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
report = {'status': 'started', 'scdoc': SCDOC, 'samples_per_dir': N, 'tol_m': TOL}


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
    save()
    # Parts first (STEP document, never saved).
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    move = Matrix.CreateTranslation(Vector.Create(*SHIFT_M))
    for n in DISTAL:
        tools[n].Transform(move)

    DocumentOpen.Execute(SCDOC)
    root = GetRootPart()
    bodies = list(root.GetAllBodies())
    report['bodies'] = [{'name': u(b.Name), 'volume_m3': float(b.Shape.Volume), 'faces': len(list(b.Faces)),
                         'closed': bool(b.Shape.IsClosed), 'manifold': bool(b.Shape.IsManifold)} for b in bodies]
    report['root_members_with_group'] = [n for n in dir(root) if 'group' in n.lower()]
    save()
    groups = list(root.Groups)
    face_group = {}
    report['groups'] = {}
    for g in groups:
        members = list(g.Members)
        faces = [m for m in members if hasattr(m, 'Shape') and m.Shape.GetType().Name == 'Face']
        report['groups'][u(g.Name)] = {'members': len(members), 'faces': len(faces),
                                       'area_m2': sum(float(f.Shape.Area) for f in faces)}
        for f in faces:
            face_group.setdefault(f.Shape.GetHashCode(), []).append(u(g.Name))
    save()

    water = bodies[0]
    checks = []
    bad = []
    multi = 0
    ungrouped = 0
    for idx, dface in enumerate(list(water.Faces)):
        face = dface.Shape
        names = face_group.get(face.GetHashCode(), [])
        if len(names) != 1:
            if len(names) == 0:
                ungrouped += 1
            else:
                multi += 1
            bad.append({'face_index': idx, 'groups': names, 'problem': 'not in exactly one group'})
            continue
        gname = names[0]
        if not gname.startswith('wall_'):
            continue
        pts = samples(face)
        worst = {}
        for n, s in tools.items():
            worst[n] = max(dist(as_point(s.ProjectPoint(p)), p) for p in pts)
        owner = min(worst, key=lambda n: worst[n])
        row = {'face_index': idx, 'group': gname, 'samples': len(pts), 'owner': owner,
               'owner_max_dist_m': worst[owner], 'area_mm2': float(face.Area) * 1e6}
        others = sorted((d, n) for n, d in worst.items() if n != owner)
        row['runner_up'] = others[0][1]
        row['runner_up_max_dist_m'] = others[0][0]
        checks.append(row)
        if WALL_NAME[owner] != gname or worst[owner] > TOL:
            bad.append(dict(row, problem='owner mismatch or off-surface'))
    report['wall_faces_checked'] = len(checks)
    report['faces_not_in_exactly_one_group'] = {'none': ungrouped, 'several': multi}
    report['max_owner_dist_m'] = max(r['owner_max_dist_m'] for r in checks) if checks else None
    report['close_runner_up'] = [r for r in checks if r['runner_up_max_dist_m'] < TOL]
    report['problems'] = bad
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

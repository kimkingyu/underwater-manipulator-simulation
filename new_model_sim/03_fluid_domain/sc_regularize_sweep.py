# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only regularisation sweep for the l2/link3 LINE contact (z=-0.3412, y=-0.1226, along X).
# The distal rigid group (link3, pin4, jaw1, jaw2) is translated by a small delta along +-Y / +-Z
# on in-memory copies only; AUVM.step is just read. For every case the l2-link3 relation is
# recorded (distance, collision class, overlap volume, pairwise Unite); for the smallest delta per
# direction whose Unite works, the full box subtraction is run and the fluid body is checked.
# A tolerant Boolean on the unmodified geometry is tried last. Saves nothing but the json.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, Vector

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\regularize_sweep.json'
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
TOL = 1e-5
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
DIRS = [('+Y', (0.0, 1.0, 0.0)), ('-Y', (0.0, -1.0, 0.0)), ('+Z', (0.0, 0.0, 1.0)), ('-Z', (0.0, 0.0, -1.0))]
DELTAS = (1e-5, 2e-5, 5e-5, 1e-4)
report = {'status': 'started', 'source': SOURCE, 'distal_group': DISTAL, 'boolean_tol_m': TOL}


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


def relation(a, b):
    row = {}
    try:
        row['distance_m'] = float(a.GetClosestSeparation(b).Distance)
    except Exception as exc:
        row['distance_error'] = u(exc)
    try:
        row['collision'] = u(a.GetCollision(b))
    except Exception as exc:
        row['collision_error'] = u(exc)
    for op in ('Intersect', 'Unite'):
        try:
            base = a.Copy()
            lst = List[Body]()
            lst.Add(b.Copy())
            getattr(base, op)(lst, TOL)
            row[op] = float(base.Volume)
            if op == 'Unite':
                row['unite_closed'] = bool(base.IsClosed)
                row['unite_manifold'] = bool(base.IsManifold)
                row['unite_pieces'] = len(list(base.SeparatePieces()))
        except Exception as exc:
            row[op] = 'FAIL: ' + u(exc)
    return row


def full_subtract(box, tool_map, tol):
    fluid = box.Copy()
    v_box = float(fluid.Volume)
    out = {'tol_m': tol, 'order': []}
    for name in sorted(tool_map, key=lambda n: -tool_map[n].Volume):
        lst = List[Body]()
        lst.Add(tool_map[name].Copy())
        try:
            fluid.Subtract(lst, tol)
        except Exception as exc:
            out['result'] = 'failed at %s: %s' % (name, u(exc))
            return out
        out['order'].append(name)
    pieces = sorted(list(fluid.SeparatePieces()), key=lambda p: -p.Volume)
    main = pieces[0]
    v_tools = sum(float(s.Volume) for s in tool_map.values())
    out.update({'result': 'ok', 'pieces_m3': [float(p.Volume) for p in pieces],
                'main_closed': bool(main.IsClosed), 'main_manifold': bool(main.IsManifold),
                'main_faces': len(list(main.Faces)), 'box_m3': v_box,
                'expected_m3': v_box - v_tools, 'main_minus_expected_m3': float(main.Volume) - (v_box - v_tools)})
    return out


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        tools[name_for(float(shape.Volume))] = shape
    if sorted(tools) != sorted(['hull'] + [n for _, n in ARM_BY_VOLUME]):
        raise Exception('unexpected bodies: %s' % sorted(tools))
    created = list(BlockBody.Create(Point.Create(*LO), Point.Create(*HI),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    box = world_copy(created)
    created.Delete()
    report['baseline_l2_link3'] = relation(tools['l2'], tools['link3'])
    save()
    cases = []
    for label, vec in DIRS:
        solved = False
        for delta in DELTAS:
            move = Matrix.CreateTranslation(Vector.Create(vec[0], vec[1], vec[2]) * delta)
            moved = dict(tools)
            for n in DISTAL:
                s = tools[n].Copy()
                s.Transform(move)
                moved[n] = s
            row = {'dir': label, 'delta_m': delta}
            row.update(relation(tools['l2'], moved['link3']))
            if not solved and not u(row.get('Unite')).startswith('FAIL'):
                row['full'] = full_subtract(box, moved, TOL)
                solved = row['full'].get('result') == 'ok'
            cases.append(row)
            report['cases'] = cases
            save()
    report['tolerant_unmodified'] = [full_subtract(box, tools, t) for t in (1e-4,)]
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Diagnose the Boolean failure: subtract each tool ALONE from a fresh tank box, then try the arm
# chain in several orders and with/without pre-uniting touching parts. Writes after every test.
# Reads AUVM.step only; saves nothing except the json report.
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\boolean_diagnose.json'
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'tests': []}


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


def fresh_box():
    created = list(BlockBody.Create(Point.Create(*LO), Point.Create(*HI),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    shape = world_copy(created)
    created.Delete()
    return shape


def run_sequence(label, steps, tol):
    """steps: list of (name, shape). Subtract one by one from a fresh box; record each result."""
    entry = {'label': label, 'tolerance': tol, 'steps': []}
    report['tests'].append(entry)
    box = fresh_box()
    v_prev = float(box.Volume)
    for name, shape in steps:
        row = {'tool': name}
        try:
            lst = List[Body]()
            lst.Add(shape.Copy())
            box.Subtract(lst, tol)
            v_now = float(box.Volume)
            row['ok'] = True
            row['cut_m3'] = v_prev - v_now
            row['tool_volume_m3'] = float(shape.Volume)
            v_prev = v_now
        except Exception as exc:
            row['ok'] = False
            row['error'] = u(exc)
            entry['steps'].append(row)
            entry['result'] = 'failed at ' + name
            save()
            return False
        entry['steps'].append(row)
        save()
    entry['result'] = 'ok'
    entry['pieces'] = [float(p.Volume) for p in box.SeparatePieces()]
    save()
    return True


def unite(label, shapes, tol):
    base = shapes[0][1].Copy()
    try:
        lst = List[Body]()
        for _, s in shapes[1:]:
            lst.Add(s.Copy())
        base.Unite(lst, tol)
        report.setdefault('unions', []).append({'label': label, 'ok': True, 'volume_m3': float(base.Volume),
                                                 'sum_m3': sum(float(s.Volume) for _, s in shapes)})
        save()
        return base
    except Exception as exc:
        report.setdefault('unions', []).append({'label': label, 'ok': False, 'error': u(exc)})
        save()
        return None


try:
    save()
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        name = name_for(float(shape.Volume))
        tools[name] = shape
        report.setdefault('tool_checks', []).append({
            'name': name, 'volume_m3': float(shape.Volume), 'closed': bool(shape.IsClosed),
            'manifold': bool(shape.IsManifold), 'faces': len(list(shape.Faces))})
    save()
    order = ['hull', 'base0', 'l1', 'l2', 'link3', 'pin4', 'jaw1', 'jaw2']
    # 1) Each tool alone, from a fresh box.
    for name in order:
        run_sequence('alone:' + name, [(name, tools[name])], 1e-5)
    # 2) Whole chain in base->tip order, then tip->base, at two tolerances.
    for tol in (1e-5, 1e-6):
        run_sequence('chain_base_to_tip', [(n, tools[n]) for n in order], tol)
        run_sequence('chain_tip_to_base', [(n, tools[n]) for n in reversed(order)], tol)
    # 3) Arm parts pre-united, then hull + arm union.
    arm = unite('arm_union', [(n, tools[n]) for n in order[1:]], 1e-5)
    if arm is not None:
        run_sequence('hull_then_arm_union', [('hull', tools['hull']), ('arm_union', arm)], 1e-5)
        allu = unite('all_union', [('arm_union', arm), ('hull', tools['hull'])], 1e-5)
        if allu is not None:
            run_sequence('single_union_tool', [('all_union', allu)], 1e-5)
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

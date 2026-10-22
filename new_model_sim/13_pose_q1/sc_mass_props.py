# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: mass properties of every AUVM.step body (Body.MassProperties of the world-frame copy, CAD
# axes, metres, unit density, so Mass = volume). Inertia tensor taken with GetInertiaTensor at the
# centroid AND at the world origin; the pair lets the caller verify the convention by the parallel-axis
# theorem (I_O - I_C must equal V*(|c|^2 E - c c^T) for an inertia tensor, or its negative for a
# second-moment matrix). Also the principal frame. Writes mass_props.json.
import codecs
import json
import traceback
from SpaceClaim.Api.V242.Geometry import Point, Frame, Direction, Line

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_JSON = r'D:\work4\new_model_sim\13_pose_q1\mass_props.json'
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
report = {'status': 'started', 'source': SOURCE, 'units': 'm, unit density (mass = volume m3, inertia m5)',
          'bodies': {}}


def u(v):
    try:
        return unicode(v)
    except Exception:
        return repr(v)


def save():
    s = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        s.write(json.dumps(report, ensure_ascii=False, indent=1))
    finally:
        s.close()


def world_copy(db):
    sh = db.GetMaster().Shape.Copy()
    sh.Transform(db.GetTransformToMaster().Inverse)
    return sh


def name_for(v):
    if v > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda r: abs(r[0] - v))
    return best[1] if abs(best[0] - v) / best[0] < 2e-3 else None


def mat3(m):
    """Matrix / 3x3 array-like to nested lists, trying the common SpaceClaim layouts."""
    for get in (lambda i, j: m[i, j], lambda i, j: m[i][j], lambda i, j: m.Item[i, j]):
        try:
            return [[float(get(i, j)) for j in range(3)] for i in range(3)]
        except Exception:
            continue
    raise Exception('unknown tensor layout %s %s' % (type(m), [n for n in dir(m) if not n.startswith('_')][:40]))


def vec(p):
    return [float(p.X), float(p.Y), float(p.Z)]


try:
    save()
    DocumentOpen.Execute(SOURCE)
    for db in GetRootPart().GetAllBodies():
        sh = world_copy(db)
        v_exact = float(sh.Volume)
        name = name_for(v_exact)
        mp = sh.MassProperties
        row = {'volume_exact_m3': v_exact, 'mass_unit_density': float(mp.Mass),
               'relative_accuracy': float(mp.RelativeAccuracy)}
        pa = mp.PrincipleAxes
        row['centre_m'] = vec(pa.Origin)          # principal frame origin = centre of mass
        row['principal_axes'] = [vec(pa.DirX), vec(pa.DirY), vec(pa.DirZ)]
        for tag, org in (('at_centre', Point.Create(*row['centre_m'])), ('at_origin', Point.Origin)):
            fr = Frame.Create(org, Direction.DirX, Direction.DirY)     # world axes
            try:
                row['tensor_' + tag] = mat3(mp.GetInertiaTensor(fr))
            except Exception as exc:
                row['tensor_' + tag + '_err'] = u(exc)[:400]
        try:
            fr = Frame.Create(Point.Create(*row['centre_m']), Direction.DirX, Direction.DirY)
            row['moment_x_at_centre'] = float(mp.GetMoment(Line.Create(fr.Origin, Direction.DirX)))
        except Exception as exc:
            row['moment_err'] = u(exc)[:300]
        report['bodies'][name] = row
        save()
    report['mp_members'] = [n for n in dir(mp) if not n.startswith('_')]
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only API probe: list SpaceClaim Modeler members for distance, contact and Boolean operations,
# with their overload signatures (IronPython __doc__). Opens no document and saves nothing but json.
import codecs
import json
import traceback

OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\sc_api_probe.json'
KEYS = ('separation', 'distance', 'collision', 'clash', 'interfer', 'unite', 'subtract', 'intersect',
        'imprint', 'fuse', 'offset', 'project', 'contain', 'tolerance', 'heal', 'repair', 'merge', 'split',
        'transform', 'copy')
report = {'status': 'started'}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def members(obj):
    rows = {}
    for name in dir(obj):
        if name.startswith('_') or not any(k in name.lower() for k in KEYS):
            continue
        try:
            rows[name] = u(getattr(obj, name).__doc__)[:800]
        except Exception as exc:
            rows[name] = 'doc error: ' + u(exc)
    return rows


try:
    import SpaceClaim.Api.V242.Modeler as M
    import SpaceClaim.Api.V242.Geometry as G
    from SpaceClaim.Api.V242.Modeler import Body, Face
    report['Body'] = members(Body)
    report['Face'] = members(Face)
    report['modeler_types'] = [n for n in dir(M) if any(k in n.lower() for k in KEYS + ('options',))]
    report['geometry_types'] = [n for n in dir(G) if any(k in n.lower() for k in KEYS)]
    for tname in ('Separation', 'Collision', 'BodyCollision'):
        for ns in (M, G):
            t = getattr(ns, tname, None)
            if t is not None:
                report['type_%s_%s' % (ns.__name__.split('.')[-1], tname)] = [
                    n for n in dir(t) if not n.startswith('_')]
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    stream = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        stream.close()

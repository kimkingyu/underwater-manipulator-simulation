# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only API probe for the Enclosure command (Prepare > Enclosure): members, option defaults,
# enum values and constructor signatures. Opens no document; writes only the json report.
import codecs
import json
import traceback

OUT_JSON = r'D:\work4\new_model_sim\03_fluid_domain\sc_enclosure_api_probe.json'
SKIP = set(['Equals', 'GetHashCode', 'GetType', 'MemberwiseClone', 'ReferenceEquals', 'ToString', 'Finalize'])
report = {'status': 'started'}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def describe(obj):
    rows = {}
    for name in dir(obj):
        if name.startswith('_') or name in SKIP:
            continue
        try:
            rows[name] = u(getattr(getattr(obj, name), '__doc__', ''))[:700]
        except Exception as exc:
            rows[name] = 'error: ' + u(exc)
    return rows


def instance_values(obj):
    vals = {}
    for name in dir(obj):
        if name.startswith('_') or name in SKIP:
            continue
        try:
            value = getattr(obj, name)
            if callable(value):
                continue
            vals[name] = u(value)
        except Exception as exc:
            vals[name] = 'error: ' + u(exc)
    return vals


try:
    report['Enclosure'] = describe(Enclosure)
    report['EnclosureOptions'] = describe(EnclosureOptions)
    try:
        report['EnclosureOptions_defaults'] = instance_values(EnclosureOptions())
    except Exception as exc:
        report['EnclosureOptions_ctor_error'] = u(exc)
    report['EnclosureType_values'] = [n for n in dir(EnclosureType) if not n.startswith('_') and n[0].isupper()
                                      and n not in SKIP and not callable(getattr(EnclosureType, n))]
    report['BoxEnclosureCushion_ctor'] = u(BoxEnclosureCushion.__new__.__doc__)
    report['BoxEnclosureCushion'] = describe(BoxEnclosureCushion)
    report['EnclosureResult'] = describe(EnclosureResult)
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

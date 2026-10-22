# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only probe: which tessellation / mass-property members exist on the modeler Body, Face and on
# DesignBody / DesignFace of AUVM.step. Writes probe_tess_api.json.
import codecs
import json
import traceback

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT = r'D:\work4\new_model_sim\13_pose_q1\probe_tess_api.json'
rep = {}


def save():
    s = codecs.open(OUT, 'w', 'utf-8')
    try:
        s.write(json.dumps(rep, indent=1))
    finally:
        s.close()


def members(o):
    return [n for n in dir(o) if not n.startswith('_')]


try:
    DocumentOpen.Execute(SOURCE)
    db = list(GetRootPart().GetAllBodies())[0]
    sh = db.GetMaster().Shape
    rep['DesignBody'] = members(db)
    rep['Body'] = members(sh)
    rep['Face'] = members(list(sh.Faces)[0])
    rep['DesignFace'] = members(list(db.Faces)[0])
    rep['DesignBody_master'] = members(db.GetMaster())
    hits = {}
    for key in ('DesignBody', 'Body', 'Face', 'DesignFace', 'DesignBody_master'):
        hits[key] = [n for n in rep[key] if any(s in n.lower() for s in ('tess', 'mesh', 'mass', 'inertia', 'centroid', 'moment'))]
    rep['hits'] = hits
    try:
        t = sh.GetTessellation(None, TessellationOptions.Default)
        rep['body_tess_none'] = members(t)
    except Exception as exc:
        rep['body_tess_none_err'] = str(exc)[:300]
    rep['status'] = 'ok'
except Exception:
    rep['status'] = 'error'
    rep['traceback'] = traceback.format_exc()
finally:
    save()

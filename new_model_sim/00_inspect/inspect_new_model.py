# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only inspection of the new SolidWorks model.
# Original SolidWorks files are never saved; only new files in OUT are written.
import json
import codecs
import traceback
import System

NEW = u'D:\\work4\\\u7b80\u5316\u6a21\u578b'
OLD = u'D:\\work4\\\u673a\u5668\u4eba\\L'
OUT = u'D:\\work4\\new_model_sim\\00_inspect'
PARTS = [u'AUV.SLDPRT', u'0.SLDPRT', u'3.SLDPRT', u'l1.SLDPRT', u'l2.SLDPRT',
         u'\u5939\u72ea1.SLDPRT', u'\u5939\u72ea2.SLDPRT', u'\u5939\u72ea4.SLDPRT',
         u'l3.SLDASM', u'bl.SLDASM']
result = {'new_folder': NEW, 'old_folder': OLD, 'stage': 'start',
          'assembly': None, 'files': [], 'errors': []}


def flush():
    with codecs.open(OUT + u'\\inspection.json', 'w', 'utf-8') as stream:
        stream.write(json.dumps(result, ensure_ascii=True, indent=2))


def box_of(shape):
    b = shape.GetBoundingBox(Matrix.Identity)
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z,
            b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def body_row(body, path):
    row = {'path': path, 'name': body.Name}
    try:
        shape = body.Shape
        row['volume_m3'] = float(shape.Volume)
        row['bbox_m'] = box_of(shape)
        faces = list(shape.Faces)
        row['faces'] = len(faces)
        row['area_m2'] = sum(float(f.Area) for f in faces)
        row['closed'] = bool(shape.IsClosed)
        row['manifold'] = bool(shape.IsManifold)
    except Exception as exc:
        row['shape_error'] = str(exc)
    return row


def walk(part, path, rows, comps):
    for body in part.Bodies:
        rows.append(body_row(body, path))
    for comp in part.Components:
        child = path + [comp.Name]
        info = {'path': child,
                'bodies': len(list(comp.Content.Bodies)),
                'subcomponents': len(list(comp.Content.Components))}
        try:
            info['template'] = comp.Template.Name
        except Exception:
            pass
        comps.append(info)
        walk(comp.Content, child, rows, comps)


def inspect(path):
    entry = {'file': path}
    try:
        DocumentOpen.Execute(path)
        root = GetRootPart()
        rows, comps = [], []
        walk(root, [root.Name], rows, comps)
        entry['bodies'] = rows
        entry['components'] = comps
        entry['body_count'] = len(rows)
        entry['total_volume_m3'] = sum(r.get('volume_m3', 0.0) for r in rows)
        boxes = [r['bbox_m'] for r in rows if 'bbox_m' in r]
        if boxes:
            entry['overall_bbox_m'] = [min(b[0] for b in boxes), min(b[1] for b in boxes),
                                       min(b[2] for b in boxes), max(b[3] for b in boxes),
                                       max(b[4] for b in boxes), max(b[5] for b in boxes)]
        entry['status'] = 'ok'
    except Exception:
        entry['status'] = 'error'
        entry['error'] = traceback.format_exc()
    return entry


try:
    # 1) Full new assembly first, in a clean session.
    result['stage'] = 'assembly'
    flush()
    assembly = inspect(NEW + u'\\AUVM.SLDASM')
    result['assembly'] = assembly
    flush()
    if assembly.get('status') == 'ok':
        for target, key in ((OUT + u'\\AUVM_working_copy.scdoc', 'working_copy_scdoc'),
                            (OUT + u'\\AUVM_preview.stl', 'preview_stl')):
            try:
                DocumentSave.Execute(target)
                result[key] = 'saved'
            except Exception:
                result[key] = traceback.format_exc()
            flush()
    # 2) Individual parts, new versus old.
    result['stage'] = 'parts'
    for name in PARTS:
        for folder, tag in ((NEW, 'new'), (OLD, 'old')):
            full = folder + u'\\' + name
            if not System.IO.File.Exists(full):
                result['files'].append({'file': full, 'status': 'missing', 'version': tag})
                continue
            entry = inspect(full)
            entry['version'] = tag
            entry['name'] = name
            result['files'].append(entry)
            flush()
    result['stage'] = 'complete'
except Exception:
    result['errors'].append(traceback.format_exc())
finally:
    flush()

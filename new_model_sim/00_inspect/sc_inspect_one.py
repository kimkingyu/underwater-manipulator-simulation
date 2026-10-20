# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Inspect ONE CAD file in a fresh SpaceClaim session. The source file is never saved.
# Environment inputs: NM_INPUT (CAD file), NM_OUTPUT (json), NM_SAVE (optional .scdoc copy).
#
# Bodies inside components are occurrences: their Shape is a transformed TrimmedSpace that
# gives world-space volume and bounding box but no faces or closed/manifold flags. Those are
# read from the master body, which is identical up to a rigid transform.
import json
import codecs
import traceback
import System

src = System.Environment.GetEnvironmentVariable('NM_INPUT')
out = System.Environment.GetEnvironmentVariable('NM_OUTPUT')
save = System.Environment.GetEnvironmentVariable('NM_SAVE')
result = {'input': src, 'status': 'started', 'bodies': [], 'components': [], 'warnings': []}


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


def box_list(b):
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z,
            b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def body_row(body, owner):
    row = {'name': text(body.Name), 'owner': owner}
    try:
        placed = body.Shape
        row['bbox_m'] = box_list(placed.GetBoundingBox(Matrix.Identity))
        row['volume_m3'] = float(placed.Volume)
    except Exception as exc:
        row['placed_error'] = text(exc)
    try:
        master = body.Master.Shape
        faces = list(master.Faces)
        row['faces'] = len(faces)
        row['area_m2'] = sum([float(f.Area) for f in faces])
        row['closed'] = bool(master.IsClosed)
        row['manifold'] = bool(master.IsManifold)
        row['master_volume_m3'] = float(master.Volume)
    except Exception as exc:
        row['master_error'] = text(exc)
    return row


def merge_boxes(boxes):
    lo = [min([b[i] for b in boxes]) for i in range(3)]
    hi = [max([b[i + 3] for b in boxes]) for i in range(3)]
    return lo + hi


def walk(part, path):
    rows = []
    for body in part.Bodies:
        rows.append(body_row(body, path))
    for comp in part.Components:
        template = u''
        try:
            template = text(comp.Template.Name)
        except Exception:
            pass
        # Imported components often have a blank Name; the part name lives on the Template.
        label = text(comp.Name) or template or u'(unnamed)'
        child = path + [label]
        try:
            sub = walk(comp.Content, child)
        except Exception as exc:
            result['warnings'].append(u'walk failed at %s: %s' % (u'/'.join(child), text(exc)))
            sub = []
        info = {'path': child, 'body_count': len(sub)}
        if template:
            info['template'] = template
        boxes = [r['bbox_m'] for r in sub if 'bbox_m' in r]
        if boxes:
            info['bbox_m'] = merge_boxes(boxes)
            info['volume_m3'] = sum([r.get('volume_m3', 0.0) for r in sub])
        result['components'].append(info)
        rows += sub
    return rows


try:
    write()
    DocumentOpen.Execute(src)
    root = GetRootPart()
    result['root_name'] = text(root.Name)
    result['bodies'] = walk(root, [text(root.Name)])
    result['body_count'] = len(result['bodies'])
    result['component_count'] = len(result['components'])
    try:
        result['get_all_bodies_count'] = len(list(root.GetAllBodies()))
    except Exception as exc:
        result['warnings'].append(u'GetAllBodies failed: ' + text(exc))
    boxes = [r['bbox_m'] for r in result['bodies'] if 'bbox_m' in r]
    if not result['bodies']:
        # A rejected or unsupported file leaves only the empty start document.
        result['status'] = 'failed_no_bodies'
    elif not boxes:
        result['status'] = 'failed_no_geometry'
    else:
        bb = merge_boxes(boxes)
        result['overall_bbox_m'] = bb
        result['overall_size_m'] = [bb[i + 3] - bb[i] for i in range(3)]
        result['total_volume_m3'] = sum([r.get('volume_m3', 0.0) for r in result['bodies']])
        result['unchecked_bodies'] = [r['name'] for r in result['bodies'] if 'closed' not in r]
        result['open_or_nonmanifold'] = [r['name'] for r in result['bodies']
                                         if 'closed' in r and not (r['closed'] and r['manifold'])]
        if result.get('get_all_bodies_count') not in (None, result['body_count']):
            result['warnings'].append('walk body count differs from GetAllBodies')
        result['status'] = 'ok'
        if save:
            try:
                DocumentSave.Execute(save)
                result['saved_working_copy'] = save
            except Exception:
                result['save_error'] = traceback.format_exc()
except Exception:
    result['status'] = 'error'
    result['error'] = traceback.format_exc()
finally:
    write()

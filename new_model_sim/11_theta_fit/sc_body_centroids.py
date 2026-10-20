# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Read-only: volume, volume centroid and bbox of every body in the STEP, world frame.
# Env: NM_INPUT (CAD file), NM_OUTPUT (json). Source file is never saved.
# Centroid: tries modeler mass-property APIs first; falls back to tessellation divergence theorem.
import json
import codecs
import traceback
import System

src = System.Environment.GetEnvironmentVariable('NM_INPUT')
out = System.Environment.GetEnvironmentVariable('NM_OUTPUT')
result = {'input': src, 'status': 'started', 'bodies': [], 'errors': [], 'api_probe': {}}


def write():
    s = codecs.open(out, 'w', 'utf-8')
    try:
        s.write(json.dumps(result, ensure_ascii=True, indent=2))
    finally:
        s.close()


def text(v):
    try:
        return unicode(v)
    except Exception:
        return u'?'


def walk(part, path):
    rows = []
    for b in part.Bodies:
        rows.append((b, path))
    for c in part.Components:
        label = text(c.Name)
        if not label:
            try:
                label = text(c.Template.Name)
            except Exception:
                label = u''
        rows += walk(c.Content, path + [label or u'(unnamed)'])
    return rows


def box(shape):
    b = shape.GetBoundingBox(Matrix.Identity)
    return [b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z, b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z]


def centroid_tess(shape):
    """Divergence theorem on the face tessellation of a closed body (world frame)."""
    V = 0.0
    C = [0.0, 0.0, 0.0]
    ntri = 0
    for face in shape.Faces:
        tess = face.GetTessellation(TessellationOptions.Default) if hasattr(face, 'GetTessellation') else None
        if tess is None:
            raise RuntimeError('no GetTessellation')
        pts = [tess.Vertices[i].Position for i in range(tess.Vertices.Count)] if hasattr(tess, 'Vertices') else None
        idx = list(tess.Indices) if hasattr(tess, 'Indices') else None
        if pts is None or idx is None:
            raise RuntimeError('tessellation layout unknown: %s' % dir(tess))
        for k in range(0, len(idx), 3):
            a, b, c = pts[idx[k]], pts[idx[k + 1]], pts[idx[k + 2]]
            v = (a.X * (b.Y * c.Z - b.Z * c.Y) - a.Y * (b.X * c.Z - b.Z * c.X) + a.Z * (b.X * c.Y - b.Y * c.X)) / 6.0
            V += v
            C[0] += v * (a.X + b.X + c.X) / 4.0
            C[1] += v * (a.Y + b.Y + c.Y) / 4.0
            C[2] += v * (a.Z + b.Z + c.Z) / 4.0
            ntri += 1
    return V, [C[i] / V for i in range(3)], ntri


def centroid_grid(shape, bb, n):
    """Point-containment sampling, n cells along the longest bbox edge (fallback)."""
    L = max(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
    h = L / n
    nx, ny, nz = [max(1, int(round((bb[3 + i] - bb[i]) / h))) for i in range(3)]
    hx, hy, hz = (bb[3] - bb[0]) / nx, (bb[4] - bb[1]) / ny, (bb[5] - bb[2]) / nz
    cnt, s = 0, [0.0, 0.0, 0.0]
    for i in range(nx):
        x = bb[0] + (i + 0.5) * hx
        for j in range(ny):
            y = bb[1] + (j + 0.5) * hy
            for k in range(nz):
                z = bb[2] + (k + 0.5) * hz
                if shape.ContainsPoint(Point.Create(x, y, z)):
                    cnt += 1
                    s[0] += x
                    s[1] += y
                    s[2] += z
    return cnt * hx * hy * hz, [s[i] / cnt for i in range(3)], [nx, ny, nz]


try:
    write()
    DocumentOpen.Execute(src)
    root = GetRootPart()
    probe_done = False
    for body, path in walk(root, [text(root.Name)]):
        shape = body.Shape
        if not probe_done:
            result['api_probe']['shape'] = [n for n in dir(shape) if not n.startswith('_')]
            try:
                f0 = list(shape.Faces)[0]
                result['api_probe']['face'] = [n for n in dir(f0) if not n.startswith('_')]
            except Exception as exc:
                result['api_probe']['face_error'] = text(exc)
            probe_done = True
            write()
        bb = box(shape)
        row = {'owner': path, 'name': text(body.Name), 'volume_m3': float(shape.Volume), 'bbox_m': bb}
        try:
            V, c, ntri = centroid_tess(shape)
            row.update({'centroid_m': c, 'method': 'tessellation', 'tess_volume_m3': V, 'triangles': ntri})
        except Exception as exc:
            row['tess_error'] = text(exc)[:600]
            try:
                V, c, dims = centroid_grid(shape, bb, 60 if row['volume_m3'] > 0.01 else 40)
                row.update({'centroid_m': c, 'method': 'grid', 'grid_volume_m3': V, 'grid': dims})
            except Exception as exc2:
                row['grid_error'] = text(exc2)[:600]
        result['bodies'].append(row)
        write()
    result['status'] = 'ok'
except Exception:
    result['status'] = 'error'
    result['error'] = traceback.format_exc()
finally:
    write()

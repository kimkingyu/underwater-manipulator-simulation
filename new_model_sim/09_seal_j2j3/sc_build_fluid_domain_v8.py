# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Water domain v4 (static case) = v3 + seam sealing approved by kim on 2026-10-18:
#   (a) J3 tangency wedge: l2's R28 cylinder (axis X) touches link3's top plane at one point, where
#       the plane's R30 boundary arc reaches the tangent line. Over y_t-W <= y <= y_t the water between
#       link3's plane and l2's cylinder is filled, inside link3's footprint only, so the fill's sides
#       continue link3's vertical sides (90 deg water corners). The fill belongs to link3.
#   (b) jaw group seams (jaw1/jaw2/pin4): same filler rule and cluster boxes as j3_fluid_v2
#       (sc_build_j3_sealed.py), computed in the unshifted STEP frame.
# J2 (l1/l2) has only 0.2 mm parallel gaps, no zero-angle wedge: unchanged.
# Distal group shifted +10 um along +Y as in v3. AUVM.step is only read. Writes v4 scdoc + json only.
import codecs
import json
import math
import os
import traceback
from System.Collections.Generic import List
from System.IO import File
from System.Security.Cryptography import SHA256
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point, PointUV, Vector, Line, Direction
from SpaceClaim.Api.V242 import IDocObject, DesignBody

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
D = r'D:\work4\new_model_sim\09_seal_j2j3'
OUT_SCDOC = os.path.join(D, 'fluid_domain_v8.scdoc')
OUT_JSON = os.path.join(D, 'fluid_domain_v8.json')
V3_JSON = r'D:\work4\new_model_sim\03_fluid_domain\fluid_domain_v3.json'
V5_JSON = os.path.join(D, 'fluid_domain_v7.json')
# (c) parallel-axis cylinder gaps next to l2's R28 cylinder (axis Z), approved by kim 2026-10-19:
#     l2 vs base0/l1 R30 (0.068 mm) and l2 vs link3 R30 (0.198 mm). Fill |s| <= s_half (per gap), radial
#     extent r_big + G_MAX, given z bands; fill belongs to l2. 'big' parts are cut out of the fill.
L2_AXIS = (0.0028, -0.0946)
R_L2 = 0.028
S_HALF = 5e-3
G_MAX = 0.95e-3
CYL_GAPS = [
    {'name': 'l2_base0_l1', 'owner': 'l2', 'big_axis': (0.0, -0.0366), 'r_big': 0.030, 'big_parts': ['base0', 'l1'],
     's_lo': -5.0e-3, 's_hi': 5.0e-3, 'g_max': 0.95e-3, 'z_bands': [(-0.4840, -0.4430), (-0.3810, -0.3395)]},
    # gap(s) = 0.198 mm + s^2/2*(1/R30+1/R28): s_half 4.5 mm keeps gap(s_half) = 0.90 mm < G_MAX, so the
    # fill reaches l2 over its full width and its |s| = s_half sides are plain 90 deg steps.
    # v8: owned by link3 (shifted with it, united with the J3 tangency fill). In v7 it belonged to l2
    # while the tangency fill belonged to link3; both touch link3's top plane z = -341.2 mm, and the
    # 10 um shift between owners left duplicate faces there (Meshing: 'duplicate faces in boundary mesh').
    {'name': 'l2_link3', 'owner': 'link3', 'big_axis': (-0.0020, -0.1526), 'r_big': 0.030, 'big_parts': ['link3'],
     # v6 used |s| <= 4.5 mm: its sides cut l2/link3 faces at z = -341.2 mm 2 mm from the tangent point
     # (x = -2 mm) and gave 0.06 / 0.09 mm sliver faces that Meshing rejected. v7: sides at x = +7.95 /
     # -7.99 mm (s = -7.5 / +8.5 mm; +s points to -x), where those faces are >= 0.6 mm apart; gap there
     # 2.14 / 2.69 mm, so the radial clamp is raised to 2.85 mm for this gap only.
     's_lo': -7.5e-3, 's_hi': 8.5e-3, 'g_max': 2.85e-3, 'z_bands': [(-0.3940, -0.3412)]},
]
# 0.2 mm parallel plane gaps (J2 l1/l2, J3 l2/link3): faces in these x planes get NS 'gap_faces'
# (helper for local mesh sizing only; it is not a boundary zone).
GAP_PLANES_X = (0.0148, 0.0150)
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
TOL_BOOL = 1e-5
TOL_SEAL = 1e-7
TOL_OUTER = 1e-7
TOL_FACE = 1e-4
SHIFT_M = (0.0, 1e-5, 0.0)
N_SAMPLE = 6
DISTAL = ('link3', 'pin4', 'jaw1', 'jaw2')
JAW_MEMBERS = ['link3', 'pin4', 'jaw1', 'jaw2']
W_J3 = 5e-3
EMBED = 3e-4
R_J3 = 0.028
DA = 2e-5
DEPTHS = [2e-5, 5e-5, 1e-4, 1.5e-4, 2e-4, 3e-4, 4e-4, 5e-4, 6e-4, 8e-4, 1e-3, 1.2e-3, 1.5e-3, 2e-3, 2.2e-3, 2.5e-3, 3e-3]
H_MAX = 2.2e-3
CLUSTERS = [
    {'name': 'hub_jaw1_jaw2', 'x': (-0.0081, 0.0069), 'z': (-0.5595, -0.5406), 'planes': [-0.1451, -0.1601]},
    {'name': 'pin4_jaw1', 'x': (-0.0115, 0.0067), 'z': (-0.5593, -0.5356), 'planes': [-0.1501, -0.1551]},
    {'name': 'jaw1_lip', 'x': (-0.0247, -0.0124), 'z': (-0.5544, -0.5461), 'planes': [-0.1451, -0.1601]},
]
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
WALL_NAME = {'hull': 'wall_auv', 'base0': 'wall_base0', 'l1': 'wall_l1', 'l2': 'wall_l2',
             'link3': 'wall_link3', 'pin4': 'wall_pin4', 'jaw1': 'wall_jaw1', 'jaw2': 'wall_jaw2'}
report = {'status': 'started', 'source': SOURCE, 'tank_lo_m': LO, 'tank_hi_m': HI,
          'policy': 'v4 (J3 tangency wedge + jaw seams) + parallel-axis cylinder gap fills %s (gap<%.5f m) on l2 + gap_faces helper NS; approved by kim' % ([(g['name'], g['s_lo'], g['s_hi'], g['g_max']) for g in CYL_GAPS], G_MAX),
          'frame': '+X flow (inlet at x_min), +Y up', 'shift_distal_m': SHIFT_M}


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


def sha256(path):
    algo = SHA256.Create()
    stream = File.OpenRead(path)
    try:
        return ''.join('%02x' % int(v) for v in algo.ComputeHash(stream))
    finally:
        stream.Dispose()
        algo.Dispose()


def world_copy(design_body):
    shape = design_body.GetMaster().Shape.Copy()
    shape.Transform(design_body.GetTransformToMaster().Inverse)
    return shape


def name_for(volume):
    if volume > 0.01:
        return 'hull'
    best = min(ARM_BY_VOLUME, key=lambda row: abs(row[0] - volume))
    return best[1] if abs(best[0] - volume) / best[0] < 2e-3 else None


def bbox(obj):
    b = obj.GetBoundingBox(Matrix.Identity)
    return ([b.MinCorner.X, b.MinCorner.Y, b.MinCorner.Z], [b.MaxCorner.X, b.MaxCorner.Y, b.MaxCorner.Z])


def mm(values):
    return [round(v * 1000.0, 4) for v in values]


def make_box(lo, hi):
    created = list(BlockBody.Create(Point.Create(*lo), Point.Create(*hi),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    shape = world_copy(created)
    created.Delete()
    return shape


def lst(*bodies):
    out = List[Body]()
    for b in bodies:
        out.Add(b.Copy())
    return out


def intersect(a, b, tol=TOL_SEAL):
    c = a.Copy()
    try:
        c.Intersect(lst(b), tol)
    except Exception:
        return None
    if c.Volume <= 0:
        return None
    return c


def minus(a, b, tol=TOL_SEAL):
    c = a.Copy()
    c.Subtract(lst(b), tol)
    return c


def as_point(hit):
    return getattr(hit, 'Point', hit)


def dist(p, q):
    return ((p.X - q.X) ** 2 + (p.Y - q.Y) ** 2 + (p.Z - q.Z) ** 2) ** 0.5


def samples(face):
    box = face.BoxUV
    ru, rv = box.RangeU, box.RangeV
    pts = []
    for i in range(N_SAMPLE):
        for j in range(N_SAMPLE):
            uv = PointUV.Create(ru.Start + (ru.End - ru.Start) * (i + 0.5) / N_SAMPLE,
                                rv.Start + (rv.End - rv.Start) * (j + 0.5) / N_SAMPLE)
            if face.ContainsParam(uv):
                pts.append(face.Geometry.Evaluate(uv).Point)
    if not pts:
        b = face.GetBoundingBox(Matrix.Identity)
        pts.append(as_point(face.ProjectPoint(b.Center)))
    return pts


def plateau_depth(areas):
    if areas[-1] <= 0 or max(areas) <= areas[0] * 1.002 + 1e-12:
        return None
    for i in range(1, len(DEPTHS) - 2):
        if DEPTHS[i] > H_MAX:
            break
        if areas[i] <= areas[0] * 1.002:
            continue
        if areas[i + 1] <= areas[i] * 1.002 and areas[i + 2] <= areas[i] * 1.002:
            return DEPTHS[i]
    return None


def slab_box(cl, y0, s, d0, d1):
    ya, yb = y0 + s * d0, y0 + s * d1
    return make_box((cl['x'][0], min(ya, yb), cl['z'][0]), (cl['x'][1], max(ya, yb), cl['z'][1]))


def jaw_fillers(tools):
    fillers = dict((m, []) for m in JAW_MEMBERS)
    log = []
    for cl in CLUSTERS:
        for y0 in cl['planes']:
            for m in JAW_MEMBERS:
                for s in (-1.0, 1.0):
                    areas = []
                    for d in DEPTHS:
                        piece = intersect(tools[m], slab_box(cl, y0, s, d, d + DA))
                        areas.append(float(piece.Volume) / DA if piece is not None else 0.0)
                    h = plateau_depth(areas)
                    if h is None:
                        continue
                    row = {'cluster': cl['name'], 'plane_y_m': y0, 'part': m, 'side': s, 'h_m': h}
                    df = h / 4.0
                    ref = intersect(tools[m], slab_box(cl, y0, s, h, h + df))
                    if ref is None:
                        row['error'] = 'empty reference slab'
                        log.append(row)
                        continue
                    filler = ref.Copy()
                    for k in range(1, 5):
                        c = ref.Copy()
                        c.Transform(Matrix.CreateTranslation(Vector.Create(0, -s * k * df, 0)))
                        filler.Unite(lst(c), TOL_SEAL)
                    added = minus(filler, tools[m])
                    row['added_to_part_mm3'] = float(added.Volume) * 1e9
                    if added.Volume > 1e-15:
                        fillers[m].append(filler)
                    log.append(row)
    return fillers, log


def find_j3_tangency(tools):
    """l2 cylinder R28 with axis along X whose lowest line touches a link3 plane with normal +-Z."""
    cyl = []
    for f in tools['l2'].Faces:
        g = f.Geometry
        if g.GetType().Name != 'Cylinder' or abs(float(g.Radius) - R_J3) > 1e-6:
            continue
        d = g.Axis.Direction
        if abs(abs(d.X) - 1.0) > 1e-6:
            continue
        o = g.Axis.Origin
        lo, hi = bbox(f)
        cyl.append({'axis_y': o.Y, 'axis_z': o.Z, 'x': (lo[0], hi[0]), 'face_lo': lo, 'face_hi': hi})
    planes = []
    for f in tools['link3'].Faces:
        g = f.Geometry
        if g.GetType().Name != 'Plane':
            continue
        n = g.Frame.DirZ
        if abs(abs(n.Z) - 1.0) > 1e-6:
            continue
        lo, hi = bbox(f)
        planes.append({'z': g.Frame.Origin.Z, 'x': (lo[0], hi[0]), 'y': (lo[1], hi[1]), 'area_m2': float(f.Area)})
    matches = []
    for c in cyl:
        for p in planes:
            if abs((c['axis_z'] - R_J3) - p['z']) < 1e-6 and p['y'][0] - 1e-6 <= c['axis_y'] <= p['y'][1] + 1e-6:
                xo = (max(c['x'][0], p['x'][0]), min(c['x'][1], p['x'][1]))
                if xo[1] - xo[0] > 1e-4:
                    matches.append({'cyl': c, 'plane': p, 'x_overlap': xo})
    # A cylinder split into several faces shares one axis: merge matches per (axis, plane).
    merged = {}
    for mt in matches:
        key = (round(mt['cyl']['axis_y'], 7), round(mt['cyl']['axis_z'], 7), round(mt['plane']['z'], 7))
        if key in merged:
            xo = merged[key]['x_overlap']
            merged[key]['x_overlap'] = (min(xo[0], mt['x_overlap'][0]), max(xo[1], mt['x_overlap'][1]))
            merged[key]['faces_merged'] += 1
        else:
            merged[key] = dict(mt, faces_merged=1)
    return cyl, planes, list(merged.values())


try:
    if os.path.exists(OUT_JSON) or os.path.exists(OUT_SCDOC):
        raise Exception('Refusing to overwrite v4 outputs')
    save()
    report['source_sha256_before'] = sha256(SOURCE)
    DocumentOpen.Execute(SOURCE)
    tools = {}
    for db in GetRootPart().GetAllBodies():
        shape = world_copy(db)
        name = name_for(float(shape.Volume))
        if name is None or name in tools:
            raise Exception('Unrecognised or duplicate body volume %.9f' % float(shape.Volume))
        tools[name] = shape
    if sorted(tools) != sorted(WALL_NAME):
        raise Exception('Expected hull + 7 arm bodies, got %s' % sorted(tools))

    # (b) jaw-group fillers in the unshifted frame (same as j3_fluid_v2).
    fillers, jlog = jaw_fillers(tools)
    report['jaw_filler_log'] = jlog
    report['jaw_fillers'] = dict((m, len(v)) for m, v in fillers.items())
    save()

    # (a) J3 tangency, unshifted frame.
    cyl, planes, matches = find_j3_tangency(tools)
    report['j3_candidates'] = {'cylinders': cyl, 'planes': planes, 'matches': matches}
    save()
    if len(matches) != 1:
        raise Exception('Expected exactly one J3 tangency, found %d' % len(matches))
    mt = matches[0]
    y_t, z_axis, z_p = mt['cyl']['axis_y'], mt['cyl']['axis_z'], mt['plane']['z']
    x0, x1 = mt['x_overlap']
    h_w = R_J3 - math.sqrt(R_J3 ** 2 - W_J3 ** 2)
    # link3 lifted by (h_w + EMBED): inside the slab it is exactly link3's footprint prism, because
    # link3 is a deep solid under its top plane with vertical side faces there.
    lifted = tools['link3'].Copy()
    lifted.Transform(Matrix.CreateTranslation(Vector.Create(0, 0, h_w + EMBED)))
    slab = make_box((x0, y_t - W_J3, z_p - EMBED), (x1, y_t, z_p + h_w))
    j3_fill, tol_used, errors = None, None, []
    for tol in (1e-7, 1e-6, 1e-8):
        try:
            prism = intersect(slab, lifted, tol)
            if prism is None:
                raise Exception('empty footprint prism')
            j3_fill = minus(prism, tools['l2'], tol)
            tol_used = tol
            break
        except Exception as exc:
            errors.append({'tol': tol, 'error': u(exc)})
    report['j3_fill_attempts'] = errors
    if j3_fill is None:
        save()
        raise Exception('J3 fill construction failed at all tolerances')
    lo_f, hi_f = bbox(j3_fill)
    below = intersect(j3_fill, make_box((x0 - 1e-3, y_t - W_J3 - 1e-3, z_p - EMBED - 1e-3),
                                        (x1 + 1e-3, y_t + 1e-3, z_p)))
    over_l2 = intersect(j3_fill, tools['l2'])
    wedge_mm3 = (float(j3_fill.Volume) - (float(below.Volume) if below is not None else 0.0)) * 1e9
    report['j3_fill'] = {'y_tangent_m': y_t, 'axis_z_m': z_axis, 'plane_z_m': z_p, 'x_range_m': [x0, x1],
                         'half_width_m': W_J3, 'max_height_m': h_w, 'tol_used': tol_used,
                         'fill_bbox_mm': mm(lo_f) + mm(hi_f),
                         'fill_mm3': float(j3_fill.Volume) * 1e9,
                         'embedded_below_plane_mm3': float(below.Volume) * 1e9 if below is not None else 0.0,
                         'wedge_above_plane_mm3': wedge_mm3,
                         'wedge_upper_bound_mm3': (x1 - x0) * W_J3 * h_w * 1e9,
                         'overlap_with_l2_mm3': float(over_l2.Volume) * 1e9 if over_l2 is not None else 0.0,
                         'faces': len(list(j3_fill.Faces))}
    save()
    jf = report['j3_fill']
    if jf['overlap_with_l2_mm3'] > 1e-3:
        raise Exception('J3 fill overlaps l2')
    if not (0.0 < jf['wedge_above_plane_mm3'] < jf['wedge_upper_bound_mm3']):
        raise Exception('J3 fill wedge volume out of bounds')
    # SpaceClaim bounding boxes are padded by ~2 um on curved bodies; allow 5 um.
    if hi_f[2] > z_p + h_w + 5e-6 or lo_f[1] < y_t - W_J3 - 5e-6 or hi_f[1] > y_t + 5e-6:
        raise Exception('J3 fill leaves its slab')

    # (c) parallel-axis cylinder gaps, unshifted frame. The fill belongs to l2 (never shifted). link3's
    # +10 um Y shift moves it ~10 um TOWARD l2 at the l2/link3 gap, so link3 then overlaps the fill by
    # ~10 um (both are material: no new water sliver).
    cyl_fill_parts, cyl_rows = [], []
    for gp in CYL_GAPS:
        bax = gp['big_axis']
        r_big = gp['r_big']
        s_lo, s_hi, g_max = gp['s_lo'], gp['s_hi'], gp['g_max']
        dxa, dya = L2_AXIS[0] - bax[0], L2_AXIS[1] - bax[1]
        d_ax = math.hypot(dxa, dya)
        ang = math.atan2(dya, dxa)
        rot = Matrix.CreateRotation(Line.Create(Point.Origin, Direction.DirZ), ang)
        shift = Matrix.CreateTranslation(Vector.Create(bax[0], bax[1], 0.0))
        for (za, zb) in gp['z_bands']:
            local = make_box((r_big - 2e-3, s_lo, za), (r_big + g_max + 0.6e-3, s_hi, zb))
            clamp = list(CylinderBody.Create(Point.Create(0, 0, za - 1e-3), Point.Create(0, 0, zb + 1e-3),
                                             Point.Create(r_big + g_max, 0, za - 1e-3),
                                             ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
            clamp_shape = world_copy(clamp)
            clamp.Delete()
            local = intersect(local, clamp_shape)
            local.Transform(rot)
            local.Transform(shift)
            if gp['owner'] == 'l2':
                piece = minus(local, tools['l2'])
            else:
                # Owner is shifted by SHIFT_M later. Cut against l2 moved by -SHIFT_M so that after the
                # shift the fill coincides exactly with the real l2 (no 10 um interference at Boolean
                # tolerance); its inner side is cut by the unshifted owner and so moves with it exactly.
                l2_up = tools['l2'].Copy()
                l2_up.Transform(Matrix.CreateTranslation(Vector.Create(-SHIFT_M[0], -SHIFT_M[1], -SHIFT_M[2])))
                piece = minus(local, l2_up)
            for b in gp['big_parts']:
                piece = minus(piece, tools[b])
            others = [n for n in tools if n != 'l2' and n not in gp['big_parts']
                      and intersect(piece, tools[n]) is not None]
            pl, ph = bbox(piece)
            nint = 200
            area = 0.0
            for i in range(nint):
                s = s_lo + (i + 0.5) * (s_hi - s_lo) / nint
                area += (d_ax - math.sqrt(R_L2 ** 2 - s * s) - math.sqrt(r_big ** 2 - s * s)) * (s_hi - s_lo) / nint
            g0 = d_ax - r_big - R_L2
            probe = Point.Create(bax[0] + (r_big + 0.5 * g0) * dxa / d_ax, bax[1] + (r_big + 0.5 * g0) * dya / d_ax,
                                 0.5 * (za + zb))
            row = {'gap': gp['name'], 'z_band_m': [za, zb], 'min_gap_m': g0, 'fill_mm3': float(piece.Volume) * 1e9,
                   'pieces': len(list(piece.SeparatePieces())),
                   'nominal_section_mm2': area * 1e6, 'nominal_prism_mm3': area * (zb - za) * 1e9,
                   'ratio_to_nominal': float(piece.Volume) / (area * (zb - za)),
                   'mid_gap_point_in_fill': bool(piece.ContainsPoint(probe)),
                   'overlaps_other_parts': others, 'bbox_mm': mm(pl) + mm(ph)}
            cyl_rows.append(row)
            cyl_fill_parts.append((gp['owner'], piece))
            report['cyl_gap_fill'] = {'g_max_m': G_MAX, 'gaps': CYL_GAPS, 'bands': cyl_rows}
            save()
    for row in cyl_rows:
        if not row['mid_gap_point_in_fill']:
            raise Exception('Cylinder-gap fill misplaced for %s %s' % (row['gap'], row['z_band_m']))
        if row['overlaps_other_parts']:
            raise Exception('Cylinder-gap fill touches other parts %s' % row['overlaps_other_parts'])
        if not (0.9 <= row['ratio_to_nominal'] <= 2.5):
            raise Exception('Cylinder-gap fill volume out of bounds for %s %s' % (row['gap'], row['z_band_m']))
    cyl_fill = {}
    for owner, p in cyl_fill_parts:
        if owner in cyl_fill:
            cyl_fill[owner].Unite(lst(p), TOL_SEAL)
        else:
            cyl_fill[owner] = p.Copy()

    # Extended parts, then the v3 shift of the distal group.
    ext = {}
    for m in tools:
        e = tools[m].Copy()
        for f in fillers.get(m, []):
            e.Unite(lst(f), TOL_SEAL)
        if m == 'link3':
            # Shifted with link3 (+10 um toward the tangent line): the fill top then sits up to
            # slope*10 um inside l2 instead of leaving a gap; its sides continue link3's sides.
            e.Unite(lst(j3_fill), tol_used)
        if m in cyl_fill:
            e.Unite(lst(cyl_fill[m]), TOL_SEAL)
        ext[m] = e
    move = Matrix.CreateTranslation(Vector.Create(*SHIFT_M))
    for m in DISTAL:
        ext[m].Transform(move)
    report['tools'] = dict((n, {'part_m3': float(tools[n].Volume), 'extended_m3': float(ext[n].Volume),
                                'shifted': n in DISTAL}) for n in ext)
    save()

    # Water.
    doc = Document.Create()
    root = doc.MainPart
    fluid = make_box(LO, HI)
    v_box = float(fluid.Volume)
    report['box_volume_m3'] = v_box
    ledger, v_prev = [], v_box
    # link3 (with both fills) cannot be subtracted on its own (General Failure at every tolerance): its
    # fills meet l2 along y = -122.6 / z = -341.2. Unite l2 and link3 into one cutting tool first; face
    # ownership below is still decided against ext['l2'] / ext['link3'] separately.
    cut_tools = dict(ext)
    merge_log = []
    for tol in (TOL_SEAL, 1e-6, TOL_BOOL):
        try:
            combo = ext['l2'].Copy()
            combo.Unite(lst(ext['link3']), tol)
            pcs = len(list(combo.SeparatePieces()))
            merge_log.append({'tol': tol, 'volume_m3': float(combo.Volume), 'pieces': pcs,
                              'sum_m3': float(ext['l2'].Volume) + float(ext['link3'].Volume)})
            if pcs == 1:
                del cut_tools['l2']
                del cut_tools['link3']
                cut_tools['l2+link3'] = combo
                break
        except Exception as exc:
            merge_log.append({'tol': tol, 'error': u(exc)})
    report['l2_link3_merge'] = merge_log
    save()
    for name in sorted(cut_tools, key=lambda n: -cut_tools[n].Volume):
        row = {'tool': name, 'attempts': []}
        ledger.append(row)
        report['subtraction_ledger'] = ledger
        save()
        done = False
        for tol in (TOL_BOOL, 1e-6, 1e-7, 3e-5):
            trial = fluid.Copy()
            try:
                trial.Subtract(lst(cut_tools[name]), tol)
                if trial.Volume <= 0:
                    raise Exception('empty result')
                fluid = trial
                row['tol_used'] = tol
                done = True
                break
            except Exception as exc:
                row['attempts'].append({'tol': tol, 'error': u(exc)})
        if not done:
            save()
            raise Exception('Subtracting %s failed at all tolerances' % name)
        v_now = float(fluid.Volume)
        row['cut_m3'] = v_prev - v_now
        v_prev = v_now
    report['subtraction_ledger'] = ledger
    pieces = list(fluid.SeparatePieces())
    outer, rows = [], []
    for p in pieces:
        lo, hi = bbox(p)
        touches = all(abs(lo[k] - LO[k]) <= TOL_OUTER and abs(hi[k] - HI[k]) <= TOL_OUTER for k in range(3))
        rows.append({'volume_m3': float(p.Volume), 'reaches_all_box_sides': touches, 'bbox_mm': mm(lo) + mm(hi)})
        if touches:
            outer.append(p)
    report['pieces'] = sorted(rows, key=lambda r: -r['volume_m3'])
    save()
    if len(outer) != 1:
        raise Exception('Expected exactly one outer water piece, found %d' % len(outer))
    main = outer[0]
    imprint_log = []
    for name in sorted(ext, key=lambda n: -ext[n].Volume):
        n0 = len(list(main.Faces))
        try:
            main.Imprint(ext[name].Copy())
            imprint_log.append({'tool': name, 'faces_before': n0, 'faces_after': len(list(main.Faces))})
        except Exception as exc:
            imprint_log.append({'tool': name, 'error': u(exc)})
    report['imprint'] = imprint_log
    report['water'] = {'volume_m3': float(main.Volume), 'faces': len(list(main.Faces)),
                       'closed': bool(main.IsClosed), 'manifold': bool(main.IsManifold)}
    save()
    if not main.IsClosed or not main.IsManifold:
        raise Exception('Water body not closed/manifold')

    # Names.
    water = DesignBody.Create(root, 'water', main)
    groups = {'inlet': [], 'outlet': [], 'tank_wall': []}
    for n in WALL_NAME.values():
        groups[n] = []
    unassigned, ambiguous = [], []
    owner_max = 0.0
    for idx, dface in enumerate(list(water.Faces)):
        face = dface.Shape
        lo, hi = bbox(face)
        if hi[0] - lo[0] <= TOL_OUTER and abs(lo[0] - LO[0]) <= TOL_OUTER:
            groups['inlet'].append(dface)
            continue
        if hi[0] - lo[0] <= TOL_OUTER and abs(lo[0] - HI[0]) <= TOL_OUTER:
            groups['outlet'].append(dface)
            continue
        if any(hi[k] - lo[k] <= TOL_OUTER and (abs(lo[k] - LO[k]) <= TOL_OUTER or abs(lo[k] - HI[k]) <= TOL_OUTER)
               for k in (1, 2)):
            groups['tank_wall'].append(dface)
            continue
        pts = samples(face)
        worst = dict((n, max(dist(as_point(s.ProjectPoint(p)), p) for p in pts)) for n, s in ext.items())
        ranked = sorted((d, n) for n, d in worst.items())
        if ranked[0][0] >= TOL_FACE:
            unassigned.append({'face_index': idx, 'nearest': ranked[0][1], 'max_dist_m': ranked[0][0],
                               'area_mm2': float(face.Area) * 1e6, 'sample_mm': mm([pts[0].X, pts[0].Y, pts[0].Z])})
            continue
        if ranked[1][0] < TOL_FACE:
            ambiguous.append({'face_index': idx, 'chosen': ranked[0][1], 'other': ranked[1][1],
                              'd_chosen_m': ranked[0][0], 'd_other_m': ranked[1][0],
                              'area_mm2': float(face.Area) * 1e6})
        owner_max = max(owner_max, ranked[0][0])
        groups[WALL_NAME[ranked[0][1]]].append(dface)
    report['owner_max_dist_m'] = owner_max
    report['unassigned_faces'] = unassigned
    report['ambiguous_faces'] = ambiguous
    report['named_selections'] = dict((n, {'faces': len(f), 'area_m2': sum(float(x.Shape.Area) for x in f)})
                                      for n, f in groups.items())
    stream = codecs.open(V3_JSON, 'r', 'utf-8')
    report['named_selections_v3'] = json.load(stream)['named_selections']
    stream.close()
    stream = codecs.open(V5_JSON, 'r', 'utf-8')
    report['named_selections_v7'] = json.load(stream)['named_selections']
    stream.close()
    report['short_edges_lt_0p1mm'] = sorted(round(float(e.Length) * 1e6, 3) for e in main.Edges if e.Length < 1e-4)
    report['tiny_faces_lt_0p01mm2'] = sorted(round(float(f.Area) * 1e6, 5) for f in main.Faces if f.Area < 1e-8)
    save()
    empty = sorted(n for n, f in groups.items() if not f)
    if unassigned or empty:
        raise Exception('Naming incomplete: unassigned=%d empty=%s' % (len(unassigned), empty))
    for n in sorted(groups):
        members = List[IDocObject]()
        for f in groups[n]:
            members.Add(f)
        Group.Create(root, n, members)
    gap_faces = []
    for dface in list(water.Faces):
        f = dface.Shape
        if f.Geometry.GetType().Name != 'Plane':
            continue
        flo, fhi = bbox(f)
        if fhi[0] - flo[0] > 2e-6:
            continue
        if any(abs(0.5 * (flo[0] + fhi[0]) - px) <= 2e-6 for px in GAP_PLANES_X) and fhi[2] < -0.28:
            gap_faces.append(dface)
    report['gap_faces'] = [{'area_mm2': round(float(x.Shape.Area) * 1e6, 3)} for x in gap_faces]
    save()
    if len(gap_faces) != 4:
        raise Exception('Expected 4 gap faces, found %d' % len(gap_faces))
    members = List[IDocObject]()
    for f in gap_faces:
        members.Add(f)
    Group.Create(root, 'gap_faces', members)
    DocumentSave.Execute(OUT_SCDOC)
    report['scdoc'] = OUT_SCDOC
    report['scdoc_sha256'] = sha256(OUT_SCDOC)
    report['source_unchanged'] = sha256(SOURCE) == report['source_sha256_before']
    report['status'] = 'ok'
except Exception:
    report['status'] = 'error'
    report['traceback'] = u(traceback.format_exc())
finally:
    save()

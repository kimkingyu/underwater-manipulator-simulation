# Python Script, API Version = V242
# -*- coding: utf-8 -*-
# Trial water domain around the new model. Reads AUVM.step, writes ONLY new files in OUT_DIR.
# Method follows cfd/wetted_arm_rebuild/build_final_fluid_domain.py (BlockBody box, Body.Subtract,
# SeparatePieces, projection-based face grouping, Group.Create named selections).
#
# Frame (new model): +X = flow direction (inlet at x_min), +Y = up, Z = transverse.
# Tank = old tank mapped into the new frame (about 1.15 m clearance on every side).
import codecs
import json
import traceback
from System.Collections.Generic import List
from SpaceClaim.Api.V242.Modeler import Body
from SpaceClaim.Api.V242.Geometry import Matrix, Point
from SpaceClaim.Api.V242 import IDocObject, DesignBody

SOURCE = r'D:\work4\简化模型_导出\AUVM.step'
OUT_DIR = r'D:\work4\new_model_sim\03_fluid_domain'
OUT_SCDOC = OUT_DIR + r'\fluid_domain_trial.scdoc'
OUT_STEP = OUT_DIR + r'\fluid_domain_trial.step'
OUT_JSON = OUT_DIR + r'\fluid_domain_trial.json'
LO = (-1.5605, -1.3296, -1.8008)
HI = (1.5601, 1.9136, 1.6404)
TOL_BOOL = 1e-5
TOL_FACE = 1e-4
# Arm bodies identified by volume (verified against SolidWorks, rel. diff 1e-5).
ARM_BY_VOLUME = [(0.0006491, 'base0'), (0.0001230, 'l1'), (0.0006265, 'l2'), (0.0004371, 'link3'),
                 (0.0000269, 'jaw1'), (0.0000277, 'jaw2'), (0.0000211, 'pin4')]
manifest = {'status': 'started', 'source': SOURCE, 'tank_lo_m': LO, 'tank_hi_m': HI}


def u(value):
    try:
        return unicode(value)
    except Exception:
        return repr(value)


def save_manifest():
    stream = codecs.open(OUT_JSON, 'w', 'utf-8')
    try:
        stream.write(json.dumps(manifest, ensure_ascii=False, indent=2))
    finally:
        stream.close()


def world_copy(design_body):
    master = design_body.GetMaster()
    shape = master.Shape.Copy()
    shape.Transform(design_body.GetTransformToMaster().Inverse)
    return shape


def label_for(volume):
    best = min(ARM_BY_VOLUME, key=lambda row: abs(row[0] - volume))
    if abs(best[0] - volume) / best[0] < 2e-3:
        return best[1]
    return None


try:
    save_manifest()
    DocumentOpen.Execute(SOURCE)
    root = GetRootPart()
    design_bodies = list(root.GetAllBodies())
    tools = []
    for db in design_bodies:
        shape = world_copy(db)
        volume = float(shape.Volume)
        if volume > 0.01:
            name = 'hull'
        else:
            name = label_for(volume)
            if name is None:
                raise Exception('Unrecognised body volume %.9f' % volume)
        tools.append((name, shape))
    manifest['tools'] = [{'name': n, 'volume_m3': float(s.Volume), 'area_m2': float(s.SurfaceArea)}
                         for n, s in tools]
    if sorted(n for n, _ in tools) != sorted(['hull'] + [n for _, n in ARM_BY_VOLUME]):
        raise Exception('Expected hull + 7 arm bodies, got %s' % [n for n, _ in tools])

    # Contact check: any two touching tools would merge when subtracted (fine for walls,
    # but we record it so the wall split later is deliberate, not accidental).
    contacts = []
    for i in range(len(tools)):
        for j in range(i + 1, len(tools)):
            a = tools[i][1].Copy()
            lst = List[Body]()
            lst.Add(tools[j][1].Copy())
            try:
                a.Intersect(lst, TOL_BOOL)
                overlap = float(a.Volume)
            except Exception:
                overlap = 0.0
            if overlap > 1e-12:
                contacts.append({'a': tools[i][0], 'b': tools[j][0], 'overlap_m3': overlap})
    manifest['tool_overlaps'] = contacts

    created = list(BlockBody.Create(Point.Create(*LO), Point.Create(*HI),
                                    ExtrudeType.ForceIndependent, None).CreatedBodies)[0]
    fluid = world_copy(created)
    created.Delete()
    v_box = float(fluid.Volume)
    manifest['box_volume_m3'] = v_box
    ledger = []
    v_prev = v_box
    for name, shape in sorted(tools, key=lambda row: -row[1].Volume):
        lst = List[Body]()
        lst.Add(shape.Copy())
        fluid.Subtract(lst, TOL_BOOL)
        v_now = float(fluid.Volume)
        ledger.append({'tool': name, 'tool_volume_m3': float(shape.Volume), 'cut_m3': v_prev - v_now})
        v_prev = v_now
    manifest['subtraction_ledger'] = ledger

    pieces = list(fluid.SeparatePieces())
    pieces.sort(key=lambda p: -p.Volume)
    manifest['pieces'] = [{'volume_m3': float(p.Volume), 'faces': len(list(p.Faces))} for p in pieces]
    main = pieces[0]
    manifest['fluid'] = {'volume_m3': float(main.Volume), 'faces': len(list(main.Faces)),
                         'closed': bool(main.IsClosed), 'manifold': bool(main.IsManifold)}
    manifest['expected_fluid_volume_m3'] = v_box - sum(float(s.Volume) for _, s in tools)

    # Face grouping: outer walls by plane position, walls by nearest tool surface.
    fluid_db = DesignBody.Create(root, 'water', main)
    groups = {'inlet': [], 'outlet': [], 'tank_walls': []}
    for name, _ in tools:
        groups['wall_' + name] = []
    unassigned = []
    for idx, face in enumerate(list(fluid_db.Faces)):
        box = face.Shape.GetBoundingBox(Matrix.Identity)
        mn, mx = box.MinCorner, box.MaxCorner
        if abs(mn.X - LO[0]) < TOL_FACE and abs(mx.X - LO[0]) < TOL_FACE:
            groups['inlet'].append(face)
            continue
        if abs(mn.X - HI[0]) < TOL_FACE and abs(mx.X - HI[0]) < TOL_FACE:
            groups['outlet'].append(face)
            continue
        on_side = False
        for k, coord in ((1, 'Y'), (2, 'Z')):
            for plane in (LO[k], HI[k]):
                if abs(getattr(mn, coord) - plane) < TOL_FACE and abs(getattr(mx, coord) - plane) < TOL_FACE:
                    on_side = True
        if on_side:
            groups['tank_walls'].append(face)
            continue
        probe = face.Shape.ProjectPoint(box.Center)
        point = getattr(probe, 'Point', probe)
        best = (1e9, None)
        for name, shape in tools:
            hit = shape.ProjectPoint(point)
            q = getattr(hit, 'Point', hit)
            dist = ((q.X - point.X) ** 2 + (q.Y - point.Y) ** 2 + (q.Z - point.Z) ** 2) ** 0.5
            if dist < best[0]:
                best = (dist, name)
        if best[0] < TOL_FACE:
            groups['wall_' + best[1]].append(face)
        else:
            unassigned.append({'face_index': idx, 'nearest': best[1], 'distance_m': best[0]})
    manifest['unassigned_faces'] = unassigned
    manifest['named_selections'] = {}
    for name, faces in groups.items():
        manifest['named_selections'][name] = {'faces': len(faces),
                                              'area_m2': sum(float(f.Shape.Area) for f in faces)}
        if faces:
            members = List[IDocObject]()
            for f in faces:
                members.Add(f)
            Group.Create(root, name, members)
    manifest['tool_surface_areas_m2'] = dict((n, float(s.SurfaceArea)) for n, s in tools)

    for db in design_bodies:
        db.Delete()
    DocumentSave.Execute(OUT_SCDOC)
    manifest['saved_scdoc'] = OUT_SCDOC
    try:
        DocumentSave.Execute(OUT_STEP)
        manifest['saved_step'] = OUT_STEP
    except Exception as exc:
        manifest['step_error'] = u(exc)
    manifest['status'] = 'ok'
except Exception:
    manifest['status'] = 'error'
    manifest['traceback'] = u(traceback.format_exc())
finally:
    save_manifest()

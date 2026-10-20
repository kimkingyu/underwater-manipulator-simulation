"""Open the new SolidWorks model READ-ONLY and export neutral CAD for ANSYS.

* SolidWorks 2025 runs invisibly through COM; the assembly opens read-only and silent.
* No SolidWorks document is saved: exports use Extension.SaveAs3 with Copy|Silent.
* Every user preference changed for export is restored afterwards.
* Source SHA-256 is compared before and after; any change makes the run fail.
* Records component tree, transforms, mass properties and mates (needed for joints).
* The SolidWorks process this script starts is identified by PID and always shut down.

SolidWorks objects here expose no runtime type info (GetTypeInfo fails), so every object
is bound explicitly to its generated interface wrapper; otherwise methods are mistaken
for properties and errors are silently swallowed.

Outputs: D:/work4/简化模型_导出/ (CAD) and new_model_sim/01_sw_export/ (records).
"""
import hashlib
import json
import subprocess
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

import pythoncom
import win32com.client
from win32com.client import gencache

SRC_DIR = Path(r'D:\work4\简化模型')
ASSEMBLY = SRC_DIR / 'AUVM.SLDASM'
EXPORT_DIR = Path(r'D:\work4\简化模型_导出')
RECORD_DIR = Path(r'D:\work4\new_model_sim\01_sw_export')
ENUMS = json.loads((RECORD_DIR / 'swconst_enums.json').read_text(encoding='utf-8'))
SW = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 33, 0)


def const(enum, member):
    return ENUMS[enum][member]


DOC_ASSEMBLY = const('swDocumentTypes_e', 'swDocASSEMBLY')
OPEN_OPTS = (const('swOpenDocOptions_e', 'swOpenDocOptions_Silent')
             | const('swOpenDocOptions_e', 'swOpenDocOptions_ReadOnly'))
SAVE_OPTS = (const('swSaveAsOptions_e', 'swSaveAsOptions_Silent')
             | const('swSaveAsOptions_e', 'swSaveAsOptions_Copy'))
SAVE_CURRENT = const('swSaveAsVersion_e', 'swSaveAsCurrentVersion')
SOLID_BODY = const('swBodyType_e', 'swSolidBody')
PREF_INT = ENUMS['swUserPreferenceIntegerValue_e']
PREF_TOGGLE = ENUMS['swUserPreferenceToggle_e']
PREF_STRING = ENUMS['swUserPreferenceStringValue_e']
# Parasolid 34.1: readable by ANSYS 2024 R2, which already read v36 x_t files in this project.
PARASOLID_VERSION = const('swParasolidOutputVersion_e', 'swParasolidOutputVersion_341')
MATE_TYPES = {v: k for k, v in ENUMS['swMateType_e'].items()}
REF_TYPES = {v: k for k, v in ENUMS['swMateEntity2ReferenceType_e'].items()}
LOAD_ERRORS = ENUMS['swFileLoadError_e']
LOAD_WARNINGS = ENUMS['swFileLoadWarning_e']
SAVE_ERRORS = ENUMS['swFileSaveError_e']
SAVE_WARNINGS = ENUMS['swFileSaveWarning_e']
SUPPRESSION = {v: k for k, v in ENUMS['swComponentSuppressionState_e'].items()}


def wrap(obj, interface):
    """Bind a raw COM object to a generated interface wrapper (None stays None)."""
    if obj is None:
        return None
    return getattr(SW, interface)(getattr(obj, '_oleobj_', obj))


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def decode_flags(value, table):
    return [name for name, bit in table.items() if bit and value & bit == bit]


def sw_pids():
    out = subprocess.run(
        ['powershell', '-NoProfile', '-Command',
         "Get-CimInstance Win32_Process -Filter \"Name='SLDWORKS.exe'\" | ForEach-Object { $_.ProcessId }"],
        capture_output=True, text=True, check=True).stdout
    return {int(x) for x in out.split()}


def attempt(record, key, call):
    """Run one optional step; record its error text instead of silently dropping it."""
    try:
        return call()
    except Exception as exc:
        record.setdefault('step_errors', {})[key] = repr(exc)
        return None


def transform_of(comp):
    xf = wrap(comp.Transform2, 'IMathTransform')
    if xf is None:
        return None
    data = list(xf.ArrayData)
    return {'rotation_rows': [data[0:3], data[3:6], data[6:9]],
            'translation_m': data[9:12], 'scale': data[12]}


def component_rows(record, assy):
    rows = []
    for raw in assy.GetComponents(False) or []:
        comp = wrap(raw, 'IComponent2')
        name = comp.Name2
        parent = wrap(comp.GetParent(), 'IComponent2')
        row = {'name': name, 'parent': parent.Name2 if parent else None,
               'file': comp.GetPathName(),
               'configuration': comp.ReferencedConfiguration}
        state = comp.GetSuppression2()
        row['suppression'] = SUPPRESSION.get(state, state)
        row['hidden'] = attempt(record, name + '.hidden', lambda: bool(comp.IsHidden(True)))
        row['fixed'] = attempt(record, name + '.fixed', lambda: bool(comp.IsFixed()))
        row['virtual'] = attempt(record, name + '.virtual', lambda: bool(comp.IsVirtual))
        row['transform'] = attempt(record, name + '.transform', lambda: transform_of(comp))
        box = attempt(record, name + '.box', lambda: comp.GetBox(False, False))
        row['bbox_m'] = list(box) if box else None
        bodies = attempt(record, name + '.bodies', lambda: comp.GetBodies3(SOLID_BODY))
        if isinstance(bodies, tuple) and len(bodies) == 2 and not hasattr(bodies[0], '_oleobj_'):
            bodies = bodies[0]  # (bodies, BodiesInfo) when the out-parameter is returned
        row['solid_bodies'] = len(bodies) if bodies else 0
        rows.append(row)
    return rows


def mate_rows(record, model):
    rows = []
    feature = wrap(model.FirstFeature(), 'IFeature')
    guard = 0
    while feature is not None and guard < 100000:
        guard += 1
        if feature.GetTypeName2() == 'MateGroup':
            sub = wrap(feature.GetFirstSubFeature(), 'IFeature')
            while sub is not None:
                entry = {'name': sub.Name, 'feature_type': sub.GetTypeName2(),
                         'suppressed': attempt(record, 'mate.' + sub.Name, lambda: bool(sub.IsSuppressed()))}
                mate = wrap(sub.GetSpecificFeature2(), 'IMate2') if 'Mate' in entry['feature_type'] else None
                if mate is not None:
                    def read_mate():
                        entry['type'] = MATE_TYPES.get(mate.Type, mate.Type)
                        entry['alignment'] = mate.Alignment
                        entities = []
                        for i in range(mate.GetMateEntityCount()):
                            ent = wrap(mate.MateEntity(i), 'IMateEntity2')
                            comp = wrap(ent.ReferenceComponent, 'IComponent2')
                            ref = ent.ReferenceType2
                            entities.append({'component': comp.Name2 if comp else None,
                                             'reference_type': REF_TYPES.get(ref, ref),
                                             'params': list(ent.EntityParams or [])})
                        entry['entities'] = entities
                    attempt(record, 'mate_detail.' + sub.Name, read_mate)
                rows.append(entry)
                sub = wrap(sub.GetNextSubFeature(), 'IFeature')
        feature = wrap(feature.GetNextFeature(), 'IFeature')
    return rows


def mass_props(ext):
    mp = wrap(ext.CreateMassProperty2(), 'IMassProperty2')
    mp.UseSystemUnits = True
    mp.IncludeHiddenBodiesOrComponents = False
    return {'volume_m3': mp.Volume, 'surface_area_m2': mp.SurfaceArea, 'mass_kg': mp.Mass,
            'density_kg_m3': mp.Density, 'center_of_mass_m': list(mp.CenterOfMass)}


class Preferences:
    """Set SolidWorks user preferences for export and restore the originals afterwards."""

    def __init__(self, app):
        self.app = app
        self.saved = []

    def set_int(self, name, value):
        key = PREF_INT[name]
        self.saved.append(('int', name, key, self.app.GetUserPreferenceIntegerValue(key)))
        self.app.SetUserPreferenceIntegerValue(key, value)

    def set_toggle(self, name, value):
        key = PREF_TOGGLE[name]
        self.saved.append(('toggle', name, key, self.app.GetUserPreferenceToggle(key)))
        self.app.SetUserPreferenceToggle(key, value)

    def set_string(self, name, value):
        key = PREF_STRING[name]
        self.saved.append(('string', name, key, self.app.GetUserPreferenceStringValue(key)))
        self.app.SetUserPreferenceStringValue(key, value)

    def originals(self):
        return {name: value for _, name, _, value in self.saved}

    def restore(self):
        for kind, _, key, value in reversed(self.saved):
            if kind == 'int':
                self.app.SetUserPreferenceIntegerValue(key, value)
            elif kind == 'toggle':
                self.app.SetUserPreferenceToggle(key, value)
            else:
                self.app.SetUserPreferenceStringValue(key, value)
        self.saved = []


def export(ext, target):
    result = ext.SaveAs3(str(target), SAVE_CURRENT, SAVE_OPTS, None, None, 0, 0)
    ok, errors, warnings = result
    return {'file': str(target), 'returned': bool(ok),
            'errors': errors, 'error_flags': decode_flags(errors, SAVE_ERRORS),
            'warnings': warnings, 'warning_flags': decode_flags(warnings, SAVE_WARNINGS),
            'exists': target.exists(), 'bytes': target.stat().st_size if target.exists() else 0}


def run(record):
    app = wrap(win32com.client.Dispatch('SldWorks.Application'), 'ISldWorks')
    record['solidworks_pids_after_connect'] = sorted(sw_pids())
    app.Visible = False
    app.UserControl = False
    record['solidworks_revision'] = app.RevisionNumber()
    docs = app.GetDocuments()
    record['already_open_documents'] = 0 if docs is None else len(docs)
    if record['already_open_documents']:
        raise RuntimeError('SolidWorks already has documents open; refusing to touch a user session.')
    t0 = time.time()
    raw, errors, warnings = app.OpenDoc6(str(ASSEMBLY), DOC_ASSEMBLY, OPEN_OPTS, '', 0, 0)
    record['open'] = {'seconds': round(time.time() - t0, 1),
                      'errors': errors, 'error_flags': decode_flags(errors, LOAD_ERRORS),
                      'warnings': warnings, 'warning_flags': decode_flags(warnings, LOAD_WARNINGS)}
    model = wrap(raw, 'IModelDoc2')
    if model is None:
        raise RuntimeError('OpenDoc6 returned no document: %s' % record['open'])
    try:
        record['read_only'] = bool(model.IsOpenedReadOnly())
        if not record['read_only']:
            raise RuntimeError('Assembly was not opened read-only; aborting before any export.')
        assy = wrap(model, 'IAssemblyDoc')
        ext = wrap(model.Extension, 'IModelDocExtension')
        record['title'] = model.GetTitle()
        record['active_configuration'] = wrap(model.GetActiveConfiguration(), 'IConfiguration').Name
        record['configurations'] = list(model.GetConfigurationNames() or [])
        record['length_unit'] = model.LengthUnit
        record['resolve_lightweight'] = attempt(record, 'resolve', lambda: assy.ResolveAllLightWeightComponents(False))
        record['component_count_all'] = assy.GetComponentCount(False)
        record['component_count_top'] = assy.GetComponentCount(True)
        record['components'] = component_rows(record, assy)
        record['assembly_box_m'] = attempt(record, 'assembly_box', lambda: list(assy.GetBox(0)))
        record['mass_properties_visible'] = attempt(record, 'mass', lambda: mass_props(ext))
        record['mates'] = mate_rows(record, model)
        record['unsaved_changes_after_open'] = bool(model.GetSaveFlag())
        prefs = Preferences(app)
        try:
            prefs.set_int('swParasolidOutputVersion', PARASOLID_VERSION)
            prefs.set_toggle('swXTAssemSaveFormat', True)
            prefs.set_string('swExportOutputCoordinateSystem', '')
            record['export_preferences_original'] = prefs.originals()
            record['exports'] = [export(ext, EXPORT_DIR / 'AUVM.x_t')]
            prefs.set_int('swStepAP', 214)
            record['exports'].append(export(ext, EXPORT_DIR / 'AUVM.step'))
        finally:
            prefs.restore()
            record['preferences_restored'] = True
        record['unsaved_changes_before_close'] = bool(model.GetSaveFlag())
    finally:
        app.CloseDoc(model.GetTitle())
        record['closed_document'] = True
    return app


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    record = {'started': datetime.now().isoformat(), 'assembly': str(ASSEMBLY)}
    sources = sorted(p for p in SRC_DIR.iterdir() if p.is_file())
    before = {p.name: sha256(p) for p in sources}
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    for name in ('AUVM.x_t', 'AUVM.step'):
        if (EXPORT_DIR / name).exists():
            raise FileExistsError(EXPORT_DIR / name)
    pids_before = sw_pids()
    record['solidworks_pids_before'] = sorted(pids_before)
    if pids_before:
        raise RuntimeError('SolidWorks is already running (%s); close it first.' % sorted(pids_before))
    pythoncom.CoInitialize()
    app = None
    try:
        app = run(record)
    except Exception:
        record['error'] = traceback.format_exc()
    finally:
        if app is not None:
            attempt(record, 'exit_app', app.ExitApp)
            app = None
        pythoncom.CoUninitialize()
        time.sleep(10)
        leftover = sw_pids() - pids_before
        for pid in leftover:  # only processes this script started
            subprocess.run(['taskkill', '/PID', str(pid), '/F'], capture_output=True)
        record['solidworks_pids_force_stopped'] = sorted(leftover)
        record['solidworks_pids_remaining'] = sorted(sw_pids() - pids_before)
    after = {p.name: sha256(p) for p in sources}
    record['source_unchanged'] = before == after
    record['source_sha256'] = before
    record['lock_files_left'] = sorted(p.name for p in SRC_DIR.iterdir() if p.name.startswith('~$'))
    record['finished'] = datetime.now().isoformat()
    out = RECORD_DIR / ('sw_export_record_%s.json' % stamp)
    out.write_text(json.dumps(record, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
    summary = {k: record.get(k) for k in (
        'solidworks_revision', 'open', 'read_only', 'active_configuration', 'configurations', 'length_unit',
        'component_count_all', 'component_count_top', 'mass_properties_visible', 'exports',
        'preferences_restored', 'source_unchanged', 'lock_files_left',
        'solidworks_pids_force_stopped', 'solidworks_pids_remaining', 'step_errors', 'error')}
    summary['mates'] = len(record.get('mates') or [])
    print(json.dumps(summary, ensure_ascii=False, indent=2, default=str))
    print('RECORD:', out)
    if record.get('error') or not record['source_unchanged'] or record['solidworks_pids_remaining']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

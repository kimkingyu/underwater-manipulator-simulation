"""Package and verify the staged data, MATLAB assets and native Workbench project."""
from pathlib import Path
from datetime import datetime
import hashlib
import json
import re
import sys
import zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'deliverables/basic_models_build/PackageProject'
OUT = ROOT / 'deliverables'
PREFIX = 'UnderwaterArm_Basic/'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    project = STAGE / 'Workbench/UnderwaterArm_Base.wbpj'
    archive = STAGE / 'Workbench/UnderwaterArm_Base.wbpz'
    case = STAGE / 'Workbench/UnderwaterArm_Base_files/import_files/UnderwaterArm_Base_415k.cas.h5'
    setup_record = json.loads((STAGE / 'verification/fluent_base_setup_validation.json').read_text(encoding='utf-8'))
    matlab_record = json.loads((STAGE / 'verification/matlab_model_validation.json').read_text(encoding='utf-8'))
    assert setup_record['iterations_executed'] == 0
    assert setup_record['flow_time_after'] == 0.0
    assert setup_record['output_sha256'] == sha(case)
    assert matlab_record['movable_joints'] == 3 and not matlab_record['warning_text']
    assert 'BUILD_COMPLETE' in (STAGE / 'verification/wb_build_log.txt').read_text(encoding='utf-8')
    assert 'RESTORE_AND_REOPEN_COMPLETE' in (STAGE / 'verification/wb_restore_log.txt').read_text(encoding='utf-8')
    locations = []
    for element in ET.parse(project).iter('member-data'):
        if not element.text:
            continue
        match = re.search(r'"Location":\s*r?"([^"]*)"', element.text)
        if not match:
            continue
        raw = match.group(1)
        path = (project.parent / raw.replace('$(ProjectName)', project.stem).replace('\\', '/')).resolve()
        if not path.is_relative_to(STAGE) or not path.is_file():
            raise RuntimeError('Nonportable or missing Workbench reference: ' + raw)
        locations.append({'stored_location': raw, 'relative_path': path.relative_to(STAGE).as_posix(),
                          'bytes': path.stat().st_size, 'sha256': sha(path)})
    assert any(x['relative_path'].endswith('.cas.h5') for x in locations)
    meshes = sorted({n.attrib['filename'] for n in ET.parse(STAGE / 'model/robot.urdf').iter('mesh')})
    missing_meshes = [name for name in meshes if not (STAGE / 'model' / name).is_file()]
    if missing_meshes:
        raise RuntimeError('Missing URDF assets: ' + str(missing_meshes))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        case_entries = [name for name in z.namelist() if name.endswith('.cas.h5')]
        assert len(case_entries) == 1, 'Unexpected Case count in native archive'
        assert hashlib.sha256(z.read(case_entries[0])).hexdigest() == sha(case)
    portability = {
        'workbench_version_tested': '2024 R2',
        'native_save_archive_restore_and_reopen': True,
        'resolved_project_references': locations,
        'all_runtime_file_references_inside_package': True,
        'native_wbpz_case_matches_restored_case': True,
        'urdf_referenced_mesh_count': len(meshes), 'missing_urdf_meshes': missing_meshes,
        'matlab_version_tested': matlab_record['matlab_version'],
        'matlab_urdf_import_and_mass_matrix_passed': True,
        'iterations_executed': 0,
        'limitations': 'No mesh repair, flow solution, dynamic-UDF validation or full MATLAB pipeline rerun. Historical script paths may require changes.',
    }
    (STAGE / 'verification/package_portability_validation.json').write_text(
        json.dumps(portability, ensure_ascii=False, indent=2), encoding='utf-8')
    files = sorted(p for p in STAGE.rglob('*') if p.is_file() and
                   p.name not in {'.project_cache', '文件清单_SHA256.json'} and
                   'session_files' not in p.parts and '__pycache__' not in p.parts)
    for p in files:
        if p.is_symlink():
            raise RuntimeError('Symlink not permitted in package: ' + str(p))
    matrix52 = [p for p in files if '/cfd/parametric_cfd_matrix_52/' in p.as_posix() and p.name == 'moments.csv']
    matrix9 = [p for p in files if '/cfd/parametric_cfd_matrix/' in p.as_posix() and p.name == 'moments.csv']
    assert len(matrix52) == 52 and len(matrix9) == 9
    now = datetime.now().astimezone()
    manifest = {'created_at': now.isoformat(), 'package_type': 'data plus basic Fluent/MATLAB models and native Workbench',
                'source_count': len(files), 'source_bytes': sum(p.stat().st_size for p in files),
                'matrix52_csv_count': 52, 'historical9_csv_count': 9,
                'verification': portability, 'files': []}
    filename = '水下机械臂_数据与基础模型_Workbench版_' + now.strftime('%Y%m%d_%H%M%S') + '.zip'
    target = OUT / filename
    partial = OUT / (filename + '.partial')
    with zipfile.ZipFile(partial, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in files:
            rel = p.relative_to(STAGE).as_posix()
            before = p.stat()
            data = p.read_bytes()
            after = p.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Staged file changed: ' + rel)
            digest = hashlib.sha256(data).hexdigest()
            manifest['files'].append({'path': rel, 'bytes': len(data), 'sha256': digest})
            z.writestr(PREFIX + rel, data)
        z.writestr(PREFIX + '文件清单_SHA256.json', json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8'))
    with zipfile.ZipFile(partial) as z:
        assert z.testzip() is None
        assert len(z.namelist()) == len(set(z.namelist())) == len(files) + 1
        assert json.loads(z.read(PREFIX + '文件清单_SHA256.json')) == manifest
        for item in manifest['files']:
            data = z.read(PREFIX + item['path'])
            if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
                raise RuntimeError('Archive verification failed: ' + item['path'])
            if sha(STAGE / item['path']) != item['sha256']:
                raise RuntimeError('Staged file changed after packing: ' + item['path'])
    partial.rename(target)
    result = {'archive': target.relative_to(ROOT).as_posix(), 'bytes': target.stat().st_size,
              'MB': round(target.stat().st_size / 1e6, 2), 'MiB': round(target.stat().st_size / 2**20, 2),
              'file_count': len(files) + 1, 'sha256': sha(target),
              'workbench_archive': archive.relative_to(ROOT).as_posix(),
              'workbench_portability_passed': True, 'matlab_model_import_passed': True,
              'zip_crc_and_sha256_passed': True}
    with (OUT / (filename + '.verification.json')).open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

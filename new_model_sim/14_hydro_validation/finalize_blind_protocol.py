"""Create the one-time runnable blind protocol after research and qe preparation pass.
Does not launch Fluent. Refuses to overwrite an existing protocol or authorize after new history exists.
"""
import datetime
import hashlib
import json
from pathlib import Path

import evaluate_blind as ev

ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    destination = ROOT/'frozen/blind_protocol.json'
    if destination.exists():
        raise FileExistsError('Protocol already exists; verify instead of overwriting')
    run_root = ROOT/'cfd/runs'
    if run_root.exists() and any(run_root.rglob('history.json')):
        raise RuntimeError('New histories already exist: cannot claim a prior freeze')
    research = read(ROOT/'frozen/research_model_manifest.json')
    for record in research['files']:
        if digest(ROOT/record['path']) != record['sha256']:
            raise RuntimeError('Research snapshot changed: '+record['path'])
    config_path = ROOT/'cfd/validation_cases.json'
    cfg = ev.runner.load_config(config_path)
    frozen_design = read(ROOT/'frozen/case_design.json')
    design = {k:cfg[k] for k in ('origin_m','physics','solver','cases')}
    design['poses'] = {k:{'q':v['q']} for k,v in cfg['poses'].items()}
    frozen_design.pop('meaning',None)
    if design != frozen_design:
        raise RuntimeError('Prospective physical design changed since prior predictions')
    probe = read(ROOT/'cfd/qe/probe_pose_qe.json')
    water = read(ROOT/'cfd/qe/fluid_domain_qe.json')
    poly = read(ROOT/'cfd/qe/poly_qe_v3.json')
    base = read(ROOT/'cfd/bqe_base.json')
    api = read(ROOT/'cfd/runtime_api_probe.json')
    if probe.get('status')!='ok' or probe.get('passed') is not True:
        raise RuntimeError('qe interference probe not passed')
    if water.get('status')!='ok' or not water['water']['closed'] or not water['water']['manifold'] or water.get('unassigned_faces'):
        raise RuntimeError('qe water topology/naming not passed')
    quality = poly.get('poly_improved',{})
    if poly.get('status')!='ready_uninitialized' or quality.get('min_orth',0)<0.01 or quality.get('negative_volume') is not False or quality.get('errors') or not quality.get('mesh_check_complete'):
        raise RuntimeError('qe poly quality gate not passed')
    if base.get('status')!='ready_uninitialized' or base.get('failed_required') or base.get('flow_initialized') or base.get('flow_solved'):
        raise RuntimeError('qe base setup gate failed')
    if api.get('status')!='api_compatible_no_initialization_no_solve' or api.get('failed_required') or api.get('flow_initialized') or api.get('flow_solved') or api.get('flow_loads_computed'):
        raise RuntimeError('Real API preflight gate failed')
    for pose,info in cfg['poses'].items():
        case = (ROOT/info['base_case']).resolve()
        if not case.is_file() or digest(case)!=info.get('base_sha256'):
            raise RuntimeError('Base case missing/hash mismatch: '+pose)
    files = set(ev.REQUIRED_FROZEN)
    files.update(x['path'] for x in research['files'])
    files.update(['frozen/research_model_manifest.json','cfd/run_validation.py',
                  'cfd/bqe.cas.h5','cfd/bqe_base.json','cfd/runtime_api_probe.json',
                  'cfd/parent_quality_disposition.json','cfd/qe/probe_pose_qe.json',
                  'cfd/qe/fluid_domain_qe.json','cfd/qe/poly_qe_v3.json',
                  'finalize_blind_protocol.py'])
    records = [{'path':path,'sha256':digest(ROOT/path)} for path in sorted(files)]
    manifest = {
        'schema_version':1,'status':'frozen_before_cfd',
        'frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'research_snapshot_sha256':digest(ROOT/'frozen/research_model_manifest.json'),
        'case_config_sha256':digest(config_path),
        'approved_case_ids':[c['id'] for c in cfg['cases']],
        'files':records,
        'models_are_research_candidates_not_default_replacements':True,
        'steady_development_winner':None,
        'default_theta_unchanged':True,
        'qe_quality':quality,
        'new_blind_histories_existed_at_freeze':False,
        'post_truth_parameter_or_architecture_tuning_allowed':False,
        'scope_note':'8 prescribed steady translations and 1 locked-arm q0 diagonal whole-domain oscillation; no independent-joint-motion validation',
    }
    with destination.open('x',encoding='utf-8') as stream:
        json.dump(manifest,stream,ensure_ascii=False,indent=2,allow_nan=False)
    ev.verify_evaluation_freeze(ROOT,destination)
    print('Runnable protocol frozen and verified:',len(manifest['approved_case_ids']),'cases,',len(records),'hashed dependencies')
    print('qe min orth:',quality['min_orth'],'protocol SHA256:',digest(destination))


if __name__=='__main__': main()

"""Freeze research candidates and prospective predictions, without authorizing CFD.
This produces research_model_manifest.json, NOT blind_protocol.json. The latter additionally
requires verified qe geometry/mesh/base and a complete final CFD-config hash.
"""
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from model_components import predict_split_hydro, frozen87_steady, jsonable

ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump_new(path, obj):
    if path.exists():
        raise FileExistsError('Refusing to overwrite a frozen artifact: '+str(path))
    path.write_text(json.dumps(jsonable(obj),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def main():
    final = ROOT/'frozen/research_model_manifest.json'
    if final.exists():
        raise FileExistsError('Research snapshot already exists; inspect it instead of overwriting')
    runs = ROOT/'cfd/runs'
    if runs.exists() and any(runs.rglob('history.json')):
        raise RuntimeError('Prospective history already exists; do not pretend this is a prior freeze')
    config = read(ROOT/'cfd/validation_cases.json')
    models = {name:read(ROOT/'models'/('candidate_'+name+'_strict37.json')) for name in ('A','B','B_pair')}
    old = read(ROOT/'frozen/theta87_baseline.json')
    design = {k:config[k] for k in ('origin_m','physics','solver','cases')}
    design['poses'] = {k:{'q':v['q']} for k,v in config['poses'].items()}
    design['meaning'] = 'Prospective physical inputs fixed; base-file identity waits for verified qe preparation.'
    predictions=[]
    for case in config['cases']:
        if case['mode']!='steady_translation': continue
        nu=np.r_[case['body_velocity_m_s'],[0.,0.,0.]]
        row={'id':case['id'],'pose_id':case['pose_id'],'q':case['q'],'nu':nu.tolist(),
             'role':'prospective research predictions, not CFD truth or a claim of improvement',
             'geometry_verified':case['pose_id']=='q0','models':{}}
        for name,model in models.items():
            pred=predict_split_hydro(model,case['q'],nu)
            assert np.isfinite(np.asarray(pred['generalized'])).all()
            row['models'][name]=pred
        row['models']['legacy87']=frozen87_steady(old['theta'],case['q'],nu,models['A']['geometry'])
        predictions.append(row)
    assert len(predictions)==8
    dump_new(ROOT/'frozen/case_design.json',design)
    dump_new(ROOT/'frozen/steady_prior_predictions.json',{'cases':predictions,'default_model_changed':False,
        'selection_note':'No steady candidate passed the development acceptance gate. A is the matched reference; B and B_pair are labelled research challengers, not approved replacements. C excluded due development identifiability gaps.'})
    dynamic_source=ROOT/'reports/dynamic_oblique_yz_design_prediction.json'
    dynamic=read(dynamic_source)
    assert dynamic['id']=='O_V_q0_YZ' and len(dynamic['time_s'])==240
    dump_new(ROOT/'frozen/oscillation_prior_predictions.json',dynamic)
    files=[
        'contract.json','data/component_dataset.json','data_pipeline.py','model_components.py','fit_candidates.py','dynamics_linear.py',
        'frozen/source_manifest.json','frozen/theta87_baseline.json','frozen/theta87_baseline.mat','frozen/evaluation_protocol.json',
        'frozen/case_design.json','frozen/steady_prior_predictions.json','frozen/oscillation_prior_predictions.json',
        'models/candidate_A_strict37.json','models/candidate_B_strict37.json','models/candidate_B_pair_strict37.json','models/dynamic_linear_q0.json',
        'matlab/predict_split_hydro.m','matlab/predict_dynamic_linear.m','prepare_research_freeze.py']
    manifest={'schema_version':1,'status':'models_frozen_waiting_for_qe_preparation_and_license',
              'frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'source_git_commit':read(ROOT/'frozen/source_manifest.json').get('git_commit',read(ROOT/'contract.json')['source_git_commit']),
              'steady_development_winner':None,'default_theta_replacement_authorized':False,
              'prospective_cases':[x['id'] for x in config['cases']],
              'new_blind_loads_generated':False,
              'files':[{'path':p,'sha256':digest(ROOT/p)} for p in files],
              'pending_before_runnable_blind_protocol':['verified qe CAD/water/mesh/base','available authorized Fluent license','final CFD config and runner hashes'],
              'scope':'Research snapshot only. Not sufficient to launch run_validation; do not relabel as independent validation.'}
    dump_new(final,manifest)
    print('Research models/prior predictions frozen:',len(predictions),'steady + 1 oscillation; files',len(files))
    print('CFD authorization remains absent; default theta unchanged.')


if __name__=='__main__': main()

"""Read-only diagnostic of power redistribution across CFD components.
Writes only reports/component_power_audit.json; never rejects CFD from per-body positive work.
"""
import json
from pathlib import Path
import numpy as np
from data_pipeline import body_jacobians

ROOT = Path(__file__).resolve().parent

def audit(dataset):
    rows = []
    for case in dataset['steady']:
        J = body_jacobians(case['q'], dataset['reference'])
        zeta = np.r_[case['nu'], [0., 0., 0.]]
        twists = J @ zeta
        power = np.sum(twists * np.asarray(case['wrenches']), axis=1)
        total_power = float(np.asarray(case['nu']) @ np.asarray(case['total']))
        closure = float(abs(np.sum(power) - total_power))
        assert closure < 1e-10 * max(1., abs(total_power))
        rows.append({'id': case['id'], 'strict': case['strict'],
                     'component_power_W': dict(zip([b['id'] for b in dataset['bodies']], power.tolist())),
                     'total_fluid_power_W': total_power, 'closure_W': closure})
    positives = [{'id':r['id'],'body':b,'power_W':p} for r in rows if r['strict']
                 for b,p in r['component_power_W'].items() if p > 1e-8]
    return {'meaning':'Fluid power on prescribed moving components; rigidly linked components may exchange power through the interacting fluid.',
            'positive_component_power_is_not_bad_CFD':True,
            'independent_component_passivity_is_a_candidate_assumption_not_a_universal_data_rule':True,
            'strict_positive_component_events':positives,
            'strict_total_power_max_W':max(r['total_fluid_power_W'] for r in rows if r['strict']),
            'strict_total_power_min_W':min(r['total_fluid_power_W'] for r in rows if r['strict']),
            'max_power_closure_W':max(r['closure_W'] for r in rows), 'rows':rows}

if __name__ == '__main__':
    dataset = json.loads((ROOT/'data/component_dataset.json').read_text(encoding='utf-8'))
    result = audit(dataset)
    (ROOT/'reports/component_power_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('strict positive component events:', len(result['strict_positive_component_events']))
    print('max closure W:', result['max_power_closure_W'])
    print('strict total power range W:',result['strict_total_power_min_W'],result['strict_total_power_max_W'])

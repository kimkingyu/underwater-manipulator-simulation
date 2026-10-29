"""Verify the reviewed export and portable copies; no solver, fitting or source writes."""
import csv
import hashlib
import importlib.util
import json
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np

sys.dont_write_bytecode = True
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
import export_results as ex


class ExportResultsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle=ex.load_json(ROOT/'reports/validation_bundle.json')
        cls.score=ex.load_json(ROOT/'reports/evaluator_blind_scores.json')
        cls.package=ROOT/'handoff'
        cls.html=(cls.package/'index.html').read_text(encoding='utf-8')

    def csv_rows(self,name):
        with (self.package/'data'/name).open(encoding='utf-8-sig',newline='') as f:
            return list(csv.DictReader(f))

    def test_counts_capped_direction_and_no_default_replacement(self):
        summary=self.bundle['summary']
        self.assertEqual(summary['overall_status'],'inconclusive')
        self.assertEqual(summary['strict_steady_count'],6)
        self.assertEqual(summary['capped_steady_count'],2)
        self.assertEqual(summary['independent_oscillation_count'],1)
        self.assertEqual(sorted(summary['capped_ids']),['T_V_q0_YZ_p055','T_V_qe_YZ_p055'])
        self.assertFalse(summary['default_theta_replaced'])
        self.assertFalse(summary['full_nine_channel_validation_claim'])

    def test_first_screen_discloses_degradation_and_startup_maxima(self):
        first=self.html.split('<h2>数据与用法')[0]
        for text in ('INCONCLUSIVE','6 strict / 2 capped / 1','T_V_q0_YZ_p055','T_V_qe_YZ_p055',
                     '5.45','42.54/42.88/42.96','5.22','37.67','17.20','不替换','代表性受限',
                     '初始条件','离散','流场历史','尚未辨别','纯hull旧87没有独立基线'):
            self.assertIn(text,first)

    def test_eight_and_240_csv_rows_and_roundtrip_precision(self):
        steady=self.csv_rows('validation_steady.csv')
        dynamic=self.csv_rows('validation_oscillation.csv')
        self.assertEqual(len(steady),8)
        self.assertEqual(len(dynamic),240)
        for i,row in enumerate(steady):
            expected=ex.fields_for_loads('CFD',self.bundle['steady']['truth'],i)
            for model,preds in self.bundle['steady']['predictions'].items():
                expected.update(ex.fields_for_loads(model,preds,i))
            for key,value in expected.items():
                self.assertEqual(float(row[key]),float(value))
        for i,row in enumerate(dynamic):
            expected=ex.fields_for_loads('CFD',self.bundle['oscillation']['truth'],i)
            for model,preds in self.bundle['oscillation']['predictions'].items():
                expected.update(ex.fields_for_loads(model,preds,i))
            for key,value in expected.items():
                self.assertEqual(float(row[key]),float(value))
        self.assertNotIn('legacy87_hull_Fx_N',steady[0])
        self.assertNotIn('legacy87_base0_Fx_N',dynamic[0])
        self.assertIn('legacy87_hull_plus_base0_Fx_N',dynamic[0])

    def test_metrics_populations_windows_and_old87_present(self):
        rows=self.csv_rows('validation_metrics.csv')
        self.assertEqual(len(rows),804)
        for population,count in (('strict_only',6),('strict_plus_capped_sensitivity',8)):
            part=[r for r in rows if r['domain']=='steady' and r['window']==population]
            self.assertEqual({int(r['case_count']) for r in part},{count})
            for group in ('total6','joint3'):
                self.assertTrue(any(r['model']=='legacy87' and r['group']==group for r in part))
            self.assertFalse(any(r['model']=='legacy87' and r['group'] in ('hull_force3','hull_moment3') for r in part))
        for window,count in (('full',240),('startup_first_cycle',80),('last_two_cycles',160),('cycle2',80),('cycle3',80),('ramp',20)):
            part=[r for r in rows if r['domain']=='oscillation' and r['window']==window]
            self.assertTrue(part)
            self.assertEqual({int(r['sample_count']) for r in part},{count})

    def test_saved_dynamic_predictions_reproduce_true_metrics(self):
        o=self.bundle['oscillation']
        mask=np.asarray(o['masks']['last_two_cycles'],bool)
        for model in ('dynamic_linear','legacy87'):
            for key,group in (('total','total6'),('joint_tau','joint3')):
                truth=np.asarray(o['truth'][key])[mask]
                predicted=np.asarray(o['predictions'][model][key])[mask]
                rmse=np.sqrt(np.mean((predicted-truth)**2,axis=0))
                expected=self.score['oscillations'][0]['metrics'][model]['last_two_cycles'][group]['RMSE']
                np.testing.assert_allclose(rmse,expected,rtol=0,atol=1e-12)
        new=self.score['oscillations'][0]['metrics']['dynamic_linear']['last_two_cycles']
        np.testing.assert_allclose(new['total6']['RMSE'][:3],[.005563766529051293,.05016497393187429,.04297355940576182],rtol=0,atol=1e-15)
        np.testing.assert_allclose(new['joint3']['RMSE'],[3.9430512699448557e-5,.0004901051430013069,.00033801735067659966],rtol=0,atol=1e-15)

    def test_actual_moving_ob_and_scope_are_preserved(self):
        raw=next(r for r in self.score['collected_records'] if r['mode']=='oscillation_translation')
        exported=self.bundle['oscillation']
        np.testing.assert_array_equal(exported['moving_origin_m'],raw['moving_origin_m'])
        np.testing.assert_array_equal(exported['nu'],raw['nu'])
        np.testing.assert_array_equal(exported['nudot'],raw['nudot'])
        self.assertIn('history',exported['reference_source'])
        self.assertEqual(exported['viscous_model'],'laminar')
        self.assertFalse(self.bundle['summary']['pure_hull_legacy_prediction_available'])
        self.assertIn('不是任意运动',self.html)
        self.assertIn('完整9×9',self.html)

    def test_harmonics_are_physical_same_actual_frequency(self):
        rows=self.csv_rows('validation_harmonics.csv')
        self.assertEqual(len(rows),234)
        w=self.bundle['oscillation']['actual_motion_law']['angular_frequency_rad_s']
        self.assertEqual({float(r['actual_angular_frequency_rad_s']) for r in rows},{w})
        self.assertEqual({r['window'] for r in rows},{'full','startup_first_cycle','last_two_cycles'})
        self.assertEqual({r['unit'] for r in rows},{'N','N*m'})
        self.assertTrue(all('actual_first_sin' in r and 'predicted_second_cos' in r for r in rows))
        self.assertEqual(len(self.csv_rows('validation_cycle_repeatability.csv')),45)

    def test_readback_is_two_pairs_not_nine(self):
        r=self.bundle['readback']
        self.assertEqual(len(r['cases']),2)
        self.assertFalse(r['all_nine_restarted'])
        self.assertFalse(r['initialization_called'])
        self.assertFalse(r['iterations_called'])
        self.assertTrue(r['all_inputs_unchanged_at_end'])
        self.assertEqual([c['passing_channels'] for c in r['cases']],[57,57])
        self.assertLess(r['cases'][0]['maximum_absolute_error'],5e-13)
        self.assertLess(r['cases'][1]['maximum_absolute_error'],1.4e-14)
        index=self.bundle['case_index']
        self.assertEqual(sum(c['readback_pair_verified'] for c in index),2)

    def test_case_index_is_relative_and_no_large_artifacts_bundled(self):
        rows=self.bundle['case_index']
        self.assertEqual(len(rows),9)
        self.assertEqual(len(self.csv_rows('validation_case_index.csv')),9)
        for row in rows:
            for item in list(row['artifacts'].values())+[row['result']]:
                self.assertFalse(Path(item['stage_relative_path']).is_absolute())
                self.assertTrue(item['stage_relative_path'].startswith('cfd/runs/'))
                self.assertFalse(item['included'])
                self.assertEqual(len(item['sha256']),64)
        for path in self.package.rglob('*'):
            if path.is_file():
                self.assertFalse(path.name.endswith(('.cas.h5','.dat.h5','.msh','.scdoc','.zip','.xlsx','.md')))

    def test_original_model_bytes_and_flags_are_unchanged(self):
        for name,filename in ex.STATIC_FILES.items():
            self.assertEqual(ex.sha256(ROOT/'models'/filename),ex.sha256(self.package/'models'/filename))
            copied=ex.load_json(self.package/'models'/filename)
            self.assertEqual(copied['freeze_status'],'candidate_only_parent_controls_freeze')
            self.assertEqual(copied['scope'],ex.load_json(ROOT/'models'/filename)['scope'])
        self.assertEqual(ex.sha256(ROOT/'models/dynamic_linear_q0.json'),ex.sha256(self.package/'models/dynamic_linear_q0.json'))
        self.assertEqual(ex.sha256(ROOT/'frozen/theta87_baseline.json'),ex.sha256(self.package/'models/theta87_baseline.json'))
        dynamic=ex.load_json(self.package/'models/dynamic_linear_q0.json')
        self.assertFalse(dynamic['scope']['blind_validated'])
        self.assertFalse(dynamic['scope']['independent_case_validated'])

    def test_parameter_counts_are_schema_derived(self):
        inventory={r['id']:r for r in self.bundle['model_inventory']}
        self.assertEqual({k:r['parameter_count'] for k,r in inventory.items()},{'A':45,'B':60,'B_pair':66,'dynamic_q0':360,'legacy87':87})
        for name,filename in ex.STATIC_FILES.items():
            self.assertEqual(inventory[name]['parameter_count'],len(ex.load_json(self.package/'models'/filename)['theta']))
        d=ex.load_json(self.package/'models/dynamic_linear_q0.json')
        self.assertEqual(inventory['dynamic_q0']['parameter_count'],len(d['parameter_metadata']))
        self.assertEqual(inventory['dynamic_q0']['M_rows_shape'],[30,6])
        self.assertEqual(inventory['dynamic_q0']['M_total_shape'],[6,6])

    def test_actual_svg_points_and_no_generative_data(self):
        ns={'s':'http://www.w3.org/2000/svg'}
        for filename,curves in (('validation_dynamic_total_joint.svg',27),('validation_dynamic_hull.svg',12)):
            root=ET.parse(self.package/'data'/filename).getroot()
            polys=root.findall('.//s:polyline',ns)
            self.assertEqual(len(polys),curves)
            self.assertTrue(all(len(p.get('points').split())==240 for p in polys))

    def test_matlab_sources_have_no_workspace_absolute_dependency(self):
        for path in (self.package/'matlab').rglob('*.m'):
            text=path.read_text(encoding='utf-8')
            self.assertNotIn('D:\\work4',text)
            self.assertNotIn('D:/work4',text)
        example=(self.package/'matlab/read_validation_example.m').read_text()
        self.assertIn('physical=-Y*theta',example)
        self.assertIn('out=predict_dynamic_linear',example)
        self.assertNotIn('out=-predict_dynamic_linear',example)
        self.assertIn('max',example)

    def test_temporary_renamed_copy_readable_and_hash_identical(self):
        manifest=ex.load_json(self.package/'package_manifest.json')
        with tempfile.TemporaryDirectory(prefix='stage14_export_test_') as temp:
            copied=Path(temp)/'renamed_package'
            shutil.copytree(self.package,copied)
            bundle=ex.load_json(copied/'data/validation_bundle.json')
            self.assertEqual(bundle['summary']['overall_status'],'inconclusive')
            for entry in manifest['files']:
                self.assertEqual(ex.sha256(copied/entry['path']),entry['sha256'])
            self.assertTrue((copied/'matlab/read_validation_example.m').is_file())

    def test_matlab_and_portability_checks_are_real_pass_records(self):
        for filename in ('validation_matlab_checks.json','validation_portability_checks.json'):
            report=ex.load_json(ROOT/'reports'/filename)
            self.assertEqual(report['status'],'PASS')
            self.assertLess(report['maximum_absolute_error'],1e-10)
            self.assertEqual(report['static_cases_checked'],8)
            self.assertEqual(report['oscillation_samples_checked'],240)
            self.assertTrue(report['portable_inference_paths_verified'])
            self.assertFalse(report['default_theta_replaced'])
        self.assertTrue(ex.load_json(ROOT/'reports/validation_portability_checks.json')['tested_after_temporary_copy'])
        self.assertTrue((self.package/'data/validation_bundle.mat').is_file())

    def test_source_integrity_and_write_scope(self):
        report=ex.load_json(ROOT/'reports/validation_source_integrity.json')
        self.assertTrue(report['all_unchanged'])
        self.assertEqual(report['source_files_verified'],292)
        for name in ('blind_protocol.json','research_model_manifest.json','source_manifest.json','evaluation_protocol.json'):
            self.assertEqual(ex.sha256(ROOT/'frozen'/name),ex.sha256(self.package/'provenance'/name))
        with self.assertRaises(ValueError):
            ex.ensure_owned(ROOT/'models/not_allowed.json')
        with self.assertRaises(ValueError):
            ex.full_number(float('nan'))


if __name__=='__main__':
    unittest.main()

"""Entirely offline fourth-attempt protocol, export and fake execution coverage."""
import ast
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import mesh_construction as mesh
import definition_probe as probe
import preflight


class MeshConstructionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'stage'
        self.root.mkdir()
        (self.root / 'frozen').mkdir()
        self.attempt = self.root / mesh.ATTEMPT_RELATIVE
        self.attempt.mkdir(parents=True)
        (self.root / 'runs/mesh').mkdir()
        self.auth_path = self.root / mesh.AUTHORIZATION_RELATIVE
        self.auth_path.write_text(json.dumps(mesh.AUTHORIZATION_VALUES))
        self.baseline = self.root / mesh.AUTHORIZATION_VALUES['baseline_authorization_relative']
        self.baseline.write_text('{}')
        for path in mesh.source_paths(self.root):
            path.parent.mkdir(parents=True, exist_ok=True)
            original = STAGE / path.relative_to(self.root) if self.root in path.parents else None
            if original is not None and original.is_file() and path.suffix == '.in':
                shutil.copyfile(original, path)
            else:
                path.write_text('# offline fixture source\n')
        shutil.copyfile(STAGE / 'contract.json', self.root / 'contract.json')
        self.cad = {}
        for name in ('background', 'arm'):
            path = self.root / (name + '.scdoc')
            path.write_bytes(('fake CAD ' + name).encode())
            self.cad[name] = {'path': str(path), 'sha256': preflight.sha256(path)}
        verification = self.root / 'verify.json'
        verification.write_text('{}')
        self.cad['cad_verification'] = {'path': str(verification), 'sha256': preflight.sha256(verification)}
        self.frozen = {'contract': preflight.load_contract(self.root / 'contract.json'),
                       'protected_inputs': {'status': 'passed'}, 'cad': self.cad}
        real_digest = probe.digest
        self.digest = patch.object(probe, 'digest', side_effect=lambda path, root:
            mesh.AUTHORIZATION_VALUES['baseline_authorization_sha256'] if Path(path) == self.baseline else real_digest(path, root))
        self.digest.start()
        self.created = time.time()
        self.request = {'schema_version': 1, 'operation': mesh.OPERATION, 'job_id': mesh.JOB_ID,
            'created_unix': self.created, 'deadline_unix': self.created + 7200,
            'work_deadline_unix': self.created + 7170, 'authorization': mesh.validate_authorization(self.root),
            'sources': [{'path': str(p), 'size': p.stat().st_size, 'sha256': preflight.sha256(p)} for p in mesh.source_paths(self.root)]}
        self.save_request()
        ledger = {'job_id': mesh.JOB_ID, 'pid': os.getpid(), 'creation_time': mesh.psutil.Process(os.getpid()).create_time(),
                  'recorded_unix': time.time()}
        (self.attempt / 'launcher.json').write_text(json.dumps(ledger))

    def tearDown(self):
        self.digest.stop()
        self.tmp.cleanup()

    def save_request(self):
        (self.attempt / 'request.json').write_text(json.dumps(self.request))

    def exports(self, attempt=None, quality=None):
        attempt = attempt or self.root / mesh.MESH_ATTEMPT_RELATIVE
        attempt.mkdir(parents=True, exist_ok=True)
        systems = []
        for index, domain in enumerate(('background', 'arm')):
            name = 'FFF' if index == 0 else 'FFF-1'
            mech = attempt / 'stage15_two_domains_files/dp0' / name / 'MECH'
            mech.mkdir(parents=True)
            ext = '.msh' if index == 0 else '.msh.h5'
            source = mech / (name + ext)
            source.write_bytes(('fake native mesh ' + domain).encode())
            target = attempt / (domain + ext)
            shutil.copyfile(source, target)
            systems.append({'domain': domain, 'system_name': name,
                'mesh_export': {'source_path': str(source), 'path': str(target), 'size': target.stat().st_size}})
            metric = {'status': 'unavailable'} if quality is None else {'minimum': quality,
                'source': 'Ansys Meshing MeshMetricType.OrthogonalQuality'}
            stats = {'domain': domain, 'mesh_generated': True, 'status': 'generated_pending_export_and_fluent_checks',
                     'nodes': 100, 'elements': 200, 'quality': {'orthogonal_quality': metric}}
            (attempt / (domain + '_meshing_stats.json')).write_text(json.dumps(stats))
        completion = {'status': 'completed_pending_mesh_inventory', 'mesh_exports_written': True, 'systems': systems}
        (attempt / 'workbench_completion.json').write_text(json.dumps(completion))
        return completion

    def fake_run(self, attempt, *, total_deadline_unix, cancel_check, on_progress):
        self.assertEqual(total_deadline_unix, self.created + 7200)
        self.assertFalse(cancel_check())
        on_progress({'status': 'running', 'phase': 'background', 'owned_cleanup_verified': False})
        self.assertFalse(json.loads((self.attempt / 'job_state.json').read_text())['owned_cleanup_verified'])
        self.exports(attempt, self.quality)
        (attempt / 'workbench_execution.json').write_text(json.dumps({
            'status': 'completed_execution_pending_mesh_inventory', 'owned_cleanup_verified': True}))
        return 0

    def run_fake(self, quality=None):
        self.quality = quality
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), \
             patch.object(preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}), \
             patch.object(mesh.run_mesh_job, 'run', side_effect=self.fake_run) as run, \
             patch('subprocess.Popen') as popen:
            code = mesh.run(self.attempt, stage_root=self.root)
            popen.assert_not_called()
        return code, json.loads((self.attempt / 'probe_result.json').read_text()), run

    def test_exact_new_request_and_thirteen_sources(self):
        self.assertEqual(len(mesh.validate_request(self.attempt, self.root)['sources']), 13)
        self.assertEqual(mesh.AUTHORIZATION_VALUES['maximum_additional_attempts'], 1)
        self.assertEqual(mesh.AUTHORIZATION_VALUES['original_full_mesh_attempts_used'], 3)

    def test_old_definition_request_cannot_consume_mesh_grant(self):
        self.request.update(operation='definition_probe', job_id='definition-probe-001')
        self.save_request()
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_request(self.attempt, self.root)

    def test_7200_7170_deadlines_are_exact_not_extendable(self):
        for key in ('deadline_unix', 'work_deadline_unix'):
            self.request[key] += 1
            self.save_request()
            with self.assertRaises(mesh.RuntimeGateError):
                mesh.validate_request(self.attempt, self.root)
            self.request[key] -= 1

    def test_strict_schema_extra_keys_and_boolean_deadlines_rejected(self):
        self.request['retry'] = False
        self.save_request()
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_request(self.attempt, self.root)
        del self.request['retry']
        self.request['created_unix'] = True
        self.save_request()
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_request(self.attempt, self.root)

    def test_request_authorization_must_be_new_grant(self):
        self.request['authorization'] = {'path': str(self.baseline), 'sha256': mesh.AUTHORIZATION_VALUES['baseline_authorization_sha256']}
        self.save_request()
        with self.assertRaisesRegex(mesh.RuntimeGateError, 'new mesh authorization'):
            mesh.validate_request(self.attempt, self.root)

    def test_authorization_type_and_policy_cannot_expand(self):
        for key, value in [('mesh_generation_allowed', 1), ('maximum_total_seconds', 7201), ('fluent_allowed', True)]:
            changed = dict(mesh.AUTHORIZATION_VALUES, **{key: value})
            self.auth_path.write_text(json.dumps(changed))
            with self.assertRaises(mesh.RuntimeGateError):
                mesh.validate_authorization(self.root)

    def test_sources_missing_duplicate_modified_rejected(self):
        rows = self.request['sources']
        for invalid in (rows[:-1], rows[:-1] + [rows[0]]):
            with self.assertRaises(mesh.RuntimeGateError):
                mesh.validate_sources(invalid, self.root)
        Path(rows[0]['path']).write_text('changed')
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_sources(rows, self.root)

    def test_launcher_checks_own_pid_create_time(self):
        self.assertEqual(mesh.validate_launcher(self.attempt, self.request, lambda: 5)['pid'], os.getpid())
        ledger = json.loads((self.attempt / 'launcher.json').read_text())
        ledger['creation_time'] += 1
        (self.attempt / 'launcher.json').write_text(json.dumps(ledger))
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_launcher(self.attempt, self.request, lambda: 5)

    def test_launcher_wait_is_bounded(self):
        (self.attempt / 'launcher.json').unlink()
        with self.assertRaisesRegex(mesh.RuntimeGateError, '20-second wait'):
            mesh.validate_launcher(self.attempt, self.request, lambda: 0)

    def test_cancel_schema_and_fixed_operation(self):
        self.assertFalse(mesh.check_cancel(self.attempt))
        path = self.attempt / 'cancel_request.json'
        path.write_text(json.dumps({'job_id': mesh.JOB_ID, 'operation': 'cancel_mesh_construction', 'created_unix': time.time()}))
        self.assertTrue(mesh.check_cancel(self.attempt))
        path.write_text(json.dumps({'job_id': mesh.JOB_ID, 'operation': 'cancel_definition_probe', 'created_unix': time.time()}))
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.check_cancel(self.attempt)

    def test_preflight_has_no_directory_or_process_side_effect(self):
        before = sorted(str(p.relative_to(self.root)) for p in self.root.rglob('*'))
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), \
             patch.object(preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}), \
             patch('subprocess.Popen') as popen:
            report = mesh.inspect(self.root)
            popen.assert_not_called()
        self.assertEqual(report['status'], 'passed', report)
        self.assertTrue(report['template_preflight']['rendered_in_memory'])
        self.assertEqual(before, sorted(str(p.relative_to(self.root)) for p in self.root.rglob('*')))
        self.assertFalse((self.root / mesh.MESH_ATTEMPT_RELATIVE).exists())

    def test_generated_scripts_have_only_fixed_two_domains_no_solver(self):
        result = mesh.audit_templates(self.root, self.frozen)
        self.assertEqual(result['domains'], ['background', 'arm'])
        text = (self.root / 'mesh/meshing_stage15.py.in').read_text()
        self.assertEqual(text.count('mesh.GenerateMesh()'), 1)
        self.assertNotIn('size.GrowthRate =', text)
        self.assertIn('required_set(mesh, "GrowthRate", C["growth_rate"])', text)

    def test_template_solver_import_rejected(self):
        path = self.root / 'mesh/meshing_stage15.py.in'
        path.write_text(path.read_text() + '\nimport subprocess\n')
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.audit_templates(self.root, self.frozen)

    def test_two_true_native_export_formats_and_provenance(self):
        completion = self.exports()
        paths = mesh.validate_exports(self.root / mesh.MESH_ATTEMPT_RELATIVE, completion)
        self.assertEqual([p.name for p in paths], ['background.msh', 'arm.msh.h5'])

    def test_second_system_spaced_name_selects_existing_hyphen_directory(self):
        completion = self.exports()
        completion['systems'][1]['system_name'] = 'FFF 1'
        paths = mesh.validate_exports(self.root / mesh.MESH_ATTEMPT_RELATIVE, completion)
        self.assertEqual([p.name for p in paths], ['background.msh', 'arm.msh.h5'])

    def test_second_system_existing_literal_spaced_directory_is_supported(self):
        completion = self.exports()
        attempt = self.root / mesh.MESH_ATTEMPT_RELATIVE
        old = attempt / 'stage15_two_domains_files/dp0/FFF-1'
        renamed = old.with_name('FFF 1')
        old.rename(renamed)
        row = completion['systems'][1]
        row['system_name'] = 'FFF 1'
        row['mesh_export']['source_path'] = str(renamed / 'MECH/FFF-1.msh.h5')
        self.assertEqual(len(mesh.validate_exports(attempt, completion)), 2)

    def test_spaced_and_hyphen_directory_ambiguity_rejected(self):
        completion = self.exports()
        attempt = self.root / mesh.MESH_ATTEMPT_RELATIVE
        (attempt / 'stage15_two_domains_files/dp0/FFF 1/MECH').mkdir(parents=True)
        completion['systems'][1]['system_name'] = 'FFF 1'
        with self.assertRaisesRegex(mesh.RuntimeGateError, 'ambiguous per-system'):
            mesh.validate_exports(attempt, completion)

    def test_missing_ambiguous_empty_or_wrong_source_export_rejected(self):
        completion = self.exports()
        attempt = self.root / mesh.MESH_ATTEMPT_RELATIVE
        source = Path(completion['systems'][0]['mesh_export']['source_path'])
        other = source.with_name('other.msh')
        other.write_bytes(b'ambiguous')
        with self.assertRaisesRegex(mesh.RuntimeGateError, 'ambiguous'):
            mesh.validate_exports(attempt, completion)
        other.unlink()
        source.write_bytes(b'')
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_exports(attempt, completion)
        source.unlink()
        with self.assertRaises((mesh.RuntimeGateError, FileNotFoundError)):
            mesh.validate_exports(attempt, completion)

    def test_unsafe_system_name_or_same_export_rejected(self):
        completion = self.exports()
        attempt = self.root / mesh.MESH_ATTEMPT_RELATIVE
        completion['systems'][1]['system_name'] = '../FFF'
        with self.assertRaisesRegex(mesh.RuntimeGateError, 'unsafe'):
            mesh.validate_exports(attempt, completion)
        completion['systems'][1]['system_name'] = 'FFF'
        completion['systems'][1]['mesh_export'] = completion['systems'][0]['mesh_export']
        with self.assertRaises(mesh.RuntimeGateError):
            mesh.validate_exports(attempt, completion)

    def test_exported_but_missing_quality_is_not_physical_acceptance(self):
        code, report, run = self.run_fake()
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'completed')
        self.assertTrue(report['mesh_generated'])
        self.assertTrue(report['mesh_exports_written'])
        self.assertFalse(report['mesh_quality_verified'])
        self.assertEqual(report['mesh_inventory_status'], 'failed')
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertTrue(report['inputs_reverified_after_cleanup'])
        self.assertTrue(report['sources_reverified_after_cleanup'])
        self.assertFalse(report['solver_launched'])
        self.assertEqual(report['flow_iterations'], 0)
        manifest = json.loads((self.root / mesh.MESH_ATTEMPT_RELATIVE / 'mesh_manifest.json').read_text())
        self.assertFalse(manifest['valid'])
        self.assertEqual(run.call_count, 1)

    def test_native_quality_at_threshold_can_verify_but_motion_stays_false(self):
        code, report, _ = self.run_fake(.01)
        self.assertEqual(code, 0)
        self.assertTrue(report['mesh_quality_verified'])
        self.assertFalse(report['mesh_motion_verified'])
        self.assertEqual(report['static_overset'], 'not_run')

    def test_low_native_quality_not_verified(self):
        _, report, _ = self.run_fake(.00999)
        self.assertFalse(report['mesh_quality_verified'])
        self.assertEqual(report['mesh_inventory_status'], 'failed')

    def test_changed_source_after_cleanup_cannot_report_completed(self):
        self.quality = .02
        def changed(attempt, **kwargs):
            result = self.fake_run(attempt, **kwargs)
            Path(self.request['sources'][0]['path']).write_text('mutated after cleanup')
            return result
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), \
             patch.object(preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}), \
             patch.object(mesh.run_mesh_job, 'run', side_effect=changed):
            self.assertEqual(mesh.run(self.attempt, stage_root=self.root), 2)
        report = json.loads((self.attempt / 'probe_result.json').read_text())
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['sources_reverified_after_cleanup'])
        self.assertIn('source_reverification_error', report)

    def test_unknown_cleanup_preserves_true_exports_without_claiming_completion(self):
        self.quality = .02
        def failed_cleanup(attempt, **kwargs):
            self.fake_run(attempt, **kwargs)
            (attempt / 'workbench_execution.json').write_text(json.dumps({
                'status': 'cleanup_unverified', 'owned_cleanup_verified': False}))
            return 2
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), \
             patch.object(preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}), \
             patch.object(mesh.run_mesh_job, 'run', side_effect=failed_cleanup):
            self.assertEqual(mesh.run(self.attempt, stage_root=self.root), 2)
        report = json.loads((self.attempt / 'probe_result.json').read_text())
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertFalse(report['owned_cleanup_verified'])
        self.assertTrue(report['mesh_exports_written'])
        self.assertFalse(report['mesh_quality_verified'])

    def worker_prelaunch_failure(self, failure, *, stdout_marker=False):
        def rejected(attempt, **kwargs):
            if stdout_marker:
                (attempt / 'workbench.stdout.log').write_bytes(b'')
            raise failure
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), \
             patch.object(preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}), \
             patch.object(mesh.run_mesh_job, 'run', side_effect=rejected):
            self.assertEqual(mesh.run(self.attempt, stage_root=self.root), 2)
        return json.loads((self.attempt / 'probe_result.json').read_text())

    def test_first_native_cancel_before_launch_is_clean_without_execution_receipt(self):
        report = self.worker_prelaunch_failure(probe.ProbeCancelled('cancelled before launch'))
        self.assertEqual(report['status'], 'cancelled')
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertFalse(report['launch_attempted'])
        self.assertIn('elapsed_seconds', report)
        self.assertGreaterEqual(report['elapsed_seconds'], 0)

    def test_shared_lock_collision_does_not_touch_other_owner_and_is_not_unknown_cleanup(self):
        lock = self.root / '.ansys-task.lock'
        lock.write_text('{"token":"other-task"}')
        before = lock.read_bytes()
        report = self.worker_prelaunch_failure(mesh.RuntimeGateError('other task lock exists'))
        self.assertEqual(report['status'], 'failed')
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertFalse(report['launch_attempted'])
        self.assertEqual(lock.read_bytes(), before)

    def test_stdout_launch_marker_without_execution_receipt_is_unknown_cleanup(self):
        report = self.worker_prelaunch_failure(mesh.RuntimeGateError('launch receipt unavailable'), stdout_marker=True)
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertFalse(report['owned_cleanup_verified'])
        self.assertTrue(report['launch_attempted'])

    def test_claim_never_reruns_or_overwrites_first_result(self):
        self.run_fake()
        before = (self.attempt / 'probe_result.json').read_bytes()
        with patch.object(mesh.run_mesh_job, 'run') as run:
            self.assertEqual(mesh.run(self.attempt, stage_root=self.root), 2)
            run.assert_not_called()
        self.assertEqual(before, (self.attempt / 'probe_result.json').read_bytes())

    def test_cancel_before_generation_finishes_without_retry(self):
        (self.attempt / 'cancel_request.json').write_text(json.dumps({
            'job_id': mesh.JOB_ID, 'operation': 'cancel_mesh_construction', 'created_unix': time.time()}))
        with patch.object(probe, 'validate_frozen', return_value=self.frozen), patch.object(mesh.run_mesh_job, 'run') as run:
            self.assertEqual(mesh.run(self.attempt, stage_root=self.root), 2)
            run.assert_not_called()
        report = json.loads((self.attempt / 'probe_result.json').read_text())
        self.assertEqual(report['status'], 'cancelled')
        self.assertFalse((self.root / mesh.MESH_ATTEMPT_RELATIVE).exists())

    def test_wb_export_helper_rejects_missing_multiple_empty_and_existing_target(self):
        source = (STAGE / 'mesh/wb_stage15.wbjn.in').read_text()
        nodes = [n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name in ('native_text', 'system_directories', 'mech_directory', 'record_export_debug', 'export_mesh')]
        attempt = self.root / mesh.MESH_ATTEMPT_RELATIVE
        attempt.mkdir(parents=True)
        mech = attempt / 'stage15_two_domains_files/dp0/FFF/MECH'
        mech.mkdir(parents=True)
        def copy(a, b, overwrite):
            self.assertFalse(overwrite)
            with open(b, 'xb') as stream:
                stream.write(Path(a).read_bytes())
        ns = {'os': os, 're': mesh.re, 'C': {'project': str(attempt / 'stage15_two_domains.wbpj')},
              'R': {}, 'json': json, 'Encoding': SimpleNamespace(UTF8='utf-8'),
              'File': SimpleNamespace(Exists=os.path.exists, Copy=copy, GetAttributes=lambda p: 0,
                    WriteAllText=lambda path, data, encoding: Path(path).write_text(data, encoding=encoding)),
              'Directory': SimpleNamespace(GetFiles=lambda p: [str(x) for x in Path(p).iterdir() if x.is_file()])}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), 'offline export helpers', 'exec'), ns)
        with self.assertRaisesRegex(Exception, 'exactly one'):
            ns['export_mesh'](str(mech), 'background', set())
        a, b = mech / 'FFF.msh', mech / 'other.msh'
        a.write_bytes(b'')
        with self.assertRaisesRegex(Exception, 'empty'):
            ns['export_mesh'](str(mech), 'background', set())
        a.write_bytes(b'mesh')
        b.write_bytes(b'mesh2')
        with self.assertRaisesRegex(Exception, 'exactly one'):
            ns['export_mesh'](str(mech), 'background', set())
        b.unlink()
        result = ns['export_mesh'](str(mech), 'background', set())
        self.assertEqual(result['size'], 4)
        with self.assertRaisesRegex(Exception, 'overwrite'):
            ns['export_mesh'](str(mech), 'background', set())
        with self.assertRaisesRegex(Exception, 'unsafe'):
            ns['mech_directory'](ns['C']['project'], '../FFF')
        # The second system's public name may contain a space, while the saved
        # directory uses a hyphen. Never invent a directory if neither exists.
        with self.assertRaisesRegex(Exception, 'no existing'):
            ns['mech_directory'](ns['C']['project'], 'FFF 1')
        second = mech.parent.parent / 'FFF-1/MECH'
        second.mkdir(parents=True)
        (second / 'editor.mshdb').write_bytes(b'not a CFD mesh')
        self.assertEqual(ns['mech_directory'](ns['C']['project'], 'FFF 1'), str(second))
        ns['record_export_debug'](ns['C']['project'], 'FFF 1', 'missing_export')
        debug = json.loads((attempt / 'export_debug.json').read_text())
        self.assertEqual(debug[0]['directories'][1]['file_names'], ['editor.mshdb'])
        self.assertEqual(debug[0]['directories'][0]['file_names'], [])
        self.assertIn('FFF-1', debug[0]['dp0_names'])
        with self.assertRaisesRegex(Exception, 'exactly one'):
            ns['export_mesh'](str(second), 'arm', set())
        (mech.parent.parent / 'FFF 1/MECH').mkdir(parents=True)
        with self.assertRaisesRegex(Exception, 'ambiguous'):
            ns['mech_directory'](ns['C']['project'], 'FFF 1')


if __name__ == '__main__':
    unittest.main()

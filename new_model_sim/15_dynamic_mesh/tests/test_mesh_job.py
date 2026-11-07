import json
import sys
import tempfile
import unittest
import subprocess
import time
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import run_mesh_job as job
import runtime
import definition_probe as probe
from preflight import sha256


class MeshJobPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_test_mesh_job_", dir=str(STAGE / "runs"))
        self.root = Path(self.temp.name)
        self.background = self.root / "background.scdoc"
        self.arm = self.root / "arm.scdoc"
        self.journal = self.root / "mesh_stage15.wbjn"
        self.source = self.root / "source.py"
        for path, data in ((self.background, b"mock-bg"), (self.arm, b"mock-arm"),
                           (self.journal, b"offline fixture, never execute"), (self.source, b"pass")):
            path.write_bytes(data)
        self.verification = self.root / "verify.json"
        self.cad_data = {"mode": "verify", "state": "passed", "source_unchanged": True,
                         "cad_domains": {"state": "passed", "independent_reopen_completed": True,
                                         "physical_face_ownership_checked": True},
                         "artifacts": {"background": self.record(self.background), "arm": self.record(self.arm)}}
        self.save_verification()

    def tearDown(self):
        self.temp.cleanup()

    def record(self, path):
        return {"path": str(path), "sha256": sha256(path)}

    def save_verification(self):
        self.verification.write_text(json.dumps(self.cad_data), encoding="utf-8")
        self.plan = {"journal": self.record(self.journal), "cad_verification": self.record(self.verification),
                     "cad": [self.record(self.background), self.record(self.arm)],
                     "domains": [{"domain": "background"}, {"domain": "arm"}]}
        (self.root / "mesh_plan.json").write_text(json.dumps(self.plan), encoding="utf-8")
        self.inputs = {"inputs": [self.record(self.background), self.record(self.arm), self.record(self.verification)],
                       "sources": [self.record(self.source)]}
        (self.root / "inputs.json").write_text(json.dumps(self.inputs), encoding="utf-8")

    def test_bound_real_shape_of_acceptance_receipt_passes_offline(self):
        attempt, plan, journal = job.validate_plan(self.root, {})
        self.assertEqual(attempt, self.root)
        self.assertEqual(journal, self.journal)
        self.assertEqual(len(plan["cad"]), 2)

    def test_generated_only_or_blocked_cad_not_accepted(self):
        for state in ("not_run", "blocked", "failed"):
            self.cad_data["state"] = state
            self.save_verification()
            with self.assertRaises(job.RuntimeGateError):
                job.validate_plan(self.root, {})

    def test_missing_independent_reopen_or_ownership_rejected(self):
        for key in ("independent_reopen_completed", "physical_face_ownership_checked"):
            self.cad_data["cad_domains"][key] = False
            self.save_verification()
            with self.assertRaises(job.RuntimeGateError):
                job.validate_plan(self.root, {})
            self.cad_data["cad_domains"][key] = True

    def test_modified_journal_is_rejected(self):
        self.journal.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_changed_cad_is_rejected(self):
        self.arm.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_changed_frozen_source_is_rejected(self):
        self.source.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_missing_source_manifest_cannot_be_treated_as_zero(self):
        self.inputs["sources"] = []
        (self.root / "inputs.json").write_text(json.dumps(self.inputs), encoding="utf-8")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_same_cad_twice_is_rejected(self):
        self.plan["cad"][1] = self.record(self.background)
        (self.root / "mesh_plan.json").write_text(json.dumps(self.plan), encoding="utf-8")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_meshing_phase_is_cleared_only_by_export_completion(self):
        self.assertIsNone(job.trace_phase("geometry background\n"))
        self.assertEqual(job.trace_phase("generate background\n"), "background")
        self.assertIsNone(job.trace_phase("generate background\nexport completed background\n"))
        self.assertEqual(job.trace_phase("generate background\nexport completed background\ngeometry arm\ngenerate arm\n"), "arm")


class MeshJobExecutionTests(MeshJobPreflightTests):
    def execute_fake(self, *, failure=False, cleanup=True, uncertain=False, ownership_error=False,
                     cancel_after_launch=False, domain_timeout=False, progress_failure=False,
                     owned_uncertain=False):
        contract = {'software': {'workbench': 'FAKE_WORKBENCH_NEVER_EXECUTED'},
                    'budget': {'mesh_system_timeout_s': 3600, 'minimum_free_disk_gib': 1}}
        clock = [0.0]
        process = SimpleNamespace(pid=987654, returncode=0)
        def wait(timeout):
            if owned_uncertain:
                raise probe.UncertainCall('synthetic wait timeout after owned tree capture')
            if domain_timeout:
                clock[0] += 3601
                raise subprocess.TimeoutExpired('fake', timeout)
            return 0
        process.wait = wait
        tree = SimpleNamespace(members={987654: 123.0}, errors=['synthetic initial ownership error'] if ownership_error else [],
                               capture=lambda: None, close=Mock(return_value=cleanup))
        launched = [False]
        def spawn(*args, **kwargs):
            self.assertFalse(kwargs['shell'])
            launched[0] = True
            if uncertain:
                raise probe.UncertainCall('synthetic unreturned Popen')
            receipt = ({'status': 'failed', 'exception': 'synthetic backend failure'} if failure else
                {'status': 'completed_pending_mesh_inventory', 'mesh_exports_written': True,
                 'systems': [{'domain': 'background'}, {'domain': 'arm'}]})
            (self.root / 'workbench_completion.json').write_text(json.dumps(receipt))
            if domain_timeout:
                (self.root / 'workbench.trace.txt').write_text('generate background\n')
            return process
        observed = []
        def progress(value):
            observed.append(value)
            self.assertFalse(value['owned_cleanup_verified'])
            if progress_failure:
                raise OSError('synthetic outer state failure')
        with ExitStack() as stack:
            stack.enter_context(patch.object(job.preflight, 'load_contract', return_value=contract))
            stack.enter_context(patch.object(job.preflight, 'verify_inputs', return_value={'status': 'passed'}))
            stack.enter_context(patch.object(job.preflight, 'inspect_environment', return_value={'status': 'passed', 'blockers': []}))
            stack.enter_context(patch.object(job, 'child_environment', return_value={}))
            stack.enter_context(patch.object(job, 'AnsysTaskLock', side_effect=lambda attempt: runtime.AnsysTaskLock(attempt, stage_root=self.root)))
            stack.enter_context(patch.object(job, 'OwnedProcessTree', return_value=tree))
            stack.enter_context(patch.object(job.shutil, 'disk_usage', return_value=SimpleNamespace(free=1024 ** 4)))
            popen = stack.enter_context(patch.object(job.subprocess, 'Popen', side_effect=spawn))
            if domain_timeout:
                stack.enter_context(patch.object(job.time, 'monotonic', side_effect=lambda: clock[0]))
            code = job.run(self.root, total_deadline_unix=time.time() + 7200,
                           cancel_check=lambda: launched[0] and cancel_after_launch, on_progress=progress)
            self.assertEqual(popen.call_count, 1)
            if owned_uncertain:
                tree.close.assert_called_once_with()
            if uncertain:
                tree.close.assert_not_called()
        return code, json.loads((self.root / 'workbench_execution.json').read_text()), observed

    def test_fake_execution_keeps_legacy_integer_and_new_deadline_fields(self):
        code, report, observed = self.execute_fake()
        self.assertEqual(code, 0)
        self.assertEqual(report['total_timeout_seconds'], 7200)
        self.assertEqual(report['total_deadline_unix'] - report['work_deadline_unix'], 30)
        self.assertEqual(report['system_timeout_seconds'], 3600)
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertTrue(report['launch_attempted'])
        self.assertFalse((self.root / '.ansys-task.lock').exists())
        self.assertTrue(observed)

    def test_real_lock_collision_path_never_enters_launch_region_or_deletes_other_lock(self):
        lock = self.root / '.ansys-task.lock'
        lock.write_text('{"token":"other-task"}')
        before = lock.read_bytes()
        with self.assertRaisesRegex(job.RuntimeGateError, 'another ANSYS task'):
            self.execute_fake()
        self.assertEqual(lock.read_bytes(), before)
        self.assertFalse((self.root / 'workbench.stdout.log').exists())
        self.assertFalse((self.root / 'workbench_execution.json').exists())

    def test_initial_cancel_has_no_launch_marker_or_process(self):
        with patch.object(job.subprocess, 'Popen') as spawn:
            with self.assertRaises(probe.ProbeCancelled):
                job.run(self.root, cancel_check=lambda: True)
            spawn.assert_not_called()
        self.assertFalse((self.root / 'workbench.stdout.log').exists())
        self.assertFalse((self.root / '.ansys-task.lock').exists())

    def test_backend_failure_receipt_cleans_without_waiting_for_launcher_exit(self):
        code, report, _ = self.execute_fake(failure=True)
        self.assertEqual(code, 2)
        self.assertIn('synthetic backend failure', report['error'])
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertFalse((self.root / '.ansys-task.lock').exists())

    def test_unreturned_popen_retains_shared_lock_and_never_retries(self):
        code, report, _ = self.execute_fake(uncertain=True)
        self.assertEqual(code, 2)
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertFalse(report['owned_cleanup_verified'])
        self.assertTrue((self.root / '.ansys-task.lock').exists())

    def test_uncertain_call_after_tree_capture_still_closes_known_owned_members_and_retains_lock(self):
        code, report, _ = self.execute_fake(owned_uncertain=True)
        self.assertEqual(code, 2)
        self.assertIn('wait timeout after owned tree capture', report['error'])
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertFalse(report['owned_cleanup_verified'])
        self.assertTrue(report['task_lock_retained'])
        self.assertTrue((self.root / '.ansys-task.lock').exists())
        self.assertEqual(report['owned_processes'], [{'pid': 987654, 'creation_time': 123.0}])

    def test_unknown_ownership_never_releases_shared_lock(self):
        _, report, _ = self.execute_fake(ownership_error=True)
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertTrue(report['task_lock_retained'])
        self.assertTrue((self.root / '.ansys-task.lock').exists())

    def test_failed_cleanup_keeps_lock_instead_of_merely_writing_failed(self):
        _, report, _ = self.execute_fake(cleanup=False)
        self.assertEqual(report['status'], 'cleanup_unverified')
        self.assertTrue((self.root / '.ansys-task.lock').exists())

    def test_cancel_after_launch_cleans_owned_processes(self):
        _, report, _ = self.execute_fake(cancel_after_launch=True)
        self.assertEqual(report['status'], 'cancelled')
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertFalse((self.root / '.ansys-task.lock').exists())

    def test_per_domain_3600_limit_is_independent_of_total_7200(self):
        _, report, _ = self.execute_fake(domain_timeout=True)
        self.assertEqual(report['status'], 'failed')
        self.assertIn('per-domain 3600', report['error'])
        self.assertTrue(report['owned_cleanup_verified'])

    def test_progress_sink_failure_cannot_bypass_cleanup(self):
        _, report, _ = self.execute_fake(progress_failure=True)
        self.assertTrue(report['owned_cleanup_verified'])
        self.assertIn('progress_error', report)
        self.assertFalse((self.root / '.ansys-task.lock').exists())

    def test_reserve_deadline_rejects_launch_before_popen(self):
        with patch.object(job.subprocess, 'Popen') as spawn:
            with self.assertRaises(TimeoutError):
                job.run(self.root, total_deadline_unix=time.time() + 29)
            spawn.assert_not_called()


if __name__ == "__main__":
    unittest.main()

"""Offline fake-process coverage. No ANSYS/MCP service is started by these tests."""
import concurrent.futures
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import mcp_bridge as b


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "new_model_sim" / "15_dynamic_mesh"
        self.root.mkdir(parents=True)
        (self.root / "runs").mkdir()
        (self.root / "frozen").mkdir()
        (self.root / "reports").mkdir()
        self.put("contract.json", {"software": {"python": str(b.ANSYS_PYTHON)}})
        self.put("frozen/input_manifest.json", {"files": []})
        self.put("reports/stage15_status.json", {"overall_status": "blocked_at_mesh_preparation",
                 "gates": {"mesh_quality": "not_run"}, "mesh_attempts": [{}, {}, {}]})
        for name in b.SOURCES:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("# offline fixture\n", encoding="utf-8")
        helper = self.root.parent / "04_workbench/run_wb_fullenv.py"
        helper.parent.mkdir()
        helper.write_text("# offline environment builder", encoding="utf-8")
        grant = dict(b.FIXED_AUTH)
        files = {"background_sha256": grant["background_cad_relative"],
                 "arm_sha256": grant["arm_cad_relative"],
                 "cad_verification_sha256": grant["cad_verification_relative"]}
        for key, relative in files.items():
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("offline fixture " + key, encoding="utf-8")
            grant[key] = b.sha256(path)
        grant["input_manifest_sha256"] = b.sha256(self.root / "frozen/input_manifest.json")
        grant["contract_sha256"] = b.sha256(self.root / "contract.json")
        self.put(b.AUTHORIZATION, grant)
        self.bridge = b.Stage15Bridge(self.root)
        self.python = mock.patch.object(self.bridge, "_python", return_value="offline-python")
        self.python.start()
        self.addCleanup(self.python.stop)
        self.identity = mock.patch.object(self.bridge, "_inspect_pid", return_value={
            "pid": 123, "exists": True, "creation_time": 12345.0})
        self.identity_mock = self.identity.start()
        self.addCleanup(self.identity.stop)
        self.popen = mock.patch.object(b.subprocess, "Popen", return_value=mock.Mock(pid=123))
        self.popen_mock = self.popen.start()
        self.addCleanup(self.popen.stop)

    def put(self, relative, data):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding="utf-8")

    def load(self, relative):
        return json.loads((self.root / relative).read_text(encoding="utf-8"))

    def start(self):
        return self.bridge.start(b.START_CONFIRMATION)

    def test_wrong_confirmation_never_reserves(self):
        with self.assertRaises(b.BridgeError):
            self.bridge.start("NO")
        self.assertFalse((self.root / b.ATTEMPT).exists())
        self.popen_mock.assert_not_called()

    def test_status_is_read_only_and_not_validation(self):
        result = self.bridge.status()
        self.assertEqual(result["remaining_definition_probes"], 1)
        self.assertEqual(result["mesh_attempts_used"], 3)
        self.assertFalse(result["solver_authorized"])
        self.assertFalse(result["full_mesh_authorized"])
        self.popen_mock.assert_not_called()

    def test_authorization_bounds_strict_including_boolean_types(self):
        for name, value in (("maximum_attempts", True), ("maximum_total_seconds", 301),
                            ("full_mesh_allowed", True), ("fluent_allowed", True),
                            ("automatic_retry_allowed", True), ("attempt_relative", "../bad")):
            with self.subTest(name=name):
                grant = self.load(b.AUTHORIZATION)
                original = grant[name]
                grant[name] = value
                self.put(b.AUTHORIZATION, grant)
                with self.assertRaises(b.BridgeError):
                    self.start()
                grant[name] = original
                self.put(b.AUTHORIZATION, grant)
        self.popen_mock.assert_not_called()

    def test_input_change_blocks_before_reservation(self):
        (self.root / b.FIXED_AUTH["background_cad_relative"]).write_text("changed")
        with self.assertRaises(b.BridgeError):
            self.start()
        self.assertFalse((self.root / b.ATTEMPT).exists())

    def test_missing_authorization_does_not_start(self):
        (self.root / b.AUTHORIZATION).unlink()
        with self.assertRaises(b.BridgeError):
            self.start()
        self.popen_mock.assert_not_called()

    def test_existing_lock_is_preserved(self):
        self.put(".ansys-task.lock", {"token": "someone-else"})
        with self.assertRaises(b.BridgeError):
            self.start()
        self.assertEqual(self.load(".ansys-task.lock")["token"], "someone-else")
        self.popen_mock.assert_not_called()

    def test_start_fixed_worker_request_and_budget(self):
        result = self.start()
        self.assertEqual(result["status"], "accepted")
        request = self.load(b.ATTEMPT + "/request.json")
        self.assertEqual(request["deadline_unix"] - request["created_unix"], 300)
        self.assertEqual(request["work_deadline_unix"] - request["created_unix"], 270)
        self.assertEqual(len(request["sources"]), 10)
        self.assertTrue(all(Path(row["path"]).is_absolute() for row in request["sources"]))
        command = self.popen_mock.call_args.args[0]
        self.assertEqual(command[0], "offline-python")
        self.assertIn("--run", command)
        self.assertIn(str(self.root / "definition_probe.py"), command)
        self.assertNotIn("fluent", " ".join(command).lower())
        self.assertFalse(self.popen_mock.call_args.kwargs["shell"])
        self.assertEqual(self.load(b.ATTEMPT + "/launcher.json")["creation_time"], 12345.0)

    def test_duplicate_start_does_not_consume_another_job(self):
        self.start()
        result = self.start()
        self.assertEqual(result["status"], "already_reserved")
        self.assertEqual(self.popen_mock.call_count, 1)

    def test_new_bridge_after_restart_does_not_relaunch(self):
        self.start()
        restarted = b.Stage15Bridge(self.root)
        with mock.patch.object(restarted, "_inspect_pid", return_value={"pid": 123, "exists": False}):
            result = restarted.start(b.START_CONFIRMATION)
        self.assertEqual(result["status"], "already_reserved")
        self.assertEqual(result["job"]["status"], "cleanup_unverified")
        self.assertEqual(self.popen_mock.call_count, 1)

    def test_concurrent_calls_reserve_only_once(self):
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            results = list(executor.map(lambda _: self.start(), range(4)))
        self.assertEqual(sum(row.get("new_worker_started", False) for row in results), 1)
        self.assertEqual(self.popen_mock.call_count, 1)

    def test_popen_failure_keeps_slot_and_records_failure(self):
        self.popen_mock.side_effect = OSError("offline launch failure")
        result = self.start()
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["owned_cleanup_verified"])
        self.assertTrue((self.root / b.ATTEMPT / "launch_failure.json").is_file())
        self.start()
        self.assertEqual(self.popen_mock.call_count, 1)

    def test_identity_failure_never_creates_false_ledger(self):
        self.identity_mock.side_effect = b.BridgeError("identity unavailable")
        result = self.start()
        self.assertEqual(result["status"], "cleanup_unverified")
        self.assertFalse((self.root / b.ATTEMPT / "launcher.json").exists())
        self.assertTrue((self.root / b.ATTEMPT / "cancel_request.json").exists())

    def test_empty_reserved_directory_is_not_reused(self):
        (self.root / b.ATTEMPT).mkdir(parents=True)
        self.assertEqual(self.start()["status"], "already_reserved")
        self.popen_mock.assert_not_called()

    def test_unknown_job_and_traversal_rejected(self):
        for value in ("../contract.json", "other", str(self.root), "definition-probe-002"):
            with self.subTest(value=value), self.assertRaises(b.BridgeError):
                self.bridge.job_status(value)
        for value in ("../contract.json", "D:/work4/cfd/case", "file:stream"):
            with self.subTest(value=value), self.assertRaises(b.BridgeError):
                self.bridge.safe(value)

    def test_windows_reparse_point_rejected_without_privileged_symlink(self):
        actual = Path.lstat
        target = self.root / "runs"
        def inspected(path):
            if path == target:
                return mock.Mock(st_mode=stat_directory(), st_file_attributes=0x400)
            return actual(path)
        with mock.patch.object(Path, "lstat", inspected):
            with self.assertRaises(b.BridgeError):
                self.bridge.safe(b.ATTEMPT)

    def test_lost_worker_is_not_success_and_lock_is_retained(self):
        self.start()
        self.put(".ansys-task.lock", {"token": "owned-worker"})
        self.identity_mock.return_value = {"pid": 123, "exists": False}
        result = self.bridge.job_status(b.JOB_ID)
        self.assertEqual(result["status"], "cleanup_unverified")
        self.assertTrue((self.root / ".ansys-task.lock").exists())

    def test_pid_reuse_refuses_cancel(self):
        self.start()
        self.identity_mock.return_value = {"pid": 123, "exists": True, "creation_time": 98765.0}
        status = self.bridge.job_status(b.JOB_ID)
        self.assertTrue(status["worker_pid_reused"])
        cancelled = self.bridge.cancel(b.JOB_ID, b.CANCEL_CONFIRMATION)
        self.assertEqual(cancelled["status"], "cleanup_unverified")
        self.assertTrue(cancelled["lock_must_be_preserved"])
        self.assertFalse(cancelled["retry_permitted"])
        self.assertFalse((self.root / b.ATTEMPT / "cancel_request.json").exists())

    def test_cancel_is_only_idempotent_request_not_cleanup(self):
        self.start()
        for _ in range(2):
            result = self.bridge.cancel(b.JOB_ID, b.CANCEL_CONFIRMATION)
            self.assertEqual(result["status"], "cancellation_requested")
            self.assertFalse(result["cleanup_verified"])
        self.assertEqual(self.load(b.ATTEMPT + "/cancel_request.json")["operation"], "cancel_definition_probe")
        self.assertEqual(self.popen_mock.call_count, 1)

    def test_wrong_cancel_confirmation_has_no_side_effect(self):
        self.start()
        with self.assertRaises(b.BridgeError):
            self.bridge.cancel(b.JOB_ID, "NO")
        self.assertFalse((self.root / b.ATTEMPT / "cancel_request.json").exists())

    def test_final_receipt_requires_cleanup(self):
        self.start()
        self.put(b.ATTEMPT + "/probe_result.json", {
            "job_id": b.JOB_ID, "status": "passed", "owned_cleanup_verified": False})
        result = self.bridge.job_status(b.JOB_ID)
        self.assertEqual(result["status"], "cleanup_unverified")
        self.assertFalse(result["mesh_generated"])

    def test_result_id_mismatch_fails_closed(self):
        self.start()
        self.put(b.ATTEMPT + "/probe_result.json", {"job_id": "other", "status": "passed"})
        with self.assertRaises(b.BridgeError):
            self.bridge.job_status(b.JOB_ID)

    def test_exit_without_final_receipt_never_counts_as_pass(self):
        self.start()
        self.identity_mock.return_value = {"pid": 123, "exists": False}
        self.assertEqual(self.bridge.job_status(b.JOB_ID)["status"], "cleanup_unverified")

    def test_logs_bounded_unicode_and_redacted(self):
        self.start()
        text = "\n".join("测试行%d %s" % (i, "中" * 200) for i in range(350))
        text += "\npassword=do-not-return-me\n后端连接失败"
        (self.root / b.ATTEMPT / "worker.log").write_text(text, encoding="utf-8")
        result = self.bridge.job_log(b.JOB_ID, "worker", 200)
        self.assertLessEqual(len(result["text"]), 20000)
        self.assertNotIn("do-not-return-me", result["text"])
        self.assertIn("后端连接失败", result["text"])

    def test_partial_long_credential_line_not_exposed(self):
        self.start()
        (self.root / b.ATTEMPT / "worker.log").write_text("password=" + "sensitive" * 10000 + "\nnormal line", encoding="utf-8")
        text = self.bridge.job_log(b.JOB_ID, "worker", 200)["text"]
        self.assertNotIn("sensitive", text)
        self.assertEqual(text, "normal line")

    def test_known_prelaunch_failure_remains_failed_not_lost(self):
        self.popen_mock.side_effect = OSError("offline launch failure")
        self.start()
        status = self.bridge.job_status(b.JOB_ID)
        self.assertEqual(status["status"], "failed")
        self.assertTrue(status["owned_cleanup_verified"])

    def test_successful_receipt_with_flow_is_rejected_not_hidden(self):
        self.start()
        self.put(b.ATTEMPT + "/probe_result.json", {
            "job_id": b.JOB_ID, "status": "passed", "owned_cleanup_verified": True,
            "mesh_generated": True, "flow_iterations": 1, "mesh_motion_verified": False})
        result = self.bridge.job_status(b.JOB_ID)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["mesh_generated"])
        self.assertEqual(result["flow_iterations"], 1)

    def test_complete_probe_receipt_keeps_downstream_unverified(self):
        self.start()
        self.put(b.ATTEMPT + "/probe_result.json", {
            "job_id": b.JOB_ID, "status": "passed", "owned_cleanup_verified": True,
            "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False,
            "definition_receipt": {"mesh_counts_before": {"nodes": 0, "elements": 0},
                                   "mesh_counts_after": {"nodes": 0, "elements": 0}}})
        result = self.bridge.job_status(b.JOB_ID)
        self.assertEqual(result["status"], "passed")
        self.assertFalse(result["mesh_motion_verified"])

    def test_native_zero_counts_are_required_not_self_reported_flag(self):
        self.start()
        for counts in ({}, {"nodes": False, "elements": 0}, {"nodes": 1, "elements": 0}):
            self.put(b.ATTEMPT + "/probe_result.json", {
                "job_id": b.JOB_ID, "status": "passed", "owned_cleanup_verified": True,
                "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False,
                "definition_receipt": {"mesh_counts_before": {"nodes": 0, "elements": 0},
                                       "mesh_counts_after": counts}})
            result = self.bridge.job_status(b.JOB_ID)
            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["phase"], "native_zero_mesh_counts_missing_or_nonzero")

    def test_identity_deadlines_leave_worker_handshake_margin(self):
        self.assertEqual(b.IDENTITY_TIMEOUT_SECONDS, 15)
        self.assertGreater(b.LAUNCHER_WAIT_SECONDS, b.IDENTITY_TIMEOUT_SECONDS)
        self.assertLess(b.LAUNCHER_WAIT_SECONDS, 270)

    def test_log_invalid_stream_or_limit(self):
        self.start()
        for stream, limit in (("../secret", 1), ("worker", 201), ("worker", 0), ("worker", True)):
            with self.subTest(stream=stream, limit=limit), self.assertRaises(b.BridgeError):
                self.bridge.job_log(b.JOB_ID, stream, limit)

    def test_readonly_preflight_does_not_reserve(self):
        response = subprocess.CompletedProcess([], 0, b'{"status":"passed","fluent_launched":false}', b"")
        with mock.patch.object(b.subprocess, "run", return_value=response) as run:
            result = self.bridge.preflight()
        self.assertEqual(result["status"], "passed")
        self.assertEqual(run.call_args.kwargs["timeout"], 45)
        self.assertIn("--preflight", run.call_args.args[0])
        self.assertFalse((self.root / b.ATTEMPT).exists())
        self.popen_mock.assert_not_called()

    def test_readonly_preflight_rejects_contradictory_exit(self):
        response = subprocess.CompletedProcess([], 2, b'{"status":"passed"}', b"")
        with mock.patch.object(b.subprocess, "run", return_value=response), self.assertRaises(b.BridgeError):
            self.bridge.preflight()

    def test_preflight_timeout_never_reserves(self):
        with mock.patch.object(b.subprocess, "run", side_effect=subprocess.TimeoutExpired("mock", 45)):
            with self.assertRaises(subprocess.TimeoutExpired):
                self.bridge.preflight()
        self.assertFalse((self.root / b.ATTEMPT).exists())

    def test_preflight_stderr_credentials_are_redacted(self):
        response = subprocess.CompletedProcess([], 2, b"not json", b"password=secret-value\nerror")
        with mock.patch.object(b.subprocess, "run", return_value=response):
            with self.assertRaises(b.BridgeError) as raised:
                self.bridge.preflight()
        self.assertNotIn("secret-value", str(raised.exception))

    def test_sanitized_child_python_environment(self):
        with mock.patch.dict(b.os.environ, {"PYTHONPATH": "untrusted", "PYTHONHOME": "untrusted"}):
            env = self.bridge._environment()
        self.assertNotIn("PYTHONPATH", env)
        self.assertNotIn("PYTHONHOME", env)
        self.assertEqual(env["PYTHONDONTWRITEBYTECODE"], "1")
        self.assertEqual(env["PYTHONIOENCODING"], "utf-8")


def stat_directory():
    import stat
    return stat.S_IFDIR | 0o755


if __name__ == "__main__":
    unittest.main()

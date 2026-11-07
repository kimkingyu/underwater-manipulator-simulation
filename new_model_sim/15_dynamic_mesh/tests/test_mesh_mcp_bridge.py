"""Offline-only construction bridge tests; no real ANSYS or license call."""
import copy
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mcp_bridge as legacy
import mesh_mcp_bridge as construction
import test_mcp_bridge as fixtures


class MeshConstructionBridgeTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.BridgeTests("test_status_is_read_only_and_not_validation")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        self.put = self.fixture.put
        self.load = self.fixture.load
        for relative in construction.SOURCE_RELATIVES:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                path.write_text("# offline construction fixture\n", encoding="utf-8")
        baseline_hash = legacy.sha256(self.root / legacy.AUTHORIZATION)
        self.grant_patch = mock.patch.dict(construction.AUTH_VALUES, baseline_authorization_sha256=baseline_hash)
        self.grant_patch.start()
        self.addCleanup(self.grant_patch.stop)
        self.put(construction.GRANT, copy.deepcopy(construction.AUTH_VALUES))
        self.put("runs/mcp_sizing_diagnostic/batch_001/attempt_003/probe_result.json", {
            "job_id": "sizing-b001-a003", "status": "completed", "full_background_definitions_verified": True,
            "owned_cleanup_verified": True, "inputs_reverified_after_cleanup": True,
            "sources_reverified_after_cleanup": True})
        self.slot = construction.MeshSlot(self.root)
        self.python = mock.patch.object(self.slot, "_python", return_value="offline-python")
        self.python.start()
        self.addCleanup(self.python.stop)
        self.identity = mock.patch.object(self.slot, "_inspect_pid", return_value={
            "pid": 123, "exists": True, "creation_time": 12345.0})
        self.identity_mock = self.identity.start()
        self.addCleanup(self.identity.stop)

    def start(self):
        return self.slot.start(construction.START_TOKEN)

    def result(self, *, quality=False, exported=True, cleanup=True, flow=0):
        self.start()
        value = {"job_id": construction.JOB_ID, "operation": "mesh_construction", "status": "completed",
                 "mesh_generated": True, "mesh_exports_written": exported,
                 "mesh_inventory_status": "mesh_exported_pending_fluent_static_gate" if quality else "failed",
                 "mesh_quality_verified": quality, "owned_cleanup_verified": cleanup,
                 "inputs_reverified_after_cleanup": True, "sources_reverified_after_cleanup": True,
                 "mesh_motion_verified": False, "flow_iterations": flow, "solver_launched": False,
                 "meshes": [{"domain": name, "nodes": 10, "elements": 5,
                             "quality": {"orthogonal_quality": {"minimum": 0.02}} if quality else {}}
                            for name in ("background", "arm")]}
        self.put(construction.JOB_ATTEMPT + "/probe_result.json", value)
        return value

    def test_invalid_confirmation_never_reserves(self):
        with self.assertRaises(legacy.BridgeError):
            self.slot.start("NO")
        self.assertFalse((self.root / construction.JOB_ATTEMPT).exists())
        self.fixture.popen_mock.assert_not_called()

    def test_status_never_starts_native_work(self):
        status = self.slot.status()
        self.assertEqual(status["remaining_full_mesh_constructions"], 1)
        self.assertEqual(status["total_seconds_including_cleanup"], 7200)
        self.assertFalse(status["solver_authorized"])
        self.fixture.popen_mock.assert_not_called()

    def test_request_uses_new_scope_and_two_hour_deadline(self):
        result = self.start()
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["total_budget_seconds"], 7200)
        request = self.load(construction.JOB_ATTEMPT + "/request.json")
        self.assertEqual(request["operation"], "mesh_construction")
        self.assertEqual(request["deadline_unix"] - request["created_unix"], 7200)
        self.assertEqual(request["work_deadline_unix"] - request["created_unix"], 7170)
        self.assertEqual(len(request["sources"]), 13)
        self.assertIn(str(self.root / "mesh_construction.py"), self.fixture.popen_mock.call_args.args[0])
        self.assertEqual(self.load(construction.JOB_ATTEMPT + "/job_state.json")["deadline_unix"], request["deadline_unix"])
        self.assertFalse((self.root / construction.MESH_ATTEMPT).exists())

    def test_create_only_source_archive_has_thirteen_entries(self):
        self.start()
        archive = self.load(construction.JOB_ATTEMPT + "/source_snapshot/archive_manifest.json")
        self.assertEqual(archive["job_id"], construction.JOB_ID)
        self.assertEqual(len(archive["files"]), 13)
        for row in archive["files"]:
            path = self.root / construction.JOB_ATTEMPT / "source_snapshot" / row["archive_relative"]
            self.assertEqual(legacy.sha256(path), row["sha256"])

    def test_duplicate_start_is_idempotent(self):
        self.start()
        self.assertEqual(self.start()["status"], "already_reserved")
        self.assertEqual(self.fixture.popen_mock.call_count, 1)

    def test_preexisting_actual_attempt_cannot_be_adopted(self):
        (self.root / construction.MESH_ATTEMPT).mkdir(parents=True)
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertFalse((self.root / construction.JOB_ATTEMPT).exists())
        self.fixture.popen_mock.assert_not_called()

    def test_other_job_lock_blocks_without_deleting_it(self):
        self.put(".ansys-task.lock", {"token": "other-session"})
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertEqual(self.load(".ansys-task.lock")["token"], "other-session")
        self.fixture.popen_mock.assert_not_called()

    def test_pending_sizing_dispatch_blocks(self):
        self.put(".sizing-dispatch.lock", {"token": "pending"})
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertTrue((self.root / ".sizing-dispatch.lock").exists())
        self.fixture.popen_mock.assert_not_called()

    def test_extra_retry_or_flow_permission_cannot_be_added(self):
        for key, value in (("maximum_additional_attempts", 2), ("maximum_total_seconds", 7201),
                           ("fluent_allowed", True), ("CFD_allowed", True), ("automatic_retry_allowed", True)):
            grant = copy.deepcopy(construction.AUTH_VALUES)
            grant[key] = value
            self.put(construction.GRANT, grant)
            with self.subTest(key=key), self.assertRaises(legacy.BridgeError):
                self.start()
        self.fixture.popen_mock.assert_not_called()

    def test_background_prerequisite_must_be_genuine_completed_gate(self):
        path = "runs/mcp_sizing_diagnostic/batch_001/attempt_003/probe_result.json"
        value = self.load(path)
        value["full_background_definitions_verified"] = False
        self.put(path, value)
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.fixture.popen_mock.assert_not_called()

    def test_mesh_generated_is_not_forced_false_by_diagnostic_base(self):
        self.result()
        status = self.slot.job_status(construction.JOB_ID)
        self.assertTrue(status["mesh_generated"])
        self.assertTrue(status["mesh_exports_written"])
        self.assertFalse(status["mesh_quality_verified"])
        self.assertFalse(status["mesh_motion_verified"])
        self.assertEqual(status["flow_iterations"], 0)

    def test_completed_needs_two_domains_and_cleanup(self):
        value = self.result()
        value["meshes"].pop()
        self.put(construction.JOB_ATTEMPT + "/probe_result.json", value)
        self.assertEqual(self.slot.job_status(construction.JOB_ID)["status"], "failed")

    def test_unknown_orthogonal_quality_is_not_passed(self):
        value = self.result(quality=True)
        value["meshes"][0]["quality"]["orthogonal_quality"] = {"status": "unavailable"}
        self.put(construction.JOB_ATTEMPT + "/probe_result.json", value)
        status = self.slot.job_status(construction.JOB_ID)
        self.assertEqual(status["status"], "failed")
        self.assertFalse(status["mesh_quality_verified"])

    def test_quality_below_frozen_threshold_is_not_passed(self):
        value = self.result(quality=True)
        value["meshes"][1]["quality"]["orthogonal_quality"]["minimum"] = 0.009
        self.put(construction.JOB_ATTEMPT + "/probe_result.json", value)
        self.assertFalse(self.slot.job_status(construction.JOB_ID)["mesh_quality_verified"])

    def test_native_quality_pass_does_not_authorize_flow(self):
        self.result(quality=True)
        status = self.slot.job_status(construction.JOB_ID)
        self.assertTrue(status["mesh_quality_verified"])
        self.assertFalse(status["CFD_authorized"])
        self.assertFalse(status["mesh_motion_verified"])

    def test_unapproved_flow_invalidates_result(self):
        self.result(flow=1)
        self.assertEqual(self.slot.job_status(construction.JOB_ID)["status"], "failed")

    def test_logs_use_only_fixed_real_attempt_paths(self):
        self.start()
        self.put(construction.MESH_ATTEMPT + "/background_meshing_stats.json", {"domain": "background"})
        self.assertEqual(self.slot.job_log(construction.JOB_ID, "mesh_background")["status"], "available")
        with self.assertRaises(legacy.BridgeError):
            self.slot.job_log(construction.JOB_ID, "../../contract.json")

    def test_cancellation_is_own_job_request_not_global_process_kill(self):
        self.start()
        result = self.slot.cancel(construction.JOB_ID, construction.CANCEL_TOKEN)
        self.assertEqual(result["status"], "cancellation_requested")
        self.assertEqual(self.load(construction.JOB_ATTEMPT + "/cancel_request.json")["operation"], "cancel_mesh_construction")
        self.assertFalse(result["cleanup_verified"])


if __name__ == "__main__":
    unittest.main()

"""Batch routing/ownership tests. Every worker launch and PID query is fake."""
import concurrent.futures
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mcp_bridge as legacy
import sizing_mcp_bridge as diagnostic
import test_mcp_bridge as fixtures


class SizingBridgeTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.BridgeTests("test_status_is_read_only_and_not_validation")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        self.put = self.fixture.put
        self.load = self.fixture.load
        for relative in diagnostic.SOURCE_RELATIVES:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                path.write_text("# offline fixture\n", encoding="utf-8")
        baseline_hash = legacy.sha256(self.root / legacy.AUTHORIZATION)
        self.grant_patch = mock.patch.dict(diagnostic.AUTH_VALUES, baseline_authorization_sha256=baseline_hash)
        self.grant_patch.start()
        self.addCleanup(self.grant_patch.stop)
        self.put(diagnostic.GRANT, copy.deepcopy(diagnostic.AUTH_VALUES))
        self.budget = diagnostic.SizingBudget(self.root)
        self.live = {}
        self.next_pid = 200
        def launch(*args, **kwargs):
            pid = self.next_pid
            self.next_pid += 1
            self.live[pid] = float(pid * 100)
            return mock.Mock(pid=pid)
        self.fixture.popen_mock.side_effect = launch
        self.python = mock.patch.object(diagnostic.SizingSlot, "_python", return_value="offline-python")
        self.python.start()
        self.addCleanup(self.python.stop)
        def identity(_, pid):
            return {"pid": pid, "exists": pid in self.live, "creation_time": self.live.get(pid)}
        self.identity = mock.patch.object(diagnostic.SizingSlot, "_inspect_pid", identity)
        self.identity.start()
        self.addCleanup(self.identity.stop)

    def start(self, index=1, profile="baseline"):
        return self.budget.start(index, profile, "bounded offline hypothesis", diagnostic.START_TOKEN)

    def finish(self, index=1, status="completed", cleanup=True, fully_defined=False):
        slot = diagnostic.SizingSlot(index, stage_root=self.root)
        receipts = {phase: {
            "status": "completed", "mesh_counts": {"nodes": 0, "elements": 0},
            "zero_mesh_verified": True, "control_fully_defined": fully_defined,
            "control": {"object_state": "FullyDefined" if fully_defined else "UnderDefined", "fields": {}},
            "snapshots": [], "messages": [], "message_capture_complete": True,
        } for phase in ("define", "refresh", "readback")}
        self.put(slot.ATTEMPT + "/probe_result.json", {
            "job_id": slot.JOB_ID, "status": status, "phase": "finished", "receipts": receipts,
            "diagnostic_completed": status == "completed", "zero_mesh_verified": True,
            "control_fully_defined": fully_defined, "owned_cleanup_verified": cleanup,
            "mesh_generated": False, "flow_iterations": 0, "mesh_motion_verified": False})
        launcher = self.load(slot.ATTEMPT + "/launcher.json")
        self.live.pop(launcher["pid"], None)
        return slot

    def test_wrong_confirmation_has_no_side_effect(self):
        with self.assertRaises(legacy.BridgeError):
            self.budget.start(1, "baseline", "reason", "NO")
        self.assertFalse((self.root / diagnostic.BATCH_ROOT).exists())
        self.assertFalse((self.root / ".sizing-dispatch.lock").exists())
        self.fixture.popen_mock.assert_not_called()

    def test_attempt_profile_and_hypothesis_are_bounded(self):
        for index in (0, 4, True, "1"):
            with self.subTest(index=index), self.assertRaises(legacy.BridgeError):
                diagnostic.SizingSlot(index, stage_root=self.root)
        for profile, hypothesis in (("arbitrary", "reason"), ("baseline", ""), ("baseline", "x" * 2001)):
            with self.subTest(profile=profile), self.assertRaises(legacy.BridgeError):
                diagnostic.SizingSlot(1, profile, hypothesis, self.root)

    def test_public_job_id_parser_rejects_paths_and_other_batches(self):
        for value in ("../request.json", "sizing-b002-a001", "sizing-b001-a004", "sizing-b001-a1"):
            with self.subTest(value=value), self.assertRaises(legacy.BridgeError):
                self.budget.job(value)

    def test_bad_grant_blocks_without_reserving(self):
        grant = self.load(diagnostic.GRANT)
        grant["maximum_attempts"] = 4
        self.put(diagnostic.GRANT, grant)
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertFalse((self.root / diagnostic.BATCH_ROOT).exists())

    def test_unknown_grant_field_rejected(self):
        grant = self.load(diagnostic.GRANT)
        grant["extra_script"] = "untrusted"
        self.put(diagnostic.GRANT, grant)
        with self.assertRaises(legacy.BridgeError):
            self.start()

    def test_exact_request_source_scope_and_worker(self):
        result = self.start()
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["job_id"], "sizing-b001-a001")
        self.assertEqual(result["diagnostic_budget"]["unreserved_slots"], 2)
        request = self.load(diagnostic.BATCH_ROOT + "/attempt_001/request.json")
        self.assertEqual(request["operation"], "sizing_diagnostic")
        self.assertEqual(request["profile"], "baseline")
        self.assertEqual(request["attempt_index"], 1)
        self.assertEqual(len(request["sources"]), 14)
        self.assertEqual(request["deadline_unix"] - request["created_unix"], 300)
        self.assertEqual(request["work_deadline_unix"] - request["created_unix"], 270)
        self.assertIn(str(self.root / "sizing_diagnostic.py"), self.fixture.popen_mock.call_args.args[0])
        self.assertFalse((self.root / ".sizing-dispatch.lock").exists())

    def test_sources_are_archived_before_launch_and_never_rewritten(self):
        self.start()
        slot = self.budget.job("sizing-b001-a001")
        manifest = self.load(slot.ATTEMPT + "/source_snapshot/archive_manifest.json")
        self.assertEqual(len(manifest["files"]), 14)
        for row in manifest["files"]:
            saved = self.root / slot.ATTEMPT / "source_snapshot" / row["archive_relative"]
            self.assertEqual(legacy.sha256(saved), row["sha256"])
        old = (self.root / slot.ATTEMPT / "source_snapshot/mcp_bridge.py").read_bytes()
        (self.root / "mcp_bridge.py").write_text("# subsequent source revision", encoding="utf-8")
        self.assertEqual(self.start()["status"], "already_reserved")
        self.assertEqual((self.root / slot.ATTEMPT / "source_snapshot/mcp_bridge.py").read_bytes(), old)

    def test_completed_needs_all_native_count_receipts(self):
        self.start()
        slot = self.finish()
        value = self.load(slot.ATTEMPT + "/probe_result.json")
        value["receipts"]["readback"]["mesh_counts"]["nodes"] = 1
        self.put(slot.ATTEMPT + "/probe_result.json", value)
        result = slot.job_status(slot.JOB_ID)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["phase"], "diagnostic_native_zero_counts_missing_or_nonzero")

    def test_same_slot_is_idempotent_and_different_profile_conflicts(self):
        self.start()
        request = (self.root / diagnostic.BATCH_ROOT / "attempt_001/request.json").read_bytes()
        result = self.start()
        self.assertEqual(result["status"], "already_reserved")
        self.assertEqual(self.fixture.popen_mock.call_count, 1)
        with self.assertRaises(legacy.BridgeError):
            self.start(profile="geometry_scope")
        self.assertEqual(request, (self.root / diagnostic.BATCH_ROOT / "attempt_001/request.json").read_bytes())

    def test_evidence_driven_profile_uses_next_slot_not_a_reset(self):
        self.start()
        first = self.finish()
        old_request = (self.root / first.ATTEMPT / "request.json").read_bytes()
        result = self.start(2, "omit_local_growth")
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["diagnostic_budget"]["reserved_attempts"], 2)
        self.assertEqual(result["diagnostic_budget"]["unreserved_slots"], 1)
        request = self.load(diagnostic.BATCH_ROOT + "/attempt_002/request.json")
        self.assertEqual(request["profile"], "omit_local_growth")
        self.assertEqual(Path(request["authorization"]["path"]).name, Path(diagnostic.GRANT).name)
        self.assertEqual((self.root / first.ATTEMPT / "request.json").read_bytes(), old_request)

    def test_second_slot_cannot_skip_the_first(self):
        with self.assertRaises(legacy.BridgeError):
            self.start(2)
        self.fixture.popen_mock.assert_not_called()
        self.assertFalse((self.root / ".sizing-dispatch.lock").exists())

    def test_running_slot_blocks_new_slot_without_consuming_it(self):
        self.start()
        with self.assertRaises(legacy.BridgeError):
            self.start(2)
        self.assertFalse((self.root / diagnostic.BATCH_ROOT / "attempt_002").exists())
        self.assertEqual(self.fixture.popen_mock.call_count, 1)

    def test_uncertain_cleanup_blocks_next_slot(self):
        self.start()
        self.finish(cleanup=False)
        with self.assertRaises(legacy.BridgeError):
            self.start(2)
        self.assertEqual(self.fixture.popen_mock.call_count, 1)

    def test_worker_must_exit_even_after_result_written(self):
        self.start()
        slot = self.finish()
        launcher = self.load(slot.ATTEMPT + "/launcher.json")
        self.live[launcher["pid"]] = launcher["creation_time"]
        with self.assertRaises(legacy.BridgeError):
            self.start(2)

    def test_three_separate_slots_preserve_all_requests(self):
        for index, profile in enumerate(diagnostic.PROFILES[:3], 1):
            self.assertEqual(self.start(index, profile)["status"], "accepted")
            self.finish(index)
        status = self.budget.status()
        self.assertEqual(status["reserved_attempts"], 3)
        self.assertEqual(status["unreserved_slots"], 0)
        self.assertEqual(self.fixture.popen_mock.call_count, 3)
        for index in range(1, 4):
            self.assertTrue((self.root / diagnostic.BATCH_ROOT / ("attempt_%03d" % index) / "request.json").is_file())

    def test_collected_underdefined_data_does_not_mean_definition_pass(self):
        self.start()
        slot = self.finish(fully_defined=False)
        result = slot.job_status(slot.JOB_ID)
        self.assertEqual(result["status"], "completed")
        self.assertFalse(result["control_fully_defined"])
        self.assertFalse(result["full_background_definitions_verified"])
        self.assertFalse(result["mesh_quality_verified"])

    def background_final(self):
        final = {key: True for key in ("full_background_definitions_verified", "diagnostic_completed", "zero_mesh_verified",
                 "owned_cleanup_verified", "sources_reverified_after_cleanup", "inputs_reverified_after_cleanup",
                 "message_capture_complete", "background_scopes_stable")}
        final.update(profile="background_all_controls", status="completed", native_errors_present=False,
                     background_definition_matches={phase: True for phase in ("define", "refresh", "readback")})
        controls = [{"name": name, "object_state": "FullyDefined", "actual_geometry_ids": [1], "expected_geometry_ids": [1]}
                    for name in ("stage15_background_tet", "stage15_wall_base0", "stage15_wall_auv", "stage15_overlap_0")]
        final["receipts"] = {phase: {"message_capture_complete": True, "native_errors_present": False,
                           "background_controls": copy.deepcopy(controls)} for phase in ("define", "refresh", "readback")}
        return final

    def test_whole_background_flag_requires_three_complete_control_sets(self):
        final = self.background_final()
        self.assertTrue(diagnostic.SizingSlot.verified_background_claim(final))
        final["receipts"]["readback"]["background_controls"].pop()
        self.assertFalse(diagnostic.SizingSlot.verified_background_claim(final))

    def test_whole_background_flag_rejects_undefined_or_wrong_scope(self):
        for key, value in (("object_state", "UnderDefined"), ("actual_geometry_ids", [2])):
            final = self.background_final()
            final["receipts"]["refresh"]["background_controls"][1][key] = value
            self.assertFalse(diagnostic.SizingSlot.verified_background_claim(final))

    def test_whole_background_flag_is_not_available_to_base0_only_profile(self):
        final = self.background_final()
        final["profile"] = "omit_local_growth"
        self.assertFalse(diagnostic.SizingSlot.verified_background_claim(final))

    def test_false_zero_mesh_claim_without_cleanup_is_rejected(self):
        self.start()
        slot = self.finish(cleanup=False)
        result = slot.job_status(slot.JOB_ID)
        self.assertEqual(result["status"], "failed")

    def test_new_confirmation_only_cancels_matching_slot(self):
        self.start()
        slot = self.budget.job("sizing-b001-a001")
        with self.assertRaises(legacy.BridgeError):
            slot.cancel(slot.JOB_ID, legacy.CANCEL_CONFIRMATION)
        result = slot.cancel(slot.JOB_ID, diagnostic.CANCEL_TOKEN)
        self.assertEqual(result["status"], "cancellation_requested")
        self.assertEqual(self.load(slot.ATTEMPT + "/cancel_request.json")["operation"], "cancel_sizing_diagnostic")

    def test_log_paths_are_job_scoped(self):
        self.start()
        slot = self.budget.job("sizing-b001-a001")
        self.put(slot.ATTEMPT + "/sizing_define_stats.json", {"status": "diagnostic_only"})
        self.assertEqual(slot.job_log(slot.JOB_ID, "define")["status"], "available")
        with self.assertRaises(legacy.BridgeError):
            slot.job_log(slot.JOB_ID, "../../contract.json")

    def test_legacy_globals_and_paths_are_not_mutated(self):
        self.start()
        self.assertEqual(legacy.JOB_ID, "definition-probe-001")
        self.assertEqual(self.fixture.bridge.JOB_ID, "definition-probe-001")
        self.assertEqual(self.fixture.bridge.ATTEMPT, legacy.ATTEMPT)
        self.assertFalse((self.root / legacy.ATTEMPT).exists())

    def test_another_ownership_lock_is_not_removed(self):
        self.put(".ansys-task.lock", {"token": "other-owner"})
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertEqual(self.load(".ansys-task.lock")["token"], "other-owner")
        self.assertFalse((self.root / ".sizing-dispatch.lock").exists())
        self.fixture.popen_mock.assert_not_called()

    def test_dispatch_lock_is_never_blindly_cleared(self):
        self.put(".sizing-dispatch.lock", {"token": "other-server"})
        with self.assertRaises(legacy.BridgeError):
            self.start()
        self.assertEqual(self.load(".sizing-dispatch.lock")["token"], "other-server")
        self.fixture.popen_mock.assert_not_called()

    def test_same_slot_concurrent_calls_cannot_double_launch(self):
        def requested(_):
            try:
                return self.start()
            except legacy.BridgeError:
                return {"status": "blocked"}
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(requested, range(4)))
        self.assertEqual(self.fixture.popen_mock.call_count, 1)
        self.assertEqual(sum(item.get("new_worker_started", False) for item in results), 1)


if __name__ == "__main__":
    unittest.main()

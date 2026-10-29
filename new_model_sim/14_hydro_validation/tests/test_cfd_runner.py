"""Offline CFD protocol tests: no ANSYS installation or solver is required."""
import copy
import errno
import importlib.util
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

CFD = Path(__file__).resolve().parents[1] / "cfd"
sys.path.insert(0, str(CFD))
import run_case as runner
import run_validation


class AtomicJsonWriteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=str(CFD), prefix="_offline_atomic_")
        self.root = Path(self.temp.name)
        self.path = self.root / "history.json"
        self.old_bytes = b'{"generation": "old", "complete": true}\n'
        self.path.write_bytes(self.old_bytes)
        self.new_value = {"generation": "new", "complete": True, "values": [1, 2, 3]}
        self.real_replace = runner.os.replace
        self.real_sleep = runner.time.sleep

    def tearDown(self):
        self.temp.cleanup()

    def windows_error(self, code):
        error = PermissionError(errno.EACCES, "injected Windows replacement denial")
        error.winerror = code
        return error

    def test_windows_access_sharing_lock_errors_recover_preserving_old_snapshot(self):
        for code in (5, 32, 33):
            with self.subTest(winerror=code):
                self.path.write_bytes(self.old_bytes)
                sources = []
                def blocked_then_replace(source, destination):
                    self.assertEqual(self.path.read_bytes(), self.old_bytes)
                    self.assertEqual(Path(source).parent, self.path.parent)
                    self.assertEqual(json.loads(Path(source).read_text(encoding="utf-8")), self.new_value)
                    sources.append(source)
                    if len(sources) <= 2:
                        raise self.windows_error(code)
                    return self.real_replace(source, destination)
                with patch.object(runner.os, "replace", side_effect=blocked_then_replace) as replace, patch.object(runner.time, "sleep", wraps=self.real_sleep) as pause, patch.object(runner._LOG, "warning") as warning:
                    runner.write_json(self.path, self.new_value)
                self.assertGreaterEqual(replace.call_count, 3)
                self.assertLessEqual(replace.call_count, 6)
                self.assertEqual(len(set(sources)), 1)
                self.assertEqual([c[0][0] for c in pause.call_args_list], list(runner.JSON_REPLACE_RETRY_DELAYS_S[:pause.call_count]))
                self.assertEqual(warning.call_count, pause.call_count)
                self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")), self.new_value)
                self.assertEqual(list(self.root.glob("history.json.*.tmp")), [])

    def test_exhausted_retries_reraise_and_preserve_old_and_complete_pending_file(self):
        failure = self.windows_error(5)
        sources = []
        def always_blocked(source, destination):
            self.assertEqual(self.path.read_bytes(), self.old_bytes)
            self.assertEqual(json.loads(Path(source).read_text(encoding="utf-8")), self.new_value)
            sources.append(source)
            raise failure
        with patch.object(runner.os, "replace", side_effect=always_blocked) as replace, patch.object(runner.time, "sleep") as pause, patch.object(runner._LOG, "warning"):
            with self.assertRaises(PermissionError) as caught:
                runner.write_json(self.path, self.new_value)
        self.assertIs(caught.exception, failure)
        self.assertEqual(replace.call_count, 6)
        self.assertEqual([c[0][0] for c in pause.call_args_list], list(runner.JSON_REPLACE_RETRY_DELAYS_S))
        self.assertAlmostEqual(sum(c[0][0] for c in pause.call_args_list), .77)
        self.assertEqual(len(set(sources)), 1)
        self.assertEqual(self.path.read_bytes(), self.old_bytes)
        pending = list(self.root.glob("history.json.*.tmp"))
        self.assertEqual(len(pending), 1)
        self.assertEqual(json.loads(pending[0].read_text(encoding="utf-8")), self.new_value)

    def test_unrelated_windows_and_generic_permission_errors_are_not_retried(self):
        generic = PermissionError(errno.EACCES, "no Windows error code")
        failures = [generic, self.windows_error(2), self.windows_error(112), OSError(errno.ENOSPC, "disk full")]
        for failure in failures:
            with self.subTest(error=repr(failure), winerror=getattr(failure, "winerror", None)):
                with patch.object(runner.os, "replace", side_effect=failure) as replace, patch.object(runner.time, "sleep") as pause:
                    with self.assertRaises(OSError) as caught:
                        runner.write_json(self.path, self.new_value)
                self.assertIs(caught.exception, failure)
                self.assertEqual(replace.call_count, 1)
                pause.assert_not_called()
                self.assertEqual(self.path.read_bytes(), self.old_bytes)

    def test_serialization_failure_does_not_create_candidate_or_retry(self):
        with patch.object(runner.os, "replace") as replace, patch.object(runner.time, "sleep") as pause:
            with self.assertRaises(ValueError):
                runner.write_json(self.path, {"invalid": float("nan")})
        replace.assert_not_called()
        pause.assert_not_called()
        self.assertEqual(self.path.read_bytes(), self.old_bytes)
        self.assertEqual(list(self.root.glob("history.json.*.tmp")), [])

    def test_flush_failure_is_not_a_replace_retry(self):
        failure = OSError(errno.ENOSPC, "injected flush failure")
        with patch.object(runner.os, "fsync", side_effect=failure), patch.object(runner.os, "replace") as replace, patch.object(runner.time, "sleep") as pause:
            with self.assertRaises(OSError) as caught:
                runner.write_json(self.path, self.new_value)
        self.assertIs(caught.exception, failure)
        replace.assert_not_called()
        pause.assert_not_called()
        self.assertEqual(self.path.read_bytes(), self.old_bytes)

    def test_success_uses_unique_siblings_and_keeps_previous_pending_file(self):
        previous_pending = self.root / "history.json.tmp"
        previous_pending.write_bytes(b"retained previous failed attempt")
        with patch.object(runner.os, "replace", wraps=self.real_replace) as replace:
            runner.write_json(self.path, self.new_value)
            runner.write_json(self.path, {"generation": "next"})
        sources = {call[0][0] for call in replace.call_args_list}
        self.assertEqual(len(sources), 2)
        self.assertTrue(all(Path(source).parent == self.path.parent for source in sources))
        self.assertEqual(previous_pending.read_bytes(), b"retained previous failed attempt")
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")), {"generation": "next"})
        self.assertEqual(list(self.root.glob("history.json.*.tmp")), [])


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=str(CFD), prefix="_offline_test_")
        self.root = Path(self.temp.name)
        (self.root / "cfd").mkdir()
        (self.root / "frozen").mkdir()
        self.config = self.root / "cfd/validation_cases.json"
        self.config.write_bytes((CFD / "validation_cases.json").read_bytes())
        self.cfg = runner.load_config(self.config)
        self.manifest = self.root / "frozen/blind_protocol.json"
        self.model = self.root / "frozen/model.json"
        self.model.write_text('{"weights": []}', encoding="utf-8")
        self.record = {
            "status": "frozen_before_cfd", "case_config_sha256": runner.sha256(self.config),
            "approved_case_ids": [c["id"] for c in self.cfg["cases"]],
            "files": [{"path": "cfd/validation_cases.json", "sha256": runner.sha256(self.config)},
                      {"path": "frozen/model.json", "sha256": runner.sha256(self.model)}]}
        self.save_manifest()

    def tearDown(self):
        self.temp.cleanup()

    def save_manifest(self):
        runner.write_json(self.manifest, self.record)

    def verify(self):
        return runner.verify_freeze(self.manifest, self.config, self.root)

    def test_nine_approved_modes_speeds_and_opposite_inflow(self):
        cases = self.cfg["cases"]
        steady = [c for c in cases if c["mode"] == "steady_translation"]
        self.assertEqual(len(cases), 9)
        self.assertEqual(len(steady), 8)
        self.assertEqual(self.cfg["origin_m"], [0, .29386, -.06630])
        for c in steady:
            self.assertAlmostEqual(math.sqrt(sum(x*x for x in c["body_velocity_m_s"])), .55)
            self.assertAlmostEqual(sum(x*x for x in c["direction"]), 1.)
            for d, v, w in zip(c["direction"], c["body_velocity_m_s"], c["water_velocity_m_s"]):
                self.assertAlmostEqual(v, .55*d)
                self.assertAlmostEqual(v, -w)
        osc = cases[-1]
        self.assertEqual(osc["time_steps"], 240)
        self.assertEqual(osc["max_iterations_per_step"], 30)
        self.assertEqual(osc["viscous_model"], "laminar")
        self.assertAlmostEqual(osc["time_steps"] * osc["time_step_s"], 6.)

    def test_unknown_mode_rejected(self):
        self.cfg["cases"][0]["mode"] = "rotation"
        runner.write_json(self.config, self.cfg)
        with self.assertRaisesRegex(runner.ProtocolError, "unknown mode"):
            runner.load_config(self.config)

    def test_direction_and_inflow_corruption_rejected(self):
        self.cfg["cases"][0]["water_velocity_m_s"][0] *= -1
        runner.write_json(self.config, self.cfg)
        with self.assertRaises(runner.ProtocolError):
            runner.load_config(self.config)

    def test_valid_freeze(self):
        self.assertEqual(self.verify()["case_config_sha256"], runner.sha256(self.config))

    def test_missing_manifest_rejected(self):
        with self.assertRaises(runner.ProtocolError):
            runner.verify_freeze(None, self.config, self.root)
        self.manifest.unlink()
        with self.assertRaises(runner.ProtocolError):
            self.verify()

    def test_missing_frozen_file_rejected(self):
        self.model.unlink()
        with self.assertRaisesRegex(runner.ProtocolError, "missing/hash mismatch"):
            self.verify()

    def test_hash_mismatch_rejected(self):
        self.model.write_text("changed", encoding="utf-8")
        with self.assertRaises(runner.ProtocolError):
            self.verify()

    def test_config_hash_and_ids_rejected(self):
        self.record["case_config_sha256"] = "0" * 64
        self.save_manifest()
        with self.assertRaises(runner.ProtocolError):
            self.verify()
        self.record["case_config_sha256"] = runner.sha256(self.config)
        self.record["approved_case_ids"].pop()
        self.save_manifest()
        with self.assertRaises(runner.ProtocolError):
            self.verify()

    def test_unfrozen_and_path_traversal_rejected(self):
        self.record["status"] = "draft"
        self.save_manifest()
        with self.assertRaises(runner.ProtocolError):
            self.verify()
        self.record["status"] = "frozen_before_cfd"
        self.record["files"][1]["path"] = "../outside.json"
        self.save_manifest()
        with self.assertRaises(runner.ProtocolError):
            self.verify()

    def test_check_cannot_launch_solver(self):
        with patch.object(runner, "solve_attempt", side_effect=AssertionError("must not solve")), patch.object(runner, "execute", side_effect=AssertionError("must not execute")):
            result = runner.check_case(self.cfg["cases"][0]["id"], self.config, stage_root=self.root)
        self.assertTrue(result["check_only"])
        self.assertFalse(result["solver_launched"])
        self.assertFalse(result["ready_to_run"])
        self.assertFalse((self.root / "cfd/runs").exists())

    def test_real_execution_missing_freeze_cannot_launch_or_create_attempt(self):
        with patch.object(runner, "solve_attempt", side_effect=AssertionError("must not solve")), patch.object(runner, "serial_lock", side_effect=AssertionError("must not create runs")):
            with self.assertRaises(runner.ProtocolError):
                runner.execute(self.cfg["cases"][0]["id"], self.config)

    def test_pose_axes_and_centres_are_not_q0(self):
        q0 = runner.posed_joints(self.cfg, [0, 0, 0])
        qe = runner.posed_joints(self.cfg, [.225, -.25, -.20])
        self.assertEqual(q0[0]["centre_m"], qe[0]["centre_m"])
        self.assertNotEqual(q0[1]["axis"], qe[1]["axis"])
        self.assertNotEqual(q0[2]["centre_m"], qe[2]["centre_m"])
        for j in qe:
            self.assertAlmostEqual(sum(x*x for x in j["axis"]), 1.)

    def test_diagonal_motion_and_ramped_integral(self):
        c = self.cfg["cases"][-1]
        for t in (.1, .4, .7, 2.5):
            state = runner.oscillation_state(c, t)
            self.assertEqual(state["body_velocity_m_s"][0], 0)
            self.assertEqual(state["body_velocity_m_s"][1], state["body_velocity_m_s"][2])
            eps = 1e-6
            left, right = runner.oscillation_state(c, t-eps), runner.oscillation_state(c, t+eps)
            for i in range(3):
                numeric = (right["displacement_m"][i] - left["displacement_m"][i]) / (2*eps)
                self.assertAlmostEqual(numeric, state["body_velocity_m_s"][i], places=8)

    def completed_fixture(self):
        attempt = self.root / "attempt_001"
        attempt.mkdir()
        identity = {"case": "test", "hash": "frozen"}
        state = {"id": "same-terminal-state", "sample_count": 1, "iterations": 400}
        history = [{"iter": 400, "loads": {k: 0 for k in runner.CHANNELS}}]
        runner.write_json(attempt / "history.json", history)
        artifacts = {}
        for key, filename in (("start_case", "start.cas.h5"), ("final_case", "final.cas.h5"), ("final_data", "final.dat.h5"), ("history", "history.json"), ("transcript", "solver.trn")):
            if key != "history":
                (attempt / filename).write_bytes(b"offline mock bytes, not Fluent data")
            artifacts[key] = {"path": filename, "sha256": runner.sha256(attempt / filename)}
            if key in ("final_case", "final_data"):
                artifacts[key]["state_id"] = state["id"]
        report = {"status": "completed", "identity": identity, "failed_required": [], "execution_complete": True,
                  "terminal_state": state, "mode": "steady_translation", "converged": True, "artifacts": artifacts}
        runner.write_json(attempt / "result.json", report)
        return attempt, identity, report

    def test_started_is_not_completed(self):
        attempt, identity, report = self.completed_fixture()
        report["status"] = "started"
        runner.write_json(attempt / "result.json", report)
        self.assertFalse(runner.valid_completed(attempt, identity))

    def test_completed_requires_matching_metadata_case_and_data(self):
        attempt, identity, report = self.completed_fixture()
        self.assertTrue(runner.valid_completed(attempt, identity))
        self.assertFalse(runner.valid_completed(attempt, {"different": "metadata"}))
        (attempt / "final.dat.h5").unlink()
        self.assertFalse(runner.valid_completed(attempt, identity))

    def test_successful_skip_uses_verified_attempt_only(self):
        attempt, identity, report = self.completed_fixture()
        root = self.root / "runs"
        parent = root / "case"
        parent.mkdir(parents=True)
        destination = parent / "attempt_001"
        attempt.rename(destination)
        selected, skipped = runner.select_attempt(root, "case", identity)
        self.assertTrue(skipped)
        self.assertEqual(selected, destination)
        self.assertEqual(len(list(parent.iterdir())), 1)

    def test_all_new_python_scripts_parse_without_execution(self):
        import ast
        sources = list(CFD.glob("*.py")) + list((CFD / "qe").glob("*.py"))
        self.assertGreaterEqual(len(sources), 5)
        for source in sources:
            with self.subTest(source=source.name):
                ast.parse(source.read_text(encoding="utf-8"), filename=str(source))

    def test_preparation_script_has_no_flow_initialization_or_iteration_calls(self):
        import ast
        tree = ast.parse((CFD / "prepare_mesh_base.py").read_text(encoding="utf-8"))
        forbidden = {"iterate", "dual_time_iterate", "hybrid_initialize", "standard_initialize"}
        called = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        self.assertFalse(called & forbidden)

    def test_completed_rejects_changed_case_and_mixed_terminal_state(self):
        attempt, identity, report = self.completed_fixture()
        report["artifacts"]["final_data"]["state_id"] = "another-state"
        runner.write_json(attempt / "result.json", report)
        self.assertFalse(runner.valid_completed(attempt, identity))
        report["artifacts"]["final_data"]["state_id"] = report["terminal_state"]["id"]
        runner.write_json(attempt / "result.json", report)
        (attempt / "final.cas.h5").write_bytes(b"modified")
        self.assertFalse(runner.valid_completed(attempt, identity))

    def test_only_one_explicit_retry_history_kept(self):
        root = self.root / "runs"
        first, skipped = runner.select_attempt(root, "case", {})
        runner.write_json(first / "result.json", {"status": "started"})
        with self.assertRaises(runner.ProtocolError):
            runner.select_attempt(root, "case", {})
        second, skipped = runner.select_attempt(root, "case", {}, "recorded settings repair")
        self.assertEqual(second.name, "attempt_002")
        self.assertTrue((first / "result.json").exists())
        with self.assertRaises(runner.ProtocolError):
            runner.select_attempt(root, "case", {}, "second repair not allowed")


class MockSolverTests(unittest.TestCase):
    setUp = ProtocolTests.setUp
    tearDown = ProtocolTests.tearDown
    save_manifest = ProtocolTests.save_manifest
    verify = ProtocolTests.verify

    def solve_mock(self, fail_final_data=False, fail_init=False, oscillator=False):
        case = self.cfg["cases"][-1 if oscillator else 0]
        attempt = self.root / "mock_attempt"
        attempt.mkdir()
        base_path = self.root / "mock_base.cas.h5"
        base_path.write_bytes(b"mock base")
        base = {"path": str(base_path), "sha256": runner.sha256(base_path)}
        freeze = self.verify()
        identity = runner.identity_for(case, freeze, base)
        solver = MagicMock()
        solver.setup.cell_zone_conditions.fluid.keys.return_value = ["fff_water"]
        solver.setup.boundary_conditions.wall.keys.return_value = runner.BODY
        def transcript_start(path):
            Path(path).write_text("offline mock transcript", encoding="utf-8")
        solver.transcript.start.side_effect = transcript_start
        def write_file(file_type, file_name):
            if fail_final_data and file_type == "data":
                raise RuntimeError("mock final data failure")
            Path(file_name).write_bytes(("mock " + file_type).encode())
        solver.file.write.side_effect = write_file
        if fail_init:
            solver.solution.initialization.hybrid_initialize.side_effect = RuntimeError("mock initialization failure")
        def compute(report_defs):
            if report_defs == runner.REPS:
                return [{k: [0.0] for k in runner.CHANNELS}]
            return [{k: [0.0] for k in report_defs}]
        solver.solution.report_definitions.compute.side_effect = compute
        fluent = types.ModuleType("ansys.fluent.core")
        fluent.launch_fluent = MagicMock(return_value=solver)
        fluent.UIMode = types.SimpleNamespace(NO_GUI_OR_GRAPHICS="mock")
        ansys = types.ModuleType("ansys")
        ansys_fluent = types.ModuleType("ansys.fluent")
        ansys.fluent, ansys_fluent.core = ansys_fluent, fluent
        bootstrap = types.ModuleType("run_wb_fullenv")
        bootstrap.full_environment = lambda: {}
        modules = {"ansys": ansys, "ansys.fluent": ansys_fluent, "ansys.fluent.core": fluent, "run_wb_fullenv": bootstrap}
        error = None
        with patch.dict(sys.modules, modules), patch.object(runner, "verify_freeze", return_value=freeze):
            try:
                result = runner.solve_attempt(self.cfg, case, freeze, base, identity, attempt, None, self.config, self.manifest)
            except RuntimeError as exc:
                error, result = exc, None
        return solver, attempt, identity, result, error

    def test_mock_success_writes_start_and_same_terminal_pair(self):
        solver, attempt, identity, result, error = self.solve_mock()
        self.assertIsNone(error)
        self.assertEqual(result["status"], "completed")
        self.assertTrue(runner.valid_completed(attempt, identity))
        self.assertEqual(solver.solution.run_calculation.iterate.call_count, 16)
        self.assertEqual(solver.solution.run_calculation.dual_time_iterate.call_count, 0)
        self.assertEqual(runner.MASS_FLOW_REPORT_TYPE, "flux-massflow")
        flux_states = [call[0][1] for call in solver.solution.report_definitions.flux.__setitem__.call_args_list]
        self.assertEqual(flux_states, [{"report_type": "flux-massflow", "boundaries": [zone]} for zone in runner.BOX_N])

    def test_mock_failed_required_init_stops_without_iteration(self):
        solver, attempt, identity, result, error = self.solve_mock(fail_init=True)
        self.assertIsNotNone(error)
        self.assertEqual(solver.solution.run_calculation.iterate.call_count, 0)
        self.assertFalse(runner.valid_completed(attempt, identity))
        report = json.loads((attempt / "result.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertIn("hybrid_initialize", report["failed_required"])

    def test_mock_missing_final_data_never_completed(self):
        solver, attempt, identity, result, error = self.solve_mock(fail_final_data=True)
        self.assertIsNotNone(error)
        self.assertTrue((attempt / "final.cas.h5").exists())
        self.assertFalse(runner.valid_completed(attempt, identity))

    def test_mock_oscillation_exactly_240_steps_and_diagonal_velocity(self):
        solver, attempt, identity, result, error = self.solve_mock(oscillator=True)
        self.assertIsNone(error)
        self.assertEqual(solver.solution.run_calculation.dual_time_iterate.call_count, 240)
        self.assertEqual(solver.solution.run_calculation.iterate.call_count, 0)
        self.assertTrue(runner.valid_completed(attempt, identity))
        history = json.loads((attempt / "history.json").read_text())
        self.assertEqual(history[-1]["t"], 6.)
        self.assertEqual(history[0]["motion"]["body_velocity_m_s"][1], history[0]["motion"]["body_velocity_m_s"][2])


if __name__ == "__main__":
    unittest.main(verbosity=2)

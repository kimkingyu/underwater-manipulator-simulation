"""Only in-memory mock CFD artifacts; never read/write prospective cfd/runs."""
import copy
import fnmatch
import hashlib
import importlib.util
import json
import os
import sys
import types
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest import mock

import numpy as np

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import evaluate_blind as ev


class MemoryFiles:
    """Patch only file reads/stat/glob/hash; every mock artifact exists solely in RAM."""
    def __init__(self, root):
        self.root = root.resolve()
        self.files = {}
        self.read_log = []
        self.real_read_text = Path.read_text
        self.real_is_file = Path.is_file
        self.real_is_dir = Path.is_dir
        self.real_stat = Path.stat
        self.real_glob = Path.glob
        self.real_sha = ev.runner.sha256

    def key(self, path):
        return str(Path(path).resolve())

    def owns(self, path):
        return Path(path).resolve().is_relative_to(self.root)

    def add_bytes(self, path, data):
        self.files[self.key(path)] = bytes(data)

    def add_json(self, path, value):
        self.add_bytes(path, json.dumps(ev.dp.serializable(value), sort_keys=True, allow_nan=False).encode())

    def json(self, path):
        return json.loads(self.files[self.key(path)])

    def delete(self, path):
        self.files.pop(self.key(path), None)

    def digest(self, path):
        if self.owns(path):
            return hashlib.sha256(self.files[self.key(path)]).hexdigest()
        if Path(path).resolve() == Path(ev.runner.__file__).resolve():
            return self.real_sha(path)
        raise AssertionError("unexpected real file hash in mock: " + str(path))

    def read_text(self, path, *args, **kwargs):
        self.read_log.append(self.key(path))
        if self.owns(path):
            if self.key(path) not in self.files:
                raise FileNotFoundError(str(path))
            return self.files[self.key(path)].decode(kwargs.get("encoding") or "utf-8")
        if Path(path).resolve().is_relative_to(ROOT / "cfd/runs"):
            raise AssertionError("REAL BLIND LOAD READ FORBIDDEN")
        return self.real_read_text(path, *args, **kwargs)

    def is_file(self, path):
        return self.key(path) in self.files if self.owns(path) else self.real_is_file(path)

    def is_dir(self, path):
        if not self.owns(path):
            return self.real_is_dir(path)
        return any(Path(name).parent == Path(path).resolve() or Path(name).is_relative_to(Path(path).resolve()) for name in self.files)

    def stat(self, path, *args, **kwargs):
        if self.owns(path):
            if self.key(path) not in self.files:
                raise FileNotFoundError(str(path))
            return types.SimpleNamespace(st_size=len(self.files[self.key(path)]))
        return self.real_stat(path, *args, **kwargs)

    def glob(self, path, pattern, *args, **kwargs):
        if not self.owns(path):
            if Path(path).resolve().is_relative_to(ROOT / "cfd/runs"):
                raise AssertionError("REAL BLIND DIRECTORY ACCESS FORBIDDEN")
            return self.real_glob(path, pattern, *args, **kwargs)
        parent = Path(path).resolve()
        children = set()
        for filename in self.files:
            for ancestor in Path(filename).parents:
                if ancestor.parent == parent and fnmatch.fnmatch(ancestor.name, pattern):
                    children.add(ancestor)
        return iter(sorted(children))

    def patched(self):
        stack = ExitStack()
        # Python 3.10 Windows Path.resolve calls stat; use lexical resolution in
        # this symlink-free RAM fixture, so mocked stat never recursively resolves.
        stack.enter_context(mock.patch.object(Path, "resolve", lambda path, *a, **k: Path(os.path.abspath(path))))
        stack.enter_context(mock.patch.object(Path, "read_text", lambda path, *a, **k: self.read_text(path, *a, **k)))
        stack.enter_context(mock.patch.object(Path, "is_file", lambda path: self.is_file(path)))
        stack.enter_context(mock.patch.object(Path, "is_dir", lambda path: self.is_dir(path)))
        stack.enter_context(mock.patch.object(Path, "stat", lambda path, *a, **k: self.stat(path, *a, **k)))
        stack.enter_context(mock.patch.object(Path, "glob", lambda path, *a, **k: self.glob(path, *a, **k)))
        stack.enter_context(mock.patch.object(ev.runner, "sha256", side_effect=self.digest))
        return stack


class BlindEvaluationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.real_cfg = ev.read_json(ROOT / "cfd/validation_cases.json")
        cls.reference = ev.dp.reference_from_contract(ev.read_json(ROOT / "contract.json"))
        cls.protocol = ev.read_json(ROOT / "frozen/evaluation_protocol.json")
        cls.steady_prior = ev.read_json(ROOT / "frozen/steady_prior_predictions.json")
        cls.dynamic_prior = ev.read_json(ROOT / "frozen/oscillation_prior_predictions.json")
        cls.theta = ev.read_json(ROOT / "frozen/theta87_baseline.json")["theta"]
        cls.dynamic_model = ev.read_json(ROOT / "models/dynamic_linear_q0.json")
        cls.legacy_predictions = [ev.frozen87_dynamic(cls.theta, [0., 0., 0.], v, a, cls.reference)
                                  for v, a in zip(cls.dynamic_prior["nu"], cls.dynamic_prior["nudot"])]
        cls.legacy_prior = {"time_s": cls.dynamic_prior["time_s"], "nu": cls.dynamic_prior["nu"],
                            "nudot": cls.dynamic_prior["nudot"], "predictions": cls.legacy_predictions}

    def setUp(self):
        self.root = ROOT / "reports/evaluator_mock_workspace_never_created"
        self.fs = MemoryFiles(self.root)
        self.cfg = copy.deepcopy(self.real_cfg)
        for pose in ("q0", "qe"):
            content = ("mock base " + pose).encode()
            path = "models/mock_base_" + pose + ".cas.h5"
            self.cfg["poses"][pose]["base_case"] = path
            self.cfg["poses"][pose]["base_sha256"] = hashlib.sha256(content).hexdigest()
            self.fs.add_bytes(self.root / path, content)
        for path in ev.REQUIRED_FROZEN:
            self.fs.add_bytes(self.root / path, b"mock-frozen-dependency")
        self.fs.add_json(self.root / "cfd/validation_cases.json", self.cfg)
        self.fs.add_json(self.root / "contract.json", ev.read_json(ROOT / "contract.json"))
        self.fs.add_json(self.root / "frozen/evaluation_protocol.json", self.protocol)
        self.fs.add_json(self.root / "frozen/steady_prior_predictions.json", self.steady_prior)
        self.fs.add_json(self.root / "frozen/oscillation_prior_predictions.json", self.dynamic_prior)
        self.fs.add_json(self.root / "frozen/legacy87_oscillation_prior.json", self.legacy_prior)
        self.fs.add_json(self.root / "frozen/theta87_baseline.json", {"theta": self.theta})
        self.fs.add_json(self.root / "models/dynamic_linear_q0.json", self.dynamic_model)
        manifest = {"status": "frozen_before_cfd", "case_config_sha256": self.fs.digest(self.root / "cfd/validation_cases.json"),
                    "approved_case_ids": [c["id"] for c in self.cfg["cases"]],
                    "files": [{"path": p, "sha256": self.fs.digest(self.root / p)} for p in sorted(ev.REQUIRED_FROZEN)]}
        self.fs.add_json(self.root / "frozen/blind_protocol.json", manifest)
        self.freeze = {"path": str((self.root / "frozen/blind_protocol.json").resolve()),
                       "sha256": self.fs.digest(self.root / "frozen/blind_protocol.json"),
                       "case_config_sha256": manifest["case_config_sha256"]}

    def raw_loads(self):
        loads = {name: 0. for name in ev.runner.CHANNELS}
        for i, wall in enumerate(ev.runner.BODY):
            loads[f"fx({wall})"] = -.1 * (i + 1)
            loads[f"fy({wall})"] = .03 * (i + 1)
            loads[f"fz({wall})"] = -.02 * (i + 1)
        for kind in "fm":
            for axis in "xyz":
                loads[kind + axis] = sum(loads[f"{kind}{axis}({wall})"] for wall in ev.runner.BODY)
        return loads

    def law(self, case=None):
        case = case or self.cfg["cases"][-1]
        return {"peak_velocity_m_s": (np.asarray(case["direction"]) * case["amplitude_m"] * np.pi).tolist(),
                "angular_frequency_rad_s": float(np.pi), "ramp_s": .5, "source": "mock saved HDF expression"}

    def create_attempt(self, index=0, capped=False, offset=(.001, -.002, .003)):
        case = self.cfg["cases"][index]
        attempt = self.root / "cfd/runs" / case["id"] / "attempt_001"
        history = []
        if case["mode"] == "steady_translation":
            count = 2500 if capped else 400
            history = [{"iter": i, "loads": self.raw_loads()} for i in range(25, count + 1, 25)]
            terminal = {"id": "mock-terminal", "sample_count": len(history), "iterations": count}
        else:
            for n in range(1, 241):
                t = n * .025
                motion = ev.runner.oscillation_state(case, t)
                mesh = np.asarray(motion["displacement_m"]) + np.asarray(offset)
                motion["mesh_displacement_m"] = mesh.tolist()
                motion["mesh_minus_commanded_displacement_m"] = list(offset)
                history.append({"t": t, "loads": self.raw_loads(), "motion": motion,
                                "moving_origin_m": (np.asarray(self.cfg["origin_m"]) + mesh).tolist()})
            terminal = {"id": "mock-terminal", "sample_count": 240, "time_steps": 240, "flow_time_s": 6.}
        self.fs.add_json(attempt / "history.json", history)
        artifacts = {}
        for key, filename in (("start_case", "start.cas.h5"), ("final_case", "final.cas.h5"), ("final_data", "final.dat.h5"),
                              ("history", "history.json"), ("transcript", "solver.trn")):
            if key != "history":
                self.fs.add_bytes(attempt / filename, ("mock artifact " + filename).encode())
            artifacts[key] = {"path": filename, "sha256": self.fs.digest(attempt / filename)}
            if key in ("final_case", "final_data"):
                artifacts[key]["state_id"] = terminal["id"]
        with self.fs.patched():
            base = ev.runner.base_status(self.cfg, case, self.root)
            identity = ev.runner.identity_for(case, self.freeze, base)
        report = {"status": "completed", "case_id": case["id"], "mode": case["mode"], "spec": case,
                  "identity": identity, "origin_m": self.cfg["origin_m"], "failed_required": [], "steps": [],
                  "joint_reports_initial": ev.runner.posed_joints(self.cfg, case["q"]),
                  "execution_complete": True, "terminal_state": terminal, "artifacts": artifacts}
        if case["mode"] == "steady_translation":
            report.update(converged=not capped, outcome="capped_unsteady_mean" if capped else "strict_window_drift",
                          drift={"maximum": 0.})
        else:
            report["outcome"] = "240_steps_completed_periodicity_reported_not_assumed"
        self.fs.add_json(attempt / "result.json", report)
        return attempt, case

    def collect(self, attempt, case):
        with self.fs.patched(), mock.patch.object(ev, "read_actual_motion_law", return_value=self.law()):
            return ev.read_verified_case(attempt, case, self.cfg, self.freeze, self.reference, self.root)

    def rewrite_history_and_hash(self, attempt, history):
        self.fs.add_json(attempt / "history.json", history)
        report = self.fs.json(attempt / "result.json")
        report["artifacts"]["history"]["sha256"] = self.fs.digest(attempt / "history.json")
        self.fs.add_json(attempt / "result.json", report)

    def test_missing_protocol_pending_without_truth_access(self):
        self.fs.delete(self.root / "frozen/blind_protocol.json")
        with self.fs.patched():
            result = ev.evaluate(self.root, score=True)
        self.assertEqual(result["status"], "pending")
        self.assertFalse(result["blind_loads_opened"])
        self.assertFalse(any("history.json" in p for p in self.fs.read_log))

    def test_verified_status_never_opens_future_results(self):
        self.create_attempt()
        self.fs.read_log.clear()
        with self.fs.patched():
            result = ev.evaluate(self.root, score=False)
        self.assertEqual(result["reason"], "explicit_score_not_requested")
        self.assertFalse(any(p.endswith("result.json") or p.endswith("history.json") for p in self.fs.read_log))

    def test_freeze_hash_change_rejected_before_truth(self):
        self.create_attempt()
        self.fs.add_bytes(self.root / "models/candidate_A_strict37.json", b"changed parameters")
        with self.fs.patched(), self.assertRaises(ev.runner.ProtocolError):
            ev.evaluate(self.root, score=True)
        self.assertFalse(any(p.endswith("history.json") for p in self.fs.read_log))

    def test_missing_scoring_dependency_rejected(self):
        path = self.root / "frozen/blind_protocol.json"
        manifest = self.fs.json(path)
        manifest["files"] = [r for r in manifest["files"] if r["path"] != "evaluate_blind.py"]
        self.fs.add_json(path, manifest)
        with self.fs.patched(), self.assertRaises(ev.EvaluationError):
            ev.verify_evaluation_freeze(self.root)

    def test_no_real_results_returns_pending(self):
        with self.fs.patched():
            result = ev.evaluate(self.root, score=True)
        self.assertEqual(result["status"], "pending")
        self.assertEqual(len(result["pending_cases"]), 9)
        self.assertEqual(result["new_steady_strict_count"], 0)

    def test_missing_final_artifact_rejected(self):
        attempt, case = self.create_attempt()
        self.fs.delete(attempt / "final.dat.h5")
        with self.assertRaises(ev.EvaluationError):
            self.collect(attempt, case)

    def test_modified_artifact_hash_rejected(self):
        for filename in ("final.cas.h5", "final.dat.h5", "history.json"):
            with self.subTest(filename=filename):
                attempt, case = self.create_attempt()
                self.fs.add_bytes(attempt / filename, b"tampered")
                with self.assertRaises(ev.EvaluationError):
                    self.collect(attempt, case)

    def test_wrong_identity_pose_and_reference_rejected(self):
        for change in ("identity", "q", "origin", "joint"):
            with self.subTest(change=change):
                attempt, case = self.create_attempt()
                report = self.fs.json(attempt / "result.json")
                if change == "identity":
                    report["identity"]["base_sha256"] = "0" * 64
                elif change == "q":
                    report["spec"]["q"][0] = .1
                elif change == "origin":
                    report["origin_m"][0] += .01
                else:
                    report["joint_reports_initial"][1]["centre_m"][0] += .02
                self.fs.add_json(attempt / "result.json", report)
                with self.assertRaises(ev.EvaluationError):
                    self.collect(attempt, case)

    def test_actual_moving_origin_used_not_commanded_point(self):
        attempt, case = self.create_attempt(8)
        original = ev.dp.loads_to_components
        seen = []
        def traced(loads, q, ref, **kwargs):
            seen.append(np.asarray(kwargs["origin_global"]).copy())
            return original(loads, q, ref, **kwargs)
        with mock.patch.object(ev.dp, "loads_to_components", side_effect=traced):
            record = self.collect(attempt, case)
        history = self.fs.json(attempt / "history.json")
        np.testing.assert_allclose(seen, [h["moving_origin_m"] for h in history], rtol=0, atol=0)
        wrong = np.asarray(self.cfg["origin_m"]) + np.asarray(history[0]["motion"]["displacement_m"])
        self.assertGreater(np.max(abs(np.asarray(seen[0]) - wrong)), .001)
        wrong_loads = original(history[0]["loads"], case["q"], self.reference, origin_global=wrong)
        self.assertGreater(np.max(abs(record["total"][0] - wrong_loads["total"])), .001)

    def test_missing_or_inconsistent_actual_origin_rejected(self):
        for action in ("missing", "inconsistent"):
            attempt, case = self.create_attempt(8)
            history = self.fs.json(attempt / "history.json")
            if action == "missing":
                del history[1]["moving_origin_m"]
            else:
                history[1]["moving_origin_m"][0] += .1
            self.rewrite_history_and_hash(attempt, history)
            with self.assertRaises(ev.EvaluationError):
                self.collect(attempt, case)

    def test_capped_is_not_convergence_and_sensitivity_separate(self):
        attempt, case = self.create_attempt(0, capped=True)
        capped = self.collect(attempt, case)
        self.assertFalse(capped["strict"])
        self.assertFalse(capped["converged"])
        self.assertTrue(capped["execution_complete"])
        self.assertTrue(capped["capped"])
        self.assertEqual(capped["averaging"]["sample_count"], 8)
        prior = {p["id"]: p for p in self.steady_prior["cases"]}
        scores = ev.score_steadies([capped], prior, self.protocol)
        self.assertEqual(scores["complete_steady_claim"], "inconclusive")
        self.assertEqual(scores["strict_only"]["case_ids"], [])
        self.assertEqual(scores["strict_plus_capped_sensitivity"]["case_count"], 1)

    def test_claimed_strict_drift_failure_rejected(self):
        attempt, case = self.create_attempt()
        history = self.fs.json(attempt / "history.json")
        for row in history[-8:]:
            row["loads"]["fx"] += 100.
        self.rewrite_history_and_hash(attempt, history)
        with self.assertRaises(ev.EvaluationError):
            self.collect(attempt, case)

    def test_si_channel_metrics_and_equal_whole_case_weight(self):
        actual = np.zeros((4, 2))
        predicted = np.array([[1., -1.], [3., -3.], [3., -3.], [3., -3.]])
        score = ev.si_metrics(actual, predicted, [0, 1, 1, 1])
        np.testing.assert_allclose(score["MAE"], [2., 2.])
        np.testing.assert_allclose(score["RMSE"], [np.sqrt(5), np.sqrt(5)])
        np.testing.assert_allclose(score["signed_bias"], [2., -2.])
        np.testing.assert_allclose(score["maximum_absolute_error"], [3., 3.])
        self.assertEqual(score["case_count"], 2)

    def test_legacy_never_manufactures_pure_hull(self):
        views = ev.predicted_views(self.legacy_predictions)
        self.assertNotIn("hull_force3", views)
        self.assertNotIn("hull_moment3", views)
        self.assertNotIn("base0_6", views)
        self.assertIn("hull_plus_base0_6", views)
        with self.assertRaises(ev.EvaluationError):
            ev.frozen87_dynamic(self.theta, [.1, 0, 0], np.zeros(6), np.zeros(6), self.reference)

    def test_dynamic_windows_frequency_and_frozen_replay_no_fit(self):
        attempt, case = self.create_attempt(8)
        record = self.collect(attempt, case)
        before = json.dumps(self.dynamic_model, sort_keys=True)
        with mock.patch.object(ev.dl, "fit_model", side_effect=AssertionError("refitting forbidden")):
            predictions, replay = ev.predict_actual_dynamics(record, self.dynamic_model, self.theta, self.reference,
                                                             self.dynamic_prior, self.legacy_prior)
        self.assertEqual(before, json.dumps(self.dynamic_model, sort_keys=True))
        self.assertLess(max(replay.values()), 1e-10)
        score = ev.score_oscillation(record, predictions)
        self.assertEqual(score["sample_counts"]["full"], 240)
        self.assertEqual(score["sample_counts"]["startup_first_cycle"], 80)
        self.assertEqual(score["sample_counts"]["last_two_cycles"], 160)
        self.assertEqual(score["harmonic_angular_frequency_rad_s"], np.pi)
        self.assertNotIn("hull_force3", score["metrics"]["legacy87"]["full"])
        self.assertIn("hull_force3", score["metrics"]["dynamic_linear"]["full"])
        tampered = copy.deepcopy(self.legacy_prior)
        tampered["predictions"][0]["total"][0] += 1.
        with self.assertRaises(ev.EvaluationError):
            ev.predict_actual_dynamics(record, self.dynamic_model, self.theta, self.reference, self.dynamic_prior, tampered)

    def test_eight_completed_but_seven_strict_is_inconclusive(self):
        for i in range(9):
            self.create_attempt(i, capped=(i == 0))
        with self.fs.patched(), mock.patch.object(ev, "read_actual_motion_law", return_value=self.law()):
            result = ev.evaluate(self.root, score=True)
        self.assertEqual(result["status"], "inconclusive")
        self.assertEqual(result["new_steady_strict_count"], 7)
        self.assertEqual(result["steady"]["execution_complete_count"], 8)
        self.assertFalse(result["complete_acceptance_eligible"])
        self.assertEqual(len(result["oscillations"]), 1)
        self.assertFalse(result["parameters_changed_or_refitted"])

    def test_eight_strict_scope_and_no_model_mutation(self):
        for i in range(9):
            self.create_attempt(i)
        hashes_before = {p: self.fs.digest(self.root / p) for p in ev.REQUIRED_FROZEN}
        with self.fs.patched(), mock.patch.object(ev, "read_actual_motion_law", return_value=self.law()):
            result = ev.evaluate(self.root, score=True)
        hashes_after = {p: self.fs.digest(self.root / p) for p in ev.REQUIRED_FROZEN}
        self.assertEqual(hashes_before, hashes_after)
        self.assertEqual(result["status"], "scored_complete_scope")
        self.assertEqual(result["new_steady_strict_count"], 8)
        self.assertTrue(result["complete_acceptance_eligible"])
        self.assertFalse(result["default_theta_replacement_authorized"])
        self.assertIn("total6", result["steady"]["strict_only"]["models"]["legacy87"])

    def test_actual_hdf_motion_expression_parser(self):
        case = self.cfg["cases"][-1]
        blocks = []
        for axis, d in zip("xyz", case["direction"]):
            expression = "%.17g [m s^-1] * cos(%.17g [s^-1] * t) * min(t / %.17g [s], 1)" % (d * .02 * np.pi, np.pi, .5)
            blocks.append('((name . "blind_osc_rate_' + axis + '") (definition . "' + expression + '"))')
        text = " ".join(blocks)
        with mock.patch.object(ev.dp, "read_case_settings", return_value=text), mock.patch.object(ev.runner, "sha256", return_value="a" * 64):
            law = ev.read_actual_motion_law("memory-only.cas.h5", case)
        self.assertEqual(law["angular_frequency_rad_s"], np.pi)
        self.assertEqual(len(law["expressions"]), 3)
        with mock.patch.object(ev.dp, "read_case_settings", return_value=text.replace("3.1415926535897931", "3.2")), self.assertRaises(ev.EvaluationError):
            ev.read_actual_motion_law("memory-only.cas.h5", case)

    def test_existing_legacy_prior_is_never_overwritten(self):
        with mock.patch.object(Path, "exists", return_value=True), mock.patch.object(ev.subprocess, "run", side_effect=AssertionError("must not run")), self.assertRaises(ev.EvaluationError):
            ev.prepare_legacy_prior("not-executed-matlab.exe")

    def test_terminal_case_data_pair_state_mismatch_rejected(self):
        attempt, case = self.create_attempt()
        report = self.fs.json(attempt / "result.json")
        report["artifacts"]["final_data"]["state_id"] = "not-the-final-case-state"
        self.fs.add_json(attempt / "result.json", report)
        with self.assertRaises(ev.EvaluationError):
            self.collect(attempt, case)


if __name__ == "__main__":
    unittest.main()

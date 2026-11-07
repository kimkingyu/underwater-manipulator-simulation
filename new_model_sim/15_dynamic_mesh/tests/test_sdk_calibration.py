import ast
import math
import sys
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import calibrate_sdk


class CalibrationProbeTests(unittest.TestCase):
    def assess(self, diagnostics, expected=0.001):
        source = (STAGE / "cad/sc_calibration_probe.py.in").read_text(encoding="utf-8")
        tree = ast.parse(source)
        node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "assess_independent_control")
        def finite(value):
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("nonfinite")
            return value
        ns = {"finite": finite, "CFG": {"volume_diagnostics": {"surface_deviations_m": [5e-7, 1e-7, 2e-8]}}}
        exec(compile(ast.Module(body=[node], type_ignores=[]), "probe_function", "exec"), ns)
        return ns["assess_independent_control"]("test", diagnostics, expected)

    def data(self):
        return {"state": "passed", "levels": [
            {"signed_volume_m3": 0.001 - 0.01 * deviation,
             "reference_invariance_error_m3": 1e-14, "surface_deviation_m": deviation}
            for deviation in (5e-7, 1e-7, 2e-8)]}

    def test_known_linear_analytic_convergence_passes_without_hiding_raw_error(self):
        result = self.assess(self.data())
        self.assertEqual(result["state"], "passed")
        self.assertFalse(result["raw_finest_meets_target"])
        self.assertFalse(result["general_CAD_error_bound_claimed"])

    def test_constant_bias_cannot_be_extrapolated_away(self):
        data = self.data()
        for level in data["levels"]:
            level["signed_volume_m3"] += 1e-6
        self.assertEqual(self.assess(data)["state"], "blocked")

    def test_nonrefining_deviation_is_blocked(self):
        data = self.data()
        data["levels"][-1]["surface_deviation_m"] = data["levels"][-2]["surface_deviation_m"]
        self.assertEqual(self.assess(data)["state"], "blocked")

    def test_missing_level_is_not_zero_error(self):
        data = self.data()
        data["levels"].pop()
        self.assertEqual(self.assess(data)["state"], "blocked")

    def test_finest_bias_is_blocked(self):
        data = self.data()
        data["levels"][-1]["signed_volume_m3"] = 0.0011
        self.assertEqual(self.assess(data)["state"], "blocked")

    def test_reference_dependence_is_blocked(self):
        data = self.data()
        data["levels"][-1]["reference_invariance_error_m3"] = 1e-5
        self.assertEqual(self.assess(data)["state"], "blocked")

    def test_nonfinite_is_not_accepted(self):
        data = self.data()
        data["levels"][-1]["signed_volume_m3"] = float("nan")
        with self.assertRaises(ValueError):
            self.assess(data)

    def test_budget_rejects_third_or_boolean_attempt(self):
        for index in (0, 3, True, False):
            with self.assertRaises(ValueError):
                calibrate_sdk.generate(index)

    def test_user_authorization_is_exactly_bounded(self):
        authorization = calibrate_sdk.authorization()
        self.assertEqual(authorization["maximum_small_diagnostic_attempts"], 2)
        self.assertEqual(authorization["maximum_seconds_per_small_diagnostic"], 300)
        self.assertFalse(authorization["flow_solves_authorized_by_this_extension"])

    def test_synthetic_mode_has_no_robot_read_or_flow_call(self):
        source = (STAGE / "cad/sc_calibration_probe.py.in").read_text(encoding="utf-8")
        for token in ("DocumentOpen", "load_source", "DocumentSave", "dual_time", "fluent", "subprocess"):
            self.assertNotIn(token, source)


if __name__ == "__main__":
    unittest.main()

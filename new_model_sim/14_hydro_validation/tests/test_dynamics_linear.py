"""Bounded source, physics, scope and MATLAB algebra tests for dynamic q0."""
import copy
import json
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import dynamics_linear as dl
from data_pipeline import body_jacobians


class DynamicLinearTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads((ROOT / "data/component_dataset.json").read_text(encoding="utf-8"))
        cls.model = json.loads((ROOT / "models/dynamic_linear_q0.json").read_text(encoding="utf-8"))
        cls.report = json.loads((ROOT / "reports/dynamic_development.json").read_text(encoding="utf-8"))
        cls.options = {"period_s": 2., "ramp_s": .5, "motion_kind": "whole_domain_oscillation"}

    def predict(self, nu=None, nd=None, **kwargs):
        options = dict(self.options)
        options.update(kwargs)
        return dl.predict_dynamic_linear(self.model, np.zeros(3), np.zeros(6) if nu is None else nu,
                                         np.zeros(6) if nd is None else nd, **options)

    def test_source_dimensions_actual_frequency_and_order(self):
        checks = dl.validate_source(self.data)
        self.assertEqual(len(checks), 6)
        for c, record in zip(checks, self.data["oscillations"]):
            self.assertEqual((c["n"], c["identification_n"]), (240, 160))
            self.assertEqual(c["actual_angular_frequency_rad_s"], 3.1415927)
            self.assertEqual(c["regression_orders"], [0, 1, 2])
            self.assertLess(c["trajectory_max_abs_error"], 1e-10)
            self.assertLess(c["component_closure_max_abs"], 1e-10)
            self.assertEqual(dl.cycle_mask(record, "cycle2").sum(), 80)
            self.assertEqual(dl.cycle_mask(record, "cycle3").sum(), 80)
        self.assertEqual(self.model["training"]["fit_sample_count"], 960)
        self.assertEqual(self.model["training"]["steady_samples_used"], 0)
        self.assertEqual(self.model["training"]["nuisance_orders"], [0, 2])
        self.assertEqual(self.model["training"]["dataset_sha256"], dl.sha256_file(ROOT / "data/component_dataset.json"))

    def test_2d_export_and_parameter_names_units(self):
        A, D, J = dl.validate_model(self.model)
        self.assertEqual(A.shape, (30, 6))
        self.assertEqual(D.shape, (30, 6))
        self.assertEqual(J.shape, (30, 9))
        metadata = self.model["parameter_metadata"]
        self.assertEqual(len(metadata), 360)
        self.assertEqual(len(set(p["name"] for p in metadata)), 360)
        self.assertTrue(all("/(" in p["unit"] for p in metadata))
        expected_J = body_jacobians(np.zeros(3), self.data["reference"]).reshape(30, 9)
        np.testing.assert_allclose(J, expected_J, rtol=0, atol=1e-13)

    def test_mass_and_damping_scaled_psd(self):
        M, D = np.asarray(self.model["M_total"]), np.asarray(self.model["D_total"])
        T = np.diag(self.model["scaling"]["T_diagonal"])
        np.testing.assert_allclose(M, M.T, rtol=0, atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(T @ M @ T).min(), 0.)
        self.assertGreater(np.linalg.eigvalsh(T @ (D + D.T) @ T / 2).min(), 0.)
        raw_D = np.asarray(self.model["constraint_diagnostics"]["raw_D_total"])
        np.testing.assert_allclose(T @ (D - D.T) @ T, T @ (raw_D - raw_D.T) @ T, rtol=0, atol=1e-12)
        # Important observed couplings are not discarded as an unavoidable data limit.
        self.assertGreater(M[1, 2], 10.)
        self.assertLess(M[0, 5], -10.)
        for key in ("mass_projection", "damping_projection"):
            self.assertEqual(self.model["constraint_diagnostics"][key]["negative_eigenvalues_clipped"], 0)

    def test_psd_projection_on_indefinite_synthetic_matrix(self):
        T = np.diag([1., 1., 1., 2., 2., 2.])
        matrix = np.diag([-2., 1., 3., 4., 5., 6.])
        matrix[0, 1], matrix[1, 0] = 3., -3.
        projected, details = dl.scaled_projection(matrix, T, keep_skew=True)
        self.assertEqual(details["negative_eigenvalues_clipped"], 1)
        self.assertGreaterEqual(np.linalg.eigvalsh(T @ (projected + projected.T) @ T / 2).min(), -1e-12)
        np.testing.assert_allclose(projected - projected.T, matrix - matrix.T, atol=1e-12)

    def test_weighted_component_projection_and_kkt(self):
        J = np.asarray(self.model["J_rows"])
        B = J[:, :6].T
        sigma = np.asarray(self.model["scaling"]["allocation_sigma_physical_rows"])
        for kind, name in (("mass", "M"), ("damping", "D")):
            X = np.asarray(self.model[name + "_rows"])
            target = np.asarray(self.model[name + "_total"])
            np.testing.assert_allclose(B @ X, target, rtol=0, atol=1e-11)
            delta = np.asarray(self.model["constraint_diagnostics"][kind + "_component_allocation"]["delta_rows"])
            gradient = delta / sigma[:, None] ** 2
            multiplier = np.linalg.lstsq(B.T, gradient, rcond=None)[0]
            np.testing.assert_allclose(B.T @ multiplier, gradient, rtol=1e-10, atol=1e-10)
        # Constraints may change multiple components, never an ad-hoc tau correction.
        delta = np.asarray(self.model["constraint_diagnostics"]["mass_component_allocation"]["delta_rows"]).reshape(5, 6, 6)
        self.assertTrue(all(np.linalg.norm(part) > 0 for part in delta))

    def test_coriolis_total_coordinate_closure_and_zero_power(self):
        rng = np.random.default_rng(91024)
        J = np.asarray(self.model["J_rows"]).reshape(5, 6, 9)
        M, D = np.asarray(self.model["M_total"]), np.asarray(self.model["D_total"])
        for _ in range(100):
            nu = rng.normal(size=6) * .03
            nd = rng.normal(size=6) * .05
            p = self.predict(nu, nd, allow_unvalidated=True)
            c = dl.inertial_cross(nu, M @ nu)
            np.testing.assert_allclose(p["inertial_cross_generalized"][:6], c, rtol=0, atol=1e-12)
            np.testing.assert_allclose(p["total"], -M @ nd - c - D @ nu, rtol=0, atol=1e-11)
            self.assertLess(abs(nu @ c), 1e-12)
            V = np.einsum("bij,j->bi", J[:, :, :6], nu)
            self.assertLess(np.max(abs(np.sum(V * p["inertial_cross_wrenches"], axis=1))), 1e-12)
            # This arbitrary virtual variation checks J duality, not joint excitation.
            z = rng.normal(size=9)
            W = np.vstack((p["hull"], p["base0"], p["links"]))
            self.assertAlmostEqual(float(np.sum(np.einsum("bij,j->bi", J, z) * W)), float(z @ p["generalized"]), places=10)

    def test_dissipation_energy_balance_and_zero(self):
        rng = np.random.default_rng(14014)
        M = np.asarray(self.model["M_total"])
        for _ in range(100):
            nu, nd = rng.normal(size=6) * .02, rng.normal(size=6) * .03
            p = self.predict(nu, nd, allow_unvalidated=True)
            self.assertGreaterEqual(p["dissipation_power_W"], -1e-12)
            self.assertAlmostEqual(float(nu @ p["total"] + nu @ M @ nd), -p["dissipation_power_W"], places=11)
        zero = self.predict()
        np.testing.assert_array_equal(zero["generalized"], np.zeros(9))
        self.assertEqual(zero["dissipation_power_W"], 0.)

    def test_scope_rejects_pose_joint_and_silent_motion_extrapolation(self):
        v = np.array([.01, 0., 0., 0., 0., 0.])
        for options in ({"qdot": [.01, 0, 0]}, {"qddot": [.01, 0, 0]}, {"period_s": 1.},
                        {"ramp_s": .7}, {"motion_kind": "steady_SST"}):
            with self.assertRaises(ValueError):
                self.predict(v, **options)
        with self.assertRaises(ValueError):
            dl.predict_dynamic_linear(self.model, [.01, 0., 0.], v, np.zeros(6), allow_unvalidated=True)
        with self.assertRaises(ValueError):
            dl.predict_dynamic_linear(self.model, np.zeros(3), v, np.zeros(6))
        with self.assertRaises(ValueError):
            self.predict(v * 100)
        with self.assertRaises(ValueError):
            self.predict(np.array([.01, .01, 0., 0., 0., 0.]))
        with self.assertRaises(ValueError):
            self.predict(np.full(6, np.nan))
        p = self.predict(v * 100, allow_unvalidated=True)
        self.assertTrue(p["scope_flags"]["extrapolation"])
        self.assertIn("outside_training_velocity_envelope", p["scope_flags"]["unvalidated_reasons"])
        self.assertFalse(p["scope_flags"]["blind_validated"])
        self.assertFalse(p["scope_flags"]["replaces_steady_SST_drag"])

    def test_repeat_period_check_and_report_units_not_blind_validation(self):
        repeat = json.loads((ROOT / "models/dynamic_repeat_cycle2_q0.json").read_text())
        self.assertEqual(repeat["training"]["fit_sample_count"], 480)
        self.assertEqual(repeat["training"]["fit_window"], "cycle2")
        self.assertFalse(repeat["scope"]["independent_case_validated"])
        self.assertFalse(repeat["scope"]["blind_validated"])
        rows = self.report["per_case_channel_SI_metrics"]
        for r in rows:
            self.assertIn(r["unit"], ("N", "N*m"))
            self.assertIn(r["n"], (80, 160, 240))
            if r["model"] == "original87":
                self.assertNotIn(r["body_or_projection"], ("hull", "base0"))
        self.assertEqual({r["harmonic_order"] for r in self.report["harmonic_SI_metrics"]}, {1, 2})
        self.assertTrue(self.report["baseline_available"])
        self.assertFalse(self.report["baseline_separate_hull_base0_available"])

    def test_startup_and_full_trace_errors_are_not_hidden(self):
        rows = self.report["aggregate_full_timeseries_SI_metrics"]
        full = [r for r in rows if r["window"] == "full_240_including_startup"]
        startup = [r for r in rows if r["window"] == "startup_first80_unfitted"]
        self.assertEqual({r["n"] for r in full}, {1440})
        self.assertEqual({r["n"] for r in startup}, {480})
        new_fx = next(r for r in startup if r["model"] == "constrained_dynamic" and r["body_or_projection"] == "total" and r["channel"] == "Fx")
        self.assertGreater(new_fx["RMSE"], 1.)
        self.assertFalse(self.model["training"]["startup_transient_validated"])
        self.assertFalse(self.predict()["scope_flags"]["fluid_memory_model_included"])

    def test_predeclared_oblique_interface_without_blind_labels(self):
        forecast = dl.predict_oblique_yz(self.model)
        self.assertEqual(len(forecast["time_s"]), 240)
        np.testing.assert_allclose(forecast["direction"], np.array([0., 1., 1.]) / np.sqrt(2.))
        self.assertEqual(forecast["role"], "a_priori_nominal_design_prediction_no_labels")
        for p in forecast["predictions"]:
            self.assertTrue(p["scope_flags"]["extrapolation"])
            self.assertFalse(p["scope_flags"]["blind_validated"])
            self.assertIn("combined_axis_motion_not_independently_validated", p["scope_flags"]["unvalidated_reasons"])
        source = (ROOT / "dynamics_linear.py").read_text(encoding="utf-8")
        self.assertNotIn("cfd/runs", source)

    def test_matlab_algebra_cross_language_1e10(self):
        r = json.loads((ROOT / "reports/dynamic_matlab_tests.json").read_text(encoding="utf-8"))
        self.assertTrue(r["passed"])
        self.assertEqual(r["dataset_sha256"], self.model["training"]["dataset_sha256"])
        self.assertEqual(r["model_sha256"], dl.sha256_file(ROOT / "models/dynamic_linear_q0.json"))
        self.assertEqual(self.model["training"]["source_fingerprints"], dl.source_fingerprints())
        self.assertEqual(r["physical_time_samples"], 1440)
        self.assertEqual(r["random_fixture_samples"], 48)
        self.assertLess(r["max_abs_prediction_difference"], 1e-10)
        self.assertLess(r["max_abs_original_J_difference"], 1e-12)
        self.assertGreaterEqual(r["scope_rejections"], 8)
        self.assertFalse(r["simulation_or_solver_launched"])
        self.assertTrue(r["original_theta_hashes_unchanged"])

    def test_original87_theta_hashes_unchanged(self):
        self.assertEqual(dl.theta_hash_audit(), dl.KNOWN_THETA_HASHES)
        self.assertEqual(self.model["training"]["baseline_theta_hashes"], dl.KNOWN_THETA_HASHES)

    def test_in_memory_refit_does_not_use_steady_labels(self):
        changed = copy.deepcopy(self.data)
        changed["steady"] = []
        refit, _ = dl.fit_model(changed, dataset_sha256=self.model["training"]["dataset_sha256"])
        np.testing.assert_allclose(refit["M_rows"], self.model["M_rows"], rtol=0, atol=1e-12)
        np.testing.assert_allclose(refit["D_rows"], self.model["D_rows"], rtol=0, atol=1e-12)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(DynamicLinearTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    dl.save_json(ROOT / "reports/dynamic_python_tests.json", {
        "passed": result.wasSuccessful(), "tests_run": result.testsRun,
        "failures": len(result.failures), "errors": len(result.errors),
        "dataset_sha256": dl.sha256_file(ROOT / "data/component_dataset.json"),
        "model_sha256": dl.sha256_file(ROOT / "models/dynamic_linear_q0.json"),
        "test_source_sha256": dl.sha256_file(Path(__file__)),
        "python_version": sys.version, "original_theta_hashes": dl.theta_hash_audit(),
        "scope": "Dynamic source/constraints/algebra/recorded MATLAB prediction checks only; no CFD or control simulation"})
    sys.exit(0 if result.wasSuccessful() else 1)

"""Stage-14 raw-data and mechanics audit. Run Python -B -m unittest discover ... ."""
import copy
import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.integrate import quad

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("stage14_data_pipeline", ROOT / "data_pipeline.py")
dp = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dp)


class DataPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dataset = dp.load_json(ROOT / "data/component_dataset.json")
        cls.ref = cls.dataset["reference"]
        cls.contract = dp.load_json(ROOT / "contract.json")
        cls.byid = {row["id"]: row for row in cls.dataset["steady"]}

    def assertClose(self, left, right, atol=1e-10):
        np.testing.assert_allclose(left, right, rtol=0., atol=atol)

    def zero_raw(self):
        loads = {f"{kind}{axis}({wall})": 0. for group in self.ref["body_walls"] for wall in group
                 for kind in ("f", "m") for axis in "xyz"}
        loads.update({k: 0. for k in ("fx", "fy", "fz", "mx", "my", "mz", "tau_j1", "tau_j2", "tau_j3")})
        return loads

    def test_contract_shapes_and_strict_capped_counts(self):
        self.assertTrue(set(self.contract["dataset_required_top_keys"]).issubset(self.dataset))
        self.assertEqual(len(self.dataset["steady"]), 40)
        self.assertEqual(sum(row["strict"] for row in self.dataset["steady"]), 37)
        self.assertEqual(sorted(row["id"] for row in self.dataset["steady"] if row["unsteady_mean"]),
                         sorted(self.contract["validation"]["capped_ids"]))
        for row in self.dataset["steady"]:
            self.assertEqual(np.shape(row["wrenches"]), (5, 6))
            self.assertEqual(np.shape(row["uncertainty_wrenches_sd"]), (5, 6))
            self.assertEqual(np.shape(row["total"]), (6,))
            self.assertEqual(np.shape(row["joint_tau"]), (3,))
            self.assertIsInstance(row["strict"], bool)
            self.assertEqual(row["viscous_model"], "SST")
            self.assertIn("NOT independent measurement uncertainty", row["source"]["averaging"]["uncertainty_note"])

    def test_forty_steady_old_label_closure(self):
        for pose, name in dp.DATASET_FILES.items():
            old = dp.load_json(dp.SOURCE / name)
            for case in old["cases"]:
                if case.get("kind") == "oscillation":
                    continue
                row = self.byid[pose + "__" + case["id"]]
                self.assertClose(dp.new_label_vector(row), dp.old_label_vector(case, pose))

    def test_component_total_and_jacobian_power_closure(self):
        rng = np.random.default_rng(14)
        for row in self.dataset["steady"]:
            W = np.asarray(row["wrenches"])
            recomposed = dp.recompose_components(W, row["q"], self.ref)
            expected = np.r_[row["total"], row["joint_tau"]]
            self.assertClose(recomposed["generalized"], expected)
            J = dp.body_jacobians(row["q"], self.ref)
            self.assertClose(np.einsum("bij,bi->j", J, W), expected)
            zeta = rng.normal(size=9)
            self.assertAlmostEqual(np.sum(np.einsum("bij,j->bi", J, zeta) * W), zeta @ expected, places=10)

    def test_reference_shift_invariance(self):
        row = self.byid["qb__T_px_08"]
        history = dp.load_json(dp.WORKSPACE / row["source"]["history"])
        raw = copy.deepcopy(history[-1]["loads"])
        displacement = np.array([.17, -.23, .41])
        shifted = copy.deepcopy(raw)
        for group in self.ref["body_walls"]:
            for wall in group:
                f = np.array([raw[f"f{a}({wall})"] for a in "xyz"])
                m = np.array([raw[f"m{a}({wall})"] for a in "xyz"])
                m -= np.cross(displacement, f)
                for a, val in zip("xyz", m):
                    shifted[f"m{a}({wall})"] = val
        old = dp.loads_to_components(raw, row["q"], self.ref)
        new = dp.loads_to_components(shifted, row["q"], self.ref, raw_moment_origin=displacement)
        self.assertClose(old["wrenches"], new["wrenches"])
        self.assertClose(old["generalized"], new["generalized"])
        wrench = np.array([2., 3., 5., 7., 11., 13.])
        self.assertClose(dp.shift_wrench(dp.shift_wrench(wrench, np.zeros(3), displacement), displacement, np.zeros(3)), wrench)

    def test_rotation_load_sign_and_hull_base0_separation(self):
        raw = self.zero_raw()
        R = dp.rotation([0., 0., 1.], math.pi / 2)
        O = np.asarray(self.ref["origin_Ob_CAD_m"]) + np.array([.2, .1, -.3])
        F_body, M_body = np.array([-2., 3., 4.]), np.array([.3, -.4, .5])
        fg, mg = R @ F_body, R @ M_body + np.cross(O, R @ F_body)
        for axis, force, moment in zip("xyz", fg, mg):
            raw[f"f{axis}(wall_auv)"] = force
            raw[f"m{axis}(wall_auv)"] = moment
        result = dp.loads_to_components(raw, np.zeros(3), self.ref, O, R)
        self.assertClose(result["wrenches"][0], np.r_[F_body, M_body])
        self.assertClose(result["wrenches"][1:], np.zeros((4, 6)))
        self.assertClose(result["total"], np.r_[F_body, M_body])
        self.assertClose(result["joint_tau"], np.zeros(3))
        self.assertLess(self.byid["q0__T_px_08"]["total"][0], 0.)

    def test_q0_qb_joint_recalculation_not_fixed_report(self):
        report_diff = {}
        for pose in ("q0", "qb"):
            row = self.byid[pose + "__T_px_08"]
            history = dp.load_json(dp.WORKSPACE / row["source"]["history"])
            stats = dp.average_window(history, row["q"], self.ref)
            L = stats["mean_raw_loads"]
            fk = dp.forward_kinematics(row["q"], self.ref)
            O = np.asarray(self.ref["origin_Ob_CAD_m"])
            tau = np.zeros(3)
            for j in range(3):
                centre = O + fk["joint_centres"][j]
                axis = fk["joint_axes"][j]
                for group in self.ref["body_walls"][j + 2:]:
                    for wall in group:
                        force = np.array([L[f"f{a}({wall})"] for a in "xyz"])
                        moment = np.array([L[f"m{a}({wall})"] for a in "xyz"])
                        tau[j] += axis @ (moment - np.cross(centre, force))
            self.assertClose(tau, row["joint_tau"])
            report_diff[pose] = np.max(np.abs(tau - np.array([L[f"tau_j{j}"] for j in (1, 2, 3)])))
        self.assertLess(report_diff["q0"], 1e-10)
        self.assertGreater(report_diff["qb"], 1e-3)
        qb = dp.load_json(dp.SOURCE / "theta_dataset_qb8.json")
        fk = dp.forward_kinematics(qb["q"], self.ref)
        self.assertClose(fk["joint_centres"], qb["posed_joint_centre_rel_Ob"])
        self.assertClose(fk["joint_axes"], qb["posed_joint_axis"])
        self.assertClose(fk["body_rotations"][2:], qb["link_R"])

    def test_mean_population_sd_and_tail_indices(self):
        for key in ("q0__T_px_08", "qb__T_px_08"):
            row = self.byid[key]
            history = dp.load_json(dp.WORKSPACE / row["source"]["history"])
            stats = dp.average_window(history, row["q"], self.ref)
            averaging = row["source"]["averaging"]
            subset = [h for h in history if h["iter"] > history[-1]["iter"] - 200]
            samples = np.array([dp.loads_to_components(h["loads"], row["q"], self.ref)["wrenches"] for h in subset])
            self.assertClose(row["wrenches"], samples.mean(axis=0))
            self.assertClose(row["uncertainty_wrenches_sd"], samples.std(axis=0, ddof=0))
            self.assertEqual(averaging["sample_count"], len(subset))
            self.assertEqual(averaging["sample_iterations"], [h["iter"] for h in subset])
            self.assertEqual(stats["averaging"]["sample_indices_zero_based"], averaging["sample_indices_zero_based"])
        counts = [r["source"]["averaging"]["sample_count"] for r in self.dataset["steady"]]
        self.assertEqual({n: counts.count(n) for n in set(counts)}, {4: 4, 8: 36})

    def test_nonfinite_and_invalid_inputs_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            loads = self.zero_raw()
            loads["fx(wall_auv)"] = bad
            with self.assertRaises(ValueError):
                dp.loads_to_components(loads, [0., 0., 0.], self.ref)
            with self.assertRaises(ValueError):
                dp.average_window([{"iter": 25, "loads": loads}, {"iter": 400, "loads": self.zero_raw()}], np.zeros(3), self.ref)
        with self.assertRaises(ValueError):
            dp.loads_to_components(self.zero_raw(), [0., 0.], self.ref)
        with self.assertRaises(ValueError):
            dp.loads_to_components(self.zero_raw(), np.zeros(3), self.ref, rotation_global_from_body=np.diag([-1., 1., 1.]))
        with self.assertRaises(ValueError):
            dp.ramped_cosine_motion([-.01], .1, math.pi, .5)
        with self.assertRaises(ValueError):
            dp.write_json_owned(dp.SOURCE / "forbidden.json", {})
        self.assertFalse((dp.SOURCE / "forbidden.json").exists())

    def test_analytic_motion_integral_derivative_and_ramp_offset(self):
        times = np.array([0., .025, .17, .5, 1.2, 2., 6.])
        V, w, ramp = .062831853, 3.1415927, .5
        x, speed, accel = dp.ramped_cosine_motion(times, V, w, ramp)
        numerical = [quad(lambda s: V * math.cos(w * s) * min(s / ramp, 1.), 0., t,
                          points=[ramp] if t > ramp else None, epsabs=1e-12)[0] for t in times]
        self.assertClose(x, numerical, atol=1e-12)
        self.assertClose(speed, V * np.cos(w * times) * np.minimum(times / ramp, 1.))
        self.assertGreater(abs(x[-1] - V / w * math.sin(w * times[-1])), .01)
        h = 1e-6
        for t in (.17, 1.2):
            xm, vm, _ = dp.ramped_cosine_motion([t - h], V, w, ramp)
            xp, vp, _ = dp.ramped_cosine_motion([t + h], V, w, ramp)
            _, velocity, acceleration = dp.ramped_cosine_motion([t], V, w, ramp)
            self.assertClose((xp - xm) / (2 * h), velocity, atol=1e-9)
            self.assertClose((vp - vm) / (2 * h), acceleration, atol=1e-9)

    def test_six_real_trajectories_and_moving_origin(self):
        self.assertEqual(len(self.dataset["oscillations"]), 6)
        for row in self.dataset["oscillations"]:
            self.assertEqual(len(row["time_s"]), 240)
            self.assertEqual(sum(row["identification_mask"]), 160)
            self.assertEqual(row["viscous_model"], "laminar")
            self.assertEqual(np.shape(row["wrenches"]), (240, 5, 6))
            self.assertEqual(np.shape(row["nu"]), (240, 6))
            law = row["motion_law"]
            x, v, a = dp.ramped_cosine_motion(row["time_s"], law["peak_rate"], law["angular_frequency_rad_s"], law["ramp_s"])
            self.assertClose(x, row["motion_displacement_or_angle"])
            self.assertClose(v, np.asarray(row["nu"])[:, row["dof"]])
            self.assertClose(a, np.asarray(row["nudot"])[:, row["dof"]])
            self.assertTrue(np.max(np.abs(row["nu"])) > 0.)
            self.assertIn("requested_motion", row)
            self.assertIn("actual_motion", row)
            for i in (0, 80, 159, 239):
                result = dp.recompose_components(row["wrenches"][i], row["q"], self.ref)
                self.assertClose(result["total"], row["total"][i])
                self.assertClose(result["joint_tau"], row["joint_tau"][i])
            O = np.asarray(row["origin_global_m"])
            if row["dof"] < 3:
                self.assertClose(O[:, row["dof"]] - self.ref["origin_Ob_CAD_m"][row["dof"]], x)
            else:
                self.assertClose(O, np.tile(self.ref["origin_Ob_CAD_m"], (240, 1)))

    def test_unit_excitation_restoration_and_harmonics(self):
        t = np.arange(81, 241) * .025
        w = math.pi
        unit = np.array([-3., 2., -1.])
        sine = dp.unit_acceleration_to_sine(unit, .02, 2.)
        self.assertClose(dp.sine_to_unit_acceleration(sine, .02, 2.), unit)
        dc, cosine, second_sine, second_cosine = [np.array(v) for v in ([1., 2., 3.], [.4, .2, .1], [.05, .06, .07], [-.01, -.02, -.03])]
        y = (dc + np.sin(w * t[:, None]) * sine + np.cos(w * t[:, None]) * cosine
             + np.sin(2 * w * t[:, None]) * second_sine + np.cos(2 * w * t[:, None]) * second_cosine)
        harmonics = dp.fit_harmonics(t, y, w)
        for key, val in (("dc", dc), ("first_sin", sine), ("first_cos", cosine), ("second_sin", second_sine), ("second_cos", second_cosine)):
            self.assertClose(harmonics[key], val, atol=1e-12)
        self.assertLess(np.max(harmonics["fit_residual_rms"]), 1e-12)

    def test_bbox_uses_original_geometry_references(self):
        source = {b["name"]: b for b in dp.load_json(dp.SOURCE / "body_centroids.json")["bodies"]}
        for i, body in enumerate(self.dataset["bodies"]):
            ref = np.asarray(body["reference_CAD0_m"])
            expected_ref = self.ref["origin_Ob_CAD_m"] if i < 2 else self.ref["link_reference_CAD0_m"][i - 2]
            self.assertClose(ref, expected_ref, atol=1e-15)
            boxes = np.array([source[name]["bbox_m"] for name in body["bbox_source_part_ids"]])
            self.assertClose(np.asarray(body["bbox_local_min_m"]) + ref, boxes[:, :3].min(axis=0))
            self.assertClose(np.asarray(body["bbox_local_max_m"]) + ref, boxes[:, 3:].max(axis=0))
        self.assertEqual(self.dataset["bodies"][0]["walls"], ["wall_auv"])
        self.assertEqual(self.dataset["bodies"][1]["walls"], ["wall_base0"])

    def test_hdf5_full_coverage_units_and_actual_model(self):
        audit = dp.load_json(ROOT / "reports/data_audit.json")
        self.assertEqual(audit["status"], "PASS")
        self.assertEqual(audit["hdf5_settings"]["selected_cases_checked"], 46)
        self.assertTrue(audit["hdf5_settings"]["all_selected_checks_pass"])
        checks = audit["hdf5_settings"]["files"]
        selected = [r for r in checks if r["role"] == "selected_dataset_case"]
        self.assertEqual(sum(r["actual_model_from_case_config"] == "laminar" for r in selected), 6)
        self.assertEqual(sum(r["actual_model_from_case_config"] == "SST" for r in selected), 40)
        old = next(r for r in checks if r["path"].endswith("/b3.cas.h5"))
        self.assertTrue(old["reports"]["mx"]["scaled"])
        self.assertAlmostEqual(old["coefficient_moment_to_Nm_factor"], .6125)
        for row in selected:
            self.assertFalse(row["gravity_enabled"])
            self.assertTrue(all(r["scaled"] is False for r in row["reports"].values()))
        for row in audit["oscillations"]:
            self.assertLess(row["legacy_unit_excitation_check"]["recomputed_old_unit_max_abs"], 1e-10)
            self.assertEqual(row["period_2_3"]["sample_count_each"], 80)
            self.assertEqual(len(row["dc_first_second_harmonics_nominal_frequency"]["dc"]), 39)

    def test_frozen_manifest_scope_and_sample_hashes(self):
        manifest = dp.load_json(ROOT / "frozen/source_manifest.json")
        self.assertEqual(manifest["source_git_commit"], dp.EXPECTED_COMMIT)
        self.assertTrue(manifest["HEAD_matches_approved"])
        files = {f["path"]: f for f in manifest["files"]}
        self.assertTrue(all("runs_qd_interrupted" not in f and not f.endswith(".csv") for f in files))
        for name in ("new_model_sim/11_theta_fit/theta_dataset.json", "new_model_sim/11_theta_fit/matlab/theta_fit.json",
                     "new_model_sim/11_theta_fit/matlab/theta_fit.mat", "new_model_sim/11_theta_fit/matlab/uvms_geometry.m"):
            self.assertEqual(dp.sha256_file(dp.WORKSPACE / name), files[name]["sha256"])
        self.assertEqual(len(dp.load_json(dp.SOURCE / "matlab/theta_fit.json")["theta"]), 87)


if __name__ == "__main__":
    unittest.main()

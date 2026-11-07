"""Offline failure-regression tests; synthetic coordinates are not solver evidence."""

import copy
from pathlib import Path
import sys
import unittest

import numpy as np

STAGE = Path(__file__).resolve().parents[1]
if str(STAGE) not in sys.path:
    sys.path.insert(0, str(STAGE))

import audit
import motion


def z_rotation(angle):
    result = np.eye(4)
    c, s = np.cos(angle), np.sin(angle)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return result


class RigidAuditTests(unittest.TestCase):
    def setUp(self):
        self.reference = np.random.default_rng(1500).uniform(-0.3, 0.3, (64, 3))
        self.target = motion.relative_transform(0.67)
        self.current = motion.transform_points(self.reference, self.target)
        self.contract = motion.load_contract()

    def test_correct_commanded_rigid_motion_passes(self):
        result = audit.audit_rigid_motion(self.reference, self.current, self.target)
        self.assertTrue(result["passed"])
        self.assertTrue(result["shape_passed"])
        self.assertTrue(result["full_pose_passed"])
        self.assertEqual(result["reference_rank"], 3)
        self.assertLess(result["max_rigid_fit_residual_m"], 1e-14)
        self.assertLess(result["max_pairwise_distance_change_m"], 1e-14)
        self.assertLess(result["commanded_orientation_error_rad"], 1e-14)
        np.testing.assert_allclose(result["fitted_transform"], self.target, rtol=0, atol=1e-14)
        self.assertFalse(result["node_ids_verified"])
        self.assertTrue(result["pairwise_exhaustive"])

    def test_id_matching_reorders_current_and_preserves_large_ids(self):
        ids = np.arange(64, dtype=np.int64) + 2 ** 54
        permutation = np.random.default_rng(45).permutation(64)
        ref = {"node_ids": ids, "points": self.reference}
        cur = {"node_ids": ids[permutation], "points": self.current[permutation]}
        result = audit.audit_rigid_motion(ref, cur, self.target)
        self.assertTrue(result["node_ids_verified"])
        self.assertTrue(result["full_pose_passed"])

    def test_id_missing_extra_duplicate_noninteger_mismatch_rejected(self):
        ids = np.arange(64)
        ref = {"node_ids": ids, "points": self.reference}
        bad = [
            {"points": self.current},
            {"node_ids": ids, "coordinates": self.current},
            {"node_ids": ids[:-1], "points": self.current},
            {"node_ids": ids + 1, "points": self.current},
            {"node_ids": np.zeros(64, dtype=int), "points": self.current},
            {"node_ids": ids.astype(float), "points": self.current},
            {"node_ids": ids - 1, "points": self.current},
            {"node_ids": np.full(64, True), "points": self.current},
            {"node_ids": [[1], [2, 3]], "points": self.current},
            self.current,
        ]
        for current in bad:
            with self.subTest(current_type=type(current)), self.assertRaises(audit.GateError):
                audit.audit_rigid_motion(ref, current, self.target)

    def test_local_deformation_fails_even_if_most_nodes_are_correct(self):
        current = self.current.copy()
        current[-1, 1] += 2e-5
        with self.assertRaises(audit.GateError) as caught:
            audit.audit_rigid_motion(self.reference, current, self.target)
        self.assertFalse(caught.exception.details["shape_passed"])

    def test_uniform_scale_and_nonplanar_reflection_are_not_proper_rigid(self):
        for current in (self.reference * 1.0001, self.reference * [1, 1, -1]):
            with self.subTest(current=current[0]), self.assertRaises(audit.GateError):
                audit.audit_rigid_motion(self.reference, current, np.eye(4))

    def test_rigid_wrong_phase_fails_despite_shape_preservation(self):
        wrong = motion.transform_points(self.reference, motion.relative_transform(0.23))
        with self.assertRaises(audit.GateError) as caught:
            audit.audit_rigid_motion(self.reference, wrong, self.target)
        self.assertTrue(caught.exception.details["shape_passed"])
        self.assertIn("phase", str(caught.exception))

    def test_rigid_wrong_translation_fails(self):
        with self.assertRaisesRegex(audit.GateError, "position"):
            audit.audit_rigid_motion(self.reference, self.current + [0.002, 0, 0], self.target)

    def test_orientation_gate_is_independent_of_position_gate(self):
        small = self.reference * 1e-4
        wrong = motion.transform_points(small, z_rotation(0.001))
        self.assertLess(np.max(np.linalg.norm(wrong - small, axis=1)), 1e-4)
        with self.assertRaisesRegex(audit.GateError, "orientation"):
            audit.audit_rigid_motion(small, wrong, np.eye(4))

    def test_background_uses_stricter_fixed_wall_tolerance(self):
        drifted = self.reference + [5e-6, 0, 0]
        self.assertTrue(audit.audit_rigid_motion(self.reference, drifted, np.eye(4))["passed"])
        with self.assertRaisesRegex(audit.GateError, "fixed_wall") as caught:
            audit.audit_fixed_motion(self.reference, drifted)
        self.assertFalse(caught.exception.details["passed"])
        self.assertFalse(caught.exception.details["full_pose_passed"])
        self.assertEqual(caught.exception.details["status"], "failed")
        self.assertTrue(audit.audit_fixed_motion(self.reference, self.reference.copy())["passed"])
        self.assertTrue(audit.audit_fixed_motion(self.reference, self.reference + 2e-8)["passed"])

    def test_cycle_return_uses_its_own_limit_not_fixed_limit(self):
        close = self.reference + [5e-7, 0, 0]
        self.assertTrue(audit.audit_cycle_return(self.reference, close)["passed"])
        with self.assertRaises(audit.GateError):
            audit.audit_fixed_motion(self.reference, close)
        with self.assertRaisesRegex(audit.GateError, "cycle_return"):
            audit.audit_cycle_return(self.reference, self.reference + [2e-6, 0, 0])

    def test_missing_expected_pose_never_becomes_full_pose_pass(self):
        result = audit.audit_rigid_motion(self.reference, self.current)
        self.assertTrue(result["shape_passed"])
        self.assertFalse(result["passed"])
        self.assertFalse(result["full_pose_passed"])
        self.assertIsNone(result["max_commanded_position_error_m"])
        self.assertIsNone(result["commanded_orientation_error_rad"])

    def test_planar_noncollinear_points_fully_observe_proper_rotation(self):
        points = np.array([[0, 0, 0], [0.2, 0, 0], [0.2, 0.2, 0], [0, 0.2, 0]], dtype=float)
        transform = motion.pose_matrix(3, [0.31, -0.37, 0.11])
        current = motion.transform_points(points, transform)
        result = audit.audit_rigid_motion(points, current, transform)
        self.assertEqual(result["reference_rank"], 2)
        self.assertTrue(result["orientation_observable"])
        self.assertTrue(result["full_pose_passed"])
        np.testing.assert_allclose(result["fitted_transform"], transform, rtol=0, atol=1e-14)

    def test_three_noncollinear_points_observe_rotation(self):
        points = np.array([[0, 0, 0], [0.2, 0, 0], [0.1, 0.2, 0]], dtype=float)
        result = audit.audit_rigid_motion(points, motion.transform_points(points, self.target), self.target)
        self.assertTrue(result["passed"])
        self.assertEqual(result["reference_rank"], 2)

    def test_collinear_twist_cannot_be_called_zero_orientation_error(self):
        points = np.column_stack((np.zeros(5), np.zeros(5), np.linspace(-0.2, 0.2, 5)))
        rotated = motion.transform_points(points, z_rotation(0.2))
        result = audit.audit_rigid_motion(points, rotated, np.eye(4))
        self.assertTrue(result["shape_passed"])
        self.assertFalse(result["passed"])
        self.assertFalse(result["full_pose_passed"])
        self.assertFalse(result["orientation_observable"])
        self.assertIsNone(result["commanded_orientation_error_rad"])
        self.assertIsNone(result["fitted_transform"])

    def test_collinear_wrong_position_still_fails(self):
        points = np.column_stack((np.linspace(-0.2, 0.2, 5), np.zeros(5), np.zeros(5)))
        with self.assertRaises(audit.GateError):
            audit.audit_rigid_motion(points, points + [0.002, 0, 0], np.eye(4))

    def test_nearly_collinear_points_are_marked_unobservable(self):
        points = np.column_stack((np.linspace(-0.2, 0.2, 5), np.array([0, 1, -1, 0, 1]) * 1e-14, np.zeros(5)))
        result = audit.audit_rigid_motion(points, points, np.eye(4))
        self.assertEqual(result["reference_rank"], 1)
        self.assertFalse(result["passed"])

    def test_coincident_and_singleton_have_no_fabricated_attitude(self):
        for count in (1, 2, 5):
            points = np.zeros((count, 3))
            result = audit.audit_rigid_motion(points, points, np.eye(4))
            self.assertFalse(result["passed"])
            self.assertIsNone(result["commanded_orientation_error_rad"])
            if count == 1:
                self.assertIsNone(result["max_pairwise_distance_change_m"])
                self.assertFalse(result["shape_passed"])

    def test_nan_infinity_empty_shapes_and_nonreal_data_fail(self):
        bad = [[], np.empty((0, 3)), [[0, 0]], np.zeros(3), np.zeros((2, 3, 1)),
               np.full((64, 3), np.nan), np.full((64, 3), np.inf),
               np.full((64, 3), -np.inf), np.full((64, 3), "0"),
               np.zeros((64, 3), dtype=complex), np.zeros((64, 3), dtype=bool), None]
        for value in bad:
            with self.subTest(shape=np.shape(value)), self.assertRaises(audit.GateError):
                audit.audit_rigid_motion(self.reference, value, self.target)
        with self.assertRaises(audit.GateError):
            audit.audit_rigid_motion(self.reference[:-1], self.current, self.target)

    def test_low_precision_coordinates_fail_closed(self):
        with self.assertRaisesRegex(audit.GateError, "low-precision"):
            audit.audit_rigid_motion(self.reference.astype(np.float32), self.current, self.target)

    def test_derived_arithmetic_overflow_fails(self):
        huge = self.reference * 1e308
        with self.assertRaises(audit.GateError):
            audit.audit_rigid_motion(huge, huge, np.eye(4))

    def test_bad_transform_is_not_silently_accepted(self):
        for matrix in (np.eye(3), np.full((4, 4), np.nan), np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1])):
            with self.subTest(matrix=matrix), self.assertRaises(audit.GateError):
                audit.audit_rigid_motion(self.reference, self.current, matrix)

    def test_gate_tolerances_cannot_default_from_missing_or_nan(self):
        for bad in (None, np.nan, -1, True):
            contract = copy.deepcopy(self.contract)
            contract["gates"]["max_rigid_fit_residual_m"] = bad
            with self.subTest(bad=bad), self.assertRaises(audit.GateError):
                audit.audit_rigid_motion(self.reference, self.current, self.target, contract)
        contract = copy.deepcopy(self.contract)
        del contract["gates"]["max_rigid_fit_residual_m"]
        with self.assertRaises(audit.GateError):
            audit.audit_rigid_motion(self.reference, self.current, self.target, contract)

    def test_pairwise_sampling_is_bounded_and_deterministic(self):
        points = np.random.default_rng(15).normal(size=(50000, 3))
        transformed = motion.transform_points(points, self.target)
        first = audit.audit_rigid_motion(points, transformed, self.target)
        second = audit.audit_rigid_motion(points, transformed, self.target)
        self.assertTrue(first["passed"])
        self.assertFalse(first["pairwise_exhaustive"])
        self.assertLessEqual(first["pairwise_sample_count"], audit.PAIRWISE_SAMPLE_LIMIT)
        self.assertEqual(first["pairwise_total_pairs"], 50000 * 49999 // 2)
        self.assertEqual(first, second)

    def test_inputs_are_not_modified(self):
        reference, current = self.reference.copy(), self.current.copy()
        audit.audit_rigid_motion(reference, current, self.target)
        np.testing.assert_array_equal(reference, self.reference)
        np.testing.assert_array_equal(current, self.current)


class SharingTests(unittest.TestCase):
    def test_shared_nodes_of_one_locked_rigid_arm_are_allowed(self):
        result = audit.shared_motion_conflicts({"l1": [1, 2, 2, 3], "l2": [3, 4], "jaw": [4, 5]},
                                              {"l1": "locked_arm", "l2": "locked_arm", "jaw": "locked_arm"})
        self.assertTrue(result["passed"])
        self.assertEqual(result["same_group_shared_node_count"], 2)
        self.assertEqual(result["conflicts"], [])

    def test_cross_group_shared_nodes_are_conflicts(self):
        result = audit.shared_motion_conflicts({"fixed": [1, 2], "arm": [2, 3], "wall": [3, 4]},
                                              {"fixed": "fixed", "arm": "arm", "wall": "arm"})
        self.assertFalse(result["passed"])
        self.assertEqual(result["cross_group_shared_node_count"], 1)
        self.assertEqual(result["conflicts"][0]["node_id"], 2)
        self.assertEqual(result["same_group_shared_node_count"], 1)

    def test_same_node_in_three_groups_is_one_conflict(self):
        result = audit.shared_motion_conflicts({1: [9, 10], 2: [9, 11], 3: [9, 12]}, {1: 0, 2: 1, 3: 1})
        self.assertEqual(result["cross_group_shared_node_count"], 1)
        self.assertEqual(result["conflicts"][0]["zones"], [1, 2, 3])

    def test_missing_unknown_empty_invalid_zone_evidence_fails(self):
        cases = [({}, {}), ({"a": [1]}, {}), ({"a": [1]}, {"a": 1, "unknown": 2}),
                 ({"a": []}, {"a": 1}), ({"a": [1.0]}, {"a": 1}),
                 ({"a": [-1]}, {"a": 1}), ({"a": [1]}, {"a": None}),
                 ({"a": [1]}, {"a": True}), ({"": [1]}, {"": 1})]
        for nodes, groups in cases:
            with self.subTest(nodes=nodes, groups=groups), self.assertRaises(audit.GateError):
                audit.shared_motion_conflicts(nodes, groups)


class SolverTextTests(unittest.TestCase):
    def test_historical_orphan_phrases_are_detected(self):
        for text, count in (("9864 overset orphan cells", 9864), ("2067 orphan cells", 2067),
                            ("Warning: found 9864 overset orphan cells.", 9864),
                            ("overset orphan cells: 9,864", 9864), ("Orphan-cell count = 3", 3),
                            ("orphans: 2", 2), ("Number of overset orphan cells = 7", 7)):
            with self.subTest(text=text):
                parsed = audit.parse_solver_text(text)
                self.assertEqual(parsed["orphan_counts"], [count])
                self.assertEqual(parsed["orphan_count"], count)
                self.assertTrue(parsed["fatal_errors"])

    def test_explicit_zero_is_not_missing(self):
        parsed = audit.parse_solver_text("Number of overset orphan cells = 0\n0 negative volume cells\n"
                                         "Minimum Orthogonal Quality = 0.015\nMaximum Skewness = 0.91")
        self.assertEqual(parsed["orphan_counts"], [0])
        self.assertEqual(parsed["orphan_count"], 0)
        self.assertEqual(parsed["negative_volume_counts"], [0])
        self.assertEqual(parsed["minimum_orthogonal_quality"], 0.015)
        self.assertEqual(parsed["maximum_skewness"], 0.91)
        self.assertEqual(parsed["fatal_errors"], [])
        self.assertEqual(audit.require_zero_orphans(parsed["orphan_count"], "native transcript"), 0)

    def test_missing_numeric_evidence_never_defaults_to_zero(self):
        for text in ("", "mesh update complete", "no orphan cells", "no negative volumes",
                     "orphan cells unavailable", "Minimum Orthogonal Quality = unavailable"):
            with self.subTest(text=text):
                parsed = audit.parse_solver_text(text)
                self.assertEqual(parsed["orphan_counts"], [])
                self.assertIsNone(parsed["orphan_count"])
                self.assertIsNone(parsed["negative_volume_count"])
                self.assertIsNone(parsed["minimum_orthogonal_quality"])
                with self.assertRaises(audit.GateError):
                    audit.require_zero_orphans(parsed["orphan_count"], "transcript")

    def test_last_zero_cannot_hide_earlier_failure(self):
        parsed = audit.parse_solver_text("0 orphan cells\n9864 overset orphan cells\n0 orphan cells")
        self.assertEqual(parsed["orphan_counts"], [0, 9864, 0])
        self.assertEqual(parsed["orphan_count"], 9864)
        self.assertEqual(parsed["last_orphan_count"], 0)
        self.assertTrue(parsed["fatal_errors"])

    def test_fatal_solver_and_udf_failures(self):
        examples = ["Update_Time_Level: invalid data", "Calculation interrupted.",
                    "Error: invalid command", "Error Object: #f", "floating point exception",
                    "segmentation violation", "process aborted", "UDF not bound",
                    "missing motion UDF", "No motion UDF is bound to this zone",
                    "incorrect cg motion UDF stage15 on zone 4", "failed to load libudf",
                    "No UDF has been hooked", "zone 12 has no motion UDF", "UDF is not selected"]
        for text in examples:
            with self.subTest(text=text):
                self.assertTrue(audit.parse_solver_text(text)["fatal_errors"])

    def test_benign_error_mentions_not_fatal(self):
        for text in ("0 errors, 0 warnings", "residual error tolerance = 1e-6", "Mesh update complete", "No UDF errors"):
            with self.subTest(text=text):
                self.assertEqual(audit.parse_solver_text(text)["fatal_errors"], [])

    def test_negative_volume_counts_and_missing_count_failure(self):
        for text, count in (("2 negative volume cells", 2), ("negative cell volumes = 12", 12),
                            ("Number of negative volume cells: 0", 0),
                            ("3 cells with negative volumes", 3), ("0 negative-volume cells", 0)):
            with self.subTest(text=text):
                parsed = audit.parse_solver_text(text)
                self.assertEqual(parsed["negative_volume_counts"], [count])
                self.assertEqual(bool(parsed["fatal_errors"]), count > 0)
        parsed = audit.parse_solver_text("Warning: negative cell volume detected")
        self.assertTrue(parsed["fatal_errors"])
        self.assertIsNone(parsed["negative_volume_count"])

    def test_quality_uses_worst_of_multiple_observations(self):
        parsed = audit.parse_solver_text("Minimum Orthogonal Quality = 1.25e-2 cell 7 on zone 9\n"
                                         "Minimum Orthogonal Quality = 9.5D-3\n"
                                         "Maximum Skewness = .8\nMaximum Skewness = 0.95")
        self.assertEqual(parsed["minimum_orthogonal_quality_values"], [0.0125, 0.0095])
        self.assertEqual(parsed["minimum_orthogonal_quality"], 0.0095)
        self.assertEqual(parsed["maximum_skewness"], 0.95)

    def test_nonfinite_fractional_negative_count_evidence_fails(self):
        for text in ("orphan cells = nan", "orphan cells = -1", "-1 overset orphan cells",
                     "orphan cells = 0.0", "negative volume cells = inf",
                     "Minimum Orthogonal Quality = NaN", "Minimum Orthogonal Quality = -0.02",
                     "Maximum Skewness = 1.1"):
            with self.subTest(text=text):
                parsed = audit.parse_solver_text(text)
                self.assertTrue(parsed["fatal_errors"])
                self.assertTrue(parsed["invalid_metrics"])

    def test_zero_orphans_requires_integer_and_source(self):
        for count in (None, False, True, -1, 1, 9864, 0.0, "0", np.nan):
            with self.subTest(count=count), self.assertRaises(audit.GateError):
                audit.require_zero_orphans(count, "native counter")
        for source in (None, "", " "):
            with self.subTest(source=source), self.assertRaises(audit.GateError):
                audit.require_zero_orphans(0, source)
        self.assertEqual(audit.require_zero_orphans(np.int64(0), "native overset report"), 0)

    def test_motion_record_retains_raw_clock_and_reference(self):
        text = ("STAGE15_MOTION time=0.5 dtime=0.005 "
                "DT_CG=(-4e-06,-0.0366,-0.460702) omega=(0,0,-0.07853981633974483)")
        result = audit.parse_solver_text(text + "\n" + text)
        self.assertEqual(len(result["motion_records"]), 2)
        self.assertEqual(result["motion_records"][0]["time"], 0.5)
        self.assertEqual(result["motion_records"][0]["dtime"], 0.005)
        self.assertEqual(result["motion_records"][0]["DT_CG"], [-4e-6, -0.0366, -0.460702])
        self.assertEqual(result["motion_records"][0], result["motion_records"][1])
        self.assertEqual(result["fatal_errors"], [])
        self.assertIn("unverified", result["motion_time_semantics"])

    def test_malformed_motion_evidence_fails(self):
        for text in ("STAGE15_MOTION time=0.5", "STAGE15_MOTION time=nan dtime=0.005 DT_CG=(0,0,0) omega=(0,0,0)",
                     "STAGE15_MOTION time=0 dtime=-0.005 DT_CG=(0,0,0) omega=(0,0,0)",
                     "STAGE15_MOTION time=0 dtime=0.005 DT_CG=(0,0,0,0) omega=(0,0,0)",
                     "STAGE15_MOTION time=0 dtime=0.005 DT_CG=(0,0,0) omega=(0,0,0,0)"):
            with self.subTest(text=text):
                self.assertTrue(audit.parse_solver_text(text)["fatal_errors"])
        with self.assertRaises(audit.GateError):
            audit.parse_solver_text(None)


if __name__ == "__main__":
    unittest.main()

"""Offline arithmetic/contract tests only; no CAD or Fluent is started."""

import copy
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import numpy as np

STAGE = Path(__file__).resolve().parents[1]
if str(STAGE) not in sys.path:
    sys.path.insert(0, str(STAGE))

import motion


def about(rotation, centre):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = centre - rotation @ centre
    return result


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.contract = motion.load_contract()
        self.q0 = np.array([0.225, -0.25, -0.2])
        self.p = np.array([-0.000004, -0.0366, -0.460702])

    def test_contract_fresh_and_explicit_path(self):
        loaded = motion.load_contract(STAGE / "contract.json")
        loaded["motion"]["initial_q_rad"][0] = 3
        self.assertEqual(motion.load_contract()["motion"]["initial_q_rad"][0], 0.225)
        np.testing.assert_array_equal(self.contract["frame"]["joint_axes_CAD0"][0], [0, 0, -1])

    def test_initial_and_extrema(self):
        np.testing.assert_array_equal(motion.joint_state(0)["q_rad"], self.q0)
        expected = {0: 0.225, 0.5: 0.25, 1: 0.275, 1.5: 0.25, 2: 0.225}
        for t, q1 in expected.items():
            with self.subTest(t=t):
                state = motion.joint_state(t)
                self.assertAlmostEqual(state["q_rad"][0], q1, places=14)
                np.testing.assert_array_equal(state["q_rad"][1:], self.q0[1:])
                np.testing.assert_array_equal(state["qd_rad_s"][1:], [0, 0])
                np.testing.assert_array_equal(state["qdd_rad_s2"][1:], [0, 0])
        for t in (0, 1, 2):
            self.assertLess(abs(motion.joint_state(t)["qd_rad_s"][0]), 1e-15)
        self.assertAlmostEqual(motion.joint_state(0.5)["qd_rad_s"][0], 0.025 * math.pi)
        self.assertAlmostEqual(motion.joint_state(0)["qdd_rad_s2"][0], 0.025 * math.pi ** 2)

    def test_time_derivatives(self):
        for t in np.linspace(0, 2, 17):
            with self.subTest(t=t):
                h = 1e-5
                lower, state, upper = [motion.joint_state(s) for s in (t - h, t, t + h)]
                np.testing.assert_allclose((upper["q_rad"] - lower["q_rad"]) / (2 * h),
                                           state["qd_rad_s"], rtol=0, atol=2e-10)
                np.testing.assert_allclose((upper["qd_rad_s"] - lower["qd_rad_s"]) / (2 * h),
                                           state["qdd_rad_s2"], rtol=0, atol=2e-10)

    def test_periodicity_and_restart_order_independence(self):
        for t in (-2.13, 0, 0.125, 0.63, 1.0, 1.999, 5.2):
            first = motion.joint_state(t)
            motion.joint_state(1.731)
            repeat = motion.joint_state(t)
            periodic = motion.joint_state(t + 2)
            for key in first:
                np.testing.assert_array_equal(first[key], repeat[key])
                np.testing.assert_allclose(first[key], periodic[key], rtol=0, atol=1e-14)
            first["q_rad"][0] = 99
            self.assertNotEqual(motion.joint_state(t)["q_rad"][0], 99)

    def test_qe_initial_pose_cancels_for_every_group(self):
        for group in [0, 1, 2, 3] + list(motion.PART_GROUPS):
            with self.subTest(group=group):
                np.testing.assert_allclose(motion.relative_transform(0, group), np.eye(4), rtol=0, atol=5e-16)
                np.testing.assert_allclose(motion.relative_transform(2, group), np.eye(4), rtol=0, atol=5e-16)
        self.assertGreater(np.max(abs(motion.pose_matrix(3, self.q0) - np.eye(4))), 0.1)

    def test_all_locked_groups_have_same_relative_motion(self):
        for t in (0, 0.025, 0.5, 0.77, 1, 1.65, 2):
            target = motion.relative_transform(t, 1)
            for group in (2, 3, "fluid_arm", "wall_l2", "wall_jaw1", "wall_pin4"):
                np.testing.assert_allclose(motion.relative_transform(t, group), target, rtol=0, atol=5e-16)

    def test_relative_transform_is_new_negative_z_axis(self):
        for t in (0.5, 0.87, 1.0, 1.5):
            delta = 0.025 * (1 - math.cos(math.pi * t))
            c, s = math.cos(delta), math.sin(delta)
            rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            expected = about(rotation, self.p)
            actual = motion.relative_transform(t)
            np.testing.assert_allclose(actual, expected, rtol=0, atol=4e-16)
            np.testing.assert_allclose(motion.transform_points(self.p, actual), self.p, rtol=0, atol=3e-16)
            x = self.p + [1, 0, 0]
            self.assertLess(motion.transform_points(x, actual)[1], self.p[1])

    def test_pose_chain_matches_cad_order_not_reversed(self):
        q = np.array([0.17, -0.21, 0.13])
        c1, c2, c3 = np.cos(q)
        s1, s2, s3 = np.sin(q)
        rotations = [np.array([[c1, s1, 0], [-s1, c1, 0], [0, 0, 1]]),
                     np.array([[1, 0, 0], [0, c2, s2], [0, -s2, c2]]),
                     np.array([[1, 0, 0], [0, c3, -s3], [0, s3, c3]])]
        centres = np.array(self.contract["frame"]["joint_centres_CAD0_m"])
        expected = np.eye(4)
        for group in range(1, 4):
            expected = expected @ about(rotations[group - 1], centres[group - 1])
            np.testing.assert_allclose(motion.pose_matrix(group, q), expected, rtol=0, atol=2e-16)
        reverse = about(rotations[2], centres[2]) @ about(rotations[1], centres[1]) @ about(rotations[0], centres[0])
        self.assertGreater(np.max(abs(expected - reverse)), 1e-3)

    def test_fixed_groups_remain_identity(self):
        for group in (0, "hull", "base0", "wall_auv", "wall_base0", "fluid_background"):
            np.testing.assert_array_equal(motion.relative_transform(0.71, group), np.eye(4))

    def test_apply_to_already_posed_points_without_double_pose_or_shift(self):
        cad = np.array([[0.1, 0.2, -0.4], [-0.01, -0.15, -0.35]])
        repaired = cad + np.array(self.contract["source"]["distal_shift_before_pose_m"])
        qe = motion.transform_points(repaired, motion.pose_matrix(3, self.q0))
        moved = motion.transform_points(qe, motion.relative_transform(0.6, 3))
        direct = motion.transform_points(repaired, motion.pose_matrix(3, motion.joint_state(0.6)["q_rad"]))
        np.testing.assert_allclose(moved, direct, rtol=0, atol=3e-16)
        np.testing.assert_allclose(motion.transform_points(qe, motion.relative_transform(0)), qe, rtol=0, atol=3e-16)

    def test_reference_velocity_matches_position_derivative(self):
        reference0 = self.p + [0.12, -0.08, 0.21]
        for t in (0, 0.03, 0.33, 0.5, 0.91, 1.2, 1.9, 2):
            h = 1e-5
            derivative = (motion.j1_reference_position(t + h, reference0)
                          - motion.j1_reference_position(t - h, reference0)) / (2 * h)
            np.testing.assert_allclose(motion.j1_reference_velocity(t, reference0), derivative,
                                       rtol=0, atol=1e-10)
            np.testing.assert_allclose(motion.j1_reference_velocity(t, self.p), [0, 0, 0], rtol=0, atol=1e-17)

    def test_reference_velocity_uses_rotated_position_not_initial_position(self):
        reference0 = self.p + [0.12, -0.08, 0.21]
        t = 0.5
        omega = np.array([0, 0, -motion.joint_state(t)["qd_rad_s"][0]])
        correct = motion.j1_reference_velocity(t, reference0)
        frozen_reference_error = np.cross(omega, reference0 - self.p)
        self.assertGreater(np.linalg.norm(correct - frozen_reference_error), 1e-5)
        np.testing.assert_allclose(correct, np.cross(omega, motion.j1_reference_position(t, reference0) - self.p),
                                   rtol=0, atol=1e-17)

    def test_no_mutation_of_inputs(self):
        points = np.array([[0.3, -0.2, 0.1], [0.1, 0.4, -0.3]])
        old = points.copy()
        q = self.q0.copy()
        original_contract = copy.deepcopy(self.contract)
        motion.transform_points(points, motion.relative_transform(0.3, contract=self.contract))
        motion.pose_matrix(3, q, self.contract)
        motion.joint_state(0.5, self.contract)
        np.testing.assert_array_equal(points, old)
        np.testing.assert_array_equal(q, self.q0)
        self.assertEqual(self.contract, original_contract)

    def test_bad_times_rejected(self):
        for time in (None, True, "0", complex(1, 0), [0], np.nan, np.inf, -np.inf):
            with self.subTest(time=time), self.assertRaises(ValueError):
                motion.joint_state(time)

    def test_unknown_groups_rejected(self):
        for group in (-1, 4, "1", "wall_unknown", None, True, 1.0):
            with self.subTest(group=group), self.assertRaises(ValueError):
                motion.relative_transform(0.1, group)

    def test_bad_joint_vectors_rejected(self):
        for q in ([], [0, 1], [[0, 0, 0]], [0, np.nan, 0], [True, False, True], [0j, 0j, 0j]):
            with self.subTest(q=q), self.assertRaises(ValueError):
                motion.pose_matrix(3, q)

    def test_contract_missing_or_incompatible_values_rejected(self):
        modifications = [("frame", "length_unit", "mm"), ("frame", "angle_unit", "deg"),
                         ("motion", "period_s", 0), ("motion", "period_s", np.nan),
                         ("motion", "half_excursion_rad", -1),
                         ("motion", "active_joint_indices_1based", [1, 2]),
                         ("motion", "active_joint_indices_1based", [True]),
                         ("motion", "active_joint_indices_1based", [np.bool_(True)]),
                         ("motion", "active_joint_indices_1based", [1.0]),
                         ("motion", "reference_point_m", [0, 0, 0]),
                         ("source", "pose_rad", [0, 0, 0]),
                         ("frame", "joint_axes_CAD0", [[0, 1, 0], [-1, 0, 0], [1, 0, 0]])]
        for section, key, value in modifications:
            contract = copy.deepcopy(self.contract)
            contract[section][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                motion.joint_state(0, contract)
        contract = copy.deepcopy(self.contract)
        del contract["frame"]["joint_centres_CAD0_m"]
        with self.assertRaises(ValueError):
            motion.joint_state(0, contract)

    def test_invalid_points_and_transforms_rejected(self):
        for points in ([], [1, 2], [[1, 2]], [[0, 0, np.inf]], [["0", "0", "0"]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                motion.transform_points(points, np.eye(4))
        transforms = [np.eye(3), np.full((4, 4), np.nan)]
        for change in ("reflection", "scale", "row"):
            matrix = np.eye(4)
            if change == "reflection":
                matrix[0, 0] = -1
            elif change == "scale":
                matrix[0, 0] = 1.01
            else:
                matrix[3, 0] = 1
            transforms.append(matrix)
        for matrix in transforms:
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                motion.transform_points([1, 2, 3], matrix)

    def test_udf_frozen_constants_match_contract(self):
        source = (STAGE / "udf" / "stage15_j1_motion.c").read_text(encoding="utf-8")
        constants = {name: float(value) for name, value in re.findall(
            r"^#define\s+(STAGE15_\w+)\s+([-+.0-9eE]+)\s*$", source, re.M)}
        expected = {"STAGE15_Q10": self.q0[0], "STAGE15_Q20": self.q0[1], "STAGE15_Q30": self.q0[2],
                    "STAGE15_AMPLITUDE": self.contract["motion"]["half_excursion_rad"],
                    "STAGE15_PERIOD": self.contract["motion"]["period_s"],
                    "STAGE15_START_TIME": self.contract["motion"]["start_time_s"],
                    "STAGE15_ORIGIN_X": self.p[0], "STAGE15_ORIGIN_Y": self.p[1], "STAGE15_ORIGIN_Z": self.p[2]}
        for name, value in expected.items():
            self.assertEqual(constants[name], value, name)
        self.assertEqual(source.count("STAGE15_MOTION time="), 1)
        self.assertIn("#ifndef STAGE15_STANDALONE", source)
        self.assertIn("omega[2] = -qd[0];", source)
        self.assertIn("stage15_motion_at_time((double)time, velocity_d, omega_d)", source)
        self.assertNotIn('#include "arm_motion.c"', source)
        self.assertNotRegex(source, r"DT_CG\(dt\)\s*\[\s*\w+\s*\]\s*=")


class StandaloneCTests(unittest.TestCase):
    """Optional installed clang-cl arithmetic checks, never a Fluent process."""

    def _compiler(self):
        root = Path(motion.load_contract()["software"]["ansys_root"])
        compiler = root / "fluent" / "ntbin" / "clang" / "bin" / "clang-cl.exe"
        if not compiler.is_file():
            self.skipTest("installed clang-cl unavailable; standalone C not compiled")
        return str(compiler)

    def _checked_compile(self, command, source=None):
        result = subprocess.run(command, input=source, capture_output=True, text=True,
                                errors="replace", timeout=60)
        if result.returncode:
            output = result.stdout + result.stderr
            missing_environment = ("could not find Visual Studio installation",
                                   "'math.h' file not found", "'float.h' file not found",
                                   "'stdio.h' file not found", "'stddef.h' file not found",
                                   'Executable "link.exe" doesn\'t exist',
                                   "cannot open file 'libcmt.lib'", "cannot open file 'oldnames.lib'")
            if any(marker.lower() in output.lower() for marker in missing_environment):
                self.skipTest("standalone C toolchain incomplete: " + output.strip()[-1200:])
            self.fail("standalone C compilation failed: " + output)
        return result

    def test_standalone_source_compiles_without_fluent_headers(self):
        self._checked_compile([self._compiler(), "--driver-mode=gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                               "-DSTAGE15_STANDALONE", "-fsyntax-only",
                               str(STAGE / "udf" / "stage15_j1_motion.c")])

    def test_standalone_c_arithmetic_matches_python_and_rejects_invalid_input(self):
        harness = r'''
#include <math.h>
#include <stddef.h>
#include <stdio.h>
int stage15_joint_state(double, double[3], double[3], double[3]);
int stage15_motion_at_time(double, double[3], double[3]);
int stage15_j1_reference_position_velocity(double, const double[3], double[3], double[3]);
int main(void) {
    const double times[] = {-1.25, 0.0, 0.123, 0.5, 1.0, 1.7, 2.0, 3.4};
    const double reference0[] = {0.119996, -0.1166, -0.250702};
    double q[3], qd[3], qdd[3], velocity[3], omega[3], position[3], ref_velocity[3];
    const double *rows[7];
    int k, j, field;
    rows[0] = q; rows[1] = qd; rows[2] = qdd; rows[3] = velocity;
    rows[4] = omega; rows[5] = position; rows[6] = ref_velocity;
    for (k = 0; k < 8; ++k) {
        if (stage15_joint_state(times[k], q, qd, qdd) != 0 ||
            stage15_motion_at_time(times[k], velocity, omega) != 0 ||
            stage15_j1_reference_position_velocity(times[k], reference0, position, ref_velocity) != 0)
            return 2;
        printf("%.17g", times[k]);
        for (field = 0; field < 7; ++field)
            for (j = 0; j < 3; ++j) printf(" %.17g", rows[field][j]);
        printf("\n");
    }
    printf("INVALID=%d,%d,%d\n", stage15_joint_state(HUGE_VAL, q, qd, qdd),
           stage15_joint_state(0.0, NULL, qd, qdd),
           stage15_j1_reference_position_velocity(0.0, NULL, position, ref_velocity));
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="stage15_motion_c_") as temporary:
            executable = Path(temporary) / "stage15_motion_check.exe"
            self._checked_compile([self._compiler(), "--driver-mode=gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                                   "-DSTAGE15_STANDALONE", str(STAGE / "udf" / "stage15_j1_motion.c"),
                                   "-x", "c", "-", "-o", str(executable)], harness)
            result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr)
            lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 9)
        self.assertEqual(lines[-1], "INVALID=-1,-1,-1")
        reference0 = np.array([0.119996, -0.1166, -0.250702])
        for line in lines[:-1]:
            row = np.fromstring(line, sep=" ")
            self.assertEqual(row.shape, (22,))
            time = row[0]
            state = motion.joint_state(time)
            expected = np.concatenate((state["q_rad"], state["qd_rad_s"], state["qdd_rad_s2"],
                                       np.zeros(3), [0, 0, -state["qd_rad_s"][0]],
                                       motion.j1_reference_position(time, reference0),
                                       motion.j1_reference_velocity(time, reference0)))
            np.testing.assert_allclose(row[1:], expected, rtol=0, atol=1e-14)


if __name__ == "__main__":
    unittest.main()

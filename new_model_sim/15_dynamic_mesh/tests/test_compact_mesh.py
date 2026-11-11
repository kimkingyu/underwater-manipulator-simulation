"""Offline tests for the <=500k compact profile and polyhedral conversion gate; no ANSYS."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import compact_mesh_config as cmc
import mesh_prepare
import poly_convert
import runtime


class CompactProfileTests(unittest.TestCase):
    def setUp(self):
        self.contract = runtime.load_contract()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.background = self.root / "bg.scdoc"
        self.arm = self.root / "arm.scdoc"
        self.background.write_bytes(b"fixture")
        self.arm.write_bytes(b"fixture arm")

    def tearDown(self):
        self.temp.cleanup()

    def test_sphere_covers_arm_box_with_margin_and_is_much_smaller_than_old(self):
        bg, arm = cmc.compact_settings(self.contract, self.root)
        sphere = bg["spheres"][0]
        self.assertEqual(len(bg["spheres"]), 1)
        self.assertLess(sphere["radius_m"], 0.40)
        self.assertGreaterEqual(sphere["size_m"], 0.02)
        self.assertTrue(cmc.validate(bg, arm))

    def test_same_names_and_domains_as_baseline(self):
        bg, arm = cmc.compact_settings(self.contract, self.root)
        old_bg, old_arm = mesh_prepare.domain_settings(self.contract, self.root)
        self.assertEqual(bg["required_selections"], old_bg["required_selections"])
        self.assertEqual(arm["required_selections"], old_arm["required_selections"])
        self.assertEqual([c["name"] for c in bg["face_controls"]], [c["name"] for c in old_bg["face_controls"]])
        self.assertEqual([c["name"] for c in arm["face_controls"]], [c["name"] for c in old_arm["face_controls"]])
        self.assertEqual(bg["cell_zone"], "fluid_background")
        self.assertEqual(arm["cell_zone"], "fluid_arm")

    def test_rough_estimate_is_far_below_old_and_near_target(self):
        bg, arm = cmc.compact_settings(self.contract, self.root)
        new = cmc.rough_tet_estimate(bg, arm)["total_tets"]
        old = cmc.rough_tet_estimate(*mesh_prepare.domain_settings(self.contract, self.root))["total_tets"]
        self.assertLess(new, old / 8)
        # Planning number only: gap spheres sit mostly inside solid parts, so the volume term
        # overestimates. The binding budget is the native post-conversion count recorded in
        # COMPACT_RESULT (compact_001 background + compact_003 arm).
        actual = sum(cmc.COMPACT_RESULT[d]["polyhedral_cells"] for d in ("background", "arm"))
        self.assertLessEqual(actual, cmc.MAX_TOTAL_POLY_CELLS)
        self.assertTrue(all(cmc.COMPACT_RESULT[d]["min_orth"] >= cmc.MIN_ORTHOGONAL_QUALITY
                            for d in ("background", "arm")))

    def test_contact_sizes_match_and_bad_sphere_rejected(self):
        bg, arm = cmc.compact_settings(self.contract, self.root)
        bg["spheres"][0]["radius_m"] = 0.2
        with self.assertRaises(runtime.RuntimeGateError):
            cmc.validate(bg, arm)
        bg, arm = cmc.compact_settings(self.contract, self.root)
        arm["face_controls"][0]["size_m"] = 0.001
        with self.assertRaises(runtime.RuntimeGateError):
            cmc.validate(bg, arm)

    def test_generate_compact_writes_journal_without_launch_and_rejects_overrides(self):
        attempt = self.root / "compact1"
        with patch("subprocess.Popen") as spawn:
            plan = mesh_prepare.generate(attempt, self.background, self.arm, stage_root=self.root, compact=True)
            spawn.assert_not_called()
        self.assertEqual(plan["status"], "not_run")
        self.assertEqual(plan["domains"][0]["profile"], cmc.PROFILE)
        self.assertEqual(len(plan["domains"][1]["spheres"]), len(cmc.ARM_GAP_SPHERES))
        for s in plan["domains"][1]["spheres"]:
            self.assertLessEqual(s["radius_m"], 0.01)
            self.assertGreaterEqual(s["size_m"], cmc.MINIMUM_SIZE_M)
        frozen = json.loads((attempt / "inputs.json").read_text(encoding="utf-8"))
        self.assertTrue(any(s["path"].endswith("compact_mesh_config.py") for s in frozen["sources"]))
        with self.assertRaises(runtime.RuntimeGateError):
            mesh_prepare.generate(self.root / "c2", self.background, self.arm, stage_root=self.root,
                                  compact=True, interface_size=.01)
        with self.assertRaises(runtime.RuntimeGateError):
            mesh_prepare.generate(self.root / "c3", self.background, self.arm, stage_root=self.root,
                                  compact=True, fixed_refinement=True)


class PolyGateTests(unittest.TestCase):
    TEXT = ("Minimum Orthogonal Quality = 2.5e-02 cell 7 on zone 2 (ID: 9 on partition: 0) at location ( 1.0e-01, 2.0e-01, 3.0e-01)\n"
            "Maximum Aspect Ratio = 8.1e+01\n minimum volume (m3): 6.8e-13\n")

    def test_parse_and_pass(self):
        row = poly_convert.parse_quality(self.TEXT)
        self.assertAlmostEqual(row["min_orth"], 0.025)
        self.assertFalse(row["negative_volume"])
        self.assertEqual(poly_convert.verdict({"poly_improved": row, "polyhedral_cells": 300000}), [])

    def test_low_quality_negative_volume_and_missing_count_fail(self):
        low = poly_convert.parse_quality(self.TEXT.replace("2.5e-02", "5.0e-03").replace("6.8e-13", "-1e-12"))
        errors = poly_convert.verdict({"poly_improved": low})
        self.assertEqual(len(errors), 3)
        self.assertEqual(poly_convert.verdict({}), ["missing final minimum orthogonal quality",
                                                    "polyhedral cell count not observed"])

    def test_cell_count_parse(self):
        self.assertEqual(poly_convert.parse_cells("  318726 polyhedra cells, zone  2.\n 12 faces"), 318726)

    def test_skipped_conversion_is_not_counted_as_polyhedra(self):
        # Real compact_001 arm transcript: Fluent skipped conversion, mesh stayed tetrahedral.
        info = "354949 tetrahedral cells, zone  2.\n693948 triangular interior faces, zone  1."
        self.assertEqual(poly_convert.parse_cells(info), 0)
        self.assertEqual(poly_convert.parse_cells(info, ("tetrahedral",)), 354949)
        good = poly_convert.parse_quality(self.TEXT)
        errors = poly_convert.verdict({"poly_improved": good, "polyhedral_cells": 0,
                                       "tetrahedral_cells_remaining": 354949})
        self.assertIn("conversion incomplete: 354949 tetrahedral cells remain", errors)


if __name__ == "__main__":
    unittest.main()

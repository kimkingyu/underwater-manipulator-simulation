import ast
import sys
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import prepare_face_probe


class FaceProbePreparationTests(unittest.TestCase):
    def grant(self):
        return {"scope": "single_face_sampling_probe", "maximum_attempts": 1,
                "maximum_runtime_seconds": 300, "maximum_faces_scanned": 32,
                "maximum_failed_faces_diagnosed": 3, "complete_fluid_domain_builds_allowed": 0,
                "flow_solves_allowed": False, "original_cad_changes_allowed": False}

    def test_bounded_grant_validation_does_not_launch_or_write(self):
        self.assertEqual(prepare_face_probe.validate_authorization(self.grant())["maximum_attempts"], 1)

    def test_corrective_slot_needs_new_explicit_grant(self):
        grant = self.grant()
        self.assertEqual(prepare_face_probe.validate_authorization(grant)["attempt_id"], "attempt_001")
        grant["attempt_id"] = "attempt_002"
        with self.assertRaises(ValueError):
            prepare_face_probe.validate_authorization(grant)
        grant["startup_repair_rerun_explicitly_approved"] = True
        self.assertEqual(prepare_face_probe.validate_authorization(grant)["attempt_id"], "attempt_002")

    def test_third_local_probe_slot_is_not_implicitly_authorized(self):
        grant = self.grant()
        grant.update(attempt_id="attempt_003", startup_repair_rerun_explicitly_approved=True)
        with self.assertRaises(ValueError):
            prepare_face_probe.validate_authorization(grant)

    def test_old_budget_scope_is_not_reused(self):
        grant = self.grant()
        grant["scope"] = "primitive_volume_calibration"
        with self.assertRaises(ValueError):
            prepare_face_probe.validate_authorization(grant)

    def test_larger_runtime_and_scan_are_refused(self):
        for key, value in (("maximum_attempts", 2), ("maximum_runtime_seconds", 301),
                           ("maximum_faces_scanned", 33), ("maximum_failed_faces_diagnosed", 4)):
            grant = self.grant()
            grant[key] = value
            with self.assertRaises(ValueError):
                prepare_face_probe.validate_authorization(grant)

    def test_boolean_numeric_budget_is_refused(self):
        grant = self.grant()
        grant["maximum_attempts"] = True
        with self.assertRaises(ValueError):
            prepare_face_probe.validate_authorization(grant)

    def test_geometry_or_solver_permission_is_not_inferred(self):
        for key in ("flow_solves_allowed", "original_cad_changes_allowed"):
            grant = self.grant()
            grant[key] = True
            with self.assertRaises(ValueError):
                prepare_face_probe.validate_authorization(grant)

    def startup_config(self):
        return {"mode": "face_probe", "script_path": "not_executed.py",
                "source_path": "not_opened.step", "source_sha256": "0" * 64,
                "output_json": "not_written.json", "stage_root": str(STAGE),
                "pose_rad": [0.225, -0.25, -0.2], "joint_centres_m": [[0, 0, 0]] * 3,
                "joint_axes": [[0, 0, -1], [-1, 0, 0], [1, 0, 0]],
                "distal_shift_m": [0, 1e-5, 0]}

    def test_render_is_syntax_valid_and_does_not_execute_source(self):
        source = prepare_face_probe.render(self.startup_config())
        ast.parse(source)
        self.assertIn("def projected_facet_samples", source)
        self.assertIn("whole_CAD_pair_validated", source)
        self.assertIn("reference_face_index_is_a_hint_not_a_persistent_identifier", source)
        result = prepare_face_probe.verify_module_startup(source)
        self.assertFalse(result["cad_runtime_called"])
        self.assertFalse(result["sdk_compatibility_proven"])

    def test_missing_distal_shift_is_caught_before_cad_launch(self):
        config = self.startup_config()
        del config["distal_shift_m"]
        with self.assertRaisesRegex(ValueError, "distal_shift_m"):
            prepare_face_probe.render(config)

    def test_smoke_check_never_calls_runtime_functions(self):
        source = "def mode_main():\n    raise RuntimeError('must not execute')\ndef runtime_main():\n    raise RuntimeError('must not execute')\nruntime_main()\n"
        self.assertEqual(prepare_face_probe.verify_module_startup(source)["status"], "passed")

    def test_smoke_check_rejects_unexpected_top_level_calls(self):
        source = "def mode_main():\n    pass\ndef runtime_main():\n    pass\nprint('side effect')\nruntime_main()\n"
        with self.assertRaises(ValueError):
            prepare_face_probe.verify_module_startup(source)

    def reference_fixture(self):
        def section(label, count, first):
            return {"label": label, "shapes": [{"faces": count, "volume_m3": 1.0}],
                    "surface_fallback": {"a_to_b": {"source_face_count": count,
                        "face_results": [{"face_index": 0, "sample_count": 36, "state": "passed"},
                                         {"face_index": first, "state": "not_run"}]}}}
        return {"domains": {"arm": {
            "boolean_complement_diagnostics": {"geometry_acceptance": section("expected_vs_boolean_reconstructed_material", 617, 1)},
            "imprint_geometry_check": section("exterior_water_before_vs_after_imprint", 603, 7)}}}

    def test_material_and_water_references_cannot_be_conflated(self):
        objects = prepare_face_probe.reference_objects(self.reference_fixture())
        self.assertEqual(objects["arm_material"]["face_count"], 617)
        self.assertEqual(objects["arm_material"]["first_sampling_blocked_face_index"], 1)
        self.assertEqual(objects["arm_exterior_water"]["face_count"], 603)
        self.assertEqual(objects["arm_exterior_water"]["first_sampling_blocked_face_index"], 7)

    def test_wrong_reference_kind_is_rejected(self):
        fixture = self.reference_fixture()
        fixture["domains"]["arm"]["boolean_complement_diagnostics"]["geometry_acceptance"]["label"] = "exterior_water_before_vs_after_imprint"
        with self.assertRaises(ValueError):
            prepare_face_probe.reference_objects(fixture)

    def test_reference_face_census_mismatch_is_rejected(self):
        fixture = self.reference_fixture()
        fixture["domains"]["arm"]["imprint_geometry_check"]["shapes"][0]["faces"] = 617
        with self.assertRaises(ValueError):
            prepare_face_probe.reference_objects(fixture)

    def test_mode_has_no_complete_build_save_or_flow_call(self):
        tree = ast.parse((STAGE / "cad/sc_face_probe.py.in").read_text(encoding="utf-8"))
        names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
        self.assertFalse(names & {"build_domain", "regenerate_domain_reference", "DocumentSave", "run_volume_calibration"})
        self.assertIn("prepare_extended_parts", names)
        self.assertIn("surface_face_samples", names)


if __name__ == "__main__":
    unittest.main()

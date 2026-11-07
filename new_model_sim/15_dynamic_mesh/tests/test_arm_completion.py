import ast
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import complete_arm_cad as arm
import cad_prepare
from preflight import sha256
from prepare_face_probe import verify_module_startup


class ArmCompletionTests(unittest.TestCase):
    def grant(self):
        return {"scope": "arm_domain_completion_then_independent_reopen",
                "maximum_arm_construction_attempts": 1, "maximum_arm_construction_seconds": 1800,
                "maximum_independent_reopen_attempts": 1, "maximum_independent_reopen_seconds": 1800,
                "background_rebuild_allowed": False, "original_cad_changes_allowed": False,
                "acceptance_gate_relaxation_allowed": False, "automatic_retry_allowed": False,
                "independent_reopen_requires_construction_pass": True}

    def evidence(self):
        shape = {"faces": 617, "volume_m3": 0.0012, "surface_area_m2": 0.14,
                 "bbox_m": [[-0.1, -0.2, -0.6], [0.1, 0.0, -0.2]]}
        base = {"domains": {"arm": {"boolean_complement_diagnostics": {"geometry_acceptance": {"shapes": [shape]}}}}}
        probe = {"mode": "face_probe", "state": "blocked", "source_unchanged": True,
                 "input_integrity": {"state": "passed"}, "source_sha256_after": "a" * 64,
                 "material": copy.deepcopy(shape), "targeted_faces": [1, 12, 17],
                 "face_observations": [{"face_index_this_run": i, "state": "passed_for_this_face_sampling_only",
                                        "sampling": {"state": "passed", "final_distinct_count": 65, "fallback_used": True}}
                                       for i in (1, 12, 17)]}
        return base, probe

    def test_exact_two_step_grant(self):
        arm.validate_grant(self.grant())

    def test_corrective_arm_slot_requires_new_explicit_grant(self):
        grant = self.grant()
        grant["arm_build_attempt_id"] = "attempt_002"
        with self.assertRaises(ValueError):
            arm.validate_grant(grant)
        grant["naming_sampler_repair_rerun_explicitly_approved"] = True
        arm.validate_grant(grant)

    def test_third_arm_slot_is_not_implicitly_authorized(self):
        grant = self.grant()
        grant.update(arm_build_attempt_id="attempt_003", naming_sampler_repair_rerun_explicitly_approved=True)
        with self.assertRaises(ValueError):
            arm.validate_grant(grant)

    def test_wider_grant_or_background_rebuild_rejected(self):
        for key, value in (("maximum_arm_construction_attempts", 2), ("maximum_independent_reopen_seconds", 1801),
                           ("background_rebuild_allowed", True), ("automatic_retry_allowed", True)):
            grant = self.grant()
            grant[key] = value
            with self.assertRaises(ValueError):
                arm.validate_grant(grant)

    def test_verify_must_follow_passed_construction(self):
        grant = self.grant()
        grant["independent_reopen_requires_construction_pass"] = False
        with self.assertRaises(ValueError):
            arm.validate_grant(grant)

    def test_actual_local_observations_do_not_upgrade_raw_global_gate(self):
        base, probe = self.evidence()
        result = arm.validate_local_evidence(probe, base, "a" * 64)
        self.assertEqual(result["raw_probe_state_retained"], "blocked")
        self.assertTrue(result["local_observations_only_not_full_geometry_acceptance"])
        self.assertEqual(probe["state"], "blocked")

    def test_water_count_cannot_replace_material_count(self):
        base, probe = self.evidence()
        probe["material"]["faces"] = 603
        with self.assertRaises(ValueError):
            arm.validate_local_evidence(probe, base, "a" * 64)

    def test_changed_fingerprint_rejected(self):
        base, probe = self.evidence()
        probe["material"]["bbox_m"][0][0] += 0.01
        with self.assertRaises(ValueError):
            arm.validate_local_evidence(probe, base, "a" * 64)

    def test_missing_local_target_evidence_rejected(self):
        base, probe = self.evidence()
        probe["face_observations"].pop()
        with self.assertRaises(ValueError):
            arm.validate_local_evidence(probe, base, "a" * 64)

    def test_failed_target_is_not_accepted(self):
        base, probe = self.evidence()
        probe["face_observations"][0]["sampling"]["final_distinct_count"] = 1
        with self.assertRaises(ValueError):
            arm.validate_local_evidence(probe, base, "a" * 64)

    def test_background_branch_and_file_hash_both_required(self):
        with tempfile.TemporaryDirectory(prefix="_test_arm_completion_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "mock_background.scdoc"
            path.write_bytes(b"offline fixture, not real CAD")
            grant = self.grant()
            grant.update(background_artifact=path.relative_to(STAGE).as_posix(), background_sha256=sha256(path))
            base = {"state": "blocked", "source_unchanged": True, "source_sha256_after": "a" * 64,
                    "domains": {"background": {"state": "passed", "scdoc": str(path), "scdoc_sha256": sha256(path)}}}
            result = arm.validate_background(base, "a" * 64, grant)
            self.assertEqual(result["sha256"], sha256(path))
            self.assertEqual(base["state"], "blocked")
            path.write_bytes(b"changed fixture")
            with self.assertRaises(ValueError):
                arm.validate_background(base, "a" * 64, grant)

    def test_template_only_constructs_arm(self):
        source = (STAGE / "cad/sc_arm_completion.py.in").read_text(encoding="utf-8")
        tree = ast.parse(source)
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == "build_domain"]
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0].args[0].value, "arm")
        self.assertIn("background_rebuilt", source)
        self.assertIn("independent_verification_inherited", source)

    def test_real_template_passes_declaration_startup_without_cad(self):
        config = {"mode": "build", "script_path": "not_executed.py", "source_path": "not_opened.step",
                  "source_sha256": "0" * 64, "output_json": "not_written.json", "stage_root": str(STAGE),
                  "pose_rad": [0.225, -0.25, -0.2], "joint_centres_m": [[0, 0, 0]] * 3,
                  "joint_axes": [[0, 0, -1], [-1, 0, 0], [1, 0, 0]], "distal_shift_m": [0, 1e-5, 0]}
        source = cad_prepare.render_template("sc_arm_completion.py.in", config)
        result = verify_module_startup(source)
        self.assertFalse(result["cad_runtime_called"])
        self.assertFalse(result["sdk_compatibility_proven"])


if __name__ == "__main__":
    unittest.main()

"""Offline journal/inventory tests; no Workbench or Meshing execution."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mesh_prepare
import runtime


class MeshPrepareTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.background = self.root / "saved_background.scdoc"
        self.arm = self.root / "saved_arm.scdoc"
        self.background.write_bytes(b"offline fixture background -- not real CAD")
        self.arm.write_bytes(b"offline fixture arm -- not real CAD")
        self.attempt = self.root / "mesh1"

    def tearDown(self):
        self.temp.cleanup()

    def generate(self, **kwargs):
        return mesh_prepare.generate(self.attempt, self.background, self.arm, stage_root=self.root, **kwargs)

    def fixture_exports(self, quality=.02):
        # Explicit fixture data, never presented as actual solver output.
        runtime.write_json_new(self.attempt / "workbench_completion.json", {
            "status": "completed_pending_mesh_inventory", "systems": [{"domain": "background"}, {"domain": "arm"}]})
        paths = []
        for domain in ("background", "arm"):
            data = {"domain": domain, "mesh_generated": True, "status": "generated_pending_export_and_fluent_checks",
                    "nodes": 50, "elements": 100, "quality": {"orthogonal_quality": {"minimum": quality},
                    "skewness": {"maximum": .9, "average": .3}}}
            runtime.write_json_new(self.attempt / (domain + "_meshing_stats.json"), data)
            path = self.attempt / (domain + ".msh")
            path.write_bytes(b"offline test mesh fixture")
            paths.append(path)
        return paths

    def test_generation_never_launches_and_uses_two_explicit_domains(self):
        with patch("subprocess.Popen") as spawn:
            result = self.generate(interface_size=.008, contact_size=.001)
            spawn.assert_not_called()
        self.assertEqual(result["status"], "not_run")
        self.assertFalse(result["valid"])
        self.assertFalse(result["workbench_executed"])
        self.assertFalse(result["motion_authorized"])
        self.assertEqual([x["cell_zone"] for x in result["domains"]], ["fluid_background", "fluid_arm"])
        self.assertEqual(result["domains"][1]["face_controls"][-1]["size_m"], .008)
        self.assertEqual(result["domains"][0]["face_controls"][0]["size_m"], .001)
        script = (self.attempt / "mesh_stage15.wbjn").read_text(encoding="utf-8")
        compile(script, "generated_offline_workbench_journal", "exec")
        self.assertNotIn("__CONFIG_JSON__", script)
        self.assertNotIn("overset_j3", script)
        self.assertIn("AllTriAllTet", script)
        self.assertIn("fluid_background", script)
        self.assertIn("fluid_arm", script)
        self.assertTrue((self.attempt / "inputs.json").is_file())

    def test_distinct_actual_cad_required(self):
        with self.assertRaises(runtime.RuntimeGateError):
            mesh_prepare.generate(self.attempt, self.background, self.background, stage_root=self.root)
        with self.assertRaises(FileNotFoundError):
            mesh_prepare.generate(self.attempt, self.background, self.root / "missing.scdoc", stage_root=self.root)
        self.assertFalse(self.attempt.exists())

    def test_bad_sizes_rejected_before_output(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(runtime.RuntimeGateError):
                self.generate(interface_size=value)
        self.assertFalse(self.attempt.exists())

    def test_existing_attempt_never_overwritten(self):
        self.generate()
        before = runtime.sha256(self.attempt / "mesh_stage15.wbjn")
        with self.assertRaises(FileExistsError):
            self.generate()
        self.assertEqual(runtime.sha256(self.attempt / "mesh_stage15.wbjn"), before)

    def test_polyhedral_is_only_unverified_intent_not_conversion(self):
        result = self.generate(cell_type="polyhedral")
        self.assertEqual(result["generated_cell_type"], "tetrahedral")
        self.assertEqual(result["polyhedral_conversion"], "not_run")
        self.assertEqual(result["overset_cell_type_compatibility"], "unverified")
        self.assertFalse(result["valid"])

    def test_exit_zero_without_artifacts_never_succeeds(self):
        self.generate()
        result = mesh_prepare.collect(self.attempt, self.root / "missing1.msh", self.root / "missing2.msh")
        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["valid"])
        self.assertTrue(result["errors"])

    def test_collect_records_real_paths_counts_and_quality_but_no_static_pass(self):
        self.generate()
        paths = self.fixture_exports()
        result = mesh_prepare.collect(self.attempt, *paths)
        self.assertEqual(result["status"], "mesh_exported_pending_fluent_static_gate")
        self.assertEqual(result["meshes"][0]["nodes"], 50)
        self.assertEqual(result["meshes"][1]["file"]["path"], str(paths[1]))
        self.assertFalse(result["valid"])
        self.assertEqual(result["static_overset"], "not_run")
        self.assertFalse(result["motion_authorized"])

    def test_low_quality_records_failure_no_motion(self):
        self.generate()
        paths = self.fixture_exports(.009)
        result = mesh_prepare.collect(self.attempt, *paths)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any("below 0.01" in x for x in result["errors"]))
        self.assertFalse(result["motion_authorized"])

    def test_missing_quality_records_failure(self):
        self.generate()
        paths = self.fixture_exports(None)
        result = mesh_prepare.collect(self.attempt, *paths)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any("missing finite" in x for x in result["errors"]))

    def test_changed_cad_does_not_pass_collection(self):
        self.generate()
        paths = self.fixture_exports()
        self.arm.write_bytes(b"changed input")
        result = mesh_prepare.collect(self.attempt, *paths)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any("input changed" in x for x in result["errors"]))

    def test_stat_counts_required(self):
        self.generate()
        paths = self.fixture_exports()
        target = self.attempt / "arm_meshing_stats.json"
        stats = json.loads(target.read_text())
        stats["nodes"] = None
        target.write_text(json.dumps(stats))
        result = mesh_prepare.collect(self.attempt, *paths)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any("nodes" in x for x in result["errors"]))


if __name__ == "__main__":
    unittest.main()

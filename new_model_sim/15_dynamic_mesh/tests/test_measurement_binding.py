import json
import sys
import tempfile
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import cad_prepare


class MeasurementBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_test_binding_", dir=str(STAGE / "runs"))
        self.root = Path(self.temp.name)
        self.source_hash = "a" * 64
        self.protocol = {"test_fixture": True, "schema_version": 2, "new_volume_consistency": {
            "maximum_single_relative_accuracy": 0.01, "maximum_combined_relative_uncertainty": 0.01}}
        self.calibration = {"test_fixture": True, "stage": "15_dynamic_mesh", "mode": "calibration",
                            "state": "passed", "small_diagnostic_passed": True, "source_unchanged": True,
                            "source_sha256_before": self.source_hash, "source_sha256_after": self.source_hash}
        self.grant = {"test_fixture": True, "maximum_small_diagnostic_attempts": 2,
                      "maximum_seconds_per_small_diagnostic": 300, "maximum_additional_complete_cad_builds": 1,
                      "additional_complete_cad_build_id": "cad_build/attempt_004",
                      "original_cad_geometry_changes_allowed": False}

    def tearDown(self):
        self.temp.cleanup()

    def inputs(self):
        result = []
        for name, data in (("protocol", self.protocol), ("calibration", self.calibration), ("grant", self.grant)):
            path = self.root / (name + ".json")
            path.write_text(json.dumps(data), encoding="utf-8")
            result.append(path)
        return result

    def config(self):
        return {"mode": "build", "attempt_dir": str(STAGE / "runs/cad_build/attempt_004"),
                "source_sha256": self.source_hash, "frozen_sources": []}

    def test_three_sources_bound_with_real_hashes(self):
        paths = self.inputs()
        config = self.config()
        cad_prepare.bind_measurement_sources(config, *paths)
        self.assertEqual(len(config["frozen_sources"]), 3)
        for key, path in zip(("measurement_protocol", "calibration_reference", "additional_authorization"), paths):
            self.assertEqual(config[key]["sha256"], cad_prepare.sha256(path))

    def test_no_partial_binding(self):
        paths = self.inputs()
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(self.config(), paths[0], paths[1], None)

    def test_blocked_calibration_rejected(self):
        self.calibration["state"] = "blocked"
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(self.config(), *self.inputs())

    def test_source_mismatch_rejected(self):
        self.calibration["source_sha256_after"] = "b" * 64
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(self.config(), *self.inputs())

    def test_extra_attempt_rejected(self):
        config = self.config()
        config["attempt_dir"] = str(STAGE / "runs/cad_build/attempt_005")
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(config, *self.inputs())

    def test_wider_budget_rejected(self):
        self.grant["maximum_additional_complete_cad_builds"] = 2
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(self.config(), *self.inputs())

    def test_weaker_uncertainty_cap_rejected(self):
        self.protocol["new_volume_consistency"]["maximum_combined_relative_uncertainty"] = 0.1
        with self.assertRaises(ValueError):
            cad_prepare.bind_measurement_sources(self.config(), *self.inputs())


if __name__ == "__main__":
    unittest.main()

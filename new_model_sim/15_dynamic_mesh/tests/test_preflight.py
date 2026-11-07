import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import preflight


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_test_preflight_", dir=str(STAGE / "runs"))
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def sample(self):
        source = self.root / "source.txt"
        source.write_bytes(b"untouched\r\n")
        contract_path = self.root / "contract.json"
        contract = {"stage": "15_dynamic_mesh", "schema_version": 1,
                    "default_theta_replacement_allowed": False, "preserve_inputs": ["source.txt"]}
        contract_path.write_text(json.dumps(contract), encoding="utf-8")
        return source, contract_path, contract

    def test_freeze_and_verify_without_changing_source(self):
        source, contract_path, contract = self.sample()
        original = source.read_bytes()
        snapshot = preflight.snapshot_inputs(contract, contract_path, self.root)
        self.assertEqual(len(snapshot["files"]), 2)
        self.assertEqual(preflight.verify_inputs(snapshot, self.root)["status"], "passed")
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual(preflight.sha256(source), hashlib.sha256(original).hexdigest())

    def test_changed_source_rejected(self):
        source, contract_path, contract = self.sample()
        snapshot = preflight.snapshot_inputs(contract, contract_path, self.root)
        source.write_bytes(b"modified")
        self.assertEqual(preflight.verify_inputs(snapshot, self.root)["status"], "failed")

    def test_deleted_source_rejected(self):
        source, contract_path, contract = self.sample()
        snapshot = preflight.snapshot_inputs(contract, contract_path, self.root)
        source.unlink()
        self.assertEqual(preflight.verify_inputs(snapshot, self.root)["status"], "failed")

    def test_empty_snapshot_rejected(self):
        with self.assertRaises(preflight.PreflightError):
            preflight.verify_inputs({"files": []}, self.root)

    def test_duplicate_snapshot_path_rejected(self):
        _, contract_path, contract = self.sample()
        snapshot = preflight.snapshot_inputs(contract, contract_path, self.root)
        snapshot["files"].append(dict(snapshot["files"][0]))
        with self.assertRaises(preflight.PreflightError):
            preflight.verify_inputs(snapshot, self.root)

    def test_missing_source_during_freeze_rejected(self):
        source, contract_path, contract = self.sample()
        source.unlink()
        with self.assertRaises(preflight.PreflightError):
            preflight.snapshot_inputs(contract, contract_path, self.root)

    def test_path_escape_rejected(self):
        with self.assertRaises(preflight.PreflightError):
            preflight.within(self.root / "../outside.json", self.root)

    def test_manifest_input_escape_rejected(self):
        _, contract_path, contract = self.sample()
        contract["preserve_inputs"].append("../outside.txt")
        with self.assertRaises(preflight.PreflightError):
            preflight.snapshot_inputs(contract, contract_path, self.root)

    def test_json_exclusive_write(self):
        destination = self.root / "receipt.json"
        preflight.write_new_json(destination, {"status": "failed"}, self.root)
        with self.assertRaises(FileExistsError):
            preflight.write_new_json(destination, {"status": "passed"}, self.root)
        self.assertEqual(json.loads(destination.read_text())["status"], "failed")

    def test_nonfinite_json_does_not_create_partial_receipt(self):
        destination = self.root / "invalid.json"
        with self.assertRaises(ValueError):
            preflight.write_new_json(destination, {"value": float("nan")}, self.root)
        self.assertFalse(destination.exists())

    def test_no_implicit_output_directories(self):
        with self.assertRaises(preflight.PreflightError):
            preflight.write_new_json(self.root / "missing" / "receipt.json", {}, self.root)

    def test_theta_replacement_disallowed(self):
        _, contract_path, contract = self.sample()
        contract["default_theta_replacement_allowed"] = True
        contract_path.write_text(json.dumps(contract), encoding="utf-8")
        with self.assertRaises(preflight.PreflightError):
            preflight.load_contract(contract_path)

    def test_license_requires_both_server_and_vendor(self):
        result = preflight.parse_license_status(0, "localhost: license server UP (MASTER) v11.19.5\nansyslmd: UP v11.19.5")
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["feature_availability_and_checkout"], "not_verified")
        self.assertEqual(preflight.parse_license_status(0, "license server UP") ["status"], "blocked")
        self.assertEqual(preflight.parse_license_status(0, "ansyslmd: UP") ["status"], "blocked")
        self.assertEqual(preflight.parse_license_status(0, "") ["status"], "blocked")
        self.assertEqual(preflight.parse_license_status(1, "license server UP\nansyslmd: UP") ["status"], "blocked")


if __name__ == "__main__":
    unittest.main()

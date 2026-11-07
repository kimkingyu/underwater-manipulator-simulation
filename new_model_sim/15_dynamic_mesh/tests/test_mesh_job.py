import json
import sys
import tempfile
import unittest
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import run_mesh_job as job
from preflight import sha256


class MeshJobPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_test_mesh_job_", dir=str(STAGE / "runs"))
        self.root = Path(self.temp.name)
        self.background = self.root / "background.scdoc"
        self.arm = self.root / "arm.scdoc"
        self.journal = self.root / "mesh_stage15.wbjn"
        self.source = self.root / "source.py"
        for path, data in ((self.background, b"mock-bg"), (self.arm, b"mock-arm"),
                           (self.journal, b"offline fixture, never execute"), (self.source, b"pass")):
            path.write_bytes(data)
        self.verification = self.root / "verify.json"
        self.cad_data = {"mode": "verify", "state": "passed", "source_unchanged": True,
                         "cad_domains": {"state": "passed", "independent_reopen_completed": True,
                                         "physical_face_ownership_checked": True},
                         "artifacts": {"background": self.record(self.background), "arm": self.record(self.arm)}}
        self.save_verification()

    def tearDown(self):
        self.temp.cleanup()

    def record(self, path):
        return {"path": str(path), "sha256": sha256(path)}

    def save_verification(self):
        self.verification.write_text(json.dumps(self.cad_data), encoding="utf-8")
        self.plan = {"journal": self.record(self.journal), "cad_verification": self.record(self.verification),
                     "cad": [self.record(self.background), self.record(self.arm)],
                     "domains": [{"domain": "background"}, {"domain": "arm"}]}
        (self.root / "mesh_plan.json").write_text(json.dumps(self.plan), encoding="utf-8")
        self.inputs = {"inputs": [self.record(self.background), self.record(self.arm), self.record(self.verification)],
                       "sources": [self.record(self.source)]}
        (self.root / "inputs.json").write_text(json.dumps(self.inputs), encoding="utf-8")

    def test_bound_real_shape_of_acceptance_receipt_passes_offline(self):
        attempt, plan, journal = job.validate_plan(self.root, {})
        self.assertEqual(attempt, self.root)
        self.assertEqual(journal, self.journal)
        self.assertEqual(len(plan["cad"]), 2)

    def test_generated_only_or_blocked_cad_not_accepted(self):
        for state in ("not_run", "blocked", "failed"):
            self.cad_data["state"] = state
            self.save_verification()
            with self.assertRaises(job.RuntimeGateError):
                job.validate_plan(self.root, {})

    def test_missing_independent_reopen_or_ownership_rejected(self):
        for key in ("independent_reopen_completed", "physical_face_ownership_checked"):
            self.cad_data["cad_domains"][key] = False
            self.save_verification()
            with self.assertRaises(job.RuntimeGateError):
                job.validate_plan(self.root, {})
            self.cad_data["cad_domains"][key] = True

    def test_modified_journal_is_rejected(self):
        self.journal.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_changed_cad_is_rejected(self):
        self.arm.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_changed_frozen_source_is_rejected(self):
        self.source.write_bytes(b"changed")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_missing_source_manifest_cannot_be_treated_as_zero(self):
        self.inputs["sources"] = []
        (self.root / "inputs.json").write_text(json.dumps(self.inputs), encoding="utf-8")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_same_cad_twice_is_rejected(self):
        self.plan["cad"][1] = self.record(self.background)
        (self.root / "mesh_plan.json").write_text(json.dumps(self.plan), encoding="utf-8")
        with self.assertRaises(job.RuntimeGateError):
            job.validate_plan(self.root, {})

    def test_meshing_phase_is_cleared_only_by_export_completion(self):
        self.assertIsNone(job.trace_phase("geometry background\n"))
        self.assertEqual(job.trace_phase("generate background\n"), "background")
        self.assertIsNone(job.trace_phase("generate background\nexport completed background\n"))
        self.assertEqual(job.trace_phase("generate background\nexport completed background\ngeometry arm\ngenerate arm\n"), "arm")


if __name__ == "__main__":
    unittest.main()

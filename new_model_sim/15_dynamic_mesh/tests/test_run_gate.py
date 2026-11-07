"""Offline native-text fixtures and fake session tests. Never start Fluent."""
import argparse
import contextlib
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_gate
import runtime


GOOD = """Marked 400 solve cells in register overset-solve-cells-r0.
Marked 20 receptor cells in register overset-receptor-cells-r0.
Marked 30 donor cells in register overset-donor-cells-r0.
Marked 0 orphan cells in register overset-orphan-cells-r0.
Marked 50 dead cells in register overset-dead-cells-r0.
Volume statistics:
 minimum volume (m3): 4.173375e-13
Minimum Orthogonal Quality = 1.2e-02
"""


def roster(contract):
    return {"fluid": ["fff_fluid_background", "fff-1_fluid_arm"], "solid": [],
            "wall": sorted(contract["domains"]["fixed_walls"] + contract["domains"]["moving_walls"] + ["tank_wall"]),
            "overset": ["overset_arm"], "velocity_inlet": ["inlet"], "pressure_outlet": ["outlet"]}


class FakeSolver:
    def __init__(self, contract, *, orphan=0, missing_mark=None, save_file=True, failure=None):
        self.contract, self.orphan, self.missing_mark = contract, orphan, missing_mark
        self.save_file, self.failure = save_file, failure
        self.commands, self.transcript_path = [], None
        self.names = roster(contract)
        self.setup = NS(cell_zone_conditions=NS(fluid=dict.fromkeys(self.names["fluid"]), solid={}),
                        boundary_conditions=NS(**{k: dict.fromkeys(self.names[k]) for k in ("wall", "overset", "velocity_inlet", "pressure_outlet")}))
        self.file = NS(read_mesh=self.read, write=self.write, read_case=self.read)
        self.mesh = NS(modify_zones=NS(append_mesh=self.append, list_zones=self.list_zones))
        self.overset = NS(options=NS(verbosity=lambda *a: self.noop("verbosity", *a), expert=lambda *a: self.noop("expert", *a)),
                          create=lambda *a: self.noop("create", *a), intersect=lambda *a: self.noop("intersect", *a),
                          mark_cells=self.mark, list=self.list_interface, check=lambda: self.noop("check_interface"))
        self.tui = NS(file=NS(start_transcript=self.start, stop_transcript=self.stop),
                      mesh=NS(check=self.check, quality=self.quality), define=NS(overset_interfaces=self.overset))

    def emit(self, line):
        with self.transcript_path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")

    def noop(self, command, *args):
        self.commands.append((command, args))
        if self.failure == command:
            raise RuntimeError("fixture failure at " + command)
        return None

    def start(self, path):
        self.transcript_path = Path(path)
        self.transcript_path.write_text("Offline fake-session fixture, not real evidence\n")

    def stop(self):
        self.noop("stop_transcript")

    def read(self, file_name):
        self.noop("read", file_name)
        self.emit("400 tetrahedral cells, zone 2, binary.")

    def append(self, file_name):
        self.noop("append", file_name)
        self.emit("100 tetrahedral cells, zone 12, binary.")

    def list_zones(self):
        self.noop("list_zones")
        for index, name in enumerate(sum((self.names[k] for k in ("fluid", "wall", "overset", "velocity_inlet", "pressure_outlet")), []), 1):
            self.emit("%d %s fluid cell" % (index, name))

    def mark(self, kind):
        self.noop("mark", kind)
        if kind != self.missing_mark:
            count = {"solve": 400, "receptor": 20, "donor": 30, "orphan": self.orphan, "dead": 50}[kind]
            self.emit("Marked %d %s cells in register overset-%s-cells-r0." % (count, kind, kind))

    def list_interface(self):
        self.noop("list_interface")
        self.emit("Interface Name: ovs_stage15_arm")

    def check(self):
        self.noop("mesh_check")
        self.emit("Volume statistics:\nminimum volume (m3): 4.173375e-13")

    def quality(self):
        self.noop("quality")
        self.emit("Minimum Orthogonal Quality = 0.012")

    def write(self, file_type, file_name):
        self.noop("write_case", file_name)
        self.emit("Writing case (offline fixture)")
        if self.save_file:
            Path(file_name).write_bytes(b"offline fake case -- never real Fluent output")


class EvidenceTests(unittest.TestCase):
    def test_positive_native_evidence_with_declared_derived_negative_zero(self):
        value = run_gate.validate_static_evidence(GOOD)
        self.assertTrue(value["valid"], value["errors"])
        self.assertTrue(value["native"]["negative_volume_derived"]["derived"])
        self.assertEqual(value["native"]["negative_volume_derived"]["value"], 0)
        self.assertEqual(value["native"]["negative_volume_derived"]["minimum_volume_observations"][0]["value"], 4.173375e-13)

    def test_old_missed_overset_orphan_phrase_fails(self):
        value = run_gate.validate_static_evidence(GOOD + "WARNING: 9864 overset orphan cells in interface ovs_stage15_arm.\n")
        self.assertFalse(value["valid"])
        self.assertIn(9864, [x["value"] for x in value["native"]["orphan_observations"]])

    def test_each_required_count_missing_fails(self):
        for kind in run_gate.KINDS:
            text = "\n".join(line for line in GOOD.splitlines() if not (" " + kind + " cells") in line)
            value = run_gate.validate_static_evidence(text)
            self.assertFalse(value["valid"], kind)
            self.assertTrue(any(kind in error for error in value["errors"]))

    def test_missing_quality_or_volume_fails_not_default_zero(self):
        for fragment in ("Minimum Orthogonal", "minimum volume"):
            value = run_gate.validate_static_evidence("\n".join(x for x in GOOD.splitlines() if fragment not in x))
            self.assertFalse(value["valid"])

    def test_no_words_cannot_replace_explicit_zero(self):
        value = run_gate.validate_static_evidence(GOOD.replace("Marked 0 orphan cells", "No orphan cells"))
        self.assertFalse(value["valid"])

    def test_zero_solve_receptor_donor_fails(self):
        for kind, count in (("solve", 400), ("receptor", 20), ("donor", 30)):
            value = run_gate.validate_static_evidence(GOOD.replace("%d %s" % (count, kind), "0 " + kind))
            self.assertFalse(value["valid"])

    def test_quality_threshold_nan_negative_and_fatal_fail(self):
        variants = [GOOD.replace("1.2e-02", "9e-03"), GOOD.replace("1.2e-02", "nan"),
                    GOOD.replace("4.173375e-13", "-1e-13"), GOOD.replace("4.173375e-13", "nan"),
                    GOOD + "Update_Time_Level: invalid data\n", GOOD + "Mesh check failed\n"]
        for text in variants:
            self.assertFalse(run_gate.validate_static_evidence(text)["valid"], text)

    def test_later_zero_never_erases_earlier_failure(self):
        text = "2067 orphan cells\n" + GOOD
        self.assertFalse(run_gate.validate_static_evidence(text)["valid"])

    def test_donor_warning_is_not_classification_count(self):
        text = GOOD.replace("Marked 30 donor cells in register overset-donor-cells-r0.", "1078 donor cells with donor-receptor size ratio > 27.0.")
        self.assertFalse(run_gate.validate_static_evidence(text)["valid"])

    def test_unknown_audit_schema_fails_explicitly(self):
        with patch("audit.parse_solver_text", return_value={"mystery": 0}):
            with self.assertRaisesRegex(runtime.RuntimeGateError, "contract unrecognized"):
                run_gate.validate_static_evidence(GOOD)

    def test_none_return_is_execution_not_validation(self):
        steps = run_gate.Steps()
        self.assertIsNone(steps.call("native_check", lambda: None))
        self.assertEqual(steps.rows[0]["validation"], "not_performed")
        self.assertNotIn("valid", steps.rows[0])


class GateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.contract = runtime.load_contract()
        self.background, self.arm = self.root / "background.msh", self.root / "arm.msh"
        self.background.write_bytes(b"offline mesh background")
        self.arm.write_bytes(b"offline mesh arm")

    def tearDown(self):
        self.temp.cleanup()

    def session(self, **kwargs):
        attempt = self.root / "session"
        attempt.mkdir()
        solver = FakeSolver(self.contract, **kwargs)
        report = run_gate.static_session(solver, attempt, self.background, self.arm, self.contract)
        return solver, report, attempt

    def args(self, gate="static", **kwargs):
        values = dict(gate=gate, attempt=str(self.root / "gate"), contract=None,
                      mesh_background=str(self.background), mesh_arm=str(self.arm), case=None,
                      processors=1, api_only=False, no_diagnostic_case=False)
        values.update(kwargs)
        return argparse.Namespace(**values)

    def test_static_uses_expert_then_single_intersect_never_iterates(self):
        solver, report, attempt = self.session()
        self.assertEqual(report["status"], "candidate_pending_session_close", report["errors"])
        self.assertFalse(report["valid"])
        commands = [x[0] for x in solver.commands]
        self.assertLess(commands.index("expert"), commands.index("intersect"))
        self.assertNotIn("intersect_all", commands)
        self.assertNotIn("iterate", commands)
        self.assertEqual(report["mesh_time_steps"], 0)
        self.assertFalse((attempt / "base_uninitialized.cas.h5").exists())
        self.assertEqual(report["zone_mapping"]["canonical_to_actual"]["fluid_arm"], "fff-1_fluid_arm")
        self.assertIsInstance(report["zone_mapping"]["native_zone_ids"]["wall_l1"], int)

    def test_orphans_get_only_failed_diagnostic_case(self):
        _, report, attempt = self.session(orphan=9864)
        self.assertEqual(report["status"], "failed")
        self.assertTrue((attempt / "FAILED_DIAGNOSTIC_DO_NOT_USE_AS_BASE.cas.h5").exists())
        self.assertIsNone(report["base_candidate"])
        self.assertFalse((attempt / "base_uninitialized.cas.h5").exists())

    def test_api_none_without_native_count_cannot_pass(self):
        _, report, _ = self.session(missing_mark="donor")
        self.assertEqual(report["status"], "failed")
        self.assertTrue(any("missing" in x and "donor" in x for x in report["errors"]))

    def test_missing_case_file_after_none_write_fails(self):
        _, report, _ = self.session(save_file=False)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(any("persistence failed" in x for x in report["errors"]))

    def test_call_exception_is_preserved_and_halts_sequence(self):
        solver, report, _ = self.session(failure="intersect")
        self.assertEqual(report["status"], "failed")
        self.assertTrue(any("fixture failure at intersect" in x for x in report["errors"]))
        self.assertNotIn("mark", [x[0] for x in solver.commands])

    def test_exact_zone_mapping_rejects_ambiguous_or_extra_zone(self):
        bad = roster(self.contract)
        bad["fluid"] = ["old_fluid_background", "fluid_arm"]
        with self.assertRaises(runtime.RuntimeGateError):
            run_gate.zone_mapping(bad, self.contract)
        bad = roster(self.contract)
        bad["wall"].append("mystery_wall")
        with self.assertRaises(runtime.RuntimeGateError):
            run_gate.zone_mapping(bad, self.contract)

    def test_native_zone_ids_required(self):
        mapping = run_gate.zone_mapping(roster(self.contract), self.contract)
        with self.assertRaisesRegex(runtime.RuntimeGateError, "unique IDs"):
            run_gate.attach_zone_ids(mapping, "nothing printed")

    def test_api_only_does_not_launch_or_validate_unrun_stages(self):
        launch = Mock(side_effect=AssertionError("must not launch"))
        report = run_gate.run(self.args(api_only=True), launcher=launch, stage_root=self.root)
        launch.assert_not_called()
        self.assertEqual(report["status"], "not_run")
        self.assertFalse(report["valid"])
        self.assertTrue(all(state == "not_run" for state in report["stages"].values()))
        self.assertFalse(report["api_inventory"]["time_step_executed"])

    def test_motion_is_blocked_without_any_external_call(self):
        launch = Mock(side_effect=AssertionError("must not launch"))
        report = run_gate.run(self.args(gate="motion"), launcher=launch, stage_root=self.root)
        launch.assert_not_called()
        self.assertEqual(report["status"], "blocked")
        self.assertFalse(report["valid"])
        self.assertFalse(report["motion_authorized"])

    def test_owned_cleanup_failure_prevents_base_promotion(self):
        @contextlib.contextmanager
        def launch(*args, **kwargs):
            yield FakeSolver(self.contract), {}
            raise RuntimeError("owned close unverified")
        report = run_gate.run(self.args(), launcher=launch, stage_root=self.root,
                              guard=lambda *a, **k: {"status": "passed"},
                              upstream_verifier=lambda *a, **k: {"status": "passed"})
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["valid"])
        self.assertFalse((self.root / "gate/base_uninitialized.cas.h5").exists())
        self.assertTrue((self.root / "gate/FAILED_DIAGNOSTIC_DO_NOT_USE_AS_BASE.cas.h5").exists())
        role = json.loads((self.root / "gate/failure_case_role.json").read_text())
        self.assertFalse(role["valid_base"])

    def test_fake_complete_static_only_passes_static_and_quality(self):
        @contextlib.contextmanager
        def launch(*args, **kwargs):
            yield FakeSolver(self.contract), {}
        report = run_gate.run(self.args(), launcher=launch, stage_root=self.root,
                              guard=lambda *a, **k: {"status": "passed"},
                              upstream_verifier=lambda *a, **k: {"status": "passed"})
        self.assertEqual(report["status"], "passed", report)
        self.assertEqual(report["stages"]["mesh_cycle"], "not_run")
        self.assertEqual(report["stages"]["motion_api"], "not_run")
        self.assertFalse(report["motion_authorized"])
        self.assertTrue((self.root / "gate/base_acceptance.json").is_file())

    def test_prelaunch_block_never_calls_launcher(self):
        launcher = Mock(side_effect=AssertionError("must not launch"))
        report = run_gate.run(self.args(), launcher=launcher, stage_root=self.root,
                              guard=lambda *a, **k: {"status": "blocked", "blockers": ["license refused"]})
        launcher.assert_not_called()
        self.assertEqual(report["status"], "blocked")
        self.assertEqual(report["stages"]["static_overset"], "not_run")
        self.assertFalse(report["valid"])

    def test_missing_upstream_keeps_candidate_only(self):
        @contextlib.contextmanager
        def launcher(*args, **kwargs):
            yield FakeSolver(self.contract), {}
        report = run_gate.run(self.args(), launcher=launcher, stage_root=self.root,
                              guard=lambda *a, **k: {"status": "passed"})
        self.assertEqual(report["status"], "blocked", report)
        self.assertEqual(report["stages"]["static_overset"], "passed")
        self.assertFalse(report["valid"])
        self.assertTrue((self.root / "gate/candidate_uninitialized.cas.h5").exists())
        self.assertFalse((self.root / "gate/base_acceptance.json").exists())

    def test_guard_verifies_frozen_inputs_before_readonly_license_check(self):
        runtime.write_json_new(self.root / "contract.json", self.contract)
        (self.root / "frozen").mkdir()
        runtime.write_json_new(self.root / "frozen/input_manifest.json", {"files": []})
        with patch("preflight.verify_inputs", return_value={"status": "failed"}) as verify, \
             patch("preflight.inspect_environment") as environment:
            result = run_gate.prelaunch_guard(self.contract, stage_root=self.root)
            verify.assert_called_once()
            environment.assert_not_called()
            self.assertEqual(result["status"], "blocked")
        with patch("preflight.verify_inputs", return_value={"status": "passed"}), \
             patch("preflight.inspect_environment", return_value={"status": "blocked", "blockers": ["license"]}) as environment:
            result = run_gate.prelaunch_guard(self.contract, stage_root=self.root)
            self.assertTrue(environment.call_args.kwargs["license_probe"])
            self.assertEqual(result["status"], "blocked")

    def test_nonfinite_error_report_is_serializable(self):
        path = self.root / "invalid_number.json"
        runtime.write_json_new(path, run_gate.validate_static_evidence(GOOD.replace("4.173375e-13", "nan")))
        data = json.loads(path.read_text())
        self.assertFalse(data["valid"])
        self.assertIn("invalid_nonfinite_number", data["native"]["minimum_volume_observations"][0]["value"])


class UpstreamTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.contract = runtime.load_contract()

    def tearDown(self):
        self.temp.cleanup()

    def fixture_chain(self, verify_state="passed"):
        import mesh_prepare
        bg, arm = self.root / "background.scdoc", self.root / "arm.scdoc"
        bg.write_bytes(b"offline fixture background CAD")
        arm.write_bytes(b"offline fixture arm CAD")
        source = runtime.file_record(runtime.STAGE_ROOT.parent.parent / self.contract["source"]["step_relative_to_workspace"])
        built = self.root / "build_report.json"
        runtime.write_json_new(built, {"mode": "build", "state": "passed", "source_unchanged": True,
            "source_sha256_after": source["sha256"], "domains": {"background": {"state": "passed"}, "arm": {"state": "passed"}}})
        verify = self.root / "verify_report.json"
        runtime.write_json_new(verify, {"schema_version": 1, "stage": "15_dynamic_mesh", "mode": "verify", "state": verify_state,
            "cad_domains": {"state": verify_state, "independent_reopen_completed": True, "physical_face_ownership_checked": True},
            "cad_contact": {"state": "passed", "kernel_failures": 0, "cross_group_penetrations": 0, "unresolved_zero_clearance": 0},
            "source_unchanged": True, "input_integrity": {"state": "passed"},
            "source_sha256_before": source["sha256"], "source_sha256_after": source["sha256"],
            "build_report": str(built), "build_report_sha256": runtime.sha256(built),
            "artifacts": {"background": runtime.file_record(bg), "arm": runtime.file_record(arm)}})
        attempt = self.root / "mesh"
        mesh_prepare.generate(attempt, bg, arm, cad_verification=verify, stage_root=self.root)
        runtime.write_json_new(attempt / "workbench_completion.json", {"status": "completed_pending_mesh_inventory",
            "systems": [{"domain": "background"}, {"domain": "arm"}]})
        meshes = []
        for name in ("background", "arm"):
            path = attempt / (name + ".msh")
            path.write_bytes(b"offline fixture mesh")
            meshes.append(path)
            runtime.write_json_new(attempt / (name + "_meshing_stats.json"), {"status": "generated_pending_export_and_fluent_checks",
                "domain": name, "mesh_generated": True, "nodes": 50, "elements": 100,
                "quality": {"orthogonal_quality": {"minimum": .02}}})
        manifest = mesh_prepare.collect(attempt, *meshes)
        self.assertEqual(manifest["status"], "mesh_exported_pending_fluent_static_gate")
        return attempt / "mesh_manifest.json", meshes, verify

    def test_full_offline_fixture_chain_hashes_link(self):
        manifest, meshes, _ = self.fixture_chain()
        result = run_gate.upstream_chain(manifest, *meshes, self.contract, stage_root=self.root)
        self.assertEqual(result["status"], "passed", result)
        self.assertFalse(result["motion_authorized"])

    def test_generated_or_not_run_cad_report_cannot_promote(self):
        manifest, meshes, _ = self.fixture_chain(verify_state="not_run")
        result = run_gate.upstream_chain(manifest, *meshes, self.contract, stage_root=self.root)
        self.assertEqual(result["status"], "blocked")
        self.assertIn("verify report did not pass", result["reason"])

    def test_changed_real_artifact_hash_cannot_promote(self):
        manifest, meshes, _ = self.fixture_chain()
        meshes[1].write_bytes(b"changed mesh")
        result = run_gate.upstream_chain(manifest, *meshes, self.contract, stage_root=self.root)
        self.assertEqual(result["status"], "blocked")
        self.assertIn("hash mismatch", result["reason"])

    def test_different_input_mesh_cannot_promote(self):
        manifest, meshes, _ = self.fixture_chain()
        other = self.root / "other.msh"
        other.write_bytes(b"other mesh")
        result = run_gate.upstream_chain(manifest, meshes[0], other, self.contract, stage_root=self.root)
        self.assertEqual(result["status"], "blocked")
        self.assertIn("does not match", result["reason"])


if __name__ == "__main__":
    unittest.main()

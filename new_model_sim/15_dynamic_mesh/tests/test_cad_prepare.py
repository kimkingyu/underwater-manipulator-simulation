"""Offline only: generators, template contracts and SDK-shaped test doubles."""
import ast
import codecs
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import traceback
from types import SimpleNamespace
import unittest
from unittest.mock import patch

STAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("stage15_cad_prepare", STAGE / "cad_prepare.py")
cad = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cad)


def template_functions(filename, *names):
    source = (STAGE / "cad" / filename).read_text(encoding="utf-8")
    tree = ast.parse(source)
    nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    if len(nodes) != len(names):
        raise AssertionError("Test did not select exact template functions")
    return compile(ast.Module(body=nodes, type_ignores=[]), filename, "exec")


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_offline_test_", dir=STAGE / "cad")
        self.root = Path(self.temp.name)
        self.before = {str(path): cad.sha256(path) for path in (cad.CONTRACT_PATH, cad.RECIPE_PATH, cad.DOMAIN_REFERENCE_PATH,
                                                               cad.POSE_REFERENCE_PATH, cad.WORKSPACE_ROOT / "简化模型_导出/AUVM.step")}
        for path in (STAGE / "runs/cad_build/attempt_001/build_report.json",
                     STAGE / "runs/cad_build/attempt_002/build_report.json",
                     cad.WORKSPACE_ROOT / "new_model_sim/05_overset_cad/overset_manifest.json"):
            if path.is_file():
                self.before[str(path)] = cad.sha256(path)

    def tearDown(self):
        self.temp.cleanup()
        for path, digest in self.before.items():
            self.assertEqual(cad.sha256(Path(path)), digest, path)

    def attempt(self, name="attempt"):
        return self.root / name

    def test_probe_ast_utf8_and_not_run(self):
        result = cad.generate("probe", self.attempt())
        script = Path(result["script_path"])
        source = script.read_bytes().decode("utf-8")
        ast.parse(source)
        self.assertTrue(source.startswith("# Python Script, API Version = V242"))
        self.assertEqual(result["state"], "not_run")
        self.assertFalse(result["external_software_started"])
        self.assertFalse(Path(result["output_json"]).exists())
        config = result["config"]
        self.assertEqual(config["joint_axes"][0], [0, 0, -1])
        self.assertEqual(config["pose_rad"], [0.225, -0.25, -0.2])
        self.assertEqual(len(config["j1_samples_rad"]), 9)
        self.assertIn(0.25, config["j1_samples_rad"])
        self.assertEqual(config["j1_samples_rad"][-1], 0.275)
        self.assertEqual(config["step_length_unit"]["unit"], "mm")
        self.assertGreater(config["step_length_unit"]["declaration_count"], 0)
        self.assertNotIn("DocumentSave.Execute", source)
        self.assertNotIn("CylinderBody", source)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("exec(", source)
        # The embedded JSON must survive Python-literal and JSON unescaping.
        parsed = ast.parse(source)
        assignment = next(node for node in parsed.body if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "CFG" for t in node.targets))
        embedded = json.loads(ast.literal_eval(assignment.value.args[0]))
        self.assertEqual(embedded, config)

    def test_probe_refuses_existing_attempt(self):
        result = cad.generate("probe", self.attempt())
        digest = cad.sha256(Path(result["script_path"]))
        with self.assertRaises(FileExistsError):
            cad.generate("probe", self.attempt())
        self.assertEqual(cad.sha256(Path(result["script_path"])), digest)

    def test_refuses_empty_existing_directory(self):
        self.attempt().mkdir()
        with self.assertRaises(FileExistsError):
            cad.generate("probe", self.attempt())
        self.assertEqual(list(self.attempt().iterdir()), [])

    def test_refuses_outside_stage_and_root(self):
        for path in (STAGE, STAGE.parent / "outside", "../outside", cad.WORKSPACE_ROOT / "简化模型_导出/AUVM.step"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                cad.generate("probe", path)

    def test_relative_path_is_stage_relative(self):
        relative = self.attempt().relative_to(STAGE)
        result = cad.generate("probe", relative)
        self.assertEqual(Path(result["script_path"]).parent, self.attempt())

    def test_refuses_symlink_escape_when_supported(self):
        link = self.root / "outside_link"
        try:
            link.symlink_to(STAGE.parent, target_is_directory=True)
        except OSError:
            if os.name != "nt":
                self.skipTest("Directory symlink unavailable")
            command = "New-Item -ItemType Junction -Path '%s' -Target '%s' | Out-Null" % (str(link).replace("'", "''"), str(STAGE.parent).replace("'", "''"))
            junction = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
                                      capture_output=True, text=True, timeout=20)
            if junction.returncode != 0:
                self.skipTest("Neither directory symlink nor junction could be created")
        try:
            with self.assertRaises(ValueError):
                cad.generate("probe", link / "not_authorized")
        finally:
            if link.is_symlink():
                link.unlink()
            else:
                # os.rmdir on a Windows junction removes the link, not its target.
                link.rmdir()

    def test_resolved_junction_escape_is_rejected(self):
        with patch.object(Path, "resolve", return_value=STAGE.parent) as resolver:
            # Both root and requested path resolve equally, which must never be
            # accepted as a strictly nested output even under redirected parents.
            with self.assertRaises(ValueError):
                cad.stage_path(self.attempt())
            self.assertTrue(resolver.called)

    def test_sample_range_and_count(self):
        for count in (True, 0, 8, 10, 66, 9.0):
            with self.subTest(count=count), self.assertRaises(ValueError):
                cad.sample_angles(count, 0.225, 0.275)
        for low, high in ((0.224, 0.275), (0.225, 0.276), (float("nan"), 0.275), (0.225, float("inf"))):
            with self.assertRaises(ValueError):
                cad.sample_angles(9, low, high)
        self.assertEqual(len(cad.sample_angles(65, 0.225, 0.275)), 65)

    def test_padding_validation_happens_before_writes(self):
        for padding in (float("nan"), float("inf"), -0.1, 0.001, 0.3, True):
            with self.subTest(padding=padding), self.assertRaises(ValueError):
                cad.generate("build", self.attempt(), padding=padding)
            self.assertFalse(self.attempt().exists())

    def test_reference_hash_change_is_rejected(self):
        real_hash = cad.sha256
        def tampered(path):
            return "0" * 64 if path == cad.RECIPE_PATH else real_hash(path)
        with patch.object(cad, "sha256", side_effect=tampered), self.assertRaises(ValueError):
            cad.generate("probe", self.attempt())
        self.assertFalse(self.attempt().exists())

    def test_recipe_is_hash_pinned_and_uniquely_sliced(self):
        source = cad.approved_recipe_code()
        ast.parse(source)
        self.assertIn("def approved_extended_parts(tools):", source)
        self.assertIn("e.Unite(lst(j3_fill), tol_used)", source)
        self.assertIn("ext[m].Transform(pose_matrix(GROUP[m], Q))", source)
        for forbidden in ("CylinderBody", "cyl_fill", "DocumentSave", "OUT_SCDOC", "V4_JSON"):
            self.assertNotIn(forbidden, source)
        for text, first, last in (("a a b", "a", "b"), ("a b b", "a", "b"), ("b a", "a", "b")):
            with self.assertRaises(ValueError):
                cad.unique_slice(text, first, last)

    def test_build_script_contains_two_domains_and_excludes_legacy_tail(self):
        result = cad.generate("build", self.attempt())
        source = Path(result["script_path"]).read_text(encoding="utf-8")
        tree = ast.parse(source)
        self.assertEqual(sorted(result["config"]["output_scdoc"]), ["arm", "background"])
        self.assertIn("l2_cylinder_gap_fill_executed': False", source)
        self.assertIn("for domain in ('background', 'arm'):", source)
        for forbidden in ("cyl_fill_parts", "cyl_gap_fill", "fluid_domain_qe.scdoc", "fluid_domain_v4.json", "background_fluid.scdoc')"):
            self.assertNotIn(forbidden, source)
        # The only new cylinder is an isolated analytic measurement control,
        # never the removed l2/base0 physical gap-fill recipe.
        cylinders = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and "CylinderBody.Create" in ast.get_source_segment(source, node)]
        self.assertEqual([node.name for node in cylinders], ["volume_calibration"])
        self.assertNotIn("fluid.SeparatePieces()", source)
        self.assertIn("split_target = fluid.Copy()", source)
        self.assertIn("tessellation_used_for_acceptance': False", source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "Execute" and isinstance(node.func.value, ast.Name) and node.func.value.id == "DocumentSave":
                self.assertEqual(ast.unparse(node.args[0]), "path")
        self.assertFalse(Path(result["output_json"]).exists())

    def build_fixture(self, *, state="passed", escape=False):
        fixture = self.root / "build_fixture"
        fixture.mkdir()
        source_hash = cad.sha256(cad.WORKSPACE_ROOT / "简化模型_导出/AUVM.step")
        domains = {}
        for name in ("background", "arm"):
            path = fixture / (name + ".scdoc")
            path.write_bytes(b"offline schema fixture, not a real CAD document")
            domains[name] = {"state": "passed", "scdoc": str(path), "scdoc_sha256": cad.sha256(path)}
        if escape:
            domains["arm"]["scdoc"] = str(STAGE.parent / "not_authorized.scdoc")
        report = fixture / "build_report.json"
        report.write_text(json.dumps({"stage": "15_dynamic_mesh", "mode": "build", "state": state,
                                      "source_unchanged": True, "source_sha256_before": source_hash,
                                      "source_sha256_after": source_hash, "domains": domains}), encoding="utf-8")
        return report

    def test_verify_generation_binds_hashes_without_claiming_execution(self):
        path = self.build_fixture()
        result = cad.generate("verify", self.attempt(), build_report=path)
        source = Path(result["script_path"]).read_text(encoding="utf-8")
        ast.parse(source)
        self.assertEqual(result["state"], "not_run")
        self.assertEqual(result["config"]["build_report_sha256"], cad.sha256(path))
        self.assertIn("Window.ActiveWindow.Groups", source)
        self.assertIn("independent_reopen_completed", source)
        self.assertIn("report['artifacts']", source)
        self.assertEqual(len(result["config"]["frozen_sources"]), 7)

    def test_verify_rejects_not_run_report(self):
        path = self.build_fixture(state="not_run")
        with self.assertRaises(ValueError):
            cad.generate("verify", self.attempt(), build_report=path)
        self.assertFalse(self.attempt().exists())

    def test_verify_rejects_artifact_hash_mismatch(self):
        path = self.build_fixture()
        build = json.loads(path.read_text(encoding="utf-8"))
        Path(build["domains"]["arm"]["scdoc"]).write_bytes(b"changed")
        with self.assertRaises(ValueError):
            cad.generate("verify", self.attempt(), build_report=path)

    def test_verify_rejects_artifact_escape(self):
        path = self.build_fixture(escape=True)
        with self.assertRaises((ValueError, OSError)):
            cad.generate("verify", self.attempt(), build_report=path)

    def test_runtime_sdk_import_error_is_structured_and_cannot_overwrite(self):
        result = cad.generate("probe", self.attempt())
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        # CPython has no SpaceClaim host/System import; this intentionally tests
        # only failure-report mechanics, never starts SpaceClaim or imports clr.
        first = subprocess.run([sys.executable, "-B", result["script_path"]], capture_output=True, text=True, env=env, timeout=20)
        report_path = Path(result["output_json"])
        self.assertTrue(report_path.is_file(), first.stderr)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(report["state"], "failed")
        self.assertTrue(any(row["type"] == "execution_error" for row in report["errors"]))
        self.assertFalse(report["motion_authorized"])
        digest = cad.sha256(report_path)
        again = subprocess.run([sys.executable, "-B", result["script_path"]], capture_output=True, text=True, env=env, timeout=20)
        self.assertNotEqual(again.returncode, 0)
        self.assertEqual(cad.sha256(report_path), digest)


class TemplateBehaviorTests(unittest.TestCase):
    def pair_namespace(self):
        namespace = {"math": math, "FIXED": ("hull", "base0"), "TOL_SEAL": 1e-7,
                     "lst": lambda *args: list(args), "local_faces": lambda shape, point: {"state": "passed"}}
        exec(template_functions("sc_common.py.in", "u", "finite", "xyz", "pair_measurement", "pair_census"), namespace)
        return namespace

    def fake_shape(self, distance=0.0, overlap=0.0, fail_distance=False, fail_intersection=False):
        class Shape:
            Volume = 0.0
            def Copy(self):
                return self.__class__()
            def GetClosestSeparation(self, other):
                if fail_distance:
                    raise RuntimeError("kernel separation failure")
                point = SimpleNamespace(X=0.0, Y=0.0, Z=0.0)
                return SimpleNamespace(Distance=distance, PointA=point, PointB=point)
            def Intersect(self, others, tolerance):
                if fail_intersection:
                    raise RuntimeError("kernel Boolean failure")
                self.Volume = overlap
        return Shape()

    def test_design_contact_is_classified_without_authorizing_geometry(self):
        ns = self.pair_namespace()
        row = ns["pair_measurement"]("base0", "l1", {"base0": self.fake_shape(), "l1": self.fake_shape()})
        self.assertEqual(row["classification"], "design_contact")
        self.assertEqual(row["pair_kind"], "cross_fixed_moving")
        self.assertFalse(row["classification_is_geometry_authorization"])
        self.assertEqual(row["state"], "passed")

    def test_penetration_is_not_hidden_by_minimum_distance(self):
        ns = self.pair_namespace()
        row = ns["pair_measurement"]("base0", "l1", {"base0": self.fake_shape(overlap=1e-16), "l1": self.fake_shape()})
        self.assertEqual(row["classification"], "real_penetration")
        self.assertGreater(row["intersection_m3"], 0)

    def test_finite_water_gap_and_unresolved_contact(self):
        ns = self.pair_namespace()
        row = ns["pair_measurement"]("hull", "l1", {"hull": self.fake_shape(distance=0.0002), "l1": self.fake_shape()})
        self.assertEqual(row["classification"], "finite_water_gap")
        row = ns["pair_measurement"]("hull", "l1", {"hull": self.fake_shape(), "l1": self.fake_shape()})
        self.assertEqual(row["classification"], "unclassified_zero_clearance")

    def test_sub_tolerance_positive_gap_is_never_relabelled_design_contact(self):
        ns = self.pair_namespace()
        row = ns["pair_measurement"]("base0", "l1", {"base0": self.fake_shape(distance=5e-8), "l1": self.fake_shape()})
        self.assertEqual(row["classification"], "finite_water_gap")
        self.assertEqual(row["gap_resolution_state"], "blocked")

    def test_failed_or_nonfinite_kernel_measurement_never_defaults_to_zero(self):
        ns = self.pair_namespace()
        for kwargs in ({"fail_distance": True}, {"fail_intersection": True}, {"distance": float("nan")}, {"overlap": float("inf")}):
            with self.subTest(kwargs=kwargs):
                row = ns["pair_measurement"]("base0", "l1", {"base0": self.fake_shape(**kwargs), "l1": self.fake_shape()})
                self.assertEqual(row["state"], "failed")
                self.assertEqual(row["classification"], "unresolved")
                self.assertTrue(row["errors"])

    def test_all_28_pairs_and_12_cross_group_pairs(self):
        ns = self.pair_namespace()
        names = ("hull", "base0", "l1", "l2", "link3", "pin4", "jaw1", "jaw2")
        rows = ns["pair_census"]({name: self.fake_shape(distance=0.1) for name in names})
        self.assertEqual(len(rows), 28)
        self.assertEqual(sum(row["pair_kind"] == "cross_fixed_moving" for row in rows), 12)
        self.assertEqual(sum(row["pair_kind"] == "same_moving_rigid_group" for row in rows), 15)

    def test_topology_uses_properties_not_truthy_method_objects(self):
        ns = {}
        exec(template_functions("sc_common.py.in", "u", "topology"), ns)
        sdk = SimpleNamespace(GetMethods=lambda: [])
        shape = SimpleNamespace(IsClosed=True, IsManifold=True, PieceCount=1, GetType=lambda: sdk)
        result = ns["topology"](shape)
        self.assertEqual(result["state"], "passed")
        self.assertFalse(result["check"]["available"])
        shape.IsClosed = lambda: False
        result = ns["topology"](shape)
        self.assertEqual(result["state"], "blocked")
        self.assertNotIn("closed", result)

    def test_available_check_is_invoked_not_coerced(self):
        ns = {}
        exec(template_functions("sc_common.py.in", "u", "topology"), ns)
        method = SimpleNamespace(Name="Check", GetParameters=lambda: [], ToString=lambda: "Boolean Check()")
        sdk = SimpleNamespace(GetMethods=lambda: [method])
        shape = SimpleNamespace(IsClosed=True, IsManifold=True, PieceCount=1, GetType=lambda: sdk, Check=lambda: False)
        result = ns["topology"](shape)
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["check"]["result"], False)

    def sealed_sweep_namespace(self, *, penetration=False, initial_kernel_failure=False):
        class RigidShape:
            def Copy(self):
                return self.__class__()
            def Transform(self, matrix):
                self.rotation = matrix
        fixed = ("hull", "base0")
        moving = ("l1", "l2", "link3", "pin4", "jaw1", "jaw2")
        def pair(a, b, parts):
            classification = "finite_water_gap"
            if (a, b) == ("base0", "l1"):
                classification = "real_penetration" if penetration else "design_contact"
            return {"a": a, "b": b, "state": "passed", "classification": classification}
        ns = {"report": {}, "FIXED": fixed, "MOVING": moving, "Q": [0.225, -0.25, -0.2],
              "CFG": {"j1_samples_rad": cad.sample_angles(9, 0.225, 0.275)},
              "JC": [[-4e-6, -0.0366, -0.460702]], "JA": [[0, 0, -1]],
              "Point": SimpleNamespace(Create=lambda *args: args),
              "Direction": SimpleNamespace(Create=lambda *args: args),
              "Line": SimpleNamespace(Create=lambda *args: args),
              "Matrix": SimpleNamespace(CreateRotation=lambda line, angle: angle),
              "pair_measurement": pair, "save": lambda: None,
              "pair_census": lambda parts: [{"state": "failed" if initial_kernel_failure else "passed"}]}
        exec(template_functions("sc_domains.py.in", "sealed_motion_contacts"), ns)
        return ns, {name: RigidShape() for name in fixed + moving}

    def test_sealed_design_contact_allows_static_cad_but_never_motion(self):
        ns, parts = self.sealed_sweep_namespace()
        ns["sealed_motion_contacts"](parts)
        gate = ns["report"]["cad_contact"]
        self.assertEqual(gate["state"], "passed")
        self.assertTrue(gate["cad_build_allowed"])
        self.assertTrue(gate["overset_connectivity_pending"])
        self.assertFalse(gate["motion_authorized"])
        samples = ns["report"]["sealed_motion_samples"]
        self.assertEqual(len(samples), 9)
        self.assertEqual(sum(len(sample["pairs"]) for sample in samples), 108)
        self.assertEqual(samples[0]["relative_j1_rad"], 0.0)
        self.assertAlmostEqual(samples[-1]["relative_j1_rad"], 0.05)

    def test_sealed_cross_penetration_or_kernel_failure_blocks_cad(self):
        for kwargs in ({"penetration": True}, {"initial_kernel_failure": True}):
            ns, parts = self.sealed_sweep_namespace(**kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ns["sealed_motion_contacts"](parts)
            self.assertEqual(ns["report"]["cad_contact"]["state"], "failed")
            self.assertFalse(ns["report"]["cad_contact"]["cad_build_allowed"])


class VolumeDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.ns = {"math": math, "VOLUME_ABSOLUTE_TOLERANCE_M3": 1e-10, "VOLUME_RELATIVE_TOLERANCE": 1e-8}
        exec(template_functions("sc_common.py.in", "u", "finite"), self.ns)
        exec(template_functions("sc_volume.py.in", "volume_tolerance", "volume_ledger", "native_volume_snapshot",
                                "validated_mass_reading", "legacy_native_mass_balance", "vector_cross", "vector_dot", "triangle_volume",
                                "difference_probe"), self.ns)

    def mass_snapshot(self, value, error=0.0):
        return {"mass_properties": {"state": "passed", "volume_at_unit_density_m3": value,
                                     "estimated_absolute_error_m3": error,
                                     "relative_accuracy_reported": error / max(abs(value), 1e-30)}}

    def mass_ledger(self, box=10.0, material=2.0, water=8.0, error=0.0):
        return {"box_before_boolean": self.mass_snapshot(box, error),
                "material_before_boolean": self.mass_snapshot(material, error),
                "all_water_before_separation": self.mass_snapshot(water, error),
                "main_after_imprint": self.mass_snapshot(water, error)}

    def test_attempt001_real_numbers_remain_failed_at_original_threshold(self):
        result = self.ns["volume_ledger"](34.827455800704, 0.45051678826658415, 34.376931433138921,
                                          [34.376931433138921], 34.376931433138921, 34.376931433138921, 0.0)
        self.assertEqual(result["state"], "failed")
        self.assertAlmostEqual(result["residuals"]["final_m3"], 7.579298497262244e-6, places=17)
        self.assertEqual(result["residuals"]["separate_pieces_m3"], 0.0)
        self.assertEqual(result["residuals"]["imprint_m3"], 0.0)
        self.assertAlmostEqual(result["effective_tolerance_m3"], 3.4827455800704e-7, places=19)
        self.assertFalse(result["threshold_was_relaxed"])

    def test_complete_piece_and_cavity_ledger_reconciles(self):
        result = self.ns["volume_ledger"](10.0, 2.0, 8.0, [7.0, 0.6, 0.4], 7.0, 7.0, 1.0)
        self.assertEqual(result["state"], "passed")
        self.assertEqual(result["all_piece_sum_m3"], 8.0)
        self.assertEqual(result["discarded_cavities_m3"], 1.0)

    def test_mutated_target_reread_does_not_replace_original_all_piece_volume(self):
        # SeparatePieces is allowed to turn the original 8m3 target into one
        # 7m3 result. Reading that reference after splitting loses the 1m3 cavity.
        bad = self.ns["volume_ledger"](10.0, 2.0, 7.0, [7.0, 1.0], 7.0, 7.0, 1.0)
        self.assertEqual(bad["state"], "failed")
        self.assertEqual(bad["residuals"]["separate_pieces_m3"], -1.0)
        fixed = self.ns["volume_ledger"](10.0, 2.0, 8.0, [7.0, 1.0], 7.0, 7.0, 1.0)
        self.assertEqual(fixed["state"], "passed")

    def test_lost_cavity_and_imprint_volume_change_are_not_hidden(self):
        missing = self.ns["volume_ledger"](10.0, 2.0, 8.0, [7.0, 1.0], 7.0, 7.0, 0.0)
        changed = self.ns["volume_ledger"](10.0, 2.0, 8.0, [8.0], 8.0, 7.9, 0.0)
        self.assertEqual(missing["state"], "failed")
        self.assertEqual(changed["state"], "failed")
        self.assertEqual(missing["residuals"]["cavity_selection_m3"], 1.0)
        self.assertAlmostEqual(changed["residuals"]["imprint_m3"], -0.1)

    def test_negative_nan_or_empty_piece_list_cannot_be_abs_corrected(self):
        for pieces in ([], [-1.0, 9.0], [float("nan")]):
            with self.assertRaises(ValueError):
                self.ns["volume_ledger"](10.0, 2.0, 8.0, pieces, 8.0, 8.0, 0.0)

    def test_deleted_boolean_target_is_explicit_empty_without_volume_read(self):
        class Deleted:
            IsDeleted = True
            @property
            def Volume(self):
                raise AssertionError("Consumed/deleted target was dereferenced")
        result = self.ns["native_volume_snapshot"](Deleted())
        self.assertEqual(result["state"], "passed")
        self.assertTrue(result["empty_boolean_result"])
        self.assertEqual(result["volume_m3"], 0.0)

    def test_mass_accuracy_and_body_cache_change_are_recorded(self):
        class Shape:
            IsDeleted = False
            IsClosed = True
            IsManifold = True
            PieceCount = 1
            value = 2.0
            @property
            def Volume(self):
                return self.value
            @property
            def MassProperties(self):
                self.value = 2.0000001
                return SimpleNamespace(Mass=2.0, RelativeAccuracy=1e-9)
        result = self.ns["native_volume_snapshot"](Shape())
        self.assertAlmostEqual(result["volume_cache_change_m3"], 1e-7)
        self.assertEqual(result["mass_properties"]["estimated_absolute_error_m3"], 2e-9)
        self.assertFalse(result["mass_properties"]["precision_adjustment_available"])

    def test_zero_residual_with_unknown_or_large_uncertainty_still_blocks(self):
        rows = self.mass_ledger()
        uncalibrated = self.ns["legacy_native_mass_balance"](rows, [], 1e-7, {"native_accuracy_calibrated": False})
        uncertain = self.ns["legacy_native_mass_balance"](self.mass_ledger(error=1e-5), [], 1e-7, {"native_accuracy_calibrated": True})
        self.assertEqual(uncalibrated["state"], "blocked")
        self.assertEqual(uncertain["state"], "blocked")
        self.assertFalse(uncertain["threshold_was_relaxed"])

    def test_calibrated_metric_must_include_error_budget_in_same_threshold(self):
        good = self.ns["legacy_native_mass_balance"](self.mass_ledger(error=1e-9), [], 1e-7, {"native_accuracy_calibrated": True})
        bad = self.ns["legacy_native_mass_balance"](self.mass_ledger(water=7.99999991, error=1e-8), [], 1e-7, {"native_accuracy_calibrated": True})
        self.assertEqual(good["state"], "passed")
        self.assertEqual(bad["state"], "failed")
        self.assertGreater(abs(bad["residual_m3"]) + bad["combined_estimated_error_m3"], 1e-7)

    def test_nonfinite_inconsistent_or_missing_native_accuracy_never_passes(self):
        for key, value in (("estimated_absolute_error_m3", float("nan")),
                           ("estimated_absolute_error_m3", -1.0),
                           ("relative_accuracy_reported", float("inf")),
                           ("relative_accuracy_reported", None),
                           ("relative_accuracy_reported", False),
                           ("estimated_absolute_error_m3", 0.1)):
            rows = self.mass_ledger()
            rows["main_after_imprint"]["mass_properties"][key] = value
            with self.subTest(key=key, value=value):
                result = self.ns["legacy_native_mass_balance"](rows, [], 1e-7, {"native_accuracy_calibrated": True})
                self.assertEqual(result["state"], "blocked")

    def test_complement_probe_uses_disposable_copies_not_original_tools(self):
        class Tools(list):
            def Add(self, value):
                self.append(value)
        class GenericList:
            def __getitem__(self, key):
                return Tools
        class Shape:
            IsDeleted = False
            def Copy(self):
                return Shape()
            def Subtract(self, tools, tolerance):
                for tool in tools:
                    tool.IsDeleted = True
                self.IsDeleted = True
            @property
            def Volume(self):
                raise AssertionError("No volume may be read after target deletion")
        self.ns.update({"List": GenericList(), "Body": Shape, "TOL_BOOL": 1e-7})
        a, b = Shape(), Shape()
        result = self.ns["difference_probe"](a, b, "same_material")
        self.assertEqual(result["state"], "passed")
        self.assertTrue(result["empty"])
        self.assertTrue(result["tool_consumed"])
        self.assertFalse(a.IsDeleted)
        self.assertFalse(b.IsDeleted)

    def test_signed_tetrahedra_preserve_cavity_sign_and_translation(self):
        vertices = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        facets = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
        def integrate(points, faces, reference):
            return math.fsum(self.ns["triangle_volume"](points[a], points[b], points[c], reference) for a, b, c in faces)
        outer = integrate(vertices, facets, [0.0, 0.0, 0.0])
        offset = [1000.0, -2000.0, 3000.0]
        moved = [[point[i] + offset[i] for i in range(3)] for point in vertices]
        self.assertAlmostEqual(outer, 1.0 / 6.0)
        self.assertAlmostEqual(integrate(moved, facets, offset), outer)
        inner = [[value * 0.5 for value in point] for point in vertices]
        reverse = [(a, c, b) for a, b, c in facets]
        cavity = integrate(inner, reverse, [0.0, 0.0, 0.0])
        self.assertLess(cavity, 0.0)
        self.assertAlmostEqual(outer + cavity, 7.0 / 48.0)


class CalibrationIsolationTests(unittest.TestCase):
    def namespace(self, *, fail_inner=False, closed=True, wrong_radius=False, fail_tessellation=None):
        calls, checkpoints = [], []
        def point(x, y, z):
            return SimpleNamespace(X=x, Y=y, Z=z)
        def geom(kind, **values):
            return SimpleNamespace(GetType=lambda: SimpleNamespace(Name=kind), **values)
        class Shape:
            IsDeleted = False
            IsManifold = True
            PieceCount = 1
            def __init__(self, name, volume, bounds, z0=None, z1=None):
                self.name, self.volume, self.bounds = name, volume, bounds
                self.IsClosed = closed if "cylinder" in name else True
                self.z0, self.z1 = z0, z1
                self.Faces = []
                if "cylinder" in name:
                    radius = 0.3 if wrong_radius else 0.2
                    cylindrical = geom("Cylinder", Radius=radius, Axis=SimpleNamespace(Origin=point(0, 0, z0), Direction=point(0, 0, 1)))
                    self.Faces.append(SimpleNamespace(Geometry=cylindrical))
                    for height, sign in ((z0, -1), (z1, 1)):
                        plane = geom("Plane", Frame=SimpleNamespace(Origin=point(0, 0, height), DirZ=point(0, 0, sign)))
                        self.Faces.append(SimpleNamespace(Geometry=plane))
            @property
            def Volume(self):
                if self.IsDeleted:
                    raise RuntimeError("Deleted body volume must not be dereferenced")
                return self.volume
            @property
            def MassProperties(self):
                return SimpleNamespace(Mass=self.Volume, RelativeAccuracy=0.0)
            def Copy(self):
                return Shape(self.name, self.volume, [list(row) for row in self.bounds], self.z0, self.z1)
            def Subtract(self, tools, tolerance):
                tool = tools[0]
                if fail_inner and tool.name == "contained_cylinder":
                    tool.IsDeleted = True
                    raise RuntimeError("General Failure: original interior-cavity diagnostic")
                clipped_height = min(self.bounds[1][2], tool.z1) - max(self.bounds[0][2], tool.z0)
                self.volume -= math.pi * 0.2 ** 2 * clipped_height
                self.name = "through_hole" if tool.name == "through_cylinder" else "interior_cavity"
                tool.IsDeleted = True
        class Created:
            def __init__(self, shape):
                self.shape = shape
                self.deleted = False
            def Delete(self):
                self.deleted = True
        def cylinder_create(center, start, end, mode, info):
            coordinates = [(p.X, p.Y, p.Z) for p in (center, start, end)]
            calls.append(coordinates)
            # Candidate interpretation from attempt003's measured axis and bbox;
            # this mock checks argument consistency, NOT live V242 compatibility.
            self.assertEqual(coordinates[1][:2], (0, 0))
            self.assertEqual(coordinates[2], (0.2, 0, coordinates[1][2]))
            z0, z1 = center.Z, start.Z
            name = "contained_cylinder" if z0 == -0.3 else "through_cylinder"
            shape = Shape(name, math.pi * 0.2 ** 2 * (z1 - z0), [[-0.2, -0.2, z0], [0.2, 0.2, z1]], z0, z1)
            return SimpleNamespace(CreatedBodies=[Created(shape)])
        def tessellation(shape):
            if shape.name == fail_tessellation:
                raise RuntimeError("One diagnostic renderer failed")
            return {"state": "passed", "levels": [{"signed_volume_m3": shape.Volume}]}
        ns = {"math": math, "traceback": traceback, "report": {}, "TOL_BOOL": 1e-7, "TOL_SEAL": 1e-7,
              "Document": SimpleNamespace(Create=lambda: None), "Point": SimpleNamespace(Create=point),
              "CylinderBody": SimpleNamespace(Create=cylinder_create), "ExtrudeType": SimpleNamespace(ForceIndependent=1),
              "world_copy": lambda created: created.shape.Copy(), "lst": lambda *shapes: [shape.Copy() for shape in shapes],
              "make_box": lambda lo, hi: Shape("box", math.prod(hi[i] - lo[i] for i in range(3)), [list(lo), list(hi)]),
              "bbox": lambda shape: shape.bounds,
              "topology": lambda shape: {"state": "passed" if shape.IsClosed else "failed", "closed": shape.IsClosed,
                                           "manifold": shape.IsManifold, "piece_count": shape.PieceCount},
              "controlled_volume_diagnostics": tessellation}
        exec(template_functions("sc_common.py.in", "u", "finite", "xyz", "BlockedEvidence"), ns)
        exec(template_functions("sc_volume.py.in", "native_volume_snapshot", "validated_mass_reading", "calibration_shape_evidence",
                                "cylinder_dimension_evidence", "calibration_measure", "volume_calibration", "run_volume_calibration"), ns)
        ns["save"] = lambda: checkpoints.append(json.loads(json.dumps(ns["report"])))
        return ns, calls, checkpoints

    def test_candidate_axis_end_circle_arguments_and_mock_controls(self):
        ns, calls, _ = self.namespace()
        result = ns["volume_calibration"]()
        self.assertEqual(calls, [[(0, 0, -0.3), (0, 0, 0.3), (0.2, 0, 0.3)],
                                 [(0, 0, -0.75), (0, 0, 0.75), (0.2, 0, 0.75)]])
        self.assertEqual(len(result["checks"]), 5)
        self.assertEqual(result["state"], "passed")
        for row in result["checks"]:
            self.assertEqual(row["shape"]["solid_evidence"]["state"], "passed")
            self.assertIn("bbox_m", row["shape"])
            self.assertIn("mass_properties", row["native"])

    def test_inner_cavity_failure_retains_all_native_evidence_and_through_control(self):
        ns, _, checkpoints = self.namespace(fail_inner=True)
        result = ns["volume_calibration"]()
        rows = {row["name"]: row for row in result["checks"]}
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["native_accuracy_calibrated"])
        self.assertEqual(rows["analytic_box_minus_cylinder"]["state"], "blocked")
        self.assertEqual(rows["analytic_box_minus_cylinder"]["operation"]["state"], "failed")
        self.assertIn("General Failure", rows["analytic_box_minus_cylinder"]["operation"]["error"])
        self.assertIn("partial_target_after_error", rows["analytic_box_minus_cylinder"])
        self.assertEqual(rows["analytic_box_minus_through_cylinder"]["state"], "passed")
        for name in ("analytic_box", "analytic_cylinder", "analytic_through_cylinder"):
            self.assertIn("volume_m3", rows[name]["native"])
            self.assertIn("topology", rows[name]["shape"])
            self.assertIn("bbox_m", rows[name]["shape"])
        before_boolean = [point for point in checkpoints if point.get("volume_calibration", {}).get("checks", [])[2].get("operation", {}).get("state") == "not_run"]
        self.assertTrue(before_boolean)
        self.assertIn("cylinder", before_boolean[-1]["volume_calibration"]["checks"][2]["operand_evidence"])

    def test_nonsolid_or_wrong_radius_cylinder_blocks_without_discarding_box_evidence(self):
        for options in ({"closed": False}, {"wrong_radius": True}):
            ns, _, _ = self.namespace(**options)
            result = ns["volume_calibration"]()
            with self.subTest(options=options):
                self.assertEqual(result["state"], "blocked")
                self.assertEqual(result["checks"][0]["state"], "passed")
                self.assertEqual(result["checks"][1]["state"], "blocked")
                self.assertEqual(len(result["checks"]), 5)
                self.assertNotEqual(result["checks"][2].get("operation", {}).get("state"), "passed")

    def test_unexpected_single_measurement_error_does_not_abort_other_checks(self):
        ns, _, _ = self.namespace(fail_tessellation="box")
        result = ns["volume_calibration"]()
        self.assertEqual(result["state"], "blocked")
        self.assertEqual(result["checks"][0]["state"], "blocked")
        self.assertIn("mass_properties", result["checks"][0]["native"])
        self.assertEqual(result["checks"][-1]["state"], "passed")

    def test_workflow_guard_preserves_partial_results_and_never_validates(self):
        ns, _, _ = self.namespace()
        ns["report"]["volume_calibration"] = {"checks": [{"name": "already_recorded", "state": "passed"}]}
        def fail():
            raise RuntimeError("unexpected calibration program error")
        ns["volume_calibration"] = fail
        result = ns["run_volume_calibration"]()
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["native_accuracy_calibrated"])
        self.assertEqual(result["checks"][0]["name"], "already_recorded")
        self.assertEqual(result["errors"][0]["operation"], "calibration_workflow")

    def test_blocked_calibration_and_background_do_not_discard_arm_diagnostics(self):
        ns, _, _ = self.namespace(fail_inner=True)
        visited = []
        def build_domain(name, ext):
            visited.append(name)
            ns["report"]["domains"][name] = {"state": "blocked", "raw_body_volume_balance": {"box_m3": 10.0, "material_m3": 2.0, "all_piece_sum_m3": 8.0}}
            raise ns["BlockedEvidence"]("uncalibrated volume precision")
        ns.update({"os": SimpleNamespace(path=SimpleNamespace(exists=lambda p: False)), "inside_stage": lambda p: True,
                   "CFG": {"output_scdoc": {"background": "background.scdoc", "arm": "arm.scdoc"}},
                   "prepare_extended_parts": lambda: {}, "sealed_motion_contacts": lambda ext: None,
                   "build_domain": build_domain})
        ns["report"]["limitations"] = []
        exec(template_functions("sc_build.py.in", "mode_main"), ns)
        with self.assertRaises(ns["BlockedEvidence"]):
            ns["mode_main"]()
        self.assertEqual(visited, ["background", "arm"])
        self.assertEqual(ns["report"]["cad_construction"]["state"], "blocked")
        for name in visited:
            self.assertIn("raw_body_volume_balance", ns["report"]["domains"][name])


class ProtocolV2IntervalTests(unittest.TestCase):
    def setUp(self):
        self.ns = {"math": math, "VOLUME_ABSOLUTE_TOLERANCE_M3": 1e-10, "VOLUME_RELATIVE_TOLERANCE": 1e-8,
                   "MAXIMUM_SINGLE_RELATIVE_ACCURACY": 0.01, "MAXIMUM_COMBINED_RELATIVE_UNCERTAINTY": 0.01,
                   "FLOAT64_EPSILON": sys.float_info.epsilon, "report": {"volume_calibration": {"native_accuracy_calibrated": True}}}
        exec(template_functions("sc_common.py.in", "u", "finite"), self.ns)
        exec(template_functions("sc_volume.py.in", "volume_tolerance", "volume_ledger", "validated_mass_reading",
                                "positive_interval_mass", "conservation_interval", "native_mass_balance",
                                "legacy_native_mass_balance", "old_strict_volume_result", "analytic_box_volume",
                                "verify_native_domain_volume"), self.ns)
        exec(template_functions("sc_domains.py.in", "domain_evidence_state"), self.ns)

    def snapshot(self, volume, relative=0.001, piece_count=1, index=None):
        row = {"state": "passed", "is_deleted": False, "closed": True, "manifold": True,
               "volume_m3": volume, "piece_count": piece_count,
               "mass_properties": {"state": "passed", "volume_at_unit_density_m3": volume,
                                   "relative_accuracy_reported": relative, "estimated_absolute_error_m3": volume * relative}}
        if index is not None:
            row["piece_index"] = index
        return row

    def case(self, cavities=(1.0,), relative=0.001, main=7.0, material=2.0):
        water = main + sum(cavities)
        box = material + water
        pieces = [{"piece_index": 0, "volume_m3": main, "reaches_all_box_sides": True,
                   "native_volume_snapshot": self.snapshot(main, relative, index=0)}]
        cavity_snapshots = []
        for index, volume in enumerate(cavities, 1):
            reading = self.snapshot(volume, relative, index=index)
            cavity_snapshots.append(reading)
            pieces.append({"piece_index": index, "volume_m3": volume, "reaches_all_box_sides": False,
                           "native_volume_snapshot": reading})
        snapshots = {"box_before_boolean": self.snapshot(box, 0.1),
                     "material_before_boolean": self.snapshot(material, relative),
                     "material_after_boolean": self.snapshot(material, relative),
                     "all_water_before_separation": self.snapshot(water, relative, len(pieces)),
                     "unseparated_water_after_split": self.snapshot(water, relative, len(pieces)),
                     "main_before_imprint": self.snapshot(main, relative, index=0),
                     "main_after_imprint": self.snapshot(main, relative, index=0)}
        return snapshots, cavity_snapshots, pieces, box

    def evaluate(self, case, calibration=True):
        snapshots, cavities, pieces, box = case
        return self.ns["native_mass_balance"](snapshots, cavities, max(1e-10, box * 1e-8),
                                              {"native_accuracy_calibrated": calibration},
                                              analytic_enclosure_m3=box, piece_records=pieces)

    def test_interval_consistency_is_explicitly_not_legacy_absolute_precision(self):
        case = self.case()
        result = self.evaluate(case)
        self.assertEqual(result["state"], "passed")
        self.assertEqual(result["legacy_strict_result"]["state"], "blocked")
        self.assertTrue(result["acceptance_basis_changed"])
        self.assertTrue(result["consistency_only_no_absolute_accuracy_certificate"])
        self.assertFalse(result["enclosure_sdk_relative_accuracy_used"])
        self.assertNotIn("threshold_was_relaxed", result)
        self.assertTrue(all(row["zero_in_interval"] for row in result["interval_checks"]))
        self.assertEqual(case[0]["box_before_boolean"]["mass_properties"]["relative_accuracy_reported"], 0.1)

    def test_old_strict_failure_is_retained_even_if_new_interval_is_compatible(self):
        case = self.case()
        case[0]["main_after_imprint"] = self.snapshot(7.001, index=0)
        result = self.evaluate(case)
        self.assertEqual(result["state"], "passed")
        raw = self.ns["volume_ledger"](10, 2, 8, [7, 1], 7, 7.001, 1)
        legacy = self.ns["old_strict_volume_result"](raw, result["legacy_strict_result"])
        self.assertEqual(legacy["state"], "failed")
        self.assertEqual(legacy["relative_tolerance"], 1e-8)
        self.assertEqual(legacy["body_volume_arithmetic"]["effective_tolerance_m3"], 1e-7)
        self.assertTrue(legacy["new_interval_pass_does_not_change_this_result"])

    def test_compatible_interval_cannot_override_failed_or_missing_geometry(self):
        volume = self.evaluate(self.case())
        self.assertEqual(volume["state"], "passed")
        self.assertEqual(self.ns["domain_evidence_state"](volume, [{"state": "failed"}]), "failed")
        self.assertEqual(self.ns["domain_evidence_state"](volume, [{"state": "blocked"}]), "blocked")
        self.assertEqual(self.ns["domain_evidence_state"](volume, []), "blocked")

    def test_missing_cavity_is_rejected_even_when_its_volume_fits_error_interval(self):
        case = self.case(cavities=(1e-6,))
        case = (case[0], [], case[2], case[3])
        result = self.evaluate(case)
        self.assertEqual(result["state"], "blocked")
        self.assertIn("cavity", result["reason"].lower())

    def test_duplicate_or_nonpositive_cavity_reading_is_rejected(self):
        case = self.case(cavities=(0.5, 0.5))
        case[1][1]["piece_index"] = case[1][0]["piece_index"]
        self.assertEqual(self.evaluate(case)["state"], "blocked")
        zero = self.case(cavities=(0.0,))
        self.assertEqual(self.evaluate(zero)["state"], "blocked")

    def test_unknown_nonfinite_or_over_one_percent_single_error_is_blocked(self):
        for relative in (0.0100001, float("nan"), float("inf"), -0.001, None):
            case = self.case()
            mass = case[0]["main_after_imprint"]["mass_properties"]
            mass["relative_accuracy_reported"] = relative
            mass["estimated_absolute_error_m3"] = None if relative is None else 7.0 * relative
            with self.subTest(relative=relative):
                self.assertEqual(self.evaluate(case)["state"], "blocked")

    def test_combined_one_percent_cap_is_separate_from_single_body_cap(self):
        result = self.evaluate(self.case(relative=0.006))
        self.assertEqual(result["state"], "blocked")
        self.assertTrue(any(row["combined_relative_uncertainty"] > 0.01 for row in result["interval_checks"]))
        self.assertEqual(self.evaluate(self.case(relative=0.005))["state"], "passed")

    def test_one_ulp_cap_roundoff_does_not_turn_into_a_wider_cap(self):
        cap = 0.01
        ulp = math.ldexp(1.0, math.frexp(cap)[1] - 53)
        exact = {"label": "exact", "volume_at_unit_density_m3": 1.0, "estimated_absolute_error_m3": 0.0}
        for ulps, expected in ((0, "passed"), (1, "passed"), (2, "blocked")):
            measured = {"label": "measured", "volume_at_unit_density_m3": 1.0, "estimated_absolute_error_m3": cap + ulps * ulp}
            result = self.ns["conservation_interval"]([(1, measured), (-1, exact)], 1.0, "roundoff_boundary")
            with self.subTest(ulps=ulps):
                self.assertEqual(result["state"], expected)
                self.assertEqual(result["maximum_combined_relative_uncertainty"], 0.01)
                self.assertEqual(result["cap_comparison_roundoff_allowance_m3"], ulp)

    def test_interval_excluding_zero_is_failed_consistency_not_a_shape_certificate(self):
        case = self.case(relative=0.0001)
        case[0]["main_after_imprint"] = self.snapshot(6.5, 0.0001, index=0)
        result = self.evaluate(case)
        self.assertEqual(result["state"], "failed")
        self.assertTrue(result["requires_separate_geometry_acceptance"])

    def test_verify_recomputes_saved_mass_and_cavities_instead_of_reading_pass(self):
        snapshots, cavities, pieces, box = self.case()
        reference = {"state": "passed", "snapshots": snapshots, "cavity_snapshots": cavities,
                     "piece_records": pieces, "outer_piece_index": 0, "analytic_enclosure_m3": box}
        built = {"pieces": pieces, "volume_acceptance": {"state": "passed"}}
        actual = self.snapshot(6.0, index=0)
        self.ns["native_volume_snapshot"] = lambda shape: dict(actual)
        result = self.ns["verify_native_domain_volume"](object(), object(), built, reference)
        self.assertEqual(result["state"], "failed")
        self.assertTrue(result["cavities_independently_regenerated"])
        self.assertEqual(result["saved_fluid_snapshot"]["volume_m3"], 6.0)
        built["pieces"] = pieces[:1]
        result = self.ns["verify_native_domain_volume"](object(), object(), built, reference)
        self.assertEqual(result["state"], "blocked")

    def test_readonly_replay_of_previous_real_scalars_does_not_certify_geometry(self):
        path = STAGE / "runs/cad_build/attempt_003/build_report.json"
        if not path.is_file():
            self.skipTest("Optional previous real scalar evidence is not present")
        before = cad.sha256(path)
        prior = json.loads(path.read_text(encoding="utf-8"))
        for domain in ("background", "arm"):
            part = prior["domains"][domain]
            snapshots, pieces = part["volume_snapshots"], part["pieces"]
            cavities = []
            outer = None
            for index, piece in enumerate(pieces):
                piece["piece_index"] = index
                piece["native_volume_snapshot"]["piece_index"] = index
                if piece["reaches_all_box_sides"]:
                    outer = index
                else:
                    cavities.append(piece["native_volume_snapshot"])
            snapshots["main_before_imprint"]["piece_index"] = outer
            snapshots["main_after_imprint"]["piece_index"] = outer
            box = self.ns["analytic_box_volume"](part["box_lo_m"], part["box_hi_m"])
            result = self.evaluate((snapshots, cavities, pieces, box))
            with self.subTest(domain=domain):
                self.assertEqual(result["state"], "passed", result.get("reason"))
                self.assertTrue(result["requires_separate_geometry_acceptance"])
                self.assertNotEqual(result["legacy_strict_result"]["state"], "passed")
        self.assertEqual(cad.sha256(path), before)

    def test_missing_calibration_and_bad_analytic_enclosure_are_blocked(self):
        self.assertEqual(self.evaluate(self.case(), calibration=False)["state"], "blocked")
        case = self.case()
        self.assertEqual(self.evaluate((case[0], case[1], case[2], 0.0))["state"], "blocked")
        with self.assertRaises(ValueError):
            self.ns["analytic_box_volume"]([0, 0, 0], [1, 0, 1])


class GeometryFallbackTests(unittest.TestCase):
    def setUp(self):
        self.ns = {"math": math, "TOL_OWNER": 1e-5,
                   "report": {"measurement_protocol": {"validated": True, "definition": {"mandatory_geometric_evidence": {
                       "fallback_projection_tolerance_m": 1e-5, "finite_sampling_not_continuous_proof": True}}}}}
        exec(template_functions("sc_common.py.in", "u", "finite", "xyz", "dist", "as_point", "BlockedEvidence"), self.ns)
        exec(template_functions("sc_domains.py.in", "geometry_surface_fallback_authorized", "surface_face_samples",
                                "point_in_padded_bbox", "surface_index", "surface_candidates", "directed_surface_check",
                                "sampled_surface_equivalence", "geometry_equivalence", "domain_evidence_state"), self.ns)
        self.ns.update({"bbox": lambda entity: entity.bounds, "samples": lambda face: face.points,
                        "shape_record": lambda shape: shape.record,
                        "difference_probe": lambda a, b, label: {"state": "failed", "label": label, "error": "General Failure."}})

    def cube(self, damaged_face=False):
        def point(coordinates):
            return SimpleNamespace(X=coordinates[0], Y=coordinates[1], Z=coordinates[2])
        class Face:
            Area = 1.0
            def __init__(self, axis, side):
                self.axis, self.side = axis, side
                self.bounds = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]
                self.bounds[0][axis] = side
                self.bounds[1][axis] = side
                other = [k for k in range(3) if k != axis]
                self.points = []
                for p, q in ((0.2, 0.2), (0.8, 0.2), (0.2, 0.8)):
                    values = [0.0, 0.0, 0.0]
                    values[axis], values[other[0]], values[other[1]] = side, p, q
                    self.points.append(point(values))
            def ProjectPoint(self, p):
                values = [p.X, p.Y, p.Z]
                values = [min(1.0, max(0.0, value)) for value in values]
                values[self.axis] = self.side + (0.01 if damaged_face and self.axis == 2 and self.side == 1 else 0.0)
                return SimpleNamespace(Point=point(values))
        faces = [Face(axis, side) for axis in range(3) for side in (0.0, 1.0)]
        bounds = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]
        record = {"volume_m3": 1.0, "surface_area_m2": 6.0, "bbox_m": bounds, "faces": 6,
                  "topology": {"state": "passed", "closed": True, "manifold": True, "piece_count": 1}}
        return SimpleNamespace(Faces=faces, bounds=bounds, record=record)

    def test_general_failure_is_retained_when_complete_surface_fallback_passes(self):
        result = self.ns["geometry_equivalence"](self.cube(), self.cube(), "material")
        self.assertEqual(result["state"], "passed")
        self.assertTrue(result["fallback_used"])
        self.assertEqual(result["native_symmetric_difference"]["state"], "failed")
        self.assertEqual(result["native_symmetric_difference"]["a_minus_b"]["error"], "General Failure.")
        self.assertTrue(result["finite_sampling_not_continuous_proof"])
        surface = result["surface_fallback"]
        self.assertTrue(surface["all_faces_checked_in_both_directions"])
        self.assertEqual(len(surface["a_to_b"]["face_results"]), 6)
        self.assertLess(surface["a_to_b"]["projection_calls"], 6 * 6 * 3)

    def test_compatible_interval_does_not_rescue_different_geometry(self):
        geometry = self.ns["geometry_equivalence"](self.cube(), self.cube(damaged_face=True), "material")
        self.assertEqual(geometry["state"], "blocked")
        self.assertEqual(geometry["surface_fallback"]["state"], "failed")
        self.assertTrue(geometry["fallback_attempted"])
        self.assertFalse(geometry["fallback_used"])
        self.assertEqual(self.ns["domain_evidence_state"]({"state": "passed"}, [geometry]), "blocked")
        self.assertEqual(geometry["native_symmetric_difference"]["a_minus_b"]["state"], "failed")

    def test_unknown_failure_or_unbound_fallback_never_implies_equality(self):
        self.ns["difference_probe"] = lambda a, b, label: {"state": "failed", "error": "Out of memory"}
        result = self.ns["geometry_equivalence"](self.cube(), self.cube(), "material")
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["fallback_used"])
        self.ns["difference_probe"] = lambda a, b, label: {"state": "failed", "error": "General Failure"}
        self.ns["report"]["measurement_protocol"]["validated"] = False
        result = self.ns["geometry_equivalence"](self.cube(), self.cube(), "material")
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["fallback_used"])

    def test_native_empty_differences_are_preferred_and_nonempty_is_not_overridden(self):
        self.ns["difference_probe"] = lambda a, b, label: {"state": "passed", "empty": True}
        result = self.ns["geometry_equivalence"](self.cube(), self.cube(), "material")
        self.assertEqual(result["state"], "passed")
        self.assertFalse(result["fallback_used"])
        self.ns["difference_probe"] = lambda a, b, label: {"state": "passed", "empty": False}
        result = self.ns["geometry_equivalence"](self.cube(), self.cube(), "material")
        self.assertEqual(result["state"], "failed")
        self.assertFalse(result["fallback_used"])

    def test_projection_budget_or_missing_face_samples_are_blocked(self):
        result = self.ns["directed_surface_check"](self.cube(), self.cube(), projection_budget=1)
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["all_source_faces_sampled"])
        source = self.cube()
        source.Faces[0].points = []
        result = self.ns["directed_surface_check"](source, self.cube())
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["all_source_faces_sampled"])

    def test_component_or_bbox_mismatch_rejects_fallback(self):
        other = self.cube()
        other.record["topology"]["piece_count"] = 2
        result = self.ns["geometry_equivalence"](self.cube(), other, "material")
        self.assertEqual(result["state"], "failed")
        other = self.cube()
        other.record["bbox_m"] = [[0, 0, 0], [1, 1, 1.1]]
        result = self.ns["geometry_equivalence"](self.cube(), other, "material")
        self.assertEqual(result["state"], "blocked")
        self.assertEqual(result["surface_fallback"]["state"], "failed")


class FrozenMeasurementReferenceTests(unittest.TestCase):
    def setUp(self):
        paths = {"measurement_protocol": STAGE / "frozen/measurement_protocol_v2.json",
                 "calibration_reference": STAGE / "runs/cad_calibration/attempt_002/calibration_report.json",
                 "additional_authorization": STAGE / "frozen/diagnostic_authorization_001.json"}
        if not all(path.is_file() for path in paths.values()):
            self.skipTest("Optional frozen local v2 authorization/calibration evidence is unavailable")
        self.paths = paths
        self.before = {str(path): cad.sha256(path) for path in paths.values()}
        cfg = {name: {"path": str(path), "sha256": cad.sha256(path)} for name, path in paths.items()}
        cfg.update({"mode": "build", "source_sha256": cad.sha256(cad.WORKSPACE_ROOT / "简化模型_导出/AUVM.step")})
        self.ns = {"CFG": cfg, "report": {}, "codecs": codecs, "json": json, "traceback": traceback,
                   "TOL_OWNER": 1e-5, "VOLUME_ABSOLUTE_TOLERANCE_M3": 1e-10, "VOLUME_RELATIVE_TOLERANCE": 1e-8,
                   "MAXIMUM_SINGLE_RELATIVE_ACCURACY": 0.01, "MAXIMUM_COMBINED_RELATIVE_UNCERTAINTY": 0.01,
                   "inside_stage": lambda path: STAGE in Path(path).resolve().parents,
                   "sha256": lambda path: cad.sha256(Path(path)), "save": lambda: None}
        exec(template_functions("sc_common.py.in", "u", "BlockedEvidence"), self.ns)
        exec(template_functions("sc_volume.py.in", "read_bound_measurement_reference", "use_frozen_measurement_calibration",
                                "run_volume_calibration"), self.ns)
        self.called = []
        def unauthorized_repeat():
            self.called.append(True)
            raise AssertionError("Full build must not repeat a consumed small-diagnostic budget")
        self.ns["volume_calibration"] = unauthorized_repeat

    def tearDown(self):
        for path, digest in getattr(self, "before", {}).items():
            self.assertEqual(cad.sha256(Path(path)), digest)

    def test_actual_passed_reference_is_bound_without_repeating_controls(self):
        result = self.ns["run_volume_calibration"]()
        self.assertEqual(result["state"], "passed", result.get("errors"))
        self.assertTrue(self.ns["report"]["measurement_protocol"]["validated"])
        self.assertFalse(result["full_domain_precision_proven"])
        self.assertTrue(result["consistency_only_no_absolute_accuracy_certificate"])
        self.assertEqual(self.called, [])

    def test_hash_mismatch_blocks_and_clears_any_stale_protocol_validation(self):
        self.ns["run_volume_calibration"]()
        self.ns["CFG"]["calibration_reference"]["sha256"] = "0" * 64
        result = self.ns["run_volume_calibration"]()
        self.assertEqual(result["state"], "blocked")
        self.assertFalse(result["native_accuracy_calibrated"])
        self.assertFalse(self.ns["report"]["measurement_protocol"]["validated"])
        self.assertEqual(self.called, [])

    def test_missing_authorization_never_triggers_automatic_control_rerun(self):
        self.ns["CFG"].pop("additional_authorization")
        result = self.ns["run_volume_calibration"]()
        self.assertEqual(result["state"], "blocked")
        self.assertEqual(self.called, [])


if __name__ == "__main__":
    unittest.main()

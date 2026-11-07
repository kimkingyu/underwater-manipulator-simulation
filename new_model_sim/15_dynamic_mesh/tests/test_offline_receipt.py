"""Regression for mixed unittest/pytest reporting; nested pytest execution is mocked."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import verify_offline as verifier


class OfflineReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "receipt.json"
        self.record = None

    def execute(self, kind="passed", changed=False):
        def fake_main(args, plugins):
            collector = plugins[0]
            names = ["legacy_unittest", "parameterized_pytest[value]"]
            collector.pytest_collection_finish(SimpleNamespace(items=[SimpleNamespace(nodeid=name) for name in names]))
            for name in names:
                if kind == "unexecuted" and name == names[-1]:
                    continue
                failure = name == names[-1] and kind == "failed"
                skipped = name == names[-1] and kind == "skipped"
                collector.pytest_runtest_logreport(SimpleNamespace(
                    nodeid=name, when="call", passed=not failure and not skipped,
                    failed=failure, skipped=skipped, longrepr="fixture reason" if failure or skipped else None))
            if kind == "collection_error":
                collector.pytest_collectreport(SimpleNamespace(nodeid="bad.py", failed=True, skipped=False, longrepr="syntax error"))
            return 0 if kind in ("passed", "unexecuted", "skipped") else 1

        def saved(path, value):
            self.record = value

        snapshots = [{"module.py": "before"}, {"module.py": "after" if changed else "before"}]
        with mock.patch.object(verifier, "within", side_effect=lambda path, _: Path(path)), \
             mock.patch.object(verifier, "source_snapshot", side_effect=snapshots), \
             mock.patch.object(verifier, "write_new_json", side_effect=saved), \
             mock.patch.object(verifier.pytest, "main", side_effect=fake_main), \
             contextlib.redirect_stdout(io.StringIO()):
            return verifier.run(self.path)

    def test_mixed_cases_are_counted_and_recorded(self):
        self.assertEqual(self.execute(), 0)
        self.assertEqual(self.record["test_count"], 2)
        self.assertEqual(self.record["executed_count"], 2)
        self.assertEqual(self.record["passed_test_count"], 2)
        self.assertIn("parameterized_pytest[value]", self.record["collected_test_ids"])

    def test_skipped_case_never_passes(self):
        self.assertEqual(self.execute("skipped"), 1)
        self.assertEqual(self.record["skipped_count"], 1)

    def test_unexecuted_case_never_passes(self):
        self.assertEqual(self.execute("unexecuted"), 1)
        self.assertEqual(self.record["executed_count"], 1)

    def test_failure_has_source_bound_receipt(self):
        self.assertEqual(self.execute("failed"), 1)
        self.assertEqual(self.record["failure_count"], 1)
        self.assertTrue(self.record["source_unchanged_during_run"])

    def test_collection_error_cannot_pass(self):
        self.assertEqual(self.execute("collection_error"), 1)
        self.assertEqual(self.record["error_count"], 1)

    def test_source_change_cannot_pass(self):
        self.assertEqual(self.execute(changed=True), 1)
        self.assertFalse(self.record["source_unchanged_during_run"])

    def test_existing_report_never_overwritten(self):
        self.path.write_text("keep", encoding="utf-8")
        with mock.patch.object(verifier, "within", return_value=self.path), self.assertRaises(FileExistsError):
            verifier.run(self.path)
        self.assertEqual(self.path.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()

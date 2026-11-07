import ast
import builtins
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import run_mesh_job


class WorkbenchFailureExitTests(unittest.TestCase):
    def logger(self, fail_write=False):
        writes = []
        def append(path, text, encoding):
            if fail_write:
                raise OSError("synthetic log write failure")
            writes.append(text)
        def ascii_str(value):
            text = builtins.str(value)
            text.encode("ascii")
            return text
        ns = {"R": {"exception": "原始后端错误"}, "TRACE": "not_written.log",
              "File": SimpleNamespace(AppendAllText=append),
              "System": SimpleNamespace(Environment=SimpleNamespace(NewLine="\n")),
              "Encoding": SimpleNamespace(UTF8="UTF8"),
              "unicode": builtins.str, "str": ascii_str}
        source = (STAGE / "mesh/wb_stage15.wbjn.in").read_text(encoding="utf-8")
        tree = ast.parse(source)
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ("native_text", "log")]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), "wb_logger", "exec"), ns)
        return ns, writes

    def test_unicode_failure_text_does_not_use_python2_ascii_coercion(self):
        ns, writes = self.logger()
        ns["log"]("失败：无法在网格划分中初始化插件")
        self.assertEqual(writes, ["失败：无法在网格划分中初始化插件\n"])
        self.assertNotIn("logging_errors", ns["R"])

    def test_log_write_failure_does_not_replace_primary_exception(self):
        ns, writes = self.logger(fail_write=True)
        ns["log"]("失败：原始后端错误")
        self.assertEqual(ns["R"]["exception"], "原始后端错误")
        self.assertEqual(len(ns["R"]["logging_errors"]), 1)
        self.assertFalse(writes)

    def test_failure_receipt_is_detected_without_waiting_for_process_exit(self):
        with tempfile.TemporaryDirectory(prefix="_test_wb_failure_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "receipt.json"
            path.write_text(json.dumps({"status": "failed", "exception": "后端通信失败"}, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(run_mesh_job.RuntimeGateError, "后端通信失败"):
                run_mesh_job.observe_completion(path)

    def test_partial_json_is_not_success(self):
        with tempfile.TemporaryDirectory(prefix="_test_wb_partial_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "receipt.json"
            self.assertIsNone(run_mesh_job.observe_completion(path))
            path.write_text('{"status":', encoding="utf-8")
            self.assertIsNone(run_mesh_job.observe_completion(path))

    def test_successful_journal_receipt_is_not_mesh_validation(self):
        with tempfile.TemporaryDirectory(prefix="_test_wb_receipt_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "receipt.json"
            path.write_text(json.dumps({"status": "completed_pending_mesh_inventory", "valid": False}), encoding="utf-8")
            observed = run_mesh_job.observe_completion(path)
            self.assertFalse(observed["valid"])
            self.assertEqual(observed["status"], "completed_pending_mesh_inventory")

    def test_nonobject_receipt_rejected(self):
        with tempfile.TemporaryDirectory(prefix="_test_wb_badreceipt_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "receipt.json"
            path.write_text("[]", encoding="utf-8")
            with self.assertRaises(run_mesh_job.RuntimeGateError):
                run_mesh_job.observe_completion(path)


if __name__ == "__main__":
    unittest.main()

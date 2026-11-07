"""Offline-only tests: fake Popen/connection, never import or start Fluent."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runtime


class FakeProcess:
    pid = 12345
    returncode = None

    def poll(self):
        return self.returncode


class FakePsutil:
    class NoSuchProcess(Exception):
        pass

    def __init__(self):
        self.alive = True
        self.killed = []
        owner = self

        class ProcessInfo:
            pid = 12345

            def create_time(self):
                return 100.0

            def children(self, recursive=False):
                return []

            def is_running(self):
                return owner.alive

            def terminate(self):
                owner.killed.append(self.pid)
                owner.alive = False

            def kill(self):
                self.terminate()
        self.item = ProcessInfo()

    def Process(self, pid):
        if pid != 12345:
            raise self.NoSuchProcess()
        return self.item

    def wait_procs(self, processes, timeout):
        return ([p for p in processes if not p.is_running()], [p for p in processes if p.is_running()])


class FakeSession:
    def __init__(self):
        self.exit_calls = []

    def get_fluent_version(self):
        return "24.2.0"

    def exit(self, **kwargs):
        self.exit_calls.append(kwargs)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.contract = copy.deepcopy(runtime.load_contract())
        self.contract["budget"]["minimum_free_disk_gib"] = 0
        self.contract["budget"]["fluent_gate_timeout_s"] = 30

    def tearDown(self):
        self.temp.cleanup()

    def attempt(self, name="one"):
        path = runtime.create_attempt(self.root / name, stage_root=self.root)
        runtime.snapshot_inputs(path, inputs=[runtime.STAGE_ROOT / "contract.json"])
        return path

    def test_environment_does_not_mutate_parent_and_expands_registry(self):
        before = dict(os.environ)
        base = {"Path": "shell", "USERPROFILE": "C:/Users/person", "SystemRoot": "C:/Windows"}
        original = dict(base)
        env = runtime.full_environment(base, registry=({"ROOT": "D:/tools", "Path": "%ROOT%/bin"}, {"Path": "C:/user/bin"}))
        self.assertEqual(env["PATH"], "D:/tools/bin;C:/user/bin")
        self.assertEqual(base, original)
        self.assertEqual(dict(os.environ), before)
        self.assertIn("COMSPEC", env)

    def test_user_path_self_reference_expands_machine_path_once(self):
        env = runtime.full_environment({}, registry=({"Path": "D:/machine"}, {"Path": "%PATH%;D:/user"}))
        self.assertEqual(env["PATH"], "D:/machine;D:/user")

    def test_pid_reuse_never_terminates_unrelated_process(self):
        fake = FakePsutil()
        tree = runtime.OwnedProcessTree(FakeProcess(), fake)
        fake.item.create_time = lambda: 101.0
        self.assertTrue(tree.close())
        self.assertEqual(fake.killed, [])

    def test_early_launcher_exit_does_not_adopt_unknown_pid(self):
        fake = FakePsutil()
        process = FakeProcess()
        process.returncode = 0
        tree = runtime.OwnedProcessTree(process, fake)
        self.assertFalse(tree.close())
        self.assertEqual(fake.killed, [])
        self.assertTrue(tree.errors)

    def test_expand_is_case_insensitive(self):
        self.assertEqual(runtime.expand_environment("%root%/%MISSING%", {"Root": "D:"}), "D:/%MISSING%")

    def test_attempt_never_reuses_empty_directory_or_escapes_stage(self):
        self.attempt()
        with self.assertRaises(FileExistsError):
            runtime.create_attempt(self.root / "one", stage_root=self.root)
        with self.assertRaises(runtime.RuntimeGateError):
            runtime.create_attempt(self.root.parent / "outside_attempt", stage_root=self.root)
        with self.assertRaises(runtime.RuntimeGateError):
            runtime.create_attempt(self.root, stage_root=self.root)

    def test_lock_blocks_parallel_and_does_not_remove_foreign_lock(self):
        with runtime.AnsysTaskLock(self.root / "one", stage_root=self.root):
            with self.assertRaises(runtime.RuntimeGateError):
                with runtime.AnsysTaskLock(self.root / "two", stage_root=self.root):
                    self.fail("second lock acquired")
        self.assertFalse((self.root / ".ansys-task.lock").exists())

    def test_uncertain_ownership_retains_lock(self):
        with runtime.AnsysTaskLock(self.root / "one", stage_root=self.root) as lock:
            lock.retain("unverified cleanup")
        self.assertTrue((self.root / ".ansys-task.lock").exists())

    def test_command_is_single_process_double_and_no_shell(self):
        command = runtime.fluent_command(self.contract, self.root, 1)
        self.assertIn("3ddp", command)
        self.assertIn("-t1", command)
        self.assertNotIn("start", command)
        for count in (0, 2, True):
            with self.assertRaises(runtime.RuntimeGateError):
                runtime.fluent_command(self.contract, self.root, count)

    def test_launch_once_owned_exit_and_credential_cleanup(self):
        attempt = self.attempt()
        called, session, psutil = [], FakeSession(), FakePsutil()

        def popen(command, **kwargs):
            called.append((command, kwargs))
            (attempt / "server-info.txt").write_text("127.0.0.1:1234\nPRIVATE_PASSWORD", encoding="utf-8")
            return FakeProcess()

        before = dict(os.environ)
        with runtime.launch(attempt, self.contract, popen=popen, connect=lambda **kw: session,
                            psutil_module=psutil, stage_root=self.root) as (owned, report):
            self.assertIs(owned, session)
            self.assertFalse(report["valid"])
        self.assertEqual(len(called), 1)
        self.assertEqual(called[0][1]["cwd"], str(attempt))
        self.assertFalse(called[0][1]["shell"])
        self.assertEqual(dict(os.environ), before)
        self.assertEqual(len(session.exit_calls), 1)
        self.assertFalse((attempt / "server-info.txt").exists())
        data = (attempt / "runtime.json").read_text(encoding="utf-8")
        self.assertNotIn("PRIVATE_PASSWORD", data)
        self.assertEqual(json.loads(data)["automatic_retries"], 0)
        self.assertEqual(psutil.killed, [12345])
        self.assertFalse((self.root / ".ansys-task.lock").exists())

    def test_none_session_is_failure_and_no_retry(self):
        attempt = self.attempt()
        calls = []

        def popen(*args, **kwargs):
            calls.append(args)
            (attempt / "server-info.txt").write_text("127.0.0.1:1234\nsecret", encoding="utf-8")
            return FakeProcess()

        with self.assertRaisesRegex(runtime.RuntimeGateError, "returned None"):
            with runtime.launch(attempt, self.contract, popen=popen, connect=lambda **kw: None,
                                psutil_module=FakePsutil(), stage_root=self.root):
                self.fail("None connection accepted")
        self.assertEqual(len(calls), 1)
        self.assertEqual(json.loads((attempt / "runtime.json").read_text())["status"], "failed")

    def test_timeout_never_relaunches(self):
        attempt, calls = self.attempt(), []

        def popen(*args, **kwargs):
            calls.append(args)
            return FakeProcess()

        with self.assertRaises(TimeoutError):
            with runtime.launch(attempt, self.contract, popen=popen, connect=lambda **kw: FakeSession(),
                                psutil_module=FakePsutil(), stage_root=self.root, start_timeout=0):
                self.fail("timeout accepted")
        self.assertEqual(len(calls), 1)

    def test_second_launch_refused_before_external_call(self):
        attempt = self.attempt()
        (attempt / "runtime.json").write_text("{}")
        with patch("runtime.subprocess.Popen") as spawn:
            with self.assertRaises(FileExistsError):
                with runtime.launch(attempt, self.contract, connect=lambda **kw: FakeSession(),
                                    psutil_module=FakePsutil(), stage_root=self.root):
                    pass
            spawn.assert_not_called()

    def test_popen_exception_recorded_and_lock_removed_without_spawn(self):
        attempt = self.attempt()
        with self.assertRaisesRegex(OSError, "fixture failed"):
            with runtime.launch(attempt, self.contract, popen=lambda *a, **k: (_ for _ in ()).throw(OSError("fixture failed")),
                                connect=lambda **kw: FakeSession(), psutil_module=FakePsutil(), stage_root=self.root):
                pass
        result = json.loads((attempt / "runtime.json").read_text())
        self.assertIn("fixture failed", result["exception"]["message"])
        self.assertFalse(result["valid"])


if __name__ == "__main__":
    unittest.main()

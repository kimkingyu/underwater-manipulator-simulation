"""Stage-15 isolated, single-launch runtime. Importing this module launches nothing.

The installed PyFluent standalone launcher retries on Windows after a timeout.
Use one explicit Popen and connect only to its fresh server-info file instead.
The lock is deliberately retained if process ownership/cleanup cannot be proved.
"""
from __future__ import annotations

import contextlib
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
import traceback
import uuid

STAGE_ROOT = Path(__file__).resolve().parent


class RuntimeGateError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path):
    path = Path(path).resolve(strict=True)
    if not path.is_file() or path.stat().st_size <= 0:
        raise RuntimeGateError("required nonempty regular file: " + str(path))
    return {"path": str(path), "size": path.stat().st_size, "sha256": sha256(path)}


def json_safe(data):
    # Invalid observed numbers must remain visible in failure reports rather than
    # causing the reporter itself to lose the original solver exception.
    if isinstance(data, float) and not math.isfinite(data):
        return {"invalid_nonfinite_number": repr(data)}
    if isinstance(data, dict):
        return {str(k): json_safe(v) for k, v in data.items()}
    if isinstance(data, (list, tuple)):
        return [json_safe(v) for v in data]
    if data is None or isinstance(data, (str, int, float, bool)):
        return data
    return repr(data)


def write_json_new(path, data):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(json_safe(data), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def load_contract(path=None):
    path = Path(path or STAGE_ROOT / "contract.json").resolve(strict=True)
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if data["software"]["product_version"] != "24.2" or data["software"]["precision"] != "double":
        raise RuntimeGateError("only Fluent 24.2 double is authorized")
    if data["budget"]["automatic_solver_retries"] != 0:
        raise RuntimeGateError("automatic retries must be disabled")
    return data


def _inside(path, root):
    try:
        path.relative_to(root)
        return path != root
    except ValueError:
        return False


def create_attempt(path, *, stage_root=STAGE_ROOT):
    root = Path(stage_root).resolve(strict=True)
    raw = Path(path).absolute()
    resolved = raw.resolve()
    if not _inside(resolved, root):
        raise RuntimeGateError("attempt must be a new directory inside stage15")
    # Reject existing directories, including empty attempts, and symlink escapes.
    if raw.exists() or raw.is_symlink():
        raise FileExistsError("refusing to reuse attempt: " + str(raw))
    for ancestor in raw.parents:
        if ancestor == root:
            break
        if ancestor.is_symlink():
            raise RuntimeGateError("symlinked attempt ancestor is not permitted")
    resolved.mkdir(parents=True, exist_ok=False)
    return resolved


def expand_environment(value, env):
    lookup = {k.upper(): v for k, v in env.items()}
    return re.sub(r"%([^%]+)%", lambda m: lookup.get(m.group(1).upper(), m.group(0)), str(value))


def _registry_environment():
    import winreg
    result = []
    for root, key in ((winreg.HKEY_LOCAL_MACHINE,
                       r"SYSTEM\CurrentControlSet\Control\Session Manager\Environment"),
                      (winreg.HKEY_CURRENT_USER, "Environment")):
        values = {}
        try:
            with winreg.OpenKey(root, key) as handle:
                index = 0
                while True:
                    try:
                        name, value, kind = winreg.EnumValue(handle, index)
                    except OSError:
                        break
                    index += 1
                    if kind in (winreg.REG_SZ, winreg.REG_EXPAND_SZ):
                        values[name.upper()] = str(value)
        except FileNotFoundError:
            pass
        result.append(values)
    return result


def full_environment(base=None, *, attempt=None, registry=None, contract=None):
    """Return a child-only environment; never mutate os.environ or the registry."""
    base = dict(os.environ if base is None else base)
    env = {str(k).upper(): str(v) for k, v in base.items()}
    if registry is None:
        registry = _registry_environment() if os.name == "nt" else ()
    profile = env.get("USERPROFILE")
    if profile:
        for key, suffix in (("APPDATA", "AppData/Roaming"), ("LOCALAPPDATA", "AppData/Local")):
            env.setdefault(key, str(Path(profile) / suffix))
    root = env.get("SYSTEMROOT")
    if root:
        env.setdefault("COMSPEC", str(Path(root) / "System32/cmd.exe"))
        env.setdefault("WINDIR", root)
        env.setdefault("SYSTEMDRIVE", root[:2])
    for index, values in enumerate(registry):
        normalized = {k.upper(): str(v) for k, v in values.items()}
        old_path = env.get("PATH", "")
        env.update(normalized)
        if "PATH" in normalized:
            if "%PATH%" in normalized["PATH"].upper():
                path_context = dict(env, PATH=old_path)
                env["PATH"] = expand_environment(normalized["PATH"], path_context)
            elif index > 0:
                env["PATH"] = old_path + (";" if old_path else "") + normalized["PATH"]
    for _ in range(8):
        expanded = {k: expand_environment(v, env) for k, v in env.items()}
        if expanded == env:
            break
        env = expanded
    env.setdefault("NUMBER_OF_PROCESSORS", str(os.cpu_count() or 1))
    if contract:
        env["AWP_ROOT242"] = contract["software"]["ansys_root"]
        env.setdefault("ANSYSLMD_LICENSE_FILE", contract["software"]["license_server"])
    if attempt is not None:
        temp = Path(attempt).resolve() / "tmp"
        temp.mkdir(exist_ok=False)
        env["TEMP"] = env["TMP"] = str(temp)
        env["FLUENT_STARTUP_DIRECTORY"] = str(Path(attempt).resolve())
    return env


class AnsysTaskLock:
    """One advisory lock shared by stage15 CAD, Workbench and Fluent callers."""
    def __init__(self, attempt, *, stage_root=STAGE_ROOT):
        self.path = Path(stage_root) / ".ansys-task.lock"
        self.token = uuid.uuid4().hex
        self.attempt = str(Path(attempt).resolve())
        self.acquired = False
        self.retain_reason = None

    def __enter__(self):
        try:
            write_json_new(self.path, {"pid": os.getpid(), "token": self.token,
                                      "attempt": self.attempt, "created_unix": time.time()})
        except FileExistsError as exc:
            raise RuntimeGateError("another ANSYS task or uncleared ownership lock exists: " + str(self.path)) from exc
        self.acquired = True
        return self

    def retain(self, reason):
        self.retain_reason = str(reason)

    def __exit__(self, *_):
        if self.acquired and not self.retain_reason:
            current = json.loads(self.path.read_text(encoding="utf-8"))
            if current.get("token") != self.token:
                raise RuntimeGateError("task lock ownership changed; refusing to remove it")
            self.path.unlink()


class OwnedProcessTree:
    """Only PID+creation-time pairs descended from the Popen process are touched."""
    def __init__(self, process, psutil_module):
        self.process = process
        self.psutil = psutil_module
        self.members = {}
        self.errors = []
        if process.poll() is not None:
            self.errors.append("launcher exited before initial PID ownership was captured")
        else:
            try:
                root = self.psutil.Process(process.pid)
                self.members[root.pid] = root.create_time()
                self.capture()
            except Exception as exc:
                self.errors.append("initial ownership unavailable: " + repr(exc))

    def capture(self):
        # Track descendants of every already-owned surviving member, even after
        # the original launcher exits. Never adopt a reused PID as a new root.
        for pid, created in list(self.members.items()):
            try:
                root = self.psutil.Process(pid)
                if root.create_time() != created:
                    continue
                for item in root.children(recursive=True):
                    stamp = item.create_time()
                    if item.pid in self.members and self.members[item.pid] != stamp:
                        self.errors.append("owned descendant PID was reused: " + str(item.pid))
                        continue
                    self.members[item.pid] = stamp
            except self.psutil.NoSuchProcess:
                pass
            except Exception as exc:
                self.errors.append(repr(exc))

    def live(self):
        found = []
        for pid, created in list(self.members.items()):
            try:
                item = self.psutil.Process(pid)
                if item.create_time() == created and item.is_running():
                    found.append(item)
            except self.psutil.NoSuchProcess:
                pass
        return found

    def close(self):
        self.capture()
        for item in reversed(self.live()):
            try:
                item.terminate()
            except self.psutil.NoSuchProcess:
                pass
        _, alive = self.psutil.wait_procs(self.live(), timeout=8)
        for item in alive:
            if item.create_time() == self.members.get(item.pid):
                item.kill()
        self.psutil.wait_procs(alive, timeout=5)
        return bool(self.members) and not self.live() and not self.errors


def fluent_command(contract, attempt, processors):
    if isinstance(processors, bool) or not isinstance(processors, int) or processors != contract["budget"]["probe_processor_count"]:
        raise RuntimeGateError("static/API probes require the contract's single-process count")
    if contract["software"]["product_version"] != "24.2" or contract["software"]["precision"] != "double":
        raise RuntimeGateError("Fluent version/precision mismatch")
    return [contract["software"]["fluent"], "3ddp", "-g", "-t%d" % processors,
            "-sifile=" + str(Path(attempt) / "server-info.txt"), "-nm"]


def snapshot_inputs(attempt, inputs=(), sources=(), settings=None):
    manifest = {"created_unix": time.time(), "cwd": str(Path(attempt).resolve()),
                "python": sys.executable, "python_version": sys.version,
                "inputs": [file_record(p) for p in inputs],
                "sources": [file_record(p) for p in sources], "settings": settings or {},
                "status": "not_run", "valid": False}
    write_json_new(Path(attempt) / "inputs.json", manifest)
    return manifest


@contextlib.contextmanager
def launch(attempt, contract, *, processors=1, popen=None, connect=None,
           psutil_module=None, stage_root=STAGE_ROOT, start_timeout=300):
    """Launch exactly once. Only this fresh session and its owned tree are closed.

    Injectable process/connection hooks exist solely for offline regression tests.
    A bounded monitor checks disk and wall time; it never starts another process.
    """
    attempt = Path(attempt).resolve(strict=True)
    if not _inside(attempt, Path(stage_root).resolve()):
        raise RuntimeGateError("launch cwd must be under stage15")
    if not (attempt / "inputs.json").is_file():
        raise RuntimeGateError("freeze input/source manifest before launch")
    command = fluent_command(contract, attempt, processors)
    minimum_disk = float(contract["budget"]["minimum_free_disk_gib"]) * 1024 ** 3
    if shutil.disk_usage(attempt).free < minimum_disk:
        raise RuntimeGateError("insufficient free disk for this attempt")
    for name in ("server-info.txt", "launcher.stdout.log", "launcher.stderr.log", "runtime.json"):
        if (attempt / name).exists():
            raise FileExistsError("refusing a second launch: " + name)
    if connect is None:
        if Path(sys.executable).resolve() != Path(contract["software"]["python"]).resolve():
            raise RuntimeGateError("use the contract ANSYS Python 3.10 executable")
        import ansys.fluent.core as pyfluent
        connect = pyfluent.connect_to_fluent
    if psutil_module is None:
        import psutil as psutil_module
    popen = popen or subprocess.Popen
    env = full_environment(attempt=attempt, contract=contract)
    report = {"status": "not_run", "valid": False, "command": command,
              "processor_count": processors, "product_version": "24.2", "precision": "double",
              "automatic_retries": 0, "cwd": str(attempt), "initializations": 0,
              "flow_iterations": 0, "started_unix": time.time(),
              "environment": {k: env.get(k) for k in ("AWP_ROOT242", "ANSYSLMD_LICENSE_FILE", "TEMP", "TMP", "APPDATA", "LOCALAPPDATA", "COMSPEC")}}
    solver = process = tree = None
    stop = threading.Event()
    monitor_thread = None
    fatal = []
    with AnsysTaskLock(attempt, stage_root=stage_root) as lock:
        try:
            with (attempt / "launcher.stdout.log").open("xb") as stdout, (attempt / "launcher.stderr.log").open("xb") as stderr:
                process = popen(command, cwd=str(attempt), env=env, stdout=stdout, stderr=stderr, shell=False)
                report["launcher_pid"] = process.pid
                tree = OwnedProcessTree(process, psutil_module)
                deadline = time.monotonic() + contract["budget"]["fluent_gate_timeout_s"]

                def monitor():
                    while not stop.wait(0.5):
                        try:
                            tree.capture()
                            if time.monotonic() > deadline or shutil.disk_usage(attempt).free < minimum_disk:
                                fatal.append("wall-time budget or minimum disk breached")
                                tree.close()
                                return
                        except Exception as exc:
                            fatal.append("ownership monitor failed: " + repr(exc))
                            return

                monitor_thread = threading.Thread(target=monitor, daemon=True)
                monitor_thread.start()
                until = min(deadline, time.monotonic() + start_timeout)
                info = attempt / "server-info.txt"
                while not (info.is_file() and info.stat().st_size > 0):
                    if fatal:
                        raise RuntimeGateError(fatal[-1])
                    if process.poll() not in (None, 0):
                        raise RuntimeGateError("Fluent launcher exited before server readiness: " + str(process.returncode))
                    if time.monotonic() >= until:
                        raise TimeoutError("one Fluent launch timed out; no retry")
                    stop.wait(0.1)
                solver = connect(server_info_file_name=str(info), cleanup_on_exit=True,
                                 start_transcript=False, start_watchdog=False)
                if solver is None:
                    raise RuntimeGateError("connect_to_fluent returned None")
                report["connection_established"] = True
                version = solver.get_fluent_version()
                report["reported_version"] = str(version)
                if not re.search(r"(?:24\.2|2024\s*R2)", str(version), re.I):
                    raise RuntimeGateError("connected Fluent does not report version 24.2")
                yield solver, report
                if fatal:
                    raise RuntimeGateError(fatal[-1])
                report["status"] = "completed_execution_not_validation"
        except BaseException as exc:
            report["status"] = "failed"
            report["exception"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
            raise
        finally:
            stop.set()
            if monitor_thread:
                monitor_thread.join(timeout=2)
            cleanup_ok = tree is None and process is None
            if solver is not None:
                try:
                    solver.exit(timeout=10, timeout_force=False, wait=False)
                    report["session_exit_requested"] = True
                except Exception as exc:
                    report["exit_exception"] = repr(exc)
            if tree is not None:
                try:
                    cleanup_ok = tree.close()
                    report["owned_processes"] = [{"pid": p, "create_time": t} for p, t in tree.members.items()]
                    report["ownership_errors"] = tree.errors
                except Exception as exc:
                    report["cleanup_exception"] = repr(exc)
            report["owned_cleanup_verified"] = cleanup_ok
            # The server-info file contains a gRPC password. Never hash, print,
            # embed or retain it as evidence after this attempt's connection.
            credential_path = attempt / "server-info.txt"
            try:
                if credential_path.is_file():
                    credential_path.unlink()
                report["connection_credentials_removed"] = not credential_path.exists()
            except Exception as exc:
                report["credential_cleanup_error"] = repr(exc)
                cleanup_ok = False
            report["owned_cleanup_verified"] = cleanup_ok
            report["ended_unix"] = time.time()
            report["monitor_errors"] = fatal
            if not cleanup_ok:
                lock.retain("cleanup ownership not verified")
                report["lock_retained"] = True
            write_json_new(attempt / "runtime.json", report)
            if not cleanup_ok and sys.exc_info()[0] is None:
                raise RuntimeGateError("owned process cleanup unverified; lock retained")

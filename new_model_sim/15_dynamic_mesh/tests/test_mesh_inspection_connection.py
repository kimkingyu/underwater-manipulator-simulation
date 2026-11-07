"""Offline credential fixtures only. Never connects, spawns or uses real credentials."""
import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import types

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import mesh_inspection_connection as conn


@pytest.mark.parametrize("payload", [b"127.0.0.1:12345\nOFFLINE-FIXTURE\n", b"127.0.0.1:12345\nOFFLINE-FIXTURE",
    b"localhost:12345\r\nOFFLINE-FIXTURE\r\n", b"local-host:12345\nOFFLINE-FIXTURE\n"])
def test_complete_private_snapshot(payload):
    credentials = conn.parse_private_server_info(payload, {"localhost", "local-host"})
    assert credentials.port == 12345 and credentials.password == "OFFLINE-FIXTURE"
    assert "OFFLINE-FIXTURE" not in repr(credentials)


@pytest.mark.parametrize("payload", [b"", b"127.0.0.1:12345", b"127.0.0.1:12345\n", b"127.0.0.1:12345\n\n",
                                    b"\nOFFLINE-FIXTURE\n"])
def test_empty_or_partial_credential_never_connects(payload):
    with pytest.raises(conn.ServerInfoNotReady):
        conn.parse_private_server_info(payload, {"localhost"})


def test_original_two_line_guard_accepts_empty_secret_but_new_guard_rejects():
    partial = b"127.0.0.1:12345\n\n"
    assert len(partial.splitlines()) >= 2  # exact old readiness predicate
    with pytest.raises(conn.ServerInfoNotReady):
        conn.parse_private_server_info(partial, {"localhost"})


@pytest.mark.parametrize("payload", [b"127.0.0.1:0\nOFFLINE-FIXTURE\n", b"127.0.0.1:65536\nOFFLINE-FIXTURE\n",
    b"127.0.0.1:0123\nOFFLINE-FIXTURE\n", b"127.0.0.1:not-port\nOFFLINE-FIXTURE\n",
    b"remote.example:12345\nOFFLINE-FIXTURE\n", b"127.0.0.1:12345\nOFFLINE-FIXTURE\nextra\n",
    b"127.0.0.1:12345\nOFFLINE-FIXTURE\x00\n", b"127.0.0.1:12345\n OFFLINE-FIXTURE\n",
    b"127.0.0.1:12345\nOFFLINE-FIXTURE \n", b"127.0.0.1:12345\n\xff\n", b"x" * 8193])
def test_invalid_snapshot_is_payload_free(payload):
    with pytest.raises(conn.ServerInfoRejected) as captured:
        conn.parse_private_server_info(payload, {"localhost"})
    assert "OFFLINE-FIXTURE" not in str(captured.value)


class Clock:
    def __init__(self): self.now = 0.0
    def monotonic(self): return self.now
    def sleep(self, duration): self.now += duration


def test_stable_snapshot_waits_through_partial_and_changed_values(tmp_path, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(conn.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(conn.time, "sleep", clock.sleep)
    snapshots = iter([(1, b"127.0.0.1:12345\n\n"), (1, b"127.0.0.1:12345\nFIRST-OFFLINE\n"),
                      (2, b"127.0.0.1:12345\nSECOND-OFFLINE\n")])
    latest = [None]
    def snapshot(*a):
        try: latest[0] = next(snapshots)
        except StopIteration: pass
        return latest[0]
    monkeypatch.setattr(conn, "_private_snapshot", snapshot)
    value = conn.wait_private_credentials(tmp_path / "server.info", tmp_path,
        types.SimpleNamespace(poll=lambda: None), 1.0, {"localhost"})
    assert value.password == "SECOND-OFFLINE" and clock.now >= 0.3


def test_fixed_file_only_and_real_file_snapshot(tmp_path):
    path = tmp_path / "server.info"
    path.write_bytes(b"127.0.0.1:12345\nOFFLINE-FIXTURE\n")
    identity, data = conn._private_snapshot(path, tmp_path)
    assert data.startswith(b"127.0.0.1:") and len(identity) == 4
    with pytest.raises(conn.ServerInfoRejected):
        conn._private_snapshot(tmp_path / "other.info", tmp_path)
    with pytest.raises(conn.ServerInfoRejected):
        conn._private_snapshot(path, tmp_path / "different-attempt")


def test_wait_has_deadline_and_no_connection_retry(tmp_path, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(conn.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(conn.time, "sleep", clock.sleep)
    with pytest.raises(TimeoutError):
        conn.wait_private_credentials(tmp_path / "server.info", tmp_path,
            types.SimpleNamespace(poll=lambda: None), 0.15, {"localhost"})
    assert clock.now == pytest.approx(0.15)
    with pytest.raises(RuntimeError):
        conn.wait_private_credentials(tmp_path / "server.info", tmp_path,
            types.SimpleNamespace(poll=lambda: 0), 0.5, {"localhost"})


class InvalidPassword(ValueError): pass
class FixtureRpcError(RuntimeError):
    def __init__(self, status):
        super().__init__("OFFLINE-SENSITIVE-DETAIL")
        self.status = status
    def code(self): return types.SimpleNamespace(name=self.status)


@pytest.mark.parametrize("status,category", [("UNAUTHENTICATED", "authentication_rejected"),
    ("PERMISSION_DENIED", "authentication_rejected"), ("UNAVAILABLE", "transport_unavailable_not_proven_password_error"),
    ("DEADLINE_EXCEEDED", "bounded_startup_or_connection_timeout"), ("INTERNAL", "mode_detection_wrapped_error_actual_cause_unverified")])
def test_wrapped_error_retains_status_not_secrets(status, category):
    error = InvalidPassword("OFFLINE-SENSITIVE-DETAIL")
    error.__cause__ = FixtureRpcError(status)
    safe = conn.safe_connection_error(error)
    assert safe["category"] == category
    assert safe["exception_classes"] == ["InvalidPassword", "FixtureRpcError"]
    assert safe["rpc_status_codes"] == [status]
    assert "OFFLINE-SENSITIVE-DETAIL" not in json.dumps(safe)


ROOT = Path(r"D:/Program Files/ANSYS Inc/v242/commonfiles/CPython/3_10/winx64/Release/python/lib/site-packages/ansys/fluent/core")


def official_function(relative, name, fallback, namespace, expected_hash):
    """Compile just the hash-checked official function, never package imports.

    A committed exact semantic excerpt is used when that Windows installation is
    absent, keeping portable tests independent of Fluent and the native runtime.
    """
    path = ROOT / relative
    source = path.read_text(encoding="utf-8") if path.is_file() else fallback
    if path.is_file():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash
    node = next(x for x in ast.parse(source).body if isinstance(x, ast.FunctionDef) and x.name == name)
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), node], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, "<offline-hash-checked-official-fixture>", "exec"), namespace)
    return namespace[name]


PARSE = '''def _parse_server_info_file(file_name: str):
    with open(file_name, encoding="utf-8") as f:
        lines = f.readlines()
    ip_and_port = lines[0].strip().split(":")
    ip = ip_and_port[0]
    port = int(ip_and_port[1])
    password = lines[1].strip()
    return ip, port, password
'''
GET = '''def _get_server_info(server_info_file_name, ip=None, port=None, password=None):
    if not (ip and port) and not server_info_file_name:
        raise IpPortNotProvided()
    if (ip or port) and server_info_file_name:
        launcher_utils.logger.warning("fixture")
    else:
        if server_info_file_name:
            ip, port, password = _parse_server_info_file(server_info_file_name)
        ip = ip or os.getenv("PYFLUENT_FLUENT_IP", "127.0.0.1")
        port = port or os.getenv("PYFLUENT_FLUENT_PORT")
    if not port: raise PortNotProvided()
    return ip, port, password
'''
MODE = '''def _get_running_session_mode(fluent_connection, mode=None):
    if mode:
        session_mode = mode
    else:
        try:
            session_mode = FluentMode.get_mode("solver" if fluent_connection.scheme_eval.scheme_eval("(cx-solver-mode?)") else "meshing")
        except Exception as ex:
            raise exceptions.InvalidPassword() from ex
    return session_mode.value[0]
'''


def test_real_official_parser_and_password_branch_offline(tmp_path):
    path = tmp_path / "server.info"
    path.write_bytes(b"127.0.0.1:12345\nOFFLINE-FIXTURE\n")
    ns = {"os": os, "IpPortNotProvided": ValueError, "PortNotProvided": ValueError,
          "launcher_utils": types.SimpleNamespace(logger=types.SimpleNamespace(warning=lambda *a: None))}
    parse = official_function("session.py", "_parse_server_info_file", PARSE, ns,
        "a6266f48678e851a8962457ad7c843cc95cb6cf267885e47a6a919b8b8297097")
    get = official_function("launcher/server_info.py", "_get_server_info", GET, ns,
        "2857d719d425a56cef737c584c8bc17500e782dd109f216fc1e1aaab530474c1")
    assert get(str(path)) == ("127.0.0.1", 12345, "OFFLINE-FIXTURE")
    assert path.exists()  # official parser does not delete it
    path.write_bytes(b"127.0.0.1:12345\n\n")
    assert parse(str(path))[2] == ""  # old guard + official parser yields empty metadata
    credentials = conn.parse_private_server_info(b"127.0.0.1:12345\nOFFLINE-FIXTURE\n", {"localhost"})
    assert get(None, credentials.host, credentials.port, credentials.password)[2] == "OFFLINE-FIXTURE"


def test_real_official_mode_query_wraps_non_authentication_error():
    def failing_rpc(*a): raise FixtureRpcError("UNAVAILABLE")
    ns = {"exceptions": types.SimpleNamespace(InvalidPassword=InvalidPassword),
          "FluentMode": types.SimpleNamespace(get_mode=lambda name: None)}
    mode = official_function("launcher/pyfluent_enums.py", "_get_running_session_mode", MODE, ns,
        "9aa12e0270bcfcab3ae9f89b071d3be9b68f359483b8dad043116b9c430f5ae0")
    with pytest.raises(InvalidPassword) as captured:
        mode(types.SimpleNamespace(scheme_eval=types.SimpleNamespace(scheme_eval=failing_rpc)))
    assert conn.safe_connection_error(captured.value)["category"] == "transport_unavailable_not_proven_password_error"


def test_production_connection_module_cannot_launch_or_serialize_credentials():
    tree = ast.parse((STAGE / "mesh_inspection_connection.py").read_text(encoding="utf-8"))
    calls = {ast.unparse(x.func) for x in ast.walk(tree) if isinstance(x, ast.Call)}
    for forbidden in ("subprocess.Popen", "subprocess.run", "socket.socket", "print", "json.dump", "json.dumps", "hashlib.sha256"):
        assert forbidden not in calls

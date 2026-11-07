"""Private server-info readiness and secret-safe connection diagnostics.

Only a single native-created file in the fixed attempt is read. Credentials
remain in memory, are never returned to MCP, persisted, hashed or logged. The
public PyFluent ip/port/password arguments receive that exact validated snapshot.
This module never starts a process, opens a socket or calls a Fluent API.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import ipaddress
from pathlib import Path
import re
import stat
import time


class ServerInfoNotReady(ValueError):
    pass


class ServerInfoRejected(ValueError):
    pass


@dataclass(frozen=True)
class PrivateCredentials:
    host: str
    port: int
    password: str = field(repr=False, compare=False)


def parse_private_server_info(data, local_hosts):
    """Strict complete two-line native format; all errors are payload-free."""
    if not isinstance(data, bytes) or len(data) > 8192:
        raise ServerInfoRejected("Private server-info is oversized or not bytes")
    if not data or b"\n" not in data:
        raise ServerInfoNotReady("Private server-info endpoint line is not completely written")
    try:
        text = data.decode("utf-8-sig")
    except UnicodeError:
        raise ServerInfoRejected("Private server-info encoding is invalid") from None
    lines = text.splitlines()
    if len(lines) < 2 or not lines[0].strip() or not lines[1].strip():
        raise ServerInfoNotReady("Private server-info endpoint or credential is not ready")
    if len(lines) != 2:
        raise ServerInfoRejected("Private server-info does not have the fixed two-line format")
    endpoint, secret = lines
    if endpoint != endpoint.strip() or secret != secret.strip():
        raise ServerInfoRejected("Private server-info contains unexpected outer whitespace")
    if len(secret) > 2048 or any(ord(c) < 33 or ord(c) > 126 for c in secret):
        raise ServerInfoRejected("Private server-info credential format is invalid")
    if endpoint.count(":") != 1:
        raise ServerInfoRejected("Private server-info endpoint format is invalid")
    host, raw_port = endpoint.split(":")
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,253}", host) or not re.fullmatch(r"[0-9]{1,5}", raw_port):
        raise ServerInfoRejected("Private server-info endpoint format is invalid")
    port = int(raw_port)
    if not 1 <= port <= 65535 or str(port) != raw_port:
        raise ServerInfoRejected("Private server-info port is invalid")
    allowed = {str(name).casefold() for name in local_hosts}
    if host.casefold() not in allowed:
        try:
            local = ipaddress.ip_address(host).is_loopback
        except ValueError:
            local = False
        if not local:
            raise ServerInfoRejected("Private server-info endpoint is not local to this worker")
    return PrivateCredentials(host, port, secret)


def _private_snapshot(path, attempt):
    path, attempt = Path(path).absolute(), Path(attempt).absolute()
    if path != attempt / "server.info" or path.resolve() != path or attempt.resolve() != attempt:
        raise ServerInfoRejected("Only this fixed attempt's private server-info is permitted")
    try:
        before = path.lstat()
        if (not stat.S_ISREG(before.st_mode) or stat.S_ISLNK(before.st_mode) or
                getattr(before, "st_file_attributes", 0) & 0x400 or before.st_size > 8192):
            raise ServerInfoRejected("Private server-info is not a bounded ordinary file")
        with path.open("rb") as stream:
            data = stream.read(8193)
        after = path.lstat()
    except FileNotFoundError:
        raise ServerInfoNotReady("Private server-info file is not ready") from None
    identity = lambda info: (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns)
    if identity(before) != identity(after) or len(data) != after.st_size:
        raise ServerInfoNotReady("Private server-info changed during the bounded read")
    return identity(after), data


def wait_private_credentials(path, attempt, process, deadline, local_hosts, *, stable_seconds=0.2):
    """Two identical complete snapshots >=0.2s apart; no connect/process retry."""
    previous = None
    stable_since = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Single native process exited before authenticated connection readiness")
        try:
            identity, data = _private_snapshot(path, attempt)
            credentials = parse_private_server_info(data, local_hosts)
        except ServerInfoNotReady:
            previous = stable_since = None
        else:
            current = (identity, data)
            now = time.monotonic()
            if previous == current and stable_since is not None and now - stable_since >= stable_seconds:
                # No later re-read by PyFluent: connect receives this exact
                # complete in-memory snapshot through its public arguments.
                return credentials
            if previous != current:
                previous, stable_since = current, now
        time.sleep(min(0.05, max(0, deadline - time.monotonic())))
    raise TimeoutError("Single native startup exceeded its bounded readiness deadline; no retry")


def safe_connection_error(exc):
    """Only class names and fixed RPC status codes, never exception text/details."""
    classes, codes, seen = [], [], set()
    current = exc
    allowed_codes = {"UNAUTHENTICATED", "PERMISSION_DENIED", "UNAVAILABLE", "DEADLINE_EXCEEDED",
                     "UNKNOWN", "INTERNAL", "CANCELLED", "FAILED_PRECONDITION", "UNIMPLEMENTED"}
    while current is not None and len(classes) < 5 and id(current) not in seen:
        seen.add(id(current))
        name = type(current).__name__
        classes.append(name if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]{0,79}", name) else "Exception")
        try:
            code = current.code()
            code_name = getattr(code, "name", None)
            if code_name in allowed_codes and code_name not in codes:
                codes.append(code_name)
        except Exception:
            pass
        current = current.__cause__ or current.__context__
    if any(code in codes for code in ("UNAUTHENTICATED", "PERMISSION_DENIED")):
        category = "authentication_rejected"
    elif "UNAVAILABLE" in codes:
        category = "transport_unavailable_not_proven_password_error"
    elif "DEADLINE_EXCEEDED" in codes or "TimeoutError" in classes:
        category = "bounded_startup_or_connection_timeout"
    elif "InvalidPassword" in classes:
        category = "mode_detection_wrapped_error_actual_cause_unverified"
    elif any(name in classes for name in ("ServerInfoRejected", "ServerInfoNotReady")):
        category = "private_server_info_not_validated"
    else:
        category = "native_operation_error_unverified_cause"
    return {"category": category, "exception_classes": classes, "rpc_status_codes": codes,
            "exception_text_omitted": True, "credentials_persisted": False}

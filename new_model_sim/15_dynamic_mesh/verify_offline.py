"""Run stage15 offline regression tests and preserve a source-bound receipt."""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time

import pytest
from datetime import datetime
from pathlib import Path

from preflight import STAGE, sha256, within, write_new_json


def source_snapshot():
    result = {}
    for path in sorted(STAGE.rglob("*")):
        relative = path.relative_to(STAGE)
        if not path.is_file() or relative.parts[0] in ("runs", "reports", "frozen"):
            continue
        if path.suffix in (".py", ".c", ".in", ".wbjn", ".json"):
            result[relative.as_posix()] = sha256(path)
    return result


def run(report_path):
    report_path = within(report_path, STAGE)
    if report_path.exists():
        raise FileExistsError("refusing to overwrite " + str(report_path))

    class Results:
        def __init__(self):
            self.collected = []
            self.executed = set()
            self.passed = set()
            self.failures = []
            self.errors = []
            self.skipped = {}

        def pytest_collection_finish(self, session):
            self.collected = [item.nodeid for item in session.items]

        def pytest_collectreport(self, report):
            if report.failed:
                self.errors.append({"test": report.nodeid, "phase": "collection", "traceback": str(report.longrepr)})
            elif report.skipped:
                self.skipped[report.nodeid] = str(report.longrepr)

        def pytest_runtest_logreport(self, report):
            self.executed.add(report.nodeid)
            if report.skipped or getattr(report, "wasxfail", None):
                self.skipped[report.nodeid] = str(report.longrepr or getattr(report, "wasxfail", ""))
            elif report.failed:
                target = self.failures if report.when == "call" else self.errors
                target.append({"test": report.nodeid, "phase": report.when, "traceback": str(report.longrepr)})
            elif report.when == "call" and report.passed:
                self.passed.add(report.nodeid)

    before = source_snapshot()
    stream = io.StringIO()
    started = time.monotonic()
    result = Results()
    previous_autoload = os.environ.get("PYTEST_DISABLE_PLUGIN_AUTOLOAD")
    previous_addopts = os.environ.pop("PYTEST_ADDOPTS", None)
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    try:
        with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            # Pytest collects both the original unittest cases and new fixture /
            # parameterized safety tests. unittest.discover silently omitted the latter.
            code = int(pytest.main([str(STAGE / "tests"), "-q", "--tb=short", "-p", "no:cacheprovider",
                                   "--confcutdir", str(STAGE / "tests"), "-o", "addopts="], plugins=[result]))
    finally:
        if previous_autoload is None:
            os.environ.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
        else:
            os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = previous_autoload
        if previous_addopts is not None:
            os.environ["PYTEST_ADDOPTS"] = previous_addopts
    after = source_snapshot()
    source_unchanged = before == after
    count = len(result.collected)
    passed = (code == 0 and count > 0 and count == len(result.executed) == len(result.passed)
              and not result.failures and not result.errors and not result.skipped and source_unchanged)
    report = {
        "schema_version": 2,
        "status": "passed" if passed else "failed",
        "created_local": datetime.now().astimezone().isoformat(),
        "scope": "offline code regression only; no CAD/mesh/Fluent physics validation",
        "test_runner": "pytest (including unittest and parameterized cases)",
        "test_runner_exit_code": code,
        "python_executable": sys.executable,
        "python_version": sys.version,
        "test_count": count,
        "executed_count": len(result.executed),
        "passed_test_count": len(result.passed),
        "failure_count": len(result.failures),
        "error_count": len(result.errors),
        "skipped_count": len(result.skipped),
        "skipped": [{"test": name, "reason": reason} for name, reason in result.skipped.items()],
        "failures": result.failures,
        "errors": result.errors,
        "collected_test_ids": result.collected,
        "seconds": time.monotonic() - started,
        "source_unchanged_during_run": source_unchanged,
        "source_sha256_before": before,
        "source_sha256_after": after,
        "test_output": stream.getvalue(),
        "real_motion_api_verified": False,
        "real_geometry_verified": False,
    }
    write_new_json(report_path, report)
    print(json.dumps({key: report[key] for key in
                      ("status", "test_count", "failure_count", "error_count", "skipped_count",
                       "source_unchanged_during_run", "seconds")}, ensure_ascii=False))
    print("report=" + str(report_path))
    if not passed:
        print(stream.getvalue()[-18000:])
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    raise SystemExit(run(parser.parse_args().report))

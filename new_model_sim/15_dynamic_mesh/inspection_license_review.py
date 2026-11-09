"""Explicitly approved narrow licensing-helper review; never launches or cleans.

Historical execution and new QA sources are deliberately separate bindings.
Only the dedicated confirmed MCP review writer may create the fixed proof.
"""
from __future__ import annotations
import ctypes
import math
import os
from pathlib import Path
import re
import time

try:
    import psutil
except ImportError:
    psutil = None  # MCP transport uses the fixed official Python for read-only evaluation.
from mcp_bridge import STAGE, Stage15Bridge, BridgeError, read_json, sha256, write_new

OLD_GRANT = "frozen/mesh_inspection_full_record_authorization_001.json"
OLD_SOURCES = "frozen/mesh_inspection_full_record_sources_001.json"
APPROVAL = "frozen/mesh_inspection_license_classification_authorization_001.json"
RUNTIME_MANIFEST = "frozen/mesh_inspection_license_runtime_001.json"
REVIEW = "reports/full_record_license_review_001.json"
REVIEW_SEAL = "reports/full_record_license_review_seal_001.json"
REVIEW_CONFIRMATION = "WRITE_STAGE15_LICENSE_CLASSIFICATION_REVIEW_ONCE"
APPROVAL_SHA256 = "4da10d83f068eeafee4cd349437ee5a3505831e23996c88ac6233c08e2f35da8"
CONCLUSION = "connection+normal-exit+final cleanup satisfied under explicitly user-approved licensing-helper classification"
NATIVE_NAME = re.compile(r"(?:fluent[^/\\]*|cx\d+|fl\d+|cortex[^/\\]*|spaceclaim[^/\\]*|runwb2|ansyswbu[^/\\]*|ansysmeshing[^/\\]*|meshing)\.exe\Z", re.I)


def canonical_file(value):
    """Exact existing file, no reparse at any ancestor; Windows short paths via API."""
    if type(value) is not str or not Path(value).is_absolute():
        raise BridgeError("Executable must be an absolute file")
    path = Path(value)
    for parent in (path, *path.parents):
        stat = parent.lstat()
        if parent.is_symlink() or getattr(stat, "st_file_attributes", 0) & 0x400:
            raise BridgeError("Executable reparse path forbidden")
    if not path.is_file():
        raise BridgeError("Executable file unavailable")
    if os.name != "nt":
        return str(path.resolve()).casefold()
    function = ctypes.WinDLL("kernel32", use_last_error=True).GetLongPathNameW
    function.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p, ctypes.c_uint]
    function.restype = ctypes.c_uint
    output = ctypes.create_unicode_buffer(32768)
    length = function(str(path), output, len(output))
    if not 0 < length < len(output):
        raise BridgeError("Windows executable long-path identity unavailable")
    return os.path.normpath(output.value).casefold()


def helper_installation(slot):
    approval = load_approval(slot)
    contract = slot._json("contract.json")
    expected = Path(contract["software"]["ansys_root"]) / "licensingclient/winx64/ansyscl.exe"
    if canonical_file(str(expected)) != canonical_file(approval["helper"]["path"]):
        raise BridgeError("Helper installation root mismatch")
    if expected.stat().st_size != approval["helper"]["size"] or sha256(expected) != approval["helper"]["sha256"]:
        raise BridgeError("Official helper binary changed")
    return expected


def classify_live_helpers(slot, final):
    """No process action. Allow only exact approved binary with complete owned IDs."""
    live = final.get("owned_live_before_cleanup")
    identities = final.get("owned_process_identity")
    owned = final.get("owned_processes")
    if not isinstance(live, list) or len(live) > 256 or not isinstance(identities, list) or not isinstance(owned, list):
        raise BridgeError("Owned cleanup identity lists missing")
    if not live:
        return []
    expected = helper_installation(slot)
    pairs = []
    for row in live:
        if set(row) != {"pid", "creation_time"} or type(row["pid"]) is not int or row["pid"] <= 0 or type(row["creation_time"]) not in (int, float) or not math.isfinite(row["creation_time"]):
            raise BridgeError("Live helper PID/create-time invalid")
        pair = (row["pid"], row["creation_time"])
        if pair in pairs or sum((x.get("pid"), x.get("creation_time")) == pair for x in owned) != 1:
            raise BridgeError("Live helper absent/duplicated in owned tree")
        matches = [x for x in identities if (x.get("pid"), x.get("creation_time")) == pair]
        if len(matches) != 1 or matches[0].get("name", "").casefold() != "ansyscl.exe":
            raise BridgeError("Only exact licensing helper identity may remain before cleanup")
        item = matches[0]
        if canonical_file(item.get("executable")) != canonical_file(str(expected)) or not os.path.samefile(item["executable"], expected):
            raise BridgeError("Licensing helper executable identity mismatch")
        pairs.append(pair)
    return [{"pid": pid, "creation_time": stamp, "classification": "official_licensing_helper",
             "canonical_executable": canonical_file(str(expected))} for pid, stamp in pairs]


def load_approval(slot):
    if sha256(slot.safe(APPROVAL)) != APPROVAL_SHA256:
        raise BridgeError("Explicit classification approval binding changed")
    approval = slot._json(APPROVAL)
    if (approval.get("additional_connection_attempts") != 0 or approval.get("additional_QA_attempts") != 0 or
            approval.get("original_result_must_remain_failed") is not True or
            approval.get("user_choice") != "按此分类继续（推荐）" or
            approval.get("runtime_manifest_relative") != RUNTIME_MANIFEST or
            sha256(slot.safe(RUNTIME_MANIFEST)) != approval.get("runtime_manifest_sha256")):
        raise BridgeError("Classification approval scope invalid")
    return approval


class HistoricalConnection:
    """Read-only facade: sources are the OLD frozen sources, never current QA code."""
    def __init__(self, root):
        from inspection_batch002_bridge import FullRecordValidationSlot
        self._slot = FullRecordValidationSlot(root)
        self.root = root
        self.JOB_ID = self._slot.JOB_ID
        self.ATTEMPT = self._slot.ATTEMPT
        self.slot = self._slot.slot
        self.AUTHORIZATION = OLD_GRANT
        self.SOURCE_MANIFEST = OLD_SOURCES
    def safe(self, path): return self._slot.safe(path)
    def _json(self, path): return self._slot._json(path)
    def _sources(self): return self._json(OLD_SOURCES)["files"]
    def authorize(self):
        approval = load_approval(self)
        if sha256(self.safe(OLD_GRANT)) != approval["old_authorization_sha256"] or sha256(self.safe(OLD_SOURCES)) != approval["old_source_manifest_sha256"]:
            raise BridgeError("Actual historical grant/source manifest changed")
        grant = self._json(OLD_GRANT)
        if (grant.get("full_record_job_id") != self.JOB_ID or grant.get("full_record_slot") != self.slot or
                grant.get("full_record_attempt_relative") != self.ATTEMPT or grant.get("additional_connection_attempts") != 1 or
                grant.get("additional_QA_attempts") != 0 or grant.get("source_manifest_relative") != OLD_SOURCES or
                grant.get("source_manifest_sha256") != approval["old_source_manifest_sha256"]):
            raise BridgeError("Historical explicit one-connection grant invalid")
        runtime = self._json(RUNTIME_MANIFEST)
        if len(self._sources()) != 20 or not runtime.get("files"):
            raise BridgeError("Historical execution evidence incomplete")
        for row in runtime["files"]:
            relative = row["path"]
            if relative not in runtime_paths(self):
                raise BridgeError("Unexpected runtime artifact path")
            path = self.safe(relative)
            if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]:
                raise BridgeError("Original runtime artifact changed")
        if {r["path"] for r in runtime["files"]} != set(runtime_paths(self)) or len(runtime["files"]) != len(runtime_paths(self)):
            raise BridgeError("Original runtime artifact set changed")
        return grant


def runtime_paths(conn):
    """Fixed structured artifacts and exactly the frozen source archive; no logs/credentials."""
    names = ["request.json", "probe_result.json", "native_result.json", "readiness.json", "launcher.json", "native_claim.json",
        "fluent_process.json", "command_audit.json", "normal_exit_attempted.json", "normal_exit_completed.json",
        "transcript_start_attempted.json", "transcript_start_completed.json", "transcript_stop_attempted.json",
        "transcript_stop_completed.json", "source_snapshot/archive_manifest.json", "source_snapshot/execution_binding.json"]
    # Construct expected archive paths from OLD source rows; never accept a free path in a runtime manifest.
    for row in conn._sources():
        source = Path(row["path"])
        try: relative = source.relative_to(conn.root).as_posix()
        except ValueError:
            if source != conn.root.parent / "04_workbench/run_wb_fullenv.py":
                raise BridgeError("Unexpected historical external source")
            relative = "external/run_wb_fullenv.py"
        names.append("source_snapshot/" + relative)
    names += ["source_snapshot/execution_binding/" + Path(r).name for r in (OLD_GRANT, OLD_SOURCES)]
    return [conn.ATTEMPT + "/" + n for n in names]


def process_quiescence(slot, final, *, active_qa=None):
    """Read-only old PID reuse checks plus global native scan. No kill/wait/cleanup."""
    states = []
    old = HistoricalConnection(slot.root)
    launcher = old._json(old.ATTEMPT + "/launcher.json")
    if launcher.get("job_id") != old.JOB_ID:
        raise BridgeError("Original worker launcher identity missing")
    original_processes = list(final["owned_processes"])
    launcher_pair = {"pid": launcher.get("pid"), "creation_time": launcher.get("creation_time")}
    if launcher_pair not in original_processes:
        original_processes.append(launcher_pair)
    for row in original_processes:
        pid, stamp = row.get("pid"), row.get("creation_time")
        if type(pid) is not int or pid <= 0 or type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise BridgeError("Original owned process identity invalid")
        try:
            actual = psutil.Process(pid).create_time()
        except psutil.NoSuchProcess:
            states.append({"pid": pid, "creation_time": stamp, "state": "exited"}); continue
        if actual == stamp:
            raise BridgeError("An original owned process is still alive")
        states.append({"pid": pid, "creation_time": stamp, "state": "pid_reused_not_old_job"})
    # Current QA can recheck its gate while its own exact verified tree is running.
    anchor = None
    if active_qa is not None and slot.safe(".ansys-task.lock").exists():
        lock = slot._json(".ansys-task.lock")
        launcher = slot._json(active_qa.ATTEMPT + "/launcher.json")
        if lock.get("attempt") != str(active_qa.safe(active_qa.ATTEMPT)) or lock.get("pid") != launcher.get("pid"):
            raise BridgeError("Active QA ownership lock mismatch")
        p = psutil.Process(launcher["pid"])
        if p.create_time() != launcher.get("creation_time") or launcher.get("job_id") != active_qa.JOB_ID:
            raise BridgeError("Active QA launcher identity mismatch")
        anchor = (p.pid, p.create_time())
    for process in psutil.process_iter(["pid", "name"]):
        name = process.info.get("name")
        if not isinstance(name, str):
            raise BridgeError("Global process identity unavailable")
        if not NATIVE_NAME.fullmatch(name): continue
        try:
            chain = [process] + process.parents()
            allowed = anchor is not None and any((p.pid, p.create_time()) == anchor for p in chain[:256])
        except psutil.NoSuchProcess:
            continue
        if not allowed:
            raise BridgeError("Global unrelated native process blocks review/QA")
    for name in (".ansys-task.lock", ".mesh-inspection-dispatch.lock", ".sizing-dispatch.lock"):
        if not slot.safe(name).exists(): continue
        if active_qa is None or name == ".sizing-dispatch.lock":
            raise BridgeError("Existing task/dispatch lock blocks review")
        lock = slot._json(name)
        if name == ".ansys-task.lock" and lock.get("attempt") == str(active_qa.safe(active_qa.ATTEMPT)): continue
        if name == ".mesh-inspection-dispatch.lock" and lock.get("job_id") == active_qa.JOB_ID: continue
        raise BridgeError("Unrelated lock blocks QA recheck")
    return states


def evaluate_review(stage_root=STAGE, *, active_qa=None):
    """No persistence. Full old evidence gate and current QA/source/input checks."""
    if psutil is None:
        return read_only_dispatch(stage_root, qa_slot=active_qa.slot if active_qa else None)
    from inspection_batch002_bridge import Batch002Slot, FullRecordValidationSlot, GRANT, MANIFEST
    qa = active_qa or Batch002Slot("background", stage_root)
    qa.authorize()
    old = HistoricalConnection(stage_root)
    old.authorize()
    final = old._json(old.ATTEMPT + "/probe_result.json")
    if final.get("status") != "failed" or final.get("error") is not None or final.get("safe_error_classification") is not None:
        raise BridgeError("Review applies only to the preserved exact original helper-only failed result")
    classification = classify_live_helpers(old, final)
    expected_helper = load_approval(old).get("expected_historical_helper")
    if (not classification or len(classification) != 1 or expected_helper != {
            "pid": classification[0]["pid"], "creation_time": classification[0]["creation_time"]}):
        raise BridgeError("Original failure must match the explicitly approved exact historical helper")
    # Reuse every existing native/input/deadline/source/PIPE/log/identity gate.
    evidence = qa._connection_evidence(old, classified=True)
    created = final["created_unix"]
    for row in final["owned_processes"]:
        if not created <= row["creation_time"] <= created + final["elapsed_seconds"]:
            raise BridgeError("Owned PID time outside original job interval")
    states = process_quiescence(old, final, active_qa=active_qa)
    approval = load_approval(old)
    return {"schema_version": 1, "job_id": old.JOB_ID, "original_status": final["status"],
        "conclusion": CONCLUSION, "original_result_unchanged": True, "additional_connection_attempts": 0, "additional_QA_attempts": 0,
        "classification": classification, "old_authorization_sha256": approval["old_authorization_sha256"],
        "old_source_manifest_sha256": approval["old_source_manifest_sha256"], "runtime_manifest_sha256": sha256(old.safe(RUNTIME_MANIFEST)),
        "classification_authorization_sha256": sha256(old.safe(APPROVAL)), "new_QA_authorization_sha256": sha256(qa.safe(GRANT)),
        "new_QA_source_manifest_sha256": sha256(qa.safe(MANIFEST)), "native_evidence": evidence,
        "owned_process_exit_states": states, "global_native_policy_verified": True}


def stable_review(value):
    # PID reuse can change after writing; recheck still required but no stale state equality.
    return {k: v for k, v in value.items() if k != "owned_process_exit_states"}


def write_review(confirmation, stage_root=STAGE):
    """Only a fixed explicit parent-reviewed request writes a create-only proof."""
    if confirmation != REVIEW_CONFIRMATION:
        raise BridgeError("Explicit fixed review confirmation required; no native execution")
    if psutil is None:
        return read_only_dispatch(stage_root, write=True)
    slot = Stage15Bridge(stage_root)
    value = evaluate_review(stage_root)
    if slot.safe(REVIEW).exists():
        existing = slot._json(REVIEW)
        if (slot._json(REVIEW_SEAL) != {"review_sha256": sha256(slot.safe(REVIEW)), "classification_authorization_sha256": APPROVAL_SHA256} or
                stable_review(existing) != stable_review(value)):
            raise BridgeError("Existing independent review differs; never overwrite")
        return {"status": "already_reviewed", "review_sha256": sha256(slot.safe(REVIEW)), "native_started": False}
    if slot.safe(REVIEW_SEAL).exists():
        raise BridgeError("Orphaned review seal; never overwrite")
    write_new(slot.safe(REVIEW), value)
    write_new(slot.safe(REVIEW_SEAL), {"review_sha256": sha256(slot.safe(REVIEW)), "classification_authorization_sha256": APPROVAL_SHA256})
    return {"status": "reviewed", "conclusion": CONCLUSION, "review_sha256": sha256(slot.safe(REVIEW)), "native_started": False}


def read_only_dispatch(stage_root, *, qa_slot=None, write=False):
    """Fixed read-only evaluator subprocess, NOT an ANSYS/native worker launcher."""
    import json
    import subprocess
    slot = Stage15Bridge(stage_root)
    if slot.root != STAGE:
        raise BridgeError("Production read-only dispatch requires the fixed stage root")
    args = [slot._python(), "-B", str(slot.safe("inspection_license_review.py"))]
    if write:
        args += ["--write-review"]
    else:
        args += ["--evaluate"]
        if qa_slot is not None:
            if qa_slot not in ("background", "arm"):
                raise BridgeError("Read-only evaluator accepts original QA slots only")
            args += ["--qa-slot", qa_slot]
    result = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True, timeout=60,
        cwd=str(slot.root), env=slot._environment(), shell=False)
    if result.returncode != 0 or len(result.stdout) > 65536:
        raise BridgeError("Fixed read-only review evaluator failed; no native execution")
    value = json.loads(result.stdout.decode("utf-8-sig"))
    if not isinstance(value, dict):
        raise BridgeError("Read-only evaluator returned invalid evidence")
    return value


def review_gate(qa):
    """Re-evaluate both source versions and original artifacts on EVERY QA gate."""
    existing = qa._json(REVIEW)
    seal = qa._json(REVIEW_SEAL)
    if seal != {"review_sha256": sha256(qa.safe(REVIEW)), "classification_authorization_sha256": APPROVAL_SHA256}:
        raise BridgeError("Independent review seal mismatch")
    actual = evaluate_review(qa.root, active_qa=qa)
    if (seal != {"review_sha256": sha256(qa.safe(REVIEW)), "classification_authorization_sha256": APPROVAL_SHA256} or
            stable_review(existing) != stable_review(actual)):
        raise BridgeError("Independent review/source/runtime binding changed")
    return {"status": "passed", "job_id": actual["job_id"], "review_sha256": sha256(qa.safe(REVIEW)),
        "runtime_manifest_sha256": actual["runtime_manifest_sha256"],
        "classification_authorization_sha256": actual["classification_authorization_sha256"],
        "new_QA_source_manifest_sha256": actual["new_QA_source_manifest_sha256"]}


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--evaluate", action="store_true")
    mode.add_argument("--write-review", action="store_true")
    parser.add_argument("--qa-slot", choices=("background", "arm"))
    args = parser.parse_args()
    if psutil is None or args.write_review and args.qa_slot:
        parser.error("Official Python/process reader required; write accepts no QA slot")
    from inspection_batch002_bridge import Batch002Slot
    try:
        value = write_review(REVIEW_CONFIRMATION) if args.write_review else evaluate_review(
            active_qa=Batch002Slot(args.qa_slot) if args.qa_slot else None)
        print(json.dumps(value, ensure_ascii=False, allow_nan=False))
    except Exception as exc:
        print(json.dumps({"status": "blocked", "error_class": type(exc).__name__, "native_started": False}))
        raise SystemExit(1)

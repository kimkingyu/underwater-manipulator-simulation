"""Run approved validation cases serially; --check never imports or launches Fluent."""
import argparse
import json
import sys
from pathlib import Path

from run_case import DEFAULT_CONFIG, ProtocolError, check_case, execute, load_config, verify_freeze


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--freeze-manifest", type=Path)
    parser.add_argument("--case-id", action="append", dest="case_ids")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--repair-reason")
    args = parser.parse_args(argv)
    try:
        cfg = load_config(args.case_config)
        all_ids = [c["id"] for c in cfg["cases"]]
        selected = args.case_ids if args.case_ids else all_ids
        if len(set(selected)) != len(selected) or any(cid not in all_ids for cid in selected):
            raise ProtocolError("duplicate/unknown requested case IDs")
        if args.repair_reason and len(selected) != 1:
            raise ProtocolError("an explicit repair retry must select exactly one case")
        if args.check:
            rows = [check_case(cid, args.case_config, args.freeze_manifest) for cid in selected]
            print(json.dumps({"status": "checked_no_solver", "case_count": len(rows),
                              "steady_count": sum(c["mode"] == "steady_translation" for c in rows),
                              "oscillation_count": sum(c["mode"] == "oscillation_translation" for c in rows),
                              "all_ready": all(c["ready_to_run"] for c in rows), "cases": rows}, indent=2))
            return 0
        # A missing/invalid freeze is rejected before even the first case/attempt is created.
        verify_freeze(args.freeze_manifest, args.case_config)
        for cid in selected:
            result = execute(cid, args.case_config, args.freeze_manifest, args.repair_reason)
            print(json.dumps(result), flush=True)
        return 0
    except (ProtocolError, ValueError, KeyError, OSError, RuntimeError) as exc:
        print(json.dumps({"status": "refused_or_failed", "error": str(exc), "remaining_cases": "not started; automatic retries disabled"}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

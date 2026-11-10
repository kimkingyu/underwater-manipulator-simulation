"""Convert one stage15 tetrahedral domain mesh to polyhedra in Fluent and gate its quality.

Usage (official Ansys Python 3.10):
    python poly_convert.py --mesh runs/mesh/compact_001/background.msh --domain background

Same proven sequence as 12_coarse_mesh/fluent_poly.py: read mesh -> quality ->
convert-domain -> quality -> exactly one improve-quality -> quality -> mesh info ->
write <domain>_poly.cas.h5. No initialization, iteration, motion or overset assembly.
Never overwrites; writes <domain>_poly.json next to the mesh.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

GATE = 0.01
STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE.parent / "04_workbench"))


def parse_quality(text):
    row = {}
    for key, pattern in (("min_orth", r"Minimum Orthogonal Quality =\s*([-+0-9.eE]+)"),
                         ("max_aspect", r"Maximum Aspect Ratio =\s*([-+0-9.eE]+)"),
                         ("min_volume_m3", r"minimum volume \(m3\):\s*([-+0-9.eE]+)")):
        found = re.findall(pattern, text)
        row[key] = float(found[-1]) if found else None
    loc = re.findall(r"Minimum Orthogonal Quality =.*?at location \(([^)]*)\)", text)
    row["min_orth_location_m"] = loc[-1].strip() if loc else None
    row["negative_volume"] = bool(row["min_volume_m3"] is not None and row["min_volume_m3"] <= 0)
    row["errors"] = [l.strip() for l in text.splitlines() if "Error" in l or "error:" in l][:10]
    return row


def parse_cells(text):
    return sum(int(n) for n in re.findall(r"^\s*(\d+)\s+(?:polyhedra|polyhedral|tetrahedral|mixed)\s+cells", text, re.M))


def verdict(report):
    final = report.get("poly_improved") or report.get("poly") or {}
    errors = []
    if final.get("min_orth") is None:
        errors.append("missing final minimum orthogonal quality")
    elif final["min_orth"] < GATE:
        errors.append("final minimum orthogonal quality %.6g < %.2f" % (final["min_orth"], GATE))
    if final.get("negative_volume"):
        errors.append("negative or zero cell volume")
    if final.get("errors"):
        errors.append("Fluent reported errors during quality/check")
    if not report.get("polyhedral_cells"):
        errors.append("polyhedral cell count not observed")
    if report.get("domain") == "arm" and "mesh_info" in report and not any(
            "overset" in line for line in report["mesh_info"]):
        errors.append("overset boundary faces not observed after conversion")
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--domain", required=True, choices=("background", "arm"))
    args = parser.parse_args(argv)
    mesh = Path(args.mesh).resolve(strict=True)
    if STAGE not in mesh.parents:
        raise SystemExit("mesh must be inside stage15")
    out = mesh.parent / (args.domain + "_poly.json")
    trn = mesh.parent / (args.domain + "_poly.trn")
    case = mesh.parent / (args.domain + "_poly.cas.h5")
    for p in (out, trn, case):
        if p.exists():
            raise SystemExit("refusing to overwrite " + str(p))
    from run_wb_fullenv import full_environment
    os.environ.update(full_environment())
    import ansys.fluent.core as pyfluent

    report = {"domain": args.domain, "mesh": str(mesh), "gate_min_orth": GATE, "status": "failed",
              "initialized": False, "iterations": 0, "motion": False, "overset_assembled": False}

    def save():
        out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    def transcript_since(before):
        time.sleep(2)
        return trn.read_bytes().decode("gbk", errors="replace")[len(before):]

    def quality(label):
        before = trn.read_bytes().decode("gbk", errors="replace") if trn.exists() else ""
        solver.tui.mesh.quality()
        solver.tui.mesh.check()
        report[label] = parse_quality(transcript_since(before))
        save()

    started = time.time()
    solver = pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double", processor_count=1,
                                    ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS, start_timeout=300,
                                    cleanup_on_exit=True, start_transcript=False)
    try:
        solver.transcript.start(str(trn))
        solver.file.read_mesh(file_name=str(mesh))
        quality("tet")
        t0 = time.time()
        solver.tui.mesh.polyhedra.convert_domain()
        report["convert_seconds"] = round(time.time() - t0, 1)
        quality("poly")
        solver.tui.mesh.repair_improve.improve_quality()
        quality("poly_improved")
        before = trn.read_bytes().decode("gbk", errors="replace")
        solver.tui.mesh.mesh_info(0)
        info = transcript_since(before)
        report["mesh_info"] = [l.strip() for l in info.splitlines() if "cells" in l or "faces" in l or "nodes" in l][:30]
        report["polyhedral_cells"] = parse_cells(info)
        solver.file.write(file_type="case", file_name=str(case))
        report["case_written"] = case.exists()
    except Exception as exc:
        report["exception"] = repr(exc)[:2000]
    finally:
        try:
            solver.transcript.stop()
        except Exception:
            pass
        solver.exit()
    report["seconds"] = round(time.time() - started, 1)
    report["errors"] = ([report["exception"]] if "exception" in report else []) + verdict(report)
    report["status"] = "failed" if report["errors"] else "passed_static_single_domain"
    save()
    print(json.dumps({k: report.get(k) for k in ("status", "polyhedral_cells", "errors")}, ensure_ascii=False))
    return 0 if report["status"] != "failed" else 2


if __name__ == "__main__":
    raise SystemExit(main())

"""Read-only: parse a binary Fluent .msh (ANSYS Meshing export), compute per-tet
orthogonal-quality proxy (min dihedral-based), and report worst cells with the
nearest wall zone. No ANSYS launch, no writes except one JSON report.

Usage: python bad_cell_locator.py runs/mesh/compact_001/arm.msh out.json [top_n]
"""
import json
import re
import sys
from pathlib import Path

import numpy as np


def read_sections(data):
    pos = 0
    nodes = None
    faces = []
    names = {}
    n = len(data)
    while True:
        i = data.find(b"(", pos)
        if i < 0:
            break
        m = re.match(rb"\((\d+)", data[i:i + 12])
        if not m:
            pos = i + 1
            continue
        idx = int(m.group(1))
        if idx == 3010:  # binary double nodes
            hdr_end = data.index(b")", i + 1)
            hdr = data[i + 1:hdr_end].decode()
            parts = hdr.split("(")[1].split()
            zid, start, end = int(parts[0], 16), int(parts[1], 16), int(parts[2], 16)
            nd = int(parts[4]) if len(parts) > 4 else 3
            body = data.index(b"(", hdr_end) + 1
            count = end - start + 1
            arr = np.frombuffer(data, dtype="<f8", count=count * nd, offset=body).reshape(count, nd)
            nodes = arr if nodes is None else np.vstack([nodes, arr])
            pos = body + count * nd * 8
        elif idx == 2013:  # binary int faces
            hdr_end = data.index(b")", i + 1)
            hdr = data[i + 1:hdr_end].decode()
            parts = hdr.split("(")[1].split()
            zid, start, end, btype, etype = [int(p, 16) for p in parts[:5]]
            body = data.index(b"(", hdr_end) + 1
            count = end - start + 1
            if etype != 3:
                raise SystemExit("only uniform triangle face zones supported, got elemType %d" % etype)
            arr = np.frombuffer(data, dtype="<i4", count=count * 5, offset=body).reshape(count, 5)
            faces.append((zid, arr))
            pos = body + count * 5 * 4
        elif idx == 45:
            j = data.index(b")", i)
            txt = data[i:j + 1].decode(errors="replace")
            mm = re.match(r"\(45 \((\d+) (\S+) (\S+)\)", txt)
            if mm:
                names[int(mm.group(1))] = (mm.group(2), mm.group(3))
            pos = j + 1
        else:
            pos = i + 1
    return nodes, faces, names


def main():
    mesh, out = Path(sys.argv[1]), Path(sys.argv[2])
    top = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    if out.exists():
        raise SystemExit("refusing to overwrite")
    data = mesh.read_bytes()
    nodes, faces, names = read_sections(data)
    ncell = max(int(max(a[:, 3].max(), a[:, 4].max())) for _, a in faces)
    cell_nodes = [set() for _ in range(ncell + 1)]
    cell_wall = {}
    for zid, a in faces:
        kind = names.get(zid, ("?", "?"))[0]
        for row in a:
            v = (row[0] - 1, row[1] - 1, row[2] - 1)
            for c in (row[3], row[4]):
                if c > 0:
                    cell_nodes[c].update(v)
                    if kind in ("wall", "overset"):
                        cell_wall.setdefault(int(c), set()).add(names[zid][1])
    tets = np.array([sorted(s) for s in cell_nodes[1:]], dtype=np.int64)
    if tets.shape[1] != 4:
        raise SystemExit("non-tet cells present")
    p = nodes[tets]  # (n,4,3)
    vol = np.abs(np.einsum("ij,ij->i", p[:, 1] - p[:, 0], np.cross(p[:, 2] - p[:, 0], p[:, 3] - p[:, 0]))) / 6
    # Orthogonal-quality proxy: for each face, cos between face normal and centroid->face-centroid vector.
    cen = p.mean(axis=1)
    q = np.ones(len(p))
    for f in ((1, 2, 3), (0, 2, 3), (0, 1, 3), (0, 1, 2)):
        a, b, c = p[:, f[0]], p[:, f[1]], p[:, f[2]]
        nrm = np.cross(b - a, c - a)
        fc = (a + b + c) / 3
        d = fc - cen
        cos = np.abs(np.einsum("ij,ij->i", nrm, d)) / (np.linalg.norm(nrm, axis=1) * np.linalg.norm(d, axis=1) + 1e-300)
        q = np.minimum(q, cos)
    order = np.argsort(q)[:top]
    rows = []
    for k in order:
        cid = int(k) + 1
        rows.append({"cell": cid, "q_proxy": float(q[k]), "volume_m3": float(vol[k]),
                     "centroid_m": [round(float(x), 6) for x in cen[k]],
                     "touches": sorted(cell_wall.get(cid, []))})
    hist = {t: int((q < t).sum()) for t in (1e-4, 1e-3, 1e-2, 5e-2)}
    report = {"mesh": str(mesh), "cells": int(len(p)), "proxy": "min |cos(face normal, centroid->face centroid)| per tet",
              "count_below": hist, "worst": rows}
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"cells": len(p), "count_below": hist, "worst": rows[:8]}, indent=1))


if __name__ == "__main__":
    main()

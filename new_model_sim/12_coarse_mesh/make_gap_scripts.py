"""Write sc_gap_faces_v11.py and sc_gap_faces_v5_redo.py from the v5 script, changing only the three
path constants and moving the overwrite guard before the first json write."""
from pathlib import Path

ROOT = Path("D:/work4/new_model_sim")
base = (ROOT / "09_seal_j2j3" / "sc_gap_faces_v5.py").read_text(encoding="utf-8")
lines = base.splitlines(keepends=True)
keys = ("SCDOC =", "OUT_SCDOC =", "OUT_JSON =")
idx = {k: next(i for i, l in enumerate(lines) if l.startswith(k)) for k in keys}
guard_old = "    if os.path.exists(OUT_JSON) or os.path.exists(OUT_SCDOC):\n        raise Exception('Refusing to overwrite')\n"
assert guard_old in base


def mk(paths, out_name):
    ls = list(lines)
    for k, p in zip(keys, paths):
        ls[idx[k]] = "%s r'%s'\n" % (k, str(p).replace("/", "\\"))
    s = "".join(ls).replace(guard_old, "", 1)
    s = s.replace("\ntry:\n", "\nif os.path.exists(OUT_JSON) or os.path.exists(OUT_SCDOC):\n"
                  "    raise SystemExit('Refusing to overwrite')\ntry:\n", 1)
    (ROOT / "12_coarse_mesh" / out_name).write_text(s, encoding="utf-8")


M = ROOT / "12_coarse_mesh"
mk([M / "fluid_domain_v11.scdoc", M / "fluid_domain_v11g.scdoc", M / "gap_faces_v11.json"], "sc_gap_faces_v11.py")
mk([ROOT / "09_seal_j2j3" / "fluid_domain_v5.scdoc", M / "fluid_domain_v5g_REDO.scdoc", M / "gap_faces_v5_REDO.json"],
   "sc_gap_faces_v5_redo.py")

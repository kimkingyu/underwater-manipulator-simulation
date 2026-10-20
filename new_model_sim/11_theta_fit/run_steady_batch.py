"""Run every steady case of cases.json on base THETA_BASE (default b5) one after another, skipping cases
whose runs/CASE_result.json already exists; export N*m loads after each. Log: runs/steady_batch.log"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
ANSYS_PY = r"D:\Program Files\ANSYS Inc\v242\commonfiles\CPython\3_10\winx64\Release\python\python.exe"
PY = r"C:\Users\admin\AppData\Local\Programs\Python\Python312\python.exe"
GROUP = sys.argv[1] if len(sys.argv) > 1 else "steady"     # steady | oscillation
cases = dict((k, v) for k, v in json.loads((D / "cases.json").read_text(encoding="utf-8"))[GROUP].items()
             if not k.startswith("_"))
log = D / "runs" / ("%s_batch.log" % GROUP)
env = dict(os.environ, THETA_BASE=os.environ.get("THETA_BASE", "b5"))


def w(s):
    with open(log, "a", encoding="utf-8") as fp:
        fp.write(time.strftime("%H:%M:%S ") + s + "\n")


w("start base=%s cases=%d" % (env["THETA_BASE"], len(cases)))
for cid in cases:
    if (D / "runs" / ("%s_result.json" % cid)).exists():
        w("skip %s (exists)" % cid)
        continue
    t0 = time.time()
    p = subprocess.run([ANSYS_PY, "-W", "ignore", str(D / "fluent_run.py"), cid, "18"], cwd=str(D), env=env,
                       capture_output=True, text=True, errors="replace")
    tail = (p.stdout or "").strip().splitlines()[-1:] or [""]
    w("%s rc=%s %.0fs %s" % (cid, p.returncode, time.time() - t0, tail[0][:300]))
    if GROUP == "steady" and (D / "runs" / ("%s_result.json" % cid)).exists():
        e = subprocess.run([PY, str(D / "export_loads.py"), cid], cwd=str(D), capture_output=True, text=True)
        w("  export rc=%s" % e.returncode)
w("done")

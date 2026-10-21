"""Steady cases at the extra arm poses, one after another (18 cores each).
Pose -> base case / run dir: qb -> bqb / runs_qb, qc -> bqc / runs_qc, qd -> bqd / runs_qd.
Skips cases whose RUNDIR/CASE_result.json exists. Log: runs_pose_batch.log"""
import os
import subprocess
import time
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
ANSYS_PY = r"D:\Program Files\ANSYS Inc\v242\commonfiles\CPython\3_10\winx64\Release\python\python.exe"
CASES = ["T_px_08", "T_nx_08", "T_py_08", "T_ny_08", "T_pz_08", "T_nz_08", "T_px_03", "T_py_03"]
POSES = ["qc", "qd", "qb"]
log = D / "runs_pose_batch.log"


def w(s):
    with open(log, "a", encoding="utf-8") as fp:
        fp.write(time.strftime("%m-%d %H:%M:%S ") + s + "\n")


w("start poses=%s cases=%s" % (POSES, CASES))
for pose in POSES:
    run = D / ("runs_" + pose)
    run.mkdir(exist_ok=True)
    env = dict(os.environ, THETA_BASE="b" + pose, THETA_RUNDIR="runs_" + pose)
    for cid in CASES:
        if (run / ("%s_result.json" % cid)).exists():
            w("skip %s/%s (exists)" % (pose, cid))
            continue
        t0 = time.time()
        with open(run / ("%s.stdout" % cid), "w", encoding="utf-8", errors="replace") as fp:
            p = subprocess.run([ANSYS_PY, "-W", "ignore", str(D / "fluent_run.py"), cid, "18"], cwd=str(D), env=env,
                               stdout=fp, stderr=subprocess.STDOUT)
        tail = (run / ("%s.stdout" % cid)).read_text(encoding="utf-8", errors="replace").strip().splitlines()[-1:] or [""]
        w("%s/%s rc=%s %.0fs %s" % (pose, cid, p.returncode, time.time() - t0, tail[0][:250]))
w("done")

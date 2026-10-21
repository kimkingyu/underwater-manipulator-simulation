"""Steady cases at an extra arm pose (runs_TAG/) -> theta_dataset_TAG.json, same conventions as
build_dataset_qb.py (posed joint axes/centres for tau, link-frame flink about the posed reference point).
Usage: python build_dataset_pose.py TAG q1 q2 q3
Takes every runs_TAG/T_*_result.json with status ok, or failed at the 2500-iteration cap with nothing
required failing (tail mean, flagged unsteady_mean), as build_dataset.py does."""
import json
import sys
from pathlib import Path

import bd_lib as bd

TAG = sys.argv[1]
Q = [float(x) for x in sys.argv[2:5]]
D = Path(r"D:\work4\new_model_sim\11_theta_fit")
RUN = D / ("runs_" + TAG)
CASES = json.loads((D / "cases.json").read_text(encoding="utf-8"))
O = CASES["origin_m"]
JC = [j[0] for j in bd.JOINTS]
JA = [j[1] for j in bd.JOINTS]
JW = [j[2] for j in bd.JOINTS]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A, x):
    return [sum(A[i][k] * x[k] for k in range(3)) for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def fk(q):
    Rc = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    Tc = [0.0, 0.0, 0.0]
    pj, aj, P, R = [], [], [], []
    for i in range(3):
        p = add(mv(Rc, bd.sub(JC[i], O)), Tc)
        a = mv(Rc, JA[i])
        Ri = bd.axang(a, q[i])
        Tc = add(mv(Ri, bd.sub(Tc, p)), p)
        Rc = mm(Ri, Rc)
        pj.append(p); aj.append(a)
        P.append(add(mv(Rc, bd.sub(bd.LINK_REF[i], O)), Tc))
        R.append(Rc)
    return pj, aj, P, R


PJ, AJ, P, R = fk(Q)


def sample(L):
    f, m = {}, {}
    for z in bd.WALLS:
        fg = [L["f%s(%s)" % (c, z)] for c in "xyz"]
        mg = [L["m%s(%s)" % (c, z)] for c in "xyz"]
        f[z] = fg
        m[z] = bd.sub(mg, bd.cross(O, fg))
    F = [sum(f[z][i] for z in bd.WALLS) for i in range(3)]
    M = [sum(m[z][i] for z in bd.WALLS) for i in range(3)]
    tau = [sum(bd.dot(AJ[j], bd.sub(m[z], bd.cross(PJ[j], f[z]))) for z in JW[j]) for j in range(3)]
    flink = []
    for i, zs in enumerate(bd.LINKS):
        fl = [sum(f[z][k] for z in zs) for k in range(3)]
        ml = [sum(m[z][k] for z in zs) for k in range(3)]
        flink += bd.mT(R[i], fl) + bd.mT(R[i], bd.sub(ml, bd.cross(P[i], fl)))
    return F, M, tau, flink


out = {"q": Q, "tag": TAG, "origin_m": O, "posed_joint_centre_rel_Ob": PJ, "posed_joint_axis": AJ,
       "posed_link_ref_rel_Ob": P, "link_R": R, "cases": []}
for rp in sorted(RUN.glob("T_*_result.json")):
    cid = rp.name[:-len("_result.json")]
    res = json.loads(rp.read_text(encoding="utf-8"))
    capped = (res.get("status") == "failed" and not res.get("failed_required") and res.get("iterations", 0) >= 2500)
    if res.get("status") != "ok" and not capped:
        continue
    hist = json.loads((RUN / ("%s_hist.json" % cid)).read_text(encoding="utf-8"))
    last = hist[-1]["iter"]
    tail = [h["loads"] for h in hist if h["iter"] > last - 200]
    Lm = dict((k, sum(t[k] for t in tail) / len(tail)) for k in tail[0])
    F, M, tau, flink = sample(Lm)
    out["cases"].append({"id": cid, "status": res.get("status"), "unsteady_mean": bool(capped),
                         "iterations": res.get("iterations"),
                         "nu": list(CASES["steady"][cid]["v"]) + [0, 0, 0],
                         "F": F, "M": M, "tau": tau, "flink": flink})
OUT_TAG = sys.argv[5] if len(sys.argv) > 5 else TAG     # e.g. qb8, keeping the committed 3-case qb file
dst = D / ("theta_dataset_%s.json" % OUT_TAG)
dst.write_text(json.dumps(out, indent=1), encoding="utf-8")
print(TAG, Q, [(c["id"], c["status"]) for c in out["cases"]])

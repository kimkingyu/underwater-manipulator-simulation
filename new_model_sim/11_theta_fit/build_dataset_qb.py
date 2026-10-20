"""Pose-qb steady cases (runs_qb/, base bqb, arm at q = [0.45, -0.50, -0.40]) -> theta_dataset_qb.json.

Same 27 channels and conventions as build_dataset.py, but at the posed arm:
  tau    distal walls about the POSED joint centre p_j, on the POSED joint axis a_j (the Fluent tau_j*
         reports use the q = 0 axes and are ignored here)
  flink  per link: force and moment about the POSED reference point P_i, expressed in LINK FRAME i
         (R_i' * body-axis value), which is what Y_uvms' Ylink predicts. At q = 0 link frame = body
         axes, so this matches build_dataset.py there.
Kinematics copied from matlab/uvms_link_kin.m (fk) and uvms_geometry.m. Steady tail = last 200 iterations."""
import json
import math
from pathlib import Path

import bd_lib as bd   # constants/helpers of build_dataset.py without its dataset write

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
RUN = D / "runs_qb"
Q = [0.45, -0.50, -0.40]
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
    """Posed joint centres/axes and link reference points (relative to O_b, body axes), link rotations."""
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
        f[z] = fg                                    # body axes = global axes (no mesh motion)
        m[z] = bd.sub(mg, bd.cross(O, fg))           # about O_b
    F = [sum(f[z][i] for z in bd.WALLS) for i in range(3)]
    M = [sum(m[z][i] for z in bd.WALLS) for i in range(3)]
    tau = [sum(bd.dot(AJ[j], bd.sub(m[z], bd.cross(PJ[j], f[z]))) for z in JW[j]) for j in range(3)]
    flink = []
    for i, zs in enumerate(bd.LINKS):
        fl = [sum(f[z][k] for z in zs) for k in range(3)]
        ml = [sum(m[z][k] for z in zs) for k in range(3)]
        flink += bd.mT(R[i], fl) + bd.mT(R[i], bd.sub(ml, bd.cross(P[i], fl)))
    return F, M, tau, flink


out = {"q": Q, "origin_m": O, "posed_joint_centre_rel_Ob": PJ, "posed_joint_axis": AJ,
       "posed_link_ref_rel_Ob": P, "link_R": R, "cases": []}
for cid in ("T_px_08", "T_py_08", "T_pz_08"):
    rp = RUN / ("%s_result.json" % cid)
    if not rp.exists():
        continue
    res = json.loads(rp.read_text(encoding="utf-8"))
    hist = json.loads((RUN / ("%s_hist.json" % cid)).read_text(encoding="utf-8"))
    last = hist[-1]["iter"]
    tail = [h["loads"] for h in hist if h["iter"] > last - 200]
    Lm = dict((k, sum(t[k] for t in tail) / len(tail)) for k in tail[0])
    F, M, tau, flink = sample(Lm)
    tau_q0axes = [Lm["tau_j%d" % j] for j in (1, 2, 3)]
    out["cases"].append({"id": cid, "status": res.get("status"), "iterations": res.get("iterations"),
                         "converged": res.get("converged"),
                         "nu": list(CASES["steady"][cid]["v"]) + [0, 0, 0],
                         "F": F, "M": M, "tau": tau, "flink": flink,
                         "tau_fluent_report_q0_axes": tau_q0axes})
(D / "theta_dataset_qb.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
print(json.dumps([(c["id"], c["status"], [round(x, 3) for x in c["F"] + c["tau"]]) for c in out["cases"]]))

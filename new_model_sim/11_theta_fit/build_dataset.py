"""Collect finished theta-fit CFD cases into theta_dataset.json for the MATLAB fit (stdlib only).

Body frame = CAD axes (+Y up), origin O_b = hull volume centroid (cases.json origin_m). Arm at the CAD
pose (q = 0). For every sample this writes, in BODY axes:
  nu     body velocity relative to the water [u v w p q r]  (steady: cases.json; osc: A*om*cos(om*t))
  nudot  its time derivative                                 (steady: 0;          osc: -A*om^2*sin(om*t))
  F, M   total fluid force on all 8 body walls and its moment about O_b
  tau    J1..J3 fluid torques (distal walls, about the joint centre, on the joint axis), recombined from
         the per-zone reports so that moving joints in the oscillation cases are handled exactly
  flink  fluid force on link 1 (l1), link 2 (l2), link 3 (link3 + jaw1 + jaw2 + pin4), 3 x 3 = 9 values
Oscillation: Fluent reports are in fixed global axes about the global origin while the whole mesh moves,
so per-zone loads are moved to the current O_b(t) and rotated into the body frame; first period dropped.
Only cases whose CASE_result.json has status ok are taken."""
import json
import math
from pathlib import Path

D = Path(r"D:\work4\new_model_sim\11_theta_fit")
RUN = D / "runs"
CASES = json.loads((D / "cases.json").read_text(encoding="utf-8"))
O = CASES["origin_m"]
WALLS = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
LINKS = [["wall_l1"], ["wall_l2"], ["wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]]
JOINTS = [([-4e-06, -0.0366, -0.460702], [0.0, 0.0, -1.0], WALLS[2:]),
          ([0.016686, -0.0666, -0.511202], [-1.0, 0.0, 0.0], WALLS[3:]),
          ([0.014796, -0.1226, -0.313202], [1.0, 0.0, 0.0], WALLS[4:])]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def rot(axis, ang):
    """Rotation matrix (rows) about a unit coordinate axis index 0..2."""
    c, s = math.cos(ang), math.sin(ang)
    if axis == 0:
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == 1:
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def axang(a, t):
    """Rotation matrix (rows) about unit vector a by angle t (Rodrigues)."""
    c, s, C = math.cos(t), math.sin(t), 1 - math.cos(t)
    x, y, z = a
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def mT(R, x):  # R^T x
    return [sum(R[k][i] * x[k] for k in range(3)) for i in range(3)]


def link_centroids():
    """Volume-weighted centroids of the link bodies (body_centroids.json, grid method)."""
    b = dict((x["name"], x) for x in json.loads((D / "body_centroids.json").read_text(encoding="utf-8"))["bodies"])
    groups = [["0:7955"], ["0:8167"], ["0:1425", "0:2393", "0:4783", "0:7716"]]
    out = []
    for g in groups:
        V = sum(b[n]["volume_m3"] for n in g)
        out.append([sum(b[n]["volume_m3"] * b[n]["centroid_m"][i] for n in g) / V for i in range(3)])
    return out


def sample(L, Ot, R):
    """Per-zone global loads L (moments about the global origin) -> body-frame quantities about O_b(t)."""
    f, m = {}, {}
    for z in WALLS:
        fg = [L["f%s(%s)" % (q, z)] for q in "xyz"]
        mg = [L["m%s(%s)" % (q, z)] for q in "xyz"]
        f[z] = mT(R, fg)
        m[z] = mT(R, sub(mg, cross(Ot, fg)))          # about O_b(t), body axes
    F = [sum(f[z][i] for z in WALLS) for i in range(3)]
    M = [sum(m[z][i] for z in WALLS) for i in range(3)]
    tau = []
    for c, a, zs in JOINTS:
        cb = sub(c, O)                                   # joint centre relative to O_b, body frame
        tau.append(sum(dot(a, sub(m[z], cross(cb, f[z]))) for z in zs))
    flink = []
    for zs in LINKS:
        flink += [sum(f[z][i] for z in zs) for i in range(3)]
    return F, M, tau, flink


cases = []
for group in ("steady", "oscillation"):
    for cid, spec in CASES[group].items():
        if cid.startswith("_"):
            continue
        rp = RUN / ("%s_result.json" % cid)
        if not rp.exists():
            continue
        res = json.loads(rp.read_text(encoding="utf-8"))
        # steady cases that hit the iteration cap with a periodically oscillating (wake) load are
        # accepted on their last-200-iteration mean when nothing failed; flagged unsteady_mean
        capped = (res.get("status") == "failed" and not res.get("failed_required")
                  and group == "steady" and res.get("iterations", 0) >= 2500)
        if res.get("status") != "ok" and not capped:
            continue
        rec = {"id": cid, "kind": group, "unsteady_mean": bool(capped), "t": [], "nu": [], "nudot": [], "F": [], "M": [], "tau": [], "flink": []}
        if group == "steady" and "w" in spec:
            # R_*: transient rigid rotation of the whole mesh with omega ramped in over rot_ramp_s.
            # Rotation angle th(t) = wm*(t^2/(2 ramp)) for t < ramp, wm*(t - ramp/2) after. Loads are
            # rotated into the body frame and averaged over the second half of the turn.
            hist = json.loads((RUN / ("%s_hist.json" % cid)).read_text(encoding="utf-8"))
            w = spec["w"]
            wm = math.sqrt(sum(c * c for c in w))
            ax = [c / wm for c in w]
            ramp = res.get("rot_ramp_s", 0.0)
            ang = lambda tt: wm * (tt * tt / (2 * ramp) if tt < ramp else tt - ramp / 2)
            th_end = ang(hist[-1]["t"])
            tail = [h for h in hist if h["t"] >= ramp and ang(h["t"]) >= th_end / 2]
            vals = [sum(sample(h["loads"], O, axang(ax, ang(h["t"]))), []) for h in tail]
            acc = [sum(v[i] for v in vals) / len(vals) for i in range(18)]
            rec["t"].append(0.0)
            rec["nu"].append([0.0, 0.0, 0.0] + list(w))
            rec["nudot"].append([0.0] * 6)
            rec["rot_tail_angle_rad"] = [ang(tail[0]["t"]), th_end]
            rec["tail_rel_std"] = dict(("q%d" % i, (sum((v[i] - acc[i]) ** 2 for v in vals) / len(vals)) ** 0.5
                                        / max(abs(acc[i]), 1e-9)) for i in (3, 4, 5, 6, 7, 8))
            rec["F"].append(acc[0:3]); rec["M"].append(acc[3:6])
            rec["tau"].append(acc[6:9]); rec["flink"].append(acc[9:18])
        elif group == "steady":
            # T_* (uniform current) and R_* (steady MRF rotation, loads at the instant frames coincide)
            nu = list(spec.get("v", [0, 0, 0])) + list(spec.get("w", [0, 0, 0]))
            hist = json.loads((RUN / ("%s_hist.json" % cid)).read_text(encoding="utf-8"))
            last = hist[-1]["iter"]
            tail = [h["loads"] for h in hist if h["iter"] > last - 200]
            Lm = dict((k, sum(t[k] for t in tail) / len(tail)) for k in tail[0])
            rec["tail_iters"] = [h["iter"] for h in hist if h["iter"] > last - 200]
            rec["tail_rel_std"] = dict((k, (sum((t[k] - Lm[k]) ** 2 for t in tail) / len(tail)) ** 0.5
                                        / max(abs(Lm[k]), 1e-9)) for k in ("fx", "fy", "fz", "tau_j1", "tau_j2", "tau_j3"))
            res["loads_final"] = Lm
            F, M, tau, fl = sample(Lm, O, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            rec["t"].append(0.0)
            rec["nu"].append(nu)
            rec["nudot"].append([0.0] * 6)
            rec["tau_report"] = [res["loads_final"]["tau_j%d" % j] for j in (1, 2, 3)]
            for k, v in (("F", F), ("M", M), ("tau", tau), ("flink", fl)):
                rec[k].append(v)
        else:
            hist = json.loads((RUN / ("%s_hist.json" % cid)).read_text(encoding="utf-8"))
            dof, A, T = spec["dof"], spec["amp"], spec["period"]
            om = 2 * math.pi / T
            ramp = res.get("osc_ramp_s", 0.0)
            rate = lambda tt: A * om * math.cos(om * tt) * (min(tt / ramp, 1.0) if ramp > 0 else 1.0)

            def disp(tt, n=4000):           # x(t) = integral_0^t rate (Simpson), same law as Fluent
                if tt <= 0:
                    return 0.0
                hh = tt / n
                s_ = rate(0) + rate(tt) + sum((4 if k % 2 else 2) * rate(k * hh) for k in range(1, n))
                return s_ * hh / 3
            bad = [h["t"] for h in hist if any(v != v for v in h["loads"].values())]
            if bad:
                print("skip %s: NaN loads from t = %s" % (cid, bad[0]))
                continue
            # Body-frame loads over the last two periods, projected on sin(om t): the component in phase
            # with the acceleration -A om^2 sin(om t). One sample per case: nu = 0, nudot = unit
            # acceleration in dof, loads = in-phase amplitude / (A om^2) (sign: load per +1 accel).
            # The out-of-phase part (laminar, ~velocity) is kept for information only.
            rows = []
            for h in hist:
                t = h["t"]
                if t <= T + 1e-9 or t < ramp:
                    continue
                x = disp(t)
                Ot = list(O)
                if dof < 3:
                    Ot[dof] += x
                    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
                else:
                    R = rot(dof - 3, x)
                F, M, tau, fl = sample(h["loads"], Ot, R)
                rows.append((t, F + M + tau + fl))
            n = len(rows)
            sinp = [sum(v[i] * math.sin(om * t) for t, v in rows) * 2 / n for i in range(18)]
            cosp = [sum(v[i] * math.cos(om * t) for t, v in rows) * 2 / n for i in range(18)]
            g = -1.0 / (A * om * om)          # load = -MA * a, a = -A om^2 sin -> sin amp = MA A om^2
            unit = [g * c for c in sinp]      # fluid load per unit +acceleration
            nd = [0.0] * 6
            nd[dof] = 1.0
            rec["t"].append(0.0)
            rec["nu"].append([0.0] * 6)
            rec["nudot"].append(nd)
            rec["F"].append(unit[0:3]); rec["M"].append(unit[3:6])
            rec["tau"].append(unit[6:9]); rec["flink"].append(unit[9:18])
            rec["osc_samples"] = n
            rec["out_of_phase_per_unit_velocity"] = [-c / (A * om) for c in cosp]
            if dof >= 3:
                # kim 2026-10-20: rotational damping from the velocity-phase part of the rotary
                # oscillations (laminar). Second sample: nu = peak rate A*om in dof, nudot = 0,
                # loads = cos amplitude (in phase with the rate A om cos(om t)).
                nu = [0.0] * 6
                nu[dof] = A * om
                rec["t"].append(0.0)
                rec["nu"].append(nu)
                rec["nudot"].append([0.0] * 6)
                rec["F"].append(cosp[0:3]); rec["M"].append(cosp[3:6])
                rec["tau"].append(cosp[6:9]); rec["flink"].append(cosp[9:18])
                rec["velocity_sample_laminar"] = True
            rec["spec"] = spec
        cases.append(rec)

geo = {"origin_m": O, "joint_centre_m": [j[0] for j in JOINTS], "joint_axis": [j[1] for j in JOINTS],
       "link_ref_m": link_centroids(), "links": LINKS,
       "note": "body frame = CAD axes, origin O_b; link frames = CAD axes at q = 0, origin at link_ref_m"}
(D / "theta_dataset.json").write_text(json.dumps({"geometry": geo, "cases": cases}, indent=1), encoding="utf-8")
chk = [max(abs(a - b) for a, b in zip(c["tau"][0], c["tau_report"])) for c in cases if "tau_report" in c]
print(json.dumps({"cases": [c["id"] for c in cases], "n_samples": sum(len(c["t"]) for c in cases),
                  "steady_tau_recombine_maxdiff": max(chk) if chk else None,
                  "link_ref_m": geo["link_ref_m"]}, indent=1))

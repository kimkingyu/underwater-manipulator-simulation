"""Independent, freeze-gated validation runner. --check is strictly solver-free."""
import argparse
import contextlib
import hashlib
import json
import logging
import math
import os
import re
import sys
import time
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
STAGE = HERE.parent
DEFAULT_CONFIG = HERE / "validation_cases.json"
BODY = ["wall_auv", "wall_base0", "wall_l1", "wall_l2", "wall_link3", "wall_jaw1", "wall_jaw2", "wall_pin4"]
REPS = ["fx", "fy", "fz", "mx", "my", "mz", "tau_j1", "tau_j2", "tau_j3"]
BOX_N = {"xmin": [-1, 0, 0], "xmax": [1, 0, 0], "ymin": [0, -1, 0], "ymax": [0, 1, 0], "zmin": [0, 0, -1], "zmax": [0, 0, 1]}
CHANNELS = set(REPS + [k + "(" + z + ")" for k in REPS[:6] for z in BODY])
MODES = {"steady_translation", "oscillation_translation"}
MASS_FLOW_REPORT_TYPE = "flux-massflow"  # Confirmed by the actual Fluent 24.2 API probe.
# Only atomic-file replacement is retried, never a solver action or protocol failure.
JSON_REPLACE_RETRY_DELAYS_S = (0.02, 0.05, 0.10, 0.20, 0.40)
JSON_REPLACE_RETRY_WINERRORS = frozenset((5, 32, 33))
_LOG = logging.getLogger(__name__)


class ProtocolError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def object_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def write_json(path, value):
    """Replace a complete JSON file; never truncate/delete the previous snapshot.

    Windows winerror 5 can also mean permanent access denial, so its retries are
    bounded just like sharing/lock errors 32/33: at most six replace calls and
    0.77 seconds of backoff. Other errors and write/flush failures propagate at
    once. On exhausted replacement retries the original exception is re-raised
    and the uniquely named, complete pending file is retained for diagnosis.
    """
    path = Path(path)
    payload = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)
    # A sibling keeps rename on one filesystem; a unique name cannot clobber a
    # previous failed write or another writer's candidate. Close before replace.
    tmp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    with tmp.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    for attempt in range(len(JSON_REPLACE_RETRY_DELAYS_S) + 1):
        try:
            os.replace(str(tmp), str(path))
            return
        except OSError as exc:
            if getattr(exc, "winerror", None) not in JSON_REPLACE_RETRY_WINERRORS or attempt == len(JSON_REPLACE_RETRY_DELAYS_S):
                raise
            delay = JSON_REPLACE_RETRY_DELAYS_S[attempt]
            _LOG.warning("Atomic JSON replace blocked (winerror=%s): retry %d/%d in %.2fs; target=%s", exc.winerror,
                         attempt + 1, len(JSON_REPLACE_RETRY_DELAYS_S), delay, path)
            time.sleep(delay)


def finite_vector(value, name):
    if not isinstance(value, list) or len(value) != 3 or any(type(x) not in (float, int) or not math.isfinite(x) for x in value):
        raise ProtocolError(name + " must be a finite 3-vector")
    return value


def close_vector(a, b, tol=1e-12):
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def load_config(path=DEFAULT_CONFIG):
    path = Path(path)
    cfg = json.loads(path.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != 1 or cfg.get("origin_m") != [0.0, 0.29386, -0.06630]:
        raise ProtocolError("schema/original Ob mismatch")
    if cfg.get("body_walls") != BODY:
        raise ProtocolError("body wall order mismatch")
    if set(cfg.get("poses", {})) != {"q0", "qe"} or cfg["poses"]["q0"]["q"] != [0, 0, 0] or cfg["poses"]["qe"]["q"] != [0.225, -0.25, -0.20]:
        raise ProtocolError("approved q0/qe pose mismatch")
    expected_physics = {"density_kg_m3": 1025.0, "dynamic_viscosity_Pa_s": 0.00105,
                        "steady_viscous_model": "k-omega-sst", "inlet_turbulent_intensity": 0.01,
                        "inlet_turbulent_viscosity_ratio": 10.0, "box_parallel_faces": "free_slip",
                        "oscillation_box_faces": "pressure-outlet"}
    if cfg.get("physics") != expected_physics:
        raise ProtocolError("approved physical settings changed")
    if cfg.get("joint_centres_CAD0_m") != [[-0.000004, -0.0366, -0.460702], [0.016686, -0.0666, -0.511202], [0.014796, -0.1226, -0.313202]] or cfg.get("joint_axes_CAD0") != [[0, 0, -1], [-1, 0, 0], [1, 0, 0]]:
        raise ProtocolError("reference joint geometry mismatch")
    s = cfg["solver"]
    fixed = {"processor_count": 18, "parallel_cases": 1, "steady_max_iterations": 2500,
             "steady_chunk_iterations": 25, "steady_window_iterations": 200,
             "steady_drift_tolerance": 0.005, "steady_min_iterations": 400,
             "automatic_retries": 0, "max_explicit_repair_retries": 1,
             "oscillation_time_steps": 240, "oscillation_time_step_s": 0.025, "oscillation_max_iterations_per_step": 30}
    if any(s.get(k) != v for k, v in fixed.items()):
        raise ProtocolError("approved serial solver budget/settings changed")
    expected = {"T_V_%s_%s_%s055" % (pose, plane, sign)
                for pose in ("q0", "qe") for plane in ("XY", "YZ") for sign in ("p", "n")}
    expected.add("O_V_q0_YZ")
    cases = cfg.get("cases", [])
    if len(cases) != 9 or {c.get("id") for c in cases} != expected:
        raise ProtocolError("must contain exactly the nine approved case IDs")
    for c in cases:
        if c.get("mode") not in MODES:
            raise ProtocolError("unknown mode: " + str(c.get("mode")))
        if c.get("pose_id") not in cfg["poses"] or c["id"].split("_")[2] != c.get("pose_id") or c.get("q") != cfg["poses"][c["pose_id"]]["q"]:
            raise ProtocolError("case pose mismatch")
        d = finite_vector(c.get("direction"), "direction")
        if abs(sum(x * x for x in d) - 1) > 1e-12:
            raise ProtocolError("direction must be unit length")
        if c["mode"] == "steady_translation":
            if c["id"].startswith("O_") or c.get("speed_m_s") != 0.55:
                raise ProtocolError("steady speed/ID mismatch")
            plane = c["id"].split("_")[3]
            sign = -1 if c["id"].split("_")[4].startswith("n") else 1
            raw = [2, 1, 0] if plane == "XY" else [0, 1, 2]
            if not close_vector(d, [sign * x / math.sqrt(5) for x in raw]):
                raise ProtocolError("approved steady direction mismatch")
            v = finite_vector(c.get("body_velocity_m_s"), "body velocity")
            w = finite_vector(c.get("water_velocity_m_s"), "water velocity")
            if not close_vector(v, [0.55 * x for x in d]) or not close_vector(w, [-x for x in v]):
                raise ProtocolError("body/water velocity sign or magnitude mismatch")
        else:
            settings = {"id": "O_V_q0_YZ", "pose_id": "q0", "amplitude_m": 0.02, "period_s": 2.0,
                        "ramp_s": 0.5, "time_step_s": 0.025, "time_steps": 240,
                        "max_iterations_per_step": 30, "viscous_model": "laminar"}
            if any(c.get(k) != v for k, v in settings.items()) or not close_vector(d, [0, 1 / math.sqrt(2), 1 / math.sqrt(2)]):
                raise ProtocolError("approved oscillation settings mismatch")
    return cfg


def safe_stage_path(root, relative):
    if not isinstance(relative, str) or not relative or "\\" in relative or Path(relative).is_absolute() or ":" in relative:
        raise ProtocolError("frozen path must be a POSIX path relative to stage 14")
    if ".." in Path(relative).parts:
        raise ProtocolError("frozen path traversal rejected")
    result = (root / relative).resolve()
    try:
        result.relative_to(root.resolve())
    except ValueError as exc:
        raise ProtocolError("frozen path escapes stage 14") from exc
    return result


def verify_freeze(manifest_path, config_path=DEFAULT_CONFIG, stage_root=STAGE):
    root, config_path = Path(stage_root).resolve(), Path(config_path).resolve()
    expected_path = root / "frozen/blind_protocol.json"
    if not manifest_path or Path(manifest_path).resolve() != expected_path.resolve() or not expected_path.is_file():
        raise ProtocolError("--freeze-manifest must name the existing 14/frozen/blind_protocol.json")
    manifest = json.loads(expected_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "frozen_before_cfd":
        raise ProtocolError("protocol is not frozen_before_cfd")
    cfg = load_config(config_path)
    config_digest = sha256(config_path)
    if manifest.get("case_config_sha256") != config_digest:
        raise ProtocolError("case_config_sha256 mismatch")
    ids = [c["id"] for c in cfg["cases"]]
    approved = manifest.get("approved_case_ids")
    if not isinstance(approved, list) or len(approved) != len(ids) or set(approved) != set(ids):
        raise ProtocolError("approved_case_ids must exactly match current configuration")
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        raise ProtocolError("freeze files must be nonempty")
    paths = set()
    for item in files:
        if not isinstance(item, dict) or not {"path", "sha256"}.issubset(item):
            raise ProtocolError("invalid frozen file record")
        path = safe_stage_path(root, item.get("path"))
        digest = item.get("sha256")
        if path in paths or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ProtocolError("duplicate path or invalid frozen digest")
        paths.add(path)
        if not path.is_file() or sha256(path) != digest:
            raise ProtocolError("frozen file missing/hash mismatch: " + item["path"])
    if config_path not in paths:
        raise ProtocolError("case configuration is absent from frozen files")
    return {"path": str(expected_path), "sha256": sha256(expected_path), "case_config_sha256": config_digest}


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def rotate(v, axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    cr, dot = cross(axis, v), sum(a * b for a, b in zip(axis, v))
    return [c * v[i] + s * cr[i] + (1 - c) * dot * axis[i] for i in range(3)]


def posed_joints(cfg, q, displacement=None):
    centres, axes = cfg["joint_centres_CAD0_m"], cfg["joint_axes_CAD0"]
    shift = displacement or [0, 0, 0]
    rows = []
    for j in range(3):
        centre, axis = list(centres[j]), list(axes[j])
        for k in reversed(range(j)):
            centre = [x + y for x, y in zip(rotate([centre[i] - centres[k][i] for i in range(3)], axes[k], q[k]), centres[k])]
            axis = rotate(axis, axes[k], q[k])
        rows.append({"name": "tau_j%d" % (j + 1), "centre_m": [centre[i] + shift[i] for i in range(3)],
                     "axis": axis, "zones": BODY[2 + j:]})
    return rows


def oscillation_state(spec, t):
    amp, period, ramp = spec["amplitude_m"], spec["period_s"], spec["ramp_s"]
    omega = 2 * math.pi / period
    speed = amp * omega * math.cos(omega * t) * min(t / ramp, 1.0)
    acceleration = amp * omega * (math.cos(omega * t) / ramp - omega * t / ramp * math.sin(omega * t)) if t < ramp else -amp * omega ** 2 * math.sin(omega * t)
    def xr(t0):
        return amp / ramp * (t0 * math.sin(omega * t0) + (math.cos(omega * t0) - 1) / omega)
    disp = xr(t) if t <= ramp else xr(ramp) + amp * (math.sin(omega * t) - math.sin(omega * ramp))
    return {"body_velocity_m_s": [speed * x for x in spec["direction"]],
            "water_relative_velocity_m_s": [-speed * x for x in spec["direction"]],
            "body_acceleration_m_s2": [acceleration * x for x in spec["direction"]],
            "displacement_m": [disp * x for x in spec["direction"]]}


def base_status(cfg, case, stage_root=STAGE):
    pose = cfg["poses"][case["pose_id"]]
    base = (Path(stage_root) / pose["base_case"]).resolve()
    digest = sha256(base) if base.is_file() and base.stat().st_size > 0 else None
    return {"path": str(base), "exists": digest is not None, "expected_sha256": pose.get("base_sha256"),
            "sha256": digest, "ready": digest is not None and digest == pose.get("base_sha256")}


def identity_for(case, freeze, base):
    return {"case_id": case["id"], "case_spec_sha256": object_hash(case),
            "case_config_sha256": freeze["case_config_sha256"], "freeze_manifest_sha256": freeze["sha256"],
            "base_sha256": base["sha256"], "runner_sha256": sha256(Path(__file__))}


def valid_completed(attempt, expected):
    """Started files are never success. Verify paired terminal state and artifact bytes."""
    attempt = Path(attempt)
    try:
        report = json.loads((attempt / "result.json").read_text(encoding="utf-8"))
        if report.get("status") != "completed" or report.get("identity") != expected or report.get("failed_required") != []:
            return False
        if report.get("execution_complete") is not True or not report.get("terminal_state", {}).get("id"):
            return False
        artifacts = report["artifacts"]
        for key, filename in (("start_case", "start.cas.h5"), ("final_case", "final.cas.h5"), ("final_data", "final.dat.h5"), ("history", "history.json"), ("transcript", "solver.trn")):
            record = artifacts[key]
            p = attempt / filename
            if record["path"] != filename or not p.is_file() or not p.stat().st_size or sha256(p) != record["sha256"]:
                return False
            if key in ("final_case", "final_data") and record.get("state_id") != report["terminal_state"]["id"]:
                return False
        history = json.loads((attempt / "history.json").read_text(encoding="utf-8"))
        if not history or len(history) != report["terminal_state"]["sample_count"]:
            return False
        state = report["terminal_state"]
        if report["mode"] == "steady_translation":
            if state["iterations"] != history[-1]["iter"] or state["iterations"] > 2500:
                return False
            if not (report.get("converged") or state["iterations"] == 2500):
                return False
        elif report["mode"] == "oscillation_translation":
            if len(history) != 240 or state.get("time_steps") != 240 or abs(history[-1]["t"] - 6.0) > 1e-10:
                return False
        else:
            return False
        return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def select_attempt(run_root, case_id, expected, repair_reason=None):
    parent = Path(run_root) / case_id
    attempts = sorted(parent.glob("attempt_[0-9][0-9][0-9]")) if parent.exists() else []
    for p in reversed(attempts):
        if valid_completed(p, expected):
            return p, True
    if attempts:
        if not repair_reason or not repair_reason.strip():
            raise ProtocolError("existing incomplete/stale attempt: explicit --repair-reason required; no automatic retry")
        if len(attempts) >= 2:
            raise ProtocolError("the one explicit repair retry has already been used")
    elif repair_reason:
        raise ProtocolError("repair-reason is only for an existing failed/interrupted/stale attempt")
    target = parent / ("attempt_%03d" % (len(attempts) + 1))
    target.mkdir(parents=True, exist_ok=False)
    return target, False


@contextlib.contextmanager
def serial_lock(run_root):
    root = Path(run_root)
    root.mkdir(parents=True, exist_ok=True)
    stream = (root / ".serial.lock").open("a+b")
    if stream.tell() == 0:
        stream.write(b"0")
        stream.flush()
    stream.seek(0)
    try:
        if os.name == "nt":
            import msvcrt
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        stream.close()
        raise ProtocolError("another validation case owns the serial solver lock") from exc
    try:
        yield
    finally:
        stream.close()


def check_case(case_id, config_path=DEFAULT_CONFIG, manifest_path=None, stage_root=STAGE):
    cfg = load_config(config_path)
    case = next((c for c in cfg["cases"] if c["id"] == case_id), None)
    if case is None:
        raise ProtocolError("unknown case " + case_id)
    freeze = None
    error = None
    if manifest_path:
        freeze = verify_freeze(manifest_path, config_path, stage_root)
    else:
        error = "not supplied; real execution blocked"
    base = base_status(cfg, case, stage_root)
    result = {"case_id": case_id, "mode": case["mode"], "check_only": True, "solver_launched": False,
              "case_config_sha256": sha256(config_path), "freeze": freeze, "freeze_blocker": error,
              "base": base, "posed_joint_reports": posed_joints(cfg, case["q"]), "ready_to_run": bool(freeze and base["ready"])}
    if freeze and base["ready"]:
        expected = identity_for(case, freeze, base)
        runs = Path(stage_root) / "cfd/runs" / case_id
        result["verified_completed_attempts"] = [p.name for p in sorted(runs.glob("attempt_[0-9][0-9][0-9]")) if valid_completed(p, expected)]
    return result


def flatten_report(raw):
    out = {}
    for row in raw:
        if not isinstance(row, dict):
            raise RuntimeError("unexpected Fluent report response")
        for key, value in row.items():
            while isinstance(value, (list, tuple)):
                value = value[0]
            out[key] = float(value)
    if any(not math.isfinite(v) for v in out.values()):
        raise RuntimeError("non-finite report")
    return out


def conservation(loads, mass, joints):
    sums = {k: loads[k] - sum(loads[k + "(" + z + ")"] for z in BODY) for k in REPS[:6]}
    torque_error = {}
    for j in joints:
        force = [sum(loads[k + "(" + z + ")"] for z in j["zones"]) for k in REPS[:3]]
        moment = [sum(loads[k + "(" + z + ")"] for z in j["zones"]) for k in REPS[3:6]]
        shifted = [x - y for x, y in zip(moment, cross(j["centre_m"], force))]
        torque_error[j["name"]] = loads[j["name"]] - sum(a * b for a, b in zip(j["axis"], shifted))
    net = sum(mass.values())
    return {"zone_sum_residual": sums, "joint_projection_residual_Nm": torque_error,
            "mass_flow_kg_s_by_boundary": mass, "mass_imbalance_kg_s": net,
            "mass_imbalance_fraction": abs(net) / max(0.5 * sum(abs(v) for v in mass.values()), 1e-12)}


def sample_stats(rows):
    values = [h["loads"] for h in rows]
    mean = {k: sum(v[k] for v in values) / len(values) for k in CHANNELS}
    sd = {k: math.sqrt(sum((v[k] - mean[k]) ** 2 for v in values) / len(values)) for k in CHANNELS}
    return {"sample_count": len(rows), "mean": mean, "population_sd": sd}


def load_drift(history, n):
    now, previous = sample_stats(history[-n:]), sample_stats(history[-2 * n:-n])
    kind = lambda k: "tau" if k.startswith("tau") else k[0]
    scales = {g: max(abs(v) for k, v in now["mean"].items() if kind(k) == g) for g in ("f", "m", "tau")}
    errors = {k: abs(now["mean"][k] - previous["mean"][k]) / max(scales[kind(k)], 1e-9) for k in CHANNELS}
    return {"maximum": max(errors.values()), "by_kind": {g: max(v for k, v in errors.items() if kind(k) == g) for g in scales}, "last_window": now}


def cycle_stats(history, samples_per_cycle):
    cycles = [sample_stats(history[i:i + samples_per_cycle]) for i in range(0, len(history), samples_per_cycle)]
    second, third = history[-2 * samples_per_cycle:-samples_per_cycle], history[-samples_per_cycle:]
    rmse = {k: math.sqrt(sum((a["loads"][k] - b["loads"][k]) ** 2 for a, b in zip(second, third)) / samples_per_cycle) for k in CHANNELS}
    return {"cycles": cycles, "phase_matched_cycle2_vs3_RMSE": rmse, "periodic_convergence_assumed": False}


def execute(case_id, config_path=DEFAULT_CONFIG, manifest_path=None, repair_reason=None):
    # This gate precedes any directory creation, import of pyfluent, initialization or solve.
    freeze = verify_freeze(manifest_path, config_path)
    cfg = load_config(config_path)
    case = next((c for c in cfg["cases"] if c["id"] == case_id), None)
    if case is None or case["mode"] not in MODES:
        raise ProtocolError("unknown case or mode")
    base = base_status(cfg, case)
    if not base["ready"]:
        raise ProtocolError("base case missing or not bound to its configured SHA256")
    expected = identity_for(case, freeze, base)
    with serial_lock(HERE / "runs"):
        attempt, skip = select_attempt(HERE / "runs", case_id, expected, repair_reason)
        if skip:
            return {"case_id": case_id, "status": "skipped_verified_completed", "attempt": str(attempt)}
        return solve_attempt(cfg, case, freeze, base, expected, attempt, repair_reason, config_path, manifest_path)


def solve_attempt(cfg, case, freeze, base, expected, attempt, repair_reason, config_path, manifest_path):
    report = {"status": "started", "case_id": case["id"], "mode": case["mode"], "spec": case,
              "identity": expected, "repair_reason": repair_reason, "origin_m": cfg["origin_m"],
              "processor_count": 18, "steps": [], "failed_required": [], "execution_complete": False,
              "joint_reports_initial": posed_joints(cfg, case["q"]), "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    history, solver = [], None
    def save():
        write_json(attempt / "history.json", history)
        write_json(attempt / "result.json", report)
    def required(name, function):
        row = {"name": name, "required": True, "ok": False}
        report["steps"].append(row)
        start = time.monotonic()
        try:
            value = function()
            row["ok"] = True
            return value
        except BaseException as exc:
            row["error"] = repr(exc)
            report["failed_required"].append(name)
            raise
        finally:
            row["seconds"] = round(time.monotonic() - start, 3)
            save()
    def val(x):
        return {"option": "value", "value": float(x)}
    save()
    try:
        # Recheck all frozen bytes immediately before an actual launch.
        if verify_freeze(manifest_path, config_path) != freeze or sha256(base["path"]) != base["sha256"]:
            raise ProtocolError("inputs changed after preflight")
        sys.path.insert(0, str(STAGE.parent / "04_workbench"))
        from run_wb_fullenv import full_environment
        os.environ.update(full_environment())
        import ansys.fluent.core as pyfluent
        solver = required("launch_solver", lambda: pyfluent.launch_fluent(product_version="24.2", mode="solver", precision="double",
                          processor_count=18, additional_arguments="-t18 -mpi=msmpi", ui_mode=pyfluent.UIMode.NO_GUI_OR_GRAPHICS,
                          start_timeout=300, cleanup_on_exit=True, start_transcript=False, cwd=str(attempt)))
        required("start_transcript", lambda: solver.transcript.start(str(attempt / "solver.trn")))
        required("read_base", lambda: solver.file.read_case(file_name=base["path"]))
        setup, bc = solver.setup, solver.setup.boundary_conditions
        fluids = list(setup.cell_zone_conditions.fluid.keys())
        if len(fluids) != 1 or set(BODY) - set(bc.wall.keys()):
            raise RuntimeError("base fluid/body-wall topology mismatch")
        fluid = setup.cell_zone_conditions.fluid[fluids[0]]
        rd = solver.solution.report_definitions
        def joint_reports(displacement=None):
            joints = posed_joints(cfg, case["q"], displacement)
            for j in joints:
                m = rd.moment[j["name"]]
                m.zones, m.mom_center, m.mom_axis = j["zones"], j["centre_m"], j["axis"]
                m.report_output_type = "Moment"
            report["joint_reports_current"] = joints
            return joints
        def turbulence(obj, back=False):
            t = obj.turbulence
            t.turbulence_specification = "Intensity and Viscosity Ratio"
            if back:
                t.backflow_turbulent_intensity, t.backflow_turbulent_viscosity_ratio = 0.01, 10
            else:
                t.turbulent_intensity, t.turbulent_viscosity_ratio = 0.01, 10
        def boundary_conditions():
            roles = {}
            if case["mode"] == "steady_translation":
                setup.models.viscous.model, setup.models.viscous.k_omega_model = "k-omega", "sst"
                setup.general.solver.time = "steady"
                current = case["water_velocity_m_s"]
                for zone, normal in BOX_N.items():
                    dot = sum(a * b for a, b in zip(normal, current))
                    role = "velocity-inlet" if dot < -1e-12 else "pressure-outlet" if dot > 1e-12 else "wall"
                    bc.set_zone_type(zone_list=[zone], new_type=role)
                    if role == "velocity-inlet":
                        m = bc.velocity_inlet[zone].momentum
                        m.velocity_specification_method = "Components"
                        m.velocity_components = [val(x) for x in current]
                        turbulence(bc.velocity_inlet[zone])
                    elif role == "pressure-outlet":
                        bc.pressure_outlet[zone].momentum.gauge_pressure = val(0)
                        turbulence(bc.pressure_outlet[zone], True)
                    else:
                        bc.wall[zone].momentum.shear_condition = "Specified Shear"
                    roles[zone] = role
            else:
                for zone in BOX_N:
                    bc.set_zone_type(zone_list=[zone], new_type="pressure-outlet")
                    bc.pressure_outlet[zone].momentum.gauge_pressure = val(0)
                    roles[zone] = "pressure-outlet"
                setup.models.viscous.model = "laminar"
                setup.general.solver.time = "unsteady-2nd-order"
                omega = 2 * math.pi / case["period_s"]
                velocities = []
                for axis, d in zip("xyz", case["direction"]):
                    name = "blind_osc_rate_" + axis
                    setup.named_expressions[name] = {"definition": "%.17g [m s^-1] * cos(%.17g [s^-1] * t) * min(t / %.17g [s], 1)" % (d * case["amplitude_m"] * omega, omega, case["ramp_s"])}
                    velocities.append(name)
                mm = fluid.mesh_motion
                mm.enable = True
                mm.moving_mesh_axis_origin = [val(x) for x in cfg["origin_m"]]
                mm.moving_mesh_velocity = velocities
            report["box_roles"] = roles
        required("boundary_conditions", boundary_conditions)
        required("posed_joint_reports", joint_reports)
        def mass_reports():
            for zone in BOX_N:
                rd.flux["mass_" + zone] = {"report_type": MASS_FLOW_REPORT_TYPE, "boundaries": [zone]}
        required("conservation_reports", mass_reports)
        def mesh_reference_centroid():
            names = ["motion_centroid_" + axis for axis in "xyz"]
            values = flatten_report(rd.compute(report_defs=names))
            return [values[name] for name in names]
        if case["mode"] == "oscillation_translation":
            def create_motion_reference():
                for axis in "xyz":
                    rd.surface["motion_centroid_" + axis] = {"report_type": "surface-areaavg", "field": axis + "-coordinate", "surface_names": ["wall_auv"]}
                return mesh_reference_centroid()
            initial_centroid = required("mesh_reference_centroid_initial", create_motion_reference)
            report["mesh_reference_centroid_initial_m"] = initial_centroid
        required("write_start_case", lambda: solver.file.write(file_type="case", file_name=str(attempt / "start.cas.h5")))
        def sample(**indices):
            loads = flatten_report(rd.compute(report_defs=REPS))
            if set(loads) != CHANNELS or any(abs(x) > 1e6 for x in loads.values()):
                raise RuntimeError("missing/nonfinite/exploding 57-channel loads")
            mass = flatten_report(rd.compute(report_defs=["mass_" + z for z in BOX_N]))
            if set(mass) != {"mass_" + z for z in BOX_N}:
                raise RuntimeError("incomplete boundary mass reports")
            row = dict(indices, loads=loads, conservation=conservation(loads, mass, report["joint_reports_current"]))
            history.append(row)
            save()
        if case["mode"] == "steady_translation":
            required("hybrid_initialize", lambda: solver.solution.initialization.hybrid_initialize())
            converged = False
            for iteration in range(25, 2501, 25):
                required("iterate_%d" % iteration, lambda: solver.solution.run_calculation.iterate(iter_count=25))
                required("sample_%d" % iteration, lambda: sample(iter=iteration))
                if iteration >= 400:
                    report["drift"] = load_drift(history, 8)
                    if report["drift"]["maximum"] < 0.005:
                        converged = True
                        break
            report["converged"] = converged
            report["outcome"] = "strict_window_drift" if converged else "capped_unsteady_mean"
            report["tail_statistics"] = sample_stats(history[-8:])
            terminal = {"iterations": iteration}
        else:
            def initialize_rest():
                ini = solver.solution.initialization
                ini.initialization_type = "standard"
                ini.defaults = {"x-velocity": 0.0, "y-velocity": 0.0, "z-velocity": 0.0, "pressure": 0.0}
                ini.standard_initialize()
            def transient_controls():
                tc = solver.solution.run_calculation.transient_controls
                tc.time_step_size, tc.max_iter_per_time_step = case["time_step_s"], case["max_iterations_per_step"]
            required("transient_controls", transient_controls)
            required("standard_initialize_rest", initialize_rest)
            for n in range(1, 241):
                required("time_step_%d" % n, lambda: solver.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=30))
                t = n * case["time_step_s"]
                motion = oscillation_state(case, t)
                centroid = required("mesh_reference_centroid_%d" % n, mesh_reference_centroid)
                displacement = [a - b for a, b in zip(centroid, initial_centroid)]
                motion["mesh_displacement_m"] = displacement
                motion["mesh_minus_commanded_displacement_m"] = [a - b for a, b in zip(displacement, motion["displacement_m"])]
                required("moving_joint_centres_%d" % n, lambda: joint_reports(displacement))
                required("sample_%d" % n, lambda: sample(t=t, motion=motion, moving_origin_m=[a + b for a, b in zip(cfg["origin_m"], displacement)]))
            report["cycle_statistics"] = cycle_stats(history, 80)
            report["outcome"] = "240_steps_completed_periodicity_reported_not_assumed"
            terminal = {"time_steps": 240, "flow_time_s": 6.0}
        terminal.update({"id": uuid.uuid4().hex, "sample_count": len(history)})
        report["terminal_state"] = terminal
        # No initialization/iteration/report mutation is permitted between these two writes.
        required("write_final_case", lambda: solver.file.write(file_type="case", file_name=str(attempt / "final.cas.h5")))
        required("write_final_data", lambda: solver.file.write(file_type="data", file_name=str(attempt / "final.dat.h5")))
        required("stop_transcript", solver.transcript.stop)
        required("exit_solver", solver.exit)
        solver = None
        artifacts = {}
        for key, filename in (("start_case", "start.cas.h5"), ("final_case", "final.cas.h5"), ("final_data", "final.dat.h5"), ("history", "history.json"), ("transcript", "solver.trn")):
            path = attempt / filename
            if not path.is_file() or path.stat().st_size == 0:
                raise RuntimeError("required output missing/empty: " + filename)
            artifacts[key] = {"path": filename, "sha256": sha256(path)}
            if key in ("final_case", "final_data"):
                artifacts[key]["state_id"] = terminal["id"]
        report["artifacts"] = artifacts
        report["execution_complete"] = True
        report["status"] = "completed"
        report["solver_statistics"] = {"raw_residuals_and_solver_messages": "solver.trn", "iterations_or_time_steps": terminal,
                                       "steady_acceptance": "original 0.5% adjacent load-window drift; not a residual convergence claim"}
        save()
        if not valid_completed(attempt, expected):
            raise RuntimeError("post-write completion verification failed")
        return {"case_id": case["id"], "status": "completed", "outcome": report["outcome"], "attempt": str(attempt)}
    except BaseException as exc:
        report["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        report["execution_complete"] = False
        report["error"] = repr(exc)
        save()
        raise
    finally:
        if solver is not None:
            try:
                solver.transcript.stop()
                solver.exit()
            except Exception as exc:
                report["cleanup_error"] = repr(exc)
                save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_id")
    parser.add_argument("--case-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--freeze-manifest", type=Path)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--repair-reason")
    args = parser.parse_args(argv)
    try:
        result = check_case(args.case_id, args.case_config, args.freeze_manifest) if args.check else execute(args.case_id, args.case_config, args.freeze_manifest, args.repair_reason)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0
    except (ProtocolError, ValueError, KeyError, OSError, RuntimeError) as exc:
        print(json.dumps({"status": "refused_or_failed", "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

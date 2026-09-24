"""Run the underwater-arm Fluent case through PyFluent without the GUI.

The script defaults to the latest case/data pair produced by the Workbench run.
It first performs a one-step stability test; use --steps > 1 only after that
smoke test succeeds.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import os
import re
import sys
import time
from pathlib import Path
from typing import Any

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode


WORK_DIR = Path(r"D:\work4\cfd\1_files\dp0\FFF\Fluent")
DEFAULT_CASE = WORK_DIR / "FFF-7-00000.cas.h5"
DEFAULT_DATA = WORK_DIR / "FFF-7-00000.dat.h5"
UDF_DIR = WORK_DIR / "libudf_arm_test"
CONFIG_FILE = WORK_DIR / "arm_motion_config.h"
LOG_FILE = WORK_DIR / "pyfluent_smoke.log"
DEFAULT_CSV = WORK_DIR / "moment_j1_j2_j3.csv"
DEFAULT_OUTPUT_CASE = WORK_DIR / "pyfluent_smoke_result.cas.h5"


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("arm_pyfluent")
    logger.setLevel(logging.INFO)
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
        handler.close()
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")

    file_handler = logging.FileHandler(LOG_FILE, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    return logger


def call_tui(logger: logging.Logger, method: Any, *args: Any) -> Any:
    """Call a generated TUI method and log the command boundary."""
    logger.info("TUI %s %s", getattr(method, "__name__", repr(method)), args)
    return method(*args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, default=DEFAULT_CASE)
    parser.add_argument("--data", type=Path)
    parser.add_argument("--geometry-reference-case", type=Path,
                        help="Unmodified t=0 case required for restart geometry checks")
    parser.add_argument("--processors", type=int, default=18)
    parser.add_argument("--mpi", choices=("msmpi", "intel"), default="msmpi")
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--time-step", type=float, default=0.01)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--output-case", type=Path, default=DEFAULT_OUTPUT_CASE)
    parser.add_argument("--checkpoint-every", type=int, default=25)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--show-gui", action="store_true")
    parser.add_argument("--keep-alive", action="store_true")
    args = parser.parse_args()
    if not args.case.name.endswith(".cas.h5"):
        parser.error("--case must name a .cas.h5 file")
    if args.data is None:
        args.data = args.case.with_name(args.case.name.removesuffix(".cas.h5") + ".dat.h5")
    if min(args.processors, args.steps, args.iterations, args.checkpoint_every) < 1:
        parser.error("processors, steps and iterations must be positive")
    if not math.isfinite(args.time_step) or args.time_step <= 0:
        parser.error("--time-step must be finite and positive")
    return args


def load_case(session: Any, case_file: Path, data_file: Path, logger: logging.Logger) -> None:
    """Read exactly the requested pair; never silently switch to another data file."""
    if not case_file.is_file() or not data_file.is_file():
        raise FileNotFoundError(f"Missing case/data pair: {case_file}, {data_file}")
    call_tui(logger, session.tui.file.read_case, str(case_file))
    call_tui(logger, session.tui.file.read_data, str(data_file))
    logger.info("Loaded explicit case/data pair: %s / %s", case_file, data_file)


def read_motion_config() -> dict[str, float]:
    """Read the numeric trajectory and joint-center macros used by the UDF."""
    if not CONFIG_FILE.exists():
        raise FileNotFoundError(f"Motion configuration header does not exist: {CONFIG_FILE}")

    text = CONFIG_FILE.read_text(encoding="ascii")
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    names = [
        "ARM_CFG_T_DEPLOY_S",
        "ARM_CFG_T_WORK_S",
        "ARM_CFG_T_RETRACT_S",
        "ARM_CFG_Q1_STOW_DEG",
        "ARM_CFG_Q2_STOW_DEG",
        "ARM_CFG_Q3_STOW_DEG",
        "ARM_CFG_Q1_WORK_DEG",
        "ARM_CFG_Q2_WORK_DEG",
        "ARM_CFG_Q3_WORK_DEG",
        "ARM_CFG_Q1_SWEEP_HALF_DEG",
        "ARM_CFG_Q1_SWEEP_DIRECTION",
        "ARM_FIXED_JOINT1_CX_M",
        "ARM_FIXED_JOINT1_CY_M",
        "ARM_FIXED_JOINT1_CZ_M",
        "ARM_FIXED_JOINT2_CX_M",
        "ARM_FIXED_JOINT2_CY_M",
        "ARM_FIXED_JOINT2_CZ_M",
        "ARM_FIXED_JOINT3_CX_M",
        "ARM_FIXED_JOINT3_CY_M",
        "ARM_FIXED_JOINT3_CZ_M",
    ]
    values: dict[str, float] = {}
    for name in names:
        match = re.search(rf"^\s*#define\s+{re.escape(name)}\s+({number})", text, re.MULTILINE)
        if not match:
            raise ValueError(f"Missing numeric macro {name} in {CONFIG_FILE}")
        values[name] = float(match.group(1))
    return values


def joint_angles(time_s: float, config: dict[str, float]) -> tuple[float, float, float]:
    """Return q1/q2/q3 in radians using the same piecewise trajectory as the UDF."""
    pi = math.pi
    t_deploy = config["ARM_CFG_T_DEPLOY_S"]
    t_work = config["ARM_CFG_T_WORK_S"]
    t_retract = config["ARM_CFG_T_RETRACT_S"]
    q_stow = tuple(
        math.radians(config[f"ARM_CFG_Q{i}_STOW_DEG"]) for i in (1, 2, 3)
    )
    q_work = tuple(
        math.radians(config[f"ARM_CFG_Q{i}_WORK_DEG"]) for i in (1, 2, 3)
    )
    q1_sweep = math.radians(config["ARM_CFG_Q1_SWEEP_HALF_DEG"])
    q1_sweep *= config["ARM_CFG_Q1_SWEEP_DIRECTION"]

    if time_s < 0.0:
        return q_stow
    if time_s <= t_deploy:
        u = time_s / t_deploy
        s = 0.5 * (1.0 - math.cos(pi * u))
        return tuple(q_stow[i] + (q_work[i] - q_stow[i]) * s for i in range(3))
    if time_s <= t_deploy + t_work:
        tw = time_s - t_deploy
        omega = 2.0 * pi / t_work
        q1 = q_work[0] + q1_sweep * (1.0 - math.cos(omega * tw))
        return q1, q_work[1], q_work[2]
    if time_s <= t_deploy + t_work + t_retract:
        u = (time_s - t_deploy - t_work) / t_retract
        s = 0.5 * (1.0 - math.cos(pi * u))
        return tuple(q_work[i] + (q_stow[i] - q_work[i]) * s for i in range(3))
    return q_stow


def matrix_multiply(a: tuple[tuple[float, ...], ...], b: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def matrix_vector(a: tuple[tuple[float, ...], ...], x: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(a[i][k] * x[k] for k in range(3)) for i in range(3))


def joint_axes(time_s: float, config: dict[str, float]) -> dict[str, tuple[float, float, float]]:
    """Return instantaneous global joint axes, matching calc_link_motion() in C."""
    q1, q2, _ = joint_angles(time_s, config)
    c1, s1 = math.cos(q1), math.sin(q1)
    c2, s2 = math.cos(q2), math.sin(q2)

    rx_m90 = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))
    ry_m90 = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))
    rz_q1 = ((c1, -s1, 0.0), (s1, c1, 0.0), (0.0, 0.0, 1.0))
    rz_q2 = ((c2, -s2, 0.0), (s2, c2, 0.0), (0.0, 0.0, 1.0))
    r1 = matrix_multiply(rx_m90, rz_q1)
    temp1 = matrix_multiply(ry_m90, rz_q2)
    r2 = matrix_multiply(r1, temp1)

    return {
        "j1": (0.0, 1.0, 0.0),
        "j2": (-c1, 0.0, s1),
        "j3": matrix_vector(r2, (0.0, 0.0, -1.0)),
    }


def build_moment_specs(config: dict[str, float]) -> dict[str, tuple[list[float], list[str]]]:
    """Use downstream body-wall groups for each joint moment."""
    return {
        "j1": (
            [config["ARM_FIXED_JOINT1_CX_M"], config["ARM_FIXED_JOINT1_CY_M"], config["ARM_FIXED_JOINT1_CZ_M"]],
            ["lk1", "lk2", "lk3w1", "lk3w2", "lk3w3", "lk3w4"],
        ),
        "j2": (
            [config["ARM_FIXED_JOINT2_CX_M"], config["ARM_FIXED_JOINT2_CY_M"], config["ARM_FIXED_JOINT2_CZ_M"]],
            ["lk2", "lk3w1", "lk3w2", "lk3w3", "lk3w4"],
        ),
        "j3": (
            [config["ARM_FIXED_JOINT3_CX_M"], config["ARM_FIXED_JOINT3_CY_M"], config["ARM_FIXED_JOINT3_CZ_M"]],
            ["lk3w1", "lk3w2", "lk3w3", "lk3w4"],
        ),
    }


def configure_moment_reports(session: Any, config: dict[str, float], logger: logging.Logger) -> dict[tuple[str, str], str]:
    """Create moment-component reports without report plots or report files."""
    moment = session.settings.solution.report_definitions.moment
    existing = set(moment.get_object_names())
    specs = build_moment_specs(config)
    report_names: dict[tuple[str, str], str] = {}
    unit_axes = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}

    for joint, (center, zones) in specs.items():
        for component, axis in unit_axes.items():
            name = f"pyfluent_{joint}_m{component}"
            report = moment[name] if name in existing else moment.create(name=name)
            report.set_state(
                {
                    "report_output_type": "Moment",
                    "mom_center": center,
                    "mom_axis": axis,
                    "reference_frame": "global",
                    "zones": zones,
                    "per_zone": False,
                    "average_over": 1,
                }
            )
            report_names[(joint, component)] = name

    logger.info("Created %d component-only Moment reports; plots/files disabled", len(report_names))
    for key, name in report_names.items():
        logger.info("Report %s -> %s", key, name)
    return report_names


def unpack_report_value(payload: Any) -> tuple[float, int | None]:
    """Return the scalar and opaque auxiliary index; the latter is NOT a sample count."""
    if isinstance(payload, (list, tuple)):
        if not payload:
            raise ValueError("Empty Fluent report response")
        value = float(payload[0])
        auxiliary = int(payload[1]) if len(payload) > 1 else None
    else:
        value, auxiliary = float(payload), None
    if not math.isfinite(value):
        raise ValueError(f"Non-finite Fluent report value: {payload!r}")
    return value, auxiliary


def compute_moment_row(
    session: Any,
    report_names: dict[tuple[str, str], str],
    config: dict[str, float],
    flow_time: float,
    centers: dict[str, list[float]],
) -> dict[str, float]:
    """Compute about the solver's CURRENT joint origins, never fixed initial pivots."""
    from fluent_checks import scalar_reports

    reports = session.settings.solution.report_definitions
    for (joint, _), name in report_names.items():
        reports.moment[name].mom_center = centers[joint]
    names = list(report_names.values())
    values = scalar_reports(reports.compute(report_defs=names), names)
    axes = joint_axes(flow_time, config)
    q1, q2, q3 = joint_angles(flow_time, config)
    row = {"flow_time_s": flow_time, "q1_rad": q1, "q2_rad": q2, "q3_rad": q3}
    for joint in ("j1", "j2", "j3"):
        components = [values[report_names[(joint, axis)]] for axis in ("x", "y", "z")]
        for i, axis in enumerate(("x", "y", "z")):
            row[f"{joint}_m{axis}"] = components[i]
            row[f"{joint}_center_{axis}_m"] = centers[joint][i]
            row[f"{joint}_axis_{axis}"] = axes[joint][i]
        row[f"{joint}_torque_nm"] = sum(axes[joint][i] * components[i] for i in range(3))
    return row


def write_moment_csv(csv_file: Path, rows: list[dict[str, float | int | None]]) -> None:
    if not rows:
        raise ValueError("No moment rows were collected")
    csv_file.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with csv_file.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def prepare_geometry_reference(args: argparse.Namespace) -> dict[str, Any]:
    """Reject incompatible topology and anchor every restart to an explicit t=0 mesh."""
    from fluent_checks import wall_geometry_check
    from rigid_mesh_audit import file_sha256, read_wall_mesh, require_independent_motion

    current = read_wall_mesh(args.case)
    require_independent_motion(current)
    reference_path = getattr(args, 'geometry_reference_case', None)
    if reference_path is None:
        if not math.isclose(current['flow_time_s'], 0., rel_tol=0., abs_tol=1e-12):
            raise ValueError('RESTART_REFERENCE_REQUIRED: supply --geometry-reference-case '
                             'with the unchanged t=0 case; a checkpoint is not a new shape baseline')
        reference_path = args.case
    reference_path = Path(reference_path).resolve()
    reference = current if reference_path == args.case.resolve() else read_wall_mesh(reference_path)
    if not math.isclose(reference['flow_time_s'], 0., rel_tol=0., abs_tol=1e-12):
        raise ValueError('GEOMETRY_REFERENCE_NOT_INITIAL: reference case must be at t=0')
    require_independent_motion(reference)
    baseline = {name: item['points'] for name, item in reference['walls'].items()}
    current_points = {name: item['points'] for name, item in current['walls'].items()}
    wall_geometry_check(baseline, current_points, tolerance=1e-5)
    return {'case': str(reference_path), 'sha256': file_sha256(reference_path),
            'flow_time_s': 0., 'rigid_radius_tolerance_m': 1e-5,
            'source_flow_time_s': current['flow_time_s'], 'vertices': baseline}


def run_smoke_test(session: Any, args: argparse.Namespace, logger: logging.Logger) -> None:
    """Advance an already prepared fluid-side case with checked, durable output."""
    from fluent_checks import (
        ARM_WALLS, JOINT_WALL, check_volume, configure_volume_check,
        joint_centers_from_motion, motion_state, native_moment_check,
        save_checkpoint, wall_geometry_check, wall_interface_metadata, wall_vertices,
        write_json_atomic,
    )
    import hashlib
    import numpy as np

    config = read_motion_config()
    specs = build_moment_specs(config)
    walls = case_wall_adjacency(args.case)
    wall_ids = {wall["name"]: wall["id"] for wall in walls}
    require_fluid_report_walls(walls, set(ARM_WALLS))
    libraries = [str(v).strip('"') for v in session.rp_vars('udf/libname')]
    if len(libraries) != 1 or Path(libraries[0]).name != 'libudf_arm_test':
        raise ValueError(f"Expected only libudf_arm_test, got {libraries}")
    compiled_config = UDF_DIR / 'src' / 'arm_motion_config.h'
    if not compiled_config.is_file() or CONFIG_FILE.read_bytes() != compiled_config.read_bytes():
        raise ValueError('Report trajectory config differs from the compiled-library source header')
    actual_processors = int(str(session.rp_vars('parallel/nprocs_string')).strip('"'))
    if actual_processors != args.processors:
        raise RuntimeError(f'Requested {args.processors} nodes but Fluent started {actual_processors}')
    start_time = float(session.rp_vars('flow-time'))
    initial_motion = motion_state(session, wall_ids)
    if abs(start_time) < 1e-12:
        for joint, (center, _) in specs.items():
            actual = initial_motion[JOINT_WALL[joint]]['origin']
            if np.linalg.norm(np.asarray(actual) - center) > 1e-6:
                raise ValueError(f'Initial dynamic origin inconsistent with configuration: {joint}')
    from scipy.spatial import cKDTree
    from rigid_mesh_audit import file_sha256
    reference = getattr(args, '_geometry_reference', None) or prepare_geometry_reference(args)
    if file_sha256(reference['case']) != reference['sha256']:
        raise ValueError('GEOMETRY_REFERENCE_CHANGED: t=0 case changed after preflight')
    if not math.isclose(start_time, reference['source_flow_time_s'], rel_tol=0., abs_tol=1e-10):
        raise ValueError('Case/data flow time differs from the preflighted snapshot')
    initial_vertices = reference['vertices']
    start_vertices = wall_vertices(session)
    wall_interfaces = wall_interface_metadata(initial_vertices)
    wall_geometry_check(initial_vertices, start_vertices,
                        tolerance=reference['rigid_radius_tolerance_m'])
    start_trees = {name: cKDTree(points) for name, points in start_vertices.items()}
    report_names = configure_moment_reports(session, config, logger)
    volume_report = configure_volume_check(session)
    check_volume(session, volume_report)
    call_tui(logger, session.tui.mesh.check)
    call_tui(logger, session.tui.solve.set.transient_controls.time_step_size, args.time_step)

    sidecar = args.csv.with_suffix('.validation.json')
    diagnostic = args.csv.with_suffix('.checks.jsonl')
    native_directory = args.csv.parent / (args.csv.stem + '_native')
    native_directory.mkdir(exist_ok=False)
    metadata = {
        'valid': False, 'reason': 'Run in progress; not yet validated',
        'source_case': str(args.case.resolve()), 'source_data': str(args.data.resolve()),
        'start_time_s': start_time, 'requested_steps': args.steps, 'time_step_s': args.time_step,
        'actual_compute_nodes': actual_processors, 'loaded_libraries': libraries,
        'config_sha256': hashlib.sha256(CONFIG_FILE.read_bytes()).hexdigest(),
        'wall_adjacency': walls, 'initial_motion': initial_motion,
        'wall_interface_metadata': wall_interfaces,
        'geometry_reference': {key: value for key, value in reference.items() if key != 'vertices'},
        'geometry_policy': 'all_rigid_walls_against_unchanged_t0_no_shared_vertex_exemptions',
        'sign_convention': 'fluid_on_arm_positive_URDF_joint_axis',
        'materials': session.settings.setup.materials.fluid.get_state(),
        'velocity_inlets': session.settings.setup.boundary_conditions.velocity_inlet.get_state(),
        'gravity_enabled': session.rp_vars('gravity?'),
        'native_verification': [], 'checkpoints': [], 'completed_steps': 0,
    }
    write_json_atomic(sidecar, metadata)
    ever_moved = {name: False for name in ARM_WALLS}
    rows = []
    previous_time = start_time
    csv_stream = args.csv.open('x', newline='', encoding='utf-8')
    checks_stream = diagnostic.open('x', encoding='utf-8')
    writer = None
    try:
        for step in range(1, args.steps + 1):
            logger.info('Advancing step %d/%d from t=%.9g s', step, args.steps, previous_time)
            started = time.perf_counter()
            call_tui(logger, session.tui.solve.dual_time_iterate, 1, args.iterations)
            flow_time = float(session.rp_vars('flow-time'))
            if not math.isclose(flow_time, previous_time + args.time_step, rel_tol=1e-9, abs_tol=1e-10):
                raise RuntimeError(f'Unexpected physical time: {previous_time} -> {flow_time}')
            minimum_volume = check_volume(session, volume_report)
            motion = motion_state(session, wall_ids)
            centers = joint_centers_from_motion(motion)
            current_vertices = wall_vertices(session)
            geometry = wall_geometry_check(
                initial_vertices, current_vertices, tolerance=reference['rigid_radius_tolerance_m'],
                interface_metadata=wall_interfaces)
            for name in ARM_WALLS:
                geometry[name]['displacement_from_reference_m'] = geometry[name]['displacement_from_start_m']
                geometry[name]['displacement_from_start_m'] = float(
                    start_trees[name].query(current_vertices[name])[0].max())
                ever_moved[name] |= geometry[name]['displacement_from_start_m'] > 2e-7
            row = compute_moment_row(session, report_names, config, flow_time, centers)
            row['minimum_fluid_cell_volume_m3'] = minimum_volume
            row['step_index'] = int(session.rp_vars('time-step'))
            if writer is None:
                writer = csv.DictWriter(csv_stream, fieldnames=list(row))
                writer.writeheader()
            if step == 1 or step == args.steps:
                native = native_moment_check(session, native_directory, row['step_index'], specs,
                                             centers, joint_axes(flow_time, config), row)
                metadata['native_verification'].append({'flow_time_s': flow_time, 'reports': native})
            writer.writerow(row)
            csv_stream.flush()
            os.fsync(csv_stream.fileno())
            rows.append(row)
            checks_stream.write(json.dumps({'flow_time_s': flow_time, 'motion': motion,
                                           'geometry': geometry, 'min_fluid_volume_m3': minimum_volume},
                                          allow_nan=False) + '\n')
            checks_stream.flush()
            metadata['completed_steps'] = step
            metadata['end_time_s'] = flow_time
            metadata['ever_moved'] = ever_moved.copy()
            if step % args.checkpoint_every == 0 and step < args.steps:
                stem = args.output_case.name.removesuffix('.cas.h5')
                checkpoint = args.output_case.with_name(f'{stem}_{row["step_index"]:06d}.cas.h5')
                metadata['checkpoints'].append(save_checkpoint(session, checkpoint, flow_time))
            write_json_atomic(sidecar, metadata)
            logger.info('t=%.6f; min(V)=%.6g; tau=[%.8g, %.8g, %.8g]; elapsed=%.1fs',
                        flow_time, minimum_volume, row['j1_torque_nm'], row['j2_torque_nm'],
                        row['j3_torque_nm'], time.perf_counter() - started)
            previous_time = flow_time
        metadata['checkpoints'].append(save_checkpoint(session, args.output_case, previous_time))
        # Reaching the end is not sufficient: moving trajectories need observed wetted-wall motion.
        q_start, q_end = joint_angles(start_time, config), joint_angles(previous_time, config)
        trajectory_moved = any(abs(row[f'q{i+1}_rad'] - q_start[i]) > 1e-5
                               for row in rows for i in range(3))
        if trajectory_moved and not all(ever_moved.values()):
            missing = [name for name in ARM_WALLS if not ever_moved[name]]
            raise RuntimeError(f'Prescribed motion but no observed wetted-wall displacement: {missing}')
        metadata['valid'] = True
        metadata['reason'] = ('Fluid adjacency, UDF binding, positive volumes, all rigid-wall '
                              'radius invariants and native moments checked; no interface exemptions')
        metadata['scope'] = ('This run segment only; not full-cycle, restart-baseline, '
                             'mesh/time-step independence or model-accuracy certification')
        write_json_atomic(sidecar, metadata)
        logger.info('CHECKED_RUN_COMPLETE: %d rows, %.6g..%.6g s', len(rows), start_time, previous_time)
    except BaseException as exc:
        metadata['valid'] = False
        metadata['reason'] = f'{type(exc).__name__}: {exc}'
        write_json_atomic(sidecar, metadata)
        raise
    finally:
        checks_stream.close()
        csv_stream.close()


def case_wall_adjacency(case_file: Path) -> list[dict[str, Any]]:
    """Read actual face-to-cell connectivity, not just boundary names."""
    import h5py

    with h5py.File(case_file, "r") as case:
        text = case["settings/Thread Variables"][0].decode("utf-8")
        threads = {
            int(zone_id): (zone_type, name)
            for zone_id, zone_type, name in re.findall(
                r"\(39 \((\d+) (\S+) (\S+) \d+\)", text
            )
        }
        zones = case["meshes/1/faces/zoneTopology"]
        names = zones["name"][0].decode("utf-8").split(";")
        result = []
        for i, name in enumerate(names):
            zone_id = int(zones["id"][i])
            if threads.get(zone_id, (None, None))[0] != "wall":
                continue
            adjacent = [int(zones[side][i]) for side in ("c0", "c1")]
            adjacent_types = [threads.get(z, ("none", "none"))[0] for z in adjacent]
            result.append({"name": name, "id": zone_id, "adjacent": adjacent,
                           "adjacent_types": adjacent_types,
                           "faces": int(zones["maxId"][i] - zones["minId"][i] + 1)})
        return result


def require_fluid_report_walls(walls: list[dict[str, Any]], selected: set[str]) -> None:
    """Fail closed before launching an expensive run on solid-only walls."""
    by_name = {wall["name"]: wall for wall in walls}
    missing = sorted(selected - by_name.keys())
    if missing:
        raise ValueError(f"Missing report walls: {missing}")
    invalid = [name for name in sorted(selected)
               if "fluid" not in by_name[name]["adjacent_types"]]
    if invalid:
        fluid_walls = [w["name"] for w in walls if "fluid" in w["adjacent_types"]]
        raise ValueError(
            f"INVALID_CFD_TOPOLOGY: {invalid} have no adjacent fluid cells. "
            f"Available fluid-side walls: {fluid_walls}. Separate and bind the actual "
            "wetted arm surfaces; do not use whole-vehicle forces as joint torques."
        )


def main() -> int:
    args = parse_args()
    logger = configure_logging()
    logger.info("PyFluent version: %s", pyfluent.__version__)
    logger.info("Case: %s", args.case)
    logger.info("Data: %s", args.data)

    if not args.case.exists():
        logger.error("Case file does not exist: %s", args.case)
        return 2
    if not args.data.exists():
        logger.error("Data file does not exist: %s", args.data)
        return 2
    try:
        config = read_motion_config()
        selected = {z for _, zones in build_moment_specs(config).values() for z in zones}
        walls = case_wall_adjacency(args.case)
        require_fluid_report_walls(walls, selected)
        args._geometry_reference = prepare_geometry_reference(args)
        logger.info("Fluid adjacency, independent motion topology and t=0 reference verified")
    except Exception:
        logger.exception("PREFLIGHT=FAIL; no solver launched and no result overwritten")
        return 2
    if args.preflight_only:
        logger.info("PREFLIGHT=PASS (topology and reference only; not CFD accuracy certification)")
        return 0
    output_data = args.output_case.with_name(args.output_case.name.replace(".cas.h5", ".dat.h5"))
    if not args.output_case.name.endswith(".cas.h5"):
        logger.error("Output case must end with .cas.h5")
        return 2
    protected_inputs = (args.case.resolve(), args.data.resolve(),
                        Path(args._geometry_reference['case']).resolve())
    for output in (args.csv, args.output_case, output_data, args.csv.with_suffix('.validation.json'),
                   args.csv.with_suffix('.checks.jsonl'), args.csv.parent / (args.csv.stem + '_native')):
        if output.exists() or output.resolve() in protected_inputs:
            logger.error("Refusing to overwrite existing input or output: %s", output)
            return 2

    # PyFluent's standalone launcher resolves Fluent through AWP_ROOT242.
    os.environ.setdefault("AWP_ROOT242", r"D:\Program Files\ANSYS Inc\v242")
    # Microsoft MPI is the tested Windows default; do not force Intel-specific overrides.
    os.environ.setdefault("PROCESSOR_ARCHITECTURE", "AMD64")
    logger.info("AWP_ROOT242=%s; nodes=%d; MPI=%s; arch=%s", os.environ["AWP_ROOT242"],
                args.processors, args.mpi, os.environ["PROCESSOR_ARCHITECTURE"])

    session = None
    try:
        session = pyfluent.launch_fluent(
            product_version="24.2",
            version="3d",
            mode="solver",
            precision="double",
            processor_count=args.processors,
            additional_arguments=f"-t{args.processors} -mpi={args.mpi}",
            ui_mode=UIMode.GUI if args.show_gui else UIMode.NO_GUI_OR_GRAPHICS,
            cwd=str(WORK_DIR),
            start_timeout=180,
            start_transcript=True,
            cleanup_on_exit=not args.keep_alive,
        )
        logger.info("Fluent session started")
        load_case(session, args.case, args.data, logger)
        run_smoke_test(session, args, logger)
        logger.info("PYFLUENT_SMOKE_TEST=PASS")
        if args.keep_alive:
            logger.info("Keeping Fluent session alive (--keep-alive)")
            input("Press Enter to close the Fluent session... ")
        return 0
    except Exception:
        logger.exception("PYFLUENT_SMOKE_TEST=FAIL")
        return 1
    finally:
        if session is not None:
            try:
                session.exit()
                logger.info("Fluent session exited")
            except Exception:
                logger.exception("Failed to exit Fluent session cleanly")


if __name__ == "__main__":
    raise SystemExit(main())

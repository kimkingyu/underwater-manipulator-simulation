"""Checked PyFluent 2024 R2 operations used by the arm transient driver."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
from ansys.fluent.core.services.field_data import SurfaceDataType


ARM_WALLS = ('lk1', 'lk2', 'lk3w1', 'lk3w2', 'lk3w3', 'lk3w4')
JOINT_WALL = {'j1': 'lk1', 'j2': 'lk2', 'j3': 'lk3w1'}
NUMBER = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'


def write_json_atomic(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.flush()
    temporary.replace(path)


def motion_state(session: Any, wall_ids: dict[str, int]) -> dict[str, dict[str, Any]]:
    records = session.rp_vars('dynamesh/dynamic-zones')
    by_id = {int(record[0]): {item[0]: item[1] for item in record[1:] if len(item) == 2}
             for record in records}
    result = {}
    for name in ARM_WALLS:
        if name not in wall_ids or wall_ids[name] not in by_id:
            raise ValueError(f'Missing dynamic fluid wall binding: {name}')
        record = by_id[wall_ids[name]]
        joint = 1 if name == 'lk1' else 2 if name == 'lk2' else 3
        udf = str(record.get('udf', '')).strip('"')
        if udf != f'joint{joint}_motion::libudf_arm_test':
            raise ValueError(f'Unexpected UDF for {name}: {udf}')
        if record.get('motion-relative', False):
            raise ValueError(f'{name}: absolute-velocity UDF must not be relative motion')
        if int(record.get('type', -1)) != 1:
            raise ValueError(f'{name}: expected rigid-body dynamic zone type 1')
        if record.get('bc-exclude-motion', False):
            raise ValueError(f'{name}: boundary condition excludes mesh velocity')
        origin = np.asarray(record['origin'], dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError(f'Invalid joint reference point: {name}')
        result[name] = {'origin': origin.tolist(), 'udf': udf,
                        'velocity': list(record.get('velo', [0, 0, 0])),
                        'omega': list(record.get('omega', [0, 0, 0]))}
    origins = [result[name]['origin'] for name in ARM_WALLS[2:]]
    if any(np.linalg.norm(np.asarray(o) - origins[0]) > 1e-8 for o in origins[1:]):
        raise ValueError('The four link-3 walls no longer share a common reference point')
    return result


def joint_centers_from_motion(motion: dict) -> dict[str, list[float]]:
    return {joint: motion[wall]['origin'] for joint, wall in JOINT_WALL.items()}


def scalar_reports(response: Any, names: list[str]) -> dict[str, float]:
    result = {}
    for record in response:
        if not isinstance(record, dict):
            raise ValueError(f'Unexpected report response: {response!r}')
        for name, payload in record.items():
            if name in result:
                raise ValueError(f'Duplicate report returned: {name}')
            value = payload[0] if isinstance(payload, (list, tuple)) and payload else payload
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f'Non-finite report value: {name}')
            result[name] = value
    if set(result) != set(names):
        raise ValueError(f'Missing/extra report values: expected {names}, got {list(result)}')
    return result


def configure_volume_check(session: Any) -> str:
    name = 'pyfluent_min_fluid_volume'
    volumes = session.settings.solution.report_definitions.volume
    report = volumes[name] if name in volumes.get_object_names() else volumes.create(name=name)
    report.report_type = 'volume-min'
    report.set_state({'field': 'cell-volume',
                      'cell_zones': list(session.settings.setup.cell_zone_conditions.fluid.get_object_names()),
                      'per_zone': False, 'average_over': 1})
    return name


def check_volume(session: Any, report_name: str) -> float:
    values = scalar_reports(session.settings.solution.report_definitions.compute(report_defs=[report_name]),
                            [report_name])
    volume = values[report_name]
    if volume <= 0:
        raise RuntimeError(f'Nonpositive fluid cell volume: {volume:.17g}')
    return volume


def wall_vertices(session: Any) -> dict[str, np.ndarray]:
    result = {}
    for name in ARM_WALLS:
        data = session.fields.field_data.get_surface_data(SurfaceDataType.Vertices, surface_name=name)
        points = np.asarray([(p.x, p.y, p.z) for p in data.data], dtype=float)
        if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError(f'Invalid/empty wetted wall vertices: {name}')
        # Fluent can return repeated parallel-interface vertices.
        result[name] = np.unique(points, axis=0)
    return result


def wall_interface_metadata(walls: dict[str, np.ndarray], decimals: int = 9) -> dict[str, dict[str, int]]:
    """Count coincident vertices for diagnostics, not physical classification.

    A shared coordinate does not prove a deformable interface. Split surfaces
    on the same rigid body can share every vertex; independently moving bodies
    can also be incorrectly stitched together. Neither permits a shape waiver.
    """
    owners: dict[tuple[float, float, float], set[str]] = {}
    points_by_wall = {}
    for name in ARM_WALLS:
        points = np.asarray(walls[name], dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
            raise ValueError(f'Invalid/empty wetted wall vertices: {name}')
        points_by_wall[name] = points
        for point in points:
            key = tuple(np.round(point, decimals))
            owners.setdefault(key, set()).add(name)
    result = {}
    for name, points in points_by_wall.items():
        shared = sum(len(owners[tuple(np.round(point, decimals))]) > 1 for point in points)
        result[name] = {
            'shared_vertices': int(shared),
            'private_vertices': int(len(points) - shared),
        }
    return result


def wall_geometry_check(before: dict, after: dict, tolerance: float = 0.02,
                        interface_metadata: dict[str, dict[str, int]] | None = None) -> dict:
    """Check every rigid wall, including shared-node and small wall zones.

    Coincident-vertex counts are diagnostics only. A rigid zone is not allowed
    to deform simply because it has no private vertices. This radius invariant
    is a necessary check, not a proof of correct articulation or mesh accuracy.
    """
    from scipy.spatial import cKDTree
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Rigid wall tolerance must be finite and nonnegative')
    result = {}
    for name in ARM_WALLS:
        initial = np.asarray(before[name], dtype=float)
        current = np.asarray(after[name], dtype=float)
        for points in (initial, current):
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
                raise ValueError(f'Invalid/empty wetted wall vertices: {name}')
        if initial.shape != current.shape:
            raise RuntimeError(f'Rigid wall node count changed: {name} {initial.shape} -> {current.shape}')
        metadata = (interface_metadata or {}).get(
            name, {'shared_vertices': 0, 'private_vertices': len(initial)})
        counts = [metadata.get(key) for key in ('shared_vertices', 'private_vertices')]
        if any(type(count) is not int or count < 0 for count in counts) or sum(counts) != len(initial):
            raise ValueError(f'Invalid shared/private vertex counts: {name}')
        ca, cb = initial.mean(axis=0), current.mean(axis=0)
        ra = np.sort(np.linalg.norm(initial - ca, axis=1))
        rb = np.sort(np.linalg.norm(current - cb, axis=1))
        error = float(np.max(np.abs(ra - rb)))
        if not math.isfinite(error):
            raise ValueError(f'Non-finite rigid wall radius error: {name}')
        if error > tolerance:
            raise RuntimeError(f'Rigid wall deformed: {name}, radius error={error:.6g} m; '
                               'shared vertices do not exempt a rigid wall')
        displacement = float(cKDTree(initial).query(current)[0].max())
        result[name] = {
            'vertices': int(len(current)), 'centroid_m': cb.tolist(),
            'shared_vertices': counts[0], 'private_vertices': counts[1],
            'rigid_shape_check': 'passed', 'rigid_radius_error_m': error,
            'displacement_from_start_m': displacement,
        }
    return result


def native_moment_check(session: Any, directory: Path, step: int, specs: dict,
                        centers: dict, axes: dict, row: dict) -> dict:
    result = {}
    for joint, (_, zones) in specs.items():
        path = directory / f'native_{step:06d}_{joint}.txt'
        if path.exists():
            raise FileExistsError(f'Refusing to overwrite native verification: {path}')
        session.settings.results.report.forces(
            option='moments', wall_zones=zones, momentum_center=centers[joint],
            momentum_axis=list(axes[joint]), write_to_file=True,
            file_name=str(path), append_data=False)
        if not path.is_file():
            raise RuntimeError(f'Native force report was not written: {path}')
        text = path.read_text(errors='replace')
        match = re.search(r'^Net\s+\([^)]*\)\s+\([^)]*\)\s+\(([^)]*)\)', text, re.MULTILINE)
        if not match:
            raise ValueError(f'Cannot parse native total moment vector: {path}')
        vector = [float(v) for v in re.findall(NUMBER, match.group(1))]
        if len(vector) != 3 or not all(math.isfinite(v) for v in vector):
            raise ValueError(f'Invalid native moment vector: {path}')
        projected = sum(v*a for v, a in zip(vector, axes[joint]))
        actual = float(row[f'{joint}_torque_nm'])
        if not math.isclose(projected, actual, rel_tol=2e-6, abs_tol=1e-8):
            raise RuntimeError(f'Native/settings torque mismatch {joint}: {projected} vs {actual}')
        result[joint] = {'file': str(path), 'native_torque_nm': projected,
                         'settings_torque_nm': actual, 'absolute_difference_nm': abs(projected-actual)}
    return result


class StepConvergence:
    """Observe Fluent's explicit convergence and end-of-time-step markers."""
    def __init__(self, session: Any):
        import threading
        self.session = session
        self.finished = threading.Event()
        self.converged = False
        self.expected_time = None
        self.callback_id = session.transcript.register_callback(self.receive)

    def begin(self, expected_time: float) -> None:
        self.finished.clear()
        self.converged = False
        self.expected_time = expected_time

    def receive(self, text: str) -> None:
        if self.expected_time is None:
            return
        if 'solution is converged' in text.lower():
            self.converged = True
        match = re.search(r'Flow time\s*=\s*(' + NUMBER + ')', text)
        if match and math.isclose(float(match.group(1)), self.expected_time, abs_tol=1e-9, rel_tol=1e-9):
            self.finished.set()

    def require(self, wait_seconds: float = 30) -> None:
        if not self.finished.wait(wait_seconds):
            raise RuntimeError('Missing solver end-of-time-step transcript confirmation')
        if not self.converged:
            raise RuntimeError('Inner iterations did not converge; increase --iterations and restart the last verified checkpoint')

    def close(self) -> None:
        self.session.transcript.unregister_callback(self.callback_id)


def save_checkpoint(session: Any, case_path: Path, expected_time: float) -> dict:
    """Verify the written pair itself, not merely that the write command returned."""
    import h5py
    data_path = case_path.with_name(case_path.name.removesuffix('.cas.h5') + '.dat.h5')
    if case_path.exists() or data_path.exists():
        raise FileExistsError(f'Checkpoint already exists: {case_path}')
    session.tui.file.write_case_data(str(case_path))
    for path in (case_path, data_path):
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f'Missing or empty checkpoint artifact: {path}')
    with h5py.File(case_path, 'r') as case:
        text = case['settings/Rampant Variables'][0].decode()
        match = re.search(r'^\(flow-time\s+(' + NUMBER + r')\)', text, re.MULTILINE)
        if not match or not math.isclose(float(match.group(1)), expected_time, abs_tol=1e-9, rel_tol=1e-10):
            raise RuntimeError(f'Checkpoint flow-time mismatch: {case_path}')
    with h5py.File(data_path, 'r') as data:
        if 'results/1/phase-1/cells/SV_P' not in data:
            raise RuntimeError(f'Pressure solution missing in checkpoint: {data_path}')
    return {'case': str(case_path), 'data': str(data_path), 'flow_time_s': expected_time}

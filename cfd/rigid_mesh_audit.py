"""
Rigid mesh audit module for CFD dynamic mesh H5 case files.
Audits wall node topological separation across independent kinematic motion groups.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import h5py
import numpy as np

WALL_GROUPS: Dict[str, str] = {
    'lk1': 'j1',
    'lk2': 'j2',
    'lk3w1': 'j3',
    'lk3w2': 'j3',
    'lk3w3': 'j3',
    'lk3w4': 'j3',
}


def file_sha256(path: Union[str, Path], chunk_size: int = 65536) -> str:
    """Compute SHA256 digest of a file incrementally without modifying it."""
    file_path = Path(path)
    if not file_path.is_file():
        raise FileNotFoundError(f"File not found: {file_path}")
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _extract_flow_time(f: h5py.File) -> float:
    """Extract and validate flow-time from settings/Rampant Variables."""
    if "settings" not in f or "Rampant Variables" not in f["settings"]:
        raise ValueError("Missing 'settings/Rampant Variables' dataset in case file")

    ds = f["settings/Rampant Variables"]
    val = ds[()]
    if hasattr(val, "tobytes"):
        text = val.tobytes().decode("latin1", errors="ignore")
    elif isinstance(val, (bytes, bytearray)):
        text = val.decode("latin1", errors="ignore")
    elif isinstance(val, np.ndarray) and val.dtype.kind in ("S", "U", "O"):
        if val.size > 0:
            item = val.flat[0]
            if isinstance(item, (bytes, bytearray)):
                text = item.decode("latin1", errors="ignore")
            else:
                text = str(item)
        else:
            raise ValueError("Empty Rampant Variables dataset")
    else:
        text = str(val)

    m = re.search(r"\(flow-time\s+([^\)\s]+)\)", text)
    if not m:
        raise ValueError("Pattern '(flow-time NUM)' not found in Rampant Variables")

    raw_val = m.group(1).strip()
    try:
        flow_time = float(raw_val)
    except ValueError as exc:
        raise ValueError(f"Unable to parse flow-time value '{raw_val}' as float") from exc

    if not math.isfinite(flow_time):
        raise ValueError(f"flow-time must be finite, got: {flow_time}")
    if flow_time < 0.0:
        raise ValueError(f"flow-time cannot be negative, got: {flow_time}")

    return flow_time


def _parse_zone_topology(
    zt_faces: h5py.Group, required_walls: Dict[str, str]
) -> Dict[str, Tuple[int, int]]:
    """Parse face zoneTopology and locate minId, maxId for required wall names."""
    if "name" not in zt_faces or "minId" not in zt_faces or "maxId" not in zt_faces:
        raise ValueError("meshes/1/faces/zoneTopology missing required fields (name, minId, maxId)")

    name_ds = zt_faces["name"]
    raw_names = name_ds[()]
    names_list: List[str] = []

    if isinstance(raw_names, (bytes, bytearray)):
        names_list = raw_names.decode("latin1").split(";")
    elif isinstance(raw_names, str):
        names_list = raw_names.split(";")
    elif hasattr(raw_names, "shape") and raw_names.shape == (1,):
        item = raw_names[0]
        if isinstance(item, (bytes, bytearray)):
            names_list = item.decode("latin1").split(";")
        else:
            names_list = str(item).split(";")
    elif hasattr(raw_names, "__iter__"):
        for item in raw_names:
            if isinstance(item, (bytes, bytearray)):
                names_list.append(item.decode("latin1"))
            else:
                names_list.append(str(item))
    else:
        names_list = str(raw_names).split(";")

    min_ids = np.asarray(zt_faces["minId"][:], dtype=np.int64).flatten()
    max_ids = np.asarray(zt_faces["maxId"][:], dtype=np.int64).flatten()

    if len(names_list) != len(min_ids) or len(min_ids) != len(max_ids):
        raise ValueError(
            f"zoneTopology field length mismatch: {len(names_list)} names, "
            f"{len(min_ids)} minIds, {len(max_ids)} maxIds"
        )

    wall_ranges: Dict[str, Tuple[int, int]] = {}
    for i, name in enumerate(names_list):
        clean_name = name.strip()
        if clean_name in required_walls:
            z_min = int(min_ids[i])
            z_max = int(max_ids[i])
            if z_min <= 0 or z_max < z_min:
                raise ValueError(f"Invalid face zone range for '{clean_name}': [{z_min}, {z_max}]")
            if clean_name in wall_ranges:
                raise ValueError(f"Duplicate wall zone definition for '{clean_name}'")
            wall_ranges[clean_name] = (z_min, z_max)

    missing = set(required_walls.keys()) - set(wall_ranges.keys())
    if missing:
        raise ValueError(f"Missing required wall zone(s) in case file: {sorted(missing)}")

    return wall_ranges


class _FaceBlock:
    def __init__(self, key: str, min_id: int, max_id: int, nnodes: np.ndarray, nodes: np.ndarray):
        self.key = key
        self.min_id = min_id
        self.max_id = max_id
        self.nnodes = nnodes
        self.nodes = nodes
        self.offsets = np.zeros(len(nnodes) + 1, dtype=np.int64)
        np.cumsum(nnodes, out=self.offsets[1:], dtype=np.int64)


def _load_face_blocks(fn_group: h5py.Group) -> List[_FaceBlock]:
    """Load and validate all face blocks under meshes/1/faces/nodes."""
    blocks: List[_FaceBlock] = []
    for key in fn_group.keys():
        b_item = fn_group[key]
        if not isinstance(b_item, h5py.Group):
            continue

        if "minId" not in b_item.attrs or "maxId" not in b_item.attrs:
            raise ValueError(f"Face block '{key}' missing minId/maxId attributes")
        min_id = int(np.asarray(b_item.attrs["minId"]).item())
        max_id = int(np.asarray(b_item.attrs["maxId"]).item())
        if min_id <= 0 or max_id < min_id:
            raise ValueError(f"Face block '{key}' has invalid ID range: [{min_id}, {max_id}]")

        if "nnodes" not in b_item or "nodes" not in b_item:
            raise ValueError(f"Face block '{key}' missing 'nnodes' or 'nodes' dataset")

        nnodes_ds = b_item["nnodes"]
        nodes_ds = b_item["nodes"]

        expected_faces = max_id - min_id + 1
        if len(nnodes_ds) != expected_faces:
            raise ValueError(
                f"Face block '{key}' nnodes length {len(nnodes_ds)} != expected {expected_faces}"
            )

        nnodes = np.asarray(nnodes_ds[:], dtype=np.int64)
        if np.any(nnodes < 0):
            raise ValueError(f"Face block '{key}' contains negative nnodes")

        total_nodes = int(np.sum(nnodes, dtype=np.int64))
        if len(nodes_ds) != total_nodes:
            raise ValueError(
                f"Face block '{key}' nodes dataset length {len(nodes_ds)} != sum(nnodes) {total_nodes}"
            )

        nodes = np.asarray(nodes_ds[:], dtype=np.int64)
        blocks.append(_FaceBlock(key, min_id, max_id, nnodes, nodes))

    if not blocks:
        raise ValueError("No face blocks found under meshes/1/faces/nodes")

    blocks.sort(key=lambda b: b.min_id)
    for i in range(len(blocks) - 1):
        if blocks[i].max_id >= blocks[i + 1].min_id:
            raise ValueError(
                f"Overlapping face block ranges: block '{blocks[i].key}' [{blocks[i].min_id}, {blocks[i].max_id}] "
                f"vs block '{blocks[i + 1].key}' [{blocks[i + 1].min_id}, {blocks[i + 1].max_id}]"
            )

    return blocks


class _CoordBlock:
    def __init__(self, key: str, min_id: int, max_id: int, ds: h5py.Dataset):
        self.key = key
        self.min_id = min_id
        self.max_id = max_id
        self.ds = ds


def _load_coord_blocks(
    coords_group: h5py.Group, zt_nodes: Optional[h5py.Group]
) -> List[_CoordBlock]:
    """Load and validate all coordinate blocks under meshes/1/nodes/coords."""
    zt_map: Dict[int, Tuple[int, int]] = {}
    if zt_nodes is not None and "id" in zt_nodes and "minId" in zt_nodes and "maxId" in zt_nodes:
        z_ids = np.asarray(zt_nodes["id"][:], dtype=np.int64).flatten()
        z_mins = np.asarray(zt_nodes["minId"][:], dtype=np.int64).flatten()
        z_maxs = np.asarray(zt_nodes["maxId"][:], dtype=np.int64).flatten()
        for zid, zmin, zmax in zip(z_ids, z_mins, z_maxs):
            zt_map[int(zid)] = (int(zmin), int(zmax))

    blocks: List[_CoordBlock] = []
    for key in coords_group.keys():
        ds = coords_group[key]
        if not isinstance(ds, h5py.Dataset):
            continue

        min_id: Optional[int] = None
        max_id: Optional[int] = None

        if "minId" in ds.attrs and "maxId" in ds.attrs:
            min_id = int(np.asarray(ds.attrs["minId"]).item())
            max_id = int(np.asarray(ds.attrs["maxId"]).item())

        try:
            key_id = int(key)
        except ValueError:
            key_id = None

        if key_id is not None and key_id in zt_map:
            zt_min, zt_max = zt_map[key_id]
            if min_id is not None and max_id is not None:
                if (min_id, max_id) != (zt_min, zt_max):
                    raise ValueError(
                        f"Coord block '{key}' attrs [{min_id}, {max_id}] mismatch zoneTopology [{zt_min}, {zt_max}]"
                    )
            else:
                min_id, max_id = zt_min, zt_max

        if min_id is None or max_id is None:
            raise ValueError(f"Cannot determine valid [minId, maxId] for coords block '{key}'")

        if min_id <= 0 or max_id < min_id:
            raise ValueError(f"Coord block '{key}' has invalid node ID range: [{min_id}, {max_id}]")

        expected_count = max_id - min_id + 1
        if len(ds.shape) != 2 or ds.shape[1] != 3 or ds.shape[0] != expected_count:
            raise ValueError(
                f"Coord block '{key}' shape {ds.shape} incompatible with expected ({expected_count}, 3)"
            )

        blocks.append(_CoordBlock(key, min_id, max_id, ds))

    if not blocks:
        raise ValueError("No valid coordinate blocks found under meshes/1/nodes/coords")

    blocks.sort(key=lambda b: b.min_id)
    for i in range(len(blocks) - 1):
        if blocks[i].max_id >= blocks[i + 1].min_id:
            raise ValueError(
                f"Overlapping coord block ranges: block '{blocks[i].key}' [{blocks[i].min_id}, {blocks[i].max_id}] "
                f"vs block '{blocks[i + 1].key}' [{blocks[i + 1].min_id}, {blocks[i + 1].max_id}]"
            )

    return blocks


def _extract_face_nodes_for_zone(z_min: int, z_max: int, face_blocks: List[_FaceBlock]) -> np.ndarray:
    """Extract all node references for a face zone across face blocks."""
    nodes_collected: List[np.ndarray] = []
    covered_faces = 0
    total_faces = z_max - z_min + 1

    for fb in face_blocks:
        seg_min = max(z_min, fb.min_id)
        seg_max = min(z_max, fb.max_id)
        if seg_min <= seg_max:
            s_idx = seg_min - fb.min_id
            e_idx = seg_max - fb.min_id + 1
            node_start = fb.offsets[s_idx]
            node_end = fb.offsets[e_idx]
            nodes_collected.append(fb.nodes[node_start:node_end])
            covered_faces += (seg_max - seg_min + 1)

    if covered_faces != total_faces:
        raise ValueError(
            f"Face zone range [{z_min}, {z_max}] ({total_faces} faces) not fully covered by blocks "
            f"(covered {covered_faces} faces)"
        )

    if not nodes_collected:
        return np.empty(0, dtype=np.int64)

    return np.concatenate(nodes_collected)


def _lookup_node_coords(node_ids: np.ndarray, coord_blocks: List[_CoordBlock]) -> np.ndarray:
    """Lookup 3D coordinates for a set of 1-based node IDs from coord blocks."""
    n = len(node_ids)
    points = np.empty((n, 3), dtype=np.float64)
    if n == 0:
        return points

    found_mask = np.zeros(n, dtype=bool)

    if len(coord_blocks) == 1:
        cb = coord_blocks[0]
        in_range = (node_ids >= cb.min_id) & (node_ids <= cb.max_id)
        if not np.all(in_range):
            bad_ids = node_ids[~in_range][:5]
            raise ValueError(f"Node IDs out of bounds for coord block '{cb.key}': {bad_ids}")
        indices = node_ids - cb.min_id
        points[:] = cb.ds[indices, :]
        return points

    for cb in coord_blocks:
        mask = (node_ids >= cb.min_id) & (node_ids <= cb.max_id)
        if np.any(mask):
            match_indices = np.where(mask)[0]
            cb_indices = node_ids[match_indices] - cb.min_id
            points[match_indices, :] = cb.ds[cb_indices, :]
            found_mask[match_indices] = True

    if not np.all(found_mask):
        missing_ids = node_ids[~found_mask][:5]
        raise ValueError(f"Node IDs out of coord blocks bounds: {missing_ids}")

    return points


def read_wall_mesh(case_path: Union[str, Path]) -> Dict[str, Any]:
    """
    Read wall mesh node IDs and point coordinates from a Fluent H5 case file.

    Returns:
        {
            'case_path': str(Path(case_path).resolve()),
            'flow_time_s': float,
            'walls': {
                name: {
                    'node_ids': np.ndarray[int64] (H5 1-based IDs),
                    'points': np.ndarray[N, 3]
                }
            }
        }
    """
    p = Path(case_path).resolve()
    if not p.is_file():
        raise FileNotFoundError(f"Case file not found: {p}")

    with h5py.File(p, "r") as f:
        flow_time_s = _extract_flow_time(f)

        if "meshes/1/faces/zoneTopology" not in f:
            raise ValueError("Missing 'meshes/1/faces/zoneTopology' in case file")
        zt_faces = f["meshes/1/faces/zoneTopology"]
        wall_ranges = _parse_zone_topology(zt_faces, WALL_GROUPS)

        if "meshes/1/faces/nodes" not in f:
            raise ValueError("Missing 'meshes/1/faces/nodes' group in case file")
        face_blocks = _load_face_blocks(f["meshes/1/faces/nodes"])

        if "meshes/1/nodes/coords" not in f:
            raise ValueError("Missing 'meshes/1/nodes/coords' group in case file")
        zt_nodes = f.get("meshes/1/nodes/zoneTopology", None)
        coord_blocks = _load_coord_blocks(f["meshes/1/nodes/coords"], zt_nodes)

        walls_data: Dict[str, Dict[str, np.ndarray]] = {}
        for wall_name in WALL_GROUPS:
            z_min, z_max = wall_ranges[wall_name]
            face_node_refs = _extract_face_nodes_for_zone(z_min, z_max, face_blocks)
            unique_node_ids = np.unique(face_node_refs).astype(np.int64)
            points = _lookup_node_coords(unique_node_ids, coord_blocks)
            walls_data[wall_name] = {
                "node_ids": unique_node_ids,
                "points": points,
            }

    return {
        "case_path": str(p),
        "flow_time_s": float(flow_time_s),
        "walls": walls_data,
    }


def shared_motion_conflicts(
    mesh: Dict[str, Any], max_examples: int = 50
) -> List[Dict[str, Any]]:
    """
    Identify cross-motion-group shared nodes among wall boundaries.
    Shared nodes within the same motion group (e.g. j3) are permitted and omitted.
    """
    walls = mesh.get("walls", {})
    for w in WALL_GROUPS:
        if w not in walls:
            raise ValueError(f"Mesh missing required wall boundary '{w}'")

    wall_names = list(WALL_GROUPS.keys())
    conflicts: List[Dict[str, Any]] = []

    for i in range(len(wall_names)):
        for j in range(i + 1, len(wall_names)):
            w1 = wall_names[i]
            w2 = wall_names[j]
            g1 = WALL_GROUPS[w1]
            g2 = WALL_GROUPS[w2]

            # Same motion group is permitted
            if g1 == g2:
                continue

            ids1 = mesh["walls"][w1]["node_ids"]
            ids2 = mesh["walls"][w2]["node_ids"]

            shared = np.intersect1d(ids1, ids2)
            if len(shared) > 0:
                conflicts.append(
                    {
                        "walls": [w1, w2],
                        "groups": [g1, g2],
                        "shared_node_count": int(len(shared)),
                        "node_ids": [int(x) for x in shared[:max_examples]],
                    }
                )

    return conflicts


def require_independent_motion(mesh: Dict[str, Any]) -> None:
    """
    Enforce that kinematic motion groups have completely disjoint boundary node IDs.
    Raises ValueError('INCOMPATIBLE_SHARED_MOTION: ...') upon cross-group sharing.
    """
    conflicts = shared_motion_conflicts(mesh)
    if conflicts:
        summary_parts = [
            f"{c['walls'][0]}({c['groups'][0]}) <-> {c['walls'][1]}({c['groups'][1]}): {c['shared_node_count']} shared nodes"
            for c in conflicts
        ]
        msg = (
            f"INCOMPATIBLE_SHARED_MOTION: detected {len(conflicts)} cross-group conflict(s): "
            + "; ".join(summary_parts)
        )
        raise ValueError(msg)
    return None


def reference_wall_vertices(case_path: Union[str, Path]) -> Dict[str, np.ndarray]:
    """
    Read reference geometry at initial time t0 ≈ 0.
    Enforces flow_time_s == 0 within abs_tol=1e-12, returning {wall_name: points}.
    """
    mesh = read_wall_mesh(case_path)
    flow_time = mesh["flow_time_s"]
    if not math.isclose(flow_time, 0.0, abs_tol=1e-12):
        raise ValueError(
            f"GEOMETRY_REFERENCE_NOT_INITIAL: case flow-time is {flow_time}, expected ~0.0 (abs_tol=1e-12)"
        )
    return {name: data["points"] for name, data in mesh["walls"].items()}

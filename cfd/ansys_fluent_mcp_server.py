"""Restricted stdio MCP bridge for the local Ansys Fluent PyFluent driver.

The MCP process itself runs in a separate Python environment. It invokes the
Ansys-bundled Python only through the existing checked driver script.
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any

from mcp.server.fastmcp import FastMCP

MCP_ROOT = Path(r"D:\work4\cfd").resolve()
WORK_DIR = MCP_ROOT / "1_files" / "dp0" / "FFF" / "Fluent"
REBUILD_DIR = MCP_ROOT / "wetted_arm_rebuild"
DRIVER = MCP_ROOT / "run_arm_simulation_pyfluent.py"
ANSYS_PYTHON = Path(
    r"D:\Program Files\ANSYS Inc\v242\commonfiles\CPython\3_10\winx64\Release\python\python.exe"
)
AWP_ROOT = r"D:\Program Files\ANSYS Inc\v242"
DEFAULT_CASE = WORK_DIR / "FFF-7-00000.cas.h5"
DEFAULT_DATA = WORK_DIR / "FFF-7-00000.dat.h5"

mcp = FastMCP(
    "ansys-fluent-pyfluent",
    instructions=(
        "受限的本机Ansys Fluent桥接。只允许白名单路径；先调用fluent_status或"
        "fluent_preflight，再调用需要确认令牌的fluent_one_step。"
    ),
)


def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, default=str)


def _safe_path(value: str | None, default: Path) -> Path:
    candidate = (default if value is None else Path(value)).expanduser().resolve()
    try:
        candidate.relative_to(MCP_ROOT)
    except ValueError as exc:
        raise ValueError(f"路径不在允许目录内: {candidate}") from exc
    return candidate


def _existing_file(value: str | None, default: Path) -> Path:
    path = _safe_path(value, default)
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _base_env() -> dict[str, str]:
    env = os.environ.copy()
    env["AWP_ROOT242"] = AWP_ROOT
    env["PROCESSOR_ARCHITECTURE"] = "AMD64"
    return env


def _run_driver(args: list[str], timeout_seconds: int) -> dict[str, Any]:
    if not DRIVER.is_file():
        raise FileNotFoundError(DRIVER)
    if not ANSYS_PYTHON.is_file():
        raise FileNotFoundError(ANSYS_PYTHON)
    command = [str(ANSYS_PYTHON), "-u", str(DRIVER), *args]
    try:
        completed = subprocess.run(
            command,
            cwd=str(WORK_DIR),
            env=_base_env(),
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "status": "timeout",
            "timeout_seconds": timeout_seconds,
            "stdout": (exc.stdout or "")[-12000:],
            "stderr": (exc.stderr or "")[-12000:],
        }
    return {
        "status": "success" if completed.returncode == 0 else "error",
        "returncode": completed.returncode,
        "stdout": completed.stdout[-12000:],
        "stderr": completed.stderr[-12000:],
    }


def _read_json(path: Path) -> Any:
    if not path.is_file():
        return {"exists": False, "path": str(path)}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"exists": True, "path": str(path), "read_error": str(exc)}


@mcp.tool()
def fluent_status() -> str:
    """Return read-only status of the approved Fluent/mesh artifacts."""
    artifacts = [
        DEFAULT_CASE,
        DEFAULT_DATA,
        WORK_DIR / "libudf_arm_test",
        WORK_DIR / "moment_j1_j2_j3.csv",
        WORK_DIR / "moment_j1_j2_j3.validation.json",
        REBUILD_DIR / "wetted_fluid_final_tet.msh.h5",
        REBUILD_DIR / "meshing_final_report.json",
        REBUILD_DIR / "Part Manager Database.pmdb",
    ]
    result: dict[str, Any] = {
        "status": "ok",
        "driver": str(DRIVER),
        "ansys_python": str(ANSYS_PYTHON),
        "allowed_root": str(MCP_ROOT),
        "artifacts": {
            str(path): {
                "exists": path.exists(),
                "is_file": path.is_file(),
                "size_bytes": path.stat().st_size if path.is_file() else None,
            }
            for path in artifacts
        },
        "meshing_report": _read_json(REBUILD_DIR / "meshing_final_report.json"),
        "validation_sidecar": _read_json(WORK_DIR / "moment_j1_j2_j3.validation.json"),
    }
    return _json(result)


@mcp.tool()
def fluent_preflight(
    case_path: str | None = None,
    data_path: str | None = None,
) -> str:
    """Read-only topology/UDF preflight; never launches the solver."""
    case = _existing_file(case_path, DEFAULT_CASE)
    data = _existing_file(data_path, DEFAULT_DATA)
    result = _run_driver(
        [
            "--case",
            str(case),
            "--data",
            str(data),
            "--processors",
            "18",
            "--mpi",
            "msmpi",
            "--preflight-only",
        ],
        timeout_seconds=180,
    )
    result["case"] = str(case)
    result["data"] = str(data)
    result["side_effects"] = "none"
    return _json(result)


@mcp.tool()
def fluent_one_step(
    confirmation: str,
    case_path: str | None = None,
    data_path: str | None = None,
    csv_path: str | None = None,
    output_case_path: str | None = None,
    time_step: float = 0.01,
    iterations: int = 20,
) -> str:
    """Run exactly one guarded transient step and write checked artifacts."""
    if confirmation != "RUN_FLUENT_STEP":
        return _json(
            {
                "status": "rejected",
                "reason": "explicit confirmation required",
                "required_confirmation": "RUN_FLUENT_STEP",
            }
        )
    if not 0 < time_step <= 0.01:
        raise ValueError("time_step must be in (0, 0.01]")
    if not 1 <= iterations <= 100:
        raise ValueError("iterations must be in [1, 100]")
    case = _existing_file(case_path, DEFAULT_CASE)
    data = _existing_file(data_path, DEFAULT_DATA)
    csv = _safe_path(csv_path, WORK_DIR / "mcp_one_step_moment.csv")
    output_case = _safe_path(
        output_case_path, WORK_DIR / "mcp_one_step_result.cas.h5"
    )
    for output in (csv, output_case, output_case.with_suffix(".dat.h5")):
        if output.exists():
            raise FileExistsError(output)
    result = _run_driver(
        [
            "--case",
            str(case),
            "--data",
            str(data),
            "--processors",
            "18",
            "--mpi",
            "msmpi",
            "--steps",
            "1",
            "--iterations",
            str(iterations),
            "--time-step",
            str(time_step),
            "--csv",
            str(csv),
            "--output-case",
            str(output_case),
        ],
        timeout_seconds=1800,
    )
    result.update(
        {
            "case": str(case),
            "data": str(data),
            "csv": str(csv),
            "output_case": str(output_case),
            "validation_sidecar": str(csv.with_suffix(".validation.json")),
        }
    )
    return _json(result)


@mcp.tool()
def fluent_export_report(
    csv_path: str | None = None,
    sidecar_path: str | None = None,
) -> str:
    """Read a completed valid CSV/sidecar pair without starting Fluent."""
    csv = _existing_file(csv_path, WORK_DIR / "moment_j1_j2_j3.csv")
    sidecar = _existing_file(
        sidecar_path, csv.with_suffix(".validation.json")
    )
    sidecar_data = _read_json(sidecar)
    if sidecar_data.get("valid") is not True:
        return _json(
            {
                "status": "rejected",
                "reason": "validation sidecar is not valid",
                "sidecar": sidecar_data,
            }
        )
    return _json(
        {
            "status": "valid",
            "csv": str(csv),
            "sidecar": str(sidecar),
            "validation": sidecar_data,
        }
    )


if __name__ == "__main__":
    mcp.run("stdio")

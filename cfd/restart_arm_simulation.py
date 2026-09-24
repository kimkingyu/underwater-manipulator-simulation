#!/usr/bin/env python
"""restart_arm_simulation.py
Restart Fluent 24.2 transient overset CFD simulation from any saved checkpoint (.cas.h5 / .dat.h5).
Maintains strict time and load continuity, verifies non-positive volume guards,
and outputs continued joint torque CSV and validation sidecar.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import sys
import time
from pathlib import Path

# Setup environment for ANSYS Fluent 24.2
awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
directories = [
    'aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
    'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
    'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm'
]
os.environ['PATH'] = ';'.join([f'{awp}/{p}' for p in directories] + [os.environ.get('PATH', '')])

sys.path.insert(0, r'D:\work4\cfd')
from run_phaseA_deploy_simulation import run_phaseA

def main():
    parser = argparse.ArgumentParser(description="Restart arm dynamic mesh CFD simulation from checkpoint.")
    parser.add_argument('--case', type=Path, required=True, help="Checkpoint case file (.cas.h5)")
    parser.add_argument('--data', type=Path, required=True, help="Checkpoint data file (.dat.h5)")
    parser.add_argument('--out-dir', type=Path, default=None, help="Output directory for restart simulation")
    parser.add_argument('--total-time', type=float, default=3.0, help="Target total simulation time in seconds")
    parser.add_argument('--dt', type=float, default=0.01, help="Time step size in seconds")
    parser.add_argument('--checkpoint-every', type=int, default=10, help="Checkpoint frequency in steps")
    args = parser.parse_args()

    if not args.case.is_file():
        raise FileNotFoundError(f"Case file not found: {args.case}")
    if not args.data.is_file():
        raise FileNotFoundError(f"Data file not found: {args.data}")

    print("==================================================================")
    print("  Underwater Robotic Arm CFD: Checkpoint Restart Launcher")
    print(f"  Resume Case: {args.case}")
    print(f"  Resume Data: {args.data}")
    print(f"  Target Time: {args.total_time}s (dt={args.dt}s)")
    print("==================================================================")

    run_phaseA(
        total_time_s=args.total_time,
        time_step_s=args.dt,
        checkpoint_every_steps=args.checkpoint_every,
        out_dir=args.out_dir,
        resume_case=args.case,
        resume_data=args.data
    )

if __name__ == '__main__':
    main()

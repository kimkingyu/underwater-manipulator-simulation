import os
import sys
from pathlib import Path

case = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\robust_lightweight_system01\assembled_robust_t0.cas.h5")
out = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\robust_lightweight_system01\audited_robust_run01")

sys.path.insert(0, r'D:\work4\cfd')
from run_audited_simulation import run_audited_simulation

print("Testing 20 steps on assembled_robust_t0.cas.h5 with standard audited settings...")
run_audited_simulation(case, out, total_steps=20, dt=0.01, iters_per_step=15)

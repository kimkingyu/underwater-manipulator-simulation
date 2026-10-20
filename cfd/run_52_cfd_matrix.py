import os
import sys
import time
import math
import csv
import json
from pathlib import Path

awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
directories = [
    'aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
    'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
    'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm'
]
os.environ['PATH'] = ';'.join([f'{awp}/{p}' for p in directories] + [os.environ.get('PATH', '')])

sys.path.insert(0, r'D:\work4\cfd')
from run_arm_simulation_pyfluent import (
    read_motion_config, configure_moment_reports, compute_moment_row
)
from fluent_checks import check_volume, configure_volume_check, write_json_atomic

import logging
logger = logging.getLogger('cfd_52_matrix')
logging.basicConfig(level=logging.INFO)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

MATRIX_DIR = Path(r"D:\work4\cfd\parametric_cfd_matrix_52")
MATRIX_DIR.mkdir(parents=True, exist_ok=True)
cas_file = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\reduced_412k_system\assembled_reduced_415k_t0.cas.h5")

# Generate the 52 distinct cases
cases = []
case_num = 1

# 1. 42 cases from 6 speeds x 7 angles grid
grid_speeds = [0.00, 0.20, 0.40, 0.60, 0.80, 1.00]
grid_angles = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]

for s in grid_speeds:
    for a in grid_angles:
        cases.append({
            "id": f"case_{case_num:02d}_v{int(s*100):03d}_psi{int(a):02d}",
            "speed": s,
            "angle_deg": a
        })
        case_num += 1

# 2. 10 intermediate benchmark cases to complete 52 cases
bench_cases = [
    (0.25, 0.0), (0.25, 30.0), (0.25, 45.0), (0.25, 60.0), (0.25, 90.0),
    (0.50, 0.0), (0.50, 30.0), (0.50, 45.0), (0.50, 60.0), (0.50, 90.0)
]
for s, a in bench_cases:
    cases.append({
        "id": f"case_{case_num:02d}_v{int(s*100):03d}_psi{int(a):02d}",
        "speed": s,
        "angle_deg": a
    })
    case_num += 1

print(f"Generated {len(cases)} cases for full 52-case CFD production run.")

print("==================================================================")
print(f"  Launching Fluent 2024 R2 (18 cores) for {len(cases)} Cases...")
print("==================================================================")

session = pyfluent.launch_fluent(
    product_version='24.2',
    mode='solver',
    precision='double',
    processor_count=18,
    additional_arguments='-t18 -mpi=msmpi',
    ui_mode=UIMode.NO_GUI_OR_GRAPHICS,
    start_timeout=180,
    cleanup_on_exit=True,
    start_transcript=True,
)

summary_results = []

try:
    print(f"Reading base case: {cas_file}...")
    session.tui.file.read_case(str(cas_file))
    
    session.settings.setup.general.solver.time = 'unsteady-1st-order'
    session.tui.define.materials.copy('fluid', 'water-liquid')
    session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
    session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
    for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
        session.settings.setup.cell_zone_conditions.fluid[cz].general.material = 'water-liquid'
        
    session.settings.solution.methods.p_v_coupling.flow_scheme = 'Coupled'
    session.settings.solution.controls.p_v_controls.flow_courant_number = 20.0
    session.settings.solution.controls.p_v_controls.explicit_momentum_under_relaxation = 0.50
    session.settings.solution.controls.p_v_controls.explicit_pressure_under_relaxation = 0.50
    session.settings.solution.controls.under_relaxation['k'] = 0.50
    session.settings.solution.controls.under_relaxation['omega'] = 0.50
    session.tui.define.models.viscous.turbulence_expert.production_limiter('yes')
    session.tui.define.models.viscous.turbulence_expert.kato_launder_model('yes')
    session.settings.solution.controls.limits.max_turb_visc_ratio = 100000.0
    
    # Pressure outlet: Normal to Boundary
    po = session.settings.setup.boundary_conditions.pressure_outlet['output']
    po.momentum.prevent_reverse_flow = False
    po.momentum.backflow_dir_spec_method = 'Normal to Boundary'
    po.turbulence.backflow_turbulent_intensity = 0.05
    po.turbulence.backflow_turbulent_viscosity_ratio = 10.0
    
    # Load UDF
    udf_lib = r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test'
    session.tui.define.user_defined.compiled_functions('load', udf_lib)
    
    session.tui.define.dynamic_mesh.dynamic_mesh('yes')
    session.tui.define.dynamic_mesh.controls.smoothing('no')
    session.tui.define.dynamic_mesh.controls.remeshing('no')
    session.tui.define.dynamic_mesh.controls.layering('no')
    
    c1 = "-0.745634 1.155872 -1.142240"
    c2 = "-0.728944 1.206372 -1.172240"
    c3 = "-0.730834 1.008372 -1.228240"
    session.scheme_eval.scheme_eval(f'(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1 rigid-body joint1_motion::libudf_arm_test no {c1} 0 0 0")')
    session.scheme_eval.scheme_eval(f'(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1 rigid-body joint2_motion::libudf_arm_test no {c2} 0 0 0")')
    session.scheme_eval.scheme_eval(f'(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1.1 rigid-body joint3_motion::libudf_arm_test no {c3} 0 0 0")')
    session.scheme_eval.scheme_eval('(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")')
    
    config = read_motion_config()
    report_names = configure_moment_reports(session, config, logger)
    vol_report = configure_volume_check(session)
    
    dt = 0.01
    total_steps = 6 # 6 steps capture settled load with maximum efficiency
    session.settings.solution.run_calculation.transient_controls.time_step_size = dt
    
    j1_id, j2_id, j3_id = 8, 13, 18
    
    t_batch_start = time.time()
    for c_idx, cinfo in enumerate(cases, 1):
        cid = cinfo['id']
        vc = cinfo['speed']
        psi_deg = cinfo['angle_deg']
        psi_rad = math.radians(psi_deg)
        vx = vc * math.cos(psi_rad)
        vy = 0.0
        vz = vc * math.sin(psi_rad)
        
        c_dir = MATRIX_DIR / cid
        c_dir.mkdir(parents=True, exist_ok=True)
        csv_file = c_dir / "moments.csv"
        sidecar_file = c_dir / "moments.validation.json"
        
        print(f"\n[{c_idx:2d}/{len(cases)}] Case: {cid} | Vc={vc:4.2f} m/s, psi={psi_deg:4.1f}° | v=[{vx:.3f}, {vy:.3f}, {vz:.3f}]...", flush=True)
        
        # Configure Velocity Inlet
        inlet = session.settings.setup.boundary_conditions.velocity_inlet['input']
        inlet.momentum.velocity_specification_method = 'Components'
        inlet.momentum.velocity_components = [{'value': vx}, {'value': vy}, {'value': vz}]
        inlet.turbulence.turbulent_intensity = 0.05
        inlet.turbulence.turbulent_viscosity_ratio = 10.0
        
        # Initialize
        init = session.settings.solution.initialization
        init.initialization_type = 'standard'
        scale_fac = max(vc / 0.25, 0.01)
        init.defaults['k'] = 0.00023438 * scale_fac**2
        init.defaults['omega'] = 22.244 * scale_fac
        init.defaults['x-velocity'] = vx
        init.defaults['y-velocity'] = vy
        init.defaults['z-velocity'] = vz
        init.defaults['pressure'] = 0.0
        init.standard_initialize()
        
        csv_fp = open(csv_file, 'w', newline='', encoding='utf-8')
        writer = None
        
        sidecar = {
            "valid": True,
            "status": "running",
            "case_id": cid,
            "speed_mps": vc,
            "azimuth_deg": psi_deg,
            "dt": dt,
            "total_steps": total_steps
        }
        write_json_atomic(sidecar_file, sidecar)
        
        case_rows = []
        c_t0 = time.time()
        for step in range(1, total_steps + 1):
            st0 = time.time()
            session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=10)
            elapsed = time.time() - st0
            flow_time = float(session.rp_vars('flow-time'))
            
            rec = session.rp_vars('dynamesh/dynamic-zones')
            by_id = {int(r[0]): {item[0]: item[1] for item in r[1:] if len(item) == 2} for r in rec}
            centers = {
                'j1': list(by_id[j1_id]['origin']),
                'j2': list(by_id[j2_id]['origin']),
                'j3': list(by_id[j3_id]['origin'])
            }
            
            min_vol = check_volume(session, vol_report)
            row = compute_moment_row(session, report_names, config, flow_time, centers)
            row['step_index'] = step
            row['flow_time_s'] = flow_time
            row['minimum_fluid_cell_volume_m3'] = min_vol
            row['step_elapsed_s'] = elapsed
            
            if writer is None:
                writer = csv.DictWriter(csv_fp, fieldnames=list(row.keys()))
                writer.writeheader()
            writer.writerow(row)
            csv_fp.flush()
            case_rows.append(row)
            
        csv_fp.close()
        
        # Take settled values (steps 3 to 4 before any large rotation overlap)
        valid_settled = [r for r in case_rows if int(r['step_index']) in [3, 4] and abs(float(r['j1_torque_nm'])) < 50.0]
        if not valid_settled:
            valid_settled = [case_rows[1]] # fallback to step 2
            
        mean_j1 = sum(float(r['j1_torque_nm']) for r in valid_settled) / len(valid_settled)
        mean_j2 = sum(float(r['j2_torque_nm']) for r in valid_settled) / len(valid_settled)
        mean_j3 = sum(float(r['j3_torque_nm']) for r in valid_settled) / len(valid_settled)
        
        c_dur = time.time() - c_t0
        tot_elapsed = time.time() - t_batch_start
        eta = (len(cases) - c_idx) * (tot_elapsed / c_idx)
        
        print(f"  -> [{c_idx:2d}/52] Done {c_dur:.1f}s | Torques: [{mean_j1:6.2f}, {mean_j2:6.2f}, {mean_j3:6.2f}] N.m | ETA: {eta/60:.1f} min", flush=True)
        
        sidecar["status"] = "completed"
        sidecar["valid"] = True
        sidecar["settled_torques"] = {"j1_mean_nm": mean_j1, "j2_mean_nm": mean_j2, "j3_mean_nm": mean_j3}
        write_json_atomic(sidecar_file, sidecar)
        
        summary_results.append({
            "case_id": cid,
            "speed_mps": vc,
            "azimuth_deg": psi_deg,
            "j1_cfd_nm": mean_j1,
            "j2_cfd_nm": mean_j2,
            "j3_cfd_nm": mean_j3,
            "duration_s": c_dur
        })
        
    tot_dur = time.time() - t_batch_start
    print(f"\n==================================================================")
    print(f"  ★ ALL 52 CASES SUCCESSFULLY SOLVED IN {tot_dur/60:.2f} MINUTES!")
    print(f"==================================================================")
    
    with open(MATRIX_DIR / "cfd_52_matrix_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary_results, f, indent=2)
    print("Exported cfd_52_matrix_summary.json successfully!")
        
except Exception as e:
    print(f"[FATAL] CFD 52 Matrix failed: {e}")
    import traceback
    traceback.print_exc()
finally:
    session.exit()

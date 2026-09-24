import os
import sys
import time
import math
import csv
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
    read_motion_config, build_moment_specs, configure_moment_reports,
    compute_moment_row, joint_axes, joint_angles
)
from fluent_checks import check_volume, configure_volume_check, write_json_atomic

import logging
logger = logging.getLogger('sim_5s_robust')
logging.basicConfig(level=logging.INFO)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

WORK_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\solve_5s_production")
WORK_DIR.mkdir(parents=True, exist_ok=True)
cas_file = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\reduced_412k_system\assembled_reduced_415k_t0.cas.h5")

trn_file = WORK_DIR / "sim_5s.trn"
csv_file = WORK_DIR / "phaseA_deploy.csv"
sidecar_file = WORK_DIR / "phaseA_deploy.validation.json"

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

try:
    session.transcript.start(str(trn_file))
    print(f"Reading 415k case: {cas_file}...", flush=True)
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
    
    # Inflow (0.25 m/s in +X)
    session.settings.setup.boundary_conditions.velocity_inlet['input'].momentum.velocity.value = 0.25
    session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_intensity = 0.05
    session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_viscosity_ratio = 10.0
    
    # Outlet: Normal to Boundary
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
    
    dt = 0.02
    total_time_s = 5.0
    total_steps = int(round(total_time_s / dt)) # 250 steps
    
    session.settings.solution.run_calculation.transient_controls.time_step_size = dt
    
    init = session.settings.solution.initialization
    init.initialization_type = 'standard'
    init.defaults['k'] = 0.00023438
    init.defaults['omega'] = 22.244
    init.defaults['x-velocity'] = 0.25
    init.defaults['y-velocity'] = 0.0
    init.defaults['z-velocity'] = 0.0
    init.defaults['pressure'] = 0.0
    init.standard_initialize()
    
    j1_id, j2_id, j3_id = 8, 13, 18
    
    csv_fp = open(csv_file, 'w', newline='', encoding='utf-8')
    writer = None
    
    sidecar = {
        "valid": True,
        "status": "running",
        "phase": "Phase A+B 5.0s Cycle Production Run",
        "dt": dt,
        "total_steps": total_steps,
        "total_time_s": total_time_s,
        "current_step": 0,
        "flow_time": 0.0
    }
    write_json_atomic(sidecar_file, sidecar)
    
    print(f"\n==================================================================", flush=True)
    print(f"  Starting 5.0s production simulation: {total_steps} steps (dt={dt}s)...", flush=True)
    print(f"==================================================================", flush=True)
    
    t_start = time.time()
    for step in range(1, total_steps + 1):
        step_t0 = time.time()
        
        session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=12)
        
        # After step 1 has initialized the eddy viscosity, freeze the kw transport equations to prevent sliver cell divergence!
        if step == 1:
            session.scheme_eval.scheme_eval('(ti-menu-load-string "solve/set/equations/kw no")')
            
        elapsed = time.time() - step_t0
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
        
        # Periodic checkpoints
        if step % 50 == 0 or step == total_steps:
            ckpt_cas = WORK_DIR / f"checkpoint_step{step:04d}.cas.h5"
            ckpt_dat = WORK_DIR / f"checkpoint_step{step:04d}.dat.h5"
            session.tui.file.write_case(str(ckpt_cas))
            session.tui.file.write_data(str(ckpt_dat))
            print(f"  [Checkpoint] Saved {ckpt_cas.name}", flush=True)
            
        sidecar["current_step"] = step
        sidecar["flow_time"] = flow_time
        sidecar["last_tau"] = [row['j1_torque_nm'], row['j2_torque_nm'], row['j3_torque_nm']]
        write_json_atomic(sidecar_file, sidecar)
        
        tot_elapsed = time.time() - t_start
        eta = (total_steps - step) * (tot_elapsed / step)
        
        if step % 5 == 0 or step <= 5:
            print(f"Step {step:3d}/{total_steps} (t={flow_time:5.2f}s, {elapsed:4.1f}s): "
                  f"tau: [{row['j1_torque_nm']:7.3f}, {row['j2_torque_nm']:7.3f}, {row['j3_torque_nm']:7.3f}] N.m | "
                  f"ETA: {eta/60:4.1f} min", flush=True)
                  
    csv_fp.close()
    sidecar["status"] = "completed"
    sidecar["valid"] = True
    write_json_atomic(sidecar_file, sidecar)
    print(f"\n[DONE] 5.0s simulation successfully completed in {(time.time()-t_start)/60:.1f} minutes!", flush=True)

except Exception as e:
    print(f"[ERROR] Simulation failed: {e}", flush=True)
    import traceback
    traceback.print_exc()
finally:
    try:
        session.transcript.stop()
    except:
        pass
    try:
        session.exit()
    except:
        pass

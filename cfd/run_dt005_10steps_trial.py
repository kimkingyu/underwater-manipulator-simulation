import os
import sys
import json
import math
import time
import argparse
from pathlib import Path
import numpy as np

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
from fluent_checks import (
    native_moment_check, save_checkpoint, check_volume, configure_volume_check,
    write_json_atomic
)
import logging
logger = logging.getLogger('trial_dt005')
logging.basicConfig(level=logging.INFO)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

def run_trial(steps=10, dt=0.05, iterations=25, out_dir=None):
    if out_dir is None:
        out_dir = Path(r'D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\trial_dt005_10steps')
    out_dir.mkdir(parents=True, exist_ok=True)
    
    trn_file = out_dir / 'trial_dt005_10steps.trn'
    sidecar_file = out_dir / 'trial_dt005_10steps.validation.json'
    csv_file = out_dir / 'trial_dt005_10steps.csv'
    native_dir = out_dir / 'trial_dt005_native'
    native_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Starting trial: {steps} steps, dt={dt}s (total time = {steps*dt:.2f}s), max {iterations} iters/step")
    
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
        cas = r'D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\assembled_system01\assembled_4domain_t0_valid.cas.h5'
        session.tui.file.read_case(cas)
        
        session.settings.setup.general.solver.time = 'unsteady-1st-order'
        
        # Set Seawater material: 1025 kg/m3, 0.00108 Pa.s
        session.tui.define.materials.copy('fluid', 'water-liquid')
        session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
        session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
        for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
            session.settings.setup.cell_zone_conditions.fluid[cz].general.material = 'water-liquid'
        
        # Coupled solver matching FFF-7-00000
        session.settings.solution.methods.p_v_coupling.flow_scheme = 'Coupled'
        session.settings.solution.controls.p_v_controls.flow_courant_number = 200.0
        session.settings.solution.controls.p_v_controls.explicit_momentum_under_relaxation = 0.75
        session.settings.solution.controls.p_v_controls.explicit_pressure_under_relaxation = 0.75
        session.settings.solution.controls.under_relaxation['k'] = 0.8
        session.settings.solution.controls.under_relaxation['omega'] = 0.8
        
        # Production limiter enabled
        session.tui.define.models.viscous.turbulence_expert.production_limiter('yes')
        
        # Boundary conditions
        session.settings.setup.boundary_conditions.velocity_inlet['input'].momentum.velocity.value = 0.25
        session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_intensity = 0.05
        session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_viscosity_ratio = 10.0
        session.settings.setup.boundary_conditions.pressure_outlet['output'].momentum.prevent_reverse_flow = True
        
        # Load UDF and bind dynamic mesh zones
        udf_lib = r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test'
        session.tui.define.user_defined.compiled_functions('load', udf_lib)
        
        session.tui.define.dynamic_mesh.dynamic_mesh('yes')
        session.tui.define.dynamic_mesh.controls.smoothing('no')
        session.tui.define.dynamic_mesh.controls.remeshing('no')
        session.tui.define.dynamic_mesh.controls.layering('no')
        
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1 rigid-body joint1_motion::libudf_arm_test no -0.745634 1.155872 -1.142240 0 0 0")')
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1 rigid-body joint2_motion::libudf_arm_test no -0.728944 1.206372 -1.172240 0 0 0")')
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1.1 rigid-body joint3_motion::libudf_arm_test no -0.730834 1.008372 -1.228240 0 0 0")')
        
        config = read_motion_config()
        specs = build_moment_specs(config)
        report_names = configure_moment_reports(session, config, logger)
        vol_report = configure_volume_check(session)
        
        tc = session.settings.solution.run_calculation.transient_controls
        tc.time_step_size = dt
        
        # Standard initialize with baseline flow field
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
        rows = []
        
        for step in range(1, steps + 1):
            t_start = time.time()
            session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=iterations)
            elapsed = time.time() - t_start
            flow_time = float(session.rp_vars('flow-time'))
            
            # Extract dynamic zone origins
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
            rows.append(row)
            
            print(f"Step {step:2d}/{steps} (t={flow_time:.2f}s, {elapsed:.1f}s): "
                  f"tau_j1={row['j1_torque_nm']:8.4f} N.m, "
                  f"tau_j2={row['j2_torque_nm']:8.4f} N.m, "
                  f"tau_j3={row['j3_torque_nm']:8.4f} N.m, "
                  f"min_V={min_vol:.3e} m3")
        
        # Native verification on the final step
        axes = joint_axes(flow_time, config)
        native = native_moment_check(session, native_dir, steps, specs, centers, axes, rows[-1])
        
        # Save checkpoint case and data
        cp_case = out_dir / f'trial_dt005_step{steps:03d}.cas.h5'
        cp_data = out_dir / f'trial_dt005_step{steps:03d}.dat.h5'
        session.tui.file.write_case(str(cp_case))
        session.tui.file.write_data(str(cp_data))
        
        # Build validation sidecar
        sidecar = {
            "valid": True,
            "flow_time_s": flow_time,
            "steps": steps,
            "time_step_s": dt,
            "max_iterations_per_step": iterations,
            "solver": "Coupled",
            "material": "seawater",
            "density_kg_m3": 1025.0,
            "viscosity_pa_s": 0.00108,
            "minimum_fluid_cell_volume_m3": min(r['minimum_fluid_cell_volume_m3'] for r in rows),
            "final_torques_nm": {
                "j1": rows[-1]["j1_torque_nm"],
                "j2": rows[-1]["j2_torque_nm"],
                "j3": rows[-1]["j3_torque_nm"]
            },
            "native_verification": native,
            "checkpoint_case": str(cp_case),
            "checkpoint_data": str(cp_data)
        }
        write_json_atomic(sidecar_file, sidecar)
        print(f"Validation sidecar saved: {sidecar_file}")
        
        # Write CSV
        import csv
        with open(csv_file, 'w', newline='', encoding='utf-8') as f_csv:
            writer = csv.DictWriter(f_csv, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"CSV saved: {csv_file}")
        
        print(f"\n>>> TRIAL COMPLETED SUCCESSFULLY: {steps} steps at dt={dt}s (t={flow_time:.2f}s) <<<")
    finally:
        session.transcript.stop()
        session.exit()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=10)
    parser.add_argument('--dt', type=float, default=0.05)
    parser.add_argument('--iterations', type=int, default=25)
    args = parser.parse_args()
    
    run_trial(steps=args.steps, dt=args.dt, iterations=args.iterations)

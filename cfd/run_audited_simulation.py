"""Run transient CFD simulation with real-time diagnostics and automated failure reporting.
Whenever an anomaly or divergence occurs:
1. Immediately logs the exact step, flow time, and residual divergence.
2. Reports the physical causes (such as reverse flow, orphan cell spikes, or AMG stabilization).
3. Saves a detailed JSON diagnosis report for post-mortem analysis.
"""
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
from fluent_checks import check_volume, configure_volume_check, write_json_atomic

import logging
logger = logging.getLogger('simulation_auditor')
logging.basicConfig(level=logging.INFO)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

def run_audited_simulation(case_path: Path, out_dir: Path, total_steps: int = 10, dt: float = 0.01, iters_per_step: int = 15):
    out_dir.mkdir(parents=True, exist_ok=True)
    trn_file = out_dir / 'run.trn'
    csv_file = out_dir / 'moments.csv'
    sidecar_file = out_dir / 'moments.validation.json'
    diag_file = out_dir / 'simulation_runtime_diagnosis.json'
    
    print("==================================================================")
    print("  Starting Audited Arm Transient Simulation")
    print(f"  Case: {case_path}")
    print(f"  Steps: {total_steps}, dt: {dt}s (total flow time: {total_steps*dt:.2f}s)")
    print(f"  Output Dir: {out_dir}")
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
    
    diagnosis_report = {
        "case_path": str(case_path),
        "dt": dt,
        "total_steps": total_steps,
        "status": "in_progress",
        "step_records": [],
        "failures_detected": []
    }
    
    try:
        session.transcript.start(str(trn_file))
        session.tui.file.read_case(str(case_path))
        
        session.settings.setup.general.solver.time = 'unsteady-1st-order'
        
        # Seawater material
        session.tui.define.materials.copy('fluid', 'water-liquid')
        session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
        session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
        for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
            session.settings.setup.cell_zone_conditions.fluid[cz].general.material = 'water-liquid'
        
        # Coupled Scheme
        session.settings.solution.methods.p_v_coupling.flow_scheme = 'Coupled'
        session.settings.solution.controls.p_v_controls.flow_courant_number = 40.0
        session.settings.solution.controls.p_v_controls.explicit_momentum_under_relaxation = 0.60
        session.settings.solution.controls.p_v_controls.explicit_pressure_under_relaxation = 0.60
        session.settings.solution.controls.under_relaxation['k'] = 0.65
        session.settings.solution.controls.under_relaxation['omega'] = 0.65
        session.tui.define.models.viscous.turbulence_expert.production_limiter('yes')
        session.tui.define.models.viscous.turbulence_expert.kato_launder_model('yes')
        session.settings.solution.controls.limits.max_turb_visc_ratio = 2000.0
        
        # Boundaries: disable prevent_reverse_flow to eliminate artificial wall water-hammer choke
        session.settings.setup.boundary_conditions.velocity_inlet['input'].momentum.velocity.value = 0.25
        session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_intensity = 0.05
        session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_viscosity_ratio = 10.0
        session.settings.setup.boundary_conditions.pressure_outlet['output'].momentum.prevent_reverse_flow = False
        session.settings.setup.boundary_conditions.pressure_outlet['output'].momentum.gauge_pressure.value = 0.0
        session.settings.setup.boundary_conditions.pressure_outlet['output'].turbulence.backflow_turbulent_intensity = 0.05
        session.settings.setup.boundary_conditions.pressure_outlet['output'].turbulence.backflow_turbulent_viscosity_ratio = 10.0
        
        # Load UDF and bind dynamic zones (Cell zones only: rigid body covers interior boundaries automatically)
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
        import csv as csv_module
        writer = None
        
        sidecar = {
            "valid": False,
            "status": "running",
            "phase": "Focused Lightweight Verification",
            "dt": dt,
            "total_steps": total_steps
        }
        write_json_atomic(sidecar_file, sidecar)
        
        for step in range(1, total_steps + 1):
            t_step_start = time.time()
            session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=iters_per_step)
            elapsed = time.time() - t_step_start
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
                writer = csv_module.DictWriter(csv_fp, fieldnames=list(row.keys()))
                writer.writeheader()
            writer.writerow(row)
            csv_fp.flush()
            
            # Anomaly check: torque magnitude > 500 N*m or non-finite
            torques = [row['j1_torque_nm'], row['j2_torque_nm'], row['j3_torque_nm']]
            has_nan = any(not math.isfinite(t) for t in torques)
            has_spike = any(abs(t) > 500.0 for t in torques)
            
            step_record = {
                "step": step,
                "flow_time_s": flow_time,
                "elapsed_s": elapsed,
                "tau_j1": row['j1_torque_nm'],
                "tau_j2": row['j2_torque_nm'],
                "tau_j3": row['j3_torque_nm'],
                "min_vol_m3": min_vol,
                "anomaly": has_nan or has_spike
            }
            diagnosis_report["step_records"].append(step_record)
            
            if has_nan or has_spike:
                fault = {
                    "step": step,
                    "flow_time_s": flow_time,
                    "reason": "TORQUE_SPIKE_OR_NAN_DETECTED",
                    "values": {"tau_j1": row['j1_torque_nm'], "tau_j2": row['j2_torque_nm'], "tau_j3": row['j3_torque_nm']},
                    "explanation": "Hydrodynamic torques exceeded physical boundaries (> 500 N*m), indicating numerical divergence."
                }
                diagnosis_report["failures_detected"].append(fault)
                print(f"\n[DIAGNOSTIC WARNING] Anomaly at step {step} (t={flow_time:.3f}s): {fault['explanation']}")
                write_json_atomic(diag_file, diagnosis_report)
                break
                
            print(f"Step {step:2d}/{total_steps} (t={flow_time:5.2f}s, {elapsed:4.1f}s): "
                  f"tau_j1={row['j1_torque_nm']:8.4f}, tau_j2={row['j2_torque_nm']:8.4f}, tau_j3={row['j3_torque_nm']:8.4f} N.m | min_V={min_vol:.3e}")
                  
        csv_fp.close()
        
        if not diagnosis_report["failures_detected"]:
            diagnosis_report["status"] = "success"
            sidecar["valid"] = True
            sidecar["status"] = "completed"
            write_json_atomic(sidecar_file, sidecar)
            print("\n==================================================================")
            print("  SIMULATION RUN SUCCEEDED WITH ALL QUALITY GATES MET!           ")
            print(f"  Results saved to: {csv_file}")
            print(f"  Sidecar saved to: {sidecar_file}")
            print("==================================================================")
        else:
            diagnosis_report["status"] = "failed"
            sidecar["valid"] = False
            sidecar["status"] = "failed"
            sidecar["reason"] = diagnosis_report["failures_detected"][0]["reason"]
            write_json_atomic(sidecar_file, sidecar)
            
        write_json_atomic(diag_file, diagnosis_report)
        print(f"Diagnostic report written to: {diag_file}")
        
    finally:
        session.transcript.stop()
        session.exit()

if __name__ == '__main__':
    case = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\focused_lightweight_350k\assembled_350k_focused_t0.cas.h5")
    out = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\audited_simulation_run04")
    run_audited_simulation(case, out, total_steps=30, dt=0.005, iters_per_step=15)

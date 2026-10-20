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
    read_motion_config, joint_angles, joint_axes, case_wall_adjacency
)
from fluent_checks import wall_vertices, wall_geometry_check, configure_volume_check, check_volume

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

def run_preview(total_time_s=16.0, time_step_s=0.05, checkpoint_interval_s=2.0, out_dir=None, start_case=None):
    if out_dir is None:
        out_dir = Path(r'D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\mesh_motion_preview03')
    out_dir.mkdir(parents=True, exist_ok=True)
    
    log_file = out_dir / 'mesh_motion_preview.jsonl'
    summary_file = out_dir / 'mesh_motion_preview_summary.json'
    trn_file = out_dir / 'mesh_motion_preview.trn'
    
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
    
    summary = {
        "status": "started",
        "total_time_s": total_time_s,
        "time_step_s": time_step_s,
        "checkpoints": [],
        "min_volume_overall_m3": float('inf'),
        "max_rigid_displacement_error_m": 0.0,
        "all_positive_volumes": True,
        "completed_steps": 0
    }
    
    try:
        session.transcript.start(str(trn_file))
        cas_path = Path(cas)
        dat_path = cas_path.with_name(cas_path.name.replace('.cas.h5', '.dat.h5'))
        session.tui.file.read_case(str(cas_path))
        has_data = False
        if dat_path.is_file():
            print(f"Reading data: {dat_path}")
            session.tui.file.read_data(str(dat_path))
            has_data = True
        
        start_time = float(session.rp_vars('flow-time'))
        total_steps = int(round((total_time_s - start_time) / time_step_s))
        print(f"Starting pure mesh motion preview: {start_time:.2f}s -> {total_time_s:.2f}s ({total_steps} steps, dt={time_step_s}s)")
        summary["start_time_s"] = start_time
        summary["total_steps"] = total_steps
        
        session.settings.setup.general.solver.time = 'unsteady-1st-order'
        
        # Load UDF and bind rigid bodies
        udf_lib = r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test'
        session.tui.define.user_defined.compiled_functions('load', udf_lib)
        
        session.tui.define.dynamic_mesh.dynamic_mesh('yes')
        session.tui.define.dynamic_mesh.controls.smoothing('no')
        session.tui.define.dynamic_mesh.controls.remeshing('no')
        session.tui.define.dynamic_mesh.controls.layering('no')
        
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1 rigid-body joint1_motion::libudf_arm_test no -0.745634 1.155872 -1.142240 0 0 0")')
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1 rigid-body joint2_motion::libudf_arm_test no -0.728944 1.206372 -1.172240 0 0 0")')
        session.scheme_eval.scheme_eval('(ti-menu-load-string "define/dynamic-mesh/zones/create fluid.1.1.1 rigid-body joint3_motion::libudf_arm_test no -0.730834 1.008372 -1.228240 0 0 0")')
        
        tc = session.settings.solution.run_calculation.transient_controls
        tc.time_step_size = time_step_s
        
        if not has_data:
            init = session.settings.solution.initialization
            init.initialization_type = 'standard'
            init.standard_initialize()
            if abs(start_time) > 1e-6:
                session.scheme_eval.scheme_eval(f'(rpsetvar \'flow-time {start_time})')
            print(f"Initialized time level variables (flow-time={start_time:.2f}s).")
        else:
            print(f"Loaded existing time level data (flow-time={start_time:.2f}s).")
        
        # Configure volume check report
        vol_report = configure_volume_check(session)

        # Baseline vertices at t=0
        print("Extracting baseline vertices at t=0...")
        ref_vertices = wall_vertices(session)
        
        next_checkpoint_s = checkpoint_interval_s
        records = []
        
        with open(log_file, 'w', encoding='utf-8') as f_log:
            for step in range(1, total_steps + 1):
                t_step_start = time.time()
                session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=0)
                flow_time = float(session.rp_vars('flow-time'))
                step_elapsed = time.time() - t_step_start
                
                # Check mesh volume via official report
                min_vol = check_volume(session, vol_report)
                if min_vol <= 0.0 or not math.isfinite(min_vol):
                    summary["all_positive_volumes"] = False
                    raise RuntimeError(f"NEGATIVE_CELL_VOLUME_DETECTED: step {step}, flow_time={flow_time:.4f}s, min_vol={min_vol}")
                    
                summary["min_volume_overall_m3"] = min(summary["min_volume_overall_m3"], min_vol)
                
                # Verify rigid body geometry every 10 steps or at checkpoints
                is_checkpoint = (flow_time >= next_checkpoint_s - 1e-6) or (step == total_steps)
                check_geom = is_checkpoint or (step % 10 == 0)
                
                geom_err = 0.0
                if check_geom:
                    curr_vertices = wall_vertices(session)
                    geom_stats = wall_geometry_check(ref_vertices, curr_vertices, tolerance=1e-5)
                    # Check max radius error across all walls
                    for wname, stats in geom_stats.items():
                        err = stats.get('max_radius_error_m', 0.0)
                        geom_err = max(geom_err, err)
                    summary["max_rigid_displacement_error_m"] = max(summary["max_rigid_displacement_error_m"], geom_err)
                
                rec = {
                    "step": step,
                    "flow_time_s": flow_time,
                    "min_volume_m3": min_vol,
                    "max_rigid_error_m": geom_err,
                    "step_elapsed_s": step_elapsed
                }
                f_log.write(json.dumps(rec) + '\n')
                f_log.flush()
                
                if step % 10 == 0 or is_checkpoint:
                    print(f"Step {step:4d}/{total_steps} (t={flow_time:6.3f}s): min_vol={min_vol:.4e} m3, rigid_err={geom_err:.2e} m, dt_wall={step_elapsed:.2f}s")
                
                if is_checkpoint:
                    cp_path = out_dir / f"checkpoint_t{flow_time:06.2f}s.cas.h5"
                    print(f"Saving checkpoint at t={flow_time:.2f}s -> {cp_path.name}...")
                    session.tui.file.write_case(str(cp_path))
                    summary["checkpoints"].append({
                        "flow_time_s": flow_time,
                        "step": step,
                        "case_file": str(cp_path)
                    })
                    next_checkpoint_s += checkpoint_interval_s
                
                summary["completed_steps"] = step
                
        summary["status"] = "completed"
        print(f"\n>>> PURE MESH MOTION PREVIEW COMPLETED SUCCESSFULLY! <<<")
        print(f"Total steps: {total_steps}, min volume: {summary['min_volume_overall_m3']:.4e} m3, max rigid error: {summary['max_rigid_displacement_error_m']:.2e} m")
    finally:
        session.transcript.stop()
        session.exit()
        with open(summary_file, 'w', encoding='utf-8') as f_sum:
            json.dump(summary, f_sum, indent=2)
        print(f"Summary written to: {summary_file}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--total-time', type=float, default=1.0, help='Total motion preview time in seconds')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step size in seconds')
    parser.add_argument('--checkpoint-interval', type=float, default=0.5, help='Checkpoint interval in seconds')
    parser.add_argument('--out-dir', type=Path, default=None, help='Output directory for checkpoints and logs')
    parser.add_argument('--start-case', type=Path, default=None, help='Starting case file for restart')
    args = parser.parse_args()
    
    run_preview(total_time_s=args.total_time, time_step_s=args.dt, checkpoint_interval_s=args.checkpoint_interval, out_dir=args.out_dir, start_case=args.start_case)

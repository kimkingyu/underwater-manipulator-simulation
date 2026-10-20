import os
import sys
import time
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
from fluent_checks import check_volume, configure_volume_check

import logging
logger = logging.getLogger('matched_820k')
logging.basicConfig(level=logging.INFO)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

WORK_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\matched_820k_system")
WORK_DIR.mkdir(parents=True, exist_ok=True)
cas_path = WORK_DIR / "assembled_matched_820k_t0.cas.h5"

SRC_350K = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\focused_lightweight_350k")

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

trn_file = WORK_DIR / "matched_assembly_and_run.trn"
try:
    session.transcript.start(str(trn_file))
    
    # 1. Read 12mm background and assemble
    bg_cas = r"D:\work4\cfd\test_orphan_reduction\bg_12mm.cas.h5"
    print("Reading background...")
    session.tui.file.read_case(bg_cas)
    print("Appending J1, J2, J3...")
    session.tui.mesh.modify_zones.append_mesh(str(SRC_350K / "j1.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(SRC_350K / "j2.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(SRC_350K / "j3.cas.h5"))
    
    session.tui.define.boundary_conditions.zone_type('output', 'pressure-outlet')
    session.tui.define.boundary_conditions.zone_type('overset_j1', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j2', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j3', 'overset')
    
    threads = session.scheme_eval.scheme_eval('(map (lambda (th) (list (thread-id th) (symbol->string (thread-name th)))) (get-cell-threads))')
    bg_id = [t[0] for t in threads if t[1] == 'fluid'][0]
    j1_id = [t[0] for t in threads if t[1] == 'fluid.1'][0]
    j2_id = [t[0] for t in threads if t[1] == 'fluid.1.1'][0]
    j3_id = [t[0] for t in threads if t[1] == 'fluid.1.1.1'][0]
    
    session.scheme_eval.scheme_eval(f'(ti-menu-load-string "define/overset-interfaces/create arm_interface {bg_id} () {j1_id} {j2_id} {j3_id} ()")')
    session.scheme_eval.scheme_eval('(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")')
    session.tui.mesh.check()
    
    session.tui.file.write_case(str(cas_path))
    print(f"Saved assembled matched case: {cas_path}")
    
    # 2. Physics & Controls
    session.settings.setup.general.solver.time = 'unsteady-1st-order'
    session.tui.define.materials.copy('fluid', 'water-liquid')
    session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
    session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
    for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
        session.settings.setup.cell_zone_conditions.fluid[cz].general.material = 'water-liquid'
        
    session.settings.solution.methods.p_v_coupling.flow_scheme = 'Coupled'
    session.settings.solution.controls.p_v_controls.flow_courant_number = 40.0
    session.settings.solution.controls.p_v_controls.explicit_momentum_under_relaxation = 0.60
    session.settings.solution.controls.p_v_controls.explicit_pressure_under_relaxation = 0.60
    session.settings.solution.controls.under_relaxation['k'] = 0.65
    session.settings.solution.controls.under_relaxation['omega'] = 0.65
    session.tui.define.models.viscous.turbulence_expert.production_limiter('yes')
    session.tui.define.models.viscous.turbulence_expert.kato_launder_model('yes')
    session.settings.solution.controls.limits.max_turb_visc_ratio = 2000.0
    
    session.settings.setup.boundary_conditions.velocity_inlet['input'].momentum.velocity.value = 0.25
    session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_intensity = 0.05
    session.settings.setup.boundary_conditions.velocity_inlet['input'].turbulence.turbulent_viscosity_ratio = 10.0
    session.settings.setup.boundary_conditions.pressure_outlet['output'].momentum.prevent_reverse_flow = False
    
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
    
    total_steps = 20
    print(f"\n==================================================================")
    print(f"  Starting matched 820k transient run: {total_steps} steps (dt={dt}s)...")
    print(f"==================================================================")
    
    for step in range(1, total_steps + 1):
        t0 = time.time()
        session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=15)
        elapsed = time.time() - t0
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
        
        print(f"Step {step:2d}/{total_steps} (t={flow_time:5.2f}s, {elapsed:4.1f}s): "
              f"tau_j1={row['j1_torque_nm']:8.4f}, tau_j2={row['j2_torque_nm']:8.4f}, tau_j3={row['j3_torque_nm']:8.4f} N.m | min_V={min_vol:.3e}", flush=True)
              
    print(f"\nSUCCESS! All {total_steps} steps completed on matched 820k system!")
finally:
    session.transcript.stop()
    session.exit()

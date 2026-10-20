import os
import sys
from pathlib import Path

os.environ["AWP_ROOT242"] = r"D:\Program Files\ANSYS Inc\v242"
cas_file = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\focused_lightweight_350k\assembled_350k_focused_t0.cas.h5")
out_dir = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\test_locate_turb_cells")
out_dir.mkdir(parents=True, exist_ok=True)

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

session = pyfluent.launch_fluent(
    product_version='24.2',
    mode='solver',
    precision='double',
    processor_count=8,
    additional_arguments='-t8 -mpi=msmpi',
    ui_mode=UIMode.NO_GUI_OR_GRAPHICS,
    start_timeout=180,
    cleanup_on_exit=True,
    start_transcript=True,
)

trn_file = out_dir / "locate.trn"
try:
    session.transcript.start(str(trn_file))
    session.tui.file.read_case(str(cas_file))
    session.settings.setup.general.solver.time = 'unsteady-1st-order'
    
    session.tui.define.materials.copy('fluid', 'water-liquid')
    session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
    session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
    for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
        session.settings.setup.cell_zone_conditions.fluid[cz].general.material = 'water-liquid'
        
    session.settings.solution.methods.p_v_coupling.flow_scheme = 'Coupled'
    session.settings.solution.controls.p_v_controls.flow_courant_number = 35.0
    session.settings.solution.controls.p_v_controls.explicit_momentum_under_relaxation = 0.65
    session.settings.solution.controls.p_v_controls.explicit_pressure_under_relaxation = 0.65
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
    
    session.settings.solution.run_calculation.transient_controls.time_step_size = 0.01
    
    init = session.settings.solution.initialization
    init.initialization_type = 'standard'
    init.defaults['k'] = 0.00023438
    init.defaults['omega'] = 22.244
    init.defaults['x-velocity'] = 0.25
    init.defaults['y-velocity'] = 0.0
    init.defaults['z-velocity'] = 0.0
    init.defaults['pressure'] = 0.0
    init.standard_initialize()
    
    print("Advancing 8 steps cleanly...")
    session.settings.solution.run_calculation.dual_time_iterate(time_step_count=8, max_iter_per_step=15)
    
    print("Advancing step 9...")
    session.settings.solution.run_calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=15)
    
    print("\n--- Probing fields at Step 9 ---")
    for cz in ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']:
        print(f"\nChecking max turbulent viscosity ratio in {cz}:")
        session.scheme_eval.scheme_eval(f'(ti-menu-load-string "report/volume-integrals/maximum-cell-value {cz} () cell-turb-viscosity-ratio ()")')
        session.scheme_eval.scheme_eval(f'(ti-menu-load-string "report/volume-integrals/maximum-cell-value {cz} () cell-velocity-magnitude ()")')
        session.scheme_eval.scheme_eval(f'(ti-menu-load-string "report/volume-integrals/maximum-cell-value {cz} () pressure ()")')
        session.scheme_eval.scheme_eval(f'(ti-menu-load-string "report/volume-integrals/minimum-cell-value {cz} () pressure ()")')
        
finally:
    session.transcript.stop()
    session.exit()

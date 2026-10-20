"""Create a seawater base setup from the existing mesh. Never iterate or initialize."""
import os
from pathlib import Path
import json
import hashlib

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'deliverables/basic_models_build/Fluent'
CHECK = ROOT / 'deliverables/basic_models_build/verification'
AWP = Path(r'D:\Program Files\ANSYS Inc\v242')
os.environ['AWP_ROOT242'] = str(AWP)
os.environ['ANSYSWB_SYSDIR'] = 'winx64'
os.environ['PROCESSOR_ARCHITECTURE'] = 'AMD64'
paths = ['aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
         'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
         'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm']
os.environ['PATH'] = ';'.join([str(AWP / p) for p in paths] + [os.environ.get('PATH', '')])
import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

source = ROOT / 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/reduced_412k_system/assembled_reduced_415k_t0.cas.h5'
target = OUT / 'UnderwaterArm_Base_415k.cas.h5'
if target.exists():
    raise FileExistsError(target)
source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
session = pyfluent.launch_fluent(product_version='24.2', mode='solver', precision='double',
                                 processor_count=2, additional_arguments='-t2 -mpi=msmpi',
                                 ui_mode=UIMode.NO_GUI_OR_GRAPHICS, start_timeout=180,
                                 cleanup_on_exit=True, start_transcript=True, cwd=str(OUT))
try:
    session.tui.file.read_case(str(source))
    old_time = float(session.rp_vars('flow-time'))
    session.settings.setup.general.solver.time = 'unsteady-1st-order'
    session.tui.define.materials.copy('fluid', 'water-liquid')
    session.settings.setup.materials.fluid['water-liquid'].density.value = 1025.0
    session.settings.setup.materials.fluid['water-liquid'].viscosity.value = 0.00108
    zones = ['fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1']
    for name in zones:
        session.settings.setup.cell_zone_conditions.fluid[name].general.material = 'water-liquid'
    inlet = session.settings.setup.boundary_conditions.velocity_inlet['input']
    inlet.momentum.velocity_specification_method = 'Components'
    inlet.momentum.velocity_components = [{'value': 0.25}, {'value': 0.0}, {'value': 0.0}]
    inlet.turbulence.turbulent_intensity = 0.05
    inlet.turbulence.turbulent_viscosity_ratio = 10.0
    session.settings.solution.run_calculation.transient_controls.time_step_size = 0.01
    # A portable fixed-pose base has no compiled library path to the original PC.
    # Dynamic motion sources are delivered separately, not silently activated.
    session.tui.define.dynamic_mesh.dynamic_mesh('no')
    report = {
        'source': source.relative_to(ROOT).as_posix(), 'source_sha256': source_hash,
        'purpose': 'Portable seawater fixed-pose base setup; existing overset mesh preserved.',
        'fluent_version': str(session.get_fluent_version()),
        'iterations_executed': 0, 'initialization_executed': False,
        'flow_time_before': old_time, 'flow_time_after': float(session.rp_vars('flow-time')),
        'fluid_materials': {name: session.settings.setup.cell_zone_conditions.fluid[name].general.material() for name in zones},
        'density_kg_m3': session.settings.setup.materials.fluid['water-liquid'].density.value(),
        'viscosity_Pa_s': session.settings.setup.materials.fluid['water-liquid'].viscosity.value(),
        'inlet_velocity_components': inlet.momentum.velocity_components(),
        'time_step_s': session.settings.solution.run_calculation.transient_controls.time_step_size(),
        'gravity_enabled': bool(session.rp_vars('gravity?')),
        'dynamic_mesh_enabled': False, 'motion_udf_loaded': False,
        'known_limitations': 'Inherited poor-quality cells/overset orphans remain. Not a converged result or a validated dynamic setup.',
    }
    assert report['flow_time_before'] == report['flow_time_after'] == 0.0
    assert report['density_kg_m3'] == 1025.0
    assert all(v == 'water-liquid' for v in report['fluid_materials'].values())
    session.tui.file.write_case(str(target))
    report['output_sha256'] = hashlib.sha256(target.read_bytes()).hexdigest()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
    (CHECK / 'fluent_base_setup_validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
finally:
    session.exit()

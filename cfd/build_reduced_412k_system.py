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

OUT_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\reduced_412k_system")
OUT_DIR.mkdir(parents=True, exist_ok=True)

SRC_350K = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\focused_lightweight_350k")
bg_neu = r"D:\work4\cfd\test_bg_200k.neu"
bg_cas = OUT_DIR / "bg_212k.cas.h5"

j3_neu = r"D:\work4\cfd\test_j3_defeatured.neu"
j3_cas = OUT_DIR / "j3_defeatured.cas.h5"
assembled_cas = OUT_DIR / "assembled_reduced_412k_t0.cas.h5"

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

trn_file = OUT_DIR / "assembly_412k.trn"
try:
    session.transcript.start(str(trn_file))
    
    # 1. Convert background
    print("Converting reduced background (212k)...")
    session.tui.file.import_.gambit(bg_neu)
    session.tui.mesh.check()
    session.tui.mesh.repair_improve.improve_quality()
    session.tui.file.write_case(str(bg_cas))
    
    # 2. Convert defeatured J3
    print("Converting defeatured J3 (131k)...")
    session.tui.file.import_.gambit(j3_neu)
    session.tui.mesh.check()
    session.tui.mesh.repair_improve.improve_quality()
    session.tui.file.write_case(str(j3_cas))
    
    # 3. Assemble
    print("Assembling 4 domains...")
    session.tui.file.read_case(str(bg_cas))
    session.tui.mesh.modify_zones.append_mesh(str(SRC_350K / "j1.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(SRC_350K / "j2.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(j3_cas))
    
    session.tui.define.boundary_conditions.zone_type('output', 'pressure-outlet')
    session.tui.define.boundary_conditions.zone_type('overset_j1', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j2', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j3', 'overset')
    
    threads = session.scheme_eval.scheme_eval('(map (lambda (th) (list (thread-id th) (symbol->string (thread-name th)))) (get-cell-threads))')
    bg_id = [t[0] for t in threads if t[1] == 'fluid'][0]
    j1_id = [t[0] for t in threads if t[1] == 'fluid.1'][0]
    j2_id = [t[0] for t in threads if t[1] == 'fluid.1.1'][0]
    j3_id = [t[0] for t in threads if t[1] == 'fluid.1.1.1'][0]
    
    print(f"Creating overset interface: bg={bg_id}, j1={j1_id}, j2={j2_id}, j3={j3_id}...")
    session.scheme_eval.scheme_eval(f'(ti-menu-load-string "define/overset-interfaces/create arm_interface {bg_id} () {j1_id} {j2_id} {j3_id} ()")')
    session.scheme_eval.scheme_eval('(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")')
    
    session.tui.mesh.check()
    session.tui.mesh.quality()
    session.tui.file.write_case(str(assembled_cas))
    print(f"\nSuccessfully assembled and written 412k case: {assembled_cas}")
finally:
    session.transcript.stop()
    session.exit()

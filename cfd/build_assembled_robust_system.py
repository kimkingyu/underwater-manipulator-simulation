import os
import sys
import json
import time
import shutil
from pathlib import Path

awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
directories = [
    'aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
    'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
    'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm'
]
os.environ['PATH'] = ';'.join([f'{awp}/{p}' for p in directories] + [os.environ.get('PATH', '')])

import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

OUT_DIR = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\robust_lightweight_system01")
OUT_DIR.mkdir(parents=True, exist_ok=True)

SRC_350K = Path(r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\focused_lightweight_350k")

# Copy background, j1, j2
shutil.copy(SRC_350K / "background.neu", OUT_DIR / "background.neu")
shutil.copy(SRC_350K / "j1.neu", OUT_DIR / "j1.neu")
shutil.copy(SRC_350K / "j2.neu", OUT_DIR / "j2.neu")
# Copy new superclean j3
shutil.copy(r"D:\work4\cfd\test_j3_superclean.neu", OUT_DIR / "j3.neu")

print("Files staged. Starting Fluent conversion & assembly...")

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

trn_file = OUT_DIR / "assembly_robust.trn"
try:
    session.transcript.start(str(trn_file))
    
    domains = [
        ("background", OUT_DIR / "background.neu", OUT_DIR / "background.cas.h5"),
        ("j1", OUT_DIR / "j1.neu", OUT_DIR / "j1.cas.h5"),
        ("j2", OUT_DIR / "j2.neu", OUT_DIR / "j2.cas.h5"),
        ("j3", OUT_DIR / "j3.neu", OUT_DIR / "j3.cas.h5")
    ]
    
    for name, neu_p, cas_p in domains:
        print(f"Converting {name}...")
        session.tui.file.import_.gambit(str(neu_p))
        session.tui.mesh.check()
        session.tui.mesh.repair_improve.improve_quality()
        session.tui.file.write_case(str(cas_p))
        
    print("\nAssembling domains...")
    session.tui.file.read_case(str(OUT_DIR / "background.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j1.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j2.cas.h5"))
    session.tui.mesh.modify_zones.append_mesh(str(OUT_DIR / "j3.cas.h5"))
    
    session.tui.define.boundary_conditions.zone_type('output', 'pressure-outlet')
    session.tui.define.boundary_conditions.zone_type('overset_j1', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j2', 'overset')
    session.tui.define.boundary_conditions.zone_type('overset_j3', 'overset')
    
    threads = session.scheme_eval.scheme_eval('(map (lambda (th) (list (thread-id th) (symbol->string (thread-name th)))) (get-cell-threads))')
    bg_id = [t[0] for t in threads if t[1] == 'fluid'][0]
    j1_id = [t[0] for t in threads if t[1] == 'fluid.1'][0]
    j2_id = [t[0] for t in threads if t[1] == 'fluid.1.1'][0]
    j3_id = [t[0] for t in threads if t[1] == 'fluid.1.1.1'][0]
    
    print(f"Creating overset interface with bg={bg_id}, j1={j1_id}, j2={j2_id}, j3={j3_id}...")
    cmd_intf = f'(ti-menu-load-string "define/overset-interfaces/create arm_interface {bg_id} () {j1_id} {j2_id} {j3_id} ()")'
    session.scheme_eval.scheme_eval(cmd_intf)
    
    cmd_prio = '(ti-menu-load-string "define/overset-interfaces/grid-priorities arm_interface 0 3 2 1")'
    session.scheme_eval.scheme_eval(cmd_prio)
    
    session.tui.define.overset_interfaces.check()
    session.tui.mesh.check()
    session.tui.mesh.quality()
    
    out_case = OUT_DIR / "assembled_robust_t0.cas.h5"
    session.tui.file.write_case(str(out_case))
    print(f"\nSuccessfully written assembled robust case: {out_case}")
finally:
    session.transcript.stop()
    session.exit()

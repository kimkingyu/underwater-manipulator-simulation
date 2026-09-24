import os
import sys
import shutil
import subprocess
from pathlib import Path

awp = r'D:\Program Files\ANSYS Inc\v242'
fluent_inc = r'D:\Program Files\ANSYS Inc\v242\fluent'

src_header = Path(r'D:\work4\src\cfd\arm_motion_config.h')
if not src_header.is_file():
    raise FileNotFoundError(f"Source header not found: {src_header}")

targets = [
    Path(r'D:\work4\cfd\wetted_arm_rebuild\arm_motion_config.h'),
    Path(r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test\src\arm_motion_config.h'),
    Path(r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test\win64\3ddp_node\arm_motion_config.h'),
    Path(r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test\win64\3ddp_host\arm_motion_config.h'),
    Path(r'D:\work4\libudf_arm_test\src\arm_motion_config.h'),
    Path(r'D:\work4\libudf_arm_test\win64\3ddp_node\arm_motion_config.h'),
    Path(r'D:\work4\libudf_arm_test\win64\3ddp_host\arm_motion_config.h'),
    Path(r'D:\work4\arm_motion_config.h'),
    Path(r'D:\work4\cfd\1_files\dp0\FFF\Fluent\arm_motion_config.h'),
    Path(r'D:\work4\cfd\1_files\dp0\FFF\Fluent\libudf_arm_test\src\arm_motion_config.h'),
    Path(r'D:\work4\cfd\1_files\dp0\FFF\Fluent\libudf_arm_test\win64\3ddp_node\arm_motion_config.h'),
    Path(r'D:\work4\cfd\1_files\dp0\FFF\Fluent\libudf_arm_test\win64\3ddp_host\arm_motion_config.h'),
]

print(f"Propagating {src_header} to {len(targets)} locations...")
for t in targets:
    t.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_header, t)
    print(f"  Copied to: {t}")

env = os.environ.copy()
env['FLUENT_INC'] = fluent_inc
env['AWP_ROOT242'] = awp
env['ANSYSWB_SYSDIR'] = 'winx64'
env['PROCESSOR_ARCHITECTURE'] = 'AMD64'
py_dir = f"{awp}/commonfiles/CPython/3_10/winx64/Release/python"
env['PYTHONHOME'] = py_dir
env['PYTHONPATH'] = py_dir
env['PATH'] = f"{py_dir};{py_dir}/Scripts;{fluent_inc}/fluent24.2.0/bin/winx64;{env.get('PATH', '')}"

udf_dirs = [
    Path(r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test\win64\3ddp_host'),
    Path(r'D:\work4\cfd\wetted_arm_rebuild\libudf_arm_test\win64\3ddp_node'),
    Path(r'D:\work4\libudf_arm_test\win64\3ddp_host'),
    Path(r'D:\work4\libudf_arm_test\win64\3ddp_node'),
]

for d in udf_dirs:
    if not d.is_dir():
        continue
    print(f"\n--- Compiling UDF in {d} ---")
    scons_bat = d / 'scons.bat'
    if not scons_bat.is_file():
        print(f"  Warning: scons.bat not found in {d}")
        continue
    
    # Run scons
    cmd = [str(scons_bat)]
    p = subprocess.run(cmd, cwd=str(d), env=env, capture_output=True, text=True)
    print("STDOUT:", p.stdout.strip())
    if p.stderr.strip():
        print("STDERR:", p.stderr.strip())
    if p.returncode != 0:
        print(f"Compilation failed in {d} with code {p.returncode}")
        sys.exit(p.returncode)
    else:
        dll = d / 'libudf_arm_test.dll'
        if dll.is_file():
            print(f"  SUCCESS! Generated {dll.name}, size: {dll.stat().st_size} bytes, mtime: {dll.stat().st_mtime}")
        else:
            print(f"  Warning: {dll} not found after build")

print("\n>>> ALL UDF LIBRARIES COMPILED SUCCESSFULLY! <<<")

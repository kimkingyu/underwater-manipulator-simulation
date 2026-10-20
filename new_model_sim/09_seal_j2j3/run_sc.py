"""Run one SpaceClaim script headless. Usage: python run_sc.py SCRIPT.py [timeout_s]
The script itself guards its outputs; this wrapper only launches and reports the exit code."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, r"D:\work4\new_model_sim\04_workbench")
from run_wb_fullenv import full_environment  # noqa: E402

SPACECLAIM = r"D:\Program Files\ANSYS Inc\v242\scdm\SpaceClaim.exe"
script = Path(sys.argv[1]).resolve()
timeout = int(sys.argv[2]) if len(sys.argv) > 2 else 3600
merged = {}
for k, v in list(os.environ.items()) + list(full_environment().items()):
    merged[k.upper()] = (k, v)
env = dict(merged.values())
t0 = time.time()
proc = subprocess.Popen([SPACECLAIM, "/Headless=True", "/Splash=False", "/Welcome=False",
                         "/ExitAfterScript=True", "/RunScript=" + str(script)],
                        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    code = proc.wait(timeout=timeout)
except subprocess.TimeoutExpired:
    proc.kill()
    proc.wait()
    code = "timeout"
print(json.dumps({"script": str(script), "exit_code": code, "seconds": round(time.time() - t0, 1)}))

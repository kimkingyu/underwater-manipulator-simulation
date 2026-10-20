"""Run a Workbench journal with the full Windows environment rebuilt from the registry.

The agent's shell starts with a stripped environment (no APPDATA, LOCALAPPDATA, ProgramData,
ProgramFiles, ComSpec, NUMBER_OF_PROCESSORS, ANSYSLMD_LICENSE_FILE ...). AnsysWBU.exe then
crashes in Ans.CadInt.AssemblyContainer242.dll when the Meshing editor starts. This launcher
rebuilds the machine + user environment exactly as Explorer would, for the child process only.
Nothing in the registry or the system settings is changed.

Usage: python run_wb_fullenv.py JOURNAL.wbjn TRACE.txt [timeout_s]
"""
import os
import subprocess
import sys
import time
import winreg
from pathlib import Path

RUNWB2 = r"D:\Program Files\ANSYS Inc\v242\Framework\bin\Win64\RunWB2.exe"


def read_env(root, key):
    out = {}
    with winreg.OpenKey(root, key) as k:
        i = 0
        while True:
            try:
                name, value, kind = winreg.EnumValue(k, i)
            except OSError:
                break
            out[name] = (value, kind)
            i += 1
    return out


def expand(value, env):
    # Expand %VAR% against the environment being built (case-insensitive like Windows).
    lower = {k.lower(): v for k, v in env.items()}
    result, i = [], 0
    while i < len(value):
        if value[i] == '%':
            j = value.find('%', i + 1)
            if j > i + 1:
                name = value[i + 1:j]
                if name.lower() in lower:
                    result.append(lower[name.lower()])
                    i = j + 1
                    continue
        result.append(value[i])
        i += 1
    return ''.join(result)


def full_environment():
    env = {}
    profile = os.environ.get('USERPROFILE', r'C:\Users\admin')
    sysroot = os.environ.get('SystemRoot', r'C:\WINDOWS')
    # Values Windows itself provides to every logon session.
    env.update({
        'SystemRoot': sysroot, 'windir': sysroot, 'SystemDrive': sysroot[:2],
        'USERPROFILE': profile, 'HOMEDRIVE': profile[:2], 'HOMEPATH': profile[2:],
        'USERNAME': os.environ.get('USERNAME', 'admin'), 'USERDOMAIN': os.environ.get('USERDOMAIN', ''),
        'COMPUTERNAME': os.environ.get('COMPUTERNAME', os.environ.get('USERDOMAIN', '')),
        'APPDATA': os.path.join(profile, 'AppData', 'Roaming'),
        'LOCALAPPDATA': os.path.join(profile, 'AppData', 'Local'),
        'ALLUSERSPROFILE': r'C:\ProgramData', 'ProgramData': r'C:\ProgramData', 'PUBLIC': r'C:\Users\Public',
        'ProgramFiles': r'C:\Program Files', 'ProgramFiles(x86)': r'C:\Program Files (x86)',
        'ProgramW6432': r'C:\Program Files', 'CommonProgramFiles': r'C:\Program Files\Common Files',
        'CommonProgramFiles(x86)': r'C:\Program Files (x86)\Common Files',
        'CommonProgramW6432': r'C:\Program Files\Common Files',
    })
    machine = read_env(winreg.HKEY_LOCAL_MACHINE,
                       r'SYSTEM\CurrentControlSet\Control\Session Manager\Environment')
    user = read_env(winreg.HKEY_CURRENT_USER, 'Environment')
    for name, (value, kind) in machine.items():
        env[name] = expand(value, env) if kind == winreg.REG_EXPAND_SZ else value
    machine_path = env.get('Path', '')
    for name, (value, kind) in user.items():
        v = expand(value, env) if kind == winreg.REG_EXPAND_SZ else value
        if name.lower() == 'path':
            env['Path'] = machine_path + (';' if machine_path else '') + v
        else:
            env[name] = v
    return env


def main():
    journal, trace = Path(sys.argv[1]), Path(sys.argv[2])
    timeout = int(sys.argv[3]) if len(sys.argv) > 3 else 3000
    if trace.exists():
        raise SystemExit('refusing to overwrite ' + str(trace))
    env = full_environment()
    keys = ('APPDATA', 'LOCALAPPDATA', 'ProgramData', 'ProgramFiles', 'ComSpec',
            'NUMBER_OF_PROCESSORS', 'ANSYSLMD_LICENSE_FILE', 'AWP_ROOT242', 'TEMP', 'TMP')
    for k in keys:
        print('%-22s %s' % (k, env.get(k, '<MISSING>')))
    t0 = time.time()
    p = subprocess.Popen([RUNWB2, '-I', '-X', '-R', str(journal)], cwd=str(journal.parent), env=env)
    try:
        code = p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        code = 'timeout (left running, pid %d)' % p.pid
    print('exit', code, 'seconds', round(time.time() - t0, 1))
    print(trace.read_text(encoding='utf-8-sig')[:8000] if trace.exists() else 'NO TRACE')


if __name__ == '__main__':
    main()

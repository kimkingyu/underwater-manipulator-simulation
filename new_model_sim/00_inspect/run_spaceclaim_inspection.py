"""Inspect CAD files with SpaceClaim, one fresh headless session per file.

Usage:
  python run_spaceclaim_inspection.py --out OUT_DIR FILE_OR_FOLDER [...] [--no-save]

Inputs are only read (SHA-256 checked before/after). Results go to a new
timestamped folder inside OUT_DIR, so earlier results are never overwritten.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
AWP = Path(r'D:\Program Files\ANSYS Inc\v242')
SPACECLAIM = AWP / 'scdm' / 'SpaceClaim.exe'
SCRIPT = HERE / 'sc_inspect_one.py'
CAD_SUFFIXES = {'.x_t', '.x_b', '.xmt_txt', '.xmt_bin', '.step', '.stp',
                '.igs', '.iges', '.sldprt', '.sldasm'}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def collect(items):
    files = []
    for item in items:
        path = Path(item)
        if path.is_dir():
            files += sorted(p for p in path.iterdir()
                            if p.is_file() and p.suffix.lower() in CAD_SUFFIXES)
        elif path.is_file():
            files.append(path)
        else:
            raise FileNotFoundError(path)
    if not files:
        raise FileNotFoundError('No CAD files found in: ' + ', '.join(items))
    return files


def inspect(source, run_dir, save, timeout):
    out_json = run_dir / (source.name + '.json')
    env = dict(os.environ, AWP_ROOT242=str(AWP), NM_INPUT=str(source),
               NM_OUTPUT=str(out_json),
               NM_SAVE=str(run_dir / (source.name + '.scdoc')) if save else '')
    before = sha256(source)
    start = time.time()
    proc = subprocess.Popen(
        [str(SPACECLAIM), '/Headless=True', '/Splash=False', '/Welcome=False',
         '/ExitAfterScript=True', '/RunScript=' + str(SCRIPT)],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        exit_code = proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        exit_code = 'timeout'
    if out_json.exists():
        data = json.loads(out_json.read_text(encoding='utf-8'))
    else:
        data = {'input': str(source), 'status': 'no_output'}
    data.update(exit_code=exit_code, elapsed_s=round(time.time() - start, 1),
                source_sha256=before, source_unchanged=sha256(source) == before)
    out_json.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    return data


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--no-save', action='store_true')
    parser.add_argument('--timeout', type=int, default=900)
    parser.add_argument('inputs', nargs='+')
    args = parser.parse_args()
    files = collect(args.inputs)
    run_dir = Path(args.out) / datetime.now().strftime('run_%Y%m%d_%H%M%S')
    run_dir.mkdir(parents=True, exist_ok=False)
    summary = []
    for source in files:
        data = inspect(source, run_dir, not args.no_save, args.timeout)
        row = {key: data.get(key) for key in (
            'input', 'status', 'body_count', 'component_count', 'overall_size_m',
            'total_volume_m3', 'exit_code', 'elapsed_s', 'source_unchanged')}
        row['open_or_nonmanifold_count'] = len(data.get('open_or_nonmanifold') or [])
        row['unchecked_count'] = len(data.get('unchecked_bodies') or [])
        row['warnings'] = data.get('warnings') or []
        summary.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    (run_dir / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print('RESULTS:', run_dir)
    if not all(r['source_unchanged'] for r in summary):
        raise SystemExit('A source file changed during inspection')


if __name__ == '__main__':
    main()

"""Archive existing simulation artifacts without running or modifying simulations."""
from pathlib import Path
from datetime import datetime
import os
import sys
import json
import hashlib
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'deliverables'
TOP = '水下机械臂仿真工程'
DIRECTORIES = ('cfd', 'data', 'docs', 'experiments', 'model', 'scripts',
               'src', 'urdf', '机器人', 'libudf_arm_test')
ROOT_FILES = ('main.m', 'run_full_project_pipeline.m', 'run_hydro_surrogate.m',
              'arm_motion.c', 'arm_motion_config.h', 'test_motion.c', 'README.md',
              'LICENSE', 'model.zip', 'local_mtihp7iv_7p8p9q_urdf_stl.zip',
              'assemble_cmd.txt', 'exact_physics_full_cmd.txt', 'make_valid_cmd.txt',
              '水下机械臂_52组CFD与动力学及GPR代理模型对比与偏差分析表.xlsx')
SKIP_DIRS = {'.git', '.narrafork', '__pycache__', '.pytest_cache', '.mypy_cache',
             'node_modules', '.venv', 'venv', '.mcp-venv', '.mcp-venv312', '.ipynb_checkpoints'}
SKIP_EXT = {'.pyc', '.pyo', '.lock', '.lck', '.tmp', '.pem', '.key', '.pfx', '.p12'}
CHUNK = 4 * 1024 * 1024


def excluded_reason(path):
    rel = path.relative_to(ROOT)
    name = path.name.lower()
    if path.is_symlink() or path.is_junction():
        return 'symbolic link or junction; not followed'
    if any(part.lower() in SKIP_DIRS for part in rel.parts):
        return 'cache, interpreter environment, or workspace metadata'
    if rel.as_posix() == 'docs/papers':
        return 'reference literature, not simulation engineering artifacts'
    if name.startswith(('~$', '.env', 'serverinfo', 'server_info', 'server-info', 'sifile')):
        return 'temporary lock, environment, or solver connection file'
    if name in {'credentials.json', 'secrets.json', 'id_rsa', 'id_ed25519'}:
        return 'credential filename'
    if path.suffix.lower() in SKIP_EXT:
        return 'cache, temporary lock, or private-key file type'
    return None


def inventory():
    selected = []
    excluded = []
    for name in DIRECTORIES:
        parent = ROOT / name
        if not parent.is_dir():
            raise FileNotFoundError(parent)
        for base, dirs, files in os.walk(parent, followlinks=False):
            for d in list(dirs):
                p = Path(base) / d
                reason = excluded_reason(p)
                if reason:
                    dirs.remove(d)
                    excluded.append({'path': p.relative_to(ROOT).as_posix() + '/', 'reason': reason})
            for name in files:
                p = Path(base) / name
                reason = excluded_reason(p)
                if reason:
                    excluded.append({'path': p.relative_to(ROOT).as_posix(), 'reason': reason})
                else:
                    selected.append(p)
    for name in ROOT_FILES:
        p = ROOT / name
        if not p.is_file():
            raise FileNotFoundError(p)
        selected.append(p)
    selected.extend(p for p in ROOT.iterdir() if p.is_file() and (
        p.suffix.lower() == '.trn' or (p.suffix.lower() == '.log' and
        (p.name.startswith('fluent-') or p.name == 'cortexerror.log'))))
    selected = sorted(set(selected), key=lambda p: p.relative_to(ROOT).as_posix())
    return selected, excluded


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    files, excluded = inventory()
    total = sum(p.stat().st_size for p in files)
    summary = {'files': len(files), 'source_bytes': total,
               'source_GiB': round(total / 2**30, 3), 'excluded_entries': excluded}
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if '--inventory' in sys.argv:
        return
    if not OUT.is_dir():
        raise FileNotFoundError(OUT)
    if shutil.disk_usage(OUT).free < total * 1.03 + 100_000_000:
        raise RuntimeError('Insufficient space for a conservative ZIP upper bound')
    now = datetime.now().astimezone()
    name = '水下机械臂_仿真数据与工程文件_' + now.strftime('%Y%m%d_%H%M%S') + '.zip'
    target = OUT / name
    partial = OUT / (name + '.partial')
    manifest = {
        'created_at': now.isoformat(), 'project_root_in_archive': TOP,
        'scope': 'Existing engineering/data snapshot, including failed and unverified historical runs. Not a certified successful simulation set.',
        'excluded_entries': excluded,
        'excluded_top_level_categories': ['.git', '.narrafork', 'Fluent scratch directories',
                                           'GPU caches', 'reference papers', 'unrelated institutional documents'],
        'source_file_count': len(files), 'source_bytes': total, 'files': [],
    }
    note = '''水下机械臂仿真数据与工程文件归档

一、内容
保留原工作区的相对目录：
- main.m、run_full_project_pipeline.m、run_hydro_surrogate.m：MATLAB入口。
- src、experiments：动力学、轨迹、控制、代理模型、CFD接口和扩展试验源码。
- model、urdf、机器人及原始模型ZIP：机构描述、STL/CAD等几何资产。
- cfd：网格、Case/Data、UDF、自动化脚本、52组和历史工况数据、检查点及日志。
- data：MAT存档、CSV、GPR模型及历史对比表。
- docs：证据核验版报告、核验记录、图表动画与历史报告。
- 根目录Fluent日志：保留原生计算过程证据，包括失败记录。

二、必须先读
请优先查看 docs/水下机械臂_前期工作详细报告_证据核验版.docx 或同名Markdown。
本包是现有文件快照，不代表所有算例均成功、收敛或完成全周期。
52组多为短时记录；部分数据复用、汇总改写及旧表来源问题已在核验报告披露。
旧Excel、历史报告与绘图保留用于追溯，不能把其中“3.28%误差”等已撤回结论当作验证结果。
README内的16秒方案等历史描述不一定对应当前配置；以实际数据与核验报告为准。
此次只打包与校验文件，不启动MATLAB/Fluent、不重训模型、不改写原始文件。

三、使用提示
解压后保持目录结构。MATLAB与Ansys Fluent需自行安装并具备相应授权。
Python虚拟环境和运行缓存未打包；第三方依赖需按实际脚本导入项配置。
部分脚本仍含D:/work4或本机安装目录等绝对路径，换机器后请先检查并调整。
已编译UDF具有平台/版本依赖，必要时根据源码重新编译；不能保证直接跨机器运行。
不要直接运行旧的一键汇总/Excel生成程序覆盖数据，先复制工作目录并阅读脚本。
超过4 GB的ZIP采用ZIP64；解压目标磁盘需有足够空间，建议NTFS或exFAT。

四、完整性
文件清单_SHA256.json记录每个收录源文件的路径、大小、修改时间与SHA-256。
打包程序会解压读取每个条目并核对SHA-256/CRC，同时检查打包期间源文件是否变化。
这证明归档字节完整性，不证明数值收敛、模型精度或科学结论。
本包排除Git/会话元数据、Python环境、缓存、临时连接/锁文件、参考论文及无关单位简介。
详细排除项见清单。归档中的失败数据和中间网格有意保留，不视为可信最终结果。
'''
    processed = 0
    last_print = -1
    stats = {}
    with zipfile.ZipFile(partial, 'x', compression=zipfile.ZIP_DEFLATED,
                         compresslevel=1, allowZip64=True) as archive:
        archive.writestr(TOP + '/00_打包说明.txt', note.encode('utf-8-sig'))
        for index, p in enumerate(files, 1):
            before = p.stat()
            rel = p.relative_to(ROOT).as_posix()
            arc = TOP + '/' + rel
            info = zipfile.ZipInfo.from_file(p, arc, strict_timestamps=False)
            info.compress_type = zipfile.ZIP_DEFLATED
            info._compresslevel = 1
            digest = hashlib.sha256()
            count = 0
            with p.open('rb') as source, archive.open(info, 'w', force_zip64=True) as dest:
                while block := source.read(CHUNK):
                    digest.update(block)
                    dest.write(block)
                    count += len(block)
            after = p.stat()
            if before.st_size != count or before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
                raise RuntimeError('Source changed during archiving: ' + rel)
            stats[p] = (before.st_size, before.st_mtime_ns)
            manifest['files'].append({'path': rel, 'bytes': count, 'mtime_ns': before.st_mtime_ns,
                                      'sha256': digest.hexdigest()})
            processed += count
            progress = int(processed * 100 / max(total, 1))
            if progress // 10 > last_print:
                print(json.dumps({'phase': 'compress', 'percent': progress, 'files': index,
                                  'total_files': len(files)}, ensure_ascii=False), flush=True)
                last_print = progress // 10
        archive.writestr(TOP + '/文件清单_SHA256.json',
                         json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8'))
    print('Verifying every archive entry against SHA-256 and ZIP CRC...', flush=True)
    with zipfile.ZipFile(partial, 'r') as archive:
        names = archive.namelist()
        if len(names) != len(set(names)) or len(names) != len(files) + 2:
            raise RuntimeError('Unexpected or duplicate archive entries')
        loaded = json.loads(archive.read(TOP + '/文件清单_SHA256.json'))
        archive.read(TOP + '/00_打包说明.txt')
        if loaded != manifest:
            raise RuntimeError('Manifest mismatch')
        verified_bytes = 0
        last_print = -1
        for entry in manifest['files']:
            digest = hashlib.sha256()
            count = 0
            with archive.open(TOP + '/' + entry['path']) as stream:
                while block := stream.read(CHUNK):
                    digest.update(block)
                    count += len(block)
            if digest.hexdigest() != entry['sha256'] or count != entry['bytes']:
                raise RuntimeError('Archive verification failed: ' + entry['path'])
            verified_bytes += count
            progress = int(verified_bytes * 100 / max(total, 1))
            if progress // 20 > last_print:
                print(json.dumps({'phase': 'verify', 'percent': progress}), flush=True)
                last_print = progress // 20
    for p, expected in stats.items():
        s = p.stat()
        if (s.st_size, s.st_mtime_ns) != expected:
            raise RuntimeError('Source changed since snapshot: ' + str(p))
    partial.rename(target)
    with target.open('rb') as stream:
        zip_sha = hashlib.file_digest(stream, 'sha256').hexdigest()
    result = {'archive': target.relative_to(ROOT).as_posix(),
              'archive_bytes': target.stat().st_size,
              'archive_GiB': round(target.stat().st_size / 2**30, 3),
              'source_file_count': len(files), 'source_bytes': total,
              'archive_entries': len(files) + 2,
              'sha256': zip_sha, 'verification': 'all entries CRC/SHA-256 passed; source stat fingerprints unchanged'}
    with (OUT / (name + '.verification.json')).open('x', encoding='utf-8') as receipt:
        json.dump(result, receipt, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()

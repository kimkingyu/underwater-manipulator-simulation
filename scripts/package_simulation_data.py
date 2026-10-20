"""Create a small, read-only snapshot of simulation result data and audit context."""
import csv
import hashlib
import io
import json
import math
import os
from collections import defaultdict
from datetime import datetime
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'deliverables'
PREFIX = '水下机械臂_数据精简包/'
REPORT = 'docs/水下机械臂_前期工作详细报告_证据核验版'
OMITTED = {
    'large_solver_assets': '网格、Case/Data、CAD、检查点和全场数据；仍在原工作区及完整工程包中',
    'presentation_assets': '动画、图片、参考论文、历史说明文档',
    'legacy_disputed_tables': [
        'data/full_52_cases_comparison.csv',
        'data/full_52_cases_comparison_with_deviations.xlsx',
        '水下机械臂_52组CFD与动力学及GPR代理模型对比与偏差分析表.xlsx',
    ],
    'environments': 'Python虚拟环境、缓存、第三方工具目录、Git和会话元数据',
    'redundant_demo': 'data/hydro_surrogate/smoke/',
}


def build_selection():
    selected = {}
    for p in (ROOT / 'data').rglob('*'):
        if not p.is_file() or p.is_symlink():
            continue
        rel = p.relative_to(ROOT).as_posix()
        if rel in OMITTED['legacy_disputed_tables'] or '/smoke/' in rel:
            continue
        if p.suffix.lower() in {'.mat', '.csv', '.json', '.txt'}:
            selected[rel] = 'MATLAB结果、代理模型与数据集、工况参数'
    for base, dirs, names in os.walk(ROOT / 'cfd', followlinks=False):
        dirs[:] = [n for n in dirs if not n.startswith('.') and n not in
                   {'__pycache__', 'vendor', 'node_modules', 'venv'}
                   and not (Path(base) / n).is_symlink() and not (Path(base) / n).is_junction()]
        for name in names:
            p = Path(base) / name
            if p.is_symlink():
                continue
            rel = p.relative_to(ROOT).as_posix()
            matrix = rel.startswith(('cfd/parametric_cfd_matrix/', 'cfd/parametric_cfd_matrix_52/'))
            if p.suffix.lower() == '.csv' or name.endswith('.validation.json') or (
                    matrix and p.suffix.lower() == '.json'):
                selected[rel] = 'CFD原样时序、历史校验sidecar或汇总；不代表收敛认证'
    extras = {
        REPORT + '.docx': '证据核验报告Word版',
        REPORT + '.md': '证据核验报告正文',
        REPORT + '_核验记录.json': '原核验记录及来源哈希',
        'cfd/wetted_arm_rebuild/cfd_vs_matlab_report.json': '30步历史对比数组与指标',
        'cfd/source_case_settings.json': '源算例设置记录',
        'src/cfd/cfd_case_parameters.m': '当前工况配置快照；不是所有历史算例的配置',
        'src/cfd/arm_motion_config.h': '当前运动参数快照；不是所有历史算例的配置',
        'src/dynamics/underwater_hydro_defaults.m': '当前解析模型参数快照',
        'fluent-20260924-014845-33960.trn': '网格数量与质量问题的原生日志证据',
        'cfd/wetted_arm_rebuild/wetted_fluid_step1_run_native/native_000001_j1.txt': '载荷采集核对的原生报告示例',
    }
    selected.update(extras)
    for rel in selected:
        if not (ROOT / rel).is_file():
            raise FileNotFoundError(rel)
    return dict(sorted(selected.items()))


def describe_csv(rel, content):
    reader = csv.DictReader(io.StringIO(content.decode('utf-8-sig')))
    rows = list(reader)
    columns = reader.fieldnames or []
    result = {'path': rel, 'rows': len(rows), 'columns': columns,
              'interpretation': '记录条数和时间范围，不是求解完成或收敛判定'}
    column = next((n for n in ('flow_time_s', 'time_s', 'time', 't') if n in columns), None)
    if column:
        times = []
        for row in rows:
            try:
                value = float(row.get(column, ''))
            except (ValueError, TypeError):
                continue
            if math.isfinite(value):
                times.append(value)
        result['time_column'] = column
        if times:
            result['recorded_time_range'] = [min(times), max(times)]
    sidecar = rel[:-4] + '.validation.json'
    if (ROOT / sidecar).is_file():
        result['sidecar_path'] = sidecar
    return result


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    selected = build_selection()
    series = [p for p in selected if p.startswith('cfd/') and p.endswith('.csv')]
    cases52 = [p for p in series if p.startswith('cfd/parametric_cfd_matrix_52/') and p.endswith('/moments.csv')]
    cases9 = [p for p in series if p.startswith('cfd/parametric_cfd_matrix/') and p.endswith('/moments.csv')]
    assert len(cases52) == 52 and len(cases9) == 9, 'Missing matrix CSVs'
    assert 'data/joint_loads_data.mat' in selected
    assert 'data/underarm_3dof_sim_data.mat' in selected
    assert 'data/hydro_surrogate/full/model.mat' in selected
    assert 'data/hydro_surrogate/full/dataset.mat' in selected
    assert 'data/hydro_surrogate/full/fresh_audit/dataset.mat' in selected
    assert all((p[:-4] + '.validation.json') in selected for p in cases52 + cases9)
    now = datetime.now().astimezone()
    filename = '水下机械臂_数据精简包_' + now.strftime('%Y%m%d_%H%M%S') + '.zip'
    target = OUT / filename
    temporary = OUT / (filename + '.partial')
    if not OUT.is_dir():
        raise FileNotFoundError(OUT)
    manifest = {'created_at': now.isoformat(), 'type': 'result-data-only snapshot',
                'excluded_categories': OMITTED, 'files': []}
    index = {'matrix52_csv_count': len(cases52), 'historical9_csv_count': len(cases9),
             'all_cfd_csv_count': len(series), 'cfd_series': [],
             'duplicate_cfd_csv_groups': [],
             'warning': '相同CSV按工况目录原样保留，不计为独立求解。旧sidecar中的valid/completed字段不等于通过科学或收敛验证。'}
    hashes = defaultdict(list)
    stat_snapshots = {}
    notes = '''水下机械臂：数据精简包（不含完整求解工程）

先看这里
本包供读取已有数据、重新统计、作图和核查来源使用。没有重新求解、重训或修正原始数据。
它不包含Fluent网格、Case/Data、完整压力/速度场、CAD及中间检查点，不能独立用于Fluent续算或全场后处理。
完整工程包和原工作区均保留，未被本次精简删除或覆盖。

目录
1. data/：关节负载、控制和UVMS仿真MAT；几何物性；GPR模型、训练/评估数据与历史预测记录。
2. cfd/parametric_cfd_matrix_52/：52组原样力矩CSV和原sidecar/汇总。
3. cfd/parametric_cfd_matrix/：历史9组原样力矩CSV和原sidecar/汇总。
4. cfd/wetted_arm_rebuild/及其余cfd子目录：历史短时和长时尝试的CSV与sidecar。
5. docs/：证据核验版Word、Markdown及核验JSON。请优先阅读它们。
6. src/：仅三个轻量配置快照，便于解释当前参数；不代表所有历史数据使用同一配置。
7. CFD时序索引.json：各CSV列名、实际记录条数与时间范围、同内容文件分组。
8. 文件清单_SHA256.json：来源相对路径、大小、SHA-256和选入理由。

数据可信范围
- 52组矩阵主要是0.03—0.08秒短时记录，含重复/复用；不能视为52组全部收敛的稳态真值。
- 保留每个工况自己的目录，哪怕CSV内容相同，避免丢失工况对应关系；索引标出重复来源。
- CFD原始CSV按字节保留，不删除异常值；失败/发散时序也保留用于判断有效范围。
- 原有汇总和validation文件也原样保留以便追溯，不是本次重新签发的验证。
- case_32、40、41、42的历史汇总固定改写问题已在核验报告披露；勿直接把汇总中的代表点当原始求解值。
- 已撤回结论的52组综合对比CSV/Excel、旧宣传图表没有收入本精简包。
- GPR模型、解析代理报告、数据集和测试输出来自现有存档，可能属于不同生成批次；不能假设它们一一对应。
- 报告提及的一些源码和大工程证据本包未收录；需进一步复现时查看完整工程包或原仓库。

使用
保留相对目录解压。CSV/JSON可直接读取；MAT和GPR对象需相应MATLAB环境或兼容读取工具。
数据包不附带第三方软件、工具箱和授权，也不保证跨版本直接加载训练模型。
SHA-256和ZIP校验仅验证文件完整性，不证明仿真物理精度。
'''
    with zipfile.ZipFile(temporary, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        z.writestr(PREFIX + '00_先读说明.txt', notes.encode('utf-8-sig'))
        for rel, reason in selected.items():
            source = ROOT / rel
            before = source.stat()
            content = source.read_bytes()
            after = source.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Source changed: ' + rel)
            digest = hashlib.sha256(content).hexdigest()
            manifest['files'].append({'path': rel, 'bytes': len(content), 'sha256': digest,
                                      'source_mtime_ns': before.st_mtime_ns, 'selection_reason': reason})
            stat_snapshots[rel] = (before.st_size, before.st_mtime_ns)
            info = zipfile.ZipInfo.from_file(source, PREFIX + rel)
            z.writestr(info, content, compress_type=zipfile.ZIP_DEFLATED, compresslevel=6)
            if rel in series:
                index['cfd_series'].append(describe_csv(rel, content))
                hashes[digest].append(rel)
        index['duplicate_cfd_csv_groups'] = [v for v in hashes.values() if len(v) > 1]
        manifest['source_file_count'] = len(selected)
        manifest['source_bytes'] = sum(v['bytes'] for v in manifest['files'])
        z.writestr(PREFIX + 'CFD时序索引.json', json.dumps(index, ensure_ascii=False, indent=2).encode('utf-8'))
        z.writestr(PREFIX + '文件清单_SHA256.json', json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8'))
    with zipfile.ZipFile(temporary) as z:
        assert len(z.namelist()) == len(set(z.namelist())) == len(selected) + 3
        bad = z.testzip()
        if bad:
            raise RuntimeError('CRC failed: ' + bad)
        assert json.loads(z.read(PREFIX + '文件清单_SHA256.json')) == manifest
        assert json.loads(z.read(PREFIX + 'CFD时序索引.json')) == index
        for item in manifest['files']:
            data = z.read(PREFIX + item['path'])
            if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
                raise RuntimeError('SHA-256 failed: ' + item['path'])
            current = (ROOT / item['path']).stat()
            if (current.st_size, current.st_mtime_ns) != stat_snapshots[item['path']]:
                raise RuntimeError('Source changed before verification: ' + item['path'])
        assert not any(n.endswith(('.h5', '.msh', '.neu', '.gif', '.xlsx', '.png')) for n in z.namelist())
    temporary.rename(target)
    with target.open('rb') as stream:
        archive_hash = hashlib.file_digest(stream, 'sha256').hexdigest()
    result = {'archive': target.relative_to(ROOT).as_posix(), 'archive_bytes': target.stat().st_size,
              'archive_MiB': round(target.stat().st_size / 2**20, 3),
              'source_file_count': len(selected), 'source_bytes': manifest['source_bytes'],
              'archive_entries': len(selected) + 3, 'matrix52_csv_count': 52,
              'historical9_csv_count': 9, 'all_cfd_csv_count': len(series),
              'sha256': archive_hash, 'validation': 'CRC and every source SHA-256 passed; source stats unchanged'}
    receipt = OUT / (filename + '.verification.json')
    with receipt.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

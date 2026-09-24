function result = prepare_fluent_udf(cfg)
%PREPARE_FLUENT_UDF Stage source and header beside the Fluent case.
% Edit cfd_case_parameters.m, then run result = prepare_fluent_udf().
% In Fluent's compiled-functions dialog, add targetSource under Source Files
% and targetHeader under Header Files. Use library name libudf_parallel.
% Staging is NOT compilation or loading. Existing library directories and
% DLLs are left untouched; Fluent manages them during an explicit build.

if nargin < 1 || isempty(cfg)
    cfg = cfd_case_parameters();
end
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
sourcePath = fullfile(rootDir, 'src', 'cfd', 'arm_motion.c');
targetDir = fullfile(rootDir, 'cfd', '1_files', 'dp0', 'FFF', 'Fluent');
if ~isfile(sourcePath)
    error('cfd:SourceMissing', '找不到 UDF 源文件: %s', sourcePath);
end
if ~isfolder(targetDir)
    error('cfd:FluentSourceDir', '找不到 Fluent 案例目录: %s', targetDir);
end

% Fluent's Windows SConstruct reads C files using the system encoding.
% ASCII is portable across GBK/UTF-8; reject non-ASCII before staging.
assert_ascii_file(sourcePath);
generated = write_cfd_motion_config(cfg);
assert_ascii_file(generated.headerPath);
% Keep compiler inputs outside libudf_parallel/src to avoid mixing user
% inputs with Fluent's generated build tree or selecting stale v5 copies.
targetSource = fullfile(targetDir, 'arm_motion_parameterized.c');
targetHeader = fullfile(targetDir, 'arm_motion_config.h');
copyfile(sourcePath, targetSource, 'f');
copyfile(generated.headerPath, targetHeader, 'f');

result = struct('config', cfg, 'generated', generated, ...
    'sourcePath', sourcePath, 'targetSource', targetSource, ...
    'targetHeader', targetHeader);
fprintf('Fluent 编译输入已准备，现有库尚未重新编译或加载:\n');
fprintf('  源文件栏添加: %s\n', targetSource);
fprintf('  头文件栏添加: %s\n', targetHeader);
fprintf('  库名称: libudf_parallel\n');
fprintf('请从编译列表移除旧版源文件条目（不要删除磁盘文件）。\n');
fprintf('编译成功并确认主机和节点库已更新后，再点击加载。\n');
end

function assert_ascii_file(path)
fid = fopen(path, 'rb');
if fid < 0
    error('cfd:SourceRead', '无法读取编译输入文件: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid, Inf, '*uint8');
if any(bytes > 127)
    error('cfd:UdfEncoding', ...
        'Fluent 编译输入必须为纯 ASCII（包括注释、无 BOM），请检查: %s', path);
end
end

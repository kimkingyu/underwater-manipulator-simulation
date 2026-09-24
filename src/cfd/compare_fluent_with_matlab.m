function report = compare_fluent_with_matlab(fluentDataFile, currentSpeed, currentAngleDeg, options)
%COMPARE_FLUENT_WITH_MATLAB Compare Fluent CFD joint moments with MATLAB hydro models.
%
% Usage:
%   report = compare_fluent_with_matlab('path/to/fluent_moment.csv');
%   report = compare_fluent_with_matlab('path/to/fluent_moment.csv', 0.25, 45.0);
%   report = compare_fluent_with_matlab('path/to/fluent_moment.csv', 0.25, 45.0, options);
%
% Options struct fields:
%   options.allow_invalid    - logical scalar (default false). When true, allows
%                              processing known invalid/zero CFD data in
%                              diagnostic mode without throwing an error before
%                              exporting figures. Figures are marked INVALID
%                              and no valid metrics are calculated.
%   options.include_buoyancy - logical scalar (default false). Whether MATLAB
%                              hydrodynamic reference includes hydrostatic
%                              buoyancy. Defaults to false to match zero-gravity
%                              dynamic fluid CFD simulations.
%   options.require_sidecar  - logical scalar (default false). If true, an explicit
%                              *.validation.json sidecar file is mandatory.
%   options.actual_vc_world  - finite 1x3 or 3x1 real vector [m/s]. Actual measured
%                              flow velocity vector in world frame. If provided,
%                              overrides currentSpeed and currentAngleDeg.
%   options.env_verified     - logical scalar (default false). Whether actual boundary
%                              conditions and medium properties have been verified.
%                              If false, condition assertion is rejected and status
%                              is 'unverified' (do not overclaim).
%   options.out_figure       - string/char. Path to save the comparison figure.
%   options.save_metrics_mat - logical scalar (default false). Save traceable MAT file.
%   options.metrics_mat_path - string/char. Path for metrics MAT file.
%   options.col_indices      - 1x4 positive integer vector [time_idx, tau1_idx, tau2_idx, tau3_idx].

if nargin < 1 || isempty(fluentDataFile)
    [fName, fPath] = uigetfile({'*.out;*.txt;*.csv;*.dat', 'Fluent Monitor Files (*.out, *.txt, *.csv, *.dat)'; ...
                                '*.*', 'All Files (*.*)'}, '选择 Fluent 导出的关节力矩监控文件');
    if isequal(fName, 0)
        fprintf('未选择文件，退出比对。\n');
        report = struct([]);
        return;
    end
    fluentDataFile = fullfile(fPath, fName);
end

if nargin < 2 || isempty(currentSpeed)
    currentSpeed = 0.25; % 默认 0.25 m/s
end
if nargin < 3 || isempty(currentAngleDeg)
    currentAngleDeg = 45.0; % 默认方位角
end
if nargin < 4 || isempty(options)
    options = struct();
end

is_allow_invalid = isfield(options, 'allow_invalid') && ...
                   isscalar(options.allow_invalid) && ...
                   islogical(options.allow_invalid) && ...
                   options.allow_invalid;
is_diagnostic = false;
diagnostic_reason = '';

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(rootDir, 'src')));

% 0. 检查 Sidecar 验证文件 (*.validation.json)
[fPath, fBase, fExt] = fileparts(fluentDataFile);
sidecarCandidate1 = fullfile(fPath, [fBase, '.validation.json']);
sidecarCandidate2 = fullfile(fPath, [fBase, fExt, '.validation.json']);
sidecarFile = '';
if isfile(sidecarCandidate1)
    sidecarFile = sidecarCandidate1;
elseif isfile(sidecarCandidate2)
    sidecarFile = sidecarCandidate2;
end

hasSidecar = ~isempty(sidecarFile);
sidecarValid = [];
sidecarReason = '';
if hasSidecar
    try
        sidecarContent = jsondecode(fileread(sidecarFile));
        if isfield(sidecarContent, 'valid')
            val = sidecarContent.valid;
            if islogical(val) && isscalar(val)
                sidecarValid = val;
            elseif isnumeric(val) && isscalar(val) && (val == 0 || val == 1)
                sidecarValid = (val == 1);
            else
                error('compare_fluent:InvalidSidecarFormat', 'Sidecar valid 字段必须是标量逻辑值 (scalar logical)。');
            end
        else
            error('compare_fluent:InvalidSidecarFormat', 'Sidecar 文件缺失 valid 字段。');
        end
        if isfield(sidecarContent, 'reason')
            sidecarReason = char(sidecarContent.reason);
        end
    catch jsonErr
        if startsWith(jsonErr.identifier, 'compare_fluent:')
            rethrow(jsonErr);
        end
        error('compare_fluent:SidecarReadError', 'Sidecar 验证文件解析失败: %s (%s)', sidecarFile, jsonErr.message);
    end
end

% 严格拦截已知无效 sidecar：未开启 allow_invalid 时必须在覆盖原图前报错
if hasSidecar && isequal(sidecarValid, false)
    if ~is_allow_invalid
        error('compare_fluent:InvalidSidecar', ...
            'CFD 结果已被 sidecar 判定为无效: %s (原因: %s)。拒绝作为验证结果。', ...
            fluentDataFile, sidecarReason);
    else
        is_diagnostic = true;
        diagnostic_reason = sprintf('Sidecar 判定无效: %s', sidecarReason);
        fprintf('  [诊断模式] Sidecar 声明数据无效: %s。继续运行仅做诊断，图上将注明 INVALID，不计算有效指标。\n', sidecarReason);
    end
end

if isfield(options, 'require_sidecar') && options.require_sidecar && ~hasSidecar
    error('compare_fluent:MissingSidecar', ...
        '未找到验证 sidecar (*.validation.json): %s，options.require_sidecar 要求显式验证。', fluentDataFile);
end

% 1. 读取并校验 Fluent 监控文件
fprintf('=== [1/4] 读取 Fluent 监控文件: %s ===\n', fluentDataFile);
cfd = parse_fluent_monitor(fluentDataFile, options);
fprintf('  成功解析 Fluent 数据: %d 个时间点 (%.4f s ~ %.4f s)\n', ...
    numel(cfd.time), cfd.time(1), cfd.time(end));

% 全零力矩检测 (注意: 全零本身不等于已知固侧，只有 sidecar invalid 才确知)
isAllZero = all(abs(cfd.tau(:)) < 1e-12);
if isAllZero
    if hasSidecar && isequal(sidecarValid, true)
        fprintf('  [提示] 监控力矩全为零，但 sidecar 明确标记 valid: true，视为静止/零流动的真实物理零值。\n');
    else
        zeroReason = '检测到未验证的全零关节力矩数据';
        if ~is_allow_invalid
            error('compare_fluent:UnverifiedZeroMoment', ...
                '%s，拒绝作为有效验证结果。如需排查，请设置 options.allow_invalid=true 开启诊断模式。', zeroReason);
        else
            is_diagnostic = true;
            diagnostic_reason = zeroReason;
            fprintf('  [诊断模式] %s。图上将醒目标注 INVALID，不计算有效指标。\n', zeroReason);
        end
    end
end

% 2. 装载 MATLAB 动力学仿真基准与时间范围边界校验 (拒绝外插)
fprintf('=== [2/4] 加载 MATLAB 动力学基准并验证时间范围 ===\n');
refMat = fullfile(rootDir, 'data', 'joint_loads_data.mat');
if ~isfile(refMat)
    error('compare_fluent:MissingRefMat', '未找到 MATLAB 基准数据文件: %s', refMat);
end
ref = load(refMat);

timeTol = 1e-6;
cfd_t_min = min(cfd.time);
cfd_t_max = max(cfd.time);
ref_t_min = min(ref.time);
ref_t_max = max(ref.time);

if cfd_t_min < ref_t_min - timeTol || cfd_t_max > ref_t_max + timeTol
    error('compare_fluent:ExtrapolationRefused', ...
        'CFD 时间范围 [%.4f, %.4f] s 超出 MATLAB 参考基准范围 [%.4f, %.4f] s，拒绝外插。', ...
        cfd_t_min, cfd_t_max, ref_t_min, ref_t_max);
end

t_common = cfd.time;
% 有界插值提取关节位形与运动导数，严格禁止使用 'extrap'
q_int   = interp1(ref.time, ref.q_seq,   t_common, 'linear');
qd_int  = interp1(ref.time, ref.qd_seq,  t_common, 'linear');
qdd_int = interp1(ref.time, ref.qdd_seq, t_common, 'linear');

% 3. 环境与来流矢量配置 (禁止过度断言未核验边界条件)
urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
if ~isfile(urdfPath)
    error('compare_fluent:MissingURDF', '未找到机械臂 URDF 模型: %s', urdfPath);
end
robot = importrobot(urdfPath);
robot.DataFormat = 'row';

env_eval = ref.env;
flow_speed_specified = false;
if isfield(options, 'actual_vc_world') && ~isempty(options.actual_vc_world)
    vc_in = options.actual_vc_world;
    if ~(isnumeric(vc_in) && numel(vc_in) == 3 && all(isfinite(vc_in)) && isreal(vc_in))
        error('compare_fluent:InvalidActualVelocity', ...
            'options.actual_vc_world 必须是包含 3 个有限实数的向量 [vx, vy, vz]。');
    end
    vc_world = reshape(double(vc_in), 3, 1);
    env_eval.vc_world = vc_world;
    env_eval.current_speed = norm(vc_world);
    env_eval.current_psi = atan2(vc_world(2), vc_world(1));
    env_eval.current_alpha = atan2(vc_world(3), hypot(vc_world(1), vc_world(2)));
    flow_speed_specified = true;
else
    radAngle = deg2rad(currentAngleDeg);
    env_eval.current_speed = currentSpeed;
    env_eval.current_psi = radAngle;
    env_eval.current_alpha = 0.0;
    env_eval.vc_world = [currentSpeed * cos(radAngle); currentSpeed * sin(radAngle); 0.0];
end

% 仅当 options.env_verified == true 时才视为环境已完全核验；
% 即使指定了实际速度，其他介质物性与边界未核验时切勿过度断言 (don't overclaim)
env_verified = isfield(options, 'env_verified') && ...
               isscalar(options.env_verified) && ...
               islogical(options.env_verified) && ...
               options.env_verified;

if env_verified
    flow_verified = true;
    cond_status = 'verified';
else
    flow_verified = false;
    cond_status = 'unverified';
    if flow_speed_specified
        warning('compare_fluent:FlowConditionUnverified', ...
            ['已应用 options.actual_vc_world 速度矢量，但环境介质物性及边界设置未经独立实测核验 (未设 options.env_verified=true)。\n' ...
             '工况状态标为 unverified，指标仅供参考。']);
    else
        warning('compare_fluent:FlowConditionUnverified', ...
            ['未检测到实测环境 metadata，禁止仅凭 cfd_case_parameters 声称实测入口 45 度。\n' ...
             '工况状态标为 unverified，指标仅供参考。']);
    end
end

% 4. 严密 fluid-on-arm 口径重算: evaluate_hydro_reference
% 口径: parts.drag + parts.added_inertia + parts.added_coriolis
% CFD 不开 gravity，默认不含浮力；是否计入浮力由 options.include_buoyancy 明确指定
[~, parts] = evaluate_hydro_reference(robot, ref.hydro, env_eval, q_int, qd_int, qdd_int, env_eval.vc_world.');

include_buoyancy = isfield(options, 'include_buoyancy') && ...
                    isscalar(options.include_buoyancy) && ...
                    islogical(options.include_buoyancy) && ...
                    options.include_buoyancy;
if include_buoyancy
    tau_matlab_dynamic = parts.drag + parts.added_inertia + parts.added_coriolis + parts.buoyancy;
else
    tau_matlab_dynamic = parts.drag + parts.added_inertia + parts.added_coriolis;
end

% 代理模型预测口径同步
surrogateFile = fullfile(rootDir, 'data', 'hydro_surrogate', 'full', 'model.mat');
hasSurrogate = isfile(surrogateFile);
if hasSurrogate
    s = load(surrogateFile, 'model');
    [tau_surr_all, sinfo] = predict_hydro_surrogate(s.model, q_int, qd_int, qdd_int, env_eval.vc_world.', 'warn');
    if include_buoyancy
        tau_surr_dynamic = tau_surr_all;
    else
        tau_surr_dynamic = sinfo.dynamic;
    end
else
    tau_surr_dynamic = nan(size(tau_matlab_dynamic));
end

% 判定全流程有效性 (Valid 准入规则)
% 只有非诊断无效、具有显式 valid:true sidecar、且环境条件已核验时，才给予有效判定
is_sidecar_true = hasSidecar && isequal(sidecarValid, true);
is_fully_verified = (~is_diagnostic) && is_sidecar_true && flow_verified;

% 5. 定量统计分析 (严格禁止相关性自动翻转 CFD 符号)
fprintf('=== [3/4] 误差统计与精度评估 ===\n');
jointNames = {'关节 1 (基座水平扫掠)', '关节 2 (肩部俯仰)', '关节 3 (小臂与夹爪)'};
metrics = cell(3, 1);

fprintf('%-24s | %-10s | %-10s | %-12s | %-8s\n', '关节', 'RMSE(N*m)', 'MAE(N*m)', 'CFD峰值(N*m)', 'R^2');
fprintf('----------------------------------------------------------------------\n');
for j = 1:3
    y_cfd = cfd.tau(:, j); % 严格保持原始数值，绝不自动反向
    y_mat = tau_matlab_dynamic(:, j);
    
    if is_diagnostic
        rmse = nan;
        mae = nan;
        pk_cfd = max(abs(y_cfd));
        pk_mat = max(abs(y_mat));
        pk_err_pct = nan;
        r2 = nan;
        status_str = 'INVALID - 诊断数据';
        fprintf('%-24s | %10s | %10s | %12.4f | %8s  [%s]\n', ...
            jointNames{j}, 'NaN', 'NaN', pk_cfd, 'NaN', status_str);
    else
        err = y_cfd - y_mat;
        rmse = sqrt(mean(err.^2));
        mae = mean(abs(err));
        pk_cfd = max(abs(y_cfd));
        pk_mat = max(abs(y_mat));
        pk_err_pct = abs(pk_cfd - pk_mat) / max(pk_mat, 1e-6) * 100;
        
        sst = sum((y_mat - mean(y_mat)).^2);
        if sst > 1e-8
            r2 = 1 - sum(err.^2) / sst;
        else
            r2 = nan;
        end
        
        if ~is_fully_verified
            status_str = 'UNVERIFIED - 仅供参考';
            fprintf('%-24s | %10.4f | %10.4f | %12.4f | %8.4f  [%s]\n', ...
                jointNames{j}, rmse, mae, pk_cfd, r2, status_str);
        else
            fprintf('%-24s | %10.4f | %10.4f | %12.4f | %8.4f\n', ...
                jointNames{j}, rmse, mae, pk_cfd, r2);
        end
    end
    
    metrics{j} = struct('Joint', j, 'Name', jointNames{j}, ...
                        'RMSE', rmse, 'MAE', mae, ...
                        'PeakCFD', pk_cfd, 'PeakMATLAB', pk_mat, ...
                        'PeakErrPct', pk_err_pct, 'R2', r2, ...
                        'Valid', is_fully_verified);
end
fprintf('----------------------------------------------------------------------\n\n');

% 6. 绘制时序对比曲线图
fprintf('=== [4/4] 绘制时序对比曲线图 ===\n');
if is_diagnostic
    figTitle = 'Fluent CFD 与 MATLAB 水动力矩拟合对比 [INVALID 诊断模式 - 不可作为验证依据]';
elseif ~is_fully_verified
    figTitle = 'Fluent CFD 与 MATLAB 水动力矩拟合对比 [UNVERIFIED 未核验 - 指标仅供参考]';
else
    figTitle = 'Fluent CFD 与 MATLAB 水动力矩拟合对比';
end

f = figure('Name', figTitle, 'Color', 'w', 'Position', [100, 100, 1100, 750], 'Visible', 'off');

for j = 1:3
    subplot(3, 1, j);
    plot(t_common, cfd.tau(:, j), 'r-', 'LineWidth', 1.8, 'DisplayName', 'ANSYS Fluent CFD (原始值)'); hold on;
    plot(t_common, tau_matlab_dynamic(:, j), 'b--', 'LineWidth', 1.6, 'DisplayName', 'MATLAB 参考水动力');
    if hasSurrogate
        plot(t_common, tau_surr_dynamic(:, j), 'g:', 'LineWidth', 1.4, 'DisplayName', 'MATLAB 水动力代理模型');
    end
    
    grid on;
    xlabel('时间 t / s', 'FontSize', 10);
    ylabel(sprintf('%s / N*m', jointNames{j}), 'FontSize', 10);
    
    if is_diagnostic
        title(sprintf('%s 水动力矩对比 [INVALID 诊断数据: %s]', jointNames{j}, diagnostic_reason), ...
            'FontSize', 11, 'Color', [0.8, 0.1, 0.1]);
        yl = ylim(); xl = xlim();
        text(mean(xl), mean(yl), 'INVALID DATA (Not for Validation)', ...
            'Color', [0.8, 0.1, 0.1], 'FontSize', 13, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'BackgroundColor', [1, 0.95, 0.95]);
    elseif ~is_fully_verified
        title(sprintf('%s 水动力矩对比 [UNVERIFIED - 指标仅供参考] (RMSE = %.4f N*m, 峰值偏差 = %.1f%%)', ...
            jointNames{j}, metrics{j}.RMSE, metrics{j}.PeakErrPct), 'FontSize', 11, 'Color', [0.75, 0.4, 0.1]);
        yl = ylim(); xl = xlim();
        text(mean(xl), yl(1) + 0.15*diff(yl), 'UNVERIFIED (Reference Only - Flow/Sidecar Not Fully Verified)', ...
            'Color', [0.75, 0.4, 0.1], 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'BackgroundColor', [1, 0.98, 0.92]);
    else
        title(sprintf('%s 水动力矩对比 (RMSE = %.4f N*m, 峰值偏差 = %.1f%%)', ...
            jointNames{j}, metrics{j}.RMSE, metrics{j}.PeakErrPct), 'FontSize', 11);
    end
    legend('Location', 'best', 'FontSize', 9);
end

% 确定输出路径并保存 (默认未核验/诊断图另存，绝不覆盖正式图)
if isfield(options, 'out_figure') && ~isempty(options.out_figure)
    outFig = options.out_figure;
else
    if is_diagnostic
        outFig = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_hydro_diagnostic.png');
    elseif ~is_fully_verified
        outFig = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_hydro_unverified.png');
    else
        outFig = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_hydro.png');
    end
end

outDir = fileparts(outFig);
if ~exist(outDir, 'dir') && ~isempty(outDir)
    mkdir(outDir);
end

try
    exportgraphics(f, outFig, 'Resolution', 200);
    fprintf('对比图已保存至: %s\n', outFig);
catch
    saveas(f, outFig);
    fprintf('对比图 (saveas) 已保存至: %s\n', outFig);
end
close(f);

% 7. 可追溯 metrics MAT 保存
save_mat = isfield(options, 'save_metrics_mat') && ...
           isscalar(options.save_metrics_mat) && ...
           islogical(options.save_metrics_mat) && ...
           options.save_metrics_mat;
if save_mat
    if isfield(options, 'metrics_mat_path') && ~isempty(options.metrics_mat_path)
        matSavePath = options.metrics_mat_path;
    else
        matSavePath = fullfile(rootDir, 'cfd', '1_files', 'dp0', 'FFF', 'Fluent', 'matlab_comparison_report.mat');
    end
    matSaveDir = fileparts(matSavePath);
    if ~exist(matSaveDir, 'dir') && ~isempty(matSaveDir)
        mkdir(matSaveDir);
    end
    save(matSavePath, 'metrics', 't_common', 'cfd', 'tau_matlab_dynamic', ...
        'tau_surr_dynamic', 'is_diagnostic', 'is_fully_verified', ...
        'cond_status', 'include_buoyancy');
    fprintf('可追溯评估指标 MAT 已保存至: %s\n', matSavePath);
end

report = struct(...
    'valid', is_fully_verified, ...
    'is_diagnostic', is_diagnostic, ...
    'diagnostic_reason', diagnostic_reason, ...
    'condition_status', cond_status, ...
    'flow_condition_verified', flow_verified, ...
    'sidecar_verified', is_sidecar_true, ...
    'include_buoyancy', include_buoyancy, ...
    'metrics', {metrics}, ...
    'time', t_common, ...
    'cfd_tau', cfd.tau, ...
    'matlab_tau', tau_matlab_dynamic, ...
    'surrogate_tau', tau_surr_dynamic, ...
    'figurePath', outFig, ...
    'sidecar_file', sidecarFile, ...
    'options', options);
end

%% 内部支撑函数: 解析并校验 Fluent 监控文件
function cfd = parse_fluent_monitor(filePath, options)
    if nargin < 2
        options = struct();
    end
    [~, ~, ext] = fileparts(filePath);

    % CSV 格式必须包含必需列名，杜绝错列与静默补零
    if strcmpi(ext, '.csv')
        opts = detectImportOptions(filePath, 'VariableNamingRule', 'preserve');
        tableData = readtable(filePath, opts);
        
        required = {'flow_time_s', 'j1_torque_nm', 'j2_torque_nm', 'j3_torque_nm'};
        missing = required(~ismember(required, tableData.Properties.VariableNames));
        if ~isempty(missing)
            error('compare_fluent:MissingRequiredColumns', ...
                'CSV 监控文件缺失必需列: %s。禁止静默回退至错列猜测。', strjoin(missing, ', '));
        end
        
        raw_time = double(tableData.('flow_time_s'));
        raw_tau  = double([tableData.('j1_torque_nm'), ...
                           tableData.('j2_torque_nm'), ...
                           tableData.('j3_torque_nm')]);
    else
        % 非 CSV 文本监控文件按数值矩阵解析
        fid = fopen(filePath, 'r');
        if fid == -1
            error('compare_fluent:FileOpenError', '无法打开监控文件: %s', filePath);
        end
        cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

        data = [];
        while ~feof(fid)
            line = strtrim(fgetl(fid));
            if isempty(line) || startsWith(line, '#') || startsWith(line, '"') || startsWith(line, '(')
                continue;
            end
            line = strrep(line, ',', ' ');
            nums = sscanf(line, '%f');
            if numel(nums) >= 2
                data = [data; nums.']; %#ok<AGROW>
            end
        end

        if isempty(data)
            error('compare_fluent:EmptyData', '未能从监控文件中提取到有效数值数据。');
        end

        nCols = size(data, 2);
        if isfield(options, 'col_indices') && ~isempty(options.col_indices)
            cols = options.col_indices;
            if ~(isnumeric(cols) && numel(cols) == 4 && all(cols >= 1) && all(cols <= nCols))
                error('compare_fluent:InvalidColIndices', ...
                    'options.col_indices 必须为 4 维正整数向量，且不得超出实际列数 %d。', nCols);
            end
            raw_time = data(:, cols(1));
            raw_tau  = data(:, cols(2:4));
        elseif nCols == 4
            raw_time = data(:, 1);
            raw_tau  = data(:, 2:4);
        else
            error('compare_fluent:InvalidColumnCount', ...
                '无表头监控数据必须恰好为 4 列 [time, tau1, tau2, tau3]，当前为 %d 列，拒绝静默补零或错列猜测。', nCols);
        end
    end

    % 数值有限性检验
    if any(~isfinite(raw_time)) || any(~isfinite(raw_tau(:)))
        error('compare_fluent:NonFiniteData', 'CFD 监控数据包含 NaN 或 Inf 非有限数值。');
    end

    if isempty(raw_time)
        error('compare_fluent:EmptyData', 'CFD 监控数据为空。');
    end

    % 严格检查时间乱序 (拒绝时间倒退)
    diff_t = diff(raw_time);
    if any(diff_t < -1e-12)
        error('compare_fluent:TimeOutOfOrder', 'CFD 监控数据存在时间乱序或倒退，拒绝处理。');
    end

    % 检查重复时间点：相同力矩合并去重，冲突力矩明确拒绝
    timeTol = 1e-9;
    tauTol = 1e-5; % 力矩容差 [N*m]
    nPoints = numel(raw_time);
    keepMask = true(nPoints, 1);
    
    i = 1;
    while i <= nPoints
        j = i + 1;
        while j <= nPoints && abs(raw_time(j) - raw_time(i)) <= timeTol
            tauDiff = abs(raw_tau(j, :) - raw_tau(i, :));
            if max(tauDiff) > tauTol
                error('compare_fluent:ConflictingDuplicateTime', ...
                    '时间 t = %.6f s 处检测到冲突的力矩数据 (行 %d 与行 %d 最大偏差 %.3e N*m > 容差 %.3e N*m)，拒绝处理。', ...
                    raw_time(i), i, j, max(tauDiff), tauTol);
            end
            keepMask(j) = false; % 相同时间且力矩一致，合并去重
            j = j + 1;
        end
        i = j;
    end

    cfd.time = raw_time(keepMask);
    cfd.tau  = raw_tau(keepMask, :);
end

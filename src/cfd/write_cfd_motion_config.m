function result = write_cfd_motion_config(cfg)
%WRITE_CFD_MOTION_CONFIG Validate a case and generate the UDF parameter header.
%   result = write_cfd_motion_config()
%   result = write_cfd_motion_config(cfg)
%
% The generated header is the only file consumed by the C UDF. The MATLAB
% structure is also saved as a JSON/MAT snapshot under data/cfd_cases/<caseId>.

if nargin < 1 || isempty(cfg)
    cfg = cfd_case_parameters();
end
validate_cfd_case_config(cfg);

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
headerPath = fullfile(rootDir, 'src', 'cfd', 'arm_motion_config.h');
caseDir = fullfile(rootDir, 'data', 'cfd_cases', cfg.caseId);
if ~isfolder(caseDir)
    mkdir(caseDir);
end

write_header(headerPath, cfg);
jsonPath = fullfile(caseDir, 'parameters.json');
matPath = fullfile(caseDir, 'parameters.mat');
summaryPath = fullfile(caseDir, 'summary.txt');

jsonText = jsonencode(cfg);
write_text_file(jsonPath, jsonText);
save(matPath, 'cfg');
write_summary(summaryPath, cfg, headerPath);

result = struct('config', cfg, 'headerPath', headerPath, ...
    'caseDirectory', caseDir, 'jsonPath', jsonPath, ...
    'matPath', matPath, 'summaryPath', summaryPath);

fprintf('CFD 参数已验证并写入:\n');
fprintf('  UDF 参数头: %s\n', headerPath);
fprintf('  工况快照:   %s\n', caseDir);
fprintf('  总时长: %.6g s, 时间步长: %.6g s, 完整步数: %d\n', ...
    cfg.solver.totalTimeS, cfg.solver.timeStepS, cfg.solver.fullSteps);
fprintf('  提醒: 参数变化后必须重新编译/加载 Fluent UDF。\n');
end

function validate_cfd_case_config(cfg)
requiredTop = {'caseId','motion','solver','environment','geometry','output'};
for k = 1:numel(requiredTop)
    if ~isfield(cfg, requiredTop{k})
        error('cfd:ConfigField', '缺少配置字段: %s', requiredTop{k});
    end
end
if ~(ischar(cfg.caseId) || (isstring(cfg.caseId) && isscalar(cfg.caseId)))
    error('cfd:CaseId', 'caseId 必须是字符或标量字符串。');
end
cfg.caseId = char(cfg.caseId);
if isempty(regexp(cfg.caseId, '^[A-Za-z0-9_-]+$', 'once'))
    error('cfd:CaseId', 'caseId 只能包含字母、数字、下划线和短横线。');
end

m = cfg.motion;
assert_vector(m.qStowDeg, 3, 'motion.qStowDeg');
assert_vector(m.qWorkDeg, 3, 'motion.qWorkDeg');
assert_vector(m.durationsS, 3, 'motion.durationsS');
assert_scalar_positive(m.durationsS(1), 'motion.durationsS(1)');
assert_scalar_positive(m.durationsS(2), 'motion.durationsS(2)');
assert_scalar_positive(m.durationsS(3), 'motion.durationsS(3)');
assert_scalar_nonnegative(m.q1SweepHalfDeg, 'motion.q1SweepHalfDeg');
if ~isfinite(m.q1SweepDirection) || ~ismember(m.q1SweepDirection, [-1 1])
    error('cfd:MotionDirection', 'motion.q1SweepDirection 必须为 +1 或 -1。');
end

s = cfg.solver;
assert_scalar_positive(s.timeStepS, 'solver.timeStepS');
if ~isscalar(s.previewSteps) || ~isfinite(s.previewSteps) || ...
        s.previewSteps < 1 || s.previewSteps ~= round(s.previewSteps)
    error('cfd:PreviewSteps', 'solver.previewSteps 必须是正整数。');
end
expectedTotal = sum(m.durationsS);
if ~isfield(s, 'totalTimeS') || abs(s.totalTimeS - expectedTotal) > 1e-12
    error('cfd:TotalTime', 'solver.totalTimeS 必须等于三段时长之和。');
end
expectedSteps = round(expectedTotal / s.timeStepS);
if ~isfield(s, 'fullSteps') || s.fullSteps ~= expectedSteps
    error('cfd:FullSteps', 'solver.fullSteps 必须等于总时长/时间步长的四舍五入值。');
end

fields = {'densityKgM3','gravityMS2','dynamicViscosityPaS', ...
    'currentSpeedMPS','currentAzimuthDeg','currentElevationDeg'};
for k = 1:numel(fields)
    value = cfg.environment.(fields{k});
    if ~isscalar(value) || ~isfinite(value)
        error('cfd:Environment', 'environment.%s 必须是有限标量。', fields{k});
    end
end
if cfg.environment.densityKgM3 <= 0 || cfg.environment.gravityMS2 <= 0 || ...
        cfg.environment.dynamicViscosityPaS <= 0 || cfg.environment.currentSpeedMPS < 0
    error('cfd:EnvironmentRange', '流体密度、重力、黏度必须为正，来流速度不能为负。');
end

assert_vector(cfg.geometry.r12M, 3, 'geometry.r12M');
assert_vector(cfg.geometry.r23M, 3, 'geometry.r23M');
if ~isequal(size(cfg.geometry.jointCentersM), [3 3]) || ...
        any(~isfinite(cfg.geometry.jointCentersM), 'all')
    error('cfd:JointCenters', 'geometry.jointCentersM 必须为有限的 3x3 矩阵。');
end
if ~isequal(size(cfg.geometry.jointAxesInitial), [3 3]) || ...
        any(~isfinite(cfg.geometry.jointAxesInitial), 'all')
    error('cfd:JointAxes', 'geometry.jointAxesInitial 必须为有限的 3x3 矩阵。');
end
fixedR12 = [0.01669012, 0.05050000, -0.03000000];
fixedR23 = [-0.19800000, 0.05600000, 0.00189012];
fixedCenters = [-0.745634, 1.155872, -1.142240; ...
    -0.728944, 1.206372, -1.172240; ...
    -0.730834, 1.008372, -1.228240];
if max(abs(cfg.geometry.r12M - fixedR12), [], 'all') > 1e-12 || ...
        max(abs(cfg.geometry.r23M - fixedR23), [], 'all') > 1e-12 || ...
        max(abs(cfg.geometry.jointCentersM - fixedCenters), [], 'all') > 1e-12
    error('cfd:GeometryLocked', ...
        '几何轴心和 R12/R23 是已验证固定参数，不能在普通工况配置中修改。');
end
end

function assert_vector(value, n, name)
if ~isnumeric(value) || ~isequal(size(value), [1 n]) || any(~isfinite(value))
    error('cfd:ConfigVector', '%s 必须是有限的 1x%d 数值向量。', name, n);
end
end

function assert_scalar_positive(value, name)
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('cfd:ConfigPositive', '%s 必须是正的有限标量。', name);
end
end

function assert_scalar_nonnegative(value, name)
if ~isscalar(value) || ~isfinite(value) || value < 0
    error('cfd:ConfigNonnegative', '%s 必须是非负的有限标量。', name);
end
end

function write_header(path, cfg)
fid = fopen(path, 'wt');
if fid < 0
    error('cfd:HeaderWrite', '无法写入参数头文件: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
q0 = cfg.motion.qStowDeg;
qw = cfg.motion.qWorkDeg;
d = cfg.motion.durationsS;
r12 = cfg.geometry.r12M;
r23 = cfg.geometry.r23M;
c = cfg.geometry.jointCentersM;

fprintf(fid, '#ifndef ARM_MOTION_CONFIG_H\n#define ARM_MOTION_CONFIG_H\n\n');
fprintf(fid, '/* Generated by write_cfd_motion_config.m. Edit cfd_case_parameters.m instead. */\n');
fprintf(fid, '#define ARM_MOTION_PI 3.14159265358979323846\n\n');
fprintf(fid, '/* User-editable motion parameters. */\n');
fprintf(fid, '#define ARM_CFG_T_DEPLOY_S             %.17g\n', d(1));
fprintf(fid, '#define ARM_CFG_T_WORK_S               %.17g\n', d(2));
fprintf(fid, '#define ARM_CFG_T_RETRACT_S            %.17g\n', d(3));
fprintf(fid, '#define ARM_CFG_Q1_STOW_DEG            %.17g\n', q0(1));
fprintf(fid, '#define ARM_CFG_Q2_STOW_DEG            %.17g\n', q0(2));
fprintf(fid, '#define ARM_CFG_Q3_STOW_DEG            %.17g\n', q0(3));
fprintf(fid, '#define ARM_CFG_Q1_WORK_DEG            %.17g\n', qw(1));
fprintf(fid, '#define ARM_CFG_Q2_WORK_DEG            %.17g\n', qw(2));
fprintf(fid, '#define ARM_CFG_Q3_WORK_DEG            %.17g\n', qw(3));
fprintf(fid, '#define ARM_CFG_Q1_SWEEP_HALF_DEG      %.17g\n', cfg.motion.q1SweepHalfDeg);
fprintf(fid, '#define ARM_CFG_Q1_SWEEP_DIRECTION     %.17g\n', cfg.motion.q1SweepDirection);
fprintf(fid, '#define ARM_CFG_TIME_STEP_S            %.17g\n', cfg.solver.timeStepS);
fprintf(fid, '#define ARM_CFG_PREVIEW_STEPS          %d\n\n', cfg.solver.previewSteps);
fprintf(fid, '/* Case metadata. */\n');
fprintf(fid, '#define ARM_CASE_DENSITY_KG_M3         %.17g\n', cfg.environment.densityKgM3);
fprintf(fid, '#define ARM_CASE_GRAVITY_M_S2          %.17g\n', cfg.environment.gravityMS2);
fprintf(fid, '#define ARM_CASE_CURRENT_SPEED_MPS     %.17g\n', cfg.environment.currentSpeedMPS);
fprintf(fid, '#define ARM_CASE_CURRENT_AZIMUTH_DEG   %.17g\n', cfg.environment.currentAzimuthDeg);
fprintf(fid, '#define ARM_CASE_CURRENT_ELEVATION_DEG %.17g\n\n', cfg.environment.currentElevationDeg);
fprintf(fid, '/* Fixed validated geometry. */\n');
fprintf(fid, '#define ARM_FIXED_R12_X_M              %.17g\n', r12(1));
fprintf(fid, '#define ARM_FIXED_R12_Y_M              %.17g\n', r12(2));
fprintf(fid, '#define ARM_FIXED_R12_Z_M              %.17g\n', r12(3));
fprintf(fid, '#define ARM_FIXED_R23_X_M              %.17g\n', r23(1));
fprintf(fid, '#define ARM_FIXED_R23_Y_M              %.17g\n', r23(2));
fprintf(fid, '#define ARM_FIXED_R23_Z_M              %.17g\n', r23(3));
for j = 1:3
    fprintf(fid, '#define ARM_FIXED_JOINT%d_CX_M          %.17g\n', j, c(j,1));
    fprintf(fid, '#define ARM_FIXED_JOINT%d_CY_M          %.17g\n', j, c(j,2));
    fprintf(fid, '#define ARM_FIXED_JOINT%d_CZ_M          %.17g\n', j, c(j,3));
end
fprintf(fid, '\n#define ARM_CFG_DEG_TO_RAD             (ARM_MOTION_PI / 180.0)\n');
fprintf(fid, '#define ARM_CFG_T_TOTAL_S              (ARM_CFG_T_DEPLOY_S + ARM_CFG_T_WORK_S + ARM_CFG_T_RETRACT_S)\n');
fprintf(fid, '#define ARM_CFG_OMEGA_B_RAD_S          (2.0 * ARM_MOTION_PI / ARM_CFG_T_WORK_S)\n');
fprintf(fid, '#define ARM_CFG_Q1_STOW_RAD            (ARM_CFG_Q1_STOW_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q2_STOW_RAD            (ARM_CFG_Q2_STOW_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q3_STOW_RAD            (ARM_CFG_Q3_STOW_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q1_WORK_RAD            (ARM_CFG_Q1_WORK_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q2_WORK_RAD            (ARM_CFG_Q2_WORK_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q3_WORK_RAD            (ARM_CFG_Q3_WORK_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '#define ARM_CFG_Q1_SWEEP_HALF_RAD      (ARM_CFG_Q1_SWEEP_HALF_DEG * ARM_CFG_DEG_TO_RAD)\n');
fprintf(fid, '\n#endif /* ARM_MOTION_CONFIG_H */\n');
end

function write_summary(path, cfg, headerPath)
fid = fopen(path, 'wt');
if fid < 0
    error('cfd:SummaryWrite', '无法写入工况摘要: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'case_id=%s\n', cfg.caseId);
fprintf(fid, 'header=%s\n', headerPath);
fprintf(fid, 'q_stow_deg=[%.12g %.12g %.12g]\n', cfg.motion.qStowDeg);
fprintf(fid, 'q_work_deg=[%.12g %.12g %.12g]\n', cfg.motion.qWorkDeg);
fprintf(fid, 'durations_s=[%.12g %.12g %.12g]\n', cfg.motion.durationsS);
fprintf(fid, 'q1_sweep_half_deg=%.12g\n', cfg.motion.q1SweepHalfDeg);
fprintf(fid, 'q1_sweep_direction=%.12g\n', cfg.motion.q1SweepDirection);
fprintf(fid, 'time_step_s=%.12g\n', cfg.solver.timeStepS);
fprintf(fid, 'total_time_s=%.12g\n', cfg.solver.totalTimeS);
fprintf(fid, 'full_steps=%d\n', cfg.solver.fullSteps);
fprintf(fid, 'density_kg_m3=%.12g\n', cfg.environment.densityKgM3);
fprintf(fid, 'current_speed_mps=%.12g\n', cfg.environment.currentSpeedMPS);
fprintf(fid, 'current_azimuth_deg=%.12g\n', cfg.environment.currentAzimuthDeg);
fprintf(fid, 'current_elevation_deg=%.12g\n', cfg.environment.currentElevationDeg);
fprintf(fid, 'sign_convention=%s\n', cfg.output.signConvention);
end

function write_text_file(path, text)
fid = fopen(path, 'wt');
if fid < 0
    error('cfd:TextWrite', '无法写入文件: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, text, 'char');
end

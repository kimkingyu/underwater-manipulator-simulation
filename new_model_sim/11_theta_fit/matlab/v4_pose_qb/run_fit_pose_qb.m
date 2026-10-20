function run_fit_pose_qb()
%RUN_FIT_POSE_QB Reproduce q0 regression, pose refit, checks, and comparison report.
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);
backup = fullfile(root, 'v3_cop87_q0');
files = {'theta_fit.mat', 'theta_fit.json', 'theta_table.csv', 'loo_errors.csv'};
for i = 1:numel(files)
    assert(isfile(fullfile(backup, files{i})), 'Missing q0 backup: %s', files{i});
    assert(isequal(read_bytes(fullfile(root, files{i})), read_bytes(fullfile(backup, files{i}))), ...
        'Root differs from backup before test: %s', files{i});
end
old = load(fullfile(backup, 'theta_fit.mat'));
restore_guard = onCleanup(@() restore_files(root, backup, files));
% Exercise the original zero-argument call, which writes to the root directory.
reg = fit_theta();
regression = struct();
regression.max_theta_absolute = max(abs(reg.theta - old.theta));
regression.max_theta_relative = max(abs(reg.theta - old.theta) ./ max(abs(old.theta), 1));
regression.max_fit_error_delta = max(abs(reg.fit_rel_err(:) - old.fit_rel_err(:)));
regression.max_loo_error_delta = max(abs(reg.loo_rel_err(:) - old.loo_rel_err(:)));
regression.cond_scaled_delta = abs(reg.cond_scaled - old.cond_scaled);
assert(regression.max_theta_relative < 1e-9, 'q0 theta regression failed');
assert(regression.max_fit_error_delta < 1e-12 && regression.max_loo_error_delta < 1e-12, ...
    'q0 error regression failed');
clear restore_guard;
regression.root_restored_byte_exact = true;
for i = 1:numel(files)
    assert(isequal(read_bytes(fullfile(root, files{i})), read_bytes(fullfile(backup, files{i}))), ...
        'Root not restored: %s', files{i});
end
base_file = fullfile(root, '..', 'theta_dataset.json');
pose_file = fullfile(root, '..', 'theta_dataset_qb.json');
new = fit_theta(base_file, here, [], [], pose_file);
assert(numel(old.case_ids) == 22 && numel(new.case_ids) == 25);
assert(all(strcmp(new.case_ids(end-2:end), {'T_px_08@qb'; 'T_py_08@qb'; 'T_pz_08@qb'})));
assert(isequal(new.fixed, old.fixed), 'Default constraints changed');
P = jsondecode(fileread(pose_file));
assert(strcmp(P.cases(3).status, 'failed'), 'Expected failed qb case was not retained');
% Two pose files, string-array input, a per-case q override, and forced zero nudot.
% Keep identical case order/loads so every saved fit and LOO value must agree.
tmp = tempname; mkdir(tmp);
tmp_guard = onCleanup(@() rmdir(tmp, 's'));
P1 = struct('q', P.q, 'cases', P.cases(1));
P1.cases.nudot = ones(6, 1); % Must be ignored for an appended steady dataset.
P2 = struct('q', [9; 9; 9], 'cases', P.cases(2:3));
for i = 1:2, P2.cases(i).q = P.q; end
p1 = fullfile(tmp, 'pose1.json'); p2 = fullfile(tmp, 'pose2.json');
write_json(p1, P1); write_json(p2, P2);
check = fit_theta(base_file, tmp, [], [], [string(p1), string(p2)]);
regression.multi_pose_theta_delta = max(abs(check.theta - new.theta));
regression.multi_pose_loo_delta = max(abs(check.loo_rel_err(:) - new.loo_rel_err(:)));
assert(regression.multi_pose_theta_delta < 1e-12 && regression.multi_pose_loo_delta < 1e-12);
assert(isequal(check.case_ids, new.case_ids));
% A base dataset with per-case poses and no dataset-level q must be equivalent.
B = jsondecode(fileread(base_file)); C = B.cases;
if isstruct(C), C = num2cell(C); end
C = C(:);
for i = 1:numel(C), C{i}.q = zeros(3, 1); end
for i = 1:numel(P.cases)
    k = P.cases(i); k.q = P.q; k.nudot = zeros(size(k.nu)); k.id = [k.id '@qb'];
    C{end + 1, 1} = k; %#ok<AGROW>
end
combined = fullfile(tmp, 'combined.json'); write_json(combined, struct('cases', {C}));
check = fit_theta(combined, tmp);
regression.per_case_pose_theta_delta = max(abs(check.theta - new.theta));
regression.per_case_pose_loo_delta = max(abs(check.loo_rel_err(:) - new.loo_rel_err(:)));
assert(regression.per_case_pose_theta_delta < 1e-12 && regression.per_case_pose_loo_delta < 1e-12);
clear tmp_guard;
regression.passed = true;
write_json(fullfile(here, 'regression.json'), regression);

% Parameter tables. Largest change means descending absolute numerical delta;
% units differ, so this ranking is not a physical importance ranking.
names = old.names(:); units = old.units(:);
delta = new.theta - old.theta;
pct = 100 * delta ./ abs(old.theta); pct(abs(old.theta) < 1e-12) = NaN;
params = table((1:numel(names))', names, units, old.theta, new.theta, delta, pct, ...
    'VariableNames', {'index', 'name', 'unit', 'old', 'new', 'delta', 'change_percent'});
writetable(params, fullfile(here, 'parameter_comparison.csv'));
main_idx = [1:9, 13:15, 19:36];
[~, order] = sort(abs(delta), 'descend'); top = order(1:10);
writetable(params(top, :), fullfile(here, 'top10_parameter_changes.csv'));

% Global NRMS is RMS over cases, consistent with equal CASE weight in fitting.
gn = {'F'; 'M'; 'tau'; 'link'; 'linkM'};
a = sqrt(mean(old.fit_rel_err(:, 6:10).^2, 1))';
b = sqrt(mean(new.fit_rel_err(:, 6:10).^2, 1))';
c = sqrt(mean(old.loo_rel_err(:, 6:10).^2, 1))';
d = sqrt(mean(new.loo_rel_err(:, 6:10).^2, 1))';
metrics = table(gn, a, b, c, d, 'VariableNames', ...
    {'group', 'old22_fit_nrms', 'new25_fit_nrms', 'old22_loo_nrms', 'new25_loo_nrms'});
writetable(metrics, fullfile(here, 'group_comparison.csv'));

% Fair shared scale for all25: reconstruct old frozen-theta qb predictions,
% while reading the historical eval CSV verbatim for the requested comparison.
E = readtable(fullfile(root, 'eval_theta_qb.csv'), 'TextType', 'string', 'VariableNamingRule', 'preserve');
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
new_to_old = zeros(1, 5); old_qb = zeros(3, 5);
for g = 1:5
    r = find(grp == g, 1);
    new_to_old(g) = new.channel_scale(r) / old.channel_scale(r);
    for i = 1:3
        row = E.case == string(P.cases(i).id) & E.group == string(gn{g});
        assert(nnz(row) == 1);
        old_qb(i, g) = E.qb_nrms(row);
    end
end
G = uvms_geometry();
old_qb_check = zeros(3, 5);
for i = 1:3
    k = P.cases(i);
    [Yg, Yl] = Y_uvms(P.q(:), [k.nu(:); zeros(3, 1)], zeros(9, 1), G);
    e = -[Yg; Yl] * old.theta - [k.F(:); k.M(:); k.tau(:); k.flink(:)];
    for g = 1:5
        idx = grp == g;
        old_qb_check(i, g) = sqrt(mean(e(idx).^2)) / old.channel_scale(find(idx, 1));
    end
end
assert(max(abs(old_qb_check(:) - old_qb(:))) < 1e-12, 'Historical qb CSV mismatch');
new_fit_oldscale = new.fit_rel_err(:, 6:10) .* new_to_old;
new_loo_oldscale = new.loo_rel_err(:, 6:10) .* new_to_old;
frozen25 = sqrt(mean([old.fit_rel_err(:, 6:10); old_qb].^2, 1))';
newfit25 = sqrt(mean(new_fit_oldscale.^2, 1))';
% Old comparator is old22 LOCO plus three unseen-pose predictions, NOT all25 LOCO.
oldheld25 = sqrt(mean([old.loo_rel_err(:, 6:10); old_qb].^2, 1))';
newloo25 = sqrt(mean(new_loo_oldscale.^2, 1))';
common = table(gn, frozen25, newfit25, oldheld25, newloo25, 'VariableNames', ...
    {'group', 'old_frozen25_residual', 'new25_fit', 'old22_loo_plus_unseen_qb', 'new25_loo'});
writetable(common, fullfile(here, 'common_scale_all25.csv'));
qbrows = cell(15, 8); row = 0;
for i = 1:3
    for g = 1:5
        row = row + 1;
        qbrows(row, :) = {new.case_ids{22+i}, gn{g}, P.cases(i).status, old_qb(i,g), ...
            new.loo_rel_err(22+i, 5+g), new_loo_oldscale(22+i,g), ...
            new.loo_rel_err(22+i, g), 100*(new_loo_oldscale(22+i,g)/old_qb(i,g)-1)};
    end
end
qb = cell2table(qbrows, 'VariableNames', {'case', 'group', 'status', 'old_qb_nrms', ...
    'new_loo_nrms_native', 'new_loo_nrms_oldscale', 'new_loo_rel', 'change_percent_oldscale'});
writetable(qb, fullfile(here, 'qb_loo_comparison.csv'));
cond_table = table(old.cond_scaled, new.cond_scaled, new.cond_scaled/old.cond_scaled, ...
    nnz(~old.fixed), nnz(old.excited), nnz(new.excited), 'VariableNames', ...
    {'old_cond_scaled', 'new_cond_scaled', 'new_over_old', 'free_parameters', 'old_excited', 'new_excited'});
writetable(cond_table, fullfile(here, 'condition_comparison.csv'));

fid = fopen(fullfile(here, 'summary.md'), 'w', 'n', 'UTF-8');
assert(fid >= 0); close_guard = onCleanup(@() fclose(fid));
fprintf(fid, '# v4_pose_qb 拟合结果\n\n');
fprintf(fid, '- 旧版：q=0 的 22 工况；新版：22+3=25 工况，87 维参数。coupling=lin、link_cop=true，约束未改变。\n');
fprintf(fid, '- qb 姿态：[0.45, -0.50, -0.40] rad。T_pz_08@qb 的 status=failed，仍纳入，未降权。其数据为迭代上限后尾段 200 步均值。\n');
fprintf(fid, '- 保留既有“各工况等权、组内按数据集 RMS 缩放”算法；留一是留一个工况，qb 留一仍包含同名 q0 工况以及另两个 qb 工况，不是留整种姿态。\n');
fprintf(fid, '- 汇总 NRMS = sqrt(mean(各工况 nrms.^2))，不是各工况 nrms 的算术平均。\n\n');
fprintf(fid, '## 回归测试\n\n');
fprintf(fid, '- 原零参数调用：max|Δθ|=%.17g；max(|Δθ|/max(1,|θ旧|))=%.17g（阈值 1e-9）。\n', ...
    regression.max_theta_absolute, regression.max_theta_relative);
fprintf(fid, '- 拟合误差最大差=%.17g；留一误差最大差=%.17g；缩放条件数差=%.17g。\n', ...
    regression.max_fit_error_delta, regression.max_loo_error_delta, regression.cond_scaled_delta);
fprintf(fid, '- 四个根目录基准文件已逐字节还原；单文件姿态、多个姿态文件、逐工况 q 覆盖、额外集强制零加速度均通过等价性断言。\n\n');
fprintf(fid, '## 主要参数\n\n'); print_parameters(fid, params(main_idx, :));
fprintf(fid, '\n## 变化最大的十个参数\n\n按 |Δθ| 数值降序；参数单位不同，不能据此判断物理重要性。旧值近零时相对变化不定义，记为 —。\n\n');
print_parameters(fid, params(top, :));
fprintf(fid, '\n## 全数据集拟合与留一（各自数据集原生尺度）\n\n旧版 22 工况与新版 25 工况的数据组成、RMS 分母均不同，不能把这张表直接理解为同测试集改进幅度。\n\n');
fprintf(fid, '| 组 | 旧22 拟合 | 新25 拟合 | 旧22 留一 | 新25 留一 |\n|---|---:|---:|---:|---:|\n');
for g=1:5, fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f |\n', gn{g}, a(g), b(g), c(g), d(g)); end
fprintf(fid, '\n## 同25工况、同 q0 RMS 分母的补充比较\n\n旧 θ 全程不重拟；“旧留一+未见qb”为原22工况留一误差与3个 qb 直接预测误差合并，不是重新训练的25工况留一。\n\n');
fprintf(fid, '| 组 | 旧θ 残差/预测 | 新θ 拟合 | 旧22留一+未见qb | 新25 留一 |\n|---|---:|---:|---:|---:|\n');
for g=1:5, fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f |\n', gn{g}, frozen25(g), newfit25(g), oldheld25(g), newloo25(g)); end
fprintf(fid, '\n## qb 工况留一\n\n旧预测读取 eval_theta_qb.csv 的 qb_nrms，并已用旧 θ 重新计算验证一致。公平比较使用“新留一/q0尺度”列；原生尺度列使用25工况组RMS。负变化表示改善。\n\n');
fprintf(fid, '| 工况 | 组 | 旧直接预测 | 新留一/原生尺度 | 新留一/q0尺度 | 变化%% |\n|---|---|---:|---:|---:|---:|\n');
for i = 1:height(qb)
    fprintf(fid, '| %s | %s | %.6f | %.6f | %.6f | %+.2f |\n', ...
        qb.case{i}, qb.group{i}, qb.old_qb_nrms(i), qb.new_loo_nrms_native(i), ...
        qb.new_loo_nrms_oldscale(i), qb.change_percent_oldscale(i));
end
fprintf(fid, '\n## 缩放后的条件数\n\n');
fprintf(fid, '| 旧22 | 新25 | 新/旧 | 自由参数 | 旧/新激励参数 |\n|---:|---:|---:|---:|---:|\n');
fprintf(fid, '| %.9g | %.9g | %.6f | %d | %d / %d |\n', old.cond_scaled, new.cond_scaled, ...
    new.cond_scaled/old.cond_scaled, nnz(~old.fixed), nnz(old.excited), nnz(new.excited));
fprintf(fid, '\n条件数保持 fit_theta 原定义：固定列剔除，组权与工况权缩放后再将各列归一，σmax/max(σmin,eps)。\n');
fprintf(fid, '\n## 输出与复现\n\n运行 `addpath(''v4_pose_qb''); run_fit_pose_qb`。新模型为本目录 theta_fit.mat/json；其余 CSV 记录完整参数、汇总和逐工况误差。根目录四个原模型输出保持备份字节不变。\n');
clear close_guard;
disp(regression); disp(metrics); disp(common); disp(cond_table); disp(qb);
end

function restore_files(root, backup, files)
for i = 1:numel(files), copyfile(fullfile(backup, files{i}), fullfile(root, files{i}), 'f'); end
end

function b = read_bytes(path)
fid = fopen(path, 'rb'); assert(fid >= 0); guard = onCleanup(@() fclose(fid));
b = fread(fid, inf, '*uint8');
end

function write_json(path, value)
fid = fopen(path, 'w'); assert(fid >= 0); guard = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(value, 'PrettyPrint', true));
end

function print_parameters(fid, T)
fprintf(fid, '| 参数 | 单位 | 旧θ | 新θ | Δθ | 变化%% |\n|---|---|---:|---:|---:|---:|\n');
for i = 1:height(T)
    if isnan(T.change_percent(i)), pct = '—'; else, pct = sprintf('%+.2f', T.change_percent(i)); end
    fprintf(fid, '| %s | %s | %.9g | %.9g | %+.9g | %s |\n', ...
        T.name{i}, T.unit{i}, T.old(i), T.new(i), T.delta(i), pct);
end
end

%% test_smooth_cycle_pipeline.m
% 校验三段式轨迹的解析导数自洽性与段边界平滑性。
%
% 三段拼接的关键风险是段与段之间出现速度/加速度跃变，导致 CFD 动网格在拼接时刻
% 遭遇冲击。本脚本从 calc_joint_loads.m 的输出中独立复核这一点。

currentScript = mfilename('fullpath');
rootDir  = fileparts(fileparts(fileparts(currentScript)));
dataFile = fullfile(rootDir, 'data', 'joint_loads_data.mat');

if ~isfile(dataFile)
    fprintf('未找到轨迹数据，正在运行 calc_joint_loads.m 生成...\n');
    run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));
end

d = load(dataFile);
time = d.time;
q_seq = d.q_seq;
qd_seq = d.qd_seq;
qdd_seq = d.qdd_seq;
traj = d.traj;
N = numel(time);
dt = time(2) - time(1);

fprintf('=== 三段式轨迹平滑性与自洽性检验 ===\n');
fprintf('  %s\n\n', traj.form);

% 1. 端点静止
fprintf('  【端点静止】\n');
fprintf('    t = 0     角速度:  %.3e deg/s\n', norm(rad2deg(qd_seq(1,:))));
fprintf('    t = %4.1fs 角速度:  %.3e deg/s\n', time(end), norm(rad2deg(qd_seq(end,:))));
fprintf('    首末位形偏差:      %.3e deg\n\n', norm(rad2deg(q_seq(end,:) - q_seq(1,:))));

% 2. 段边界平滑性 (三段拼接的核心风险点)
k1 = round(traj.T_deploy_s / dt) + 1;
k2 = round((traj.T_deploy_s + traj.T_work_s) / dt) + 1;
fprintf('  【段边界平滑性】\n');
bname = {'伸展 -> 往复', '往复 -> 收回'};
for i = 1:2
    if i == 1, kb = k1; tb = traj.T_deploy_s;
    else,      kb = k2; tb = traj.T_deploy_s + traj.T_work_s; end
    % 跨边界取前后各一帧，检验位置与速度是否连续过渡
    dq_jump  = norm(rad2deg(q_seq(kb+1,:)  - 2*q_seq(kb,:)  + q_seq(kb-1,:)));
    fprintf('    t = %4.1fs (%s): 速度 %.3e deg/s | 位置二阶差分 %.3e deg\n', ...
        tb, bname{i}, norm(rad2deg(qd_seq(kb,:))), dq_jump);
end
fprintf('    (段边界速度须为零，否则动网格在拼接时刻遭遇冲击)\n\n');

% 3. 解析导数与中心差分交叉校验
% 段边界处解析式切换，中心差分跨越边界会引入伪误差，故剔除边界邻域
qd_num  = zeros(N, 3);
qdd_num = zeros(N, 3);
for j = 1:3
    qd_num(:, j)  = gradient(q_seq(:, j), dt);
    qdd_num(:, j) = gradient(qd_num(:, j), dt);
end
mask = true(N, 1);
mask([1:3, N-2:N]) = false;
mask(max(1,k1-3):min(N,k1+3)) = false;
mask(max(1,k2-3):min(N,k2+3)) = false;

err_qd  = max(max(abs(qd_seq(mask,:)  - qd_num(mask,:))));
err_qdd = max(max(abs(qdd_seq(mask,:) - qdd_num(mask,:))));
fprintf('  【解析导数自洽性 (剔除段边界邻域后与中心差分对比)】\n');
fprintf('    解析 qd  与数值差分最大偏差:  %.3e rad/s\n', err_qd);
fprintf('    解析 qdd 与数值差分最大偏差:  %.3e rad/s^2\n\n', err_qdd);

% 4. 行程汇总
fprintf('  【各关节行程】\n');
for j = 1:3
    fprintf('    关节 %d: [%6.1f, %6.1f] deg (跨度 %.1f°, 峰值 %.1f deg/s)\n', ...
        j, min(rad2deg(q_seq(:,j))), max(rad2deg(q_seq(:,j))), ...
        range(rad2deg(q_seq(:,j))), max(abs(rad2deg(qd_seq(:,j)))));
end

%% test_rigorous_pipeline.m
% 三段式轨迹的动力学严密性交叉校验。
%
% 另外两个脚本已分别覆盖运动学/碰撞 (verify_coordinated_trajectory.m) 与
% 段边界平滑性 (test_smooth_cycle_pipeline.m)。本脚本专注于动力学本身，
% 用与 calc_joint_loads.m 完全独立的途径复算同一批量，逐项比对:
%
%   1. 刚体项独立复算: 用 MATLAB 内置 inverseDynamics 一次性求出
%      M*qdd + C*qd + G，与主脚本手工分解的三项之和对比。
%      主脚本把刚体力矩拆成 massMatrix / velocityProduct / gravityTorque 三块，
%      若拆分方式有误 (漏项、符号反、qd 与 qdd 错配)，此处必然暴露。
%
%   2. 附加质量矩阵物理合法性: M_add(q) 必须对称正定。
%      附加质量是流体动能的二次型系数，非对称或含非正特征值即无物理意义。
%
%   3. 附加质量科氏项的反对称性: dM_add/dt - 2*C_add 必须反对称。
%      这是拉格朗日方程的能量一致性条件，等价于附加质量动能不被凭空创造或销毁。
%      主脚本用 Christoffel 符号构造 C_add，此处独立数值验证该性质。
%
%   4. 力矩合成闭合性: 各分量之和必须严格等于 tau_total。

currentScript = mfilename('fullpath');
rootDir  = fileparts(fileparts(fileparts(currentScript)));
urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
dataFile = fullfile(rootDir, 'data', 'joint_loads_data.mat');

if ~isfile(dataFile)
    fprintf('未找到负载数据，正在运行 calc_joint_loads.m 生成...\n');
    run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));
end

d = load(dataFile);
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -d.env.g];

q_seq   = d.q_seq;
qd_seq  = d.qd_seq;
qdd_seq = d.qdd_seq;
hydro   = d.hydro;
N       = size(q_seq, 1);

fprintf('=== 三段式轨迹动力学严密性交叉校验 ===\n');
fprintf('数据源: %d 帧, 总时长 %.1f s\n\n', N, d.time(end));

% 为控制耗时，均匀抽取校验帧 (含首末帧与两处段边界)
dt = d.time(2) - d.time(1);
kb = [1, round(d.traj.T_deploy_s/dt)+1, ...
      round((d.traj.T_deploy_s+d.traj.T_work_s)/dt)+1, N];
kchk = unique([round(linspace(1, N, 60)), kb]);

%% 1. 刚体项独立复算 (inverseDynamics vs 手工三项分解)
err_rigid = zeros(numel(kchk), 3);
for i = 1:numel(kchk)
    k = kchk(i);
    % 内置逆动力学: 一次性给出 M*qdd + C*qd + G (无外力)
    tau_ref = inverseDynamics(robot, q_seq(k,:), qd_seq(k,:), qdd_seq(k,:));
    % 主脚本的手工分解: 刚体惯性 + 科氏 + 重力
    % 注意 tau_coriolis 中含附加质量诱导项 C_add*qd，需扣除后才可比
    C_add = eval_added_mass_coriolis(robot, q_seq(k,:), qd_seq(k,:), hydro);
    tau_cor_rigid = d.tau_coriolis(k,:) - (C_add * qd_seq(k,:).').';
    tau_manual = d.tau_inertial_rigid(k,:) + tau_cor_rigid + d.tau_gravity(k,:);
    err_rigid(i,:) = tau_manual - tau_ref;
end
max_err_rigid = max(abs(err_rigid(:)));
fprintf('1. 刚体项独立复算 (内置 inverseDynamics vs 手工分解):\n');
fprintf('   - 校验帧数: %d\n', numel(kchk));
fprintf('   - 最大绝对偏差: %.3e N*m\n', max_err_rigid);
fprintf('   => %s\n\n', verdict(max_err_rigid < 1e-9));

%% 2. 附加质量矩阵对称正定性
min_sym = 0; min_eig = inf;
for i = 1:numel(kchk)
    k = kchk(i);
    M_add = eval_added_mass_matrix(robot, q_seq(k,:), hydro);
    min_sym = max(min_sym, max(max(abs(M_add - M_add.'))));
    min_eig = min(min_eig, min(eig((M_add + M_add.')/2)));
end
fprintf('2. 附加质量矩阵 M_add(q) 物理合法性:\n');
fprintf('   - 最大非对称量 max|M - M^T|: %.3e\n', min_sym);
fprintf('   - 全局最小特征值: %.6e (须 > 0)\n', min_eig);
fprintf('   => %s\n\n', verdict(min_sym < 1e-12 && min_eig > 0));

%% 3. 附加质量科氏项反对称性 (拉格朗日能量一致性)
% 检验 dM_add/dt - 2*C_add 是否反对称。取速度非零的帧才有意义。
h = 1e-6;
max_asym = 0;
n_tested = 0;
for i = 1:numel(kchk)
    k = kchk(i);
    qd_k = qd_seq(k,:);
    if norm(qd_k) < 1e-6
        continue;   % 静止帧该性质退化，跳过
    end
    n_tested = n_tested + 1;
    % dM_add/dt = sum_j (dM_add/dq_j) * qd_j，用中心差分沿实际速度方向求
    Mp = eval_added_mass_matrix(robot, q_seq(k,:) + h*qd_k, hydro);
    Mm = eval_added_mass_matrix(robot, q_seq(k,:) - h*qd_k, hydro);
    dMdt = (Mp - Mm) / (2*h);
    C_add = eval_added_mass_coriolis(robot, q_seq(k,:), qd_k, hydro);
    S = dMdt - 2*C_add;
    % 反对称 <=> S + S^T = 0，用相对量度量
    max_asym = max(max_asym, max(max(abs(S + S.'))) / max(1, max(max(abs(dMdt)))));
end
fprintf('3. 附加质量科氏项反对称性 (dM/dt - 2C 须反对称):\n');
fprintf('   - 有效校验帧数 (速度非零): %d\n', n_tested);
fprintf('   - 最大相对不对称量: %.3e\n', max_asym);
fprintf('   => %s\n\n', verdict(max_asym < 1e-6));

%% 4. 力矩合成闭合性 (全帧)
tau_sum = d.tau_inertial_rigid + d.tau_inertial_added + d.tau_coriolis + ...
          d.tau_gravity + d.tau_buoyancy + d.tau_hydro_drag;
max_close = max(max(abs(tau_sum - d.tau_total)));
fprintf('4. 力矩分量合成闭合性 (全 %d 帧):\n', N);
fprintf('   - max|sum(components) - tau_total|: %.3e N*m\n', max_close);
fprintf('   => %s\n\n', verdict(max_close < 1e-12));

%% 汇总
allpass = (max_err_rigid < 1e-9) && (min_sym < 1e-12) && (min_eig > 0) && ...
          (max_asym < 1e-6) && (max_close < 1e-12);
fprintf('================================================\n');
if allpass
    fprintf('动力学严密性交叉校验: 全部通过\n');
else
    fprintf('动力学严密性交叉校验: 存在未通过项，见上\n');
end
fprintf('================================================\n');

%% --- 支撑函数 (与 calc_joint_loads.m 中的实现保持一致) ---
function s = verdict(ok)
    if ok, s = '通过'; else, s = '未通过'; end
end

function M_add = eval_added_mass_matrix(robot, q, hydro)
    M_add = zeros(3, 3);
    for i = 1:numel(hydro)
        T_body = getTransform(robot, q, hydro(i).name);
        R_body = T_body(1:3, 1:3);
        J_geom = geometricJacobian(robot, q, hydro(i).name);
        r_com_world = R_body * hydro(i).com_local.';
        J_com = J_geom(4:6, :) - skew_mat(r_com_world) * J_geom(1:3, :);
        M_add = M_add + J_com.' * (R_body * hydro(i).added_mass * R_body.') * J_com;
    end
end

function C_add = eval_added_mass_coriolis(robot, q, qd, hydro)
    h = 1e-6;
    dMdq = cell(3, 1);
    for k = 1:3
        dq = zeros(1, 3); dq(k) = h;
        dMdq{k} = (eval_added_mass_matrix(robot, q + dq, hydro) - ...
                   eval_added_mass_matrix(robot, q - dq, hydro)) / (2 * h);
    end
    C_add = zeros(3, 3);
    for i = 1:3
        for j = 1:3
            s = 0;
            for k = 1:3
                s = s + 0.5 * (dMdq{k}(i, j) + dMdq{j}(i, k) - dMdq{i}(j, k)) * qd(k);
            end
            C_add(i, j) = s;
        end
    end
end

function S = skew_mat(v)
    S = [    0, -v(3),  v(2); ...
          v(3),     0, -v(1); ...
         -v(2),  v(1),     0 ];
end

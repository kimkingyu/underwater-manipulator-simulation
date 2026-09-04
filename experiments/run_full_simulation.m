%% run_full_simulation.m
% =========================================================================
% 9-DOF 水下机器人-机械臂系统 (UVMS) 完整综合仿真工程
%
% 包含完整闭环链条:
%   1. 物理环境配置: 海水物性 + 文献级三维洋流场 (0.25 m/s, 45度斜吹)
%   2. 机构物理模型: 基于 URDF 的 9-DOF 耦合刚体树 + 6x6 附加质量 + 水动力非线性阻尼
%   3. 自主任务规划: 逆运动学求解 + 空间最小急动度 (Minimum Jerk) 五次多项式平滑轨迹
%   4. 双层闭环控制: AUV 6-DOF 悬停定点定姿控制 + 机械臂关节力矩控制
%   5. 状态积分求解: 四阶龙格-库塔 (RK4) 高精度前向动力学积分
%   6. 科研全景评估: 自动计算末端精度、基座漂移、能耗力矩，导出数据与高清仪表盘
% =========================================================================

clear; clc; close all;

fprintf('============================================================\n');
fprintf('       9-DOF UVMS 水下机械臂系统 完整综合仿真启动\n');
fprintf('============================================================\n\n');

scriptDir = fileparts(mfilename('fullpath'));
urdfFile = fullfile(scriptDir, 'robot.urdf');

%% 1. 环境与洋流参数设置 (依据 Fossen 2011 & Heshmati-Alamdari 2018)
env = struct();
env.rho_water    = 1025;                    % 海水密度 [kg/m^3]
env.g            = 9.81;                    % 重力加速度 [m/s^2]
env.current_mag  = 0.25;                    % 洋流幅值 [m/s] (典型作业流速)
env.current_psi  = deg2rad(45);             % 来流水平方位角 [rad] (45度斜向)
env.current_vc   = [env.current_mag * cos(env.current_psi); ...
                    env.current_mag * sin(env.current_psi); ...
                    0.0];                   % 世界坐标系洋流矢量 [m/s]

%% 2. 机械臂与 AUV 9-DOF 动力学模型构建
armRobot = importrobot(urdfFile);
armRobot.DataFormat = 'row';
armRobot.Gravity = [0 0 -env.g];

uvms = rigidBodyTree('DataFormat', 'row');
uvms.Gravity = [0 0 -env.g];

% AUV 6-DOF 浮动基座虚拟关节链
jointTypes = {'prismatic', 'prismatic', 'prismatic', 'revolute', 'revolute', 'revolute'};
jointAxes = [1 0 0; 0 1 0; 0 0 1; 1 0 0; 0 1 0; 0 0 1];
parentNames = {uvms.BaseName, 'virtual_x', 'virtual_y', 'virtual_z', 'virtual_roll', 'virtual_pitch'};
bodyNames = {'virtual_x', 'virtual_y', 'virtual_z', 'virtual_roll', 'virtual_pitch'};

for i = 1:5
    b = rigidBody(bodyNames{i});
    j = rigidBodyJoint(['joint_' bodyNames{i}], jointTypes{i});
    j.JointAxis = jointAxes(i, :);
    b.Joint = j;
    b.Mass = 0;
    b.CenterOfMass = [0 0 0];
    b.Inertia = zeros(1, 6);
    addBody(uvms, b, parentNames{i});
end

auvBody = copy(armRobot.Base);
auvBody.Name = 'auv_base';
jAuv = rigidBodyJoint('joint_virtual_yaw', 'revolute');
jAuv.JointAxis = jointAxes(6, :);
auvBody.Joint = jAuv;
addBody(uvms, auvBody, 'virtual_pitch');

for i = 1:armRobot.NumBodies
    srcBody = armRobot.Bodies{i};
    newBody = copy(srcBody);
    parentName = srcBody.Parent.Name;
    if strcmp(parentName, armRobot.BaseName)
        parentName = 'auv_base';
    end
    addBody(uvms, newBody, parentName);
end

% 水动力学参数 (附加质量、阻尼、稳心恢复)
m_auv = auvBody.Mass;
M_added = zeros(9, 9);
M_added(1:3, 1:3) = diag([0.40, 0.40, 0.50] * m_auv);  % AUV 平移附加质量
M_added(4:6, 4:6) = diag([0.30, 0.30, 0.30] * 20.0);   % AUV 转动附加惯量
M_added(7:9, 7:9) = diag([0.05, 0.15, 0.12]);          % 机械臂连杆附加质量

D_linear_auv = [60, 60, 80, 25, 25, 20];               % AUV 线性阻尼
D_quad_auv   = [120, 120, 150, 40, 40, 35];            % AUV 二次流体阻力
D_arm        = [1.5, 2.5, 2.0];                        % 机械臂水下等效阻尼
K_restore    = m_auv * env.g * 0.05;                   % 定倾中心自复原刚度 (GM = 0.05 m)

%% 3. 抓取作业任务与轨迹规划
% 末端夹爪抓取中心偏移 (前述从 4 个 STL 网格实测精确得出)
tcpOffset = [0.280, -0.030, -0.0168];

% 抓取目标位置 (世界坐标系，位于 AUV 腹部斜前方可达空间内)
target_pos = [0.120, 0.400, -0.480]; 

% 初始位姿与目标逆解求解
qArm_0 = [0, 0, 0];
ik = inverseKinematics('RigidBodyTree', armRobot);
ik.SolverParameters.MaxIterations = 200;
ik.SolverParameters.SolutionTolerance = 1e-4;
[q_sol, ~] = ik('link_004', trvec2tform(target_pos), [0 0 0 1 1 1], qArm_0);

optFunc = @(q) norm(get_tcp_pos(armRobot, q, tcpOffset) - target_pos);
qArm_goal = fmincon(optFunc, q_sol, [], [], [], [], ...
    [-1.57, -1.57, -1.57], [1.57, 1.57, 1.57], [], ...
    optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp'));

actual_target_err = norm(get_tcp_pos(armRobot, qArm_goal, tcpOffset) - target_pos);
fprintf('[1/5] 抓取任务与逆解求解完毕:\n');
fprintf('      - 目标抓取坐标: [%.3f, %.3f, %.3f] m\n', target_pos);
fprintf('      - 目标关节角度: [%.3f, %.3f, %.3f] rad (误差: %.2f mm)\n\n', ...
    qArm_goal, actual_target_err * 1000);

% 轨迹时序 (五次多项式最小急动度规划)
tEnd = 10.0;
dt = 0.005;
time = 0:dt:tEnd;
nSteps = numel(time);

qArm_des   = zeros(nSteps, 3);
qdArm_des  = zeros(nSteps, 3);
qddArm_des = zeros(nSteps, 3);

for k = 1:nSteps
    t = time(k);
    if t < 1.0
        qArm_des(k, :) = qArm_0;
    elseif t <= 5.0
        tau = (t - 1.0) / 4.0;
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / 4.0;
        sdd = (60*tau - 180*tau^2 + 120*tau^3) / 16.0;
        qArm_des(k, :) = qArm_0 + s * (qArm_goal - qArm_0);
        qdArm_des(k, :) = sd * (qArm_goal - qArm_0);
        qddArm_des(k, :) = sdd * (qArm_goal - qArm_0);
    elseif t <= 7.0
        qArm_des(k, :) = qArm_goal;
    else
        tau = (t - 7.0) / 3.0;
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / 3.0;
        sdd = (60*tau - 180*tau^2 + 120*tau^3) / 9.0;
        qArm_des(k, :) = qArm_goal + s * (qArm_0 - qArm_goal);
        qdArm_des(k, :) = sd * (qArm_0 - qArm_goal);
        qddArm_des(k, :) = sdd * (qArm_0 - qArm_goal);
    end
end
fprintf('[2/5] 最小急动度高阶平滑抓取轨迹规划完成 (时长: %.1f s, 步长: %.3f s)\n\n', tEnd, dt);

%% 4. 控制器配置
ctrl = struct();
ctrl.Kp_auv_pos = [450, 450, 550];
ctrl.Kd_auv_pos = [220, 220, 260];
ctrl.Kp_auv_att = [280, 280, 220];
ctrl.Ki_auv_att = [60,  60,  50];
ctrl.Kd_auv_att = [90,  90,  70];

ctrl.Kp_arm = [160, 140, 110];
ctrl.Kd_arm = [32,  28,  22];

%% 5. 状态初始化与 RK4 动力学前向积分
q = zeros(1, 9);
qd = zeros(1, 9);
q(7:9) = qArm_0;
int_att_err = zeros(1, 3);

log_q        = zeros(nSteps, 9);
log_qd       = zeros(nSteps, 9);
log_tau_auv  = zeros(nSteps, 6);
log_tau_arm  = zeros(nSteps, 3);
log_tcp_pos  = zeros(nSteps, 3);
log_p_damp   = zeros(nSteps, 1);

fprintf('[3/5] 开始执行 9-DOF UVMS 水下耦合动力学积分解算...\n');
tic;

for k = 1:nSteps
    % 姿态误差积分
    int_att_err = int_att_err + q(4:6) * dt;
    int_att_err = max(min(int_att_err, 0.5), -0.5);
    
    % AUV 保持定点定姿
    tau_auv = [ -ctrl.Kp_auv_pos .* q(1:3) - ctrl.Kd_auv_pos .* qd(1:3), ...
                -ctrl.Kp_auv_att .* q(4:6) - ctrl.Ki_auv_att .* int_att_err - ctrl.Kd_auv_att .* qd(4:6) ];
    % 机械臂关节力矩控制 (带电机额定硬件饱和限幅 10 N*m)
    tau_arm = ctrl.Kp_arm .* (qArm_des(k, :) - q(7:9)) + ctrl.Kd_arm .* (qdArm_des(k, :) - qd(7:9));
    tau_arm = max(min(tau_arm, 10.0), -10.0);
    tau_cmd = [tau_auv, tau_arm];
    
    % 记录
    log_q(k, :)       = q;
    log_qd(k, :)      = qd;
    log_tau_auv(k, :) = tau_auv;
    log_tau_arm(k, :) = tau_arm;
    
    T_tcp = getTransform(uvms, q, 'link_004');
    log_tcp_pos(k, :) = (T_tcp(1:3, 1:3) * tcpOffset.' + T_tcp(1:3, 4)).';
    
    % 计算阻尼耗散功率
    v_rel = qd(1:6);
    v_rel(1:3) = v_rel(1:3) - env.current_vc.';
    F_damp_auv = D_linear_auv .* v_rel + D_quad_auv .* abs(v_rel) .* v_rel;
    F_damp_arm = D_arm .* qd(7:9);
    log_p_damp(k) = dot([qd(1:6), qd(7:9)], [F_damp_auv, F_damp_arm]);
    
    % RK4 步进 (含洋流相对流速)
    k1 = uvms_accel_full(uvms, q, qd, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore, env.current_vc);
    k2 = uvms_accel_full(uvms, q + 0.5*dt*qd, qd + 0.5*dt*k1, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore, env.current_vc);
    k3 = uvms_accel_full(uvms, q + 0.5*dt*(qd + 0.5*dt*k1), qd + 0.5*dt*k2, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore, env.current_vc);
    k4 = uvms_accel_full(uvms, q + dt*(qd + 0.5*dt*k2), qd + dt*k3, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore, env.current_vc);
    
    q = q + (dt/6) * (qd + 2*(qd + 0.5*dt*k1) + 2*(qd + 0.5*dt*k2) + (qd + dt*k3));
    qd = qd + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

simDuration = toc;
fprintf('      -> 动力学积分成功完成！耗时: %.2f 秒 (加速比: %.2fx)\n\n', ...
    simDuration, tEnd / simDuration);

%% 6. 定量评估指标计算
tracking_err_deg = abs(rad2deg(qArm_des - log_q(:, 7:9)));
max_tracking_err = max(tracking_err_deg, [], 1);
tcp_dist_to_target = vecnorm(log_tcp_pos - target_pos, 2, 2);
min_tcp_error_mm = min(tcp_dist_to_target) * 1000;
max_auv_drift_mm = max(vecnorm(log_q(:, 1:3), 2, 2)) * 1000;
max_auv_tilt_deg = max(vecnorm(rad2deg(log_q(:, 4:6)), 2, 2));
max_joint_torque = max(abs(log_tau_arm), [], 1);
total_mechanical_energy_J = sum(sum(abs(log_tau_arm .* log_qd(:, 7:9)))) * dt;

fprintf('==================== 仿真综合性能量化评估报告 ====================\n');
fprintf('  1. 目标抓取定位精度:\n');
fprintf('     - 抓取点最小空间误差: %.3f mm (到达预定抓取位)\n', min_tcp_error_mm);
fprintf('     - 机械臂关节最大动态跟踪误差: [q1: %.2f°, q2: %.2f°, q3: %.2f°]\n', ...
    max_tracking_err(1), max_tracking_err(2), max_tracking_err(3));
fprintf('  2. AUV 浮动基座稳定性 (受 0.25 m/s 洋流与反冲):\n');
fprintf('     - 最大空间平移漂移量: %.2f mm\n', max_auv_drift_mm);
fprintf('     - 最大姿态倾斜角: %.2f°\n', max_auv_tilt_deg);
fprintf('     - 终态残余定位误差: %.2f mm\n', norm(log_q(end, 1:3)) * 1000);
fprintf('  3. 执行器负载与能耗:\n');
fprintf('     - 各关节峰值力矩: [%.2f, %.2f, %.2f] N*m (额定上限 10.0 N*m，完全满足)\n', ...
    max_joint_torque(1), max_joint_torque(2), max_joint_torque(3));
fprintf('     - 抓取全过程机械臂总能耗: %.2f J\n', total_mechanical_energy_J);
fprintf('     - 水下最大阻尼耗散功率: %.3f W\n', max(log_p_damp));
fprintf('==================================================================\n\n');

%% 7. 绘制科研级综合仪表盘
fprintf('[4/5] 正在生成全景科研级仿真图表...\n');
fDash = figure('Name', '9-DOF UVMS 水下机械臂抓取综合仿真仪表盘', ...
    'Color', 'w', 'Position', [60, 60, 1300, 800]);

% 子图 1: 空间 3D 抓取真实轨迹
subplot(2, 3, 1);
plot3(log_tcp_pos(:, 1), log_tcp_pos(:, 2), log_tcp_pos(:, 3), 'b-', 'LineWidth', 2); hold on;
scatter3(log_tcp_pos(1, 1), log_tcp_pos(1, 2), log_tcp_pos(1, 3), 60, 'go', 'filled');
scatter3(target_pos(1), target_pos(2), target_pos(3), 80, 'r^', 'filled');
% 绘制洋流方向箭头
quiver3(0.2, 0.2, -0.3, env.current_vc(1)*0.4, env.current_vc(2)*0.4, 0, ...
    'Color', [0 0.6 0.8], 'LineWidth', 2, 'MaxHeadSize', 0.8);
text(0.2, 0.2, -0.28, sprintf('洋流 V_c=%.2f m/s', env.current_mag), 'Color', [0 0.5 0.7], 'FontWeight', 'bold');
grid on; axis equal;
xlabel('X (前向) / m'); ylabel('Y (侧向) / m'); zlabel('Z (垂向) / m');
title('【1】末端笛卡尔空间抓取轨迹');
legend('末端空间路径', '起点', '目标点', 'Location', 'best');
view(135, 25);

% 子图 2: 机械臂三轴关节角度跟踪
subplot(2, 3, 2);
plot(time, rad2deg(qArm_des(:, 1)), 'r--', time, rad2deg(log_q(:, 7)), 'r-', 'LineWidth', 1.2); hold on;
plot(time, rad2deg(qArm_des(:, 2)), 'g--', time, rad2deg(log_q(:, 8)), 'g-', 'LineWidth', 1.2);
plot(time, rad2deg(qArm_des(:, 3)), 'b--', time, rad2deg(log_q(:, 9)), 'b-', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('关节角度 / deg');
title('【2】机械臂各关节角度跟踪');
legend('q1_d', 'q1', 'q2_d', 'q2', 'q3_d', 'q3', 'Location', 'best');

% 子图 3: 机械臂各关节动态跟踪误差
subplot(2, 3, 3);
plot(time, tracking_err_deg(:, 1), 'r-', time, tracking_err_deg(:, 2), 'g-', time, tracking_err_deg(:, 3), 'b-', 'LineWidth', 1.3);
grid on; xlabel('时间 / s'); ylabel('跟踪误差 / deg');
title('【3】关节动态跟踪误差曲线');
legend('e_q1', 'e_q2', 'e_q3', 'Location', 'best');

% 子图 4: AUV 船体 6-DOF 空间平移与姿态扰动
subplot(2, 3, 4);
yyaxis left;
plot(time, log_q(:, 1:3)*1000, 'LineWidth', 1.3);
ylabel('平移漂移 / mm');
yyaxis right;
plot(time, rad2deg(log_q(:, 4:6)), '--', 'LineWidth', 1.3);
ylabel('姿态角 / deg');
grid on; xlabel('时间 / s');
title('【4】AUV 浮动基座空间漂移与晃动');
legend('X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)', 'Location', 'best');

% 子图 5: 机械臂关节电机驱动力矩
subplot(2, 3, 5);
plot(time, log_tau_arm, 'LineWidth', 1.4); hold on;
yline(10, 'r--', '力矩上限 +10 N*m');
yline(-10, 'r--', '力矩上限 -10 N*m');
grid on; xlabel('时间 / s'); ylabel('驱动力矩 / N*m');
title('【5】机械臂关节力矩 (含流体阻力)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'best');
ylim([-12, 12]);

% 子图 6: 流体阻尼耗散功率与机械臂功耗
subplot(2, 3, 6);
plot(time, log_p_damp, 'm-', 'LineWidth', 1.5); hold on;
plot(time, sum(abs(log_tau_arm .* log_qd(:, 7:9)), 2), 'k--', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('瞬时功率 / W');
title('【6】水下流体耗散功率与机械功率');
legend('水动力阻尼耗散功率', '电机瞬时机械总功率', 'Location', 'best');

dashImgPath = fullfile(scriptDir, 'uvms_full_simulation_dashboard.png');
exportgraphics(fDash, dashImgPath, 'Resolution', 200);
fprintf('      -> 全景仪表盘图表已保存至: %s\n', dashImgPath);

%% 8. 保存仿真完整数据集供后续二次开发
simDataFile = fullfile(scriptDir, 'uvms_full_sim_data.mat');
save(simDataFile, 'time', 'log_q', 'log_qd', 'log_tau_auv', 'log_tau_arm', ...
    'log_tcp_pos', 'target_pos', 'qArm_des', 'env', 'ctrl', 'tcpOffset');
fprintf('[5/5] 仿真完整时序数据已保存至: %s\n\n', simDataFile);
fprintf('全部仿真工作顺利完成！\n');

%% 内部求解函数
function pos = get_tcp_pos(robot, q, tcpOffset)
    T = getTransform(robot, q, 'link_004');
    pos = (T(1:3, 1:3) * tcpOffset.' + T(1:3, 4)).';
end

function qdd = uvms_accel_full(robot, q, qd, tau, M_added, D_lin_auv, D_quad_auv, D_arm, K_restore, vc_world)
    M_rigid = massMatrix(robot, q);
    M_total = M_rigid + M_added;
    C_term  = velocityProduct(robot, q, qd);
    G_term  = gravityTorque(robot, q);
    G_term(1:3) = 0; % 静水浮力平衡
    G_term(4)   = G_term(4) + K_restore * q(4);
    G_term(5)   = G_term(5) + K_restore * q(5);
    
    % 洋流相对速度修正
    v_auv = qd(1:6);
    v_rel_auv = v_auv;
    v_rel_auv(1:3) = v_auv(1:3) - vc_world.';
    
    F_damping = zeros(1, 9);
    F_damping(1:6) = D_lin_auv .* v_rel_auv + D_quad_auv .* abs(v_rel_auv) .* v_rel_auv;
    F_damping(7:9) = D_arm .* qd(7:9);
    
    qdd = (M_total \ (tau - C_term - G_term - F_damping).').';
end

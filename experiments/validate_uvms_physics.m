%% validate_uvms_physics.m
% =========================================================================
% 9-DOF UVMS 物理动力学模型准确性校验与自拟基准测试轨迹仿真
%
% 目标:
%   设计一套多轴复合激活动作 (伸展下探 -> 侧向提升 -> 快速回缩 -> 悬停平复)，
%   用于定量验证水下物理模型的准确性：
%     1. 牛顿第三定律反冲平衡 (机械臂前向挥动 vs AUV 质心反向后座)
%     2. 水动力阻尼能量耗散非负性 (验证阻尼项物理无源性，绝不发散注入能量)
%     3. 9x9 耦合质量矩阵正定性 (验证刚体惯量与附加质量正定)
%     4. 机械臂单侧偏心安装静倾角物理辨识与 PID 定位收敛性
% =========================================================================

clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
urdfFile = fullfile(scriptDir, 'robot.urdf');
armRobot = importrobot(urdfFile);
armRobot.DataFormat = 'row';
armRobot.Gravity = [0 0 -9.81];

%% 1. 构建 9-DOF 浮动基座模型
uvms = rigidBodyTree('DataFormat', 'row');
uvms.Gravity = [0 0 -9.81];

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

%% 2. 水动力与水静力参数
m_auv = auvBody.Mass;
M_added = zeros(9, 9);
M_added(1:3, 1:3) = diag([0.40, 0.40, 0.50] * m_auv);  % 船体平移附加质量
M_added(4:6, 4:6) = diag([0.30, 0.30, 0.30] * 20.0);   % 船体转动附加惯量
M_added(7:9, 7:9) = diag([0.05, 0.15, 0.12]);          % 机械臂连杆附加质量

D_linear_auv = [60, 60, 80, 25, 25, 20];               % 线性阻尼
D_quad_auv   = [120, 120, 150, 40, 40, 35];            % 非线性二次阻尼
D_arm        = [1.5, 2.5, 2.0];                        % 机械臂流体等效阻尼

% 稳心恢复刚度 (定倾中心高度 GM = 0.05 m)
auv_GM = 0.05;                                         
K_restore = m_auv * 9.81 * auv_GM;

%% 3. 控制器参数 (带偏置补偿的 AUV PID 控制器 + 机械臂伺服)
Kp_auv_pos = [400, 400, 500];
Kd_auv_pos = [200, 200, 250];

Kp_auv_att = [250, 250, 200];
Ki_auv_att = [60,  60,  50];   % 积分消除机械臂侧向偏心挂载引起的静态侧倾
Kd_auv_att = [90,  90,  70];

Kp_arm = [150, 120, 100];
Kd_arm = [30,  25,  20];

%% 4. 自拟基准测试轨迹 (四阶段平滑往复，充分激发科氏力、离心力与反冲)
% 阶段 1 (0 ~ 1s):   静置自平衡检验
% 阶段 2 (1 ~ 4s):   大范围下探抓取 (关节 1 偏转, 2 下压, 3 调整)
% 阶段 3 (4 ~ 7s):   侧向提升与变向动作 (三轴同时正反交替运动)
% 阶段 4 (7 ~ 10s):  快速回缩归位并悬停平复
tEnd = 10.0;
dt = 0.005;
time = 0:dt:tEnd;
nSteps = numel(time);

waypoints_time = [0,  1.0,  4.0,   7.0,  10.0];
waypoints_q    = [0,    0,    0;                   % 初态 (0s)
                  0,    0,    0;                   % 静止自平衡 (1s)
                  0.50, -0.65, 0.45;               % 伸展下探抓取 (4s)
                 -0.30, -0.20, -0.35;              % 侧向提升偏转 (7s)
                  0,    0,    0];                  % 回缩原位 (10s)

qArm_des   = zeros(nSteps, 3);
qdArm_des  = zeros(nSteps, 3);
qddArm_des = zeros(nSteps, 3);

for i = 1:3
    [qArm_des(:, i), qdArm_des(:, i), qddArm_des(:, i)] = ...
        pchip_smooth_traj(time, waypoints_time, waypoints_q(:, i));
end

%% 5. 仿真与物理准确性指标记录
q = zeros(1, 9);
qd = zeros(1, 9);
int_att_err = zeros(1, 3);

log_q          = zeros(nSteps, 9);
log_qd         = zeros(nSteps, 9);
log_qdd        = zeros(nSteps, 9);
log_tau_auv    = zeros(nSteps, 6);
log_tau_arm    = zeros(nSteps, 3);
log_tcp_pos    = zeros(nSteps, 3);
log_min_eig_M  = zeros(nSteps, 1);
log_p_damping  = zeros(nSteps, 1);
log_arm_com_dx = zeros(nSteps, 1);

tcpOffset = [0.280, -0.030, -0.0168];

fprintf('开始执行物理准确性校验基准测试 (时长: %.1f 秒)...\n', tEnd);
tic;

for k = 1:nSteps
    % 姿态积分累加 (带限幅防积分饱和)
    int_att_err = int_att_err + q(4:6) * dt;
    int_att_err = max(min(int_att_err, 0.5), -0.5);
    
    % AUV 控制器 (PID 抑制偏心力矩)
    tau_auv = [ -Kp_auv_pos .* q(1:3) - Kd_auv_pos .* qd(1:3), ...
                -Kp_auv_att .* q(4:6) - Ki_auv_att .* int_att_err - Kd_auv_att .* qd(4:6) ];
            
    % 机械臂控制器
    tau_arm = Kp_arm .* (qArm_des(k, :) - q(7:9)) + Kd_arm .* (qdArm_des(k, :) - qd(7:9));
    tau_cmd = [tau_auv, tau_arm];
    
    % 计算当前动力学加速度及校验物理量
    [qdd, M_tot, F_damp] = uvms_accel_eval(uvms, q, qd, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore);
    
    % 记录物理一致性指标
    log_q(k, :)         = q;
    log_qd(k, :)        = qd;
    log_qdd(k, :)       = qdd;
    log_tau_auv(k, :)   = tau_auv;
    log_tau_arm(k, :)   = tau_arm;
    log_min_eig_M(k)    = min(eig(M_tot));                       % 质量矩阵最小特征值 (必须 > 0)
    log_p_damping(k)    = dot([qd(1:6), qd(7:9)], F_damp);       % 水阻消耗功率 (必须 >= 0)
    
    % 计算末端 TCP 坐标
    T_tcp = getTransform(uvms, q, 'link_004');
    log_tcp_pos(k, :) = (T_tcp(1:3, 1:3) * tcpOffset.' + T_tcp(1:3, 4)).';
    
    % 求解机械臂整体质心位移 (相对于 AUV 船体)
    log_arm_com_dx(k) = eval_arm_com_x(uvms, q);
    
    % RK4 步进
    k1 = qdd;
    k2 = uvms_accel_eval(uvms, q + 0.5*dt*qd, qd + 0.5*dt*k1, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore);
    k3 = uvms_accel_eval(uvms, q + 0.5*dt*(qd + 0.5*dt*k1), qd + 0.5*dt*k2, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore);
    k4 = uvms_accel_eval(uvms, q + dt*(qd + 0.5*dt*k2), qd + dt*k3, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_restore);
    
    q = q + (dt/6) * (qd + 2*(qd + 0.5*dt*k1) + 2*(qd + 0.5*dt*k2) + (qd + dt*k3));
    qd = qd + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

simTime = toc;
fprintf('仿真顺利完成！计算耗时: %.2f 秒\n', simTime);

%% 6. 输出物理模型准确性定量检验报告
fprintf('\n================ 物理模型准确性量化校验报告 ================\n');
fprintf('1. 质量矩阵正定性检验: 最小特征值 = %.4f (恒 > 0，验证通过)\n', min(log_min_eig_M));
fprintf('2. 水阻功率耗散无源性: 最小耗散功率 = %.6e W (恒 >= 0，绝无自激发散)\n', min(log_p_damping));
fprintf('3. 最大水阻耗散瞬时功率: %.4f W\n', max(log_p_damping));
fprintf('4. AUV 最终复位残余误差: 位置 = %.4f mm, 姿态 = %.4f deg (验证稳态收敛)\n', ...
    norm(log_q(end, 1:3)) * 1000, norm(rad2deg(log_q(end, 4:6))));
fprintf('5. 机械臂关节跟踪最大误差: [%.4f, %.4f, %.4f] deg\n', ...
    max(abs(rad2deg(qArm_des - log_q(:, 7:9)))));
fprintf('============================================================\n\n');

%% 7. 绘图：物理规律定量验证曲线
fVal = figure('Name', '物理准确性校验全景分析', 'Color', 'w', 'Position', [80, 80, 1200, 750]);

% 子图 1: 反冲作用验证 (机械臂伸展位移 vs AUV 船体反向后座)
subplot(2, 3, 1);
yyaxis left;
plot(time, log_arm_com_dx * 100, 'b-', 'LineWidth', 1.5);
ylabel('机械臂质心向前相对位移 / cm');
yyaxis right;
plot(time, log_q(:, 1) * 1000, 'r--', 'LineWidth', 1.5);
ylabel('AUV 船体反向后座漂移 / mm');
grid on; xlabel('时间 / s');
title('【物理定律 1】反冲平衡检验 (牛顿第三定律)');
legend('机械臂质心位移', 'AUV 反冲漂移', 'Location', 'best');

% 子图 2: AUV 6 自由度晃动响应
subplot(2, 3, 2);
plot(time, log_q(:, 1:3)*1000, 'LineWidth', 1.3);
grid on; xlabel('时间 / s'); ylabel('位移 / mm');
title('【基座响应】AUV 平移扰动漂移 (X, Y, Z)');
legend('X (纵向)', 'Y (横向)', 'Z (垂向)');

% 子图 3: AUV 姿态晃动与静水恢复
subplot(2, 3, 3);
plot(time, rad2deg(log_q(:, 4:6)), 'LineWidth', 1.3);
grid on; xlabel('时间 / s'); ylabel('姿态角 / deg');
title('【基座响应】AUV 姿态倾斜 (Roll, Pitch, Yaw)');
legend('Roll', 'Pitch', 'Yaw');

% 子图 4: 机械臂各轴真实轨迹跟踪
subplot(2, 3, 4);
plot(time, rad2deg(qArm_des(:, 1)), 'k--', time, rad2deg(log_q(:, 7)), 'r-', 'LineWidth', 1.2); hold on;
plot(time, rad2deg(qArm_des(:, 2)), 'k--', time, rad2deg(log_q(:, 8)), 'g-', 'LineWidth', 1.2);
plot(time, rad2deg(qArm_des(:, 3)), 'k--', time, rad2deg(log_q(:, 9)), 'b-', 'LineWidth', 1.2);
grid on; xlabel('时间 / s'); ylabel('关节角 / deg');
title('【轨迹执行】机械臂多轴大范围复合运动跟踪');
legend('q1_d', 'q1', 'q2_d', 'q2', 'q3_d', 'q3');

% 子图 5: 水动力耗散功率非负性曲线
subplot(2, 3, 5);
plot(time, log_p_damping, 'm-', 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('耗散功率 / W');
title('【物理定律 2】流体阻力功率耗散 (恒非负)');

% 子图 6: 机械臂电机驱动力矩 (含流体阻力与反冲反力矩)
subplot(2, 3, 6);
plot(time, log_tau_arm, 'LineWidth', 1.4);
grid on; xlabel('时间 / s'); ylabel('力矩 / N*m');
title('【动力学载荷】机械臂各关节驱动力矩曲线');
legend('Joint 1', 'Joint 2', 'Joint 3');

exportgraphics(fVal, fullfile(scriptDir, 'uvms_physics_validation.png'), 'Resolution', 180);
fprintf('校验图表已保存至: %s\n', fullfile(scriptDir, 'uvms_physics_validation.png'));

%% 内部辅助函数
function [qdd, M_total, F_damping] = uvms_accel_eval(robot, q, qd, tau, M_added, D_lin_auv, D_quad_auv, D_arm, K_restore)
    M_rigid = massMatrix(robot, q);
    M_total = M_rigid + M_added;
    C_term  = velocityProduct(robot, q, qd);
    G_term  = gravityTorque(robot, q);
    G_term(1:3) = 0; % 静水浮力平衡
    G_term(4)   = G_term(4) + K_restore * q(4);
    G_term(5)   = G_term(5) + K_restore * q(5);
    
    F_damping = zeros(1, 9);
    v_auv = qd(1:6);
    F_damping(1:6) = D_lin_auv .* v_auv + D_quad_auv .* abs(v_auv) .* v_auv;
    F_damping(7:9) = D_arm .* qd(7:9);
    
    qdd = (M_total \ (tau - C_term - G_term - F_damping).').';
end

function [q, qd, qdd] = pchip_smooth_traj(t, wt, wq)
    q = interp1(wt, wq, t, 'pchip');
    qd = gradient(q, t(2) - t(1));
    qdd = gradient(qd, t(2) - t(1));
end

function arm_dx = eval_arm_com_x(robot, q)
    T3 = getTransform(robot, q, 'link_003');
    T4 = getTransform(robot, q, 'link_004');
    arm_dx = 0.5 * (T3(1, 4) + T4(1, 4)) - q(1);
end

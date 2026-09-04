%% simulate_uvms_dynamics.m
% =========================================================================
% 9-DOF 水下机器人-机械臂系统 (UVMS) 动力学与浮动基座仿真
%
% 系统自由度分配:
%   [1:3] AUV 空间平移: X, Y, Z (m)
%   [4:6] AUV 空间转角: Roll, Pitch, Yaw (rad)
%   [7:9] 机械臂关节:   link_002, link_003, link_004 (rad)
%
% 控制架构:
%   - AUV 船体: 6 维定点定姿 PD 悬停控制器 (模拟推进器闭环定位)
%   - 机械臂:   3 关节平滑轨迹跟踪 PD 控制器
% =========================================================================

clear; clc; close all;

%% 1. 构建 9-DOF UVMS 刚体树模型 (直接继承 URDF 惯量、质量与质心)
scriptDir = fileparts(mfilename('fullpath'));
urdfFile = fullfile(scriptDir, 'robot.urdf');
armRobot = importrobot(urdfFile);
armRobot.DataFormat = 'row';
armRobot.Gravity = [0 0 -9.81];

uvms = rigidBodyTree('DataFormat', 'row');
uvms.Gravity = [0 0 -9.81];

% 6 自由度浮动基座虚拟关节链
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

% 添加 AUV 船体 (继承 URDF base_link 完整物理属性)
auvBody = copy(armRobot.Base);
auvBody.Name = 'auv_base';
jAuv = rigidBodyJoint('joint_virtual_yaw', 'revolute');
jAuv.JointAxis = jointAxes(6, :);
auvBody.Joint = jAuv;
addBody(uvms, auvBody, 'virtual_pitch');

% 添加 3 个机械臂运动连杆
for i = 1:armRobot.NumBodies
    srcBody = armRobot.Bodies{i};
    newBody = copy(srcBody);
    parentName = srcBody.Parent.Name;
    if strcmp(parentName, armRobot.BaseName)
        parentName = 'auv_base';
    end
    addBody(uvms, newBody, parentName);
end

%% 2. 水动力参数配置
rhoWater = 1025; % 海水密度 [kg/m^3]

% AUV 附加质量矩阵 (6x6 估算，平移约取本体质量的 40%，转动约取 30%)
m_auv = auvBody.Mass;
M_added = zeros(9, 9);
M_added(1:3, 1:3) = diag([0.4, 0.4, 0.5] * m_auv);
M_added(4:6, 4:6) = diag([0.3, 0.3, 0.3] * 20.0);

% 机械臂连杆附加质量 (主要在垂直截面，较小)
M_added(7, 7) = 0.05;
M_added(8, 8) = 0.15;
M_added(9, 9) = 0.12;

% 水动力线性阻尼与二次阻尼系数 (阻碍船体与连杆在水中的快速运动)
D_linear_auv = [60, 60, 80, 25, 25, 20];       % [Fx, Fy, Fz, Tx, Ty, Tz]
D_quad_auv   = [120, 120, 150, 40, 40, 35];
D_arm        = [1.5, 2.5, 2.0];                % 机械臂关节水动力等效阻尼 [N*m*s/rad]

% AUV 浮力配平参数 (中性浮力，重心低于浮心产生天然稳心恢复力矩)
auv_metacentric_height = 0.03; % 稳心恢复高度 GM [m]
K_buoyancy_restore = m_auv * 9.81 * auv_metacentric_height; % 倾斜自复原刚度 [N*m/rad]

%% 3. 控制器参数配置
% AUV 6 维 PD 悬停控制器 (定点定姿)
Kp_auv_pos = [350, 350, 450];    % 位置刚度 [N/m]
Kd_auv_pos = [180, 180, 220];    % 位置阻尼 [N*s/m]
Kp_auv_att = [120, 120, 100];    % 姿态刚度 [N*m/rad]
Kd_auv_att = [60,  60,  50];     % 姿态阻尼 [N*m*s/rad]

% 机械臂 3 关节伺服跟踪 PD 参数
Kp_arm = [120, 100, 80];         % 关节比例增益 [N*m/rad]
Kd_arm = [25,  20,  15];         % 关节微分增益 [N*m*s/rad]

%% 4. 机械臂测试动作轨迹设定
% 提示：以后确定了具体轨迹，直接修改此处的期望函数
tEnd = 8.0;                      % 仿真总时长 [s]
dt = 0.005;                      % 积分步长 [s]
time = 0:dt:tEnd;
nSteps = numel(time);

% 预设动作：在 t=1s~4s 从 q0 移动到 qTarget，随后保持
qArm_0 = [0, 0, 0];
qArm_target = [0.45, -0.50, 0.40]; % [q1, q2, q3] (rad)

qArm_des = zeros(nSteps, 3);
qdArm_des = zeros(nSteps, 3);
qddArm_des = zeros(nSteps, 3);

for k = 1:nSteps
    t = time(k);
    if t < 1.0
        qArm_des(k, :) = qArm_0;
    elseif t <= 4.0
        tau = (t - 1.0) / 3.0; % 归一化时间 [0, 1]
        % 五次多项式平滑过渡 (速度、加速度端点全为 0)
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / 3.0;
        sdd = (60*tau - 180*tau^2 + 120*tau^3) / 9.0;
        
        qArm_des(k, :) = qArm_0 + s * (qArm_target - qArm_0);
        qdArm_des(k, :) = sd * (qArm_target - qArm_0);
        qddArm_des(k, :) = sdd * (qArm_target - qArm_0);
    else
        qArm_des(k, :) = qArm_target;
    end
end

%% 5. 状态初始化与 RK4 动力学前向积分
% 状态向量 x = [q; qd]，长度 18
q = zeros(1, 9);
qd = zeros(1, 9);
q(7:9) = qArm_0;

% 历史记录变量
log_q = zeros(nSteps, 9);
log_qd = zeros(nSteps, 9);
log_tau_auv = zeros(nSteps, 6);
log_tau_arm = zeros(nSteps, 3);
log_tcp_pos = zeros(nSteps, 3);

tcpOffset = [0.280, -0.030, -0.0168]; % 夹爪抓取中心偏移 [m]

fprintf('开始 9-DOF UVMS 水下动力学与浮动基座仿真 (时长 %.1f 秒)...\n', tEnd);
tic;

for k = 1:nSteps
    t = time(k);
    
    % --- 求解当前时刻控制器输出 ---
    % 1) AUV 6-DOF 悬停控制力矩 (目标保持在 0 位姿)
    pos_err = q(1:3);
    vel_err = qd(1:3);
    att_err = q(4:6);
    omg_err = qd(4:6);
    
    tau_auv = [ -Kp_auv_pos .* pos_err - Kd_auv_pos .* vel_err, ...
                -Kp_auv_att .* att_err - Kd_auv_att .* omg_err ];
            
    % 2) 机械臂关节控制力矩 (PD + 重力/水静力前馈)
    arm_pos_err = qArm_des(k, :) - q(7:9);
    arm_vel_err = qdArm_des(k, :) - qd(7:9);
    tau_arm = Kp_arm .* arm_pos_err + Kd_arm .* arm_vel_err;
    
    % 力矩限制
    tau_arm = max(min(tau_arm, 15), -15);
    
    tau_cmd = [tau_auv, tau_arm];
    
    % 记录当前时刻数据
    log_q(k, :) = q;
    log_qd(k, :) = qd;
    log_tau_auv(k, :) = tau_auv;
    log_tau_arm(k, :) = tau_arm;
    
    % 计算夹爪在世界坐标系中的位移
    T_tcp = getTransform(uvms, q, 'link_004');
    p_tcp_world = T_tcp(1:3, 1:3) * tcpOffset.' + T_tcp(1:3, 4);
    log_tcp_pos(k, :) = p_tcp_world.';
    
    % --- RK4 数值积分步进 ---
    k1 = uvms_accel(uvms, q, qd, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_buoyancy_restore);
    k2 = uvms_accel(uvms, q + 0.5*dt*qd, qd + 0.5*dt*k1, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_buoyancy_restore);
    k3 = uvms_accel(uvms, q + 0.5*dt*(qd + 0.5*dt*k1), qd + 0.5*dt*k2, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_buoyancy_restore);
    k4 = uvms_accel(uvms, q + dt*(qd + 0.5*dt*k2), qd + dt*k3, tau_cmd, M_added, D_linear_auv, D_quad_auv, D_arm, K_buoyancy_restore);
    
    q = q + (dt/6) * (qd + 2*(qd + 0.5*dt*k1) + 2*(qd + 0.5*dt*k2) + (qd + dt*k3));
    qd = qd + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

simTime = toc;
fprintf('仿真完成！耗时: %.2f 秒 (加速比: %.1fx)\n', simTime, tEnd / simTime);

%% 6. 绘图展示
% 图 1: AUV 船体受机械臂反冲的 6-DOF 晃动与复位响应
f1 = figure('Name', 'AUV 船体晃动响应', 'Color', 'w', 'Position', [100, 100, 1100, 600]);
subplot(2, 2, 1);
plot(time, log_q(:, 1:3) * 1000, 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('位移偏移 / mm');
title('AUV 空间平移位移漂移量 (X, Y, Z)');
legend('X (纵向)', 'Y (横向)', 'Z (垂向)', 'Location', 'best');

subplot(2, 2, 2);
plot(time, rad2deg(log_q(:, 4:6)), 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('姿态晃动 / deg');
title('AUV 姿态倾斜与偏转角 (Roll, Pitch, Yaw)');
legend('Roll (横摇)', 'Pitch (俯仰)', 'Yaw (艏摇)', 'Location', 'best');

subplot(2, 2, 3);
plot(time, log_tau_auv(:, 1:3), 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('控制力 / N');
title('AUV 虚拟推进器恢复力 (Fx, Fy, Fz)');
legend('Fx', 'Fy', 'Fz', 'Location', 'best');

subplot(2, 2, 4);
plot(time, log_tau_auv(:, 4:6), 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('控制力矩 / N*m');
title('AUV 虚拟推进器恢复力矩 (Tx, Ty, Tz)');
legend('Tx', 'Ty', 'Tz', 'Location', 'best');
exportgraphics(f1, fullfile(scriptDir, 'uvms_auv_response.png'), 'Resolution', 180);

% 图 2: 机械臂关节运动跟踪与驱动力矩
f2 = figure('Name', '机械臂关节与末端轨迹', 'Color', 'w', 'Position', [150, 150, 1100, 600]);
subplot(2, 2, 1);
plot(time, rad2deg(qArm_des(:, 1)), 'r--', time, rad2deg(log_q(:, 7)), 'b-', 'LineWidth', 1.3); hold on;
plot(time, rad2deg(qArm_des(:, 2)), 'm--', time, rad2deg(log_q(:, 8)), 'g-', 'LineWidth', 1.3);
plot(time, rad2deg(qArm_des(:, 3)), 'k--', time, rad2deg(log_q(:, 9)), 'c-', 'LineWidth', 1.3);
grid on; xlabel('时间 / s'); ylabel('关节角度 / deg');
title('机械臂关节轨迹跟踪');
legend('q1_d', 'q1', 'q2_d', 'q2', 'q3_d', 'q3', 'Location', 'best');

subplot(2, 2, 2);
plot(time, log_tau_arm, 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('驱动力矩 / N*m');
title('机械臂各关节驱动力矩 (含反冲与水阻抵消)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'best');

subplot(2, 2, [3, 4]);
plot(time, log_tcp_pos(:, 1), 'r-', time, log_tcp_pos(:, 2), 'g-', time, log_tcp_pos(:, 3), 'b-', 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('世界坐标 / m');
title('夹爪中心末端在世界坐标系中的真实空间位移');
legend('X_tcp', 'Y_tcp', 'Z_tcp', 'Location', 'best');
exportgraphics(f2, fullfile(scriptDir, 'uvms_arm_tracking.png'), 'Resolution', 180);

fprintf('仿真曲线已保存至:\n  %s\n  %s\n', ...
    fullfile(scriptDir, 'uvms_auv_response.png'), ...
    fullfile(scriptDir, 'uvms_arm_tracking.png'));

%% 内部动力学加速度求解函数
function qdd = uvms_accel(robot, q, qd, tau, M_added, D_lin_auv, D_quad_auv, D_arm, K_restore)
    % 1. 广义质量矩阵 (多体刚体质量 + 水下附加质量)
    M_rigid = massMatrix(robot, q);
    M_total = M_rigid + M_added;
    
    % 2. 科氏力与离心力项 C(q, qd)*qd
    C_term = velocityProduct(robot, q, qd);
    
    % 3. 重力项 (AUV 设为中性浮力，因此前 3 维垂直重力被静水浮力抵消)
    G_term = gravityTorque(robot, q);
    G_term(1:3) = 0; % AUV 纵横垂向重浮力平衡
    
    % AUV 横摇/俯仰稳心恢复力矩 (重心在浮心下方的自复原)
    G_term(4) = G_term(4) + K_restore * q(4);
    G_term(5) = G_term(5) + K_restore * q(5);
    
    % 4. 水下阻尼外力/力矩 (AUV 阻尼 + 机械臂连杆流体阻力)
    F_damping = zeros(1, 9);
    v_auv = qd(1:6);
    F_damping(1:6) = D_lin_auv .* v_auv + D_quad_auv .* abs(v_auv) .* v_auv;
    F_damping(7:9) = D_arm .* qd(7:9);
    
    % 5. 广义加速度求解: qdd = M^-1 * (tau - C - G - D)
    qdd = (M_total \ (tau - C_term - G_term - F_damping).').';
end

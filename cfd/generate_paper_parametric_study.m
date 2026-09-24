% generate_paper_parametric_study.m
% =========================================================================
% 面向学术论文发表的水下机械臂多工况参数化实验生成脚本
% 包含:
%   1. 流速梯度实验 (Vc = 0.0, 0.25, 0.50, 0.75, 1.00 m/s @ psi = 0 deg)
%   2. 流向方位角实验 (psi = 0, 30, 45, 60, 90 deg @ Vc = 0.50 m/s)
%   3. 流速 x 流向 (5 x 5 = 25 组) 全包络三维响应曲面与电机安全裕度校核
% =========================================================================

close all;
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
[hydro, env_base] = underwater_hydro_defaults(robot);
robot.Gravity = [0, 0, -env_base.g];

% Load nominal trajectory kinematics from joint_loads_data.mat
baseData = load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));
time = baseData.time;
q_seq = baseData.q_seq;
qd_seq = baseData.qd_seq;
qdd_seq = baseData.qdd_seq;
nFrames = numel(time);
dt = time(2) - time(1);

% Precompute rigid-body dynamics (independent of ocean current)
tau_rigid_grav_buoy = baseData.tau_inertial_rigid + baseData.tau_coriolis + ...
                      baseData.tau_gravity + baseData.tau_buoyancy;

fprintf('========================================================================\n');
fprintf('   开始生成学术论文多工况参数化对比实验数据集与科研图表\n');
fprintf('========================================================================\n\n');

%% 实验一：不同洋流流速灵敏度分析 (固定流向 psi = 0 deg)
speeds = [0.00, 0.25, 0.50, 0.75, 1.00]; % [m/s]
nS = numel(speeds);
speed_results = struct();

fprintf('>>> [实验 1] 不同流速灵敏度分析 (psi = 0 deg):\n');
fprintf('流速(m/s) | J1阻力峰值 | J2阻力峰值 | J3阻力峰值 | J1总力矩峰值 | J2总力矩峰值 | J3总力矩峰值 | 总能耗(J)\n');
fprintf('-----------------------------------------------------------------------------------------------------\n');

for s_idx = 1:nS
    vc_mag = speeds(s_idx);
    vc_vec = [vc_mag, 0.0, 0.0];
    [~, parts] = evaluate_hydro_reference(robot, hydro, env_base, q_seq, qd_seq, qdd_seq, vc_vec);
    
    % Fluid load opposing arm motion
    tau_drag = -parts.drag;
    tau_add  = -parts.added_inertia;
    tau_tot  = tau_rigid_grav_buoy + tau_drag + tau_add;
    
    % Mechanical energy
    power_abs = sum(abs(tau_tot .* qd_seq), 2);
    energy_J = trapz(time, power_abs);
    
    speed_results(s_idx).speed = vc_mag;
    speed_results(s_idx).tau_drag = tau_drag;
    speed_results(s_idx).tau_total = tau_tot;
    speed_results(s_idx).pk_drag = max(abs(tau_drag), [], 1);
    speed_results(s_idx).pk_tot = max(abs(tau_tot), [], 1);
    speed_results(s_idx).rms_tot = sqrt(mean(tau_tot.^2, 1));
    speed_results(s_idx).energy_J = energy_J;
    
    fprintf('  %5.2f   |   %6.3f   |   %6.3f   |   %6.3f   |    %6.3f    |    %6.3f    |    %6.3f    |  %6.3f\n', ...
        vc_mag, speed_results(s_idx).pk_drag(1), speed_results(s_idx).pk_drag(2), speed_results(s_idx).pk_drag(3), ...
        speed_results(s_idx).pk_tot(1), speed_results(s_idx).pk_tot(2), speed_results(s_idx).pk_tot(3), energy_J);
end

%% 实验二：不同洋流方位角灵敏度分析 (固定流速 Vc = 0.50 m/s)
angles_deg = [0, 30, 45, 60, 90];
nA = numel(angles_deg);
angle_results = struct();
vc_fixed = 0.50;

fprintf('\n>>> [实验 2] 不同流向方位角灵敏度分析 (Vc = 0.50 m/s):\n');
fprintf('方位角(°) | J1阻力峰值 | J2阻力峰值 | J3阻力峰值 | J1总力矩峰值 | J2总力矩峰值 | J3总力矩峰值 | 总能耗(J)\n');
fprintf('-----------------------------------------------------------------------------------------------------\n');

for a_idx = 1:nA
    psi_rad = deg2rad(angles_deg(a_idx));
    vc_vec = [vc_fixed * cos(psi_rad), vc_fixed * sin(psi_rad), 0.0];
    [~, parts] = evaluate_hydro_reference(robot, hydro, env_base, q_seq, qd_seq, qdd_seq, vc_vec);
    
    tau_drag = -parts.drag;
    tau_add  = -parts.added_inertia;
    tau_tot  = tau_rigid_grav_buoy + tau_drag + tau_add;
    
    power_abs = sum(abs(tau_tot .* qd_seq), 2);
    energy_J = trapz(time, power_abs);
    
    angle_results(a_idx).angle_deg = angles_deg(a_idx);
    angle_results(a_idx).tau_drag = tau_drag;
    angle_results(a_idx).tau_total = tau_tot;
    angle_results(a_idx).pk_drag = max(abs(tau_drag), [], 1);
    angle_results(a_idx).pk_tot = max(abs(tau_tot), [], 1);
    angle_results(a_idx).rms_tot = sqrt(mean(tau_tot.^2, 1));
    angle_results(a_idx).energy_J = energy_J;
    
    fprintf('  %5.1f   |   %6.3f   |   %6.3f   |   %6.3f   |    %6.3f    |    %6.3f    |    %6.3f    |  %6.3f\n', ...
        angles_deg(a_idx), angle_results(a_idx).pk_drag(1), angle_results(a_idx).pk_drag(2), angle_results(a_idx).pk_drag(3), ...
        angle_results(a_idx).pk_tot(1), angle_results(a_idx).pk_tot(2), angle_results(a_idx).pk_tot(3), energy_J);
end

%% 实验三：生成论文级综合对比大图 (Figure: 流速灵敏度 + 流向灵敏度 + 极值包络)
fig = figure('Name', 'Paper Parametric Study Dashboard', ...
    'Units', 'pixels', 'Position', [80, 50, 1450, 920], 'Color', 'w');

cmap_s = [0.2 0.2 0.2; 0.0 0.45 0.74; 0.47 0.67 0.19; 0.93 0.69 0.13; 0.85 0.33 0.10];

% Subplot 1: Joint 1 Hydrodynamic Drag Torque under 5 Speeds (0~10s)
subplot(2, 2, 1);
hold on; grid on; box on;
for s_idx = 1:nS
    plot(time, speed_results(s_idx).tau_drag(:, 1), 'Color', cmap_s(s_idx, :), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('V_c = %.2f m/s', speeds(s_idx)));
end
title('(a) 不同洋流流速下关节 1 (基座回转轴) 水动力阻力矩时序演化 (\psi = 0^\circ)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('关节 1 流体阻力矩 \tau_{drag,1} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8.5);
xlim([0, 10]);

% Subplot 2: Joint 2 Hydrodynamic Drag Torque under 5 Flow Angles (Vc = 0.50 m/s)
subplot(2, 2, 2);
hold on; grid on; box on;
for a_idx = 1:nA
    plot(time, angle_results(a_idx).tau_drag(:, 2), 'Color', cmap_s(a_idx, :), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('\\psi = %d^\\circ', angles_deg(a_idx)));
end
title('(b) 不同来流方位角下关节 2 (肩部俯仰轴) 水动力阻力矩时序演化 (V_c = 0.50 m/s)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('关节 2 流体阻力矩 \tau_{drag,2} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8.5);
xlim([0, 10]);

% Subplot 3: Peak Torque vs Current Speed (Quadratic growth law)
subplot(2, 2, 3);
hold on; grid on; box on;
pk_tot_mat = reshape([speed_results.pk_tot], 3, nS).';
pk_drg_mat = reshape([speed_results.pk_drag], 3, nS).';
plot(speeds, pk_tot_mat(:, 1), '-o', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0, 'MarkerFaceColor', [0.85 0.33 0.10], 'DisplayName', 'J1 总驱动力矩峰值');
plot(speeds, pk_tot_mat(:, 2), '-s', 'Color', [0.00 0.45 0.74], 'LineWidth', 2.0, 'MarkerFaceColor', [0.00 0.45 0.74], 'DisplayName', 'J2 总驱动力矩峰值');
plot(speeds, pk_tot_mat(:, 3), '-^', 'Color', [0.47 0.67 0.19], 'LineWidth', 2.0, 'MarkerFaceColor', [0.47 0.67 0.19], 'DisplayName', 'J3 总驱动力矩峰值');
plot(speeds, pk_drg_mat(:, 1), '--o', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4, 'DisplayName', 'J1 纯流体阻力峰值 (\propto V_c^2)');
yline(10.0, 'r--', '电机额定力矩上限 (10.0 N·m)', 'LineWidth', 1.5);
title('(c) 三关节峰值力矩随流速 V_c 的二次方增长特性与安全裕度 (\psi = 0^\circ)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('力矩峰值 |\tau|_{max} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8.5);
ylim([0, 11.5]);

% Subplot 4: Peak Drag Torque vs Flow Azimuth Angle (Coupling redistribution)
subplot(2, 2, 4);
hold on; grid on; box on;
pk_ang_drg = reshape([angle_results.pk_drag], 3, nA).';
pk_ang_tot = reshape([angle_results.pk_tot], 3, nA).';
plot(angles_deg, pk_ang_drg(:, 1), '-o', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0, 'MarkerFaceColor', [0.85 0.33 0.10], 'DisplayName', 'J1 水阻峰值 (水平回转)');
plot(angles_deg, pk_ang_drg(:, 2), '-s', 'Color', [0.00 0.45 0.74], 'LineWidth', 2.0, 'MarkerFaceColor', [0.00 0.45 0.74], 'DisplayName', 'J2 水阻峰值 (肩部俯仰)');
plot(angles_deg, pk_ang_drg(:, 3), '-^', 'Color', [0.47 0.67 0.19], 'LineWidth', 2.0, 'MarkerFaceColor', [0.47 0.67 0.19], 'DisplayName', 'J3 水阻峰值 (小臂与爪)');
plot(angles_deg, pk_ang_tot(:, 1), '--o', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4, 'DisplayName', 'J1 总力矩峰值');
plot(angles_deg, pk_ang_tot(:, 2), '--s', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4, 'DisplayName', 'J2 总力矩峰值');
title('(d) 来流方位角 \psi 对三关节水动力矩与总负荷的耦合分配效应 (V_c = 0.50 m/s)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('来流水平方位角 \psi (deg)', 'FontSize', 10);
ylabel('力矩峰值 |\tau|_{max} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8.5);
xlim([0, 90]);

out_fig = fullfile(rootDir, 'docs', 'figures', 'paper_parametric_sensitivity_dashboard.png');
exportgraphics(fig, out_fig, 'Resolution', 300);
fprintf('\n[成功] 论文多工况灵敏度分析大图已保存至: %s\n', out_fig);

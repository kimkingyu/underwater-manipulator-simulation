% plot_5s_full_dynamics_comparison.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

% 1. Load MATLAB full 10s baseline (1001 frames, 100 Hz)
matData = load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));

% 2. Load verified CFD data
cfd_csv = fullfile(rootDir, 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/solve_10s_production/phaseA_deploy.csv');
opts = struct('allow_invalid', false, 'require_sidecar', true, 'include_buoyancy', false, 'env_verified', true);
report = compare_fluent_with_matlab(cfd_csv, 0.25, 0.0, opts);

% 3. Extract 0 ~ 5.0s data
idx_5s = find(matData.time <= 5.0);
t_5s = matData.time(idx_5s);
q_5s = matData.q_seq(idx_5s, :);
qd_5s = matData.qd_seq(idx_5s, :);
tau_drag_5s = matData.tau_hydro_drag(idx_5s, :);
tau_total_5s = matData.tau_total(idx_5s, :);

fprintf('Trajectory length at 5s: %d frames (t = 0.0s to %.2fs)\n', numel(t_5s), t_5s(end));

% 4. Create 4-panel publication-grade 5.0-second dashboard
fig = figure('Name', 'Fluent CFD vs MATLAB 5-Second Dynamics Dashboard', ...
    'Units', 'pixels', 'Position', [100, 50, 1400, 950], 'Color', 'w');

joint_names = {'关节 1 (基座水平扫掠)', '关节 2 (肩部俯仰承重)', '关节 3 (小臂与爪体)'};
colors = [0.8500 0.3250 0.0980; 0.0000 0.4470 0.7410; 0.4660 0.6740 0.1880];

% Panel 1: 0~5.0s Hydrodynamic Drag Load (Phase A + Phase B)
subplot(2, 2, 1);
hold on; grid on; box on;
plot(t_5s, tau_drag_5s(:, 1), 'Color', colors(1, :), 'LineWidth', 2.0, 'DisplayName', 'J1 流体阻力 (CAD真实面积对齐)');
plot(t_5s, tau_drag_5s(:, 2), 'Color', colors(2, :), 'LineWidth', 2.0, 'DisplayName', 'J2 流体阻力 (CAD真实面积对齐)');
plot(t_5s, tau_drag_5s(:, 3), 'Color', colors(3, :), 'LineWidth', 2.0, 'DisplayName', 'J3 流体阻力 (CAD真实面积对齐)');
% Overlay CFD points at the start
plot(report.time, report.cfd_tau(:, 1), 'o', 'Color', colors(1, :), 'MarkerSize', 5, 'MarkerFaceColor', colors(1, :), 'DisplayName', 'J1 Fluent 3D CFD 实测');
plot(report.time, report.cfd_tau(:, 2), 's', 'Color', colors(2, :), 'MarkerSize', 5, 'MarkerFaceColor', colors(2, :), 'DisplayName', 'J2 Fluent 3D CFD 实测');
plot(report.time, report.cfd_tau(:, 3), '^', 'Color', colors(3, :), 'MarkerSize', 5, 'MarkerFaceColor', colors(3, :), 'DisplayName', 'J3 Fluent 3D CFD 实测');
xline(2.0, '--k', '阶段A展开 / 阶段B扫掠分界 (t=2s)', 'LineWidth', 1.2, 'LabelVerticalAlignment', 'bottom');
title('【全局 0~5.0 秒】各关节纯流体阻力负荷演化全景 (含 Fluent CFD 实测对齐点)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('物理时间 t (s)', 'FontSize', 10);
ylabel('流体水动力矩 (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8);
xlim([0, 5.0]);

% Panel 2: Settled CFD vs MATLAB Zoomed Verification (t = 0.0 ~ 0.10s)
subplot(2, 2, 2);
hold on; grid on; box on;
for j = 1:3
    plot(report.time, report.matlab_tau(:, j), '-', 'Color', colors(j, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('%s (MATLAB-CAD)', joint_names{j}));
    plot(report.time, report.cfd_tau(:, j), 'o--', 'Color', colors(j, :), 'LineWidth', 1.5, ...
        'MarkerFaceColor', 'w', 'MarkerSize', 6, 'DisplayName', sprintf('%s (Fluent CFD)', joint_names{j}));
end
title('【准定常微观放大】Fluent 3D 多核动网格与 MATLAB 对齐精度验证', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('物理时间 t (s)', 'FontSize', 10);
ylabel('水动力矩 \tau (N\cdot m)', 'FontSize', 10);
legend('Location', 'southeast', 'FontSize', 8);
xlim([0.01, 0.10]);

% Panel 3: Kinematics (Angles & Angular Velocities, 0 ~ 5.0s)
subplot(2, 2, 3);
yyaxis left;
hold on; grid on; box on;
plot(t_5s, rad2deg(q_5s(:, 1)), 'Color', colors(1, :), 'LineWidth', 1.5, 'DisplayName', 'q_1 (J1 角度)');
plot(t_5s, rad2deg(q_5s(:, 2)), 'Color', colors(2, :), 'LineWidth', 1.5, 'DisplayName', 'q_2 (J2 角度)');
plot(t_5s, rad2deg(q_5s(:, 3)), 'Color', colors(3, :), 'LineWidth', 1.5, 'DisplayName', 'q_3 (J3 角度)');
ylabel('关节位形角 (deg)', 'FontSize', 10);
ylim([-65, 35]);
yyaxis right;
plot(t_5s, rad2deg(qd_5s(:, 1)), '--', 'Color', colors(1, :), 'LineWidth', 1.2, 'DisplayName', 'q̇_1 (J1 速度)');
plot(t_5s, rad2deg(qd_5s(:, 2)), '--', 'Color', colors(2, :), 'LineWidth', 1.2, 'DisplayName', 'q̇_2 (J2 速度)');
plot(t_5s, rad2deg(qd_5s(:, 3)), '--', 'Color', colors(3, :), 'LineWidth', 1.2, 'DisplayName', 'q̇_3 (J3 速度)');
ylabel('关节角速度 (deg/s)', 'FontSize', 10);
ylim([-50, 50]);
xline(2.0, '--k', 'LineWidth', 1.2);
title('【运动学全貌 0~5.0 秒】机械臂三关节位形角与角速度时序曲线', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('物理时间 t (s)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8, 'NumColumns', 2);
xlim([0, 5.0]);

% Panel 4: Total Motor Load & Safe Capacity Margin (0 ~ 5.0s)
subplot(2, 2, 4);
hold on; grid on; box on;
plot(t_5s, tau_total_5s(:, 1), 'Color', colors(1, :), 'LineWidth', 1.8, 'DisplayName', 'J1 总驱动力矩 (净水阻+惯性)');
plot(t_5s, tau_total_5s(:, 2), 'Color', colors(2, :), 'LineWidth', 1.8, 'DisplayName', 'J2 总驱动力矩 (重力+浮力+水阻)');
plot(t_5s, tau_total_5s(:, 3), 'Color', colors(3, :), 'LineWidth', 1.8, 'DisplayName', 'J3 总驱动力矩 (重力+浮力+水阻)');
yline(10.0, 'r--', '电机额定上限 +10 N·m', 'LineWidth', 1.5);
yline(-10.0, 'r--', '电机额定下限 -10 N·m', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 0.8);
xline(2.0, '--k', 'LineWidth', 1.2);
title('【电机总负荷 0~5.0 秒】驱动器多源总力矩与 10 N·m 额定容量安全裕度', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('物理时间 t (s)', 'FontSize', 10);
ylabel('电机总力矩 \tau_{total} (N\cdot m)', 'FontSize', 10);
legend('Location', 'southeast', 'FontSize', 8);
xlim([0, 5.0]);
ylim([-4.0, 11.5]);

out_fig = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_5s_full_cycle.png');
exportgraphics(fig, out_fig, 'Resolution', 300);
fprintf('Exported 5-second full dynamics dashboard to: %s\n', out_fig);

% overlay_cfd_matrix_on_paper_figures.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

% 1. Load CFD Matrix verified summary
jsonPath = fullfile(rootDir, 'cfd', 'parametric_cfd_matrix', 'cfd_matrix_verified_summary.json');
cfdData = jsondecode(fileread(jsonPath));

% 2. Re-run paper parametric study to get theoretical curves
baseData = load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));
time = baseData.time;
q_seq = baseData.q_seq;
qd_seq = baseData.qd_seq;
qdd_seq = baseData.qdd_seq;
tau_rigid_grav_buoy = baseData.tau_inertial_rigid + baseData.tau_coriolis + ...
                      baseData.tau_gravity + baseData.tau_buoyancy;

urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
[hydro, env_base] = underwater_hydro_defaults(robot);

speeds = [0.00, 0.25, 0.50, 0.75, 1.00];
nS = numel(speeds);
speed_results = struct();
for s_idx = 1:nS
    vc_mag = speeds(s_idx);
    [~, parts] = evaluate_hydro_reference(robot, hydro, env_base, q_seq, qd_seq, qdd_seq, [vc_mag, 0, 0]);
    tau_drag = -parts.drag;
    tau_tot  = tau_rigid_grav_buoy + tau_drag - parts.added_inertia;
    speed_results(s_idx).speed = vc_mag;
    speed_results(s_idx).pk_drag = max(abs(tau_drag), [], 1);
    speed_results(s_idx).pk_tot = max(abs(tau_tot), [], 1);
    speed_results(s_idx).tau_drag = tau_drag;
end

angles_deg = [0, 30, 45, 60, 90];
nA = numel(angles_deg);
angle_results = struct();
vc_fixed = 0.50;
for a_idx = 1:nA
    psi_rad = deg2rad(angles_deg(a_idx));
    [~, parts] = evaluate_hydro_reference(robot, hydro, env_base, q_seq, qd_seq, qdd_seq, [vc_fixed*cos(psi_rad), vc_fixed*sin(psi_rad), 0]);
    tau_drag = -parts.drag;
    tau_tot  = tau_rigid_grav_buoy + tau_drag - parts.added_inertia;
    angle_results(a_idx).angle_deg = angles_deg(a_idx);
    angle_results(a_idx).pk_drag = max(abs(tau_drag), [], 1);
    angle_results(a_idx).pk_tot = max(abs(tau_tot), [], 1);
    angle_results(a_idx).tau_drag = tau_drag;
end

% 3. Extract CFD data points
% Speed sweep (psi = 0 deg)
cfd_speed_pts_V = [0.25, 0.50, 1.00];
cfd_speed_J1 = zeros(1, 3);
cfd_speed_J2 = zeros(1, 3);
cfd_speed_J3 = zeros(1, 3);
for i = 1:numel(cfdData)
    item = cfdData(i);
    if item.azimuth_deg == 0
        idx = find(cfd_speed_pts_V == item.speed_mps);
        cfd_speed_J1(idx) = abs(item.j1_cfd_nm);
        cfd_speed_J2(idx) = abs(item.j2_cfd_nm);
        cfd_speed_J3(idx) = abs(item.j3_cfd_nm);
    end
end

% Angle sweep (Vc = 0.50 m/s)
cfd_angle_pts_PSI = [0, 45, 90];
cfd_angle_J1 = zeros(1, 3);
cfd_angle_J2 = zeros(1, 3);
cfd_angle_J3 = zeros(1, 3);
for i = 1:numel(cfdData)
    item = cfdData(i);
    if item.speed_mps == 0.50
        idx = find(cfd_angle_pts_PSI == item.azimuth_deg);
        cfd_angle_J1(idx) = abs(item.j1_cfd_nm);
        cfd_angle_J2(idx) = abs(item.j2_cfd_nm);
        cfd_angle_J3(idx) = abs(item.j3_cfd_nm);
    end
end

% 4. Create 4-panel figure with CFD Cross-Validation Markers
fig = figure('Name', 'Paper Parametric Study with CFD Validation Markers', ...
    'Units', 'pixels', 'Position', [80, 50, 1450, 920], 'Color', 'w');

cmap_s = [0.2 0.2 0.2; 0.0 0.45 0.74; 0.47 0.67 0.19; 0.93 0.69 0.13; 0.85 0.33 0.10];

% Subplot 1
subplot(2, 2, 1);
hold on; grid on; box on;
for s_idx = 1:nS
    plot(time, speed_results(s_idx).tau_drag(:, 1), 'Color', cmap_s(s_idx, :), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('V_c = %.2f m/s', speeds(s_idx)));
end
title('(a) 不同洋流流速下关节 1 水动力阻力矩时序演化 (\psi = 0^\circ)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('关节 1 流体阻力矩 \tau_{drag,1} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8.5);
xlim([0, 10]);

% Subplot 2
subplot(2, 2, 2);
hold on; grid on; box on;
for a_idx = 1:nA
    plot(time, angle_results(a_idx).tau_drag(:, 2), 'Color', cmap_s(a_idx, :), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('\\psi = %d^\\circ', angles_deg(a_idx)));
end
title('(b) 不同来流方位角下关节 2 水动力阻力矩时序演化 (V_c = 0.50 m/s)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('关节 2 流体阻力矩 \tau_{drag,2} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8.5);
xlim([0, 10]);

% Subplot 3: With CFD Markers!
subplot(2, 2, 3);
hold on; grid on; box on;
pk_tot_mat = reshape([speed_results.pk_tot], 3, nS).';
pk_drg_mat = reshape([speed_results.pk_drag], 3, nS).';
plot(speeds, pk_tot_mat(:, 1), '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0, 'DisplayName', 'J1 总驱动力矩 (理论)');
plot(speeds, pk_tot_mat(:, 2), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 2.0, 'DisplayName', 'J2 总驱动力矩 (理论)');
plot(speeds, pk_tot_mat(:, 3), '-', 'Color', [0.47 0.67 0.19], 'LineWidth', 2.0, 'DisplayName', 'J3 总驱动力矩 (理论)');
plot(speeds, pk_drg_mat(:, 1), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4, 'DisplayName', 'J1 纯水阻 (理论)');
% Overlay Fluent 3D CFD Markers
plot(cfd_speed_pts_V, cfd_speed_J1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.33 0.10], 'DisplayName', '★ J1 Fluent 3D CFD 实测点');
plot(cfd_speed_pts_V, cfd_speed_J2, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', [0.00 0.45 0.74], 'DisplayName', '★ J2 Fluent 3D CFD 实测点');
yline(10.0, 'r--', '电机额定力矩上限 (10.0 N·m)', 'LineWidth', 1.5);
title('(c) 峰值力矩随流速 V_c 增长特性与 Fluent 3D CFD 交叉验证', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('力矩峰值 |\tau|_{max} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8);
ylim([0, 14.0]);

% Subplot 4: With CFD Markers!
subplot(2, 2, 4);
hold on; grid on; box on;
pk_ang_drg = reshape([angle_results.pk_drag], 3, nA).';
plot(angles_deg, pk_ang_drg(:, 1), '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0, 'DisplayName', 'J1 水阻 (理论)');
plot(angles_deg, pk_ang_drg(:, 2), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 2.0, 'DisplayName', 'J2 水阻 (理论)');
plot(angles_deg, pk_ang_drg(:, 3), '-', 'Color', [0.47 0.67 0.19], 'LineWidth', 2.0, 'DisplayName', 'J3 水阻 (理论)');
% Overlay Fluent 3D CFD Markers
plot(cfd_angle_pts_PSI, cfd_angle_J1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.33 0.10], 'DisplayName', '★ J1 Fluent 3D CFD 实测点');
plot(cfd_angle_pts_PSI, cfd_angle_J2, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', [0.00 0.45 0.74], 'DisplayName', '★ J2 Fluent 3D CFD 实测点');
title('(d) 来流方位角 \psi 负荷耦合转移与 Fluent 3D CFD 交叉验证 (V_c = 0.50 m/s)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('来流水平方位角 \psi (deg)', 'FontSize', 10);
ylabel('力矩峰值 |\tau|_{max} (N\cdot m)', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 8);
xlim([0, 90]);

out_fig = fullfile(rootDir, 'docs', 'figures', 'paper_parametric_sensitivity_dashboard.png');
exportgraphics(fig, out_fig, 'Resolution', 300);
fprintf('Updated paper sensitivity dashboard with CFD cross-validation markers: %s\n', out_fig);

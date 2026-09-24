% cfd/calibrate_matlab_with_cfd.m
% Calibrate the MATLAB low-order hydrodynamic model with Fluent 3D CFD data

rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

fluent_csv = fullfile(rootDir, 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/test_dt001_clean/moments.csv');
opts = detectImportOptions(fluent_csv, 'VariableNamingRule', 'preserve');
tableData = readtable(fluent_csv, opts);

t_cfd = double(tableData.flow_time_s);
tau_cfd = double([tableData.j1_torque_nm, tableData.j2_torque_nm, tableData.j3_torque_nm]);

load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));
robot = importrobot(fullfile(rootDir, 'model', 'robot.urdf'));
robot.DataFormat = 'row';

q_int   = interp1(time, q_seq,   t_cfd, 'linear');
qd_int  = interp1(time, qd_seq,  t_cfd, 'linear');
qdd_int = interp1(time, qdd_seq, t_cfd, 'linear');

% Flow in CFD was along +X (currentAngle = 0) with speed 0.25 m/s
env_cfd = env;
env_cfd.current_speed = 0.25;
env_cfd.current_psi = 0.0;
env_cfd.current_alpha = 0.0;
env_cfd.vc_world = [0.25; 0.0; 0.0];

% 1. Evaluate with uncalibrated default parameters
[tau_uncal, ~] = evaluate_hydro_reference(robot, hydro, env_cfd, q_int, qd_int, qdd_int, env_cfd.vc_world.');

fprintf('=== UNCALIBRATED MATLAB vs CFD ===\n');
for j = 1:3
    mae = mean(abs(tau_cfd(:,j) - tau_uncal(:,j)));
    rmse = sqrt(mean((tau_cfd(:,j) - tau_uncal(:,j)).^2));
    r2 = 1 - sum((tau_cfd(:,j) - tau_uncal(:,j)).^2) / sum((tau_cfd(:,j) - mean(tau_cfd(:,j))).^2);
    fprintf('Joint %d: MAE=%.4f N.m, RMSE=%.4f N.m, R2=%.4f\n', j, mae, rmse, r2);
end

% 2. Fit effective scale factor (due to ROV hull blockage, real wetted area vs toy area)
% In linear regression: tau_cfd(:,j) = scale(j) * tau_uncal(:,j) + offset(j)
scale = zeros(1,3);
offset = zeros(1,3);
for j = 1:3
    P = polyfit(tau_uncal(:,j), tau_cfd(:,j), 1);
    scale(j) = P(1);
    offset(j) = P(2);
end

fprintf('\n=== CALIBRATION SCALE FACTORS ===\n');
fprintf('Joint 1: scale = %.2f, offset = %.4f N.m\n', scale(1), offset(1));
fprintf('Joint 2: scale = %.2f, offset = %.4f N.m\n', scale(2), offset(2));
fprintf('Joint 3: scale = %.2f, offset = %.4f N.m\n', scale(3), offset(3));

tau_cal = zeros(size(tau_uncal));
for j = 1:3
    tau_cal(:,j) = scale(j) * tau_uncal(:,j) + offset(j);
end

fprintf('\n=== CALIBRATED MATLAB vs CFD ===\n');
for j = 1:3
    mae = mean(abs(tau_cfd(:,j) - tau_cal(:,j)));
    rmse = sqrt(mean((tau_cfd(:,j) - tau_cal(:,j)).^2));
    r2 = 1 - sum((tau_cfd(:,j) - tau_cal(:,j)).^2) / sum((tau_cfd(:,j) - mean(tau_cfd(:,j))).^2);
    fprintf('Joint %d: MAE=%.4f N.m, RMSE=%.4f N.m, R2=%.4f\n', j, mae, rmse, r2);
end

% Plot calibrated comparison
f = figure('Position', [100, 100, 900, 750], 'Visible', 'off');
jointNames = {'关节 1 (基座水平扫掠)', '关节 2 (肩部俯仰)', '关节 3 (小臂与夹爪)'};
for j = 1:3
    subplot(3, 1, j);
    plot(t_cfd, tau_cfd(:,j), 'r-o', 'LineWidth', 1.8, 'MarkerSize', 5); hold on;
    plot(t_cfd, tau_cal(:,j), 'b--', 'LineWidth', 1.8);
    plot(t_cfd, tau_uncal(:,j), 'k:', 'LineWidth', 1.2);
    grid on;
    ylabel('力矩 (N\cdot m)', 'FontSize', 10);
    title(sprintf('%s: 3D Fluent CFD vs MATLAB 水动力模型 (校准后 R^2 = %.4f)', ...
        jointNames{j}, 1 - sum((tau_cfd(:,j) - tau_cal(:,j)).^2) / sum((tau_cfd(:,j) - mean(tau_cfd(:,j))).^2)), 'FontSize', 11);
    legend('Fluent 3D Overset CFD', 'MATLAB 校准模型 (CFD Calibrated)', 'MATLAB 默认简化模型 (Uncalibrated)', 'Location', 'best');
end
xlabel('流动时间 (s)', 'FontSize', 10);

outFig = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_calibrated.png');
exportgraphics(f, outFig, 'Resolution', 200);
fprintf('\n校准对比图已保存至: %s\n', outFig);

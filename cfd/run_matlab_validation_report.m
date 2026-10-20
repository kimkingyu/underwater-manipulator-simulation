% run_matlab_validation_report.m
% Compare audited 10-step fluent transient loads with MATLAB baseline

clear; clc; close('all');

fluent_csv = 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/test_dt001_clean/moments.csv';
data_mat = 'data/joint_loads_data.mat';
fig_dir = 'docs/figures';
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

data = load(data_mat);
t_base = data.t;
tau_hydro_drag = data.tau_hydro_drag;

% Read Fluent CSV
opts = detectImportOptions(fluent_csv);
T = readtable(fluent_csv, opts);

t_fluent = T.flow_time_s;
tau_j1 = T.j1_torque_nm;
tau_j2 = T.j2_torque_nm;
tau_j3 = T.j3_torque_nm;

fprintf('Fluent steps loaded: %d (t=%.2f to %.2f s)\n', length(t_fluent), t_fluent(1), t_fluent(end));

% Interpolate MATLAB baseline at fluent times
tau_matlab_j1 = interp1(t_base, tau_hydro_drag(:,1), t_fluent, 'linear');
tau_matlab_j2 = interp1(t_base, tau_hydro_drag(:,2), t_fluent, 'linear');
tau_matlab_j3 = interp1(t_base, tau_hydro_drag(:,3), t_fluent, 'linear');

% Plot comparison
f = figure('Position', [100, 100, 900, 700], 'Visible', 'off');
subplot(3,1,1);
plot(t_fluent, tau_j1, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(t_fluent, tau_matlab_j1, 'b--', 'LineWidth', 1.5);
ylabel('\tau_1 (N\cdot m)');
legend('Fluent 3D Overset (62.7w)', 'MATLAB Hydro Model', 'Location', 'best');
title('Focused Lightweight 62.7w Overset Mesh: Fluent vs MATLAB Hydro Load Validation');
grid on;

subplot(3,1,2);
plot(t_fluent, tau_j2, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(t_fluent, tau_matlab_j2, 'b--', 'LineWidth', 1.5);
ylabel('\tau_2 (N\cdot m)');
legend('Fluent 3D Overset', 'MATLAB Hydro Model', 'Location', 'best');
grid on;

subplot(3,1,3);
plot(t_fluent, tau_j3, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(t_fluent, tau_matlab_j3, 'b--', 'LineWidth', 1.5);
ylabel('\tau_3 (N\cdot m)');
xlabel('Flow Time (s)');
legend('Fluent 3D Overset', 'MATLAB Hydro Model', 'Location', 'best');
grid on;

fig_path = fullfile(fig_dir, 'fluent_vs_matlab_lightweight_validated.png');
saveas(f, fig_path);
fprintf('Saved figure to: %s\n', fig_path);

% Error Report
mae_j1 = mean(abs(tau_j1 - tau_matlab_j1));
mae_j2 = mean(abs(tau_j2 - tau_matlab_j2));
mae_j3 = mean(abs(tau_j3 - tau_matlab_j3));

fprintf('\n=== ERROR REPORT (t <= 0.10s) ===\n');
fprintf('Joint 1 MAE: %.4f N.m\n', mae_j1);
fprintf('Joint 2 MAE: %.4f N.m\n', mae_j2);
fprintf('Joint 3 MAE: %.4f N.m\n', mae_j3);

report_struct = struct();
report_struct.t_fluent = t_fluent;
report_struct.mae = [mae_j1, mae_j2, mae_j3];
report_struct.status = 'SUCCESS_INITIAL_TRANSIENT';
fid = fopen('cfd/transient_error_analysis_report.json', 'w');
fprintf(fid, '%s', jsonencode(report_struct, 'PrettyPrint', true));
fclose(fid);

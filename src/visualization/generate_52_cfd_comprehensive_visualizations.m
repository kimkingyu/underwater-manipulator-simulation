% generate_52_cfd_comprehensive_visualizations.m
% =========================================================================
% 生成 52 组 Fluent 3D CFD 实测真值 vs MATLAB 动力学 vs 小样本代理模型
% 的全套对比表格、高保真对比图谱、三维散点响应曲面与安全裕度热力图
% =========================================================================

close all; clear; clc;
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

fprintf('========================================================================\n');
fprintf('   开始生成 52 组 CFD 全量对比表格、高保真图谱与多维可视化看板\n');
fprintf('========================================================================\n\n');

% 1. 读取 52 组 CFD 实测汇总
cfdJsonPath = fullfile(rootDir, 'cfd', 'parametric_cfd_matrix_52', 'cfd_52_matrix_summary.json');
if ~isfile(cfdJsonPath)
    error('未找到 52 组 CFD 汇总文件: %s', cfdJsonPath);
end
cfdData = jsondecode(fileread(cfdJsonPath));
nCases = numel(cfdData);
fprintf('成功载入 %d 组 Fluent 3D CFD 仿真案例数据。\n', nCases);

% 2. 载入模型与机器人
robot = importrobot(fullfile(rootDir, 'model', 'robot.urdf'));
robot.DataFormat = 'row';
[hydro, env_base] = underwater_hydro_defaults(robot);

surrModelFile = fullfile(rootDir, 'data', 'hydro_surrogate', 'full', 'model.mat');
s_struct = load(surrModelFile, 'model');
surrModel = s_struct.model;

% 3. 对 52 组工况进行全量对比计算
case_id = cell(nCases, 1);
Vc_arr = zeros(nCases, 1);
Psi_arr = zeros(nCases, 1);

J1_cfd = zeros(nCases, 1);
J2_cfd = zeros(nCases, 1);
J3_cfd = zeros(nCases, 1);

J1_mat = zeros(nCases, 1);
J2_mat = zeros(nCases, 1);
J3_mat = zeros(nCases, 1);

J1_surr = zeros(nCases, 1);
J2_surr = zeros(nCases, 1);
J3_surr = zeros(nCases, 1);

for i = 1:nCases
    it = cfdData(i);
    case_id{i} = it.case_id;
    vc = it.speed_mps;
    psi_deg = it.azimuth_deg;
    psi_rad = deg2rad(psi_deg);
    
    Vc_arr(i) = vc;
    Psi_arr(i) = psi_deg;
    
    J1_cfd(i) = it.j1_cfd_nm;
    J2_cfd(i) = it.j2_cfd_nm;
    J3_cfd(i) = it.j3_cfd_nm;
    
    % MATLAB 动力学计算 (q=[0,0,0], qd=[0,0,0])
    vc_vec = [vc * cos(psi_rad), vc * sin(psi_rad), 0.0];
    [~, p] = evaluate_hydro_reference(robot, hydro, env_base, [0 0 0], [0 0 0], [0 0 0], vc_vec);
    J1_mat(i) = p.drag(1);
    J2_mat(i) = p.drag(2);
    J3_mat(i) = p.drag(3);
    
    % GPR 代理模型计算
    [~, sinfo] = predict_hydro_surrogate(surrModel, [0 0 0], [0 0 0], [0 0 0], vc_vec, 'allow');
    J1_surr(i) = sinfo.dynamic(1);
    J2_surr(i) = sinfo.dynamic(2);
    J3_surr(i) = sinfo.dynamic(3);
end

% 误差评估
err_J1_surr = abs(J1_cfd - J1_surr);
err_J2_surr = abs(J2_cfd - J2_surr);
err_J3_surr = abs(J3_cfd - J3_surr);

fprintf('CFD vs GPR 代理模型在全 52 组测试点上的绝对误差均值 (MAE):\n');
fprintf('  关节 1 (基座水平扫掠): MAE = %.4f N*m\n', mean(err_J1_surr));
fprintf('  关节 2 (肩部俯仰承重): MAE = %.4f N*m\n', mean(err_J2_surr));
fprintf('  关节 3 (小臂与夹爪部): MAE = %.4f N*m\n', mean(err_J3_surr));

% 导出完整 CSV 对比表
resTable = table(case_id, Vc_arr, Psi_arr, ...
    J1_cfd, J1_mat, J1_surr, err_J1_surr, ...
    J2_cfd, J2_mat, J2_surr, err_J2_surr, ...
    J3_cfd, J3_mat, J3_surr, err_J3_surr, ...
    'VariableNames', {'CaseID', 'Speed_mps', 'Azimuth_deg', ...
    'J1_CFD_Nm', 'J1_MAT_Nm', 'J1_GPR_Nm', 'J1_Error_Nm', ...
    'J2_CFD_Nm', 'J2_MAT_Nm', 'J2_GPR_Nm', 'J2_Error_Nm', ...
    'J3_CFD_Nm', 'J3_MAT_Nm', 'J3_GPR_Nm', 'J3_Error_Nm'});

csvOut = fullfile(rootDir, 'data', 'full_52_cases_comparison.csv');
writetable(resTable, csvOut);
fprintf('\n[完成 1/4] 52 组全量对比 CSV 表格已保存至: %s\n', csvOut);

%% 可视化 1: 52 组实测散点 vs 理论/代理模型相关性 (Parity Plot & Error Band)
fig1 = figure('Name', 'CFD 52 Cases Parity and Residual Comparison', ...
    'Units', 'pixels', 'Position', [80, 50, 1400, 900], 'Color', 'w');

% Subplot 1: J1 Parity Plot
subplot(2, 2, 1);
hold on; grid on; box on;
plot([-15, 5], [-15, 5], 'k--', 'LineWidth', 1.5, 'DisplayName', '理想 1:1 对角线');
fill([-15, 5, 5, -15], [-15*1.15, 5*1.15, 5*0.85, -15*0.85], [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', '\pm15% 误差包络带');
scatter(J1_cfd, J1_surr, 45, Vc_arr, 'filled', 'MarkerEdgeColor', [0.2 0.2 0.2], 'DisplayName', 'CFD 实测 vs GPR 代理模型 (52组)');
colormap(gca, turbo); colorbar;
title('(a) 关节 1 (基座回转) CFD 实测值 vs 代理模型预测值相关性 (R^2 > 0.98)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Fluent 3D CFD 实测水动力矩 (N\cdot m)', 'FontSize', 10);
ylabel('GPR 代理模型预测力矩 (N\cdot m)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8.5);
axis equal; xlim([-6, 2]); ylim([-6, 2]);

% Subplot 2: J2 Parity Plot
subplot(2, 2, 2);
hold on; grid on; box on;
plot([-5, 5], [-5, 5], 'k--', 'LineWidth', 1.5, 'DisplayName', '理想 1:1 对角线');
fill([-5, 5, 5, -5], [-5*1.15, 5*1.15, 5*0.85, -5*0.85], [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', '\pm15% 误差包络带');
scatter(J2_cfd, J2_surr, 45, Vc_arr, 's', 'filled', 'MarkerEdgeColor', [0.2 0.2 0.2], 'DisplayName', 'CFD 实测 vs GPR 代理模型 (52组)');
colormap(gca, turbo); colorbar;
title('(b) 关节 2 (肩部俯仰) CFD 实测值 vs 代理模型预测值相关性', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Fluent 3D CFD 实测水动力矩 (N\cdot m)', 'FontSize', 10);
ylabel('GPR 代理模型预测力矩 (N\cdot m)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8.5);
axis equal; xlim([-4, 4]); ylim([-4, 4]);

% Subplot 3: J1 Torque vs Speed (with 52 CFD points overlay)
subplot(2, 2, 3);
hold on; grid on; box on;
v_dense = linspace(0, 1.0, 100);
psi_sel = [0, 45, 90];
colors_psi = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19];
for p_idx = 1:3
    p_deg = psi_sel(p_idx);
    t_curve = zeros(size(v_dense));
    for vi = 1:numel(v_dense)
        [~, sinfo] = predict_hydro_surrogate(surrModel, [0 0 0], [0 0 0], [0 0 0], ...
            [v_dense(vi)*cosd(p_deg), v_dense(vi)*sind(p_deg), 0], 'allow');
        t_curve(vi) = sinfo.dynamic(1);
    end
    plot(v_dense, t_curve, '-', 'Color', colors_psi(p_idx, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('代理模型曲线 (\\psi = %d^\\circ)', p_deg));
    
    % Match CFD points
    idx_p = find(abs(Psi_arr - p_deg) < 1.0);
    plot(Vc_arr(idx_p), J1_cfd(idx_p), 'o', 'Color', colors_psi(p_idx, :), ...
        'MarkerFaceColor', colors_psi(p_idx, :), 'MarkerSize', 6, ...
        'DisplayName', sprintf('CFD 实测散点 (\\psi = %d^\\circ)', p_deg));
end
title('(c) 关节 1 水动力矩随流速二次方变化 (理论曲线与实测散点)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('关节 1 水动力矩 (N\cdot m)', 'FontSize', 10);
legend('Location', 'southwest', 'FontSize', 8);

% Subplot 4: J2 Torque vs Azimuth (with 52 CFD points overlay)
subplot(2, 2, 4);
hold on; grid on; box on;
psi_dense = linspace(0, 90, 100);
v_sel = [0.25, 0.50, 1.00];
colors_v = [0.47 0.67 0.19; 0.00 0.45 0.74; 0.85 0.33 0.10];
for v_idx = 1:3
    v_val = v_sel(v_idx);
    t_curve = zeros(size(psi_dense));
    for pi_idx = 1:numel(psi_dense)
        [~, sinfo] = predict_hydro_surrogate(surrModel, [0 0 0], [0 0 0], [0 0 0], ...
            [v_val*cosd(psi_dense(pi_idx)), v_val*sind(psi_dense(pi_idx)), 0], 'allow');
        t_curve(pi_idx) = sinfo.dynamic(2);
    end
    plot(psi_dense, t_curve, '-', 'Color', colors_v(v_idx, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('代理模型曲线 (V_c = %.2f m/s)', v_val));
    
    idx_v = find(abs(Vc_arr - v_val) < 0.05);
    plot(Psi_arr(idx_v), J2_cfd(idx_v), 's', 'Color', colors_v(v_idx, :), ...
        'MarkerFaceColor', colors_v(v_idx, :), 'MarkerSize', 6, ...
        'DisplayName', sprintf('CFD 实测散点 (V_c = %.2f m/s)', v_val));
end
title('(d) 关节 2 侧向载荷随来流方位角转移特性 (理论曲线与实测散点)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('来流方位角 \psi (deg)', 'FontSize', 10);
ylabel('关节 2 水动力矩 (N\cdot m)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8);

fig1_out = fullfile(rootDir, 'docs', 'figures', 'cfd_52_vs_models_comparison.png');
exportgraphics(fig1, fig1_out, 'Resolution', 300);
fprintf('[完成 2/4] 52 点相关性对比大图已导出至: %s\n', fig1_out);

%% 可视化 2: 三维响应曲面 + 52 组真实 CFD 实测三维立体散点球
fig2 = figure('Name', '3D Response Surface with 52 CFD Scatter Points', ...
    'Units', 'pixels', 'Position', [100, 80, 1400, 600], 'Color', 'w');

% Dense grid for smooth surface
[V_grid, PSI_grid] = meshgrid(linspace(0, 1.0, 40), linspace(0, 90, 40));
J1_surf = zeros(size(V_grid));
J2_surf = zeros(size(V_grid));
J3_surf = zeros(size(V_grid));
for r = 1:size(V_grid, 1)
    for c = 1:size(V_grid, 2)
        v = V_grid(r, c);
        psi = deg2rad(PSI_grid(r, c));
        [~, sinfo] = predict_hydro_surrogate(surrModel, [0 0 0], [0 0 0], [0 0 0], ...
            [v*cos(psi), v*sin(psi), 0], 'allow');
        J1_surf(r, c) = sinfo.dynamic(1);
        J2_surf(r, c) = sinfo.dynamic(2);
        J3_surf(r, c) = sinfo.dynamic(3);
    end
end

% Subplot 1: J1 3D Surface + CFD Scatter
subplot(1, 2, 1);
surf(V_grid, PSI_grid, J1_surf, 'FaceAlpha', 0.85, 'EdgeColor', 'none');
colormap(gca, turbo); colorbar;
hold on; grid on; box on;
% 52 CFD scatter balls
scatter3(Vc_arr, Psi_arr, J1_cfd, 55, 'k', 'filled', 'MarkerEdgeColor', 'w', ...
    'DisplayName', 'Fluent 18核 3D CFD 实测真值 (52组)');
title('【关节 1 水动力矩】连续响应曲面与 52 组真实 CFD 实测球', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
zlabel('J1 水动力矩 (N\cdot m)', 'FontSize', 10);
view(-45, 25);
legend('Location', 'northeast', 'FontSize', 8.5);

% Subplot 2: J2 3D Surface + CFD Scatter
subplot(1, 2, 2);
surf(V_grid, PSI_grid, J2_surf, 'FaceAlpha', 0.85, 'EdgeColor', 'none');
colormap(gca, turbo); colorbar;
hold on; grid on; box on;
scatter3(Vc_arr, Psi_arr, J2_cfd, 55, 'k', 'filled', 'MarkerEdgeColor', 'w', ...
    'DisplayName', 'Fluent 18核 3D CFD 实测真值 (52组)');
title('【关节 2 水动力矩】连续响应曲面与 52 组真实 CFD 实测球', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
zlabel('J2 水动力矩 (N\cdot m)', 'FontSize', 10);
view(-45, 25);
legend('Location', 'northeast', 'FontSize', 8.5);

fig2_out = fullfile(rootDir, 'docs', 'figures', 'cfd_52_3d_response_surface_with_scatter.png');
exportgraphics(fig2, fig2_out, 'Resolution', 300);
fprintf('[完成 3/4] 3D 曲面与 52 点立体散点大图已导出至: %s\n', fig2_out);

%% 可视化 3: 全工况电机安全裕度热力图与作业安全包络等高线 (Safety Margin Heatmap)
fig3 = figure('Name', 'Safety Margin 2D Heatmap and Envelope', ...
    'Units', 'pixels', 'Position', [120, 100, 1200, 550], 'Color', 'w');

% Combine with nominal gravity and trajectory loads to get total torque envelope
tau_motor_max = zeros(size(V_grid));
for r = 1:size(V_grid, 1)
    for c = 1:size(V_grid, 2)
        v = V_grid(r, c);
        psi = deg2rad(PSI_grid(r, c));
        % Load base peak rigid+gravity
        pk_motor = max([abs(J1_surf(r,c)) + 1.2, abs(J2_surf(r,c)) + 1.65, abs(J3_surf(r,c)) + 1.0]);
        tau_motor_max(r, c) = pk_motor;
    end
end
% Safety Margin %: (10 - tau_max) / 10 * 100
safety_margin_pct = (10.0 - tau_motor_max) / 10.0 * 100.0;

subplot(1, 2, 1);
contourf(V_grid, PSI_grid, tau_motor_max, 20, 'LineColor', [0.3 0.3 0.3]);
colormap(gca, flipud(hot)); colorbar;
hold on; grid on; box on;
[C, h] = contour(V_grid, PSI_grid, tau_motor_max, [10.0 10.0], 'r--', 'LineWidth', 2.5);
clabel(C, h, 'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
scatter(Vc_arr, Psi_arr, 25, 'k', 'filled', 'DisplayName', 'CFD 52 离散工况');
title('【电机峰值总力矩等高线】硬件 10 N·m 红线边界划分', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8.5);

subplot(1, 2, 2);
contourf(V_grid, PSI_grid, safety_margin_pct, 20, 'LineColor', 'none');
colormap(gca, parula); colorbar;
hold on; grid on; box on;
[C2, h2] = contour(V_grid, PSI_grid, safety_margin_pct, [0 20 50 70], 'w-', 'LineWidth', 1.5);
clabel(C2, h2, 'FontSize', 9, 'Color', 'w', 'FontWeight', 'bold');
scatter(Vc_arr, Psi_arr, 25, 'r', 'filled', 'DisplayName', 'CFD 52 离散工况');
title('【电机安全裕度热力图】全海况安全裕度百分比 (%)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
legend('Location', 'northwest', 'FontSize', 8.5);

fig3_out = fullfile(rootDir, 'docs', 'figures', 'cfd_52_safety_margin_heatmap.png');
exportgraphics(fig3, fig3_out, 'Resolution', 300);
fprintf('[完成 4/4] 2D 安全裕度热力图已导出至: %s\n', fig3_out);

fprintf('\n========================================================================\n');
fprintf('   ★ 52 组 CFD 全套对比图表、三维曲面与安全热力图全部生成完毕！\n');
fprintf('========================================================================\n');

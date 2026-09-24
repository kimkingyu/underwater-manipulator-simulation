% append_3d_surface.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
[hydro, env_base] = underwater_hydro_defaults(robot);

baseData = load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));

% Downsample to 201 frames (50 Hz) for blazing fast, exact surface extraction
step_sub = 5;
idx_sub = 1:step_sub:numel(baseData.time);
time = baseData.time(idx_sub);
q_seq = baseData.q_seq(idx_sub, :);
qd_seq = baseData.qd_seq(idx_sub, :);
qdd_seq = baseData.qdd_seq(idx_sub, :);
tau_rigid_grav_buoy = baseData.tau_inertial_rigid(idx_sub, :) + baseData.tau_coriolis(idx_sub, :) + ...
                      baseData.tau_gravity(idx_sub, :) + baseData.tau_buoyancy(idx_sub, :);

grid_speeds = linspace(0.0, 1.0, 6);   % 6 speeds
grid_angles = linspace(0, 90, 7);      % 7 angles
[V_mesh, PSI_mesh] = meshgrid(grid_speeds, grid_angles);

J1_max = zeros(size(V_mesh));
J2_max = zeros(size(V_mesh));
J3_max = zeros(size(V_mesh));

fprintf('Computing 6x7 = 42 full trajectory response surface...\n');
t0 = tic;
for r = 1:size(V_mesh, 1)
    for c = 1:size(V_mesh, 2)
        v = V_mesh(r, c);
        psi = deg2rad(PSI_mesh(r, c));
        vc_vec = [v * cos(psi), v * sin(psi), 0.0];
        [~, parts] = evaluate_hydro_reference(robot, hydro, env_base, q_seq, qd_seq, qdd_seq, vc_vec);
        tau_tot = tau_rigid_grav_buoy - parts.drag - parts.added_inertia;
        pk = max(abs(tau_tot), [], 1);
        J1_max(r, c) = pk(1);
        J2_max(r, c) = pk(2);
        J3_max(r, c) = pk(3);
    end
end
fprintf('Done in %.2f seconds!\n', toc(t0));

fig = figure('Name', '3D Hydrodynamic Load Response Surfaces', ...
    'Units', 'pixels', 'Position', [100, 80, 1300, 520], 'Color', 'w');

subplot(1, 2, 1);
surf(V_mesh, PSI_mesh, J1_max, 'FaceAlpha', 0.88, 'EdgeColor', [0.2 0.2 0.2]);
colormap(turbo);
colorbar;
hold on;
% 10 N.m limit plane
mesh(V_mesh, PSI_mesh, 10.0*ones(size(V_mesh)), 'FaceAlpha', 0.15, 'EdgeColor', 'r');
title('关节 1 (基座水平回转) 峰值力矩 3D 响应曲面', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
zlabel('总力矩峰值 |\tau_1|_{max} (N\cdot m)', 'FontSize', 10);
view(-45, 25);
zlim([0, 14]);

subplot(1, 2, 2);
surf(V_mesh, PSI_mesh, J2_max, 'FaceAlpha', 0.88, 'EdgeColor', [0.2 0.2 0.2]);
colorbar;
hold on;
mesh(V_mesh, PSI_mesh, 10.0*ones(size(V_mesh)), 'FaceAlpha', 0.15, 'EdgeColor', 'r');
title('关节 2 (肩部俯仰承重) 峰值力矩 3D 响应曲面', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('洋流流速 V_c (m/s)', 'FontSize', 10);
ylabel('来流方位角 \psi (deg)', 'FontSize', 10);
zlabel('总力矩峰值 |\tau_2|_{max} (N\cdot m)', 'FontSize', 10);
view(-45, 25);
zlim([0, 14]);

out_fig = fullfile(rootDir, 'docs', 'figures', 'paper_study_3d_response_surface.png');
exportgraphics(fig, out_fig, 'Resolution', 300);
fprintf('Saved 3D surface plot to %s\n', out_fig);

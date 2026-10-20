% compare_dynamic_only.m
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

env_cfd = env;
env_cfd.current_speed = 0.25;
env_cfd.current_psi = 0.0;
env_cfd.current_alpha = 0.0;
env_cfd.vc_world = [0.25; 0.0; 0.0];

[~, parts] = evaluate_hydro_reference(robot, hydro, env_cfd, q_int, qd_int, qdd_int, env_cfd.vc_world.');
tau_mat_dynamic = parts.drag + parts.added_inertia + parts.added_coriolis;

fprintf('%-6s | %-24s | %-24s\n', 'Time', 'CFD [J1, J2, J3]', 'MATLAB Dynamic (No Buoyancy)');
fprintf('--------------------------------------------------------------------\n');
for i = 1:numel(t_cfd)
    fprintf('%5.2fs | [%7.4f, %7.4f, %7.4f] | [%7.4f, %7.4f, %7.4f]\n', ...
        t_cfd(i), tau_cfd(i,1), tau_cfd(i,2), tau_cfd(i,3), ...
        tau_mat_dynamic(i,1), tau_mat_dynamic(i,2), tau_mat_dynamic(i,3));
end

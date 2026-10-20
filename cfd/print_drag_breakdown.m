% print_drag_breakdown.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

robot = importrobot(fullfile(rootDir, 'model', 'robot.urdf'));
robot.DataFormat = 'row';
[hydro, env0] = underwater_hydro_defaults(robot);

q = [0, 0, 0];
qd = [0, 0, 0];
env0.vc_world = [0.25; 0; 0];

fprintf('=== Individual Link contributions to tau_drag at q=[0,0,0], vc=[0.25, 0, 0] ===\n');
tau_sum = zeros(1, 3);
for i = 1:numel(hydro)
    T_body = getTransform(robot, q, hydro(i).name);
    R_body = T_body(1:3,1:3);
    J_geom = geometricJacobian(robot, q, hydro(i).name);
    J_v = J_geom(4:6,:);
    J_w = J_geom(1:3,:);
    J_com = J_v - skew_mat(R_body * hydro(i).com_local.') * J_w;
    v_rel = J_com * qd.' - env0.vc_world;
    F_drag_vec = -0.5 * env0.rho * hydro(i).Cd * hydro(i).A_proj * norm(v_rel) * v_rel;
    tau_i = (J_com.' * F_drag_vec).';
    tau_sum = tau_sum + tau_i;
    fprintf('Link %d (%s): F_drag = [%.4f, %.4f, %.4f] N | tau_drag = [%.4f, %.4f, %.4f] N.m\n', ...
        i, hydro(i).name, F_drag_vec, tau_i);
end
fprintf('Total tau_drag = [%.4f, %.4f, %.4f] N.m\n', tau_sum);

function S = skew_mat(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

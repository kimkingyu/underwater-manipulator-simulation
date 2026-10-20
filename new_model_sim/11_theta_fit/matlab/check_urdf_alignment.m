function check_urdf_alignment()
%CHECK_URDF_ALIGNMENT  Verify that the old model/robot.urdf (rigid body) and the new-CAD theta model
%   (hydrodynamics) describe the same arm at the same pose.
%   base_link -> new CAD:  x_cad = R_bc * x_bl + t_bc  (fitted on the 3 joint centres, error 3e-17 m)
%   Checks at q = 0 and at random q: joint centres, joint axes and link COMs (URDF) vs the CFD geometry.
root = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
robot = importrobot(fullfile(root, 'model', 'robot.urdf')); robot.DataFormat = 'column';
[R_bc, t_bc] = urdf_to_cad();
G = uvms_geometry();
O = [0.0; 0.29386; -0.06630];
names = {'link_002', 'link_003', 'link_004'};
rng(3);
for trial = 0:3
    if trial == 0, q = zeros(3, 1); else, q = (rand(3, 1) - 0.5) * 2; end
    [~, Rl] = uvms_link_kin(q, zeros(3, 1), G);
    [P, ~, pj, aj] = fk_cad(q, G);
    fprintf('q = [%6.3f %6.3f %6.3f]\n', q);
    for i = 1:3
        T = getTransform(robot, q, names{i});
        c_urdf = R_bc * T(1:3, 4) + t_bc - O;                 % joint i frame origin = joint centre
        a_urdf = R_bc * T(1:3, 3);                            % joint axis = frame z
        com = R_bc * (T(1:3, 1:3) * robot.Bodies{i}.CenterOfMass' + T(1:3, 4)) + t_bc - O;
        com0 = R_bc * T(1:3, 1:3)' ; %#ok<NASGU>
        fprintf('  J%d centre err %.2e m, axis err %.2e | link COM (URDF) vs CAD centroid: %.1f mm\n', ...
            i, norm(c_urdf - pj(:, i)), norm(a_urdf - aj(:, i)), 1000 * norm(com - P(:, i)));
    end
end
end

function [P, R, pj, aj] = fk_cad(q, G)
% same forward kinematics as uvms_link_kin (positions relative to O_b, B axes)
n = G.n; P = zeros(3, n); pj = zeros(3, n); aj = zeros(3, n); R = cell(1, n);
Rc = eye(3); Tc = zeros(3, 1);
for i = 1:n
    a0 = G.joint_axis(i, :)'; c0 = G.joint_pos(i, :)';
    pj(:, i) = Rc * c0 + Tc; aj(:, i) = Rc * a0;
    K = [0 -aj(3,i) aj(2,i); aj(3,i) 0 -aj(1,i); -aj(2,i) aj(1,i) 0];
    Ri = eye(3) + sin(q(i)) * K + (1 - cos(q(i))) * K * K;
    Tc = Ri * (Tc - pj(:, i)) + pj(:, i); Rc = Ri * Rc;
    P(:, i) = Rc * G.link_pos(i, :)' + Tc; R{i} = Rc;
end
end

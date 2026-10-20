function test_buoyancy()
%TEST_BUOYANCY  Static check of the GM buoyancy model in simulate_uvms_theta (via its accel helper
%   reproduced here): at q = 0 the net gravity + buoyancy force is zero on the 6 base DOFs, and a
%   roll / pitch of 5 deg gives the restoring moment W*GM*sin(5 deg).
here = fileparts(mfilename('fullpath'));
u = build_chain(fullfile(here, '..', '..', '..', 'model', 'robot.urdf')); u.DataFormat = 'row';
n = numel(homeConfiguration(u));
mt = 0; s = [0 0 0]; q0 = zeros(1, n);
for i = 1:u.NumBodies
    b = u.Bodies{i}; if b.Mass == 0, continue; end
    T = getTransform(u, q0, b.Name);
    s = s + b.Mass * (T(1:3, 1:3) * b.CenterOfMass' + T(1:3, 4))'; mt = mt + b.Mass;
end
W = mt * 9.81; p_B = s / mt + [0 0 0.03];
net = @(q) gravityTorque(u, q) - buoy_force(u, q, W, p_B);
g0 = net(q0);
fprintf('q = 0: net base generalised force = %s (max %.2e)\n', mat2str(g0(1:6), 3), max(abs(g0(1:6))));
assert(max(abs(g0(1:6))) < 1e-9, 'not in equilibrium at q = 0');
for d = [4 5]
    q = q0; q(d) = deg2rad(5);
    g = net(q);
    fprintf('dof %d at 5 deg: holding moment %.3f N*m, expected W*GM*sin = %.3f\n', d, g(d), W * 0.03 * sind(5));
    assert(abs(g(d) - W * 0.03 * sind(5)) < 1e-3 * W * 0.03, 'restoring moment mismatch');
end
fprintf('test_buoyancy: all checks passed (W = %.1f N)\n', W);
end

function f = buoy_force(robot, q, W, p_B)
T = getTransform(robot, q, 'auv_base');
Jb = geometricJacobian(robot, q, 'auv_base');
r = T(1:3, 1:3) * p_B(:);
S = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
Jp = Jb(4:6, :) - S * Jb(1:3, :);
f = (Jp' * [0; 0; W])';
end

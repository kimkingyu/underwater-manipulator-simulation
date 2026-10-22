function test_build_chain_cad()
%TEST_BUILD_CHAIN_CAD  Static checks of the new-CAD rigid body and per-body buoyancy.
%   1. total buoyancy = total weight (neutral) to 1e-9 relative
%   2. at q = 0 the net gravity+buoyancy generalised force: zero in x, y, z; roll/pitch moment small
%      (only the arm's offset from the hull axis); joint torques reported
%   3. metacentric stiffness: roll/pitch restoring moment for a 0.01 rad tilt ~ m g GM * 0.01
%   4. COMs of the CAD bodies in CAD coordinates match mass_table.json
here = fileparts(mfilename('fullpath'));
[uvms, info] = build_chain_cad(fullfile(here, '..', '..', '..', 'model', 'robot.urdf'));
uvms.DataFormat = 'row';
mt = 0;
for i = 1:uvms.NumBodies, mt = mt + uvms.Bodies{i}.Mass; end
mb = sum([info.buoy.mass_displaced]);
fprintf('total mass %.3f kg, displaced %.3f kg, hull %.2f kg, arm incl. base0 %.3f kg\n', mt, mb, info.m_hull, info.m_arm_incl_base0);
assert(abs(mt / mb - 1) < 1e-9, 'not neutral');
gb = @(q) net_static(uvms, info, q);
f0 = gb(zeros(1, 9));
fprintf('q = 0: x y z %s N, roll pitch yaw %s N m, joints %s N m; hull trim shift (CAD x, z) %s mm\n', ...
    mat2str(f0(1:3), 3), mat2str(f0(4:6), 3), mat2str(f0(7:9), 3), mat2str(info.trim_shift_mm, 3));
assert(all(abs(f0(1:3)) < 1e-6 * mt * 9.81), 'net force at q = 0');
assert(all(abs(f0(4:6)) < 1e-6 * mt * 9.81), 'vehicle not trimmed at q = 0');
for ax = [4 5]
    q = zeros(1, 9); q(ax) = 0.01;
    f = gb(q);
    fprintf('tilt 0.01 rad about dof %d: restoring %.3f N m (m g GM * 0.01 = %.3f)\n', ax, f(ax) - f0(ax), mt * 9.81 * 0.03 * 0.01);
end
[R_bc, t_bc] = urdf_to_cad();
T = jsondecode(fileread(fullfile(here, '..', '..', '13_pose_q1', 'mass_table.json')));
map = {'link_002', 'link1'; 'link_003', 'link2'; 'link_004', 'link3'};
for k = 1:3
    b = uvms.getBody(map{k, 1});
    Tb = getTransform(uvms, zeros(1, 9), map{k, 1});
    c = R_bc * (Tb(1:3, 1:3) * b.CenterOfMass(:) + Tb(1:3, 4)) + t_bc;
    e = norm(c - T.groups.(map{k, 2}).centroid_m(:));
    fprintf('%s COM error %.2e m\n', map{k, 1}, e);
    assert(e < 1e-9);
end
fprintf('test_build_chain_cad: all passed\n');
end

function f = net_static(robot, info, q)
% gravity + per-body buoyancy, generalised force on the left-hand side (as in simulate_uvms_theta)
f = gravityTorque(robot, q);
for i = 1:numel(info.buoy)
    bb = info.buoy(i);
    T = getTransform(robot, q, bb.body);
    Jb = geometricJacobian(robot, q, bb.body);
    r = T(1:3, 1:3) * bb.cob_body(:);
    Jp = Jb(4:6, :) - [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0] * Jb(1:3, :);
    f = f - (Jp' * [0; 0; bb.mass_displaced * 9.81])';
end
end

function diag_gravity()
%DIAG_GRAVITY  Where does the 24.5 N*m roll gravity torque of the 9-DOF chain come from?
%   Prints, at q = 0, each body's mass, world COM and its contribution to the roll / pitch gravity
%   torque about the base origin, plus the chain's gravityTorque.
here = fileparts(mfilename('fullpath'));
u = build_chain(fullfile(here, '..', '..', '..', 'model', 'robot.urdf'));
u.DataFormat = 'row';
q = zeros(1, 9);
g = [0 0 -9.81];
tot = zeros(1, 3); mt = 0;
fprintf('%-10s %9s  %28s  %10s %10s\n', 'body', 'mass', 'COM world (m)', 'Mroll', 'Mpitch');
for i = 1:u.NumBodies
    b = u.Bodies{i};
    if b.Mass == 0, continue; end
    T = getTransform(u, q, b.Name);
    c = (T(1:3, 1:3) * b.CenterOfMass' + T(1:3, 4))';
    m = cross(c, b.Mass * g);
    tot = tot + m; mt = mt + b.Mass;
    fprintf('%-10s %9.3f  [%8.4f %8.4f %8.4f]  %10.3f %10.3f\n', b.Name, b.Mass, c, m(1), m(2));
end
gt = gravityTorque(u, q);
fprintf('sum of mass x g moments about origin: roll %.3f pitch %.3f   (total mass %.2f kg)\n', tot(1), tot(2), mt);
fprintf('gravityTorque(roll, pitch) = %.3f %.3f  (sign: generalised force needed to hold)\n', gt(4), gt(5));
end

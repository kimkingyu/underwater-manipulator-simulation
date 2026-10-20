function test_uvms_hydro_theta()
%TEST_UVMS_HYDRO_THETA  Checks of the 9-DOF chain hydrodynamics wrapper.
%  1) S(q)*qd(1:6) equals the base_link body twist from the rigidBodyTree geometric Jacobian.
%  2) Power consistency: qd' * tau_h equals (CFD-frame generalised velocity)' * (Y*theta) for random states.
%  3) At rest attitude and q = 0, pure surge/sway/heave of the chain reproduces Y*theta of the CFD
%     model for the same body velocity (frame change only), i.e. the T_*_08 loads.
here = fileparts(mfilename('fullpath'));
uvms = build_chain(fullfile(here, '..', '..', '..', 'model', 'robot.urdf'));
S0 = load(fullfile(here, 'theta_fit.mat')); th = S0.theta;
G = uvms_geometry();
[R_bc, ~] = urdf_to_cad();
rng(7);
for k = 1:10
    q = [randn(3, 1); 0.4 * randn(3, 1); 0.5 * randn(3, 1)]; qd = randn(9, 1);
    J = geometricJacobian(uvms, q, 'auv_base');        % [w; v] in world, of the base_link origin
    T = getTransform(uvms, q, 'auv_base'); Rw = T(1:3, 1:3);
    tw = J * qd; w_b = Rw' * tw(1:3); v_b = Rw' * tw(4:6);
    [S, ~] = local_S(q(4:6), qd(4:6));
    e1 = norm(S * qd(1:6) - [v_b; w_b]);
    assert(e1 < 1e-9, 'twist map mismatch %g', e1);
    % power check
    qdd = randn(9, 1);
    th2 = th; th2(1:6) = 0; th2(19:36) = th2(19:36) .* repmat([0 0 0 1 1 1]', 3, 1);  % no added mass: pure f(state)
    tau = uvms_hydro_theta(q, qd, zeros(9, 1), th2, G)';
    [zeta] = cfd_zeta(q, qd);
    Y = Y_uvms(q(7:9), zeta, zeros(9, 1), G);
    e2 = abs(qd' * tau - zeta' * (Y * th2));
    assert(e2 < 1e-8 * max(1, abs(qd' * tau)), 'power mismatch %g', e2);
end
% 3) surge/sway/heave at q = 0 (base_link y is CAD -z, base_link z is CAD +y)
for d = 1:3
    qd = zeros(9, 1); qd(d) = 0.8;
    tau = uvms_hydro_theta(zeros(9, 1), qd, zeros(9, 1), th, G)';
    zeta = cfd_zeta(zeros(9, 1), qd);
    tB = Y_uvms(zeros(3, 1), zeta, zeros(9, 1), G) * th;
    fprintf('chain dof %d at 0.8 m/s: CFD-frame body v = [%5.2f %5.2f %5.2f]  F_cad = [%7.1f %7.1f %7.1f]  tau_J = [%6.3f %6.3f %6.3f] | chain tau(1:3) = [%7.1f %7.1f %7.1f], tau(7:9) = [%6.3f %6.3f %6.3f]\n', ...
        d, zeta(1:3), tB(1:3), tB(7:9), tau(1:3), tau(7:9));
    assert(norm(tau(1:3) - R_bc' * tB(1:3)) < 1e-9 && norm(tau(7:9) - tB(7:9)) < 1e-12, 'frame change error');
end
fprintf('test_uvms_hydro_theta: all checks passed\n');
end

function zeta = cfd_zeta(q, qd)
[R_bc, t_bc] = urdf_to_cad(); O = [0.0; 0.29386; -0.06630]; r_b = R_bc' * (O - t_bc);
[S, ~] = local_S(q(4:6), qd(4:6)); nu = S * qd(1:6);
zeta = [R_bc * (nu(1:3) + cross(nu(4:6), r_b)); R_bc * nu(4:6); qd(7:9)];
end

function [S, Sd] = local_S(eul, euld) %#ok<INUSD>
r = eul(1); p = eul(2); y = eul(3);
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R = Rx * Ry * Rz;
T = [Rz' * Ry' * [1; 0; 0], Rz' * [0; 1; 0], [0; 0; 1]];
S = blkdiag(R', T); Sd = [];
end

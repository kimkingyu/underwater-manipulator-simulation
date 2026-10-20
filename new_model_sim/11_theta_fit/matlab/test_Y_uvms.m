function test_Y_uvms()
%TEST_Y_UVMS  Self-checks of the regressor against an independent direct evaluation.
%   1) Y*theta equals a direct implementation of M_A a + C_A zeta + D zeta + tau_f (random q, zeta, a).
%   2) C_A part is energy-neutral: zeta' * (C_A(zeta) zeta) = 0 for the added-Coriolis terms.
%   3) Jd from uvms_link_kin matches a finite difference of J along a trajectory.
G = uvms_geometry();
rng(1);
for trial = 1:20
    q = randn(3, 1); zeta = randn(9, 1); a = randn(9, 1);
    th = rand(87, 1); th(43:87) = randn(45, 1);
    Y = Y_uvms(q, zeta, a, G);
    ref = direct(q, zeta, a, th, G);
    assert(norm(Y * th - ref) < 1e-6 * max(1, norm(ref)), 'Y*theta mismatch (trial %d): %g', trial, norm(Y * th - ref));
    % Coriolis-only: a = 0, no damping, no friction
    th2 = th; th2(7:18) = 0; th2([22:24 28:30 34:36]) = 0; th2(37:87) = 0;
    Yc = Y_uvms(q, zeta, zeros(9, 1), G);
    [~, Jd] = uvms_link_kin(q, zeta(7:9), G);
    acc = zeros(9, 1);              % remove the Jd*zeta (inertial) part, keep only b(.)
    for i = 1:3
        c0 = 18 + 6 * (i - 1);
        [J] = uvms_link_kin(q, zeta(7:9), G);
        Ai = Jd{i} * zeta;
        acc = acc + J{i}' * [th2(c0 + (1:3)) .* Ai(1:3); zeros(3, 1)];
    end
    pc = zeta' * (Yc * th2 - acc);
    assert(abs(pc) < 1e-8 * max(1, norm(zeta) ^ 3), 'added Coriolis does work: %g', pc);
end
% Jd check
q = [0.3; -0.4; 0.5]; qd = [0.7; -0.2; 0.9]; dt = 1e-6;
[J0, Jd] = uvms_link_kin(q, qd, G);
J1 = uvms_link_kin(q + qd * dt, qd, G);
for i = 1:3
    err = norm((J1{i} - J0{i}) / dt - Jd{i});
    assert(err < 1e-4, 'Jd mismatch link %d: %g', i, err);
end
fprintf('test_Y_uvms: all checks passed\n');
end

function tau = direct(q, zeta, a, th, G)
MA = diag(th(1:6)); Dl = diag(th(7:12)); Dq = diag(th(13:18));
nu = zeta(1:6);
tau = zeros(9, 1);
tau(1:6) = MA * a(1:6) + b(MA, nu) + Dl * nu + Dq * (abs(nu) .* nu);
[J, Jd] = uvms_link_kin(q, zeta(7:9), G);
for i = 1:3
    c0 = 18 + 6 * (i - 1);
    Ai = blkdiag(diag(th(c0 + (1:3))), zeros(3));
    Di = blkdiag(diag(th(c0 + (4:6))), zeros(3));
    Ei = reshape(th(60 + 9 * (i - 1) + (1:9)), 3, 3);          % moment axis j (row) per drag axis k (column)
    Vi = J{i} * zeta; vi = Vi(1:3);
    wr = Ai * (J{i} * a + Jd{i} * zeta) + b(Ai, Vi) + Di * (abs(Vi) .* Vi) + [zeros(3, 1); Ei * (abs(vi) .* vi)];
    tau = tau + J{i}' * wr;
end
tau(7:9) = tau(7:9) + th(37:39) .* zeta(7:9) + th(40:42) .* sign(zeta(7:9));
v = zeta(1:3);
Cl = reshape(th(43:51), 3, 3); Cq = reshape(th(52:60), 3, 3);   % rows K M N, columns u v w
tau(4:6) = tau(4:6) + Cl * v + Cq * (abs(v) .* v);
end

function r = b(K, V)
pl = K * V; p = pl(1:3); l = pl(4:6); v = V(1:3); w = V(4:6);
r = [cross(w, p); cross(v, p) + cross(w, l)];
end

function [Y, Ylink] = Y_uvms(q, zeta, a, G)
%Y_UVMS  Regressor of the estimated term  Y*theta = M_A*a + C_A(q,zeta)*zeta + D_0 + tau_f0.
%   q    : 3x1 joint angles [rad]           (q = 0 is the CAD pose used in the CFD cases)
%   zeta : 9x1 [nu; qdot], nu = [u v w p q r] in the body frame B (CAD axes, origin = hull centroid)
%   a    : 9x1 reference acceleration (the control document's a = Jcal^-1 (xi_dd_d - Ks s - Lambda e_dot - Jcal_dot zeta))
%   Y    : 9 x 42 so that  tau_hat = Y * theta  (generalised force the actuators must supply)
%   Ylink: 9 x 42, translational force of links 1..3 in their own frames (used by fit_theta only)
%
%   theta (42 x 1), see theta_names():
%     1- 6  AUV added mass, diagonal of M_A,V   (X_udot..N_rdot, written positive)
%     7-12  AUV linear damping, diag(D_V,l)
%    13-18  AUV quadratic damping, diag(D_V,q)  (D_V(nu) nu = D_l nu + D_q |nu|.*nu)
%    19-36  link i (i = 1..3), 6 each: 3 translational added masses diag(A_i), 3 quadratic dampings
%    37-42  joint friction F_v1..3 (viscous), F_c1..3 (Coulomb)  -- fixed to 0 in the CFD fit
%   Added-Coriolis terms use b(K,V) = [w x p; v x p + w x l], [p; l] = K V, from the dynamics document.
if nargin < 4, G = uvms_geometry(); end
q = q(:); zeta = zeta(:); a = a(:);
n = G.n; N = 6 + n;
np = 18 + 6 * n + 2 * n;
Y = zeros(N, np);
Ylink = zeros(3 * n, np);

% ---- AUV body: M_A,V a + b(M_A,V, nu) + D_V(nu) nu, padded with zeros for the joint rows ----
v = zeta(1:3); w = zeta(4:6);
for k = 1:3
    e = zeros(3, 1); e(k) = 1;
    p = e * v(k);
    col = zeros(6, 1);
    col(k) = a(k);                                  % M_A a
    col = col + [cross(w, p); cross(v, p)];         % b: w x p ; v x p
    Y(1:6, k) = col;
    e2 = zeros(3, 1); e2(k) = 1;
    l = e2 * w(k);
    col = zeros(6, 1);
    col(3 + k) = a(3 + k);
    col(4:6) = col(4:6) + cross(w, l);              % b: w x l
    Y(1:6, 3 + k) = col;
end
nu = zeta(1:6);
Y(1:6, 7:12) = diag(nu);
Y(1:6, 13:18) = diag(abs(nu) .* nu);

% ---- links: sum_i J_i' [ A_i (J_i a + Jd_i zeta) + b(A_i, V_i) + D_i(V_i) V_i ] ----
[J, Jd] = uvms_link_kin(q, zeta(7:end), G);
for i = 1:n
    Vi = J{i} * zeta;  vi = Vi(1:3); wi = Vi(4:6);
    Ai = J{i} * a + Jd{i} * zeta;
    c0 = 18 + 6 * (i - 1);
    for k = 1:3
        e = zeros(3, 1); e(k) = 1;
        p = e * vi(k);
        wr = [e * Ai(k) + cross(wi, p); cross(vi, p)];      % added mass + added Coriolis (6x1 wrench)
        Y(:, c0 + k) = J{i}' * wr;
        Ylink(3 * (i - 1) + (1:3), c0 + k) = wr(1:3);
        wr = [e * abs(vi(k)) * vi(k); zeros(3, 1)];         % quadratic damping
        Y(:, c0 + 3 + k) = J{i}' * wr;
        Ylink(3 * (i - 1) + (1:3), c0 + 3 + k) = wr(1:3);
    end
end

% ---- joint friction tau_f0 = F_v qdot + F_c sgn(qdot) ----
qd = zeta(7:end);
c0 = 18 + 6 * n;
for j = 1:n
    Y(6 + j, c0 + j) = qd(j);
    Y(6 + j, c0 + n + j) = sign(qd(j));
end
end

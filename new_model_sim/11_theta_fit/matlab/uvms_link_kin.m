function [J, Jd, R] = uvms_link_kin(q, qd, G)
%UVMS_LINK_KIN  Link Jacobians J_i (6 x (6+n)) and their time derivatives for the AUV + serial arm.
%   V_i = J_i * zeta,  zeta = [nu; qdot],  nu = [v; w] in the body frame B.
%   V_i = [v_i; w_i] is the velocity of link reference point i, expressed in link frame i.
%   Jd_i = sum_j dJ_i/dq_j * qd_j  (central differences, as in the dynamics document).
%   R{i}: rotation of link frame i relative to B.
if nargin < 3, G = uvms_geometry(); end
q = q(:); qd = qd(:);
J = jac(q, G);
h = 1e-6;
Jd = cell(1, G.n);
for i = 1:G.n, Jd{i} = zeros(6, 6 + G.n); end
for j = 1:G.n
    if qd(j) == 0, continue; end
    e = zeros(G.n, 1); e(j) = h;
    Jp = jac(q + e, G); Jm = jac(q - e, G);
    for i = 1:G.n
        Jd{i} = Jd{i} + (Jp{i} - Jm{i}) / (2 * h) * qd(j);
    end
end
if nargout > 2
    [~, R] = fk(q, G);
end
end

function J = jac(q, G)
[P, R, pj, aj] = fk(q, G);
n = G.n;
J = cell(1, n);
for i = 1:n
    Jb = zeros(6, 6 + n);                       % velocity of point P_i in B axes
    Jb(1:3, 1:3) = eye(3);
    Jb(1:3, 4:6) = -skew(P(:, i));
    Jb(4:6, 4:6) = eye(3);
    for j = 1:i
        Jb(1:3, 6 + j) = cross(aj(:, j), P(:, i) - pj(:, j));
        Jb(4:6, 6 + j) = aj(:, j);
    end
    Ri = R{i};
    J{i} = blkdiag(Ri', Ri') * Jb;              % expressed in link frame i
end
end

function [P, R, pj, aj] = fk(q, G)
n = G.n;
P = zeros(3, n); pj = zeros(3, n); aj = zeros(3, n);
R = cell(1, n);
Rc = eye(3); Tc = zeros(3, 1);                  % accumulated rigid motion x -> Rc*x + Tc
for i = 1:n
    a0 = G.joint_axis(i, :)'; c0 = G.joint_pos(i, :)';
    pj(:, i) = Rc * c0 + Tc;
    aj(:, i) = Rc * a0;
    Ri = axang(aj(:, i), q(i));
    Tc = Ri * (Tc - pj(:, i)) + pj(:, i);
    Rc = Ri * Rc;
    P(:, i) = Rc * G.link_pos(i, :)' + Tc;
    R{i} = Rc;
end
end

function Rm = axang(a, t)
K = skew(a / norm(a));
Rm = eye(3) + sin(t) * K + (1 - cos(t)) * (K * K);
end

function S = skew(x)
S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end

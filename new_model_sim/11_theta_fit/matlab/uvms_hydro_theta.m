function tau_h = uvms_hydro_theta(q9, qd9, qdd9, theta, G)
%UVMS_HYDRO_THETA  CFD-fitted hydrodynamic generalised force for the 9-DOF chain of
%   experiments/simulate_uvms_dynamics.m (old model/robot.urdf rigid body + new-CAD theta hydrodynamics).
%
%   Chain states (as in that script): q9 = [x y z roll pitch yaw q1 q2 q3], base_link frame, world
%   orientation R_wb = Rz(yaw)*Ry(pitch)*Rx(roll) (prismatic x,y,z then revolute about x, y', z'').
%   Returns tau_h (1x9) = the hydrodynamic force the actuators must OVERCOME, i.e. it goes on the
%   left-hand side next to M*qdd + C + G (the script's F_damping slot):
%       M_rb*qdd + C + G + tau_h = tau_cmd
%   tau_h = J_chain' * blkdiag(R_cb', R_cb', 1) * Y_uvms(q, zeta, a) * theta, where
%   zeta / a are the body-frame velocity / acceleration of the CFD model (frame B = new CAD axes,
%   origin O_b = hull centroid). Joint angles and torques need no conversion (axes coincide, see
%   urdf_to_cad / check_urdf_alignment). The acceleration a includes the Coriolis part Jdot*qd of the
%   chain velocity map, so the added-mass term is exact for the chain, not only for the body.
persistent R_bc t_bc O
if isempty(R_bc)
    [R_bc, t_bc] = urdf_to_cad();
    O = [0.0; 0.29386; -0.06630];
end
if nargin < 5, G = uvms_geometry(); end
G.hull_K = numel(theta) == 105;               % v5_hullK theta carries the K block 88:105
q9 = q9(:); qd9 = qd9(:); qdd9 = qdd9(:);
r_b = R_bc' * (O - t_bc);                     % O_b expressed in base_link [m]

% chain velocity map: nu_bl (base_link-frame twist at the base_link origin) = S(q) * qd(1:6)
[S, Sd] = base_twist_map(q9(4:6), qd9(4:6));
nu_bl  = S * qd9(1:6);
nud_bl = S * qdd9(1:6) + Sd * qd9(1:6);
% move the reference point to O_b and rotate into the CAD axes
v = nu_bl(1:3); w = nu_bl(4:6); vd = nud_bl(1:3); wd = nud_bl(4:6);
v_ob  = v + cross(w, r_b);
vd_ob = vd + cross(wd, r_b);                 % body-frame derivative of the O_b velocity
zeta = [R_bc * v_ob; R_bc * w; qd9(7:9)];
a    = [R_bc * vd_ob; R_bc * wd; qdd9(7:9)];

Y = Y_uvms(q9(7:9), zeta, a, G);             % generalised force in (CAD body frame at O_b, joints)
tB = Y * theta(:);
F = R_bc' * tB(1:3); M = R_bc' * tB(4:6);    % back to base_link axes, about O_b
M = M + cross(r_b, F);                        % about the base_link origin
tau_h = ([S' * [F; M]; tB(7:9)])';
end

function [S, Sd] = base_twist_map(eul, euld)
% x,y,z prismatic along the world axes, then roll (x), pitch (y'), yaw (z''): R_wb = Rx*Ry*Rz order of
% the rigidBodyTree chain virtual_roll -> virtual_pitch -> virtual_yaw.
% Body twist in base_link: v_b = R_wb' * pdot,  w_b = T(eul) * euld.
[R, T] = rt(eul);
S = blkdiag(R', T);
h = 1e-7;
Sd = zeros(6);
for k = 1:3
    e = zeros(3, 1); e(k) = h;
    [Rp, Tp] = rt(eul + e); [Rm, Tm] = rt(eul - e);
    Sd = Sd + blkdiag((Rp - Rm)' / (2 * h), (Tp - Tm) / (2 * h)) * euld(k);
end
end

function [R, T] = rt(eul)
r = eul(1); p = eul(2); y = eul(3);
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R = Rx * Ry * Rz;                             % chain order: roll joint, then pitch, then yaw
% body angular velocity: w_b = Rz'*Ry'*[rd;0;0] + Rz'*[0;pd;0] + [0;0;yd]
T = [Rz' * Ry' * [1; 0; 0], Rz' * [0; 1; 0], [0; 0; 1]];
end

function out = simulate_uvms_theta(hydro_model, buoyancy, qArm_target, output_suffix, tEnd)
%SIMULATE_UVMS_THETA  9-DOF UVMS simulation of experiments/simulate_uvms_dynamics.m with the
%   hydrodynamics replaced by the CFD-fitted theta model (Y_uvms * theta).
%   hydro_model: 'theta' (default) | 'handset' (the original hand-set M_added / D_* of that script,
%   for comparison). Rigid body: old model/robot.urdf (kim 2026-10-20). Everything else -- chain,
%   PD gains, arm trajectory, metacentric restoring, integration -- is copied unchanged.
%
%   Hydrodynamics with theta enter as  (M_rb + M_A(q)) qdd = tau - C - G - h(q, qd), where the
%   added-mass matrix M_A(q) and the remainder h are evaluated from uvms_hydro_theta (it is linear in
%   qdd, so M_A columns come from unit accelerations). Joint friction theta(37:42) is 0.
%   Optional qArm_target (rad): default [0.45 -0.50 0.40]; qb is [0.45 -0.50 -0.40].
%   output_suffix: filename suffix, e.g. '_qb'; automatic for a nondefault target.
%   tEnd: duration in seconds (default 8). Nonempty suffixes never overwrite a result.
%   Example: simulate_uvms_theta('theta', 'gm', [0.45 -0.50 -0.40]);
%   Regression: simulate_uvms_theta('theta', 'gm', [], '_regression_default', 0.5);
if nargin < 1, hydro_model = 'theta'; end
if nargin < 2 || isempty(buoyancy), buoyancy = 'gm'; end
if nargin < 3 || isempty(qArm_target), qArm_target = [0.45, -0.50, 0.40]; end
validateattributes(qArm_target, {'numeric'}, {'real', 'finite', 'vector', 'numel', 3});
qArm_target = reshape(qArm_target, 1, 3);
if nargin < 4 || isempty(output_suffix)
    if isequal(qArm_target, [0.45, -0.50, 0.40])
        output_suffix = '';
    elseif isequal(qArm_target, [0.45, -0.50, -0.40])
        output_suffix = '_qb';
    else
        output_suffix = ['_q_' strrep(strrep(sprintf('%.17g_', qArm_target), '-', 'm'), '.', 'p')];
    end
end
output_suffix = char(output_suffix);
assert(isempty(output_suffix) || ~isempty(regexp(output_suffix, '^_[A-Za-z0-9_-]+$', 'once')), ...
    'simulate_uvms_theta:InvalidSuffix', 'Use a suffix such as _qb (letters, digits, underscores, hyphens only).');
if nargin < 5 || isempty(tEnd), tEnd = 8.0; end
validateattributes(tEnd, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
here = fileparts(mfilename('fullpath'));
if strcmp(buoyancy, 'gm'), sfx = ''; else, sfx = ['_' buoyancy]; end
output_path = fullfile(here, sprintf('sim_uvms_%s%s%s.mat', hydro_model, sfx, output_suffix));
assert(isempty(output_suffix) || ~isfile(output_path), 'simulate_uvms_theta:OutputExists', ...
    'Refusing to overwrite %s; choose a different output_suffix.', output_path);
root = fullfile(here, '..', '..', '..');
uvms = build_chain(fullfile(root, 'model', 'robot.urdf'));
uvms.DataFormat = 'row';
S = load(fullfile(here, 'theta_fit.mat')); theta = S.theta;
G = uvms_geometry();

m_auv = uvms.getBody('auv_base').Mass;
hs.M_added = zeros(9);
hs.M_added(1:3, 1:3) = diag([0.4, 0.4, 0.5] * m_auv);
hs.M_added(4:6, 4:6) = diag([0.3, 0.3, 0.3] * 20.0);
hs.M_added(7, 7) = 0.05; hs.M_added(8, 8) = 0.15; hs.M_added(9, 9) = 0.12;
hs.D_lin = [60, 60, 80, 25, 25, 20]; hs.D_quad = [120, 120, 150, 40, 40, 35]; hs.D_arm = [1.5, 2.5, 2.0];
K_restore = m_auv * 9.81 * 0.03;
if strcmp(buoyancy, 'gm')
    % neutral buoyancy with the centre of buoyancy GM = 0.03 m above the total COM (the original
    % script's stated assumption). p_B in base_link, from the q = 0 total COM.
    [cg, mt] = total_com(uvms);
    buoy.W = mt * 9.81;
    buoy.p_B = cg + [0 0 0.03];
else
    buoy = [];                      % 'script': original handling (unbalanced roll gravity, -17 deg list)
end

Kp_auv_pos = [350, 350, 450]; Kd_auv_pos = [180, 180, 220];
Kp_auv_att = [120, 120, 100]; Kd_auv_att = [60, 60, 50];
Kp_arm = [120, 100, 80]; Kd_arm = [25, 20, 15];

% dt = 0.001 s (original script: 0.005). With the CFD added inertia the arm joints are 2-4x lighter
% than with the hand-set values, the fastest PD mode rises from 333 to 610 1/s and explicit RK4 at
% 0.005 s amplifies it by 1.48 (diverges; see diag_stiffness.m). Both models use the same dt here.
dt = 0.001; time = 0:dt:tEnd; nSteps = numel(time);
qArm_0 = [0, 0, 0];
qArm_des = zeros(nSteps, 3); qdArm_des = zeros(nSteps, 3);
for k = 1:nSteps
    t = time(k);
    if t < 1.0
        qArm_des(k, :) = qArm_0;
    elseif t <= 4.0
        tau = (t - 1.0) / 3.0;
        s = 10*tau^3 - 15*tau^4 + 6*tau^5; sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / 3.0;
        qArm_des(k, :) = qArm_0 + s * (qArm_target - qArm_0);
        qdArm_des(k, :) = sd * (qArm_target - qArm_0);
    else
        qArm_des(k, :) = qArm_target;
    end
end

q = zeros(1, 9); qd = zeros(1, 9); q(7:9) = qArm_0;
L.q = zeros(nSteps, 9); L.qd = zeros(nSteps, 9); L.tau_auv = zeros(nSteps, 6); L.tau_arm = zeros(nSteps, 3);
L.tau_hydro = zeros(nSteps, 9); L.tcp = zeros(nSteps, 3);
tcpOffset = [0.280, -0.030, -0.0168];
acc = @(q_, qd_, tc_) accel(uvms, q_, qd_, tc_, hydro_model, theta, G, hs, K_restore, buoy);
tic;
for k = 1:nSteps
    tau_auv = [-Kp_auv_pos .* q(1:3) - Kd_auv_pos .* qd(1:3), -Kp_auv_att .* q(4:6) - Kd_auv_att .* qd(4:6)];
    tau_arm = Kp_arm .* (qArm_des(k, :) - q(7:9)) + Kd_arm .* (qdArm_des(k, :) - qd(7:9));
    tau_arm = max(min(tau_arm, 15), -15);
    tau_cmd = [tau_auv, tau_arm];
    L.q(k, :) = q; L.qd(k, :) = qd; L.tau_auv(k, :) = tau_auv; L.tau_arm(k, :) = tau_arm;
    T_tcp = getTransform(uvms, q, 'link_004');
    L.tcp(k, :) = (T_tcp(1:3, 1:3) * tcpOffset.' + T_tcp(1:3, 4)).';
    [k1, h1] = acc(q, qd, tau_cmd);
    L.tau_hydro(k, :) = h1;
    k2 = acc(q + 0.5*dt*qd, qd + 0.5*dt*k1, tau_cmd);
    k3 = acc(q + 0.5*dt*(qd + 0.5*dt*k1), qd + 0.5*dt*k2, tau_cmd);
    k4 = acc(q + dt*(qd + 0.5*dt*k2), qd + dt*k3, tau_cmd);
    q = q + (dt/6) * (qd + 2*(qd + 0.5*dt*k1) + 2*(qd + 0.5*dt*k2) + (qd + dt*k3));
    qd = qd + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end
fprintf('[%s, buoyancy %s] simulated %.1f s in %.1f s\n', hydro_model, buoyancy, tEnd, toc);
out = L; out.time = time; out.qArm_des = qArm_des; out.hydro_model = hydro_model; out.buoyancy = buoyancy;
save(output_path, '-struct', 'out');
end

function [qdd, h_now] = accel(robot, q, qd, tau, model, theta, G, hs, K_restore, buoy)
M_rigid = massMatrix(robot, q);
C_term = velocityProduct(robot, q, qd);
G_term = gravityTorque(robot, q);
if isempty(buoy)
    % original script: translational gravity removed, ad-hoc restoring on roll/pitch only
    G_term(1:3) = 0;
    G_term(4) = G_term(4) + K_restore * q(4);
    G_term(5) = G_term(5) + K_restore * q(5);
else
    % neutral buoyancy: force W (= total weight) straight up, applied at the body-fixed centre of
    % buoyancy p_B (base_link frame) = q-0 total COM + GM along base_link +z. Generalised force on the
    % left-hand side: G_term - J_pB' * [0 0 W].
    T = getTransform(robot, q, 'auv_base');
    Jb = geometricJacobian(robot, q, 'auv_base');          % [w; v] world, of the base_link origin
    r = T(1:3, 1:3) * buoy.p_B(:);                          % origin -> p_B, world
    Jp = Jb(4:6, :) - skew(r) * Jb(1:3, :);                 % linear velocity Jacobian of p_B
    G_term = G_term - (Jp' * [0; 0; buoy.W])';
end
if strcmp(model, 'handset')
    MA = hs.M_added;
    h = zeros(1, 9);
    v = qd(1:6);
    h(1:6) = hs.D_lin .* v + hs.D_quad .* abs(v) .* v;
    h(7:9) = hs.D_arm .* qd(7:9);
else
    h = uvms_hydro_theta(q, qd, zeros(9, 1), theta, G);           % everything except M_A*qdd
    MA = zeros(9);
    for j = 1:9
        e = zeros(9, 1); e(j) = 1;
        MA(:, j) = (uvms_hydro_theta(q, qd, e, theta, G) - h)';   % exact: linear in qdd
    end
    MA = (MA + MA') / 2;
end
qdd = ((M_rigid + MA) \ (tau - C_term - G_term - h).').';
h_now = h + qdd * MA';                                            % total hydrodynamic force at this state
end

function [cg, mt] = total_com(robot)
% total COM of all bodies at q = 0, expressed in base_link (= world at q = 0)
q = zeros(1, numel(homeConfiguration(robot)));
mt = 0; s = [0 0 0];
for i = 1:robot.NumBodies
    b = robot.Bodies{i};
    if b.Mass == 0, continue; end
    T = getTransform(robot, q, b.Name);
    s = s + b.Mass * (T(1:3, 1:3) * b.CenterOfMass' + T(1:3, 4))';
    mt = mt + b.Mass;
end
cg = s / mt;
end

function S = skew(x)
S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end

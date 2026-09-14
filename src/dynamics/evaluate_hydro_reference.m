function [tau, parts] = evaluate_hydro_reference(robot, hydro, env, q, qd, qdd, vc)
%EVALUATE_HYDRO_REFERENCE Fluid-on-arm joint moments of the analytical model.
% Inputs: q/qd/qdd N-by-3 [rad, rad/s, rad/s^2], vc N-by-3 or 1-by-3 [m/s].
% Output: tau N-by-3 [N*m], positive along each URDF joint axis.
% tau = buoyancy + drag - M_add*qdd - C_add*qd.
% NO rigid-body inertia, gravity, rigid-body Coriolis or motor torque.
% This reproduces the existing instantaneous analytical model; it does not
% validate its coefficients, contact geometry, wake history or current-dependent
% added-mass physics. The surrogate inherits these limitations.
X = hydro_surrogate_inputs(q, qd, qdd, vc);
if ~strcmp(robot.DataFormat, 'row')
    error('hydro:DataFormat', 'robot.DataFormat must be row.');
end
n = size(X,1);
fields = {'buoyancy','drag','added_inertia','added_coriolis'};
for i = 1:numel(fields)
    parts.(fields{i}) = zeros(n,3);
end
for k = 1:n
    e = env;
    e.vc_world = X(k,10:12).';
    [M, b, d] = eval_link_hydro_components(robot, X(k,1:3), X(k,4:6), hydro, e);
    C = eval_added_mass_coriolis(robot, X(k,1:3), X(k,4:6), hydro);
    parts.buoyancy(k,:) = b;
    parts.drag(k,:) = d;
    parts.added_inertia(k,:) = -(M * X(k,7:9).').';
    parts.added_coriolis(k,:) = -(C * X(k,4:6).').';
end
tau = parts.buoyancy + parts.drag + parts.added_inertia + parts.added_coriolis;
end

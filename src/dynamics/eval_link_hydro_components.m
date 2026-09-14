function [M_add, tau_buoyancy, tau_drag] = eval_link_hydro_components(robot, q, qd, hydro, env)
%EVAL_LINK_HYDRO_COMPONENTS Existing COM/CB analytical hydrodynamics.
% q [rad], qd [rad/s], row vectors. Buoyancy/drag are fluid-on-arm moments.
% Added mass is a joint-space inertia, not a signed fluid torque.
M_add = zeros(3,3);
tau_buoyancy = zeros(1,3);
tau_drag = zeros(1,3);
for i = 1:numel(hydro)
    T_body = getTransform(robot, q, hydro(i).name);
    R_body = T_body(1:3,1:3);
    J_geom = geometricJacobian(robot, q, hydro(i).name);
    J_v = J_geom(4:6,:);
    J_w = J_geom(1:3,:);
    F_buoy_vec = [0; 0; env.rho * env.g * hydro(i).volume];
    J_cb = J_v - skew_mat(R_body * hydro(i).cb_local.') * J_w;
    tau_buoyancy = tau_buoyancy + (J_cb.' * F_buoy_vec).';
    J_com = J_v - skew_mat(R_body * hydro(i).com_local.') * J_w;
    v_rel = J_com * qd.' - env.vc_world;
    F_drag_vec = -0.5 * env.rho * hydro(i).Cd * hydro(i).A_proj * norm(v_rel) * v_rel;
    tau_drag = tau_drag + (J_com.' * F_drag_vec).';
    M_add_world = R_body * hydro(i).added_mass * R_body.';
    M_add = M_add + J_com.' * M_add_world * J_com;
end
end

function S = skew_mat(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

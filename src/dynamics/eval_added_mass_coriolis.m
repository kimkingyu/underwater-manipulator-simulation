function C_add = eval_added_mass_coriolis(robot, q, qd, hydro)
%EVAL_ADDED_MASS_CORIOLIS Christoffel matrix of the existing added-mass model.
% Central difference step and tensor convention match calc_joint_loads.m.
h = 1e-6;
dMdq = cell(3,1);
for k = 1:3
    dq = zeros(1,3); dq(k) = h;
    dMdq{k} = (added_mass_matrix(robot, q+dq, hydro) - ...
        added_mass_matrix(robot, q-dq, hydro)) / (2*h);
end
C_add = zeros(3,3);
for i = 1:3
    for j = 1:3
        for k = 1:3
            C_add(i,j) = C_add(i,j) + 0.5 * ...
                (dMdq{k}(i,j) + dMdq{j}(i,k) - dMdq{i}(j,k)) * qd(k);
        end
    end
end
end

function M_add = added_mass_matrix(robot, q, hydro)
M_add = zeros(3,3);
for i = 1:numel(hydro)
    T = getTransform(robot, q, hydro(i).name);
    R = T(1:3,1:3);
    J = geometricJacobian(robot, q, hydro(i).name);
    r = R * hydro(i).com_local.';
    S = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
    J_com = J(4:6,:) - S * J(1:3,:);
    M_add = M_add + J_com.' * (R * hydro(i).added_mass * R.') * J_com;
end
end

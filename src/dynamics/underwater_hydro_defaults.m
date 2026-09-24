function [hydro, env] = underwater_hydro_defaults(robot)
%UNDERWATER_HYDRO_DEFAULTS Shared parameters of the existing analytical model.
% SI units. These are assumed model parameters, not CFD-calibrated values.
% Changing density/geometry/Cd/added mass requires retraining the surrogate.
env = struct('rho', 1025.0, 'g', 9.81, 'current_speed', 0.25, ...
    'current_psi', deg2rad(0), 'current_alpha', 0);
env.vc_world = env.current_speed * [cos(env.current_alpha)*cos(env.current_psi); ...
    cos(env.current_alpha)*sin(env.current_psi); sin(env.current_alpha)];
names = {'link_002', 'link_003', 'link_004'};
volumes = [0.0001222, 0.0006235, 0.0005062];
cb = [0.0083 0.0081 0.0266; -0.0990 0.0280 0.0173; 0.1211 -0.0282 -0.0149];
% Real 4.STEP CAD frontal projected areas (m^2) for link_002, link_003, link_004
areas = [0.0288, 0.0420, 0.0728];
cd = [1.1, 1.1, 1.2];
added = [0.10 0.10 0.05; 0.45 0.50 0.20; 0.40 0.45 0.25];
hydro = repmat(struct('name','','mass',0,'volume',0,'com_local',zeros(1,3), ...
    'cb_local',zeros(1,3),'A_proj',0,'Cd',0,'added_mass',zeros(3)),1,3);
for i = 1:3
    body = getBody(robot, names{i});
    hydro(i).name = names{i};
    hydro(i).mass = body.Mass;
    hydro(i).volume = volumes(i);
    hydro(i).com_local = body.CenterOfMass;
    hydro(i).cb_local = cb(i,:);
    hydro(i).A_proj = areas(i);
    hydro(i).Cd = cd(i);
    hydro(i).added_mass = diag(added(i,:));
end
end

function [uvms, info] = build_chain_cad(urdfFile)
%BUILD_CHAIN_CAD  build_chain with the rigid-body data of the new CAD (13_pose_q1/mass_table.json).
%   Joint frames/kinematics are those of the old URDF (identical to the CAD joints: urdf_to_cad maps the
%   URDF joint centres/axes onto the CAD ones exactly, and the URDF link COMs land within 0.01-0.85 mm of
%   the CAD centroids -- diag_cad_frames.m). Only Mass, CenterOfMass and Inertia are replaced:
%     link_002 = l1, link_003 = l2, link_004 = link3 + pin4 + jaw1 + jaw2   (aluminium 2700 kg/m3)
%     auv_base = hull + base0, the hull mass set for neutral buoyancy of the whole vehicle
%                (kim 2026-10-22): m_hull = rho_w * V_total - m_arm(incl. base0); hull COM placed GM = 0.03 m
%                below the hull centroid (CAD -Y = down), hull inertia = CAD shape at uniform density.
%   info: masses, displacement, centre of buoyancy of every body (for the per-body buoyancy).
here = fileparts(mfilename('fullpath'));
uvms = build_chain(urdfFile);
T = jsondecode(fileread(fullfile(here, '..', '..', '13_pose_q1', 'mass_table.json')));
[R_bc, t_bc] = urdf_to_cad();                     % x_cad = R_bc * x_bl + t_bc
GM = 0.03;
rho_w = T.rho_water;
q0 = zeros(1, numel(homeConfiguration(uvms)));
if strcmp(uvms.DataFormat, 'column'), q0 = q0(:); end

% hull mass for neutral buoyancy
V_tot = T.hull.volume_m3;
m_arm = 0;
for g = {'base0', 'link1', 'link2', 'link3'}
    V_tot = V_tot + T.groups.(g{1}).volume_m3;
    m_arm = m_arm + T.groups.(g{1}).mass_kg;
end
m_hull = rho_w * V_tot - m_arm;
% Hull COM: GM below the hull centroid (CAD +Y up), and shifted horizontally (CAD x, z) so the total COM
% sits straight below the total centre of buoyancy at q = 0 -- the vehicle is trimmed, as the 'gm' mode
% assumes. Without the trim the arm's wet weight (~31 N, ~0.34 m off the hull centroid) lists it ~4 deg.
V_c = T.hull.volume_m3 * T.hull.centroid_m(:); m_c = zeros(3, 1);
for g = {'base0', 'link1', 'link2', 'link3'}
    x = T.groups.(g{1});
    V_c = V_c + x.volume_m3 * x.centroid_m(:);
    m_c = m_c + x.mass_kg * x.centroid_m(:);
end
cob = V_c / V_tot;                                  % total centre of buoyancy, CAD
c_hull = T.hull.centroid_m(:) - [0; GM; 0];
h = [1 3];                                          % horizontal CAD axes
c_hull(h) = (rho_w * V_tot * cob(h) - m_c(h)) / m_hull;
% inertia about the COM: the uniform-density shape inertia (how the lowered mass is distributed is not
% known; the alternatives differ by m*GM^2 = 0.4 kg m2, about 1 % of the hull inertia)
I_hull = (m_hull / T.hull.volume_m3) * T.hull.inertia_per_density_m5;
[m_b, c_b, I_b] = merge({m_hull, c_hull, I_hull}, ...
                        {T.groups.base0.mass_kg, T.groups.base0.centroid_m(:), T.groups.base0.inertia_kgm2});

bodies = {'auv_base', {m_b, c_b, I_b}; ...
          'link_002', grp(T.groups.link1); 'link_003', grp(T.groups.link2); 'link_004', grp(T.groups.link3)};
info = struct('m_hull', m_hull, 'm_arm_incl_base0', m_arm, 'V_total', V_tot, 'displacement_kg', rho_w * V_tot, ...
              'GM', GM, 'c_hull_cad', c_hull, 'trim_shift_mm', 1e3 * (c_hull(h) - T.hull.centroid_m(h))');
for k = 1:size(bodies, 1)
    name = bodies{k, 1}; d = bodies{k, 2};
    b = uvms.getBody(name);
    Tb = getTransform(uvms, q0, name);             % body frame in base_link at q = 0
    R_cb = R_bc * Tb(1:3, 1:3);                    % body axes -> CAD axes
    o_cad = R_bc * Tb(1:3, 4) + t_bc;              % body origin in CAD
    c_body = R_cb' * (d{2} - o_cad);               % COM in the body frame
    I_c = R_cb' * d{3} * R_cb;                     % inertia about the COM, body axes
    I_o = I_c + d{1} * (c_body' * c_body * eye(3) - c_body * c_body');   % about the body origin
    b.Mass = d{1};
    b.CenterOfMass = c_body';
    b.Inertia = [I_o(1, 1) I_o(2, 2) I_o(3, 3) I_o(2, 3) I_o(1, 3) I_o(1, 2)];
    replaceBody(uvms, name, b);
    info.(name) = struct('mass', d{1}, 'com_cad', d{2}');
end
% centres of buoyancy per body (displaced-volume centroid), body frame, and displaced mass
cb = {'auv_base', {T.hull, T.groups.base0}; 'link_002', {T.groups.link1}; ...
      'link_003', {T.groups.link2}; 'link_004', {T.groups.link3}};
for k = 1:size(cb, 1)
    parts = cb{k, 2}; V = 0; s = zeros(3, 1);
    for p = 1:numel(parts)
        V = V + parts{p}.volume_m3; s = s + parts{p}.volume_m3 * parts{p}.centroid_m(:);
    end
    Tb = getTransform(uvms, q0, cb{k, 1});
    R_cb = R_bc * Tb(1:3, 1:3); o_cad = R_bc * Tb(1:3, 4) + t_bc;
    info.buoy(k) = struct('body', cb{k, 1}, 'mass_displaced', rho_w * V, 'cob_body', (R_cb' * (s / V - o_cad))');
end
end

function d = grp(g)
d = {g.mass_kg, g.centroid_m(:), g.inertia_kgm2};
end

function [m, c, I] = merge(a, b)
m = a{1} + b{1};
c = (a{1} * a{2} + b{1} * b{2}) / m;
I = zeros(3);
for p = {a, b}
    x = p{1}; d = x{2} - c;
    I = I + x{3} + x{1} * (d' * d * eye(3) - d * d');
end
end

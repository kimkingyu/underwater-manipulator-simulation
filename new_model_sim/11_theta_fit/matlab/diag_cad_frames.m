function diag_cad_frames()
%DIAG_CAD_FRAMES  Check the CAD -> URDF body-frame mapping used by build_chain_cad: the old URDF link COMs,
%   carried to CAD coordinates through getTransform (q = 0) and urdf_to_cad, must land on the CAD volume
%   centroids of the matching parts (13_pose_q1/mass_table.json). Prints the tree and the distances.
here = fileparts(mfilename('fullpath'));
uvms = build_chain(fullfile(here, '..', '..', '..', 'model', 'robot.urdf'));
uvms.DataFormat = 'row';
T = jsondecode(fileread(fullfile(here, '..', '..', '13_pose_q1', 'mass_table.json')));
[R_bc, t_bc] = urdf_to_cad();
q0 = zeros(1, numel(homeConfiguration(uvms)));
fprintf('%d bodies:', uvms.NumBodies);
for i = 1:uvms.NumBodies
    b = uvms.Bodies{i};
    fprintf(' %s(%.3g kg, parent %s)', b.Name, b.Mass, b.Parent.Name);
end
fprintf('\n');
map = {'link_002', 'link1'; 'link_003', 'link2'; 'link_004', 'link3'};
for k = 1:size(map, 1)
    b = uvms.getBody(map{k, 1});
    Tb = getTransform(uvms, q0, map{k, 1});
    c_bl = Tb(1:3, 1:3) * b.CenterOfMass(:) + Tb(1:3, 4);
    c_cad = R_bc * c_bl + t_bc;
    g = T.groups.(map{k, 2});
    fprintf('%s: URDF COM in CAD %s mm, CAD centroid %s mm, distance %.2f mm\n', map{k, 1}, ...
        mat2str(round(c_cad' * 1e3, 1)), mat2str(round(g.centroid_m' * 1e3, 1)), norm(c_cad - g.centroid_m) * 1e3);
end
b = uvms.getBody('auv_base');
c_cad = R_bc * b.CenterOfMass(:) + t_bc;
fprintf('auv_base: URDF COM in CAD %s mm, CAD hull centroid %s mm\n', mat2str(round(c_cad' * 1e3, 1)), ...
    mat2str(round(T.hull.centroid_m' * 1e3, 1)));
end

function G = uvms_geometry()
%UVMS_GEOMETRY  Geometry of the new AUV + 3-joint arm used by Y_uvms / fit_theta.
%   Body frame B: CAD axes of AUVM.step (+Y up), origin O_b = hull volume centroid.
%   All vectors below are expressed in B at q = 0 (the CAD pose used in every CFD case).
%   Joint centres/axes: old URDF mapped to the new CAD (02_ansys_read_check/joint_candidates_from_old_urdf.json).
%   Link reference points: volume centroids of l1, l2 and link3+jaw1+jaw2+pin4 (body_centroids.json).
%   Link frame i: origin at its reference point, axes = B axes rotated by the joint rotations up to i.

O = [0.0, 0.29386, -0.06630];                               % O_b in CAD coordinates [m]
c = [-4e-06, -0.0366, -0.460702;                            % joint centres, CAD [m]
      0.016686, -0.0666, -0.511202;
      0.014796, -0.1226, -0.313202];
G.joint_axis = [0 0 -1; -1 0 0; 1 0 0];                     % unit joint axes in B at q = 0
G.joint_pos  = c - O;                                       % joint centres relative to O_b, B axes
cl = [ 0.008435498615527, -0.044975304407476, -0.487494357028531;
      -0.001229762295081, -0.094600000000002, -0.412200000000006;
      -0.000070233630796, -0.151150773375763, -0.435756276305762];
G.link_pos = cl - O;                                        % link reference points relative to O_b, B axes
G.n = 3;
G.names = {'link1 (l1)', 'link2 (l2)', 'link3 (link3+jaws+pin)'};
end

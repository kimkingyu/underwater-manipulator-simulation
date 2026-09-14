function Phi = hydro_buoyancy_features(q)
%HYDRO_BUOYANCY_FEATURES Tensor-product Fourier basis, fitted from data.
% 27 columns: products of {1,sin(qj),cos(qj)} for the three joints.
% No robot kinematics or assumed buoyancy coefficients are evaluated here.
% Static buoyancy depends on posture, not velocity, acceleration or current.
validateattributes(q,{'numeric'},{'real','finite','2d','ncols',3,'nonempty'});
Phi = ones(size(q,1),1);
for j = 1:3
    Phi = [Phi, Phi.*sin(q(:,j)), Phi.*cos(q(:,j))]; %#ok<AGROW>
end
end

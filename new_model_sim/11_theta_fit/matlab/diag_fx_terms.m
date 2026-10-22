function diag_fx_terms()
%DIAG_FX_TERMS  Which theta terms make up the predicted AUV Fx in the +x steady cases at 0.3 and 0.8 m/s:
%   contribution of every nonzero theta column to row 1 of Y*theta, largest first.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'theta_fit.mat'));
G = uvms_geometry();
for u = [0.3 0.8]
    Yg = Y_uvms(zeros(3, 1), [u 0 0 0 0 0 0 0 0]', zeros(9, 1), G);
    contrib = -Yg(1, :)' .* S.theta;                 % fluid Fx (sign as the CFD load)
    [~, o] = sort(abs(contrib), 'descend');
    fprintf('u = %.1f m/s: predicted Fx %.2f N\n', u, sum(contrib));
    for i = o(1:6)'
        if abs(contrib(i)) < 1e-6, break; end
        fprintf('   %-10s theta %10.4g  -> %+8.2f N\n', S.names{i}, S.theta(i), contrib(i));
    end
end
end

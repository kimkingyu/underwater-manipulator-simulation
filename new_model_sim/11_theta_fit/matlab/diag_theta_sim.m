function diag_theta_sim()
%DIAG_THETA_SIM  Break the theta hydrodynamic force at early states of sim_uvms_theta into parameter
%   groups, and compare the early arm motion of the two hydro models.
here = fileparts(mfilename('fullpath'));
o = load(fullfile(here, 'sim_uvms_theta.mat'));
h0 = load(fullfile(here, 'sim_uvms_handset.mat'));
S = load(fullfile(here, 'theta_fit.mat')); th = S.theta;
G = uvms_geometry();
groups = {'AUV MA', 1:6; 'AUV Dl', 7:12; 'AUV Dq', 13:18; 'link A', [19:21 25:27 31:33]; ...
          'link Dq', [22:24 28:30 34:36]; 'coupling', 43:60};
fprintf('t      | q_arm (theta)          | qd_arm (theta)          | qd_arm (handset)        | qd AUV p q r (theta)\n');
for t = [0.005 0.01 0.02 0.03 0.05 0.1]
    k = find(abs(o.time - t) < 1e-9, 1);
    fprintf('%5.3f | %s | %s | %s | %s\n', t, mat2str(o.q(k, 7:9), 3), mat2str(o.qd(k, 7:9), 3), ...
        mat2str(h0.qd(k, 7:9), 3), mat2str(o.qd(k, 4:6), 3));
end
for t = [0.01 0.05 0.1]
    k = find(abs(o.time - t) < 1e-9, 1);
    q = o.q(k, :)'; qd = o.qd(k, :)';
    fprintf('\nt = %.3f  state-only hydro force by group (joint rows 7:9 | AUV rows 1:6 norm)\n', t);
    for g = 1:size(groups, 1)
        tg = zeros(size(th)); tg(groups{g, 2}) = th(groups{g, 2});
        h = uvms_hydro_theta(q, qd, zeros(9, 1), tg, G);
        fprintf('  %-9s  J = [%8.4f %8.4f %8.4f]   |AUV F| %8.3f  |AUV M| %8.3f\n', groups{g, 1}, h(7:9), norm(h(1:3)), norm(h(4:6)));
    end
end
end

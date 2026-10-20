function T = compare_sim_vs_cfd(dataset_file)
%COMPARE_SIM_VS_CFD  AUV hydrodynamic loads of three models against the 22 Fluent cases.
%   sim  : hand-set parameters of experiments/simulate_uvms_dynamics.m (old URDF, m_auv = 127.85 kg):
%          M_A = diag([0.4 0.4 0.5]*m_auv, 0.3*20*[1 1 1]), D_l = [60 60 80 25 25 20],
%          D_q = [120 120 150 40 40 35]. Axes taken as the body axes of B, as that script does.
%   v1   : fitted theta, diagonal only (v1_diag42/theta_fit.mat)
%   v2   : fitted theta with translation->moment coupling (theta_fit.mat)
%   Fluid load = -(M_A nudot + D(nu) nu) for the diagonal models, -Y(0,[nu;0],[nudot;0])*theta for v1/v2.
%   The arm is fixed (q = 0); the sim's joint damping D_arm only acts on qdot, so it predicts 0 joint
%   torque in every case -- reported as such.
if nargin < 1, dataset_file = fullfile(fileparts(mfilename('fullpath')), '..', 'theta_dataset.json'); end
here = fileparts(mfilename('fullpath'));
S = jsondecode(fileread(dataset_file));
C = S.cases; if isstruct(C), C = num2cell(C); end
G = uvms_geometry();
m_auv = 127.84851;
MA_sim = diag([0.4 0.4 0.5 0 0 0] * m_auv + [0 0 0 0.3 0.3 0.3] * 20.0);
Dl_sim = [60 60 80 25 25 20]'; Dq_sim = [120 120 150 40 40 35]';
th1 = load(fullfile(here, 'v1_diag42', 'theta_fit.mat')); th1 = th1.theta;
th2 = load(fullfile(here, 'theta_fit.mat')); th2 = th2.theta;

ids = {}; cfd = []; sim = []; p1 = []; p2 = [];
for c = 1:numel(C)
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    F = reshape(k.F, [], 3); M = reshape(k.M, [], 3); tq = reshape(k.tau, [], 3);
    for s = 1:size(nu, 1)
        n_ = nu(s, :)'; a_ = nd(s, :)';
        ids{end + 1, 1} = sprintf('%s#%d', k.id, s); %#ok<AGROW>
        cfd(end + 1, :) = [F(s, :) M(s, :) tq(s, :)]; %#ok<AGROW>
        sim(end + 1, :) = [-(MA_sim * a_ + Dl_sim .* n_ + Dq_sim .* abs(n_) .* n_)' 0 0 0]; %#ok<AGROW>
        Y = Y_uvms(zeros(3, 1), [n_; 0; 0; 0], [a_; 0; 0; 0], G);
        p1(end + 1, :) = -(Y(:, 1:42) * th1)'; %#ok<AGROW>
        p2(end + 1, :) = -(Y * th2)'; %#ok<AGROW>
    end
end
grp = {1:3, 4:6, 7:9};
nm = {'F', 'M', 'tau'};
fprintf('\nError vs CFD, RMS over all samples of ||pred - CFD|| / RMS ||CFD||  (0 = perfect, 1 = as bad as predicting 0)\n');
fprintf('%-6s %8s %8s %8s\n', '', 'sim', 'theta_v1', 'theta_v2');
for g = 1:3
    ix = grp{g};
    r = @(P) sqrt(mean(sum((P(:, ix) - cfd(:, ix)) .^ 2, 2))) / sqrt(mean(sum(cfd(:, ix) .^ 2, 2)));
    fprintf('%-6s %8.3f %8.3f %8.3f\n', nm{g}, r(sim), r(p1), r(p2));
end
lab = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'tau1', 'tau2', 'tau3'};
T = table(ids, 'VariableNames', {'sample'});
for j = 1:9
    T.(['cfd_' lab{j}]) = cfd(:, j);
    T.(['sim_' lab{j}]) = sim(:, j);
    T.(['v2_' lab{j}]) = p2(:, j);
end
writetable(T, fullfile(here, 'compare_sim_vs_cfd.csv'));
% key coefficients side by side
fprintf('\n%-6s %10s %10s\n', 'coef', 'sim', 'CFD fit v2');
cn = {'MA_u', 'MA_v', 'MA_w', 'MA_p', 'MA_q', 'MA_r', 'Dl_u', 'Dl_v', 'Dl_w', 'Dl_p', 'Dl_q', 'Dl_r', ...
      'Dq_u', 'Dq_v', 'Dq_w', 'Dq_p', 'Dq_q', 'Dq_r'};
sv = [diag(MA_sim); Dl_sim; Dq_sim];
for i = 1:18, fprintf('%-6s %10.3g %10.3g\n', cn{i}, sv(i), th2(i)); end
end

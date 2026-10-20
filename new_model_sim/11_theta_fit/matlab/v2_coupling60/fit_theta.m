function out = fit_theta(dataset_file, out_dir, coupling)
%FIT_THETA  Least-squares fit of theta (Y_uvms) to the Fluent CFD dataset, plus leave-one-case-out check.
%   dataset_file : theta_dataset.json written by ../build_dataset.py
%   out_dir      : where theta_fit.mat / theta_fit.json / theta_table.csv / loo_errors.csv go
%
%   Every CFD sample is a rigid motion of the whole vehicle at q = 0, qdot = 0:
%       zeta = [nu; 0], a = [nudot; 0].
%   CFD gives the FLUID load on the vehicle, which is the negative of what Y*theta represents
%   (Y*theta sits on the left-hand side with M zeta_dot + C zeta + D), so the targets are
%       Y(1:9,:)  theta = -[F; M; tau]        (9 generalised components)
%       Ylink     theta = -flink              (translational fluid force on links 1..3, link frames)
%   The link rows are needed to separate the link parameters from the hull parameters.
%   Weights: each channel scaled by its RMS over the data set; each CASE carries the same total
%   weight (rows of a case with m samples are weighted 1/sqrt(m)).
%   Coupling entries 43..60 (AUV translation -> moment, linear and quadratic) are unconstrained in sign.
%   Constraints: all other hydrodynamic entries >= 0; friction entries (37..42) fixed to 0; Dq_p/q/r
%   (16..18) fixed to 0 -- rotational damping is linear only, from the velocity-phase part of the
%   laminar rotary oscillations O_p/O_q/O_r.
%   Oscillation cases enter as one unit-acceleration sample (in-phase part); rotary ones add one
%   peak-rate sample (velocity-phase part). See ../build_dataset.py.
if nargin < 1, dataset_file = fullfile(fileparts(mfilename('fullpath')), '..', 'theta_dataset.json'); end
if nargin < 2, out_dir = fileparts(mfilename('fullpath')); end
G = uvms_geometry();
S = jsondecode(fileread(dataset_file));
C = S.cases;
if isstruct(C), C = num2cell(C); end
nc = numel(C);
[names, units] = theta_names(G.n);
np = numel(names);
fixed = false(np, 1); fixed(37:42) = true;
fixed(16:18) = true;   % Dq_p, Dq_q, Dq_r: no steady-rotation data (kim 2026-10-20), fixed to 0
signed = false(np, 1); signed(43:60) = true;   % translation -> moment coupling: no sign constraint
% Coupling form: 'lin' (default) | 'quad' | 'both' | 'none'. Leave-one-case-out moment error
% (data-set RMS normalised): none 1.067, lin 0.715, quad 0.775, both 0.798 -> 'lin' generalises best;
% 'both' fits tighter but over-fits. Unused coupling entries stay 0 in theta.
if nargin < 3 || isempty(coupling), coupling = 'lin'; end
switch coupling
    case 'quad', fixed(43:51) = true;
    case 'lin',  fixed(52:60) = true;
    case 'none', fixed(43:60) = true;
    case 'both'
    otherwise, error('fit_theta: unknown coupling ''%s''', coupling);
end

% ---------- assemble ----------
H = cell(nc, 1); y = cell(nc, 1); ch = cell(nc, 1);
for c = 1:nc
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    F = reshape(k.F, [], 3); M = reshape(k.M, [], 3); T = reshape(k.tau, [], 3); L = reshape(k.flink, [], 9);
    m = size(nu, 1);
    Hc = zeros(18 * m, np); yc = zeros(18 * m, 1);
    for s = 1:m
        [Yg, Yl] = Y_uvms(zeros(3, 1), [nu(s, :)'; 0; 0; 0], [nd(s, :)'; 0; 0; 0], G);
        r = 18 * (s - 1);
        Hc(r + (1:9), :) = Yg;   yc(r + (1:9)) = -[F(s, :)'; M(s, :)'; T(s, :)'];
        Hc(r + (10:18), :) = Yl; yc(r + (10:18)) = -L(s, :)';
    end
    H{c} = Hc; y{c} = yc; ch{c} = repmat((1:18)', m, 1);
end
chall = vertcat(ch{:}); yall = vertcat(y{:});
grp = [1 1 1 2 2 2 3 3 3 4 4 4 4 4 4 4 4 4];         % F, M, tau, link force share one scale per group
scale = zeros(18, 1);
for g = 1:4
    idx = ismember(chall, find(grp == g));
    scale(grp == g) = max(sqrt(mean(yall(idx) .^ 2)), eps);
end

theta = solve(1:nc);
% ---------- excitation / identifiability ----------
[Hw, ~] = stack(1:nc);
Hf = Hw(:, ~fixed);
cn = sqrt(sum(Hf .^ 2, 1));
sv = svd(Hf ./ max(cn, eps));
excited = false(np, 1); excited(~fixed) = cn' > 1e-9 * max(cn);

% ---------- fit residual and leave-one-case-out ----------
resid = case_errors(theta, 1:nc);
loo = nan(nc, 8); loo_theta = nan(np, nc);
for c = 1:nc
    keep = setdiff(1:nc, c);
    th = solve(keep);
    loo_theta(:, c) = th;
    loo(c, :) = case_errors(th, c);
end

ids = cellfun(@(k) k.id, C, 'UniformOutput', false);
out = struct('names', {names}, 'units', {units}, 'theta', theta, 'fixed', fixed, 'excited', excited, ...
    'case_ids', {ids}, 'fit_rel_err', resid, 'loo_rel_err', loo, 'loo_theta', loo_theta, ...
    'channel_scale', scale, 'cond_scaled', sv(1) / max(sv(end), eps), 'singular_values_scaled', sv);
save(fullfile(out_dir, 'theta_fit.mat'), '-struct', 'out');
fid = fopen(fullfile(out_dir, 'theta_table.csv'), 'w');
fprintf(fid, 'index,name,value,unit,excited_by_data,fixed\n');
for i = 1:np
    fprintf(fid, '%d,%s,%.6g,%s,%d,%d\n', i, names{i}, theta(i), units{i}, excited(i), fixed(i));
end
fclose(fid);
fid = fopen(fullfile(out_dir, 'loo_errors.csv'), 'w');
fprintf(fid, ['case,fit_rel_F,fit_rel_M,fit_rel_tau,fit_rel_link,fit_nrms_F,fit_nrms_M,fit_nrms_tau,fit_nrms_link,' ...
    'loo_rel_F,loo_rel_M,loo_rel_tau,loo_rel_link,loo_nrms_F,loo_nrms_M,loo_nrms_tau,loo_nrms_link\n']);
for c = 1:nc
    fprintf(fid, '%s%s\n', ids{c}, sprintf(',%.4f', [resid(c, :), loo(c, :)]));
end
fclose(fid);
J = out; J = rmfield(J, {'loo_theta', 'singular_values_scaled'});
J.note = ['error columns 1-4 (rel): ||pred - CFD|| / ||CFD|| of the case; columns 5-8 (nrms): RMS(pred - CFD) ' ...
    '/ data-set RMS of the channel group. Groups: F AUV force, M AUV moment about O_b, tau joint torques, ' ...
    'link link forces. theta(16:18), theta(37:42) fixed to 0; coupling form: ' coupling '.'];
J.coupling = coupling;
fid = fopen(fullfile(out_dir, 'theta_fit.json'), 'w'); fprintf(fid, '%s', jsonencode(J, 'PrettyPrint', true)); fclose(fid);

    function [Hw, yw] = stack(cs)
        Hw = []; yw = [];
        for cc = cs
            m_ = numel(y{cc}) / 18;
            w = 1 ./ (scale(ch{cc}) * sqrt(m_));
            Hw = [Hw; H{cc} .* w]; %#ok<AGROW>
            yw = [yw; y{cc} .* w]; %#ok<AGROW>
        end
    end

    function th = solve(cs)
        [Hw_, yw_] = stack(cs);
        cn_ = sqrt(sum(Hw_ .^ 2, 1));
        act = ~fixed' & cn_ > 1e-9 * max(cn_);         % columns this subset of cases actually excites
        A_ = Hw_(:, act) ./ cn_(act);
        opt = optimoptions('lsqlin', 'Display', 'off', 'Algorithm', 'active-set');
        lb_ = zeros(np, 1); lb_(signed) = -inf;         % coupling terms may take either sign
        lb_ = lb_(act);
        z = lsqlin(A_, yw_, [], [], [], [], lb_, [], max(lb_, 0), opt);
        th = zeros(np, 1);                              % non-excited / fixed entries stay exactly 0
        th(act) = z ./ cn_(act)';
    end

    function e = case_errors(th, cs)
        % columns 1-4: ||pred - CFD|| / ||CFD|| of this case (relative to the case's own load)
        % columns 5-8: RMS(pred - CFD) / RMS of that channel group over the whole data set
        e = nan(numel(cs), 8);
        for ii = 1:numel(cs)
            cc = cs(ii);
            p = H{cc} * th;
            for g_ = 1:4
                idx_ = ismember(ch{cc}, find(grp == g_));
                e(ii, g_) = norm(p(idx_) - y{cc}(idx_)) / max(norm(y{cc}(idx_)), eps);
                e(ii, 4 + g_) = sqrt(mean((p(idx_) - y{cc}(idx_)) .^ 2)) / scale(find(grp == g_, 1));
            end
        end
    end
end

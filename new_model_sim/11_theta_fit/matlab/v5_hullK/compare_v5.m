function compare_v5()
%COMPARE_V5  Current theta (theta_fit.mat, 87) vs v5_hullK (105, full quadratic hull damping), on one
%   scale (q = 0 channel_scale of the current fit):
%     q = 0: fit and leave-one-case-out nrms per group (RMS over cases), steady and oscillation split
%     extra poses (qb8, qc, qd): prediction without refit, nrms per group and per-joint RMS error
%     a few channels of the cases that drove the change (T_px_03 Fx, T_py_08 Fx, T_xz_06 Fx)
%   Writes compare_v5.txt next to this file.
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
A = load(fullfile(root, 'theta_fit.mat')); B = load(fullfile(here, 'theta_fit.mat'));
sc = A.channel_scale(:);
gn = {'F', 'M', 'tau', 'link', 'linkM'};
fid = fopen(fullfile(here, 'compare_v5.txt'), 'w');
out = @(varargin) cellfun(@(f) fprintf(f, varargin{:}), {1, fid}, 'UniformOutput', false);
ids = A.case_ids(:); st = startsWith(ids, 'T_');
% loo/fit nrms columns 6:10, but each fit used its own channel_scale: rescale B to A's scale
rs = @(S, cols) S(:, cols) .* (B.channel_scale([1 4 7 10 13])' ./ sc([1 4 7 10 13])');
eAf = A.fit_rel_err(:, 6:10); eAl = A.loo_rel_err(:, 6:10);
eBf = rs(B.fit_rel_err, 6:10); eBl = rs(B.loo_rel_err, 6:10);
r = @(e) sqrt(mean(e .^ 2, 1));
out('q = 0 (22 cases), nrms on the current scale        %s\n', strjoin(cellfun(@(g) sprintf('%7s', g), gn, 'UniformOutput', false), ''));
out('  fit           current %s\n', sprintf('%7.3f', r(eAf)));
out('  fit           v5      %s\n', sprintf('%7.3f', r(eBf)));
out('  LOO all       current %s\n', sprintf('%7.3f', r(eAl)));
out('  LOO all       v5      %s\n', sprintf('%7.3f', r(eBl)));
out('  LOO steady    current %s\n', sprintf('%7.3f', r(eAl(st, :))));
out('  LOO steady    v5      %s\n', sprintf('%7.3f', r(eBl(st, :))));
out('  LOO oscill.   current %s\n', sprintf('%7.3f', r(eAl(~st, :))));
out('  LOO oscill.   v5      %s\n', sprintf('%7.3f', r(eBl(~st, :))));
G = uvms_geometry();
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
out('\nextra poses, prediction without refit (nrms, current scale) + per-joint RMS error / RMS CFD [N m]\n');
for tag = {'qb8', 'qc', 'qd'}
    P = jsondecode(fileread(fullfile(root, '..', ['theta_dataset_' tag{1} '.json'])));
    pc = P.cases; if isstruct(pc), pc = num2cell(pc); end
    for which = 1:2
        if which == 1, th = A.theta; G.hull_K = false; lab = 'current'; else, th = B.theta; G.hull_K = true; lab = 'v5     '; end
        e = zeros(numel(pc), 5); tc = zeros(numel(pc), 3); tp = tc;
        for c = 1:numel(pc)
            k = pc{c};
            y = [k.F(:); k.M(:); k.tau(:); k.flink(:)];
            [Yg, Yl] = Y_uvms(P.q(:), [k.nu(:); 0; 0; 0], zeros(9, 1), G);
            p = -[Yg; Yl] * th;
            for g = 1:5, idx = grp == g; e(c, g) = sqrt(mean((p(idx) - y(idx)) .^ 2)) / sc(find(idx, 1)); end
            tc(c, :) = y(7:9)'; tp(c, :) = p(7:9)';
        end
        out('  %-4s %s %s | J %s / %s\n', tag{1}, lab, sprintf('%7.3f', r(e)), ...
            sprintf('%6.3f', sqrt(mean((tp - tc) .^ 2))), sprintf('%6.3f', sqrt(mean(tc .^ 2))));
    end
end
D = jsondecode(fileread(fullfile(root, '..', 'theta_dataset.json')));
C = D.cases; if isstruct(C), C = num2cell(C); end
out('\nselected channels, CFD / current / v5 (fluid load, N or N m)\n');
pick = {'T_px_03', 1; 'T_px_08', 1; 'T_py_08', 1; 'T_ny_08', 3; 'T_xz_06', 1; 'T_xyz_06', 2; 'T_xyz_06', 3};
nm = {'Fx', 'Fy', 'Fz'};
for i = 1:size(pick, 1)
    k = C{strcmp(cellfun(@(x) x.id, C, 'UniformOutput', false), pick{i, 1})};
    z = [k.nu(:); 0; 0; 0];
    G.hull_K = false; pa = -Y_uvms(zeros(3, 1), z, zeros(9, 1), G) * A.theta;
    G.hull_K = true;  pb = -Y_uvms(zeros(3, 1), z, zeros(9, 1), G) * B.theta;
    j = pick{i, 2};
    out('  %-9s %s  %8.2f / %8.2f / %8.2f\n', pick{i, 1}, nm{j}, k.F(j), pa(j), pb(j));
end
fclose(fid);
end

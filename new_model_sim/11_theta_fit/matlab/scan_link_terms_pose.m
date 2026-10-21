function R = scan_link_terms_pose(pose_tags)
%SCAN_LINK_TERMS_POSE  Multi-pose version of scan_link_terms: q = 0 data set (theta_dataset.json, all 22
%   cases incl. oscillation) plus steady sets at extra poses (theta_dataset_<tag>.json, own q per case).
%   Each variant replaces columns 61:87 of Y_uvms with a link-term regressor built from the link
%   velocities AT THE CASE'S POSE (uvms_link_kin(q)), then:
%     LOO-case : leave one case out (as fit_theta),
%     LOO-pose : leave out every case of one extra pose (train on q = 0 + the other extra poses) --
%                prediction at an unseen pose.
%   Constraints as fit_theta defaults (coupling 'lin': 16:18, 37:42, 52:60 fixed; 43:87 signed).
%   Errors: nrms per group = RMS(pred - CFD) / q = 0 data-set RMS of the group (same scale for every
%   variant and every split), RMS over the cases of the split. Writes scan_link_terms_pose.json.
%   pose_tags default {'qb8', 'qc', 'qd'}.
if nargin < 1, pose_tags = {'qb8', 'qc', 'qd'}; end
here = fileparts(mfilename('fullpath'));
G = uvms_geometry();
S = jsondecode(fileread(fullfile(here, '..', 'theta_dataset.json')));
C = S.cases; if isstruct(C), C = num2cell(C); end
for c = 1:numel(C), C{c}.q = zeros(3, 1); C{c}.pose = 'q0'; end
for t = 1:numel(pose_tags)
    P = jsondecode(fileread(fullfile(here, '..', ['theta_dataset_' pose_tags{t} '.json'])));
    pc = P.cases; if isstruct(pc), pc = num2cell(pc); end
    for c = 1:numel(pc)
        k = pc{c};
        k.id = [k.id '@' pose_tags{t}]; k.q = P.q(:); k.pose = pose_tags{t};
        k.nudot = zeros(size(k.nu)); k.F = k.F(:)'; k.M = k.M(:)'; k.tau = k.tau(:)'; k.flink = k.flink(:)';
        k.nu = k.nu(:)';
        C{end + 1} = k; %#ok<AGROW>
    end
end
nc = numel(C);
pose = cellfun(@(k) k.pose, C, 'UniformOutput', false);
fprintf('%d cases: %s\n', nc, strjoin(cellfun(@(p) sprintf('%s %d', p, nnz(strcmp(pose, p))), ...
    unique(pose, 'stable'), 'UniformOutput', false), ', '));

variants = {'none', 'cop_moment', 'cross_force', 'cross_force_l3', 'cop+cross_l3'};
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
gn = {'F', 'M', 'tau', 'link', 'linkM'};
% common scale: q = 0 cases only (identical to fit_theta's channel_scale on the q = 0 set)
[H0, y0, ch0] = assemble(C(strcmp(pose, 'q0')), G, 'none');
yall = vertcat(y0{:}); chall = vertcat(ch0{:});
scale = zeros(27, 1);
for g = 1:5
    idx = ismember(chall, find(grp == g)); scale(grp == g) = max(sqrt(mean(yall(idx) .^ 2)), eps);
end
clear H0
R = struct('variants', {variants}, 'groups', {gn}, 'case_ids', {cellfun(@(k) k.id, C, 'UniformOutput', false)});
xposes = pose_tags;
for v = 1:numel(variants)
    [H, y, ch] = assemble(C, G, variants{v});
    np = size(H{1}, 2);
    % LOO-case
    e = zeros(nc, 5);
    for c = 1:nc
        th = solve(H, y, ch, setdiff(1:nc, c), scale, variants{v}, np);
        e(c, :) = errs(H{c} * th, y{c}, ch{c}, grp, scale);
    end
    % LOO-pose
    ep = cell(numel(xposes), 1); jt = cell(numel(xposes), 1);
    for t = 1:numel(xposes)
        out = find(strcmp(pose, xposes{t}));
        th = solve(H, y, ch, setdiff(1:nc, out), scale, variants{v}, np);
        ep{t} = zeros(numel(out), 5); jt{t} = zeros(numel(out), 6);
        for ii = 1:numel(out)
            p = H{out(ii)} * th;
            ep{t}(ii, :) = errs(p, y{out(ii)}, ch{out(ii)}, grp, scale);
            jt{t}(ii, :) = [-y{out(ii)}(7:9)' -p(7:9)'];   % CFD tau, predicted tau (fluid load)
        end
    end
    th_all = solve(H, y, ch, 1:nc, scale, variants{v}, np);
    eall = zeros(nc, 5);
    for c = 1:nc, eall(c, :) = errs(H{c} * th_all, y{c}, ch{c}, grp, scale); end
    row.variant = variants{v};
    row.fit_all = sqrt(mean(eall .^ 2));
    row.loo_case_q0 = sqrt(mean(e(strcmp(pose, 'q0'), :) .^ 2));
    row.loo_case_xpose = sqrt(mean(e(~strcmp(pose, 'q0'), :) .^ 2));
    row.loo_pose = cellfun(@(x) sqrt(mean(x .^ 2, 1)), ep, 'UniformOutput', false);
    row.loo_pose_all = sqrt(mean(vertcat(ep{:}) .^ 2));
    row.loo_pose_tau = jt;
    R.rows(v) = row;
    fprintf('%-14s fit %s | LOOcase q0 %s | LOOcase xpose %s | LOOpose %s\n', variants{v}, ...
        sprintf('%.3f ', row.fit_all), sprintf('%.3f ', row.loo_case_q0), ...
        sprintf('%.3f ', row.loo_case_xpose), sprintf('%.3f ', row.loo_pose_all));
end
R.scale = scale; R.pose_tags = pose_tags;
fid = fopen(fullfile(here, 'scan_link_terms_pose.json'), 'w');
fprintf(fid, '%s', jsonencode(R, 'PrettyPrint', true)); fclose(fid);
end

function [H, y, ch] = assemble(C, G, variant)
nc = numel(C); H = cell(nc, 1); y = cell(nc, 1); ch = cell(nc, 1);
for c = 1:nc
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    F = reshape(k.F, [], 3); M = reshape(k.M, [], 3); T = reshape(k.tau, [], 3);
    L = reshape(k.flink, [], 18); m = size(nu, 1); q = k.q(:);
    [J, ~] = uvms_link_kin(q, zeros(3, 1), G);
    ncol = 87; if strcmp(variant, 'cop+cross_l3'), ncol = 87 + 6; end
    Hc = zeros(27 * m, ncol); yc = zeros(27 * m, 1);
    for s = 1:m
        z = [nu(s, :)'; 0; 0; 0]; a = [nd(s, :)'; 0; 0; 0];
        [Yg, Yl] = Y_uvms(q, z, a, G);
        Yg = [Yg zeros(9, ncol - 87)]; Yl = [Yl zeros(18, ncol - 87)]; %#ok<AGROW>
        if ~any(strcmp(variant, {'cop_moment', 'cop+cross_l3'}))
            Yg(:, 61:87) = 0; Yl(:, 61:87) = 0;
        end
        for i = 1:3
            vi = J{i} * z; vi = vi(1:3); qv = abs(vi) .* vi; rl = 6 * (i - 1) + (1:6);
            col_x = 87;
            for kk = 1:3
                for j = 1:3
                    col = 60 + 9 * (i - 1) + 3 * (kk - 1) + j;
                    wr = zeros(6, 1); use = false;
                    switch variant
                        case 'cross_force', if j ~= kk, wr(j) = qv(kk); use = true; end
                        case 'cross_force_l3', if i == 3 && j ~= kk, wr(j) = qv(kk); use = true; end
                    end
                    if use, Yg(:, col) = J{i}' * wr; Yl(rl, col) = wr; end
                    if strcmp(variant, 'cop+cross_l3') && i == 3 && j ~= kk
                        col_x = col_x + 1; wr = zeros(6, 1); wr(j) = qv(kk);
                        Yg(:, col_x) = J{i}' * wr; Yl(rl, col_x) = wr;
                    end
                end
            end
        end
        r = 27 * (s - 1);
        Hc(r + (1:9), :) = Yg; yc(r + (1:9)) = -[F(s, :)'; M(s, :)'; T(s, :)'];
        Hc(r + (10:27), :) = Yl; yc(r + (10:27)) = -L(s, :)';
    end
    H{c} = Hc; y{c} = yc; ch{c} = repmat((1:27)', m, 1);
end
end

function th = solve(H, y, ch, keep, scale, variant, np)
fixed = false(np, 1); fixed([16:18 37:42 52:60]) = true;
signed = false(np, 1); signed(43:np) = true;
if strcmp(variant, 'none'), fixed(61:87) = true; end
Hw = []; yw = [];
for cc = keep
    w = 1 ./ (scale(ch{cc}) * sqrt(numel(y{cc}) / 27));
    Hw = [Hw; H{cc} .* w]; yw = [yw; y{cc} .* w]; %#ok<AGROW>
end
cn = sqrt(sum(Hw .^ 2, 1)); act = ~fixed' & cn > 1e-9 * max(cn);
lb = zeros(np, 1); lb(signed) = -inf; lb = lb(act);
opt = optimoptions('lsqlin', 'Display', 'off', 'Algorithm', 'active-set');
zz = lsqlin(Hw(:, act) ./ cn(act), yw, [], [], [], [], lb, [], max(lb, 0), opt);
th = zeros(np, 1); th(act) = zz ./ cn(act)';
end

function e = errs(p, y, ch, grp, scale)
e = zeros(1, 5);
for g = 1:5
    idx = ismember(ch, find(grp == g));
    e(g) = sqrt(mean((p(idx) - y(idx)) .^ 2)) / scale(find(grp == g, 1));
end
end

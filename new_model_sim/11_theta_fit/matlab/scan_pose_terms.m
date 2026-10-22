function R = scan_pose_terms(pose_tags)
%SCAN_POSE_TERMS  Pose-dependent extensions of the theta model, judged by leave-one-POSE-out.
%   Base = current Y_uvms (87 columns, centre-of-pressure link terms). Variants add columns:
%     base         : nothing (reference, = cop_moment of scan_link_terms_pose)
%     qlin_l23     : for links 2 and 3, the quadratic-damping (3) and CoP (9) columns times q2 and times q3
%                    (24 per link, 48 total): drag/CoP coefficients linear in the distal joint angles
%     qlin_l3      : the same for link 3 only (24)
%     jq           : joint torque j += sum_k Q(j,k,m) q_m |u_k| u_k, m = 2, 3, body-frame AUV velocity
%                    (3 joints x 3 dirs x 2 angles = 18), no link wrench
%     jq_abs       : as jq but with |q_m| (even in q)
%   All added columns are signed. Same data, constraints, scale and splits as scan_link_terms_pose.
%   Also reports a ridge-regularised qlin_l23 (lambda chosen by leave-one-pose-out on the training poses).
%   Writes scan_pose_terms.json.
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
        k.nu = k.nu(:)'; k.nudot = zeros(size(k.nu)); k.F = k.F(:)'; k.M = k.M(:)'; k.tau = k.tau(:)'; k.flink = k.flink(:)';
        C{end + 1} = k; %#ok<AGROW>
    end
end
nc = numel(C);
pose = cellfun(@(k) k.pose, C, 'UniformOutput', false);
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
[~, y0, ch0] = assemble(C(strcmp(pose, 'q0')), G, 'base');
yall = vertcat(y0{:}); chall = vertcat(ch0{:});
scale = zeros(27, 1);
for g = 1:5
    idx = ismember(chall, find(grp == g)); scale(grp == g) = max(sqrt(mean(yall(idx) .^ 2)), eps);
end
% jq_abs dropped: every scanned pose has q2, q3 <= 0, so |q| = -q and it equals jq exactly
variants = {'base', 'qlin_l23', 'qlin_l3', 'jq'};
R = struct('variants', {variants}, 'pose_tags', {pose_tags}, 'scale', scale);
R.rows = struct('variant', {}, 'n_params', {}, 'lambda_per_pose', {}, 'loo_pose', {}, 'loo_pose_all', {}, ...
                'joint_err', {}, 'joint_rms', {}, 'fit_all', {});
for v = 1:numel(variants)
    [H, y, ch] = assemble(C, G, variants{v});
    np = size(H{1}, 2);
    row = struct('variant', variants{v}, 'n_params', np, 'lambda_per_pose', []);
    [row.loo_pose, row.loo_pose_all, row.joint_err, row.joint_rms] = loo_pose(H, y, ch, pose, pose_tags, scale, grp, np, 0);
    th = solve(H, y, ch, 1:nc, scale, np, 0);
    e = zeros(nc, 5);
    for c = 1:nc, e(c, :) = errs(H{c} * th, y{c}, ch{c}, grp, scale); end
    row.fit_all = sqrt(mean(e .^ 2));
    R.rows(v) = orderfields(row, R.rows);
    report(row);
end
% ridge on the added columns of qlin_l23: lambda by nested leave-one-pose-out on the training poses
[H, y, ch] = assemble(C, G, 'qlin_l23'); np = size(H{1}, 2);
lams = [0 1e-3 1e-2 1e-1 1 10];
row = struct('variant', 'qlin_l23_ridge', 'n_params', np, 'lambda_per_pose', zeros(1, numel(pose_tags)));
ep = cell(numel(pose_tags), 1); je = zeros(numel(pose_tags), 3); jr = zeros(numel(pose_tags), 3);
for t = 1:numel(pose_tags)
    train_poses = setdiff(pose_tags, pose_tags(t), 'stable');
    keep = find(~strcmp(pose, pose_tags{t}));
    best = inf; lb = 0;
    for lam = lams
        sc = 0; n = 0;
        for u = 1:numel(train_poses)
            inner = keep(~strcmp(pose(keep), train_poses{u}));
            th = solve(H, y, ch, inner, scale, np, lam);
            for c = keep(strcmp(pose(keep), train_poses{u}))'
                sc = sc + errs(H{c} * th, y{c}, ch{c}, grp, scale) .^ 2 * [0 0 1 0 1]'; n = n + 1;
            end
        end
        if sc / n < best, best = sc / n; lb = lam; end
    end
    row.lambda_per_pose(t) = lb;
    th = solve(H, y, ch, keep, scale, np, lb);
    out = find(strcmp(pose, pose_tags{t}));
    ep{t} = zeros(numel(out), 5); tc = zeros(numel(out), 3); tp = tc;
    for ii = 1:numel(out)
        p = H{out(ii)} * th;
        ep{t}(ii, :) = errs(p, y{out(ii)}, ch{out(ii)}, grp, scale);
        tc(ii, :) = -y{out(ii)}(7:9)'; tp(ii, :) = -p(7:9)';
    end
    je(t, :) = sqrt(mean((tp - tc) .^ 2)); jr(t, :) = sqrt(mean(tc .^ 2));
end
row.loo_pose = cellfun(@(x) sqrt(mean(x .^ 2, 1)), ep, 'UniformOutput', false);
row.loo_pose_all = sqrt(mean(vertcat(ep{:}) .^ 2));
row.joint_err = je; row.joint_rms = jr; row.fit_all = nan(1, 5);
R.rows(end + 1) = orderfields(row, R.rows);
report(row);
fprintf('ridge lambda per left-out pose: %s\n', mat2str(row.lambda_per_pose));
fid = fopen(fullfile(here, 'scan_pose_terms.json'), 'w');
fprintf(fid, '%s', jsonencode(R, 'PrettyPrint', true)); fclose(fid);
end

function report(row)
fprintf('%-15s np %3d | LOO-pose all F %.3f M %.3f tau %.3f link %.3f linkM %.3f | J2/J3 err/rms:', ...
    row.variant, row.n_params, row.loo_pose_all);
for t = 1:size(row.joint_err, 1)
    fprintf('  %.2f/%.2f %.2f/%.2f', row.joint_err(t, 2), row.joint_rms(t, 2), row.joint_err(t, 3), row.joint_rms(t, 3));
end
fprintf('\n');
end

function [lp, lpa, je, jr] = loo_pose(H, y, ch, pose, tags, scale, grp, np, lam)
ep = cell(numel(tags), 1); je = zeros(numel(tags), 3); jr = zeros(numel(tags), 3);
for t = 1:numel(tags)
    out = find(strcmp(pose, tags{t}));
    th = solve(H, y, ch, find(~strcmp(pose, tags{t})), scale, np, lam);
    ep{t} = zeros(numel(out), 5); tc = zeros(numel(out), 3); tp = tc;
    for ii = 1:numel(out)
        p = H{out(ii)} * th;
        ep{t}(ii, :) = errs(p, y{out(ii)}, ch{out(ii)}, grp, scale);
        tc(ii, :) = -y{out(ii)}(7:9)'; tp(ii, :) = -p(7:9)';
    end
    je(t, :) = sqrt(mean((tp - tc) .^ 2)); jr(t, :) = sqrt(mean(tc .^ 2));
end
lp = cellfun(@(x) sqrt(mean(x .^ 2, 1)), ep, 'UniformOutput', false);
lpa = sqrt(mean(vertcat(ep{:}) .^ 2));
end

function [H, y, ch] = assemble(C, G, variant)
switch variant
    case 'base', nx = 0;
    case 'qlin_l23', nx = 48;
    case 'qlin_l3', nx = 24;
    case {'jq', 'jq_abs'}, nx = 18;
end
nc = numel(C); H = cell(nc, 1); y = cell(nc, 1); ch = cell(nc, 1);
for c = 1:nc
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    F = reshape(k.F, [], 3); M = reshape(k.M, [], 3); T = reshape(k.tau, [], 3);
    L = reshape(k.flink, [], 18); m = size(nu, 1); q = k.q(:);
    Hc = zeros(27 * m, 87 + nx); yc = zeros(27 * m, 1);
    for s = 1:m
        z = [nu(s, :)'; 0; 0; 0]; a = [nd(s, :)'; 0; 0; 0];
        [Yg, Yl] = Y_uvms(q, z, a, G);
        X = [Yg; Yl];
        switch variant
            case {'qlin_l23', 'qlin_l3'}
                links = 2:3; if strcmp(variant, 'qlin_l3'), links = 3; end
                add = [];
                for i = links
                    cols = [18 + 6 * (i - 1) + (4:6), 60 + 9 * (i - 1) + (1:9)];   % quad damping + CoP of link i
                    add = [add, X(:, cols) * q(2), X(:, cols) * q(3)]; %#ok<AGROW>
                end
                X = [X add]; %#ok<AGROW>
            case {'jq', 'jq_abs'}
                qq = q(2:3); if strcmp(variant, 'jq_abs'), qq = abs(qq); end
                u3 = z(1:3); qv = abs(u3) .* u3;
                add = zeros(27, 18); col = 0;
                for j = 1:3
                    for kk = 1:3
                        for mm = 1:2
                            col = col + 1; add(6 + j, col) = qq(mm) * qv(kk);
                        end
                    end
                end
                X = [X add]; %#ok<AGROW>
        end
        r = 27 * (s - 1);
        Hc(r + (1:27), :) = X;
        yc(r + (1:27)) = -[F(s, :)'; M(s, :)'; T(s, :)'; L(s, :)'];
    end
    H{c} = Hc; y{c} = yc; ch{c} = repmat((1:27)', m, 1);
end
end

function th = solve(H, y, ch, keep, scale, np, lam)
fixed = false(np, 1); fixed([16:18 37:42 52:60]) = true;
signed = false(np, 1); signed(43:np) = true;
Hw = []; yw = [];
for cc = keep(:)'
    w = 1 ./ (scale(ch{cc}) * sqrt(numel(y{cc}) / 27));
    Hw = [Hw; H{cc} .* w]; yw = [yw; y{cc} .* w]; %#ok<AGROW>
end
cn = sqrt(sum(Hw .^ 2, 1)); act = ~fixed' & cn > 1e-9 * max(cn);
A = Hw(:, act) ./ cn(act);
if lam > 0                                          % ridge on the pose-extension columns only
    ext = false(1, np); ext(88:np) = true; ext = ext(act);
    A = [A; sqrt(lam) * diag(double(ext))]; yw = [yw; zeros(nnz(act), 1)];
    A(end - nnz(act) + find(~ext), :) = [];
    yw(end - nnz(act) + find(~ext)) = [];
end
lb = zeros(np, 1); lb(signed) = -inf; lb = lb(act);
opt = optimoptions('lsqlin', 'Display', 'off', 'Algorithm', 'active-set');
zz = lsqlin(A, yw, [], [], [], [], lb, [], max(lb, 0), opt);
th = zeros(np, 1); th(act) = zz ./ cn(act)';
end

function e = errs(p, y, ch, grp, scale)
e = zeros(1, 5);
for g = 1:5
    idx = ismember(ch, find(grp == g));
    e(g) = sqrt(mean((p(idx) - y(idx)) .^ 2)) / scale(find(grp == g, 1));
end
end

function R = scan_hull_terms(pose_tags, variants)
%SCAN_HULL_TERMS  Alternative hull damping forms for theta, judged by leave-one-case-out and
%   leave-one-POSE-out on q = 0 (22 cases) + the steady pose sets (default qb8, qc, qd).
%   The steady q = 0 loads are purely quadratic in speed (0.8/0.3 load ratio 7.0-7.1 for every
%   channel, two-point fit gives Dl ~ 1 N s/m vs Dq ~ 335), yet the current theta puts Dl_u = 126 and
%   uses a LINEAR translation->moment coupling: it buys the oblique cases with linear terms and
%   overpredicts the 0.3 m/s cases by up to 1.75x. Variants of the hull translational columns:
%     A  current: Dl_uvw diag, Dq_uvw diag |u|u, C_lin (moment += C v)            (Y_uvms 7:9,13:15,43:51)
%     B  full quadratic:  [F; M] += K (6x3) * |v| v,  |v| = translational speed    (18 new, drops A's)
%     C  B + Dl_uvw diag (linear translational damping kept)                       (18 + 3)
%     D  B + K2 (6x3) * |v|_1-weighted:  [F; M] += K2 * (|v| .* v) diag form      (B + the diag |u|u form)
%   Rotational damping Dl_pqr (10:12), added masses, link terms (19:36, 61:87) unchanged in all.
%   Same constraints / scale as fit_theta (sign-free for K, K2, C; >= 0 for diagonal damping).
%   Writes scan_hull_terms.json.
if nargin < 1 || isempty(pose_tags), pose_tags = {'qb8', 'qc', 'qd'}; end
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
ids = cellfun(@(k) k.id, C, 'UniformOutput', false);
q0 = strcmp(pose, 'q0');
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
[~, y0, ch0] = assemble(C(q0), G, 'A');
yall = vertcat(y0{:}); chall = vertcat(ch0{:});
scale = zeros(27, 1);
for g = 1:5
    idx = ismember(chall, find(grp == g)); scale(grp == g) = max(sqrt(mean(yall(idx) .^ 2)), eps);
end
%     BL  B + every link's diagonal quadratic damping (19:36 drag part) replaced by a full 3x3 matrix
%         times |v_i| v_i (link-frame velocity of the link reference point), force at the reference point
%     BL3 B + the same for link 3 only
%     BLu B + full 3x3 matrix times |v_ik| v_ik (componentwise) for every link
if nargin < 2, variants = {'A', 'B', 'C', 'D', 'BL', 'BL3', 'BLu'}; end
R = struct('variants', {variants}, 'pose_tags', {pose_tags}, 'scale', scale, 'case_ids', {ids});
for v = 1:numel(variants)
    [H, y, ch, fixed, signed] = assemble(C, G, variants{v});
    np = size(H{1}, 2);
    th = solve(H, y, ch, 1:nc, scale, fixed, signed);
    ef = zeros(nc, 5);
    for c = 1:nc, ef(c, :) = errs(H{c} * th, y{c}, ch{c}, grp, scale); end
    el = zeros(nc, 5);
    for c = 1:nc
        thc = solve(H, y, ch, setdiff(1:nc, c), scale, fixed, signed);
        el(c, :) = errs(H{c} * thc, y{c}, ch{c}, grp, scale);
    end
    ep = cell(numel(pose_tags), 1);
    for t = 1:numel(pose_tags)
        out = find(strcmp(pose, pose_tags{t}));
        tht = solve(H, y, ch, find(~strcmp(pose, pose_tags{t})), scale, fixed, signed);
        ep{t} = zeros(numel(out), 5);
        for ii = 1:numel(out), ep{t}(ii, :) = errs(H{out(ii)} * tht, y{out(ii)}, ch{out(ii)}, grp, scale); end
    end
    row = struct('variant', variants{v}, 'n_free', nnz(~fixed), ...
        'fit_q0', rms(ef(q0, :)), 'loo_case_q0', rms(el(q0, :)), 'loo_case_steady_q0', rms(el(q0 & startsWith(ids, 'T_'), :)), ...
        'loo_case_xpose', rms(el(~q0, :)), 'loo_pose', rms(vertcat(ep{:})), 'loo_case_rows', el, 'theta', th);
    R.rows(v) = row;
    fprintf(['%s (%3d free) fit q0 %s | LOOcase q0 %s | LOOcase steady q0 %s | LOOcase xpose %s | ' ...
             'LOOpose %s\n'], variants{v}, row.n_free, f5(row.fit_q0), f5(row.loo_case_q0), ...
             f5(row.loo_case_steady_q0), f5(row.loo_case_xpose), f5(row.loo_pose));
end
fprintf('columns: F M tau link linkM (nrms, q = 0 data-set scale)\n');
fid = fopen(fullfile(here, 'scan_hull_terms.json'), 'w');
fprintf(fid, '%s', jsonencode(R, 'PrettyPrint', true)); fclose(fid);
end

function s = f5(x), s = sprintf('%.3f ', x); end
function r = rms(e), r = sqrt(mean(e .^ 2, 1)); end

function [H, y, ch, fixed, signed] = assemble(C, G, variant)
links_full = [];
switch variant
    case 'A', nx = 0;
    case {'B', 'C'}, nx = 18;
    case 'D', nx = 36;
    case {'BL', 'BLu'}, nx = 18 + 27; links_full = 1:3;
    case 'BL3', nx = 18 + 9; links_full = 3;
end
np = 87 + nx;
fixed = false(np, 1); fixed([16:18 37:42 52:60]) = true;
signed = false(np, 1); signed(43:np) = true;
for i = links_full
    fixed(18 + 6 * (i - 1) + (4:6)) = true;      % diagonal link drag replaced by the full matrix
end
if any(strcmp(variant, {'B', 'C', 'D', 'BL', 'BL3', 'BLu'}))
    fixed(43:51) = true;                     % linear coupling replaced by the 6x3 quadratic matrix
    fixed(13:15) = true;                     % diagonal |u|u: contained in K (B, C) or in K2 (D)
    if ~strcmp(variant, 'C'), fixed(7:9) = true; end    % linear translational damping dropped (B, D)
end
nc = numel(C); H = cell(nc, 1); y = cell(nc, 1); ch = cell(nc, 1);
for c = 1:nc
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    F = reshape(k.F, [], 3); M = reshape(k.M, [], 3); T = reshape(k.tau, [], 3);
    L = reshape(k.flink, [], 18); m = size(nu, 1); q = k.q(:);
    Hc = zeros(27 * m, np); yc = zeros(27 * m, 1);
    if ~isempty(links_full), Jl = uvms_link_kin(q, zeros(3, 1), G); end
    for s = 1:m
        z = [nu(s, :)'; 0; 0; 0]; a = [nd(s, :)'; 0; 0; 0];
        [Yg, Yl] = Y_uvms(q, z, a, G);
        X = [Yg; Yl];
        if nx > 0
            v = z(1:3); sp = norm(v);
            add = zeros(27, nx);
            for r = 1:6                       % K(r, kk): row r of [F; M], column kk
                for kk = 1:3
                    add(r, 3 * (r - 1) + kk) = sp * v(kk);
                    if strcmp(variant, 'D'), add(r, 18 + 3 * (r - 1) + kk) = abs(v(kk)) * v(kk); end
                end
            end
            col = 18;
            for i = links_full
                vi = Jl{i} * z; vi = vi(1:3); rl = 9 + 6 * (i - 1) + (1:6);
                if strcmp(variant, 'BLu'), g = abs(vi) .* vi; else, g = norm(vi) * vi; end
                for r = 1:3                   % link force row r (link frame), column kk
                    for kk = 1:3
                        col = col + 1;
                        wr = zeros(6, 1); wr(r) = g(kk);
                        add(1:9, col) = Jl{i}' * wr;      % generalised force (rows 1-9)
                        add(rl, col) = wr;                 % link wrench rows
                    end
                end
            end
            X = [X add]; %#ok<AGROW>
        end
        rr = 27 * (s - 1);
        Hc(rr + (1:27), :) = X;
        yc(rr + (1:27)) = -[F(s, :)'; M(s, :)'; T(s, :)'; L(s, :)'];
    end
    H{c} = Hc; y{c} = yc; ch{c} = repmat((1:27)', m, 1);
end
end

function th = solve(H, y, ch, keep, scale, fixed, signed)
np = numel(fixed);
Hw = []; yw = [];
for cc = keep(:)'
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

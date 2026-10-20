function scan_link_terms()
%SCAN_LINK_TERMS  Leave-one-case-out comparison of candidate link-term forms on the same data.
%   Each variant swaps the 27 entries 61:87 of Y_uvms for a different regressor (built here from the
%   link velocities at q = 0), refits with fit_core and reports RMS-over-cases LOO errors. Only for
%   choosing the form; the chosen one is then written into Y_uvms.
here = fileparts(mfilename('fullpath'));
S = jsondecode(fileread(fullfile(here, '..', 'theta_dataset.json')));
C = S.cases; if isstruct(C), C = num2cell(C); end
G = uvms_geometry();
variants = {'none', 'cop_moment', 'cross_force', 'cross_force_l3', 'joint_quad'};
for v = 1:numel(variants)
    [H, y, ch] = assemble(C, G, variants{v});
    r = loo(H, y, ch, variants{v});
    fprintf('%-15s LOO nrms  F %.3f  M %.3f  tau %.3f  link %.3f  linkM %.3f\n', variants{v}, r);
end
end

function [H, y, ch] = assemble(C, G, variant)
nc = numel(C); H = cell(nc, 1); y = cell(nc, 1); ch = cell(nc, 1);
for c = 1:nc
    k = C{c};
    nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
    L = reshape(k.flink, [], 18); m = size(nu, 1);
    Hc = []; yc = [];
    for s = 1:m
        z = [nu(s, :)'; 0; 0; 0]; a = [nd(s, :)'; 0; 0; 0];
        [Yg, Yl] = Y_uvms(zeros(3, 1), z, a, G);
        Yg(:, 61:87) = 0; Yl(:, 61:87) = 0;
        [J, ~] = uvms_link_kin(zeros(3, 1), zeros(3, 1), G);
        for i = 1:3
            vi = J{i} * z; vi = vi(1:3); qv = abs(vi) .* vi; rl = 6 * (i - 1) + (1:6);
            for kk = 1:3
                for j = 1:3
                    col = 60 + 9 * (i - 1) + 3 * (kk - 1) + j;
                    wr = zeros(6, 1);
                    switch variant
                        case 'cop_moment', wr(3 + j) = qv(kk);
                        case 'cross_force', if j ~= kk, wr(j) = qv(kk); end
                        case 'cross_force_l3', if i == 3 && j ~= kk, wr(j) = qv(kk); end
                    end
                    Yg(:, col) = J{i}' * wr; Yl(rl, col) = wr;
                end
            end
        end
        if strcmp(variant, 'joint_quad')     % tau_j += sum_k Q(j,k) |u_k| u_k, body velocity, 9 terms
            v3 = z(1:3); qv = abs(v3) .* v3;
            for kk = 1:3
                for j = 1:3
                    Yg(6 + j, 60 + 3 * (kk - 1) + j) = qv(kk);
                end
            end
        end
        Hc = [Hc; Yg; Yl]; yc = [yc; -[k.F(s, :)'; k.M(s, :)'; k.tau(s, :)'; L(s, :)']]; %#ok<AGROW>
    end
    H{c} = Hc; y{c} = yc; ch{c} = repmat((1:27)', m, 1);
end
end

function r = loo(H, y, ch, variant)
np = 87; nc = numel(H);
fixed = false(np, 1); fixed([16:18 37:42 52:60]) = true;
signed = false(np, 1); signed(43:87) = true;
if strcmp(variant, 'none'), fixed(61:87) = true; end
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
yall = vertcat(y{:}); chall = vertcat(ch{:});
scale = zeros(27, 1);
for g = 1:5
    idx = ismember(chall, find(grp == g)); scale(grp == g) = max(sqrt(mean(yall(idx) .^ 2)), eps);
end
e = zeros(nc, 5);
for c = 1:nc
    keep = setdiff(1:nc, c);
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
    p = H{c} * th;
    for g = 1:5
        idx = ismember(ch{c}, find(grp == g));
        e(c, g) = sqrt(mean((p(idx) - y{c}(idx)) .^ 2)) / scale(find(grp == g, 1));
    end
end
r = sqrt(mean(e .^ 2));
end

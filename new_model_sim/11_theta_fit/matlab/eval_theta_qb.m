function T = eval_theta_qb()
%EVAL_THETA_QB  Pose generalisation check: theta fitted at q = 0 (theta_fit.mat, unchanged) predicts the
%   steady loads at the pose qb = [0.45 -0.50 -0.40] (theta_dataset_qb.json, runs_qb/).
%   Channels/grouping/metrics exactly as fit_theta: rel = ||pred - CFD|| / ||CFD|| per case and group;
%   nrms = RMS(pred - CFD) / channel_scale (the q = 0 data-set RMS of the group, so qb and q = 0 nrms are
%   on the same scale). Also lists the q = 0 fit and leave-one-out errors of the same case ids, and a
%   "pose-blind" prediction Y(q = 0) for reference (what a model ignoring q would give).
%   Writes eval_theta_qb.csv and eval_theta_qb_channels.csv.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'theta_fit.mat'));
Dq = jsondecode(fileread(fullfile(here, '..', 'theta_dataset_qb.json')));
G = uvms_geometry();
q = Dq.q(:);
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
gn = {'F', 'M', 'tau', 'link', 'linkM'};
sc = S.channel_scale(:);
cs = Dq.cases; if ~iscell(cs), cs = num2cell(cs); end
rows = {}; chrows = {};
for c = 1:numel(cs)
    k = cs{c};
    y = [k.F(:); k.M(:); k.tau(:); k.flink(:)];             % CFD, the actuator must supply -y
    preds = struct();
    for mode = {'qb', 'q0blind'}
        qq = q; if strcmp(mode{1}, 'q0blind'), qq = zeros(3, 1); end
        [Yg, Yl] = Y_uvms(qq, [k.nu(:); 0; 0; 0], zeros(9, 1), G);
        preds.(mode{1}) = -[Yg; Yl] * S.theta;                % predicted fluid load
    end
    j0 = find(strcmp(S.case_ids, k.id));
    for g = 1:5
        idx = grp == g;
        e = preds.qb(idx) - y(idx); eb = preds.q0blind(idx) - y(idx);
        rows(end + 1, :) = {k.id, gn{g}, norm(e) / max(norm(y(idx)), eps), sqrt(mean(e .^ 2)) / sc(find(idx, 1)), ...
            norm(eb) / max(norm(y(idx)), eps), S.fit_rel_err(j0, g), S.fit_rel_err(j0, 5 + g), ...
            S.loo_rel_err(j0, g), S.loo_rel_err(j0, 5 + g)}; %#ok<AGROW>
    end
    for r = 1:27
        chrows(end + 1, :) = {k.id, r, gn{grp(r)}, y(r), preds.qb(r), preds.q0blind(r)}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'case', 'group', 'qb_rel', 'qb_nrms', 'q0blind_rel', ...
    'q0fit_rel', 'q0fit_nrms', 'q0loo_rel', 'q0loo_nrms'});
writetable(T, fullfile(here, 'eval_theta_qb.csv'));
writetable(cell2table(chrows, 'VariableNames', {'case', 'channel', 'group', 'cfd', 'pred_qb', 'pred_q0blind'}), ...
    fullfile(here, 'eval_theta_qb_channels.csv'));
% group summary over the three cases (RMS over cases of nrms)
fprintf('%-6s %8s %8s %8s %8s\n', 'group', 'qb', 'q0fit', 'q0loo', 'blind_rel');
for g = 1:5
    m = strcmp(T.group, gn{g});
    fprintf('%-6s %8.3f %8.3f %8.3f %8.3f\n', gn{g}, sqrt(mean(T.qb_nrms(m) .^ 2)), ...
        sqrt(mean(T.q0fit_nrms(m) .^ 2)), sqrt(mean(T.q0loo_nrms(m) .^ 2)), mean(T.q0blind_rel(m)));
end
end

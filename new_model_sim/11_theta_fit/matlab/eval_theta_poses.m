function T = eval_theta_poses(theta_file, pose_tags)
%EVAL_THETA_POSES  Predict the steady loads at extra arm poses with a fitted theta (no refit).
%   theta_file default theta_fit.mat (fitted at q = 0 only); pose_tags default {'qb8','qc','qd'}
%   (theta_dataset_<tag>.json). Per pose: nrms per group on the q = 0 channel scale of that theta file
%   (same scale as fit_theta / scan_link_terms_pose), and per-joint RMS torque error vs the RMS CFD torque.
%   Prints a table and returns it; writes nothing.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(theta_file), theta_file = fullfile(here, 'theta_fit.mat'); end
if nargin < 2, pose_tags = {'qb8', 'qc', 'qd'}; end
S = load(theta_file);
G = uvms_geometry();
grp = [1 1 1 2 2 2 3 3 3 repmat([4 4 4 5 5 5], 1, 3)];
sc = S.channel_scale(:);
rows = {};
for t = 1:numel(pose_tags)
    P = jsondecode(fileread(fullfile(here, '..', ['theta_dataset_' pose_tags{t} '.json'])));
    pc = P.cases; if isstruct(pc), pc = num2cell(pc); end
    e = zeros(numel(pc), 5); tc = zeros(numel(pc), 3); tp = zeros(numel(pc), 3);
    for c = 1:numel(pc)
        k = pc{c};
        y = [k.F(:); k.M(:); k.tau(:); k.flink(:)];
        [Yg, Yl] = Y_uvms(P.q(:), [k.nu(:); 0; 0; 0], zeros(9, 1), G);
        p = -[Yg; Yl] * S.theta;
        for g = 1:5
            idx = grp == g;
            e(c, g) = sqrt(mean((p(idx) - y(idx)) .^ 2)) / sc(find(idx, 1));
        end
        tc(c, :) = y(7:9)'; tp(c, :) = p(7:9)';
    end
    n = sqrt(mean(e .^ 2));
    je = sqrt(mean((tp - tc) .^ 2)); jc = sqrt(mean(tc .^ 2));
    rows(end + 1, :) = {pose_tags{t}, n(1), n(2), n(3), n(4), n(5), je(1), jc(1), je(2), jc(2), je(3), jc(3)}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'pose', 'F', 'M', 'tau', 'link', 'linkM', ...
    'J1_err', 'J1_rms', 'J2_err', 'J2_rms', 'J3_err', 'J3_rms'});
disp(T);
end

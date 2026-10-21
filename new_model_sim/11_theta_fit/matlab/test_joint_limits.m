function test_joint_limits()
%TEST_JOINT_LIMITS  Plot the new-CAD joint scan (joint_limits.png) and check the simulation guard:
%   default call unchanged (0.5 s run vs sim_uvms_theta_regression_default_05s.mat, max diff 0), the
%   default target warns, joint_check='error' rejects it, qb passes silently.
here = fileparts(mfilename('fullpath'));
J = jsondecode(fileread(fullfile(here, '..', '..', '13_pose_q1', 'joint_limits.json')));
fn = fieldnames(J); fn = fn(startsWith(fn, 'grid_j1'));
lv = cellfun(@(f) J.(f).q1, fn); [lv, o] = sort(lv); fn = fn(o);
fig = figure('Visible', 'off', 'Position', [50 50 1500 330]);
for i = 1:numel(fn)
    g = J.(fn{i});
    subplot(1, numel(fn), i);
    imagesc(g.q3_values, g.q2_values, double(g.free)); axis xy equal tight;
    colormap([0.85 0.3 0.3; 0.3 0.7 0.4]); caxis([0 1]);
    hold on; plot(-0.40, -0.50, 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 11);
    plot(0.40, -0.50, 'kx', 'MarkerSize', 11, 'LineWidth', 2);
    xlabel('q_3 [rad]'); if i == 1, ylabel('q_2 [rad]'); end
    title(sprintf('q_1 = %+.3f', lv(i)));
end
sgtitle('New-CAD collision scan: green free, red overlap > 0.5 mm^3;  star qb, cross old target');
exportgraphics(fig, fullfile(here, 'joint_limits.png'), 'Resolution', 130); close(fig);

assert(uvms_joint_check([0.45 -0.50 -0.40]), 'qb must be free');
assert(~uvms_joint_check([0.45 -0.50 0.40]), 'old target must collide');
assert(~uvms_joint_check([0 0.1 0]) && ~uvms_joint_check([0 0 0.1]), 'J2/J3 > 0 must collide');
assert(uvms_joint_check([0 0 0]), 'q = 0 must be free');

ref = load(fullfile(here, 'sim_uvms_theta_regression_default_05s.mat'));
tmp = fullfile(here, 'sim_uvms_theta_regression_jlim.mat');
if isfile(tmp), delete(tmp); end
lastwarn('');
simulate_uvms_theta('theta', 'gm', [], '_regression_jlim', 0.5);
[~, wid] = lastwarn;
% 0.5 s stays in the 1 s hold at q = 0, so the desired path is free and no warning is expected
assert(~strcmp(wid, 'simulate_uvms_theta:JointCollision'), 'hold phase at q = 0 should not warn');
new = load(tmp); delete(tmp);
fr = fieldnames(ref);
d = 0; nc = 0;
for k = 1:numel(fr)
    a = ref.(fr{k});
    if ~isnumeric(a), continue; end
    assert(isfield(new, fr{k}) && isequal(size(a), size(new.(fr{k}))), 'field %s missing or resized', fr{k});
    d = max(d, max(abs(a(:) - new.(fr{k})(:)))); nc = nc + 1;
end
assert(nc >= 8, 'too few numeric fields compared (%d)', nc);
fprintf('default 0.5 s regression max |diff| = %g\n', d);
assert(d == 0, 'default run changed');
lastwarn('');
simulate_uvms_theta('theta', 'gm', [], '_regression_jlim_warn', 1.6);
[~, wid] = lastwarn;
assert(strcmp(wid, 'simulate_uvms_theta:JointCollision'), 'default target should warn once the arm moves');
delete(fullfile(here, 'sim_uvms_theta_regression_jlim_warn.mat'));
try
    simulate_uvms_theta('theta', 'gm', [], '_regression_jlim_err', 1.6, 'error');
    error('test:noerror', 'joint_check=error did not reject the default target');
catch e
    assert(strcmp(e.identifier, 'simulate_uvms_theta:JointCollision'), e.message);
end
lastwarn('');
simulate_uvms_theta('theta', 'gm', [0.45 -0.50 -0.40], '_regression_jlim_qb', 1.6, 'error');
[~, wid] = lastwarn;
assert(~strcmp(wid, 'simulate_uvms_theta:JointCollision'), 'qb should not warn');
delete(fullfile(here, 'sim_uvms_theta_regression_jlim_qb.mat'));
fprintf('test_joint_limits: all passed\n');
end

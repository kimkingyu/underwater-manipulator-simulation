function plot_sim_compare(output_suffix)
%PLOT_SIM_COMPARE  Hand-set vs CFD-theta hydrodynamics in the 9-DOF UVMS simulation (old URDF).
%   plot_sim_compare('_qb') loads the _qb MAT files and writes a separate _qb PNG.
if nargin < 1, output_suffix = ''; end
output_suffix = char(output_suffix);
assert(isempty(output_suffix) || ~isempty(regexp(output_suffix, '^_[A-Za-z0-9_-]+$', 'once')), ...
    'plot_sim_compare:InvalidSuffix', 'Use a suffix such as _qb.');
here = fileparts(mfilename('fullpath'));
output_path = fullfile(here, ['sim_compare_handset_vs_theta' output_suffix '.png']);
assert(isempty(output_suffix) || ~isfile(output_path), 'plot_sim_compare:OutputExists', ...
    'Refusing to overwrite %s; choose a different suffix.', output_path);
a = load(fullfile(here, ['sim_uvms_handset' output_suffix '.mat']));
b = load(fullfile(here, ['sim_uvms_theta' output_suffix '.mat']));
assert(isequal(a.time, b.time) && isequal(a.qArm_des, b.qArm_des), ...
    'plot_sim_compare:IncompatibleRuns', 'The time vectors and desired arm trajectories must match.');
t = a.time;
f = figure('Color', 'w', 'Position', [80 80 1400 900], 'Visible', 'off');
tl = tiledlayout(3, 3, 'TileSpacing', 'compact');
if isempty(output_suffix)
    title(tl, '9-DOF UVMS (old URDF, neutral buoyancy GM = 0.03 m): hand-set hydro (dashed) vs CFD theta (solid)');
else
    title(tl, sprintf('9-DOF UVMS (old URDF, GM = 0.03 m), target [%g %g %g] rad: hand-set (dashed) vs CFD theta (solid)', b.qArm_des(end, :)));
end
lab = {'X', 'Y', 'Z'};
for i = 1:3
    nexttile; plot(t, 1000 * a.q(:, i), '--', t, 1000 * b.q(:, i), '-', 'LineWidth', 1.2); grid on;
    title(['AUV ' lab{i} ' / mm']); if i == 1, legend('hand-set', 'theta', 'Location', 'best'); end
end
alab = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    nexttile; plot(t, rad2deg(a.q(:, 3 + i)), '--', t, rad2deg(b.q(:, 3 + i)), '-', 'LineWidth', 1.2); grid on;
    title(['AUV ' alab{i} ' / deg']);
end
for j = 1:3
    nexttile;
    plot(t, a.tau_hydro(:, 6 + j), '--', t, b.tau_hydro(:, 6 + j), '-', t, b.tau_arm(:, j), ':', 'LineWidth', 1.2);
    grid on; title(sprintf('J%d: hydro torque (hand-set / theta), drive torque (theta) / N*m', j));
    if j == 1, legend('hydro hand-set', 'hydro theta', 'drive theta', 'Location', 'best'); end
    xlabel('t / s');
end
exportgraphics(f, output_path, 'Resolution', 150);
fprintf('saved %s\n', output_path);
end

function plot_sim_compare()
%PLOT_SIM_COMPARE  Hand-set vs CFD-theta hydrodynamics in the 9-DOF UVMS simulation (old URDF).
here = fileparts(mfilename('fullpath'));
a = load(fullfile(here, 'sim_uvms_handset.mat'));
b = load(fullfile(here, 'sim_uvms_theta.mat'));
t = a.time;
f = figure('Color', 'w', 'Position', [80 80 1400 900], 'Visible', 'off');
tl = tiledlayout(3, 3, 'TileSpacing', 'compact');
title(tl, '9-DOF UVMS (old URDF): hand-set hydro (dashed) vs CFD theta (solid)');
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
exportgraphics(f, fullfile(here, 'sim_compare_handset_vs_theta.png'), 'Resolution', 150);
fprintf('saved %s\n', fullfile(here, 'sim_compare_handset_vs_theta.png'));
end

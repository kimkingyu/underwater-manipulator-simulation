function M = plot_rigid_compare()
%PLOT_RIGID_COMPARE  Old-URDF vs new-CAD rigid body at pose qb, hand-set and theta hydrodynamics.
%   Loads sim_uvms_{handset,theta}_qb.mat (old URDF, 'gm' buoyancy) and sim_uvms_{handset,theta}_cad_qb.mat
%   (new CAD, per-body buoyancy), writes sim_compare_rigid_urdf_vs_cad_qb.png (refuses to overwrite) and
%   returns/prints the summary metrics (final and max joint error, peak/final drive torque, max |roll|,
%   |pitch|, |yaw|, max displacement).
here = fileparts(mfilename('fullpath'));
png = fullfile(here, 'sim_compare_rigid_urdf_vs_cad_qb.png');
assert(~isfile(png), 'plot_rigid_compare:OutputExists', 'Refusing to overwrite %s', png);
tags = {'handset_qb', 'theta_qb', 'handset_cad_qb', 'theta_cad_qb'};
lab = {'hand-set, URDF', 'theta, URDF', 'hand-set, CAD', 'theta, CAD'};
sty = {'--', '-', '--', '-'};
col = {[0.55 0.55 0.55], [0.3 0.3 0.3], [0.85 0.33 0.1], [0 0.45 0.74]};
S = cell(1, 4);
for k = 1:4, S{k} = load(fullfile(here, ['sim_uvms_' tags{k} '.mat'])); end
t = S{1}.time;
for k = 2:4, assert(isequal(S{k}.time, t) && isequal(S{k}.qArm_des, S{1}.qArm_des), 'runs differ in time/target'); end
f = figure('Color', 'w', 'Position', [60 60 1500 950], 'Visible', 'off');
tl = tiledlayout(3, 3, 'TileSpacing', 'compact');
title(tl, sprintf('Pose qb [%g %g %g] rad: old URDF (grey) vs new CAD rigid body (colour); dashed hand-set, solid theta', ...
    S{1}.qArm_des(end, :)));
ttl = {'AUV X / mm', 'AUV Y / mm', 'AUV Z / mm', 'Roll / deg', 'Pitch / deg', 'Yaw / deg', ...
       'J1 error / mrad', 'J2 error / mrad', 'J3 error / mrad'};
for p = 1:9
    nexttile; hold on; grid on;
    for k = 1:4
        if p <= 3, y = 1000 * S{k}.q(:, p);
        elseif p <= 6, y = rad2deg(S{k}.q(:, p));
        else, j = p - 6; y = 1000 * (S{k}.qArm_des(:, j) - S{k}.q(:, 6 + j));
        end
        plot(t, y, sty{k}, 'Color', col{k}, 'LineWidth', 1.2);
    end
    title(ttl{p}); xlabel('t / s');
    if p == 1, legend(lab, 'Location', 'best'); end
end
exportgraphics(f, png, 'Resolution', 130); close(f);
M = struct();
fprintf('%-16s %-26s %-26s %-26s %-26s %8s %8s %8s %8s\n', 'run', 'err end [mrad]', 'err max [mrad]', ...
    'tau peak [N m]', 'tau end [N m]', 'roll', 'pitch', 'yaw', 'disp mm');
for k = 1:4
    s = S{k};
    e = s.qArm_des - s.q(:, 7:9);
    r.err_end = e(end, :); r.err_max = max(abs(e));
    r.tau_peak = max(abs(s.tau_arm)); r.tau_end = s.tau_arm(end, :);
    r.att_max_deg = rad2deg(max(abs(s.q(:, 4:6))));
    r.disp_max_mm = 1000 * max(vecnorm(s.q(:, 1:3), 2, 2));
    r.saturated = any(abs(s.tau_arm(:)) >= 15 - 1e-9);
    r.finite = all(isfinite(s.q(:)));
    M.(tags{k}) = r;
    fprintf('%-16s %-26s %-26s %-26s %-26s %8.3f %8.3f %8.4f %8.3f\n', tags{k}, mat2str(1000 * r.err_end, 3), ...
        mat2str(1000 * r.err_max, 3), mat2str(r.tau_peak, 3), mat2str(r.tau_end, 3), r.att_max_deg, r.disp_max_mm);
end
end

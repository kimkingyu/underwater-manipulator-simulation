function plot_arm_ff()
%PLOT_ARM_FF  Joint tracking error at pose qb, new-CAD rigid body: PD only vs + static vs + full
%   feedforward (test_arm_ff runs). Top row: plant with theta hydrodynamics (feedforward model = plant).
%   Bottom row: plant with hand-set hydrodynamics (feedforward still uses theta -> model mismatch).
%   Writes sim_arm_ff_qb.png (refuses to overwrite).
here = fileparts(mfilename('fullpath'));
png = fullfile(here, 'sim_arm_ff_qb.png');
assert(~isfile(png), 'plot_arm_ff:OutputExists', 'Refusing to overwrite %s', png);
sfx = {'_qb', '_qb_ffs', '_qb_fff'}; lab = {'PD only', '+ gravity/buoyancy FF', '+ full FF (rigid + theta hydro)'};
col = {[0.5 0.5 0.5], [0.85 0.33 0.1], [0 0.45 0.74]};
f = figure('Color', 'w', 'Position', [60 60 1500 700], 'Visible', 'off');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact');
title(tl, 'Joint tracking error, pose qb [0.45 -0.5 -0.4] rad, new-CAD rigid body (log scale, |e| in mrad)');
plant = {'theta', 'handset'}; pl = {'plant: theta hydro (= FF model)', 'plant: hand-set hydro (FF model: theta)'};
for r = 1:2
    for j = 1:3
        nexttile; hold on; grid on;
        for k = 1:3
            s = load(fullfile(here, ['sim_uvms_' plant{r} '_cad' sfx{k} '.mat']));
            semilogy(s.time, max(abs(1000 * (s.qArm_des(:, j) - s.q(:, 6 + j))), 1e-4), 'Color', col{k}, 'LineWidth', 1.2);
        end
        set(gca, 'YScale', 'log'); ylim([1e-3 30]); xlabel('t / s');
        title(sprintf('J%d, %s', j, pl{r}));
        if r == 1 && j == 1, legend(lab, 'Location', 'southwest'); end
    end
end
exportgraphics(f, png, 'Resolution', 130); close(f);
end

function R = test_arm_ff()
%TEST_ARM_FF  Joint feedforward in simulate_uvms_theta (arm_ff), new-CAD rigid body, pose qb, 8 s.
%   1. regression: default arm_ff ('none') reproduces sim_uvms_{handset,theta}_cad_qb.mat (first 1.6 s
%      re-run, all numeric fields). Tolerance 1e-12 relative, not 0: the per-body buoyancy was moved
%      into gravity_buoyancy(), which changes the floating-point summation order (5e-15 rad on 0.5 rad;
%      the code is deterministic -- two runs of the current code agree bit for bit). The default 'gm'
%      mode is untouched and still checked to 0 by test_joint_limits.
%   2. runs arm_ff = 'static' and 'full' for both hydro models (suffixes _cad_qb_ffs / _cad_qb_fff)
%   3. prints final / max / RMS joint error (after t = 1 s) and drive torque for none / static / full
%   The 'handset' runs test robustness: the controller's feedforward uses theta, the plant hand-set hydro.
here = fileparts(mfilename('fullpath'));
qb = [0.45 -0.50 -0.40];
for m = {'handset', 'theta'}
    ref = load(fullfile(here, ['sim_uvms_' m{1} '_cad_qb.mat']));
    tmp = '_qb_regr_ff';                               % 'cad' adds _cad itself
    f = fullfile(here, ['sim_uvms_' m{1} '_cad' tmp '.mat']);
    if isfile(f), delete(f); end
    simulate_uvms_theta(m{1}, 'cad', qb, tmp, 1.6, 'error');
    new = load(f); delete(f);
    n = numel(new.time); d = 0; sc = 0;
    for fn = fieldnames(new)'
        a = new.(fn{1});
        if isnumeric(a) && size(a, 1) == n
            b = ref.(fn{1})(1:n, :); d = max(d, max(abs(a(:) - b(:)))); sc = max(sc, max(abs(b(:))));
        end
    end
    fprintf('regression %s cad qb (1.6 s): max |diff| = %g (relative %g)\n', m{1}, d, d / sc);
    assert(d <= 1e-12 * sc, 'default arm_ff changed the %s run', m{1});
end
modes = {'none', 'static', 'full'}; sfx = {'_qb', '_qb_ffs', '_qb_fff'};
R = struct();
fprintf('%-8s %-7s %-24s %-24s %-24s %-22s %-22s\n', 'model', 'ff', 'err end [mrad]', 'err max [mrad]', ...
    'err rms t>1 [mrad]', 'tau peak [N m]', 'tau end [N m]');
for m = {'handset', 'theta'}
    for k = 1:3
        f = fullfile(here, ['sim_uvms_' m{1} '_cad' sfx{k} '.mat']);
        if ~isfile(f)
            simulate_uvms_theta(m{1}, 'cad', qb, sfx{k}, 8, 'error', modes{k});
        end
        s = load(f);
        e = 1000 * (s.qArm_des - s.q(:, 7:9)); w = s.time >= 1;
        r = struct('err_end', e(end, :), 'err_max', max(abs(e)), 'err_rms', sqrt(mean(e(w, :) .^ 2)), ...
                   'tau_peak', max(abs(s.tau_arm)), 'tau_end', s.tau_arm(end, :), ...
                   'saturated', any(abs(s.tau_arm(:)) >= 15 - 1e-9), 'finite', all(isfinite(s.q(:))), ...
                   'att_max_deg', rad2deg(max(abs(s.q(:, 4:6)))), 'disp_max_mm', 1000 * max(vecnorm(s.q(:, 1:3), 2, 2)));
        R.([m{1} '_' modes{k}]) = r;
        fprintf('%-8s %-7s %-24s %-24s %-24s %-22s %-22s sat %d\n', m{1}, modes{k}, mat2str(r.err_end, 3), ...
            mat2str(r.err_max, 3), mat2str(r.err_rms, 3), mat2str(r.tau_peak, 3), mat2str(r.tau_end, 3), r.saturated);
    end
end
fid = fopen(fullfile(here, 'arm_ff_metrics.json'), 'w'); fprintf(fid, '%s', jsonencode(R, 'PrettyPrint', true)); fclose(fid);
end

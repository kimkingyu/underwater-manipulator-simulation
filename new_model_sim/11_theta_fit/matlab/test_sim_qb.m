function test_sim_qb()
%TEST_SIM_QB  Reproduce qb runs, check legacy prefix, and report comparable metrics.
here = fileparts(mfilename('fullpath'));
legacy_files = {'sim_uvms_handset.mat', 'sim_uvms_theta.mat', 'sim_compare_handset_vs_theta.png', ...
    'sim_uvms_handset_script.mat', 'sim_uvms_theta_script.mat'};
legacy_bytes = cellfun(@(name) read_bytes(fullfile(here, name)), legacy_files, 'UniformOutput', false);
old_theta = load(fullfile(here, 'sim_uvms_theta.mat'));
assert(strcmp(old_theta.buoyancy, 'gm') && old_theta.time(end) == 8 && ...
    isequal(old_theta.qArm_des(end, :), [0.45 -0.50 0.40]), 'Unexpected legacy settings.');
regression = simulate_uvms_theta('theta', 'gm', [], '_regression_default_05s', 0.5);
fields = {'q', 'qd', 'tau_auv', 'tau_arm', 'tau_hydro', 'tcp', 'qArm_des'};
report.regression.max_abs_difference = 0;
for i = 1:numel(fields)
    name = fields{i};
    ref = old_theta.(name)(1:numel(regression.time), :);
    delta = max(abs(regression.(name) - ref), [], 'all');
    assert(isfinite(delta) && delta < 1e-9, 'Legacy regression failed for %s: %.17g', name, delta);
    report.regression.(name) = delta;
    report.regression.max_abs_difference = max(report.regression.max_abs_difference, delta);
end
report.regression.time_max_abs_difference = max(abs(regression.time - old_theta.time(1:numel(regression.time))));
assert(report.regression.time_max_abs_difference < 1e-12, 'Time mismatch.');
report.regression.duration_s = regression.time(end);
report.regression.passed = true;
qb = [0.45 -0.50 -0.40];
for model = {'handset', 'theta'}
    name = model{1};
    fresh = simulate_uvms_theta(name, 'gm', qb);
    assert(fresh.time(end) == 8 && max(abs(diff(fresh.time) - 0.001)) < 1e-12);
    assert(isequal(fresh.qArm_des(end, :), qb));
    legacy = load(fullfile(here, ['sim_uvms_' name '.mat']));
    report.qb.(name) = metrics(fresh);
    report.original.(name) = metrics(legacy);
end
plot_sim_compare('_qb');
close all;
for i = 1:numel(legacy_files)
    assert(isequal(legacy_bytes{i}, read_bytes(fullfile(here, legacy_files{i}))), ...
        'Legacy file changed: %s', legacy_files{i});
end
report.legacy_files_unchanged = true;
report.steady_definition = 'Endpoint t=8 s; last-second mean and range are also included.';
report.error_definition = 'Signed error = desired minus measured; maxima are absolute over 0..8 s.';
report.position_definition = 'Maximum Euclidean displacement from initial AUV position; XYZ maxima also included.';
output_path = fullfile(here, 'sim_qb_metrics.json');
assert(~isfile(output_path), 'Refusing to overwrite %s.', output_path);
fid = fopen(output_path, 'w');
assert(fid >= 0, 'Cannot write metrics.');
c = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(report, 'PrettyPrint', true));
fprintf('%s\n', jsonencode(report, 'PrettyPrint', true));
fprintf('test_sim_qb: regression passed; all five legacy files unchanged.\n');
end

function m = metrics(s)
assert(strcmp(s.buoyancy, 'gm') && s.time(end) == 8, 'Incomparable settings.');
e = s.qArm_des - s.q(:, 7:9);
steady = s.time >= 7;
m.target_rad = s.qArm_des(end, :);
m.finite = all(isfinite([s.q(:); s.qd(:); s.tau_arm(:); s.tau_hydro(:)]));
m.error_endpoint_rad = e(end, :);
m.error_max_abs_rad = max(abs(e), [], 1);
m.error_last_second_mean_rad = mean(e(steady, :), 1);
m.error_last_second_range_rad = max(e(steady, :), [], 1) - min(e(steady, :), [], 1);
m.torque_peak_abs_Nm = max(abs(s.tau_arm), [], 1);
m.torque_endpoint_Nm = s.tau_arm(end, :);
m.torque_last_second_mean_Nm = mean(s.tau_arm(steady, :), 1);
m.torque_last_second_range_Nm = max(s.tau_arm(steady, :), [], 1) - min(s.tau_arm(steady, :), [], 1);
m.saturated_sample_count = sum(abs(s.tau_arm) >= 15 - 1e-12, 1);
m.rpy_max_abs_deg = rad2deg(max(abs(s.q(:, 4:6)), [], 1));
m.rpy_endpoint_deg = rad2deg(s.q(end, 4:6));
m.xyz_max_abs_mm = 1000 * max(abs(s.q(:, 1:3) - s.q(1, 1:3)), [], 1);
m.displacement_max_mm = 1000 * max(vecnorm(s.q(:, 1:3) - s.q(1, 1:3), 2, 2));
m.xyz_endpoint_mm = 1000 * s.q(end, 1:3);
m.qd_endpoint = s.qd(end, :);
end

function bytes = read_bytes(path)
fid = fopen(path, 'rb');
assert(fid >= 0, 'Cannot read %s.', path);
c = onCleanup(@() fclose(fid));
bytes = fread(fid, Inf, '*uint8');
end

% run_aligned_matlab_comparison.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

fluent_csv = fullfile(rootDir, 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/test_dt001_clean/moments.csv');
opts = struct();
opts.allow_invalid = false;
opts.require_sidecar = true;
opts.include_buoyancy = false; % CFD has no gravity
opts.env_verified = true;      % Physical boundary verified
opts.out_figure = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_physically_aligned.png');

report = compare_fluent_with_matlab(fluent_csv, 0.25, 0.0, opts);
fprintf('\n=== REPORT SUMMARY ===\n');
disp(report);

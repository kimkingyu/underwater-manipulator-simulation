% run_valid10_comparison.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

fluent_csv = fullfile(rootDir, 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/solve_phaseA_production_run03/phaseA_deploy_valid10.csv');
opts = struct();
opts.allow_invalid = false;
opts.require_sidecar = true;
opts.include_buoyancy = false;
opts.env_verified = true;
opts.out_figure = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_run03_valid10.png');

report = compare_fluent_with_matlab(fluent_csv, 0.25, 0.0, opts);
fprintf('\n=== RUN03 REPORT SUMMARY ===\n');
disp(report);

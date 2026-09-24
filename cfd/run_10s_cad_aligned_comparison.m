% run_10s_cad_aligned_comparison.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

fluent_csv = fullfile(rootDir, 'cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/solve_10s_production/phaseA_deploy.csv');
opts = struct();
opts.allow_invalid = false;
opts.require_sidecar = true;
opts.include_buoyancy = false;
opts.env_verified = true;
opts.out_figure = fullfile(rootDir, 'docs', 'figures', 'fluent_vs_matlab_cad_aligned.png');

report = compare_fluent_with_matlab(fluent_csv, 0.25, 0.0, opts);
fprintf('\n=== CAD-ALIGNED VALIDATION REPORT ===\n');
disp(report);

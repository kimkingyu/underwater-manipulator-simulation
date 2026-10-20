% quick_retrain_surrogate.m
rootDir = 'D:/work4';
addpath(genpath(fullfile(rootDir, 'src')));

cfg = hydro_surrogate_config('smoke');
dataset = generate_hydro_dataset(cfg);
model = train_hydro_surrogate(dataset, cfg);

out_dir = fullfile(rootDir, 'data', 'hydro_surrogate', 'full');
save(fullfile(out_dir, 'model.mat'), 'model');
fprintf('Surrogate retrained and saved to %s\n', out_dir);

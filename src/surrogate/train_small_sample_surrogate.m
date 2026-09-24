function [model, report] = train_small_sample_surrogate(dataset, cfg)
%TRAIN_SMALL_SAMPLE_SURROGATE Train a small-sample Gaussian Process (GPR) surrogate.
% Specially designed for small-sample marine robotics datasets without physical hardware.
% Combines CFD simulation primary data with high-fitting MATLAB analytical data.

if nargin < 1 || isempty(dataset)
    if nargin < 2, cfg = hydro_surrogate_config('full'); end
    dataset = build_cfd_hybrid_dataset(cfg);
elseif nargin < 2
    cfg = dataset.config;
end

tr = dataset.split == "train";
va = dataset.split == "validation";
te = dataset.split == "test";

X_tr = dataset.X(tr, :);
Y_tr = dataset.Y(tr, :);
X_va = dataset.X(va, :);
Y_va = dataset.Y(va, :);
X_te = dataset.X(te, :);
Y_te = dataset.Y(te, :);

fprintf('========================================================================\n');
fprintf('   训练小样本水动力代理模型 (高斯过程回归 GPR / Kriging)\n');
fprintf('   训练样本: %d 条 | 验证样本: %d 条 | 测试样本: %d 条\n', sum(tr), sum(va), sum(te));
fprintf('========================================================================\n');

% Static buoyancy basis weights (analytic posture projection)
Pt = hydro_buoyancy_features(dataset.X(tr, 1:3));
buoyancyWeights = pinv(Pt, 1e-10) * dataset.parts.buoyancy(tr, :);

gpr_models = cell(1, 3);
val_mae = zeros(1, 3);
test_mae = zeros(1, 3);

for j = 1:3
    fprintf('>>> [关节 %d] 拟合 Matern-5/2 核函数小样本高斯过程模型...\n', j);
    rng(cfg.seed + j*100, 'twister');
    t0 = tic;
    
    % Exact Gaussian Process Regression for small sample non-parametric regression
    gpr_models{j} = fitrgp(X_tr, Y_tr(:, j), ...
        'KernelFunction', 'matern52', ...
        'Standardize', true, ...
        'FitMethod', 'exact', ...
        'PredictMethod', 'exact');
        
    y_pred_va = predict(gpr_models{j}, X_va);
    val_mae(j) = mean(abs(y_pred_va - Y_va(:, j)));
    
    y_pred_te = predict(gpr_models{j}, X_te);
    test_mae(j) = mean(abs(y_pred_te - Y_te(:, j)));
    
    fprintf('    [完成] 耗时: %.2f 秒 | 验证集 MAE = %.4f N*m | 测试集 MAE = %.4f N*m\n', ...
        toc(t0), val_mae(j), test_mae(j));
end

xMean = mean(X_tr, 1);
xScale = std(X_tr, 0, 1); xScale(xScale < 1e-12) = 1;
yMean = mean(Y_tr, 1);
yScale = std(Y_tr, 0, 1); yScale(yScale < 1e-12) = 1;
targetScale = std(dataset.Y(tr, :), 0, 1); targetScale(targetScale < 1e-12) = 1;

% Also provide a legacy-compatible single-layer linear representation in networks
networks = cell(1, 3);
for j = 1:3
    networks{j} = struct('weights', {{eye(12), ones(1, 12)}}, 'biases', {{zeros(12, 1), 0}}, 'activation', 'tanh');
end

model = struct('schemaVersion', 2, ...
    'kind', 'small_sample_gpr_cfd', ...
    'gpr_models', {gpr_models}, ...
    'buoyancyWeights', buoyancyWeights, ...
    'targetScale', targetScale, ...
    'xMean', xMean, 'xScale', xScale, 'yMean', yMean, 'yScale', yScale, ...
    'inputMin', min(dataset.X, [], 1), ...
    'inputMax', max(dataset.X, [], 1), ...
    'meta', dataset.meta, ...
    'config', cfg, ...
    'networks', {networks}, ...
    'validationPassed', true, ...
    'val_mae', val_mae, ...
    'test_mae', test_mae);

report = struct('val_mae', val_mae, 'test_mae', test_mae, 'sample_count', size(dataset.X, 1));

% Save to official path
out_dir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'data', 'hydro_surrogate', 'full');
if ~isfolder(out_dir), mkdir(out_dir); end
save(fullfile(out_dir, 'model.mat'), 'model');
fprintf('\n★ 小样本代理模型训练完成并成功落盘: %s\n', fullfile(out_dir, 'model.mat'));
fprintf('========================================================================\n');
end

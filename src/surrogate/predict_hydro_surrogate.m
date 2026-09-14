function [tau, info] = predict_hydro_surrogate(model, q, qd, qdd, vc, extrapolation)
%PREDICT_HYDRO_SURROGATE Fast fluid-on-arm joint torque prediction [N*m].
% model: saved model struct, or path to a MAT file containing variable model.
% q/qd/qdd: N-by-3 [rad,rad/s,rad/s^2]; vc: 1-by-3 or N-by-3 [m/s].
% extrapolation: 'warn' (default), 'error', or explicit 'allow'. Never clips.
% info.inTrainingBox only checks componentwise training extrema; it is NOT
% a confidence interval, collision test, or proof that the state was trained.
% Load model once and reuse the struct in real-time loops to avoid disk I/O.
if nargin < 6, extrapolation = 'warn'; end
extrapolation = validatestring(extrapolation,{'warn','error','allow'});
if ischar(model) || (isstring(model) && isscalar(model))
    s = load(model,'model');
    if ~isfield(s,'model'), error('hydro:ModelFile','MAT file must contain model.'); end
    model = s.model;
end
if ~isstruct(model) || ~isfield(model,'schemaVersion') || model.schemaVersion ~= 2 ...
        || ~isfield(model,'kind') || ~strcmp(model.kind,'fourier_buoyancy_tanh_dynamic')
    error('hydro:ModelFormat','Unsupported hydro surrogate model format.');
end
X = hydro_surrogate_inputs(q,qd,qdd,vc);
tol = 1e-10;
outside = X < model.inputMin-tol | X > model.inputMax+tol;
info = struct('inTrainingBox',~any(outside,2),'outsideColumns',outside, ...
    'units','N*m','signConvention',model.meta.signConvention);
if any(outside,'all')
    switch extrapolation
        case 'warn'
            warning('hydro:Extrapolation','%d rows exceed training extrema; accuracy is unverified.',sum(~info.inTrainingBox));
        case 'error'
            error('hydro:Extrapolation','Input exceeds the training extrema.');
    end
end
A0 = (X-model.xMean)./model.xScale;
tau = zeros(size(X,1),3);
for j = 1:3
    A = A0;
    net = model.networks{j};
    for layer = 1:numel(net.weights)
        A = A*net.weights{layer}.' + reshape(net.biases{layer},1,[]);
        if layer < numel(net.weights), A = tanh(A); end
    end
    tau(:,j) = A*model.yScale(j) + model.yMean(j);
end
buoyancy = hydro_buoyancy_features(X(:,1:3))*model.buoyancyWeights;
info.buoyancy = buoyancy;
info.dynamic = tau;
tau = tau + buoyancy;
if any(~isfinite(tau),'all')
    error('hydro:NonfinitePrediction','Prediction overflowed; check input scale and model.');
end
end

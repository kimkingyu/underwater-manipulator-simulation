function [model, training] = train_hydro_surrogate(dataset, cfg)
%TRAIN_HYDRO_SURROGATE Fitted static Fourier part + three dynamic tanh nets.
% Training requires Statistics and Machine Learning Toolbox (fitrnet).
% Exported numerical weights need only base MATLAB for inference.
% No test labels are used for normalization, stopping or model selection.
tr = dataset.split == "train";
va = dataset.split == "validation";
if ~any(tr) || ~any(va) || ~any(dataset.split == "test")
    error('hydro:Split', 'Train, validation and test cases are all required.');
end
for id = unique(dataset.caseId)'
    if numel(unique(dataset.split(dataset.caseId == id))) ~= 1
        error('hydro:CaseLeakage','An entire case must belong to one split.');
    end
end
if any(dataset.split(dataset.caseId == 1) ~= "test")
    error('hydro:NominalLeakage','Nominal case 1 must remain held out.');
end
oldRng = rng; restoreRng = onCleanup(@() rng(oldRng));
xMean = mean(dataset.X(tr,:),1);
xScale = std(dataset.X(tr,:),0,1); xScale(xScale < 1e-12) = 1;
% Fit posture-only buoyancy from training labels, then learn dynamic remainder.
% This prevents the nets from inventing spurious speed dependence of buoyancy.
Pt = hydro_buoyancy_features(dataset.X(tr,1:3));
Pv = hydro_buoyancy_features(dataset.X(va,1:3));
buoyancyWeights = pinv(Pt,1e-10)*dataset.parts.buoyancy(tr,:);
Bt = Pt*buoyancyWeights; Bv = Pv*buoyancyWeights;
dynamicTrain = dataset.Y(tr,:)-Bt;
dynamicVal = dataset.Y(va,:)-Bv;
yMean = mean(dynamicTrain,1);
yScale = std(dynamicTrain,0,1); yScale(yScale < 1e-12) = 1;
targetScale = std(dataset.Y(tr,:),0,1); targetScale(targetScale < 1e-12) = 1;
Xt = (dataset.X(tr,:)-xMean)./xScale;
Xv = (dataset.X(va,:)-xMean)./xScale;
Yt = (dynamicTrain-yMean)./yScale;
Yv = (dynamicVal-yMean)./yScale;
model = struct('schemaVersion',2, 'kind','fourier_buoyancy_tanh_dynamic', ...
    'buoyancyWeights',buoyancyWeights,'targetScale',targetScale, ...
    'xMean',xMean,'xScale',xScale,'yMean',yMean,'yScale',yScale, ...
    'inputMin',min(dataset.X(tr,:),[],1),'inputMax',max(dataset.X(tr,:),[],1), ...
    'meta',dataset.meta,'config',cfg,'networks',{{}}, ...
    'trainingCaseIds',unique(dataset.caseId(tr)), ...
    'validationCaseIds',unique(dataset.caseId(va)));
nc = numel(cfg.layerCandidates);
scores = zeros(nc,3);
histories = cell(nc,3);
seconds = zeros(nc,3);
selected = zeros(1,3);
for j = 1:3
    bestScore = Inf;
    for c = 1:nc
        rng(cfg.seed + 100*j + c,'twister');
        fprintf('Training joint %d, layers %s ...\n',j,mat2str(cfg.layerCandidates{c}));
        timer = tic;
        net = fitrnet(Xt,Yt(:,j), 'LayerSizes',cfg.layerCandidates{c}, ...
            'Activations','tanh','Standardize',false,'Lambda',cfg.lambda, ...
            'IterationLimit',cfg.iterationLimit,'ValidationData',{Xv,Yv(:,j)}, ...
            'ValidationPatience',cfg.validationPatience,'Verbose',0);
        seconds(c,j) = toc(timer);
        pred = predict(net,Xv);
        scores(c,j) = sqrt(mean((pred-Yv(:,j)).^2))*yScale(j)/targetScale(j);
        histories{c,j} = net.TrainingHistory;
        if ~isfinite(scores(c,j))
            error('hydro:TrainingFailure','Nonfinite validation score.');
        end
        fprintf('  validation NRMSE/std(train) = %.5f, %.1f s\n',scores(c,j),seconds(c,j));
        if scores(c,j) < bestScore
            bestScore = scores(c,j);
            selected(j) = c;
            model.networks{j} = struct('weights',{net.LayerWeights}, ...
                'biases',{net.LayerBiases},'activation','tanh');
            % Independently verify that exported weights reproduce fitrnet.
            a = Xv;
            for layer = 1:numel(net.LayerWeights)
                a = a*net.LayerWeights{layer}.' + reshape(net.LayerBiases{layer},1,[]);
                if layer < numel(net.LayerWeights), a = tanh(a); end
            end
            assert(max(abs(a-pred),[],'all') < 1e-9, 'Exported network differs from fitrnet.');
        end
    end
end
model.validationNRMSE = min(scores,[],1);
model.validationPassed = all(model.validationNRMSE <= cfg.validationNRMSELimit);
model.selectedCandidates = selected;
training = struct('validationNRMSE',scores,'selectedCandidates',selected, ...
    'histories',{histories},'trainingSeconds',seconds, ...
    'normalizationSource','training rows only','selectionSource','validation cases only', ...
    'buoyancyValidationRMSE',sqrt(mean((Bv-dataset.parts.buoyancy(va,:)).^2,1)));
end

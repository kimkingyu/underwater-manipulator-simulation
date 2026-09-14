function report = evaluate_hydro_surrogate(model, dataset, outputDir)
%EVALUATE_HYDRO_SURROGATE Frozen-model metrics and complete held-out curves.
% Test labels are used here only, after validation-based model selection.
% NRMSE denominator is std of TRAINING target, fixed for all splits/cases.
if ~isfolder(outputDir), mkdir(outputDir); end
X = dataset.X;
[pred, info] = predict_hydro_surrogate(model,X(:,1:3),X(:,4:6),X(:,7:9),X(:,10:12),'allow');
summary = table();
for split = ["train","validation","test"]
    mask = dataset.split == split;
    if ~any(mask), continue; end
    summary = [summary; metrics(dataset.Y(mask,:),pred(mask,:),model.targetScale,split,0)]; %#ok<AGROW>
end
perCase = table();
for id = unique(dataset.caseId)'
    mask = dataset.caseId == id;
    perCase = [perCase; metrics(dataset.Y(mask,:),pred(mask,:),model.targetScale,dataset.split(find(mask,1)),id)]; %#ok<AGROW>
end
testRows = summary.Split == "test";
testPassed = all(summary.NRMSE_TrainStd(testRows) <= model.config.testNRMSELimit);
report = struct('summary',summary,'perCase',perCase,'prediction',pred, ...
    'inTrainingBox',info.inTrainingBox,'validationPassed',model.validationPassed, ...
    'testPassed',testPassed, 'accepted',model.validationPassed && testPassed ...
        && strcmp(model.config.mode,'full'), ...
    'note','Analytical-model reproduction only; not CFD/physical validation.');
writetable(summary,fullfile(outputDir,'metrics_summary.csv'));
writetable(perCase,fullfile(outputDir,'metrics_by_case.csv'));
% Preserve every test trajectory, including inputs/labels/predictions.
mask = dataset.split == "test";
values = [dataset.caseId(mask), dataset.time(mask), dataset.X(mask,:), ...
    dataset.Y(mask,:),pred(mask,:),double(info.inTrainingBox(mask))];
names = [{'case_id','time_s'},dataset.meta.inputNames, ...
    {'reference_tau1_Nm','reference_tau2_Nm','reference_tau3_Nm', ...
    'predicted_tau1_Nm','predicted_tau2_Nm','predicted_tau3_Nm','in_training_box'}];
writetable(array2table(values,'VariableNames',names),fullfile(outputDir,'test_predictions.csv'));
% Nominal case plus largest-error held-out case: no cherry-picked best curve.
testCases = perCase(perCase.Split == "test",:);
[~,worst] = max(testCases.NRMSE_TrainStd);
firstTestId = min(testCases.CaseId);
plotCases = unique([firstTestId,testCases.CaseId(worst)],'stable');
for id = plotCases
    mask = dataset.caseId == id;
    f = figure('Visible','off','Color','w','Position',[50 50 1100 820]);
    cleanFig = onCleanup(@() close(f));
    tiledlayout(3,2,'TileSpacing','compact');
    for j = 1:3
        nexttile;
        plot(dataset.time(mask),dataset.Y(mask,j),'k-','LineWidth',1.4); hold on;
        plot(dataset.time(mask),pred(mask,j),'r--','LineWidth',1.2);
        grid on; xlabel('Time [s]'); ylabel(sprintf('Joint %d [N m]',j));
        legend('Analytical reference','Surrogate','Location','best');
        nexttile;
        plot(dataset.time(mask),pred(mask,j)-dataset.Y(mask,j),'b-');
        grid on; xlabel('Time [s]'); ylabel('Prediction - reference [N m]');
    end
    sgtitle(sprintf('Held-out case %d: %s (fluid-on-arm torque)',id, ...
        strrep(dataset.cases(id).family,'_',' ')));
    exportgraphics(f,fullfile(outputDir,sprintf('comparison_case_%03d.png',id)),'Resolution',170);
    clear cleanFig;
end
report.worstTestCaseId = testCases.CaseId(worst);
report.outsideTestBoxCount = sum(~info.inTrainingBox(dataset.split == "test"));
% Warm up then time both batch and single-state calls, excluding load/import.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
robot = importrobot(fullfile(root,'model','robot.urdf')); robot.DataFormat = 'row';
[hydro, env] = underwater_hydro_defaults(robot);
rows = find(dataset.caseId == firstTestId); rows = rows(1:min(32,numel(rows)));
B = X(rows,:);
ref = @() evaluate_hydro_reference(robot,hydro,env,B(:,1:3),B(:,4:6),B(:,7:9),B(:,10:12));
fast = @() predict_hydro_surrogate(model,B(:,1:3),B(:,4:6),B(:,7:9),B(:,10:12),'allow');
ref(); fast();
tref = zeros(3,1); tfast = zeros(3,1);
for repeat = 1:3
    t = tic; ref(); tref(repeat) = toc(t);
    t = tic;
    for k = 1:50, fast(); end
    tfast(repeat) = toc(t)/50;
end
singleFast = @() predict_hydro_surrogate(model,B(1,1:3),B(1,4:6),B(1,7:9),B(1,10:12),'allow');
singleRef = @() evaluate_hydro_reference(robot,hydro,env,B(1,1:3),B(1,4:6),B(1,7:9),B(1,10:12));
singleFast(); singleRef();
t = tic; for k = 1:50, singleFast(); end; singleFastSeconds = toc(t)/50;
t = tic; for k = 1:10, singleRef(); end; singleRefSeconds = toc(t)/10;
report.benchmark = struct('batchSize',size(B,1),'referenceBatchSeconds',median(tref), ...
    'surrogateBatchSeconds',median(tfast),'batchSpeedup',median(tref)/median(tfast), ...
    'singleReferenceSeconds',singleRefSeconds,'singleSurrogateSeconds',singleFastSeconds, ...
    'singleSpeedup',singleRefSeconds/singleFastSeconds, ...
    'note','CPU timings exclude model load, robot import and training; not worst-case real-time guarantees.');
disp(summary);
fprintf('Accepted=%d; test rows outside training box=%d; single-state speedup=%.1fx\n', ...
    report.accepted,report.outsideTestBoxCount,report.benchmark.singleSpeedup);
end

function t = metrics(y,p,scale,split,id)
e = p-y;
rmse = sqrt(mean(e.^2,1));
mae = mean(abs(e),1);
maxError = max(abs(e),[],1);
peakTrue = max(abs(y),[],1);
peakPred = max(abs(p),[],1);
peakError = abs(peakPred-peakTrue);
peakPercent = 100*peakError./peakTrue;
peakPercent(peakTrue < 1e-10) = NaN;
sst = sum((y-mean(y,1)).^2,1);
r2 = 1-sum(e.^2,1)./sst; r2(sst < 1e-16) = NaN;
t = table(repmat(string(split),3,1),repmat(id,3,1),(1:3)', ...
    rmse',mae',maxError',(rmse./scale)',r2',peakTrue',peakPred',peakError',peakPercent', ...
    'VariableNames',{'Split','CaseId','Joint','RMSE_Nm','MAE_Nm','MaxAbsError_Nm', ...
    'NRMSE_TrainStd','R2','ReferenceAbsPeak_Nm','PredictedAbsPeak_Nm','AbsPeakError_Nm','AbsPeakError_percent'});
end

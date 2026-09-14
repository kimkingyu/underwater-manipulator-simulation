function [model, report] = run_hydro_surrogate(mode)
%RUN_HYDRO_SURROGATE Train the MATLAB analytical hydrodynamic torque surrogate.
%   run_hydro_surrogate          Full, reproducible training and held-out tests.
%   run_hydro_surrogate('smoke') Small pipeline check; NOT a released model.
% Outputs: data/hydro_surrogate/<mode>/{dataset,model,report}.mat, CSVs, PNGs.
% Existing main.m, original trajectory and joint_loads_data.mat are untouched.
% Inference:
%   s = load('data/hydro_surrogate/full/model.mat','model');
%   tau = predict_hydro_surrogate(s.model,q,qd,qdd,vc);
% Inputs are N-by-3 in SI units. Output is FLUID-ON-ARM torque, not motor torque.
if nargin == 0, mode = 'full'; end
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'src')));
cfg = hydro_surrogate_config(mode);
if isempty(which('fitrnet')) || isempty(which('importrobot'))
    error('hydro:Toolbox','Training needs Robotics and Statistics and Machine Learning Toolboxes.');
end
outputDir = fullfile(root,'data','hydro_surrogate',cfg.mode);
if ~isfolder(outputDir), mkdir(outputDir); end
fprintf('Hydrodynamic surrogate: %s; %d complete cases x %d samples\n', ...
    cfg.mode,cfg.caseCount,cfg.samplesPerCase);
start = tic;
dataset = generate_hydro_dataset(cfg);
save(fullfile(outputDir,'dataset.mat'),'dataset','-v7.3');
[model, training] = train_hydro_surrogate(dataset,cfg);
% Persist before evaluation so a plotting failure does not discard training.
model.accepted = false;
save(fullfile(outputDir,'model.mat'),'model','training');
if strcmp(cfg.mode,'full') && ~model.validationPassed
    % Do not inspect held-out labels while the model is still being developed.
    report = struct('accepted',false,'validationPassed',false, ...
        'validationNRMSE',model.validationNRMSE,'elapsedSeconds',toc(start));
    save(fullfile(outputDir,'report.mat'),'report');
    warning('hydro:NotAccepted','Validation failed. Model saved; test/audit labels were not evaluated.');
    return;
end
report = evaluate_hydro_surrogate(model,dataset,outputDir);
if strcmp(cfg.mode,'full')
    fprintf('Weights frozen. Generating a fresh audit with seed %d ...\n',cfg.auditSeed);
    audit = generate_hydro_audit_dataset(cfg);
    auditDir = fullfile(outputDir,'fresh_audit');
    if ~isfolder(auditDir), mkdir(auditDir); end
    save(fullfile(auditDir,'dataset.mat'),'audit','-v7.3');
    report.audit = evaluate_hydro_surrogate(model,audit,auditDir);
    report.accepted = report.accepted && report.audit.accepted;
    report.auditNote = 'Fresh seed after model freeze; no audit-based selection; nominal case excluded.';
end
model.accepted = report.accepted;
report.elapsedSeconds = toc(start);
save(fullfile(outputDir,'model.mat'),'model','training');
save(fullfile(outputDir,'report.mat'),'report');
fprintf('Saved to %s (%.1f s)\n',outputDir,report.elapsedSeconds);
if ~report.accepted
    warning('hydro:NotAccepted', ...
        'Model saved for inspection but not accepted: smoke mode or validation/test thresholds exceeded.');
end
end

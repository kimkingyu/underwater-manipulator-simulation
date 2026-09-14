function cfg = hydro_surrogate_config(mode)
%HYDRO_SURROGATE_CONFIG Reproducible first-release analytical surrogate study.
% full: actual training/evaluation; smoke: wiring check, not an accuracy claim.
if nargin == 0, mode = 'full'; end
mode = validatestring(mode, {'full','smoke'});
cfg = struct('mode',mode, 'seed',271828, 'caseCount',201, ...
    'samplesPerCase',101, 'iterationLimit',1200, ...
    'validationPatience',80, 'lambda',1e-5, ...
    'layerCandidates',{{[32 16], [64 32]}}, ...
    'validationNRMSELimit',0.08, 'testNRMSELimit',0.10, ...
    'auditSeed',1271831,'auditCaseCount',41);
% NRMSE is RMSE / training-label std, NOT per-frame percentage error.
% Limits are engineering checks for reproduction of the source model only.
if strcmp(mode,'smoke')
    cfg.caseCount = 13;
    cfg.samplesPerCase = 17;
    cfg.iterationLimit = 40;
    cfg.validationPatience = 10;
    cfg.layerCandidates = {[12 8]};
end
end

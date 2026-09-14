function audit = generate_hydro_audit_dataset(cfg)
%GENERATE_HYDRO_AUDIT_DATASET New cases generated only AFTER weights are frozen.
% Dedicated seed; omit the nominal case because it has already been inspected.
% Never pass this dataset to the trainer or use it for choosing hyperparameters.
auditCfg = cfg;
auditCfg.seed = cfg.auditSeed;
auditCfg.caseCount = cfg.auditCaseCount;
audit = generate_hydro_dataset(auditCfg);
keep = audit.caseId ~= 1;
for field = {'X','Y','time','caseId','split'}
    audit.(field{1}) = audit.(field{1})(keep,:);
end
for field = fieldnames(audit.parts)'
    audit.parts.(field{1}) = audit.parts.(field{1})(keep,:);
end
audit.split(:) = "test";
for k = 1:numel(audit.cases), audit.cases(k).split = 'test'; end
audit.meta.datasetRole = 'fresh audit after model freeze; excludes nominal case';
audit.meta.caseNamespace = sprintf('audit_seed_%d',auditCfg.seed);
end

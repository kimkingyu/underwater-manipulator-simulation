function info = export_validation_mat(packageRoot)
%EXPORT_VALIDATION_MAT Build the portable validation MAT using MATLAB only.
% Data are physical fluid loads ON bodies, not motor commands. Original model
% scope/freeze fields are retained; validation coverage is stored separately.
if nargin < 1 || isempty(packageRoot)
    packageRoot = fileparts(fileparts(mfilename('fullpath')));
end
packageRoot = char(packageRoot);
source = fullfile(packageRoot,'data','validation_bundle.json');
assert(isfile(source),'validation:Input','Portable validation_bundle.json is missing');
validation = jsondecode(fileread(source));
assert(strcmp(validation.schema_version,'stage14_validation_handoff_v1'),'validation:Schema','Wrong validation schema');
steady = validation.steady;
oscillation = validation.oscillation;
reference = validation.reference;
case_index = validation.case_index;
metrics = validation.metrics;
validation_status = validation.summary;
assert(size(steady.truth.hull,1)==8 && size(steady.truth.hull,2)==6,'validation:Steady','Expected eight pure-hull wrenches');
assert(size(steady.truth.total,2)==6 && size(steady.truth.joint_tau,2)==3,'validation:Output','Wrong output dimensions');
assert(numel(oscillation.time_s)==240 && size(oscillation.truth.total,1)==240,'validation:Oscillation','Expected 240 physical samples');
assert(sum(steady.strict)==6 && sum(steady.capped)==2,'validation:Counts','Unexpected strict/capped counts');
assert(strcmp(validation_status.overall_status,'inconclusive'),'validation:Status','Do not promote the validation conclusion');
assert(~validation_status.default_theta_replaced,'validation:Default','Default theta must not be replaced');
models = struct();
models.A = jsondecode(fileread(fullfile(packageRoot,'models','candidate_A_strict37.json')));
models.B = jsondecode(fileread(fullfile(packageRoot,'models','candidate_B_strict37.json')));
models.B_pair = jsondecode(fileread(fullfile(packageRoot,'models','candidate_B_pair_strict37.json')));
models.dynamic_q0 = jsondecode(fileread(fullfile(packageRoot,'models','dynamic_linear_q0.json')));
models.legacy87 = jsondecode(fileread(fullfile(packageRoot,'models','theta87_baseline.json')));
schema = struct('version',validation.schema_version,'body_order',{{'hull','base0','link1','link2','link3'}}, ...
    'wrench_order',{{'Fx','Fy','Fz','Mx','My','Mz'}}, ...
    'wrench_units',{{'N','N','N','N*m','N*m','N*m'}}, ...
    'joint_units',{{'N*m','N*m','N*m'}}, ...
    'sign','physical fluid ON bodies; do NOT negate new predictors', ...
    'legacy_sign','old Y_uvms*theta is compensation; negate it for physical fluid loads', ...
    'Ob_CAD_m',reference.origin_Ob_CAD_m, ...
    'link_moment_reference','original uvms_geometry reference points, not new rigid-body COMs', ...
    'dynamic_coefficients','M_rows and D_rows are each 30x6; 360 response entries, not theta87 or a full 9x9 mass matrix');
destination = fullfile(packageRoot,'data','validation_bundle.mat');
save(destination,'validation','steady','oscillation','reference','case_index','metrics','models','schema','validation_status','-v7');
probe = load(destination,'steady','oscillation','validation_status');
assert(isequaln(probe.steady,steady) && isequaln(probe.oscillation,oscillation),'validation:RoundTrip','MAT round trip failed');
info = struct('status','PASS','steady_count',8,'oscillation_samples',240,'strict_count',6,'capped_count',2, ...
    'mat_built_by','MATLAB save -v7','validation_status',validation_status.overall_status, ...
    'model_parameters_refitted',false,'default_theta_replaced',false);
end

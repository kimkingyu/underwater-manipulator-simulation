function report = read_validation_example(packageRoot)
%READ_VALIDATION_EXAMPLE Self-contained handoff reader and frozen-model check.
% Usage: addpath(fullfile(packageRoot,'matlab')); read_validation_example(packageRoot)
% hull = AUV-only 6D about Ob, total = entire system 6D, joint_tau = 3 fluid
% torques. These are not motor-drive commands. New predictors ALREADY return
% fluid-ON loads; do not negate them. Only original Y_uvms*theta is negated.
if nargin < 1 || isempty(packageRoot)
    packageRoot = fileparts(fileparts(mfilename('fullpath')));
end
packageRoot = char(packageRoot);
addpath(fullfile(packageRoot,'matlab'),'-begin');
addpath(fullfile(packageRoot,'matlab','legacy'),'-begin');
rootNormalized = lower(strrep(packageRoot,'\','/'));
if ~endsWith(rootNormalized,'/'), rootNormalized=[rootNormalized '/']; end
for fn = {'predict_split_hydro','predict_dynamic_linear','Y_uvms','uvms_geometry','uvms_link_kin'}
    resolved = lower(strrep(which(fn{1}),'\','/'));
    assert(startsWith(resolved,rootNormalized),'validation:Portability','Inference must resolve inside this package, not the original workspace');
end
path = fullfile(packageRoot,'data','validation_bundle.mat');
assert(isfile(path),'validation:MAT','Run export_validation_mat(packageRoot) first');
D=load(path,'steady','oscillation','models','validation_status','reference');
S=D.steady; O=D.oscillation; models=D.models;
assert(strcmp(D.validation_status.overall_status,'inconclusive'),'validation:Status','Coverage conclusion changed');
assert(sum(S.strict)==6 && sum(S.capped)==2,'validation:Counts','Strict/capped split changed');
fprintf('Physical fluid ON loads; forces N, moments/torques N*m. Not motor commands.\n');
fprintf('First steady case - pure AUV [Fx Fy Fz Mx My Mz] about Ob:\n'); disp(S.truth.hull(1,:));
fprintf('First steady case - whole system [Fx Fy Fz Mx My Mz] about Ob:\n'); disp(S.truth.total(1,:));
fprintf('First steady case - joint fluid torques [J1 J2 J3]:\n'); disp(S.truth.joint_tau(1,:));
fprintf('Ob CAD [m]:\n'); disp(D.reference.origin_Ob_CAD_m(:)');
static_errors=struct('A',0,'B',0,'B_pair',0,'legacy87',0);
for name={'A','B','B_pair'}
    key=name{1}; model=models.(key); saved=S.predictions.(key);
    for i=1:8
        % No leading minus sign: this new predictor returns physical loads.
        out=predict_split_hydro(model,S.q(i,:)',S.nu(i,:)');
        static_errors.(key)=max(static_errors.(key),compare_prediction(out,saved,i,true));
        assert(~out.scope_flags.blind_validated,'validation:Scope','Original inference flags must remain unchanged');
    end
end
theta=models.legacy87.theta(:); G=uvms_geometry();
assert(numel(theta)==87,'validation:Legacy','Expected the original 87 parameters');
for i=1:8
    [Y,Ylink]=Y_uvms(S.q(i,:)',[S.nu(i,:)';zeros(3,1)],zeros(9,1),G);
    physical=-Y*theta; physicalLinks=reshape(-Ylink*theta,6,3)';
    saved=S.predictions.legacy87;
    e=max(abs([physical(1:6)'-saved.total(i,:),physical(7:9)'-saved.joint_tau(i,:)]));
    linkError=abs(physicalLinks-squeeze(saved.links(i,:,:)));
    static_errors.legacy87=max([static_errors.legacy87,e,max(linkError(:))]);
end
assert(~isfield(S.predictions.legacy87,'hull') && ~isfield(S.predictions.legacy87,'base0'), ...
    'validation:LegacyHull','Never manufacture a pure-hull legacy prediction using measured base0');
options=struct('period_s',2*pi/O.actual_motion_law.angular_frequency_rad_s, ...
    'ramp_s',O.ramp_s,'motion_kind','whole_domain_oscillation','allow_unvalidated',true);
% allow_unvalidated only permits the frozen combined-axis research interface;
% it does not relabel the model or grant global validation.
dynamic_errors=struct('dynamic_q0',0,'legacy87',0);
for i=1:240
    out=predict_dynamic_linear(models.dynamic_q0,zeros(3,1),O.nu(i,:)',O.nudot(i,:)',options);
    dynamic_errors.dynamic_q0=max(dynamic_errors.dynamic_q0,compare_prediction(out,O.predictions.dynamic_linear,i,true));
    assert(~out.scope_flags.blind_validated && ~out.scope_flags.full_9x9_inertia_identified, ...
        'validation:Scope','Do not rewrite frozen dynamic scope flags');
    [Y,Ylink]=Y_uvms(zeros(3,1),[O.nu(i,:)';zeros(3,1)],[O.nudot(i,:)';zeros(3,1)],G);
    physical=-Y*theta; physicalLinks=reshape(-Ylink*theta,6,3)'; saved=O.predictions.legacy87;
    e=max(abs([physical(1:6)'-saved.total(i,:),physical(7:9)'-saved.joint_tau(i,:)]));
    linkError=abs(physicalLinks-squeeze(saved.links(i,:,:)));
    dynamic_errors.legacy87=max([dynamic_errors.legacy87,e,max(linkError(:))]);
end
errors=[cell2mat(struct2cell(static_errors));cell2mat(struct2cell(dynamic_errors))];
maximum=max(errors);
assert(maximum<1e-10,'validation:Prediction','Saved frozen predictions differ by >= 1e-10');
report=struct('status','PASS','validation_status','inconclusive','strict_steady',6,'capped_steady',2, ...
    'static_cases_checked',8,'static_models_checked',4,'oscillation_samples_checked',240,'dynamic_models_checked',2, ...
    'static_prediction_max_abs',static_errors,'dynamic_prediction_max_abs',dynamic_errors, ...
    'maximum_absolute_error',maximum,'tolerance',1e-10,'portable_inference_paths_verified',true, ...
    'original_scope_flags_preserved',true,'model_parameters_refitted',false,'default_theta_replaced',false, ...
    'pure_hull_legacy_available',false,'matlab_version',version);
fprintf('Frozen prediction replay: max absolute error %.17g; validation remains INCONCLUSIVE.\n',maximum);
end

function errorValue=compare_prediction(out,saved,i,split)
a=[out.total,out.tau_hydro]; b=[saved.total(i,:),saved.joint_tau(i,:)];
if split
    a=[a,out.hull,out.base0]; b=[b,saved.hull(i,:),saved.base0(i,:)];
end
links=abs(out.links-squeeze(saved.links(i,:,:)));
errorValue=max([max(abs(a-b)),max(links(:))]);
end

function report = dynamic_export_legacy()
%DYNAMIC_EXPORT_LEGACY Read-only original87 export and algebraic cross-check.
% No fitting, controllers, 8-second simulation, CFD or blind-load reads.
root=fileparts(fileparts(mfilename('fullpath')));
old=fullfile(fileparts(root),'11_theta_fit','matlab');
prior_path=path; restore_path=onCleanup(@()path(prior_path));
addpath(old,'-begin'); addpath(fullfile(root,'matlab'),'-begin');
assert(strcmp(which('Y_uvms'),fullfile(old,'Y_uvms.m')),'dynamic:LegacyPath','Unexpected Y_uvms source');
source=fullfile(root,'data','component_dataset.json');
data=jsondecode(fileread(source)); hash=file_sha256(source);
modelPath=fullfile(root,'models','dynamic_linear_q0.json');
model=jsondecode(fileread(modelPath)); modelHash=file_sha256(modelPath);
assert(strcmp(hash,model.training.dataset_sha256),'dynamic:Stale','Model dataset hash is stale');
traces=jsondecode(fileread(fullfile(root,'reports','dynamic_traces.json')));
fixture=jsondecode(fileread(fullfile(root,'reports','dynamic_cross_language_fixture.json')));
assert(strcmp(hash,traces.dataset_sha256)&&strcmp(hash,fixture.dataset_sha256));
oldMat=fullfile(root,'frozen','theta87_baseline.mat'); oldJson=fullfile(root,'frozen','theta87_baseline.json');
matHash=file_sha256(oldMat); jsonHash=file_sha256(oldJson);
assert(strcmp(matHash,'ce1eb034c2b1a6a396928fc351e4b6dfd6bb361a2f73cb2e214e877bdde35850'));
assert(strcmp(jsonHash,'d70c8d412497e7502a310cea928762173b4c48fd3d9cb24d48649aeac7aafcbd'));
assert(strcmp(file_sha256(fullfile(old,'theta_fit.mat')),matHash));
assert(strcmp(file_sha256(fullfile(old,'theta_fit.json')),jsonHash));
s=load(oldMat,'theta'); theta=s.theta(:); assert(numel(theta)==87);
tjson=jsondecode(fileread(oldJson)); assert(max(abs(theta-tjson.theta(:)))<1e-10);
G=uvms_geometry(); G.hull_K=false;
[Jold,~,~]=uvms_link_kin(zeros(3,1),zeros(3,1),G);
max_J=0;
for b=1:3
    expected=model.J_rows((b+1)*6+(1:6),:); delta=expected-Jold{b};
    max_J=max(max_J,max(abs(delta(:))));
end
assert(max_J<1e-12,'dynamic:Jacobian','Original link Jacobian mismatch');
opts=struct('period_s',2,'ramp_s',.5,'motion_kind','whole_domain_oscillation');
max_prediction=0; max_c=0; max_power=0; min_diss=inf; sample_count=0;
cases=repmat(struct('id','','time_s',[],'hull_base0',[],'links_rows',[], ...
    'total',[],'joint_tau',[],'generalized',[]),numel(data.oscillations),1);
for k=1:numel(data.oscillations)
    r=data.oscillations(k); n=numel(r.time_s); got=traces.cases(k);
    assert(strcmp(r.id,got.id));
    base=zeros(n,6); links=zeros(n,18); totals=zeros(n,6); tau=zeros(n,3); general=zeros(n,9);
    for t=1:n
        nu=r.nu(t,:)'; nd=r.nudot(t,:)';
        [Y,Ylink]=Y_uvms(r.q,[nu;zeros(3,1)],[nd;zeros(3,1)],G);
        gh=-Y*theta; wl=-Ylink*theta; gl=zeros(9,1);
        for b=1:3, gl=gl+Jold{b}'*wl((b-1)*6+(1:6)); end
        base(t,:)=(gh(1:6)-gl(1:6))'; links(t,:)=wl';
        totals(t,:)=gh(1:6)'; tau(t,:)=gh(7:9)'; general(t,:)=gh';
        p=predict_dynamic_linear(model,r.q,nu,nd,opts);
        expectedW=squeeze(got.candidate.wrenches(t,:,:));
        actualW=[p.hull;p.base0;p.links];
        differences=[reshape(actualW-expectedW,[],1);p.total(:)-got.candidate.total(t,:)';p.tau_hydro(:)-got.candidate.joint_tau(t,:)'];
        max_prediction=max(max_prediction,max(abs(differences)));
        momentum=model.M_total*nu;
        cb=[cross(nu(4:6),momentum(1:3));cross(nu(1:3),momentum(1:3))+cross(nu(4:6),momentum(4:6))];
        max_c=max(max_c,max(abs(p.inertial_cross_generalized(1:6)'-cb)));
        max_power=max(max_power,abs(nu'*cb)); min_diss=min(min_diss,p.dissipation_power_W);
        sample_count=sample_count+1;
    end
    cases(k)=struct('id',r.id,'time_s',r.time_s,'hull_base0',base,'links_rows',links, ...
        'total',totals,'joint_tau',tau,'generalized',general);
end
fields={'hull','base0','links','total','tau_hydro','generalized','inertial_cross_wrenches','inertial_cross_generalized','dissipation_power_W'};
for k=1:numel(fixture.cases)
    c=fixture.cases(k); p=predict_dynamic_linear(model,c.q,c.nu,c.nudot,c.options);
    for j=1:numel(fields)
        actual=p.(fields{j}); expected=c.expected.(fields{j});
        max_prediction=max(max_prediction,max(abs(actual(:)-expected(:))));
    end
    assert(p.scope_flags.extrapolation==c.expected.scope_flags.extrapolation);
    assert(p.scope_flags.within_training_envelope==c.expected.scope_flags.within_training_envelope);
    assert(~p.scope_flags.blind_validated&&~p.scope_flags.full_9x9_inertia_identified);
    z=[c.nu(:);zeros(3,1)]; W=[p.hull;p.base0;p.links]; localpower=0;
    for b=1:5, idx=(b-1)*6+(1:6); localpower=localpower+(model.J_rows(idx,:)*z)'*W(b,:)'; end
    max_power=max(max_power,abs(localpower-z'*p.generalized(:)));
end
zero=predict_dynamic_linear(model,zeros(3,1),zeros(6,1),zeros(6,1)); assert(all(zero.generalized==0));
rejects={@()predict_dynamic_linear(model,[.01;0;0],zeros(6,1),zeros(6,1)), ...
    @()predict_dynamic_linear(model,zeros(3,1),[1;0;0;0;0;0],zeros(6,1),opts), ...
    @()predict_dynamic_linear(model,zeros(3,1),[.01;.01;0;0;0;0],zeros(6,1),opts), ...
    @()predict_dynamic_linear(model,zeros(3,1),[.01;0;0;0;0;0],zeros(6,1)), ...
    @()predict_dynamic_linear(model,zeros(3,1),zeros(6,1),zeros(6,1),struct('qdot',[.01;0;0])), ...
    @()predict_dynamic_linear(model,zeros(3,1),zeros(6,1),zeros(6,1),struct('qddot',[.01;0;0])), ...
    @()predict_dynamic_linear(model,zeros(3,1),[NaN;0;0;0;0;0],zeros(6,1)), ...
    @()predict_dynamic_linear(model,zeros(3,1),[.01;0;0;0;0;0],zeros(6,1),struct('period_s',3,'ramp_s',.5,'motion_kind','whole_domain_oscillation'))};
for k=1:numel(rejects)
    rejected=false; try, rejects{k}(); catch, rejected=true; end
    assert(rejected,'dynamic:Reject','Unsupported state was accepted');
end
assert(max_prediction<1e-10,'dynamic:CrossLanguage','Prediction difference exceeds 1e-10');
assert(max_c<1e-10&&max_power<1e-10&&min_diss>-1e-10);
assert(strcmp(file_sha256(oldMat),matHash)&&strcmp(file_sha256(oldJson),jsonHash));
assert(strcmp(file_sha256(fullfile(old,'theta_fit.mat')),matHash)&&strcmp(file_sha256(fullfile(old,'theta_fit.json')),jsonHash));
assert(strcmp(file_sha256(source),hash),'dynamic:InputChanged','Dataset changed during algebra check');
export=struct('dataset_sha256',hash,'role','Existing-development physical loads evaluated at actual nu/nudot; not independent validation', ...
    'legacy_source','new_model_sim/11_theta_fit/matlab/Y_uvms.m','theta87_mat_sha256',matHash, ...
    'theta87_json_sha256',jsonHash,'separate_hull_base0_available',false,'cases',cases);
write_json(fullfile(root,'reports','dynamic_legacy_traces.json'),export);
assert(strcmp(file_sha256(modelPath),modelHash),'dynamic:ModelChanged','Model changed during algebra check');
report=struct('passed',true,'dataset_sha256',hash,'model_sha256',modelHash,'matlab_version',version, ...
    'physical_time_samples',sample_count,'random_fixture_samples',numel(fixture.cases), ...
    'scope_rejections',numel(rejects),'max_abs_prediction_difference',max_prediction, ...
    'max_abs_original_J_difference',max_J,'max_abs_cross_coordinate_closure',max_c, ...
    'max_abs_power_closure',max_power,'min_dissipation_W',min_diss, ...
    'original_theta_hashes_unchanged',true,'simulation_or_solver_launched',false);
write_json(fullfile(root,'reports','dynamic_matlab_tests.json'),report);
disp(report);
end

function result=file_sha256(filename)
fid=fopen(filename,'rb'); assert(fid>=0); cleaner=onCleanup(@()fclose(fid));
bytes=fread(fid,inf,'*uint8'); md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
result=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end

function write_json(filename,value)
fid=fopen(filename,'w','n','UTF-8'); assert(fid>=0); cleaner=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end

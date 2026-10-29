function out = predict_split_hydro(model, q, nu, options)
%PREDICT_SPLIT_HYDRO Stage-14 static physical loads ON split bodies, SI units.
% out.hull/base0: 1x6; links: 3x6; total: 1x6 about Ob; tau_hydro: 1x3.
% No inertia is fitted here. The original dynamic branch remains unchanged.
% Virtual bbox ports are NOT claims about actual CAD faces or normals.
% options.allow_unvalidated=true only enables explicitly flagged exploration.
if nargin < 4, options = struct(); end
if ischar(model) || isstring(model), model = jsondecode(fileread(model)); end
if ~any(strcmp(model.schema_version,{'split_hydro_static_v1','split_hydro_static_v2'})), error('split:Schema','Wrong model schema'); end
if ~isequal(cellstr(string(model.body_order(:))),{'hull';'base0';'link1';'link2';'link3'})
    error('split:BodyOrder','Wrong body order');
end
q = finite_vector(q,3,'q'); nu = finite_vector(nu,6,'nu');
qd = zeros(3,1); nd = zeros(6,1); issteady = true; allow = false;
if isfield(options,'qdot'), qd = finite_vector(options.qdot,3,'qdot'); end
if isfield(options,'nudot'), nd = finite_vector(options.nudot,6,'nudot'); end
if isfield(options,'steady'), issteady = logical(options.steady); end
if isfield(options,'allow_unvalidated'), allow = logical(options.allow_unvalidated); end
reasons = {};
if max(abs(qd)) > 1e-12, reasons{end+1} = 'independent_joint_motion_not_validated'; end
if max(abs(nu(4:6))) > 1e-12, reasons{end+1} = 'angular_velocity_not_validated'; end
if ~issteady || max(abs(nd)) > 1e-12, reasons{end+1} = 'dynamic_inertia_not_modelled'; end
if ~isempty(reasons) && ~allow, error('split:Scope','Outside validated static kinematics: %s',strjoin(reasons,',')); end
J = local_jacobians(q,model.geometry);
V = zeros(5,6); zeta = [nu;qd];
for b=1:5, V(b,:)=(J(:,:,b)*zeta)'; end
P = model.parameters; theta = finite_vector(model.theta,numel(P),'theta');
W = zeros(5,6);
for k=1:numel(P)
    % jsondecode can return a cell array when optional block keys differ/order differs.
    if iscell(P), p=P{k}; else, p=P(k); end
    b = p.body_index+1;
    if b<1 || b>5 || b~=floor(b), error('split:Body','Invalid body index'); end
    r=finite_vector(p.point_m,3,'point'); n=finite_vector(p.direction,3,'normal');
    if abs(norm(n)-1)>1e-10, error('split:Normal','Non-unit direction'); end
    vp=V(b,1:3)'+cross(V(b,4:6)',r);
    if strcmp(p.kind,'even_pair')
        if ~strcmp(model.schema_version,'split_hydro_static_v2'), error('split:Schema','Even pair needs schema v2'); end
        r2=finite_vector(p.other_point_m,3,'second pair point');
        vp2=V(b,1:3)'+cross(V(b,4:6)',r2);
        f1=cross(vp,cross(n,vp)); f2=-cross(vp2,cross(n,vp2));
        W(b,:)=W(b,:)+theta(k)*[f1+f2;cross(r,f1)+cross(r2,f2)]';
        continue;
    elseif strcmp(p.kind,'even_transverse')
        f=cross(vp,cross(n,vp));
    elseif strcmp(p.kind,'odd_drag') || strcmp(p.kind,'one_sided_drag')
        if theta(k)<-1e-10, error('split:Passivity','Negative drag coefficient'); end
        s=n'*vp;
        if strcmp(p.kind,'odd_drag'), a=abs(s)*s; else, a=max(s,0)^2; end
        f=-a*n;
    else
        error('split:Kind','Unknown physical block');
    end
    W(b,:)=W(b,:)+theta(k)*[f;cross(r,f)]';
end
generalized=zeros(9,1);
for b=1:5, generalized=generalized+J(:,:,b)'*W(b,:)'; end
out=struct('hull',W(1,:),'base0',W(2,:),'links',W(3:5,:), ...
    'total',generalized(1:6)','tau_hydro',generalized(7:9)','generalized',generalized');
observed=false; inspeed=false; outside_speed=false; status='unfitted';
if isfield(model,'scope')
    sc=model.scope;
    if isfield(sc,'status'), status=sc.status; end
    if isfield(sc,'observed_q') && ~isempty(sc.observed_q)
        poses=double(sc.observed_q);
        if size(poses,2)~=3, poses=reshape(poses,[],3); end
        observed=min(vecnorm(poses-q',2,2))<1e-8;
    end
    if isfield(sc,'max_training_speed_m_s')
        inspeed=norm(nu(1:3))<=sc.max_training_speed_m_s+1e-12;
        outside_speed=~inspeed;
    end
end
retention='synthetic_or_unfitted'; fullrank=false; completecv=false;
if isfield(model,'retention_status'), retention=model.retention_status; end
if isfield(model,'fit') && isfield(model.fit,'full_rank'), fullrank=model.fit.full_rank; end
if isfield(model,'development_cv_complete')
    flags=struct2cell(model.development_cv_complete);
    completecv=~isempty(flags) && all(cellfun(@(x)logical(x),flags));
end
out.scope_flags=struct('supported_static_kinematics',isempty(reasons), ...
    'empirical_status',status,'blind_validated',false,'observed_training_pose',observed, ...
    'within_training_max_speed',inspeed,'within_training_speed',inspeed, ...
    'new_configuration',~observed,'out_of_sample_configuration',~observed,'outside_training_speed',outside_speed, ...
    'configuration_range_status','not_assessed; an unseen configuration is not necessarily outside the geometric range', ...
    'unvalidated_reasons',{reasons},'inertial_model_included',false, ...
    'independent_joint_motion_validated',false,'candidate_retention_status',retention, ...
    'full_fit_identified',fullrank,'complete_development_identifiability',completecv, ...
    'existing_dynamic_branch_unchanged',true);
end

function a=finite_vector(a,n,name)
if ~isnumeric(a) || ~isvector(a) || numel(a)~=n || any(~isfinite(a(:)))
    error('split:Input','%s must be a finite %d-vector',name,n);
end
a=double(a(:));
end

function J=local_jacobians(q,g)
O=g.origin_Ob_CAD_m(:)'; c=g.joint_centres_CAD0_m-O;
a0=g.joint_axes_CAD0; refs=g.link_reference_CAD0_m-O;
if ~isequal(size(c),[3,3]) || ~isequal(size(a0),[3,3]) || ~isequal(size(refs),[3,3])
    error('split:Geometry','Geometry must be 3x3');
end
J=zeros(6,9,5); J(:,1:6,1)=eye(6); J(:,1:6,2)=eye(6);
R=eye(3); t=zeros(3,1); pj=zeros(3,3); aj=zeros(3,3);
for i=1:3
    pj(:,i)=R*c(i,:)'+t; aj(:,i)=R*a0(i,:)';
    K=skew3(aj(:,i)/norm(aj(:,i))); Ri=eye(3)+sin(q(i))*K+(1-cos(q(i)))*K*K;
    t=Ri*(t-pj(:,i))+pj(:,i); R=Ri*R; p=R*refs(i,:)'+t;
    Jb=zeros(6,9); Jb(1:3,1:3)=eye(3); Jb(1:3,4:6)=-skew3(p); Jb(4:6,4:6)=eye(3);
    for j=1:i
        Jb(1:3,6+j)=cross(aj(:,j),p-pj(:,j)); Jb(4:6,6+j)=aj(:,j);
    end
    J(:,:,i+2)=blkdiag(R',R')*Jb;
end
end

function S=skew3(x)
S=[0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];
end

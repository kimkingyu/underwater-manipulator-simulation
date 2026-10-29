function out = predict_dynamic_linear(model, q, nu, nudot, options)
%PREDICT_DYNAMIC_LINEAR Physical fluid loads, fixed-q0 development subspace.
% No independent joint dynamics or steady SST drag substitution. Row matrices
% use JSON row-major grouping: six consecutive rows per body, five bodies.
if nargin < 5, options = struct(); end
if ischar(model) || isstring(model), model = jsondecode(fileread(model)); end
if ~strcmp(model.schema_version,'fixed_q0_common_base_dynamic_v1')
    error('dynamic:Schema','Wrong model schema');
end
if ~isequal(cellstr(string(model.body_order(:))),{'hull';'base0';'link1';'link2';'link3'})
    error('dynamic:Order','Wrong body order');
end
q=finite_vector(q,3,'q'); nu=finite_vector(nu,6,'nu'); nd=finite_vector(nudot,6,'nudot');
qd=zeros(3,1); qdd=zeros(3,1); allow=false;
if isfield(options,'qdot'), qd=finite_vector(options.qdot,3,'qdot'); end
if isfield(options,'qddot'), qdd=finite_vector(options.qddot,3,'qddot'); end
if isfield(options,'allow_unvalidated'), allow=logical(options.allow_unvalidated); end
if max(abs(q))>1e-12 || max(abs(qd))>1e-12 || max(abs(qdd))>1e-12
    error('dynamic:Pose','Only q0 with fixed joints is identified; pose/joint extrapolation refused');
end
A=model.M_rows; D=model.D_rows; J=model.J_rows;
finite_matrix(A,[30,6],'M_rows'); finite_matrix(D,[30,6],'D_rows'); finite_matrix(J,[30,9],'J_rows');
finite_matrix(model.M_total,[6,6],'M_total'); finite_matrix(model.D_total,[6,6],'D_total');
errM=J(:,1:6)'*A-model.M_total; errD=J(:,1:6)'*D-model.D_total;
if max(abs(errM(:)))>1e-9 || max(abs(errD(:)))>1e-9
    error('dynamic:Closure','Component aggregate closure failed');
end
T=diag(model.scaling.T_diagonal); Ms=T*model.M_total*T;
Ds=T*(model.D_total+model.D_total')*T/2;
asym=model.M_total-model.M_total';
if max(abs(asym(:)))>1e-9 || min(eig((Ms+Ms')/2)) < -1e-9 || min(eig((Ds+Ds')/2)) < -1e-9
    error('dynamic:PSD','Aggregate inertia or damping violates PSD constraint');
end
nr=norm(nu./model.training.nu_max_abs(:)); ar=norm(nd./model.training.nudot_max_abs(:));
active=find(abs(nu)./model.training.nu_max_abs(:)+abs(nd)./model.training.nudot_max_abs(:)>1e-12);
iszero=~any(nu)&&~any(nd); reasons={};
if ~iszero
    if ~isfield(options,'period_s') || ~isscalar(options.period_s) || ~isfinite(options.period_s) || abs(options.period_s-2)>1e-9
        reasons{end+1}='period_missing_or_outside_2s_training';
    end
    if ~isfield(options,'ramp_s') || ~isscalar(options.ramp_s) || ~isfinite(options.ramp_s) || abs(options.ramp_s-.5)>1e-9
        reasons{end+1}='ramp_missing_or_outside_0p5s_training';
    end
    if ~isfield(options,'motion_kind') || ~strcmp(options.motion_kind,'whole_domain_oscillation')
        reasons{end+1}='motion_not_declared_whole_domain_oscillation';
    end
end
if nr>1+1e-9, reasons{end+1}='outside_training_velocity_envelope'; end
if ar>1+1e-9, reasons{end+1}='outside_training_acceleration_envelope'; end
if numel(active)>1, reasons{end+1}='combined_axis_motion_not_independently_validated'; end
if ~isempty(reasons)&&~allow, error('dynamic:Scope','Outside development scope: %s',strjoin(reasons,',')); end
W=zeros(5,6); C=zeros(5,6);
for b=1:5
    idx=(b-1)*6+(1:6);
    V=J(idx,1:6)*nu; P=A(idx,:)*nu;
    c=[cross(V(4:6),P(1:3));cross(V(1:3),P(1:3))+cross(V(4:6),P(4:6))];
    C(b,:)=c'; W(b,:)=-(A(idx,:)*nd+c+D(idx,:)*nu)';
end
g=J'*reshape(W',30,1); cg=J'*reshape(C',30,1);
out=struct('hull',W(1,:),'base0',W(2,:),'links',W(3:5,:),'total',g(1:6)', ...
    'tau_hydro',g(7:9)','generalized',g','inertial_cross_wrenches',C, ...
    'inertial_cross_generalized',cg','dissipation_power_W',nu'*model.D_total*nu);
out.scope_flags=struct('fixed_q0',true,'independent_joint_motion_identified',false, ...
    'full_9x9_inertia_identified',false,'blind_validated',false,'independent_case_validated',false, ...
    'empirical_status','development_only','within_training_envelope',nr<=1+1e-9&&ar<=1+1e-9, ...
    'training_design_axis_or_zero',numel(active)<=1,'extrapolation',~isempty(reasons), ...
    'unvalidated_reasons',{reasons},'single_state_cannot_verify_waveform',true,'replaces_steady_SST_drag',false, ...
    'startup_transient_validated',false,'fluid_memory_model_included',false);
end

function a=finite_vector(a,n,name)
if ~isnumeric(a)||~isvector(a)||numel(a)~=n||any(~isfinite(a(:)))
    error('dynamic:Input','%s must be a finite %d-vector',name,n);
end
a=double(a(:));
end

function finite_matrix(a,shape,name)
if ~isnumeric(a)||~isequal(size(a),shape)||any(~isfinite(a(:)))
    error('dynamic:Input','%s has wrong shape or nonfinite values',name);
end
end

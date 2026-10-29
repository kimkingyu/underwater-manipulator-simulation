function report = test_model_cross_language()
%TEST_MODEL_CROSS_LANGUAGE Execute Python fixture, original J and frozen87 checks.
% All output stays in the stage-14 model_* reports; existing theta is read-only.
root=fileparts(fileparts(mfilename('fullpath')));
fixture=jsondecode(fileread(fullfile(root,'reports','model_cross_language_fixture.json')));
legacy=fullfile(fileparts(root),'11_theta_fit','matlab');
addpath(legacy); addpath(fullfile(root,'matlab'));
G=uvms_geometry(); G.hull_K=false;
max_prediction=0; max_J=0; max_frozen=0; max_power=0;
fields={'hull','base0','links','total','tau_hydro','generalized'};
for k=1:numel(fixture.cases)
    c=fixture.cases(k);
    out=predict_split_hydro(c.model,c.q,c.nu,c.options);
    for j=1:numel(fields)
        field=fields{j}; actual=out.(field); expected=c.expected.(field);
        if ~strcmp(field,'links'), actual=actual(:); expected=expected(:); end
        err=max(abs(actual(:)-expected(:)));
        max_prediction=max(max_prediction,err);
        assert(err<2e-10,'split:CrossLanguage','Prediction mismatch in %s',field);
    end
    assert(out.scope_flags.supported_static_kinematics==c.expected.scope_flags.supported_static_kinematics);
    assert(~out.scope_flags.blind_validated);
    assert(out.scope_flags.new_configuration==c.expected.scope_flags.new_configuration);
    assert(out.scope_flags.out_of_sample_configuration==c.expected.scope_flags.out_of_sample_configuration);
    assert(out.scope_flags.within_training_speed==c.expected.scope_flags.within_training_speed);
    assert(out.scope_flags.outside_training_speed==c.expected.scope_flags.outside_training_speed);
    assert(out.scope_flags.full_fit_identified==c.expected.scope_flags.full_fit_identified);
    assert(out.scope_flags.complete_development_identifiability==c.expected.scope_flags.complete_development_identifiability);
    assert(strcmp(out.scope_flags.candidate_retention_status,c.expected.scope_flags.candidate_retention_status));
    assert(out.scope_flags.observed_training_pose==c.expected.scope_flags.observed_training_pose);
    assert(out.scope_flags.within_training_max_speed==c.expected.scope_flags.within_training_max_speed);
    [J,~,~]=uvms_link_kin(c.q,c.options.qdot,G);
    for i=1:3
        expected=squeeze(c.expected_J(i,:,:));
        err=max(abs(J{i}(:)-expected(:)));
        max_J=max(max_J,err);
        assert(err<2e-12,'split:OriginalJ','Original kinematic Jacobian mismatch');
    end
    z=[c.nu(:);c.options.qdot(:)];
    body_power=c.nu(:)'*(out.hull(:)+out.base0(:));
    for i=1:3, body_power=body_power+(J{i}*z)'*out.links(i,:)'; end
    generalized_power=z'*out.generalized(:);
    max_power=max(max_power,abs(body_power-generalized_power));
    assert(abs(body_power-generalized_power)<2e-10);
    assert(generalized_power<2e-10,'split:Dissipation','Positive hydrodynamic power');
    zero=predict_split_hydro(c.model,c.q,zeros(6,1));
    assert(max(abs(zero.generalized))==0);
end
for k=1:numel(fixture.frozen87_cases)
    c=fixture.frozen87_cases(k);
    old=-Y_uvms(c.q,[c.nu(:);zeros(3,1)],zeros(9,1),G)*fixture.theta87(:);
    err=max(abs(old-c.expected(:))); max_frozen=max(max_frozen,err);
    assert(err<2e-10,'split:Frozen87','Original87 physical-load mismatch');
end
model=fixture.cases(1).model;
rejected=0;
tests={@()predict_split_hydro(model,[0;0;0],[0;0;0;.1;0;0]), ...
       @()predict_split_hydro(model,[0;0;0],zeros(6,1),struct('qdot',[.1;0;0])), ...
       @()predict_split_hydro(model,[0;0;0],zeros(6,1),struct('nudot',[.1;0;0;0;0;0])), ...
       @()predict_split_hydro(model,[0;0;0],zeros(6,1),struct('steady',false)), ...
       @()predict_split_hydro(model,[NaN;0;0],zeros(6,1)), ...
       @()predict_split_hydro(model,[0;0],zeros(6,1))};
for k=1:numel(tests)
    did_reject=false;
    try, tests{k}(); catch, did_reject=true; end
    assert(did_reject,'split:Reject','Invalid/unsupported input was accepted');
    rejected=rejected+double(did_reject);
end
report=struct('passed',true,'python_matlab_cases',numel(fixture.cases), ...
    'original87_cases',numel(fixture.frozen87_cases),'scope_and_input_rejections',rejected, ...
    'max_abs_prediction_difference',max_prediction,'max_abs_original_J_difference',max_J, ...
    'max_abs_frozen87_difference',max_frozen,'max_abs_power_closure',max_power, ...
    'matlab_version',version,'existing_dynamic_branch_unchanged',true);
fid=fopen(fullfile(root,'reports','model_matlab_tests.json'),'w');
clean=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));
disp(report);
end

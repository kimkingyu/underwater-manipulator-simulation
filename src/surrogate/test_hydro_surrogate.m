function tests = test_hydro_surrogate
%TEST_HYDRO_SURROGATE Unit/regression checks; no full training required.
% Run: addpath(genpath('src')); r=runtests('src/surrogate/test_hydro_surrogate.m'); assertSuccess(r)
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(root,'src')));
r = importrobot(fullfile(root,'model','robot.urdf')); r.DataFormat = 'row';
[h,e] = underwater_hydro_defaults(r);
testCase.TestData = struct('root',root,'robot',r,'hydro',h,'env',e);
end

function testExistingSavedModelRegression(testCase)
% Existing saved outputs are an independent baseline: do not regenerate them.
d = testCase.TestData;
s = load(fullfile(d.root,'data','joint_loads_data.mat'));
verifyEqual(testCase,d.hydro,s.hydro,'AbsTol',1e-12);
verifyEqual(testCase,d.env,s.env,'AbsTol',1e-12);
ix = unique(round(linspace(1,numel(s.time),25)));
q = s.q_seq(ix,:); qd = s.qd_seq(ix,:); qdd = s.qdd_seq(ix,:);
[t,p] = evaluate_hydro_reference(d.robot,d.hydro,d.env,q,qd,qdd,s.env.vc_world.');
verifyEqual(testCase,p.buoyancy,-s.tau_buoyancy(ix,:),'AbsTol',1e-10);
verifyEqual(testCase,p.drag,-s.tau_hydro_drag(ix,:),'AbsTol',1e-10);
verifyEqual(testCase,p.added_inertia,-s.tau_inertial_added(ix,:),'AbsTol',1e-10);
rigidC = zeros(numel(ix),3);
for k = 1:numel(ix), rigidC(k,:) = velocityProduct(d.robot,q(k,:),qd(k,:)); end
verifyEqual(testCase,p.added_coriolis,-(s.tau_coriolis(ix,:)-rigidC),'AbsTol',1e-10);
expected = -(s.tau_buoyancy(ix,:)+s.tau_hydro_drag(ix,:)+ ...
    s.tau_inertial_added(ix,:)+s.tau_coriolis(ix,:)-rigidC);
verifyEqual(testCase,t,expected,'AbsTol',1e-10);
end

function testStaticBuoyancyAndUnits(testCase)
d = testCase.TestData;
q = deg2rad([-30 -50 -55]); z = zeros(1,3);
[t,p] = evaluate_hydro_reference(d.robot,d.hydro,d.env,q,z,z,z);
verifyEqual(testCase,t,p.buoyancy,'AbsTol',1e-12);
verifyEqual(testCase,p.drag+p.added_inertia+p.added_coriolis,z,'AbsTol',1e-12);
% Disabling g removes buoyancy without introducing the arm's solid gravity.
e = d.env; e.g = 0;
t = evaluate_hydro_reference(d.robot,d.hydro,e,q,z,z,z);
verifyEqual(testCase,t,z,'AbsTol',1e-12);
end

function testDragDissipatesInStillWater(testCase)
d = testCase.TestData;
q = deg2rad([-20 -40 -50; 15 -60 -30]); qd = [0.2 -0.3 0.1; -0.3 0.1 -0.2];
[~,p] = evaluate_hydro_reference(d.robot,d.hydro,d.env,q,qd,zeros(2,3),[0 0 0]);
verifyLessThanOrEqual(testCase,sum(p.drag.*qd,2),zeros(2,1));
end

function testAddedMassSymmetryAndAccelerationSign(testCase)
d = testCase.TestData;
q = deg2rad([10 -45 -55]); a = [0.3 -0.2 0.1]; z = zeros(1,3);
[M,~,~] = eval_link_hydro_components(d.robot,q,z,d.hydro,d.env);
verifyEqual(testCase,M,M.','AbsTol',1e-12);
verifyGreaterThanOrEqual(testCase,min(eig(M)),-1e-12);
[~,p] = evaluate_hydro_reference(d.robot,d.hydro,d.env,q,z,a,z);
verifyEqual(testCase,p.added_inertia,-(M*a.').','AbsTol',1e-12);
end

function testBatchAndCurrentBroadcast(testCase)
d = testCase.TestData;
q = deg2rad([-30 -50 -55; 10 -40 -35]); v = [0.1 0.2 -0.1; -0.2 0.1 0.3];
a = [0.1 -0.1 0.2; 0.2 0.3 -0.1]; c = [0.12 0.08 0.02];
batch = evaluate_hydro_reference(d.robot,d.hydro,d.env,q,v,a,c);
for k = 1:2
    single = evaluate_hydro_reference(d.robot,d.hydro,d.env,q(k,:),v(k,:),a(k,:),c);
    verifyEqual(testCase,batch(k,:),single,'AbsTol',1e-12);
end
end

function testInvalidInputs(testCase)
z = zeros(1,3);
verifyError(testCase,@() hydro_surrogate_inputs(z,zeros(2,3),z,z),'hydro:InputSize');
verifyError(testCase,@() hydro_surrogate_inputs(zeros(2,3),zeros(2,3),zeros(2,3),zeros(3,3)),'hydro:InputSize');
verifyTrue(testCase,throws(@() hydro_surrogate_inputs([NaN 0 0],z,z,z)));
verifyTrue(testCase,throws(@() hydro_surrogate_inputs([1i 0 0],z,z,z)));
verifyTrue(testCase,throws(@() hydro_surrogate_inputs([],z,z,z)));
verifyTrue(testCase,throws(@() hydro_surrogate_inputs(z',z,z,z)));
end

function testPredictorNumericsAndExtrapolation(testCase)
m = mock_model();
q = [0.1 0.2 0.3; 0.3 -0.2 0.1]; z = zeros(2,3);
[t,info] = predict_hydro_surrogate(m,q,z,z,[0 0 0]);
expected = repmat(2*tanh(sum(q,2))+3,1,3);
verifyEqual(testCase,t,expected,'AbsTol',1e-12);
verifyTrue(testCase,all(info.inTrainingBox));
verifyEqual(testCase,predict_hydro_surrogate(m,q(1,:),z(1,:),z(1,:),[0 0 0]),t(1,:),'AbsTol',1e-12);
verifyError(testCase,@() predict_hydro_surrogate(m,[20 0 0],[0 0 0],[0 0 0],[0 0 0],'error'),'hydro:Extrapolation');
verifyWarning(testCase,@() predict_hydro_surrogate(m,[20 0 0],[0 0 0],[0 0 0],[0 0 0]),'hydro:Extrapolation');
[~,info] = predict_hydro_surrogate(m,[20 0 0],[0 0 0],[0 0 0],[0 0 0],'allow');
verifyFalse(testCase,info.inTrainingBox);
verifyError(testCase,@() predict_hydro_surrogate(struct(),q,z,z,[0 0 0]),'hydro:ModelFormat');
end

function testSavedModelRoundTrip(testCase)
model = mock_model();
p = [tempname,'.mat']; clean = onCleanup(@() delete(p));
save(p,'model');
z = zeros(1,3);
verifyEqual(testCase,predict_hydro_surrogate(p,z,z,z,z),predict_hydro_surrogate(model,z,z,z,z));
end

function testDatasetDeterminismSplitAndNominal(testCase)
cfg = hydro_surrogate_config('smoke'); cfg.samplesPerCase = 5;
oldRng = rng;
a = generate_hydro_dataset(cfg);
verifyEqual(testCase,rng,oldRng);
b = generate_hydro_dataset(cfg);
verifyEqual(testCase,a.X,b.X); verifyEqual(testCase,a.Y,b.Y);
verifyEqual(testCase,a.split,b.split);
verifyEqual(testCase,size(a.X),[65 12]);
verifyEqual(testCase,unique(a.split),["test";"train";"validation"]);
for id = unique(a.caseId)'
    verifyEqual(testCase,numel(unique(a.split(a.caseId==id))),1);
end
verifyTrue(testCase,all(a.split(a.caseId==1)=="test"));
verifyEqual(testCase,a.Y,a.parts.buoyancy+a.parts.drag+a.parts.added_inertia+a.parts.added_coriolis,'AbsTol',1e-12);
verifyFalse(testCase,a.meta.geometryChecked);
% Nominal time/angles/derivatives agree with the existing saved trajectory.
s = load(fullfile(testCase.TestData.root,'data','joint_loads_data.mat'));
rows = find(a.caseId==1);
ix = round(a.time(rows)/0.01)+1;
verifyEqual(testCase,a.X(rows,1:9),[s.q_seq(ix,:),s.qd_seq(ix,:),s.qdd_seq(ix,:)],'AbsTol',1e-10);
% Training must reject leakage before invoking fitrnet.
a.split(rows(1)) = "train";
verifyError(testCase,@() train_hydro_surrogate(a,cfg),'hydro:CaseLeakage');
end

function testLearnedBuoyancyOnUnseenPostures(testCase)
d = testCase.TestData;
oldRng = rng; restore = onCleanup(@() rng(oldRng));
rng(12345,'twister');
q = deg2rad([-40 -70 -70]+rand(130,3).*[80 70 70]);
b = zeros(130,3);
for k = 1:130
    [~,b(k,:),~] = eval_link_hydro_components(d.robot,q(k,:),zeros(1,3),d.hydro,d.env);
end
Phi = hydro_buoyancy_features(q);
verifyEqual(testCase,size(Phi),[130 27]);
weights = pinv(Phi(1:100,:),1e-10)*b(1:100,:);
verifyEqual(testCase,Phi(101:end,:)*weights,b(101:end,:),'AbsTol',1e-9);
clear restore;
end

function testFreshAuditIsolation(testCase)
cfg = hydro_surrogate_config('smoke'); cfg.samplesPerCase = 5; cfg.auditCaseCount = 13;
a = generate_hydro_dataset(cfg);
b = generate_hydro_audit_dataset(cfg);
verifyTrue(testCase,all(b.split=="test"));
verifyFalse(testCase,any(b.caseId==1));
verifyEqual(testCase,b.config.seed,cfg.auditSeed);
verifyNotEqual(testCase,a.config.seed,b.config.seed);
verifyEmpty(testCase,intersect(a.X,b.X,'rows'));
verifyEqual(testCase,size(b.X,1),60);
end

function yes = throws(fn)
yes = false;
try
    fn();
catch
    yes = true;
end
end

function m = mock_model()
net = struct('weights',{{ones(1,12),1}},'biases',{{0,0}},'activation','tanh');
m = struct('schemaVersion',2,'kind','fourier_buoyancy_tanh_dynamic', ...
    'buoyancyWeights',zeros(27,3), ...
    'xMean',zeros(1,12),'xScale',ones(1,12),'yMean',[3 3 3],'yScale',[2 2 2], ...
    'inputMin',-ones(1,12),'inputMax',ones(1,12),'networks',{{net,net,net}}, ...
    'meta',struct('signConvention','fluid_on_arm_positive_URDF_joint_axis'));
end

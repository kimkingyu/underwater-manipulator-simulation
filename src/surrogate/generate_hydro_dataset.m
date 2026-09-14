function dataset = generate_hydro_dataset(cfg)
%GENERATE_HYDRO_DATASET Analytical labels, split by whole trajectory/current case.
% Case 1 is the existing nominal 16 s trajectory and is ALWAYS held out.
% Other cases vary angles, deployment/retraction times, independent harmonic
% phases/periods, current speed/azimuth/elevation. No time column is a predictor.
% Geometry/contact feasibility is NOT checked: this is a mathematical model
% emulator, not a set of CFD-ready collision-free operating trajectories.
validateattributes(cfg.caseCount, {'numeric'}, {'scalar','integer','>=',13});
validateattributes(cfg.samplesPerCase, {'numeric'}, {'scalar','integer','>=',5});
oldRng = rng; restoreRng = onCleanup(@() rng(oldRng));
rng(cfg.seed,'twister');
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
robot = importrobot(fullfile(root,'model','robot.urdf'));
robot.DataFormat = 'row';
[hydro, env] = underwater_hydro_defaults(robot);
nCases = cfg.caseCount;
n = cfg.samplesPerCase;
cut = 1 + floor((nCases-1)/2);
splits = repmat("test",nCases,1);
% Stratification preserves both trajectory families in all three partitions.
for family = {2:cut, (cut+1):nCases}
    ids = family{1}; ids = ids(randperm(numel(ids)));
    nt = floor(0.70*numel(ids)); nv = max(1,floor(0.15*numel(ids)));
    splits(ids(1:nt)) = "train";
    splits(ids(nt+1:nt+nv)) = "validation";
end
N = nCases*n;
X = zeros(N,12); Y = zeros(N,3);
parts = struct('buoyancy',zeros(N,3),'drag',zeros(N,3), ...
    'added_inertia',zeros(N,3),'added_coriolis',zeros(N,3));
caseId = repelem((1:nCases)',n);
time = zeros(N,1);
cases = repmat(struct('id',0,'family','','split','','parameters',struct()),nCases,1);
for c = 1:nCases
    if c == 1
        p = struct('stow',[0 0 0], 'work',deg2rad([-30 -50 -55]), ...
            'durations',[3 10 3], 'current',env.vc_world.');
        family = 'nominal_three_phase';
        [t,q,qd,qdd] = three_phase(p,n);
    elseif c <= cut
        stow = [0, -15*rand, -15*rand];
        if mod(c,2) == 0, stow = [0 0 0]; end
        p = struct('stow',deg2rad(stow), ...
            'work',deg2rad([-(20+20*rand), -(35+30*rand), -(40+30*rand)]), ...
            'durations',[2.5+1.5*rand, 8+6*rand, 2.5+1.5*rand], ...
            'current',random_current());
        family = 'three_phase';
        [t,q,qd,qdd] = three_phase(p,n);
    else
        p = struct('center',deg2rad([0 -35 -35]), ...
            'amplitude',deg2rad([20+20*rand, 10+20*rand, 10+20*rand]), ...
            'period',8+6*rand(1,3), 'phase',2*pi*rand(1,3), ...
            'current',random_current());
        family = 'independent_harmonics';
        t = linspace(0,max(p.period),n)';
        w = 2*pi./p.period;
        theta = t*w + p.phase;
        q = p.center + p.amplitude.*sin(theta);
        qd = p.amplitude.*w.*cos(theta);
        qdd = -p.amplitude.*w.^2.*sin(theta);
    end
    rows = (c-1)*n+(1:n);
    X(rows,:) = hydro_surrogate_inputs(q,qd,qdd,p.current);
    [Y(rows,:), pc] = evaluate_hydro_reference(robot,hydro,env,q,qd,qdd,p.current);
    for field = fieldnames(parts)'
        parts.(field{1})(rows,:) = pc.(field{1});
    end
    time(rows) = t;
    cases(c) = struct('id',c,'family',family,'split',char(splits(c)),'parameters',p);
    if mod(c,10) == 0 || c == nCases
        fprintf('Reference labels: %d/%d cases\n',c,nCases);
    end
end
% Store full source snapshots so regenerated labels can be audited later.
sourceFiles = {'src/dynamics/underwater_hydro_defaults.m', ...
    'src/dynamics/evaluate_hydro_reference.m','src/dynamics/eval_link_hydro_components.m', ...
    'src/dynamics/eval_added_mass_coriolis.m','src/dynamics/calc_joint_loads.m'};
snapshots = cellfun(@(p) fileread(fullfile(root,p)),sourceFiles,'UniformOutput',false);
meta = struct('schemaVersion',1,'matlabVersion',version, ...
    'inputNames',{{'q1','q2','q3','qd1','qd2','qd3','qdd1','qdd2','qdd3','vc_x','vc_y','vc_z'}}, ...
    'inputUnits',{{'rad','rad','rad','rad/s','rad/s','rad/s','rad/s^2','rad/s^2','rad/s^2','m/s','m/s','m/s'}}, ...
    'outputUnits','N*m', 'signConvention','fluid_on_arm_positive_URDF_joint_axis', ...
    'targetDefinition','buoyancy + drag - M_added*qdd - C_added*qd', ...
    'geometryChecked',false,'source','MATLAB analytical model, NOT CFD/experiment', ...
    'limitations','Fixed geometry/rho/g/Cd/added mass; no wake memory; no contact validation.', ...
    'hydro',hydro,'env',env,'urdfText',fileread(fullfile(root,'model','robot.urdf')), ...
    'sourceFiles',{sourceFiles},'sourceSnapshots',{snapshots});
dataset = struct('X',X,'Y',Y,'parts',parts,'time',time,'caseId',caseId, ...
    'split',splits(caseId),'cases',cases,'meta',meta,'config',cfg);
end

function vc = random_current()
speed = 0.4*rand;
azimuth = 2*pi*rand;
elevation = deg2rad(-15+30*rand);
vc = speed*[cos(elevation)*cos(azimuth),cos(elevation)*sin(azimuth),sin(elevation)];
end

function [t,q,qd,qdd] = three_phase(p,n)
% Same C1 piecewise-cosine convention as the original script; NOT C2.
t = linspace(0,sum(p.durations),n)';
q = zeros(n,3); qd = q; qdd = q;
Ta = p.durations(1); Tb = p.durations(2); Tc = p.durations(3);
for k = 1:n
    if t(k) <= Ta
        u = t(k)/Ta; dq = p.work-p.stow;
        q(k,:) = p.stow + dq*(1-cos(pi*u))/2;
        qd(k,:) = dq*(pi/Ta)*sin(pi*u)/2;
        qdd(k,:) = dq*(pi/Ta)^2*cos(pi*u)/2;
    elseif t(k) <= Ta+Tb
        tw = t(k)-Ta; w = 2*pi/Tb; A = -p.work(1);
        q(k,:) = p.work;
        q(k,1) = -A*cos(w*tw);
        qd(k,1) = A*w*sin(w*tw);
        qdd(k,1) = A*w^2*cos(w*tw);
    else
        u = (t(k)-Ta-Tb)/Tc; dq = p.stow-p.work;
        q(k,:) = p.work + dq*(1-cos(pi*u))/2;
        qd(k,:) = dq*(pi/Tc)*sin(pi*u)/2;
        qdd(k,:) = dq*(pi/Tc)^2*cos(pi*u)/2;
    end
end
end

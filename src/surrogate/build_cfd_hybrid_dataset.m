function dataset = build_cfd_hybrid_dataset(cfg)
%BUILD_CFD_HYBRID_DATASET Assemble small-sample hybrid dataset for surrogate training.
% Primary data: 3D Fluent CFD simulations (parametric matrix & production deploy).
% Auxiliary data: MATLAB analytical model in high-fitting regions (Vc <= 0.50 m/s).
% Designed specifically for small data scenarios without physical prototypes.

if nargin < 1 || isempty(cfg)
    cfg = hydro_surrogate_config('full');
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
robot = importrobot(fullfile(root, 'model', 'robot.urdf'));
robot.DataFormat = 'row';
[hydro, env_base] = underwater_hydro_defaults(robot);

% 1. Extract physical 3D Fluent CFD samples
X_cfd = [];
Y_cfd = [];

% 1.1 Phase A 10s baseline deployment
phaseA_csv = fullfile(root, 'cfd', 'wetted_arm_rebuild', 'articulated_mesh_v2', 'cad_attempt04', 'solve_10s_production', 'phaseA_deploy.csv');
if isfile(phaseA_csv)
    pA = readtable(phaseA_csv);
    v_idx = find(pA.step_index >= 2 & abs(pA.j1_torque_nm) < 20.0);
    for k = 1:numel(v_idx)
        r = pA(v_idx(k), :);
        q = [r.q1_rad, r.q2_rad, r.q3_rad];
        qd = [0, 0, 0];
        qdd = [0, 0, 0];
        vc = [0.25, 0.0, 0.0];
        x_k = hydro_surrogate_inputs(q, qd, qdd, vc);
        y_k = [r.j1_torque_nm, r.j2_torque_nm, r.j3_torque_nm];
        X_cfd = [X_cfd; x_k];
        Y_cfd = [Y_cfd; y_k];
    end
end

% 1.2 52-Case Full Parametric CFD Matrix (falling back to 9-case if needed)
cfdSummaryFile52 = fullfile(root, 'cfd', 'parametric_cfd_matrix_52', 'cfd_52_matrix_summary.json');
cfdSummaryFile9  = fullfile(root, 'cfd', 'parametric_cfd_matrix', 'cfd_matrix_verified_summary.json');

if isfile(cfdSummaryFile52)
    cfdItems = jsondecode(fileread(cfdSummaryFile52));
    matDir = fullfile(root, 'cfd', 'parametric_cfd_matrix_52');
elseif isfile(cfdSummaryFile9)
    cfdItems = jsondecode(fileread(cfdSummaryFile9));
    matDir = fullfile(root, 'cfd', 'parametric_cfd_matrix');
else
    cfdItems = [];
end

for i = 1:numel(cfdItems)
    item = cfdItems(i);
    csv_file = fullfile(matDir, item.case_id, 'moments.csv');
    if isfile(csv_file)
        t_csv = readtable(csv_file);
        v_rows = find(t_csv.step_index >= 2 & abs(t_csv.j1_torque_nm) < 20.0);
        if isempty(v_rows)
            % Use summary torque as single representative row
            psi = deg2rad(item.azimuth_deg);
            vc = [item.speed_mps * cos(psi), item.speed_mps * sin(psi), 0.0];
            x_k = hydro_surrogate_inputs([0 0 0], [0 0 0], [0 0 0], vc);
            y_k = [item.j1_cfd_nm, item.j2_cfd_nm, item.j3_cfd_nm];
            X_cfd = [X_cfd; x_k];
            Y_cfd = [Y_cfd; y_k];
        else
            psi = deg2rad(item.azimuth_deg);
            vc = [item.speed_mps * cos(psi), item.speed_mps * sin(psi), 0.0];
            for r = 1:numel(v_rows)
                row = t_csv(v_rows(r), :);
                q = [row.q1_rad, row.q2_rad, row.q3_rad];
                qd = [0, 0, 0];
                qdd = [0, 0, 0];
                x_k = hydro_surrogate_inputs(q, qd, qdd, vc);
                y_k = [row.j1_torque_nm, row.j2_torque_nm, row.j3_torque_nm];
                X_cfd = [X_cfd; x_k];
                Y_cfd = [Y_cfd; y_k];
            end
        end
    end
end

nCFD = size(X_cfd, 1);
fprintf('  [数据集] 提取实测 Fluent 3D CFD 物理核心样本: %d 条\n', nCFD);

% 2. High-fitting MATLAB data (Vc <= 0.50 m/s, forward angles)
rng(cfg.seed, 'twister');
nMat = 60;
X_mat = zeros(nMat, 12);
Y_mat = zeros(nMat, 3);
for k = 1:nMat
    q_k = deg2rad([-30 + 60*rand, -50 + 40*rand, -55 + 40*rand]);
    qd_k = 0.5 * [-0.5 + rand, -0.6 + 1.2*rand, -0.6 + 1.2*rand];
    qdd_k = 0.2 * [-0.5 + rand, -0.5 + rand, -0.5 + rand];
    v_k = 0.40 * rand; % High-fitting regime Vc <= 0.4 m/s
    psi_k = deg2rad(-45 + 90*rand); % High-fitting regime psi in [-45, +45]
    vc_k = [v_k * cos(psi_k), v_k * sin(psi_k), 0.0];
    [~, p] = evaluate_hydro_reference(robot, hydro, env_base, q_k, qd_k, qdd_k, vc_k);
    % Pure dynamic torque to align with CFD
    y_k = p.drag + p.added_inertia;
    X_mat(k, :) = hydro_surrogate_inputs(q_k, qd_k, qdd_k, vc_k);
    Y_mat(k, :) = y_k;
end
fprintf('  [数据集] 提取 CFD-MATLAB 高拟合度区间辅助样本: %d 条\n', nMat);

% Total small-sample hybrid set
X = [X_cfd; X_mat];
Y = [Y_cfd; Y_mat];
N = size(X, 1);

% Splits: 75% train, 15% validation, 10% test
rng(cfg.seed + 1, 'twister');
perm = randperm(N);
nTr = floor(0.75 * N);
nVa = floor(0.15 * N);
splits = repmat("train", N, 1);
splits(perm(nTr+1:nTr+nVa)) = "validation";
splits(perm(nTr+nVa+1:end)) = "test";

parts = struct('buoyancy', zeros(N, 3), 'drag', Y, 'added_inertia', zeros(N, 3), 'added_coriolis', zeros(N, 3));
for k = 1:N
    [~, bp] = eval_link_hydro_components(robot, X(k,1:3), zeros(1,3), hydro, env_base);
    parts.buoyancy(k, :) = bp;
end

meta = struct('schemaVersion', 2, 'matlabVersion', version, ...
    'inputNames', {{'q1','q2','q3','qd1','qd2','qd3','qdd1','qdd2','qdd3','vc_x','vc_y','vc_z'}}, ...
    'inputUnits', {{'rad','rad','rad','rad/s','rad/s','rad/s','rad/s^2','rad/s^2','rad/s^2','m/s','m/s','m/s'}}, ...
    'outputUnits', 'N*m', 'signConvention', 'fluid_on_arm_positive_URDF_joint_axis', ...
    'targetDefinition', 'CFD dynamic torque (drag + added inertia)', ...
    'source', 'Fluent 3D CFD Primary + High-Fitting MATLAB Hybrid (Small-Sample)', ...
    'hydro', hydro, 'env', env_base, 'sampleCount', N, 'cfdCount', nCFD, 'matlabCount', nMat);

dataset = struct('X', X, 'Y', Y, 'parts', parts, 'split', splits, ...
    'caseId', ones(N, 1), 'meta', meta, 'config', cfg);
end

%% test_smooth_cycle_pipeline.m
currentScript = mfilename('fullpath');
rootDir  = fileparts(fileparts(fileparts(currentScript)));
urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
tcpBody = rigidBody('tcp');
tcpBody.Mass = 0;
tcpBody.Inertia = [0 0 0 0 0 0];
tcpJoint = rigidBodyJoint('tcp_fixed_joint', 'fixed');
setFixedTransform(tcpJoint, trvec2tform([0.280, -0.030, -0.0168]));
tcpBody.Joint = tcpJoint;
addBody(robot, tcpBody, 'link_004');

tEnd = 10.0;
dt = 0.01;
time = 0:dt:tEnd;
N = numel(time);

% 设计平滑包络因子 s(t) 及其导数
% 在 0~1.5s 平滑起步，8.5~10s 平滑收束
s_env = zeros(size(time));
sd_env = zeros(size(time));
sdd_env = zeros(size(time));

t_ramp = 1.5;
for k = 1:N
    t = time(k);
    if t < t_ramp
        tau = t / t_ramp;
        s_env(k)   = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sd_env(k)  = (30*tau^2 - 60*tau^3 + 30*tau^4) / t_ramp;
        sdd_env(k) = (60*tau - 180*tau^2 + 120*tau^3) / (t_ramp^2);
    elseif t <= tEnd - t_ramp
        s_env(k)   = 1.0;
        sd_env(k)  = 0.0;
        sdd_env(k) = 0.0;
    else
        tau = (tEnd - t) / t_ramp;
        s_env(k)   = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sd_env(k)  = -(30*tau^2 - 60*tau^3 + 30*tau^4) / t_ramp;
        sdd_env(k) = (60*tau - 180*tau^2 + 120*tau^3) / (t_ramp^2);
    end
end

% 基础协同振幅方程
omega = 2*pi / tEnd;
q1_base = deg2rad( 30.0 * sin(omega * time) );
q2_base = deg2rad(-48.0 + 12.0 * cos(omega * time) );
q3_base = deg2rad(-45.0 + 15.0 * sin(2 * omega * time) );

% 调制后保证首末速度严格为 0
% 基准初始待机姿态
q_init = [0.0, deg2rad(-36.0), deg2rad(-45.0)];

q1_t = q_init(1) + s_env .* (q1_base - q_init(1));
q2_t = q_init(2) + s_env .* (q2_base - q_init(2));
q3_t = q_init(3) + s_env .* (q3_base - q_init(3));

q_seq = [q1_t; q2_t; q3_t].';
qd_seq = zeros(N, 3);
qdd_seq = zeros(N, 3);
for j = 1:3
    qd_seq(:, j)  = gradient(q_seq(:, j), dt);
    qdd_seq(:, j) = gradient(qd_seq(:, j), dt);
end

fprintf('=== 优化后的启停物理平滑性检验 ===\n');
fprintf('  t = 0 初始关节角速度大小:   %.8f deg/s (必须绝对为 0)\n', norm(rad2deg(qd_seq(1,:))));
fprintf('  t = 10 终止关节角速度大小:  %.8f deg/s (必须绝对为 0)\n', norm(rad2deg(qd_seq(end,:))));
fprintf('  t = 0 初始关节角加速度大小: %.8f deg/s^2 (必须绝对为 0)\n', norm(rad2deg(qdd_seq(1,:))));
fprintf('  t = 10 终止关节角加速度大小: %.8f deg/s^2 (必须绝对为 0)\n', norm(rad2deg(qdd_seq(end,:))));
fprintf('  各关节在作业段的真实跨度:\n');
fprintf('    关节 1: 跨度 = %.1f°\n', range(rad2deg(q_seq(:,1))));
fprintf('    关节 2: 跨度 = %.1f°\n', range(rad2deg(q_seq(:,2))));
fprintf('    关节 3: 跨度 = %.1f°\n', range(rad2deg(q_seq(:,3))));

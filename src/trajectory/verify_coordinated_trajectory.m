%% verify_coordinated_trajectory.m
% 三段式作业轨迹的运动学与碰撞独立校验。
%
% 轨迹定义唯一存放于 src/dynamics/calc_joint_loads.m，本脚本直接读取其输出
% data/joint_loads_data.mat，避免轨迹公式在多处重复定义而失去同步。

currentScript = mfilename('fullpath');
rootDir  = fileparts(fileparts(fileparts(currentScript)));
urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
dataFile = fullfile(rootDir, 'data', 'joint_loads_data.mat');

if ~isfile(dataFile)
    fprintf('未找到轨迹数据，正在运行 calc_joint_loads.m 生成...\n');
    run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));
end

d = load(dataFile);
q_seq = d.q_seq;
qd_seq = d.qd_seq;
time = d.time;
traj = d.traj;
N = numel(time);
dt = time(2) - time(1);

robot = importrobot(urdfPath);
robot.DataFormat = 'row';

fprintf('=== 三段式作业轨迹独立校验 ===\n');
fprintf('0. 轨迹构成:\n');
fprintf('   %s\n', traj.form);
fprintf('   待机位形 [%.0f, %.0f, %.0f] deg -> 作业位形 [%.0f, %.0f, %.0f] deg\n', ...
    traj.q_stow_deg, traj.q_work_deg);
fprintf('   时段划分: 伸展 %.1fs | 往复 %.1fs | 收回 %.1fs | 合计 %.1fs (%d 帧)\n', ...
    traj.T_deploy_s, traj.T_work_s, traj.T_retract_s, traj.total_s, N);

% 1. 关节行程
fprintf('1. 关节运动跨度:\n');
axisName = {'基座水平扫掠', '肩部俯仰伸缩', '肘部屈伸对齐'};
for j = 1:3
    fprintf('   - 关节 %d (%s): [%.1f°, %.1f°] (跨度 %.1f°, 峰值 %.1f deg/s)\n', ...
        j, axisName{j}, min(rad2deg(q_seq(:,j))), max(rad2deg(q_seq(:,j))), ...
        range(rad2deg(q_seq(:,j))), max(abs(rad2deg(qd_seq(:,j)))));
end

% 2. 段边界速度连续性 (三段拼接不得出现速度跃变)
k1 = round(traj.T_deploy_s / dt) + 1;
k2 = round((traj.T_deploy_s + traj.T_work_s) / dt) + 1;
fprintf('2. 段边界速度连续性 (拼接处须为零，否则动网格会遇到速度跃变):\n');
fprintf('   - t = 0     起步速度:      %.3e deg/s\n', norm(rad2deg(qd_seq(1,:))));
fprintf('   - t = %4.1fs 伸展/往复边界: %.3e deg/s\n', traj.T_deploy_s, norm(rad2deg(qd_seq(k1,:))));
fprintf('   - t = %4.1fs 往复/收回边界: %.3e deg/s\n', ...
    traj.T_deploy_s + traj.T_work_s, norm(rad2deg(qd_seq(k2,:))));
fprintf('   - t = %4.1fs 终止速度:      %.3e deg/s\n', time(end), norm(rad2deg(qd_seq(end,:))));
fprintf('   - 首末位形偏差: %.3e deg (须严格回到待机位)\n', ...
    norm(rad2deg(q_seq(end,:) - q_seq(1,:))));

% 3. 末端工作空间
P = d.P_tcp_des;
fprintf('3. 末端工作空间立体覆盖:\n');
fprintf('   - X (前后方向扫掠): [%.3f, %.3f] m (跨度 %.1f cm)\n', ...
    min(P(:,1)), max(P(:,1)), range(P(:,1))*100);
fprintf('   - Y (侧向由近及远): [%.3f, %.3f] m (跨度 %.1f cm)\n', ...
    min(P(:,2)), max(P(:,2)), range(P(:,2))*100);
fprintf('   - Z (垂向深度起伏): [%.3f, %.3f] m (跨度 %.1f cm)\n', ...
    min(P(:,3)), max(P(:,3)), range(P(:,3))*100);

% 4. 独立重跑碰撞检测 (不复用 mat 中的间隙记录，逐帧重新判定)
% link_002 <-> link_004 为设计意图内的收拢贴合 (CAD 零位下转台与小臂表面贴靠)，
% 单独豁免；其余所有连杆对严格检验，任一相交即判定为真实机构干涉。
col_count = 0;
contact_frames = 0;
min_d_b3 = inf;
min_d_24 = inf;
worst_k  = 1;
for k = 1:N
    [~, distMat] = checkCollision(robot, q_seq(k, :), 'SkippedSelfCollisions', 'parent');
    
    realCol = false;
    for a = 1:size(distMat,1)
        for b = (a+1):size(distMat,2)
            if a == 2 && b == 4, continue; end   % 豁免贴合对
            if isnan(distMat(a,b)), realCol = true; end
        end
    end
    if realCol, col_count = col_count + 1; end
    if isnan(distMat(2,4)), contact_frames = contact_frames + 1; end
    
    if ~isnan(distMat(1,3)), min_d_b3 = min(min_d_b3, distMat(1,3)); end
    if ~isnan(distMat(2,4)) && distMat(2,4) < min_d_24
        min_d_24 = distMat(2,4);
        worst_k  = k;
    end
end

fprintf('4. 【逐帧刚体碰撞检测】:\n');
fprintf('   - 真实机构干涉帧数 (豁免对以外): %d / %d\n', col_count, N);
fprintf('   - base_link 与 link_003 最小净间距: %.2f mm\n', min_d_b3*1000);
fprintf('   - link_002  与 link_004 (豁免对): 收拢贴合 %d 帧\n', contact_frames);
fprintf('       分离后最小净间距 %.2f mm (出现于 t = %.2f s，即脱离贴合瞬间)\n', ...
    min_d_24*1000, time(worst_k));

% 作业段应保持稳定间隙，此处单独核验
kw1 = round(traj.T_deploy_s/dt)+1;
kw2 = round((traj.T_deploy_s+traj.T_work_s)/dt)+1;
d24_work = zeros(kw2-kw1+1,1);
for i = kw1:kw2
    [~, dm] = checkCollision(robot, q_seq(i,:), 'SkippedSelfCollisions', 'parent');
    d24_work(i-kw1+1) = dm(2,4);
end
fprintf('       作业段 (%.0f~%.0fs) 最小净间距 %.2f mm\n', ...
    traj.T_deploy_s, traj.T_deploy_s+traj.T_work_s, min(d24_work)*1000);

if col_count == 0
    fprintf('   => 通过: 全程无真实机构干涉\n');
else
    fprintf('   => 未通过: 存在 %d 帧真实干涉\n', col_count);
end

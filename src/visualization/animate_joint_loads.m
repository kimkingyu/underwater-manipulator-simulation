%% animate_joint_loads.m
% =========================================================================
% 3-DOF 水下机械臂 空间作业与【实时关节负载】三维动态回放
%
% 属于模块: src/visualization/
% 数据源: data/joint_loads_data.mat
% 成果输出: docs/animations/underarm_working_trajectory.gif
% =========================================================================

clear; clc;

% 自动定位工程根目录并装载路径
currentScript = mfilename('fullpath');
visDir        = fileparts(currentScript);
srcDir        = fileparts(visDir);
rootDir       = fileparts(srcDir);

addpath(genpath(fullfile(rootDir, 'src')));

dataFile = fullfile(rootDir, 'data', 'joint_loads_data.mat');
urdfPath = fullfile(rootDir, 'model', 'robot.urdf');
gifPath  = fullfile(rootDir, 'docs', 'animations', 'underarm_working_trajectory.gif');

if ~isfile(dataFile)
    fprintf('未找到负载数据，正在自动运行负载计算工程...\n');
    run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));
end

load(dataFile);
fprintf('成功载入时序负载数据 (时长: %.1f s, 步数: %d)\n', time(end), numel(time));

robot = importrobot(urdfPath);
robot.DataFormat = 'row';

% 创建高清视窗
fAnim = figure('Name', '水下机械臂作业轨迹与关节负载实时监测', ...
    'Color', [0.96, 0.97, 0.99], 'Position', [100, 60, 950, 700]);
ax = axes('Parent', fAnim);
hold(ax, 'on');

% 视框需容纳收拢停放姿态 (URDF 零位，臂贴于框架下方) 到完全伸展的全部行程，
% 并保留部分 AUV 框架作为空间参照，以体现机械臂的悬挂安装关系。
set(ax, 'XLim', [-0.32, 0.32], 'YLim', [0.30, 0.85], 'ZLim', [-0.75, -0.12]);
axis(ax, 'manual');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'GridColor', [0.75, 0.82, 0.90], 'GridAlpha', 0.6);
% 视角选取: 转台扫掠主要发生在 X 方向 (±0.14 m)，Y 向位移很小。
% 原 (135,25) 近似正对扫掠平面，60° 横扫在画面上被压成小幅摆动；
% (210,25) 侧对该平面，左右扫掠幅度与末端弧线均能完整展开。
view(ax, 210, 25);

xlabel(ax, 'X (前向) / m', 'FontWeight', 'bold');
ylabel(ax, 'Y (侧向) / m', 'FontWeight', 'bold');
zlabel(ax, 'Z (垂向) / m', 'FontWeight', 'bold');

% 绘制末端 3D 闭合工作轨迹线 (绿色虚线)
plot3(ax, P_tcp_des(:, 1), P_tcp_des(:, 2), P_tcp_des(:, 3), ...
    'Color', [0.1, 0.65, 0.25], 'LineWidth', 2.2, 'LineStyle', '--');

% 洋流方向标
quiver3(ax, -0.15, 0.45, -0.40, env.vc_world(1)*0.35, env.vc_world(2)*0.35, 0, ...
    'Color', [0 0.55 0.85], 'LineWidth', 3.0, 'MaxHeadSize', 0.9);
text(ax, -0.15, 0.45, -0.37, sprintf('洋流 (%.2f m/s)', env.current_speed), ...
    'Color', [0 0.45 0.75], 'FontWeight', 'bold', 'FontSize', 10);

% 动态末端当前点标
hPoint = scatter3(ax, P_tcp_des(1, 1), P_tcp_des(1, 2), P_tcp_des(1, 3), ...
    90, 'mo', 'filled');

% 三点布光
hLight1 = camlight('headlight');
light('Position', [1.5, 1.5, 2.0], 'Color', [1.0, 0.98, 0.95], 'Style', 'infinite');
light('Position', [-1.5, -1.0, 1.0], 'Color', [0.80, 0.90, 1.0], 'Style', 'infinite');
lighting(ax, 'gouraud');

% 初始化机械臂
hRobot = show(robot, q_seq(1, :), 'Parent', ax, 'PreservePlot', false, ...
    'Visuals', 'on', 'Collisions', 'off', 'FastUpdate', true);

patches = findobj(ax, 'Type', 'Patch');
for p = 1:numel(patches)
    set(patches(p), 'AmbientStrength', 0.55, 'DiffuseStrength', 0.70, 'SpecularStrength', 0.35);
end

fps = 20;
stepSkip = round((numel(time) / time(end)) / fps);
frameIndices = 1:stepSkip:numel(time);
gifDelay = 1.0 / fps;

fprintf('开始播放 3D 机械臂空间作业与负载监测动画...\n');

for f = 1:numel(frameIndices)
    idx = frameIndices(f);
    t_k = time(idx);
    q_k = q_seq(idx, :);
    tau_k = tau_total(idx, :);
    p_tcp_k = P_tcp_des(idx, :);
    
    show(robot, q_k, 'Parent', ax, 'PreservePlot', false, ...
        'Visuals', 'on', 'Collisions', 'off', 'FastUpdate', true);
    
    set(hPoint, 'XData', p_tcp_k(1), 'YData', p_tcp_k(2), 'ZData', p_tcp_k(3));
    camlight(hLight1, 'headlight');
    
    title(ax, sprintf(['水下空间作业轨迹跟踪与实时关节负载监测 | t = %.2f s / %.1f s\n' ...
                       '关节角: [q1: %.1f°, q2: %.1f°, q3: %.1f°]\n' ...
                       '实时驱动负载: [\\tau_1: %.2f, \\tau_2: %.2f, \\tau_3: %.2f] N\\cdot m'], ...
        t_k, time(end), ...
        rad2deg(q_k(1)), rad2deg(q_k(2)), rad2deg(q_k(3)), ...
        tau_k(1), tau_k(2), tau_k(3)), ...
        'FontSize', 11, 'Color', [0.1, 0.2, 0.35]);
    
    drawnow;
    
    frameImg = getframe(fAnim);
    [imind, cm] = rgb2ind(frameImg.cdata, 256);
    if f == 1
        imwrite(imind, cm, gifPath, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
    else
        imwrite(imind, cm, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end
    
    pause(0.04);
end

fprintf('作业回放结束！高清动图已保存至: %s\n', gifPath);

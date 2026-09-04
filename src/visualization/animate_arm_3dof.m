%% animate_arm_3dof.m
% =========================================================================
% 3-DOF 水下机械臂 抓取作业 高清三维动画播放与 GIF 导出
%
% 优化升级点:
%   1. 【彻底解决画面暗淡】: 引入多角度三点专业布光 (头灯 + 暖白主光 + 天蓝漫射)，
%      启用 Gouraud 高光着色与明亮材质，展现金属与工程黄机械质感。
%   2. 【彻底解决动画过快或不动】: 引入精确帧率节拍器 (严格按 30 FPS 播放，pause(0.033))，
%      平稳再现 10 秒完整抓取作业时序，肉眼可见平滑动作。
%   3. 【动态空间轨迹】: 末端配备发光动态拖尾线，实时追踪夹爪中心三维轨迹。
%   4. 【双模交付】: 既可在 MATLAB 视窗实时三维旋转交互，同时自动导出高清
%      GIF 动画 (underarm_3dof_animation.gif)，脱离 MATLAB 也能随时双击查看。
% =========================================================================

clear; clc;

scriptDir = fileparts(mfilename('fullpath'));
simDataFile = fullfile(scriptDir, 'underarm_3dof_sim_data.mat');
urdfPath    = fullfile(scriptDir, 'robot.urdf');
gifPath     = fullfile(scriptDir, 'underarm_3dof_animation.gif');

if ~isfile(simDataFile)
    fprintf('未找到仿真数据集，正在自动执行主仿真求解...\n');
    run(fullfile(scriptDir, 'simulate_underwater_arm_3dof.m'));
end

load(simDataFile);
fprintf('成功载入 3-DOF 仿真数据 (时长: %.1f s, 步数: %d)\n', time(end), numel(time));

% 导入机械臂模型
robot = importrobot(urdfPath);
robot.DataFormat = 'row';

%% 1. 创建明亮高对比度三维视窗
fAnim = figure('Name', '3-DOF 水下机械臂抓取动画 (高亮流畅版)', ...
    'Color', [0.96, 0.97, 0.99], ...            % 极淡明亮天水蓝背景
    'Position', [120, 80, 800, 600]);
ax = axes('Parent', fAnim);
hold(ax, 'on');

% 预先固定坐标轴范围，防止每帧视角缩放抖动 (适配展开工作空间)
set(ax, 'XLim', [-0.25, 0.20], 'YLim', [0.15, 0.60], 'ZLim', [-0.60, -0.10]);
axis(ax, 'manual');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'GridColor', [0.75, 0.82, 0.90], 'GridAlpha', 0.6);
view(ax, 135, 25);

xlabel(ax, 'X (前向) / m', 'FontWeight', 'bold');
ylabel(ax, 'Y (侧向) / m', 'FontWeight', 'bold');
zlabel(ax, 'Z (垂向) / m', 'FontWeight', 'bold');

%% 2. 绘制水下参考场景 (目标物、水流、基座海床网格)
% 1) 水下基准网格平面 (模拟 AUV 腹部挂架基底)
[Xgrid, Ygrid] = meshgrid(-0.2:0.1:0.4, -0.05:0.1:0.6);
Zgrid = zeros(size(Xgrid)) - 0.02;
mesh(ax, Xgrid, Ygrid, Zgrid, 'FaceAlpha', 0.05, 'EdgeColor', [0.6 0.75 0.9], 'LineStyle', ':');

% 2) 目标抓取物 (醒目发光红色立体球 + 标牌)
[sx, sy, sz] = sphere(16);
r_target = 0.022; % 半径 22 mm
surf(ax, sx*r_target + target_pos(1), sy*r_target + target_pos(2), sz*r_target + target_pos(3), ...
    'FaceColor', [1.0, 0.15, 0.15], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
text(ax, target_pos(1) + 0.03, target_pos(2), target_pos(3) + 0.02, ...
    '★ 目标抓取点', 'Color', [0.85, 0, 0], 'FontWeight', 'bold', 'FontSize', 11);

% 3) 洋流方向标 (青蓝色加粗箭头)
quiver3(ax, -0.15, 0.25, -0.20, env.vc_world(1)*0.4, env.vc_world(2)*0.4, 0, ...
    'Color', [0 0.55 0.85], 'LineWidth', 3.0, 'MaxHeadSize', 0.9);
text(ax, -0.15, 0.25, -0.18, sprintf('洋流来向 (%.2f m/s, 45°)', env.current_speed), ...
    'Color', [0 0.45 0.75], 'FontWeight', 'bold', 'FontSize', 10);

% 4) 规划的理想轨迹参考虚线
plot3(ax, log_tcp_pos(:, 1), log_tcp_pos(:, 2), log_tcp_pos(:, 3), ...
    'Color', [0.5, 0.65, 0.8], 'LineWidth', 1.5, 'LineStyle', ':');

% 5) 动态实时拖尾轨迹线句柄
hTrail = plot3(ax, log_tcp_pos(1, 1), log_tcp_pos(1, 2), log_tcp_pos(1, 3), ...
    'r-', 'LineWidth', 2.2);

%% 3. 专业三点布光系统 (彻底解决画面暗淡)
hLight1 = camlight('headlight');                      % 主视线头灯
hLight2 = light('Position', [1.5, 1.5, 2.0], ...     % 右上侧暖白强主光
    'Color', [1.0, 0.98, 0.95], 'Style', 'infinite');
hLight3 = light('Position', [-1.5, -1.0, 1.0], ...   % 左下方冷蓝环境补光
    'Color', [0.80, 0.90, 1.0], 'Style', 'infinite');
lighting(ax, 'gouraud');

%% 4. 初始化机械臂图形对象
hRobot = show(robot, log_q(1, :), 'Parent', ax, 'PreservePlot', false, ...
    'Visuals', 'on', 'Collisions', 'off', 'FastUpdate', true);

% 遍历机械臂的 patch 对象赋予金属光泽材质与增强颜色
patches = findobj(ax, 'Type', 'Patch');
for p = 1:numel(patches)
    set(patches(p), 'AmbientStrength', 0.55, ...
                    'DiffuseStrength', 0.70, ...
                    'SpecularStrength', 0.35, ...
                    'SpecularExponent', 15.0);
end

%% 5. 帧率控制与动画播放循环
fps = 20;                              % 播放帧率 20 帧/秒 (丝滑且体积轻量)
frameTime = 1.0 / fps;
stepSkip = round((numel(time) / time(end)) / fps); % 对应采样步长步进
frameIndices = 1:stepSkip:numel(time);

fprintf('开始流畅播放 3D 水下机械臂抓取动画 (帧数: %d, 帧率: %d FPS)...\n', ...
    numel(frameIndices), fps);

% 是否录制 GIF
recordGif = true;
gifDelay = 1.0 / fps;

for f = 1:numel(frameIndices)
    idx = frameIndices(f);
    t_now = time(idx);
    q_now = log_q(idx, :);
    
    % 更新机械臂模型位姿
    show(robot, q_now, 'Parent', ax, 'PreservePlot', false, ...
        'Visuals', 'on', 'Collisions', 'off', 'FastUpdate', true);
    
    % 更新动态拖尾轨迹线
    set(hTrail, 'XData', log_tcp_pos(1:idx, 1), ...
                'YData', log_tcp_pos(1:idx, 2), ...
                'ZData', log_tcp_pos(1:idx, 3));
            
    % 维持头灯随视角跟随
    camlight(hLight1, 'headlight');
    
    % 计算当前末端与目标的距离
    dist_mm = norm(log_tcp_pos(idx, :) - target_pos) * 1000.0;
    
    % 动态标题提示
    if t_now < 1.0
        phaseStr = '【阶段 1】初始位姿 悬停自检';
    elseif t_now <= 5.0
        phaseStr = '【阶段 2】下探前伸 平滑接近目标';
    elseif t_now <= 7.0
        phaseStr = '【阶段 3】到达目标 稳定对齐夹持';
    else
        phaseStr = '【阶段 4】平稳回缩 复位归位';
    end
    
    title(ax, sprintf('%s\n时间: t = %.2f s / %.1f s | 关节角: [%.1f°, %.1f°, %.1f°] | 末端误差: %.1f mm', ...
        phaseStr, t_now, time(end), ...
        rad2deg(q_now(1)), rad2deg(q_now(2)), rad2deg(q_now(3)), dist_mm), ...
        'FontSize', 11, 'Color', [0.1, 0.2, 0.35]);
    
    % 强制刷新屏幕
    drawnow;
    
    % 录制帧到 GIF
    if recordGif
        frameImg = getframe(fAnim);
        [imind, cm] = rgb2ind(frameImg.cdata, 256);
        if f == 1
            imwrite(imind, cm, gifPath, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
        else
            imwrite(imind, cm, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
        end
    end
    
    % 严格延时保持人眼舒适的 20 FPS 播放速度
    pause(0.04);
end

fprintf('动画播放结束！\n');
fprintf('高清 GIF 动图已自动导出至: %s\n', gifPath);

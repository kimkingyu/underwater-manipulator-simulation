%% main.m
% =========================================================================
% 水下机器人-机械臂系统 (UVMS) 动力学与水动力负载仿真 一键启动总入口
%
% 运行方式: 在 MATLAB 命令行中直接输入 main 即可
% =========================================================================

clear; clc; close all;

% 1. 设置工程根目录与搜索路径
rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(rootDir, 'src')));
addpath(genpath(fullfile(rootDir, 'model')));

fprintf('========================================================================\n');
fprintf('   Underwater Vehicle-Manipulator System (UVMS) Simulation Toolbox\n');
fprintf('          水下机器人-机械臂系统动力学与水下负载仿真主控台\n');
fprintf('========================================================================\n\n');

fprintf('工程目录: %s\n', rootDir);
fprintf('MATLAB 版本: %s\n', version);

% 2. 检查模型资产完整性
urdfFile = fullfile(rootDir, 'model', 'robot.urdf');
if ~isfile(urdfFile)
    error('未检测到核心 URDF 模型文件，请检查 model/ 目录！');
end
fprintf('核心模型: model/robot.urdf (就绪)\n\n');

% 3. 执行核心动力学负载计算
fprintf('>>> 正在启动三轴协同作业轨迹与关节多源物理负载精确解析...\n\n');
run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));

% 4. 交互式启动 3D 动画回放
fprintf('\n>>> 负载解算完成！正在启动带实时动力学负载仪表的三维作业动画回放...\n\n');
run(fullfile(rootDir, 'src', 'visualization', 'animate_joint_loads.m'));

fprintf('========================================================================\n');
fprintf('仿真与可视化全部执行圆满完成！\n');
fprintf('  - 核心分析图表: docs/figures/joint_loads_dashboard.png\n');
fprintf('  - 高清动画动图: docs/animations/underarm_working_trajectory.gif\n');
fprintf('  - 完整时序数据: data/joint_loads_data.mat\n');
fprintf('========================================================================\n');

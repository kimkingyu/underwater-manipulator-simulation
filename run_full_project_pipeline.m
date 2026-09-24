% run_full_project_pipeline.m
% =========================================================================
% 3自由度水下机械臂系统 全流程自动化闭环集成流水线
% 一键运行: 动力学解算 -> 轨迹校验 -> 碰撞检测 -> UDF预编译 -> 小样本代理模型训练 -> CFD对齐评估 -> 全景出图
% =========================================================================

clc; clear; close all;
t_pipeline_start = tic;

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(rootDir, 'src')));

fprintf('========================================================================\n');
fprintf('   3-DOF 水下机械臂仿真项目 全流程自动化闭环集成流水线启动\n');
fprintf('========================================================================\n\n');

%% [1/7] 动力学基准解算与全周期多源物理负载提取
fprintf('>>> [1/7] 执行动力学全周期解算 (src/dynamics/calc_joint_loads.m)...\n');
t1 = tic;
run(fullfile(rootDir, 'src', 'dynamics', 'calc_joint_loads.m'));
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t1));

%% [2/7] 轨迹自洽性与分段平滑连续性独立检验
fprintf('>>> [2/7] 执行三段式轨迹平滑性独立检验 (src/trajectory/test_smooth_cycle_pipeline.m)...\n');
t2 = tic;
run(fullfile(rootDir, 'src', 'trajectory', 'test_smooth_cycle_pipeline.m'));
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t2));

%% [3/7] 逐帧几何刚体接触碰撞检测与末端工作空间校验
fprintf('>>> [3/7] 执行 1001 帧机构碰撞检测与空间包络校验 (src/trajectory/verify_coordinated_trajectory.m)...\n');
t3 = tic;
run(fullfile(rootDir, 'src', 'trajectory', 'verify_coordinated_trajectory.m'));
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t3));

%% [4/7] CFD 动网格 UDF 编译参数与工况快照固化 (同步更新 Workbench 案例目录)
fprintf('>>> [4/7] 固化 CFD 动网格 UDF 编译头文件与工况快照 (src/cfd/prepare_fluent_udf.m)...\n');
t4 = tic;
prepare_fluent_udf();
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t4));

%% [5/7] 训练无实物小样本水动力代理模型 (以实测 CFD 数据为主 + 高拟合区 MATLAB 数据融合)
fprintf('>>> [5/7] 训练小样本水动力代理模型 (src/surrogate/train_small_sample_surrogate.m)...\n');
t5 = tic;
train_small_sample_surrogate();
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t5));

%% [6/7] Fluent 3D CFD 原生载荷与 MATLAB 孪生标定对比评估
fprintf('>>> [6/7] 运行 Fluent 3D CFD 与 MATLAB 对齐误差评估 (cfd/run_10s_cad_aligned_comparison.m)...\n');
t6 = tic;
run(fullfile(rootDir, 'cfd', 'run_10s_cad_aligned_comparison.m'));
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t6));

%% [7/7] 生成 0~5.0 秒全景动力学与电机容量裕度评估大图
fprintf('>>> [7/7] 导出 0~5.0 秒科研全景 4 联对比图 (cfd/plot_5s_full_dynamics_comparison.m)...\n');
t7 = tic;
run(fullfile(rootDir, 'cfd', 'plot_5s_full_dynamics_comparison.m'));
fprintf('    [完成] 耗时: %.2f 秒\n\n', toc(t7));

%% 全流水线总结报告
totTime = toc(t_pipeline_start);
fprintf('========================================================================\n');
fprintf('   ★ 全流程流水线自动化执行完毕！总计耗时: %.2f 秒 (%.2f 分钟)\n', totTime, totTime/60);
fprintf('   核心交付成果产物清单:\n');
fprintf('     1. [动力学总集] data/joint_loads_data.mat (1001 帧, 100 Hz)\n');
fprintf('     2. [UDF 头文件] src/cfd/arm_motion_config.h 及 cfd/1_files/dp0/FFF/Fluent/arm_motion_config.h\n');
fprintf('     3. [小样本模型] data/hydro_surrogate/full/model.mat (CFD 主导小样本高斯过程模型)\n');
fprintf('     4. [CFD 对齐图] docs/figures/fluent_vs_matlab_cad_aligned.png\n');
fprintf('     5. [全景全貌图] docs/figures/fluent_vs_matlab_5s_full_cycle.png\n');
fprintf('     6. [负载面板图] docs/figures/joint_loads_dashboard.png\n');
fprintf('========================================================================\n');

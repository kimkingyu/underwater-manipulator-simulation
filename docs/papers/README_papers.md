# 水下机器人-机械臂系统 (UVMS) 洋流动力学与抓取轨迹权威文献清单

### 1. A Robust Nonlinear Model Predictive Control Approach for Underwater Vehicle-Manipulator Systems in the Presence of Ocean Currents
- **作者/出处**: S. Heshmati-Alamdari, et al. (IEEE Transactions on Control Systems Technology)
- **本地文件**: `01_Heshmati2018_UVMS_Ocean_Currents_NMPC.pdf`
- **核心内容与参数依据**: 【洋流建模与NMPC抗流轨迹控制】详细推导了洋流相对速度方程 (v_rel = v - v_c)、水平与垂向迎流角参数化公式，以及在0.2~0.5 m/s洋流下UVMS模型预测控制抓取轨迹跟踪。

### 2. Autonomous Underwater Manipulation: Trajectory Planning and Control Strategies
- **作者/出处**: D. Youakim, et al.
- **本地文件**: `02_Youakim2020_Autonomous_Underwater_Manipulation_Planning.pdf`
- **核心内容与参数依据**: 【水下机械臂自主抓取与轨迹规划】系统阐述水下电动机械臂在浮动基座下的笛卡尔空间平滑抓取轨迹规划、逆运动学求解、反冲载荷补偿。

### 3. Coordinated Motion Planning and Trajectory Tracking for Underwater Vehicle-Manipulator Systems
- **作者/出处**: Y. Zhang, et al.
- **本地文件**: `03_Zhang2022_Coordinated_Motion_Planning_UVMS.pdf`
- **核心内容与参数依据**: 【浮动基座与机械臂协同运动规划】研究浮动基座AUV与多自由度机械臂的动力学耦合解耦、推力器与关节协调抗扰分配。

### 4. Adaptive Dynamic Control of Underwater Vehicle-Manipulator Systems under External Ocean Disturbances
- **作者/出处**: M. Wang, et al.
- **本地文件**: `04_Wang2023_Adaptive_Control_UVMS_Disturbances.pdf`
- **核心内容与参数依据**: 【复杂洋流扰动下自适应动力学控制】针对不可测慢变洋流与波浪流体阻力，设计自适应前馈抵消水动力扰动，保持机械臂末端毫米级抓取精度。

### 5. Whole-Body Motion Planning and Control for Floating-Base Underwater Manipulators
- **作者/出处**: J. Kim, et al.
- **本地文件**: `05_Kim2021_Whole_Body_Planning_Floating_Manipulator.pdf`
- **核心内容与参数依据**: 【浮动基座水下机械臂全身动力学与工作空间规划】全面分析水下机械臂受水阻、浮力矩、基座反冲时的奇异点规避与最优化工作空间规划。

### 6. Underwater Manipulators: A Review & Closing the Gap in Industrial Robotics
- **作者/出处**: S. Sivčev, J. Coleman, E. Omerdić, G. Dooly, D. Toal (Ocean Engineering / Diving-ROV Specialists)
- **本地文件**: `06_Sivcev2018_Closing_Gap_Underwater_Manipulators.pdf`
- **核心内容与参数依据**: 【水下机械臂水动力建模、CFD及试验权威综述】系统综述商用与科研水下机械臂的物理几何参数、钝体迎流阻力系数折减、尾流遮蔽效应，以及基于 CFD 水槽仿真反标定解析模型的方法学规范。

### 7. Task-Priority Control of Underwater Vehicle Manipulator Systems with Current Disturbance Compensation
- **作者/出处**: P. Cataldi, et al. (arXiv / IEEE)
- **本地文件**: `07_Cataldi2019_Task_Priority_UVMS_Current_Disturbance.pdf`
- **核心内容与参数依据**: 【洋流干扰补偿与任务优先级控制】研究未知多向洋流（不同流速与流向角）对 UVMS 机械臂各关节力矩的扰动映射，建立基于相对速度动力学方程的任务空间解耦控制律。

### 8. A Bimanual Teleoperation Framework for Light Duty Underwater Vehicle-Manipulator Systems
- **作者/出处**: Y. Guan, et al. (arXiv 2024)
- **本地文件**: `08_Guan2024_Light_Duty_UVMS_Hydrodynamics_Teleoperation.pdf`
- **核心内容与参数依据**: 【轻型电动水下机械臂运动学与水动力载荷求解】给出多自由度水下电动臂在来流环境下的雅可比映射公式、末端抓取阻力与关节驱动电机关联特性。

### 9. Design, Kinematics, and Deployment of a Continuum Underwater Vehicle-Manipulator System
- **作者/出处**: Y. Guan, et al. (arXiv 2023)
- **本地文件**: `09_Guan2023_Design_Kinematics_Deployment_UVMS.pdf`
- **核心内容与参数依据**: 【水下机械臂展开部署与流场交互动力学】深入分析机械臂从紧凑待机收拢态平滑展开至作业位形期间，流阻力矩和迎流有效面积的变化规律与碰撞规避。

### 10. Kinematic and Dynamic Modeling of Free-Floating Mobile Manipulators Using Dual Quaternion Algebra
- **作者/出处**: N. Fonseca, et al. (arXiv 2020)
- **本地文件**: `10_Fonseca2020_Modeling_Floating_Manipulators_Dual_Quaternions.pdf`
- **核心内容与参数依据**: 【浮动基座机械臂对偶四元数动力学与扰动建模】采用对偶四元数高效建立含外力矩、流体浮力与动水压阻力的非线性多体动力学模型。

### 11. A Tube-based MPC Scheme for Interaction Control of Underwater Vehicle Manipulator Systems
- **作者/出处**: S. Heshmati-Alamdari, et al. (arXiv / IEEE CDC)
- **本地文件**: `11_Heshmati2018_Tube_MPC_UVMS_Ocean_Currents.pdf`
- **核心内容与参数依据**: 【多向洋流不确定性下的管道鲁棒模型预测控制】系统建模了洋流速度（0~0.5 m/s）、来流方向突变以及水动力参数摄动对机械臂轨迹跟踪性能的影响，提供力矩鲁棒边界证明。



# 水下 3-DOF 机械臂 ANSYS Fluent 3D CFD 动网格流固耦合仿真与 MATLAB 动力学标定全过程技术报告

---

## 摘要

本报告对安装于深海作业型遥控无人潜水器（ROV）船首的水下 3-DOF 电动机械臂在复杂三维洋流环境下的高保真流固耦合（Fluid-Structure Interaction, FSI）动网格数值仿真与动力学标定全流程进行系统性复盘。基于 ANSYS Fluent 2024 R2 商业求解器（18 核并行计算环境），突破了复杂 CAD 真实工业装配体的微米级缝隙网格畸变、大水槽与细长杆件跨尺度网格规模爆炸、重叠网格交界面插值孤儿单元及多轴串联机械臂运动学 UDF 翻折等 5 大工程技术瓶颈，建立了规模控制在 41.5 万单元的高精度多层刚体重叠网格（Overset Grid）计算架构。针对洋流流速（$0.00\sim 1.00\text{ m/s}$）与来流方位角（$0^\circ\sim 90^\circ$）构成的 52 组全包络工况矩阵进行了真实物理动网格瞬态求解，提取各关节流体阻力矩并对齐验证了 MATLAB 动力学解析模型与小样本高斯过程（GPR）水动力代理模型。全量 52 组实测比对证明：经 CFD 标定与补偿重构后的 MATLAB 动力学模型三轴综合平均绝对偏差（MAE）仅为 $0.0473\text{ N}\cdot\text{m}$，平均相对误差（MAPE）控制在 $3.28\%$，满量程归一化误差（NRMSE）为 $0.53\%\text{ FS}$，相比传统未修正的理论经验模型将系统误差削减了 $95.64\%$，为机械臂在复杂深海洋流扰动下的轨迹规划与自适应抗扰控制提供了高保真物理基准。

---

## 一、 工程背景与几何/物理模型定义

### 1.1 机械臂机构与 CAD 几何特征
* **机构构型**：3-DOF 串联机械臂（J1 转台水平回转偏航、J2 肩部俯仰承重、J3 肘部伸展俯仰 + 末端三指机械抓爪）；
* **几何来源**：基于 SolidWorks 工业级完整装配体导出的 `4.STEP` 标准实体，包含母船 ROV 船首骨架及机械臂 4 级运动部件（基座回转塔 `lk1`、大臂梁 `lk2`、小臂主体与夹爪机构 `lk3w1` ~ `lk3w4`），总计 23 个实体部件；
* **关键结构特征与迎水面积**：
  * **连杆 1 (`link_002` / `lk1`)**：基座偏航回转柱，包络尺寸 $0.1469\text{ m} \times 0.1469\text{ m} \times 0.2448\text{ m}$，STL 表面实测迎水投影面积为 $0.0226\text{ m}^2$；
  * **连杆 2 (`link_003` / `lk2`)**：大臂 U 型双耳叉架，包络尺寸 $0.1200\text{ m} \times 0.1450\text{ m} \times 0.4910\text{ m}$，中间带有工业级镂空通流槽，STL 实测前向投影面积 $0.0365\text{ m}^2$（相对外包络面积缩减率达 $17.4\%$）；
  * **连杆 3 (`link_004` / `lk3`)**：前臂与末端执行器，包络尺寸 $0.1060\text{ m} \times 0.1566\text{ m} \times 0.4310\text{ m}$，STL 实测前向投影面积 $0.0273\text{ m}^2$。

### 1.2 流体计算域与流体物性
* **外流场计算域尺寸**：采用真实开阔水槽环境，长 $3.12\text{ m}$（$X \in [-2.31, +0.81]\text{ m}$）、宽 $3.25\text{ m}$（$Z \in [-2.44, +0.81]\text{ m}$）、高 $2.49\text{ m}$（$Y \in [-0.05, +2.44]\text{ m}$），水槽总体积约为 $25.2\text{ m}^3$；
* **流体介质物性**：标准深海高盐度海水，密度 $\rho = 1025.0\text{ kg/m}^3$，动力黏度 $\mu = 0.00108\text{ Pa}\cdot\text{s}$（运动黏度 $\nu = 1.054 \times 10^{-6}\text{ m}^2/\text{s}$）；
* **边界条件**：
  * 来流入口（`inlet`）：速度入口（Velocity Inlet），支持流速大小 $V_c \in [0.0, 1.0]\text{ m/s}$ 与水平入流偏角 $\psi \in [0^\circ, 90^\circ]$，设置湍流强度 $I = 5\%$、湍流黏度比 $\mu_t/\mu = 10$；
  * 下游出口（`outlet`）：压力出口（Pressure Outlet），静压设为 $0\text{ Pa}$（表压）；
  * 水槽四周与底面：滑移壁面（Specified Shear, Zero Shear Stress），消除槽壁附面层阻塞效应；
  * ROV 船身与机械臂各连杆外壁：无滑移运动固壁（No-Slip Wall），挂接动网格 UDF 边界。

### 1.3 运动学测试轨迹
全周期 $10.0\text{ 秒}$ 典型水下作业轨迹（三段式连续运动）：
1. **$0.0 \sim 2.0\text{ s}$（展出阶段）**：机械臂由零位折叠状态平稳展开至作业预备姿态；
2. **$2.0 \sim 8.0\text{ s}$（往复扫掠阶段）**：各关节在 $\pm 30^\circ$ 范围内进行正弦往复水下扫掠作业，最大角速度达 $31.4^\circ/\text{s} \sim 43.2^\circ/\text{s}$；
3. **$8.0 \sim 10.0\text{ s}$（折叠收回阶段）**：机械臂平稳制动并复位至船首安全锁紧位形。

---

## 二、 数值求解器与重叠网格架构配置

### 2.1 求解器核心算法
* **求解器类型**：ANSYS Fluent 2024 R2 3D 压强基瞬态求解器（Pressure-Based Transient Solver）；
* **并行架构**：18 核 CPU 硬件并行（基于 MS-MPI 进程通信，通过 `actual_compute_nodes: 18` 验签）；
* **流场与压力离散格式**：
  * 压力速度耦合算法：PISO（Pressure-Implicit with Splitting of Operators），开启近邻校正与倾斜校正（Skewness Correction = 1），适配复杂三维动网格微元畸变；
  * 压力插值：PRESTO!（针对强旋流与大曲率曲面流动分离具有高数值稳定性）；
  * 动量方程：二阶迎风格式（Second Order Upwind）；
  * 瞬态时间推进：一阶隐式格式，基准时间步长 $\Delta t = 0.01\text{ s}$，单步内最大迭代步数设为 20~35 步，收敛残差标准设置为 $10^{-4}$。
* **湍流模型**：SST $k-\omega$ 双方程模型（剪切应力传输模型），在近壁区利用低雷诺数形式直接解析附面层流动分离，在外流区自动过渡至 $k-\epsilon$ 模型以降低对自由来流湍流参数的敏感性。

### 2.2 41.5 万四层刚体重叠网格（Overset Grid System）
为了兼顾机械臂大幅度多轴相对运动、大水槽流场捕捉与计算效率，弃用会导致网格拉伸倒扣的单域变形重构法，构建了四层刚体重叠网格系统：
1. **背景静止域（`background_fluid`）**：包裹 ROV 船身与大水槽，采用非结构四面体/多面体网格，单元数控制在 **213,420 单元**；在机械臂活动立体包络区设置过渡加密环（单元尺寸 $6\sim 8\text{ mm}$）；
2. **J1 转台子网格（`fluid_lk1`）**：包裹转台 `lk1`，柱状加密域，单元数 **28,950 单元**；
3. **J2 大臂子网格（`fluid_lk2`）**：包裹大臂 `lk2`，随 J2 轴旋转俯仰，单元数 **39,410 单元**；
4. **J3 小臂与夹爪子网格（`fluid_lk3`）**：紧贴包裹小臂梁与三指爪，随 J3 轴做合成复合运动，单元数 **133,220 单元**；
* **系统总网格量**：**41.5 万单元（415,000 cells）**。单步计算耗时仅 **4~6 秒**，相比单域超细网格（>2500 万单元）节省了 **98%** 的算力与内存开销。

---

## 三、 动网格与运动学 UDF 核心算法

### 3.1 运动学前向递推合成算法
机械臂为串联开链多刚体机构，小臂末端执行器的绝对空间运动是基座偏航回转、大臂俯仰与肘部伸展的非线性矢量叠加。采用原生 C 语言编译为动态库 `libudf_arm_test.dll`，在 `DEFINE_CG_MOTION` 宏中实现：
1. **全局绝对位姿递推**：在每个微步内部，基于机构 DH 参数及瞬时关节角速度 $[\dot{q}_1, \dot{q}_2, \dot{q}_3]^T$，通过齐次变换矩阵递推计算世界坐标系下的绝对旋转角速度矢量：
   $$\vec{\omega}_{\text{abs}, 1} = [0, \dot{q}_1, 0]^T$$
   $$\vec{\omega}_{\text{abs}, 2} = \vec{\omega}_{\text{abs}, 1} + \mathbf{R}_1(q_1) [-\dot{q}_2, 0, 0]^T$$
   $$\vec{\omega}_{\text{abs}, 3} = \vec{\omega}_{\text{abs}, 2} + \mathbf{R}_1(q_1) \mathbf{R}_2(q_2) [-\dot{q}_3, 0, 0]^T$$
2. **刚体质心平移线速度合成**：
   $$\vec{v}_{\text{cg}, i} = \vec{v}_{\text{joint}, i} + \vec{\omega}_{\text{abs}, i} \times (\vec{r}_{\text{cg}, i} - \vec{p}_{\text{joint}, i})$$
   消除了由于相对坐标系定义混乱而导致的网格几何撕裂问题。

### 3.2 关节水动力矩投影提取机制
为获取各关节旋转轴线方向上的净流体阻力矩，在 Fluent 求解循环中，通过 Surface Report 提取湿表面上的瞬时压力力矩 $\vec{M}_p$ 与黏性剪切力矩 $\vec{M}_v$：
$$\tau_{\text{hydro}, i} = (\vec{M}_{p, i} + \vec{M}_{v, i}) \cdot \vec{n}_{\text{axis}, i}$$
其中 $\vec{n}_{\text{axis}, i}$ 为实时空间转轴单位方向向量，原点取为对应连杆瞬时关节旋转中心 $\vec{p}_{\text{joint}, i}$。计算结果与 Fluent 原生导出的文本报表（如 `native_000001_j1.txt`）逐项比对，误差在 $10^{-10}\text{ N}\cdot\text{m}$ 机器精度量级。

---

## 四、 网格划分攻坚与五大关键问题排查复盘

在网格生成与数值计算推进中，先后攻克了 5 大底层工程问题（详见 [docs/mesh_generation_issues_and_solutions.md](docs/mesh_generation_issues_and_solutions.md)）：

1. **CAD 微米级几何狭缝导致网格奇异畸变**：
   * *问题*：CAD 保留制造配合公差，在销轴铰接处存在 14 处 $0.3\sim 0.755\ \mu\text{m}$ 的极微间隙，导致生成厚度数微米、长宽比数千的奇异扁平单元（Sliver Cells），最小单元体积暴跌至 $3.12 \times 10^{-14}\text{ m}^3$，网格最小正交质量低至 $1.25\times 10^{-14}$，计算瞬时报 `Floating Point Exception`；
   * *对策*：利用 OpenCASCADE 编写去特征脚本消除沉头孔与倒角，封闭非功能性微隙，保留通流间隙 $\ge 1.5\text{ mm}$，彻底消除奇异几何源。
2. **大水槽与微连杆尺差导致的网格规模爆炸**：
   * *问题*：$25\text{ m}^3$ 外流场若全局加密至附面层精度，网格量达 2500 万，直接导致 128G 工作站内存溢出（OOM）；
   * *对策*：重构成背景域与三级刚体子域解耦的四层重叠网格体系，网格总量大幅压缩至 41.5 万。
3. **重叠网格交界面插值孤儿网格（Orphan Cells）**：
   * *问题*：子网格边界尺寸（$3\sim 5\text{ mm}$）与背景粗网格（$15\sim 20\text{ mm}$）相差过大，产生 1073 个孤儿单元，引发压力震荡；
   * *对策*：在背景网格围绕机械臂扫掠空间建立 $6\sim 8\text{ mm}$ 加密过渡环，并将重叠网格优先级固化为 `grid-priorities arm_interface 0 3 2 1`，将插值孤儿限制在非流体死区内。
4. **串联刚体运动在 UDF 中的局部坐标翻折**：
   * *问题*：在动态网格区域若采用局部相对角速度，旋转后小臂出现空间倒扣，出现负体积（Negative cell volume）；
   * *对策*：重写前向运动学全局齐次矩阵绝对速度合成算法，并在每个步长挂接 `check_volume` 监控，确保流体体积恒为正值。
5. **强流冲击下的瞬态求解发散**：
   * *问题*：$1.0\text{ m/s}$ 强流下，流动分离涡剧烈脱落导致局部库朗数（CFL）过高；
   * *对策*：设置前 5 步渐变缓加初值策略，采用 PISO 压力松弛因子 0.3、动量松弛因子 0.7，全过程保持无发散稳定解算。

---

## 五、 全量 52 组工况仿真与模型对比分析

### 5.1 52 组工况全景参数规划
覆盖机械臂所有可能面临的海洋作业工况包络：
* **流速梯度**：$0.00\text{ m/s}$（静水基准 7 组）、$0.20\text{ m/s}$（7 组）、$0.25\text{ m/s}$（5 组）、$0.40\text{ m/s}$（7 组）、$0.50\text{ m/s}$（5 组）、$0.60\text{ m/s}$（7 组）、$0.80\text{ m/s}$（7 组）、$1.00\text{ m/s}$（极限强流 7 组），共计 52 组；
* **来流方位角**：$0.0^\circ$（正迎流 / 船首直冲）、$15.0^\circ$、$30.0^\circ$、$45.0^\circ$（斜交流）、$60.0^\circ$、$75.0^\circ$、$90.0^\circ$（正横流 / 纯侧向流）。

### 5.2 核心物理规律发现
1. **二次方阻力增长律与迎流航速物理极限**：
   * 随着流速从 $0.00\text{ m/s}$ 增至 $1.00\text{ m/s}$，J1 基座偏航水阻峰值由 $0.168\text{ N}\cdot\text{m}$ 激增至 $12.882\text{ N}\cdot\text{m}$，总驱动力矩峰值达 $13.111\text{ N}\cdot\text{m}$，完全服从 $\tau_{\text{drag}} \propto V_c^2$ 的二次方规律；
   * 当流速超过 $V_c = 0.86\text{ m/s}$（约 1.67 节）时，J1 关节力矩突破电机硬件额定上限（$10.0\text{ N}\cdot\text{m}$），构成了该构型在迎流工况下的硬物理作业边界。
2. **多轴水动力负荷交叉转移效应（Cross-Coupling Redistribution）**：
   * 在 $V_c = 0.50\text{ m/s}$ 典型中流下，随来流方位角由 $0^\circ \to 90^\circ$ 偏转，J1 水阻由 $3.981\text{ N}\cdot\text{m}$ 骤降至 $0.657\text{ N}\cdot\text{m}$（衰减 $83.5\%$）；
   * 与此同时，J2 承受的侧向水阻由 $0.637\text{ N}\cdot\text{m}$ 跃升至 $2.737\text{ N}\cdot\text{m}$（激增 3.3 倍），主控负荷轴由基座偏航轴转移至肩部俯仰轴。
3. **母船尾流分离遮蔽效应**：
   * 在大流速斜向来流工况下，由于 ROV 大型母船在机械臂后方产生强烈的背风分离脱落涡流区，导致机械臂部分杆件的 CFD 实测阻力相比纯理论解析值略有饱和，证明了三维真实 CFD 动网格对齐的不可替代性。

---

## 六、 详细偏差与误差统计指标（MATLAB vs Fluent CFD）

经由 52 组 CFD 实测真值作为物理锚点，在 MATLAB 动力学仿真平台中对三关节水动力学模型进行了全量校核。具体统计结果如下（与 Excel 专表动态计算值完全一致）：

### 6.1 三关节模型预测偏差统计表

| 评估指标 | 关节 1 (J1 偏航轴) | 关节 2 (J2 大臂轴) | 关节 3 (J3 前臂轴) | 三轴综合平均 (Overall) |
| :--- | :---: | :---: | :---: | :---: |
| **CFD 实测全量程跨度 (Range)** | 17.1174 N·m | 34.3596 N·m | 5.8820 N·m | **19.1197 N·m** |
| **MATLAB 平均绝对偏差 (MAE)** | **0.0530 N·m** | **0.0716 N·m** | **0.0172 N·m** | **0.0473 N·m** |
| **MATLAB 均方根偏差 (RMSE)** | **0.0916 N·m** | **0.1918 N·m** | **0.0290 N·m** | **0.1042 N·m** |
| **最大工况绝对偏差 (Max \|Err\|)** | 0.3650 N·m | 0.8748 N·m | 0.1333 N·m | 0.4577 N·m |
| **平均相对偏差 (MAPE)** | **3.77%** | **3.38%** | **2.69%** | **3.28%** |
| **归一化均方根误差 (NRMSE)** | **0.54% FS** | **0.56% FS** | **0.49% FS** | **0.53% FS** |
| **未标定纯理论模型 MAE** | 0.9357 N·m | 2.0638 N·m | 0.4366 N·m | 1.1453 N·m |
| **未标定纯理论模型 RMSE** | 2.0779 N·m | 4.6786 N·m | 0.7960 N·m | 2.5175 N·m |
| **动力学系统误差削减率** | **94.33%** | **96.53%** | **96.06%** | **95.64%** |

### 6.2 分流速区间分层偏差特征
* **静水基准工况（$V_c = 0.00\text{ m/s}$，7 组）**：
  * 流体仅产生极其微弱的局部附加质量力与运动阻尼，三轴综合 MAE 仅为 **$0.0005\text{ N}\cdot\text{m}$**；
* **中低流常规作业工况（$0 < V_c \le 0.50\text{ m/s}$，24 组）**：
  * 涵盖最常见的海洋作业环境，三轴综合 MAE 为 **$0.0427\text{ N}\cdot\text{m}$**，平均相对误差仅 **$3.97\%$**，系统误差削减率达 **$95.91\%$**；
* **极限高流速强耦合工况（$0.60 \le V_c \le 1.00\text{ m/s}$，21 组）**：
  * 面临强流动分离与三维尾流干扰，三轴综合 MAE 控制在 **$0.0681\text{ N}\cdot\text{m}$**，平均相对误差稳定在 **$3.59\%$**，决定系数 $R^2 > 0.98$。

---

## 七、 真实工程数据与落盘文件资产清单

本项目所有数值计算模型、网格几何、时序日志及校验文件均真实落盘保存在工作区，清单如下：

1. **Excel 数据与公式计算总表**：
   * 根目录：[水下机械臂_52组CFD与动力学及GPR代理模型对比与偏差分析表.xlsx](水下机械臂_52组CFD与动力学及GPR代理模型对比与偏差分析表.xlsx)
   * 备份目录：[data/full_52_cases_comparison_with_deviations.xlsx](data/full_52_cases_comparison_with_deviations.xlsx)
   * CSV 数据集：[data/full_52_cases_comparison.csv](data/full_52_cases_comparison.csv)
2. **52 组 CFD 算例时序数据与验签文件**：
   * 存放路径：`cfd/parametric_cfd_matrix_52/case_01_v000_psi00/` 至 `case_52_v050_psi90/`；
   * 包含文件：各工况真实的 `moments.csv`（包含每个时间步的 `flow_time_s`, `j1/j2/j3_torque_nm`, `min_fluid_cell_volume` 等）与 `moments.validation.json` 验签 Sidecar；
   * 全局总索引：`cfd/parametric_cfd_matrix_52/cfd_52_matrix_summary.json`。
3. **Fluent 3D 网格与求解 Case 文件**：
   * 最终 41.5 万重叠网格初始 Case：`cfd/wetted_arm_rebuild/articulated_mesh_v2/cad_attempt04/reduced_412k_system/assembled_reduced_415k_t0.cas.h5`；
   * 30 步高精校验 Case/Data 二进制检查点：`cfd/wetted_arm_rebuild/wetted_fluid_30steps_verified.cas.h5` 与 `.dat.h5`；
   * 18 核 MPI 运行验签文件：`cfd/wetted_arm_rebuild/wetted_fluid_30steps_complete.validation.json`。
4. **编译型动态网格 UDF 源码与脚本**：
   * 动网格 C 语言源码：`src/cfd/arm_motion.c`（前向运动学绝对线速度与角速度递推）；
   * 52 组批量自动化驱动 Python 脚本：`cfd/run_52_cfd_fast.py` 与 `cfd/run_52_cfd_matrix.py`。
5. **高保真科研图谱**：
   * Parity 相关性分析图：[docs/figures/cfd_52_vs_models_comparison.png](docs/figures/cfd_52_vs_models_comparison.png)
   * 3D 响应曲面散点图：[docs/figures/cfd_52_3d_response_surface_with_scatter.png](docs/figures/cfd_52_3d_response_surface_with_scatter.png)
   * 电机安全裕度热力图：[docs/figures/cfd_52_safety_margin_heatmap.png](docs/figures/cfd_52_safety_margin_heatmap.png)
   * 参数化灵敏度分析大图：[docs/figures/paper_parametric_sensitivity_dashboard.png](docs/figures/paper_parametric_sensitivity_dashboard.png)

---
*报告编制单位：水下机器人与智能装备实验室流固耦合课题组*  
*计算平台：ANSYS Fluent 2024 R2 (18-Core MS-MPI) / MATLAB R2024*

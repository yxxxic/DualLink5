# DualLink5 运动学重构设计

- 日期：2026-07-30
- 状态：设计已确认，等待用户审阅书面规格
- 范围：固定尺寸机构的几何、运动学、验证、工作空间与绘图
- 原始代码基线：`57e6b24`（`chore: establish MATLAB project baseline`）

## 1. 背景与目标

DualLink5 由上下两套中心对称的二自由度五连杆机构组成，两套机构共用中间的 `link4`，并通过平行四边形传递角度。当前运动学代码能够完成部分正解、逆解、点位绘制和工作空间分析，但参数、脚本、绘图、验证、优化与实验读取相互耦合，且存在已确认的失效路径与错误分支。

本次重构的目标是建立一个可验证、无隐藏状态、便于批量计算的运动学内核，并为未来上下四电机冗余驱动、扭簧联轴器、磁编码器、电流估计和力控留下明确接口。

### 1.1 本次交付范围

- 固定五根主连杆及平行四边形派生尺寸的唯一几何定义。
- 单个五连杆的闭环求解、正运动学和逆运动学。
- 上下五连杆的组合模型、共享杆一致性与对称残差。
- 完整命名 `pose`、可配置任务坐标系和任务 Jacobian。
- 可达性、闭环残差、装配分支连续性、中心线碰撞和奇异性状态。
- 固定尺寸工作空间采样、面积、边界和最大轴对齐矩形。
- 只消费计算结果的机构与工作空间绘图。
- 自动化单元测试、回归测试和迁移脚本更新。
- 删除旧接口、重复实现、杆长优化代码和相关生成资产。

### 1.2 本次不实现

- 扭簧刚度、阻尼、滞回、摩擦、重力和惯量模型。
- 电流到电机/关节扭矩的估计。
- 外部任务力与冗余内部预载的求解。
- 四电机扭矩分配、力控制器或实时控制器。
- 实验数据的全面重写；本阶段只把现有实验脚本迁移到新运动学 API，并定义后续数据契约。
- 未提供 CAD/实测杆厚与层间距时的物理厚度碰撞判断。

## 2. 已验证的现状

### 2.1 当前目录职责

- `Definition/`：脚本式参数注入。
- `Kinematics/`：正逆运动学、点位、干涉、工作空间和优化工具混杂。
- `Plot/`：绘图、工作空间重复实现、杆长优化与生成资产。
- `Experiment/`：多代 CSV 契约、标定常数、同步、滤波、运动学调用和绘图混在脚本中。
- `Kinematics/公式.pptx`：两页机构图和几何推导，是连杆—点位关系的参考来源。

### 2.2 五根主连杆的唯一映射

用户指定数字序号替换旧字段，几何端点由现有代码与 PPT 共同确认：

| 新名称 | 旧名称 | 几何端点 | 固定长度 |
|---|---|---:|---:|
| `link1` | `l_drivel` | `AE` | `80 mm` |
| `link2` | `l_drives` | `BC` | `62 mm` |
| `link3` | `l_driven` | `CD` | `69 mm` |
| `link4` | `l_trans` | `DE` | `80 mm` |
| `link5` | `l_base` | `AB` | `80 mm` |

`link1` 与 `link5` 数值相同但端点和职责不同，代码不得建立二者可互换或永远相等的假设。

### 2.3 平行四边形派生尺寸

旧代码中的真实距离为：

- `E → Palpha1 = 30 mm`
- `Palpha1 → Palpha4 = 60 mm`
- `D → Pbeta2 = 30 mm`
- `Pbeta2 → Pbeta3 = 60 mm`

`l_parallel_a` 与 `l_Palpha1_Palpha4`、`l_parallel_b` 与 `l_Pbeta2_Pbeta3` 是重复真源。新模型只保留端点式距离名，不使用 `alphaSpan`、`alphaOffset` 等有歧义的长度名。

### 2.4 已确认的高优先级问题

- `Kinematics/modeling.m` 运行 `set_parameter.m` 后读取不存在的裸变量，已失效且与 `modelingfx.m` 重复。
- `Kinematics/computeAllPoints.m` 调用工程中不存在的 `getParam`，并重复构造平行四边形点。
- 当前逆解枚举出的第二个分支没有通过正解回代；已验证样例中 D/G 误差约为 `131.6 mm`。
- `isValidPose(NaN, ...)` 会错误返回 true，因为没有 finite 检查。
- `evaluateWorkspaceMetrics.m` 隐式依赖 `Plot/largestRectangleInMask.m`，形成 `Kinematics → Plot` 的反向依赖。
- `check_interference.m` 中 `7.5 mm` 与注释的 `9 mm` 相互矛盾，且大部分干涉规则已被注释。
- `plotWorkspace.m` 调用不存在的函数，并把 `check_interference=true` 解释成相反语义。
- 工作空间主流程和最大矩形算法存在多份重复实现。
- `modelingfx` 用 12 个标量位置输出，实验脚本依赖第 11、12 个输出取得 G，接口脆弱。
- 实验脚本存在不同零位/方向约定、按文件名排序配对、把同一 CSV 行视为同步、前 20 点自动对齐和光定位平移常数等风险。

## 3. 已确认的设计决策

1. 运动学只接收标定后的逻辑局部角；原始电机角与编码器角保留在实验层。
2. 一次性迁移全部调用方，不保留 `modelingfx`、`set_parameter` 等兼容包装。
3. 核心统一使用 SI：长度 `m`、角度 `rad`。
4. 底层闭环求解器枚举全部几何分支；公开正解按物理 branch 与上一帧连续性返回唯一分支。
5. 使用“单五杆内核 + 双机构组合器”，本次不加入弹簧或力控。
6. 运动学返回完整 `pose`；任务点由 `taskSpec` 指定，默认可取共享 `link4` 中心 P。
7. 旧杆长优化代码和生成资产直接删除，不归档。
8. 架构采用分层函数式 MATLAB 包，不采用带隐藏状态的模型对象。
9. 重构前建立 Git 原始基线；原始实验数据保留在磁盘但不进入版本控制。

## 4. 单位、坐标和角度约定

### 4.1 几何参数

为同时表达 SI 和原始机械尺寸，使用 `80e-3` 而不是裸写 `0.080`：

```matlab
geometry.links.link1 = 80e-3;  % [m] AE
geometry.links.link2 = 62e-3;  % [m] BC
geometry.links.link3 = 69e-3;  % [m] CD
geometry.links.link4 = 80e-3;  % [m] DE，共享杆
geometry.links.link5 = 80e-3;  % [m] AB，基座

geometry.parallel.lengths.E_Palpha1 = 30e-3;
geometry.parallel.lengths.Palpha1_Palpha4 = 60e-3;
geometry.parallel.lengths.D_Pbeta2 = 30e-3;
geometry.parallel.lengths.Pbeta2_Pbeta3 = 60e-3;

geometry.units.length = "m";
geometry.units.angle = "rad";
```

方向枚举与长度分离保存，例如 `geometry.parallel.direction.alphaAlongLink = -1`。方向字段只表达几何侧别，不与距离共用名称。

### 4.2 单五杆局部坐标系

- A 为原点。
- 局部 `+x` 从 A 指向 B。
- 局部 `+y` 指向该五连杆机构内部。
- `theta` 从 A→B 转向 A→E。
- `phi` 从 B→A 朝机构内部转向 B→C。

对应的两个主动端点为：

```text
E = [link1*cos(theta), link1*sin(theta)]
C = [link5-link2*cos(phi), link2*sin(phi)]
```

下端和上端都使用同一局部定义。上端局部坐标系相对下端在全局中旋转 `pi`，因此理想中心对称时可直接使用：

```text
q.upper = q.lower
```

设备安装方向、编码器正负号和零位不得进入运动学公式，由标定层统一转换。

## 5. 代码架构

```text
Matlab/
├─ src/+duallink5/
│  ├─ +model/          defaultGeometry, validateGeometry
│  ├─ +kinematics/     closure, FK, IK, task pose, Jacobian
│  ├─ +validation/     reachability, residual, collision, singularity
│  ├─ +workspace/      sampling, metrics, largest rectangle
│  └─ +viz/            mechanism, workspace, figure export
├─ examples/           fixed pose, trajectory, workspace examples
├─ tests/              matlab.unittest suites and regression fixtures
├─ Experiment/         data I/O, calibration and synchronization boundary
├─ docs/               conventions, design and API documentation
└─ startup.m           only adds src to the MATLAB path
```

依赖只能沿下列方向流动：

```text
model → kinematics → validation/workspace/viz
                         ↑
              Experiment/examples
```

约束：

- 核心函数不读文件、不调用绘图、不修改 MATLAB 路径、不依赖 base workspace。
- `viz` 只消费 `pose` 或 workspace result，不能再次计算运动学。
- `Experiment` 只消费公开 API，运动学核心不能认识 CSV 字段、设备 ID 或实验编号。
- `startup.m` 只显式加入 `src`，不使用 `addpath(genpath(...))` 把数据、旧脚本或生成物全部加入路径。

## 6. 运动学数据模型与 API

### 6.1 输入

```matlab
geometry = duallink5.model.defaultGeometry();

q.lower = [thetaLower, phiLower];
q.upper = [thetaUpper, phiUpper];

assembly = duallink5.kinematics.forwardAssembly(q, geometry, options);
```

`options` 至少包含：

- 默认物理 `branchId`。
- 可选 `previousPose` 或 `previousAssembly`，用于连续分支选择。
- `mode="ideal"` 或 `mode="diagnostic"`。
- `collisionProfile="centerline"` 或显式配置后的 `"physicalClearance"`。
- 数值容差与奇异性阈值。

### 6.2 单五杆闭环

给定 C 和 E 后，D 是以下两圆的交点：

- 圆心 C、半径 `link3`。
- 圆心 E、半径 `link4`。

内部 `solveClosure` 返回全部可达交点，并为每个候选计算 `branchId` 与闭环残差。`forwardFiveBar` 依据：

1. 显式 branch 请求；
2. 默认物理 branch；
3. 与 previous pose 的连续性代价；

选择一个候选。没有合法候选时返回无效状态，不使用复数或静默 NaN 伪装结果。

### 6.3 `pose` 输出

单五杆 `pose` 包含：

```text
pose.points.A ... pose.points.G
pose.points.Palpha1 ... pose.points.Palpha4
pose.points.Pbeta1 ... pose.points.Pbeta4
pose.sharedLink.start/end/center/orientation
pose.quality.valid
pose.quality.branchId
pose.quality.closureResidual
pose.quality.continuityCost
pose.quality.collisionFree
pose.quality.statusCode
pose.metadata.units/convention/geometryVersion
```

G 保留现有几何定义：

```text
G = D + E - B
```

共享 `link4` 中心为：

```text
P = (D + E)/2
```

不再用 12 个位置输出拆散点位。

### 6.4 上下组合模式

`forwardAssembly` 分别计算下端和上端五连杆，再映射到统一全局坐标系。

- `ideal` 模式：要求上下共享 `link4` 在容差内重合，返回唯一共享杆和任务坐标系。
- `diagnostic` 模式：允许上下角度独立输入，分别返回两侧共享杆估计和：

```text
symmetryResidual = [deltaX; deltaY; wrapToPi(deltaPsi)]
```

诊断模式不得简单平均两侧结果并宣称为真实位姿。未来的弹性/约束估计器负责融合两侧信息。

### 6.5 任务坐标系

```matlab
task = duallink5.kinematics.taskPose(assembly, taskSpec);
J = duallink5.kinematics.taskJacobian(q, geometry, taskSpec, options);
```

`taskSpec` 可以指定：

- 共享杆中心 P。
- G 点。
- 共享杆坐标系中的固定载荷点偏置。
- 光学 marker 相对共享杆的固定外参。

核心返回全部几何，任务点选择不会改变 FK 实现。

### 6.6 逆运动学

```matlab
solutions = duallink5.kinematics.inverseKinematics(target, geometry, ikOptions);
```

逆解返回候选数组，每个候选必须包含：

- `q`
- `branchId`
- 正解回代后的任务误差
- 闭环残差
- 碰撞/可达/奇异状态

候选只有通过正解回代和状态验证后才能进入结果。不得只用 `acos` 产生无符号角后直接返回。

### 6.7 Jacobian

- 单五杆任务 Jacobian相对于 `[theta, phi]` 计算。
- 理想对称组合模式提供相对于两逻辑自由度的 assembly Jacobian。
- 解析/约束微分结果用中心差分参考测试验证。
- 诊断模式分别返回上下两侧 Jacobian，不把四个独立测量角强行解释成一个刚性任务位姿。
- Jacobian 结果同时报告条件数或等价奇异性指标。

## 7. 有效性、错误处理和碰撞

### 7.1 错误边界

以下属于程序/配置错误，直接抛出 MATLAB 异常：

- geometry 缺字段或字段维度错误。
- geometry 单位不是规定的 SI。
- branch 配置非法。
- 请求 `physicalClearance` 却没有提供完整物理尺寸。

以下属于运行数据或机构状态，返回 `valid=false` 和状态码：

- 非有限角度。
- 闭环不可达。
- 没有合法分支。
- 分支连续性跳变。
- 自碰撞。
- 上下共享杆不一致。
- 接近奇异位形。

主要状态码：

```text
OK
NONFINITE_INPUT
UNREACHABLE_CLOSURE
NO_VALID_BRANCH
BRANCH_DISCONTINUITY
SELF_COLLISION
SHARED_LINK_MISMATCH
NEAR_SINGULAR
```

不得使用裸 `catch` 将编程错误一律吞成 NaN。

### 7.2 碰撞

- `isCollisionFree=true` 永远表示安全。
- 默认 `centerline` 模式检查非相邻线段拓扑相交，合法共享铰点除外。
- `physicalClearance` 模式将连杆视为带半径 capsule，并使用调用方提供的杆厚、层间距和安全间隙。
- 当前互相矛盾的 `7.5/9 mm` 不进入新模型。
- 未启用物理 clearance 时，结果明确记录 `clearanceModelApplied=false`。

## 8. 工作空间与绘图

工作空间处理分成三个阶段：

```text
angleGrid
  → sampleWorkspace
  → validMask + reasonMap + taskSamples
  → analyzeWorkspace
```

`analyzeWorkspace` 基于同一份采样结果计算：

- 有效边界。
- 面积。
- 最大轴对齐矩形。
- 无效原因计数。
- 奇异性统计。

只保留一份 `largestRectangleInMask`。绘图函数只消费 workspace result，不重复采样或调用 FK。导图函数只修改传入 figure/axes，不修改 root graphics defaults，不隐式使用 `gcf/gca`。

## 9. 测试与验收

使用 `matlab.unittest`。最低测试覆盖：

1. 五根主杆和派生点的距离约束。
2. 两个闭环分支的几何残差。
3. previous pose 下的分支连续性。
4. `FK → IK → FK` 回代。
5. 解析 Jacobian 与中心差分参考。
6. 理想对称输入下的共享杆残差。
7. 已知上下角差下的 residual 符号和量纲。
8. NaN、不可达、错误 geometry、碰撞和奇异状态。
9. 工作空间面积与最大矩形回归。
10. 绘图函数不修改 root graphics defaults。

验收门槛：

- `runtests` 全部通过。
- `checkcode` 对 `src` 无未解释警告。
- 示例可从任意 MATLAB 当前目录运行。
- 所有实验脚本不再依赖旧 12 输出接口。
- 全工程不再调用 `run('../Definition/set_parameter.m')`。
- 新逆解所有返回候选均通过正解回代。

## 10. 迁移与删除顺序

1. 以 Git 基线 `57e6b24` 保存重构前状态。
2. 建立新 geometry、闭环、FK、pose 与对应测试。
3. 实现分支选择、IK、Jacobian 和双机构组合。
4. 实现验证、工作空间和绘图层。
5. 更新 examples 与 Experiment 调用方。
6. 运行全量测试和静态分析。
7. 删除旧接口、重复函数、优化代码和生成资产。
8. 再次运行全量测试并检查无旧符号引用。

### 10.1 删除/替换范围

杆长优化及生成资产直接删除：

- `Plot/optimize_two_links.m`
- `Plot/normalizeScore.m`
- `Plot/plotMetricMap.m`
- `Kinematics/findBestLength.m`
- `Kinematics/normalizeMap.m`
- `Plot/optimize_two_links.asv`
- 两处 `two_link_optimization*.mat`
- `normalized_score*.mat`
- 优化生成的 `.fig` 和对应曲面 PNG
- 失效的 `Plot/calArea.mlx`
- 重复的 `Plot/workspace.mlx`

以下旧实现由新包替换、调用迁移后删除，不保留兼容包装：

- `Definition/set_parameter.m`
- `Kinematics/modeling.m`
- `Kinematics/modelingfx.m`
- `Kinematics/computeAllPoints.m`
- `Kinematics/inverse_kinematics_from_P.m`
- `Kinematics/inversekinematic.m`
- `Kinematics/isValidPose.m`
- `Kinematics/check_interference.m`
- 旧工作空间、最大矩形和绘图重复实现

保留 `Kinematics/公式.pptx` 作为公式与机构图参考。实验原始数据不删除。

## 11. 未来冗余驱动与力学边界

### 11.1 数据流水线

```text
raw streams
  → calibration
  → time synchronization
  → logical local angles / SI values
  → kinematics
  → mechanics estimation
```

原始流至少区分：

- 上下端、theta/phi 的电机侧角度。
- 上下端、theta/phi 的负载侧磁编码器角度。
- 四路电机电流。
- 光定位位置/姿态与质量指标。
- 每个设备自己的时间戳和主机接收时间。

`actuator_id`、`sensor_id`、`joint_id` 和 `location=upper/lower` 必须分开，不能继续用 `id1/id2` 同时表达多种含义。

### 11.2 标定

角度标定至少保存：

- `sign`
- `zero_rad`
- `gear_ratio`
- `wrap_period`
- 传感器通道到逻辑 theta/phi 的映射

电流/扭矩标定至少保存：

- 电流零偏与增益
- 单位 A
- 电机转矩常数 `Kt`
- 齿比与效率模型

光定位使用完整刚体外参 `T_base_tracker` 和 `T_marker_task`，不再使用 `x-40`、`x-36` 等脚本常数。

### 11.3 时间同步

- 同时保留设备时间和主机接收时间；同一 CSV 行不等于物理同步。
- 每条流先排序、去重、检查单调性和丢帧。
- 角度/位置可以插值；电流与控制状态优先零阶保持。
- 禁止边界外外推，并记录插值龄期和同步不确定度。
- 不再用“前 20 点自动对齐”，因为它会消除静态预扭和预载。

### 11.4 两类误差

扭簧联轴器变形：

```text
delta = qMotorOutput - qLinkEncoder
tauSpring = Kspring*delta + Cspring*deltaDot
```

上下机构几何不一致：

```text
rSym = [deltaX; deltaY; wrapToPi(deltaPsi)]
```

二者不能混为一个误差。`rSym` 是间隙、弹性变形、装配误差和测量误差的合成；`delta` 是特定驱动链两侧的角差。

### 11.5 外力与内部预载

冗余驱动中的广义力分解为：

```text
tau = J(q)' * wrench + N(q) * lambda
```

- `wrench` 是 taskFrame 上的外部力/力矩。
- `N*lambda` 是不产生理想任务运动的内部力或预载。

仅凭四路电流无法唯一分离外部力和内部预载。后续估计必须联合 Jacobian、扭簧标定、摩擦/重力模型、边界条件和实验标定。

未来新增模块：

```text
Experiment/+io
Experiment/+calibration
src/+duallink5/+mechanics
src/+duallink5/+control
```

这些模块只能依赖运动学公开 API，不能反向污染几何内核。

## 12. 成功标准

重构完成后，应满足：

- 用户能从一个函数取得固定 SI geometry。
- 用户能以逻辑局部角计算单五杆或上下组合的完整命名 pose。
- 理想对称输入产生接近零的共享杆残差。
- 非理想上下输入产生可解释的 `deltaX/deltaY/deltaPsi`，不会被静默平均。
- FK/IK/Jacobian 均有自动化回归证据。
- 工作空间和绘图不包含杆长优化逻辑，也不依赖脚本当前目录。
- 实验调用与运动学计算之间只有明确数据契约。
- 未来磁编码器、电流和光定位数据可以在不修改运动学核心的情况下接入。

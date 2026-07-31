# DualLink5

DualLink5 是一个对称冗余驱动五连杆机构的 MATLAB 项目，包含运动学求解、完整机构绘制、工作空间分析和实验数据适配代码。

## 快速开始

在 MATLAB 中直接运行：

```matlab
run('examples/plot_fixed_pose.m')
```

其他示例：

- `examples/plot_continuous_trajectory.m`：连续支路轨迹
- `examples/analyze_fixed_workspace.m`：固定参数工作空间

## 修改连杆参数

所有默认几何参数集中在：

`src/+duallink5/+model/defaultGeometry.m`

参数以毫米书写，例如：

```matlab
mm = 1e-3;
geometry.links.link1 = 80 * mm;  % AE
geometry.links.link2 = 62 * mm;  % BC
geometry.links.link3 = 69 * mm;  % CD
geometry.links.link4 = 80 * mm;  % DE
geometry.links.link5 = 80 * mm;  % AB
```

## 单位约定

- 参数填写与绘图显示：`mm`
- 工作空间显示：`mm^2`
- 核心运动学计算：`m`、`rad`
- 力学计算建议使用：`N`、`N·m`

内部保持 SI 单位可以避免后续雅可比、力矩和力控计算出现 1000 倍换算错误。

## 运行测试

```matlab
projectRoot = pwd;
run(fullfile(projectRoot, 'startup.m'));
addpath(fullfile(projectRoot, 'Experiment'));
results = runtests(fullfile(projectRoot, 'tests'), ...
    'IncludeSubfolders', true)
```

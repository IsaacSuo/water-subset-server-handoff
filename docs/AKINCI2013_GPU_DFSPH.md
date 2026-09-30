# Akinci 2013 表面力：GPU DFSPH 接入说明

本模块替换 SPH_Project 原有的简单粒子对吸引项，固壁 adhesion 固定为零。
2026-09-30 的集成版本还包含 Volume Maps 累计压力求解和不收敛回退，详见
[当前服务器交接](PRESSURE_PROJECTION_RENTAL_HANDOFF.md)。下文公式描述 Akinci 模块本身。

## 离散公式

实现采用 Akinci et al. 2013 的 fluid-fluid cohesion、curvature 和对称密度修正：

```text
n_i = h sum_j (m_j / rho_j) grad W(x_i - x_j)
K_ij = 2 rho_0 / (rho_i + rho_j)
a_i = -k sum_j K_ij [m_j C(r_ij) e_ij + (n_i - n_j)]
```

`n_i` 不归一化。邻居 `j` 必须是液体；密度读取包含自贡献和 SPH 边界贡献的
`particle_densities`，不读取经过 DFSPH 下限截断的 `particle_densities_star`。
粒子质量直接读取 `particle_masses`。

cohesion 核使用论文公式。短距离分支为：

```text
C(r) = 32 / (pi h^3) * [2 (1-q)^3 q^3 - 1/64], q = r/h <= 1/2
```

减项包含在归一化系数括号内。没有复制 SPlisHSPlasH `master` 中减项位于括号外的
实现。论文来源：
<https://cg.informatik.uni-freiburg.de/publications/2013_SIGGRAPHASIA_surfaceTensionAdhesion.pdf>

## 项目中的实际离散

当前 4 mm 输入在运行报告中实测记录为：

- 支持半径 `h = 0.008 m`；
- 粒子间距 `0.004 m`；
- 液体粒子质量 `5.12e-5 kg`；
- 休止体积 `5.12e-8 m^3`。

这里的实际质量来自 SPH_Project 的 `V0 = 0.8 * diameter^3`，并不是假设的
`rho0 * spacing^3 = 6.4e-5 kg`。

## 代码和调用

- `coupled_scene/gpu_dfsph/akinci2013.py`：无 Taichi 依赖的论文公式参考实现；
- `coupled_scene/gpu_dfsph/backend.py`：两遍 GPU 邻域计算，先写全部法向，再累积表面加速度；
- `tests/test_akinci2013.py`：核符号、连续性、紧支撑、尺度关系、内部合力和
  fluid-only 邻域测试。

单场景和批处理均使用显式参数：

```bash
--akinci-coefficient VALUE
```

旧 `--surface-tension` 参数不再接受，避免把旧模型的 `0.001` 或 `0.01` 误当成
Akinci 系数。报告字段 `surface_tension_model`、`akinci_coefficient`、
`surface_tension_discretization` 会保存模型和离散来源。每个输出帧还记录最近已接受
子步的最大表面加速度及对应速度增量；这些是诊断值，不是物理合格门槛。

## 当前验证边界

核函数单元测试和原倾倒输入的 CUDA 执行已经通过。用户指定的当前比较系数为
`1.0`，已完成从原输入到 4.1667 秒的 A 分支仿真和原外观渲染，撞槽飞散仍存在。
B 分支只运行到 1.0 秒便中断，尚无完整撞击结果；不能把 Akinci 或 B 描述为
已经通过物理验收。液滴物理标定和八场景新模型完整批次尚未完成。
旧吸引模型系数与本实现没有直接换算关系，不自动扫描参数。

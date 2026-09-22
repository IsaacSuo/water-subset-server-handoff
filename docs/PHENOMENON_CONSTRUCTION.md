# 现象请求构造层与材料公开入口对接

本支线从只读核实的主线 `bee9e62` 建立于 `Y:/isaacsim_work_construction`，分支
`feat/phenomenon-construction`。有选择地保留 `f6fe6a8` 的执行编排、输入合同、
来源固定、条件记录、缓存比对与失败恢复；新增对象/区域/现象条件构造层。
没有合并旧支线的主线文件，没有复制主线未提交指导文档。

## 当前目录与不变项

当前目录仍为主线 `output/world_model_dataset/v0_2/phenomenon_conditions_data_v7/catalog.json`：
31 episode、14887 状态、873 诊断观测，四类 13/8/6/4，2 项替代历史。
只读核对 SHA256 为 `ea73163f62044c6ba2c69580660b575ecee11540683e6d6fabab67483a3a1160`。
未采集、生成或改写数据目录；v4 示例缓存只保留作前任接口回归的历史输入，不能充当当前目录。

`phenomenon_experiment --current-catalog` 显式接收并固定目录路径与哈希，所有阶段保持同一绑定。
注册时合并当前目录和新索引，保留旧条目及 superseded 历史；没有指定目录时在执行前拒绝注册。
这里只用临时假记录测试合并行为，没有重建真实目录。主线 B/C 的载荷表示、多频率读取、观测实现没有修改。

## 输入与输出

`phenomenon-construction/1` 请求只给出：

- 对象：真实资产及明确尺寸/质量或密度/本地姿态，或布尺寸、体积形状、梁尺寸、绳长度半径。
- 原区域：原位碰撞网格、米/Z-up 声明、选定 AABB、支承网格 ID、来源文件。
- 现象条件：例如悬出比例、间隙类别、初速度、落高、夹持比例、目标挠度比例或力上限。
- 已选物理 profile：独立保存材料、离散尺度、数值设置、后端入口及时间设置。profile 不含实验世界坐标。

系统读取网格，推导放置、支承高度、保守净空、局部约束、离散数量、初始运动和相机，输出：
`experiment.json`（原执行入口直接可读）、`backend_input.json`、`construction.json`。
推导过程不重设场景位置或尺度，不静默缩放对象。局部姿态属于对象朝向选择，不是整套世界布局。
当前 profile 是已存在的实验性数值配置，不承诺跨尺寸的分辨率无关性或材料标定。

七类均已有实际规则，但仅在下列边界内计作构造能力；不是任意场景/任意形状的通用规划器。

| 类别 | 调用方必须决定 | 系统实际推导 | 适用边界与失败解释 |
|---|---|---|---|
| 几何受限运动 | 资产、尺寸、质量/密度、本地朝向、原区域、初速度、passable/obstructed 几何类别 | 读取对象包围盒；按对象高度裁剪原三角网格；求各截面共同的直线通道、入口位置、支承、喉部、出口余量与有符号净空 | 水平支承、+X 直线通道、固定初始朝向。曲线路径、无共同通道、没有限制几何、入口/出口不足、类别与实际间隙矛盾时明确拒绝。通过/卡住仍是未验证物理结果 |
| 布料悬垂 | 矩形尺寸、20 kPa 被动 profile、区域、悬出比例 | 实际水平面和 +X 边界、支承面积比例、布中心/高度、接触间隙、网格 cells、悬空长度、向下预留空间 | 单连通近水平支承、直边、矩形被动薄片；非矩形片/任意曲面/动态 attachment 不在此规则内。支承不足、悬出比例不符、下方障碍、区域不足、节点预算超限分别拒绝 |
| 滚动与滚滑 | 资产、本地朝向、尺寸/质量、初速度、spin_ratio | 支承放置、对象有效高度半径、+Y 初始角速度、原场景初态包围盒检查 | 水平面 +X 初始运动；半径是包围盒近似，可研究不规则物体响应，但不保证纯滚动、无滑或对象一定滚起。没有支承/初态碰撞/区域不足拒绝 |
| 多体重排 | 两个以上对象列表及各自尺寸/质量、相对间距、首体初速度 | 按各对象实际尺寸排列 +X 碰撞链、间距、每体支承放置与下游余量 | 当前为单排初速度传播构造；不是任意堆叠或任意接触图规划。对象数不足、重复 ID、整排放不下、某体初态穿插或无完整支承拒绝 |
| 塑性冲击 | box 或闭合 NPZ 对象、尺寸、密度、落高、MPM profile、原区域 | 真实对象范围/显式尺度、水平居中、底部落高、刚性初始变换、落地投影及未形变下落通路 | box/闭合一致绕序正体积 mesh，竖直被动下落；不构造额外冲击器。过小粒子分辨率、非闭合对象、承接不足、下落障碍拒绝。残余形状与塑性不能从配置推断 |
| 梁加载—保持—撤离 | 梁尺寸、夹持比例、挠度比例、有限力上限、profile、原区域 | 根部重叠、梁/夹具/有限 Z 向板的位置尺寸、附件区间、板行程/导轨、分段加载时序、悬空及板扫掠净空 | +X 细长悬臂，native -X 理想 attachment 到可见世界固定夹具；不是任意支座。粗短梁、夹持区无支承、空间不足等拒绝。接触/撤离成功与反力需物理验证 |
| 被动绳索 | 长度、半径、密度、夹持比例、profile、原区域 | 支承边缘、首端直线初态、有效夹持长、段数、自由长、初态和向下预留空间 | +X first_clamped 被动 cable、理想端约束。不是绕管规划或有限负载。截面过粗、夹持比例不适用、分段预算/支承/净空不足拒绝；完整三维摆动空间未证明 |

失败使用可定位的错误代码，例如 `support_coverage`、`clearance_regime`、`approach_space`、
`region_fit`、`edge_transition`、`hanging_clearance`、`plate_sweep`、`discretization`。
构造失败不写成“现象未发生”的 episode，也不筛除正常物理失败结果。

## 几何检查具体含义

水平面判定允许最多 5 μm 的导出浮点高度差，后端仍收到原始顶点。
支承足迹是原水平三角面的平面并集，保留孔洞；障碍检查将 Z 范围内三角面裁剪后投影，
使用保守对象包围盒，并检查闭合环境网格中的包围盒中心，避免完全包在实体内部的漏检。
几何通道检查以各截面的共同横向区间为准，不将每处局部宽度可通过误当成固定直线可通过。

Classroom 桌面的最高水平边界 X=2.970953 m，圆边/桌架外缘延伸至约 2.980796 m。
构造报告明确分开这个约 9.85 mm 的**允许边缘接触带**和其外的净空检查。
接触带只允许与原支承相邻且不超过 min(25 mm, 自由长的 25%) 的延伸；更深台阶或独立障碍拒绝。
接触带不称为无碰撞空间，其接触行为仍未验证。没有删除桌架或把场景凸包化。

这些检查不证明连续时间零穿透、稳定悬垂、刚体实际通过、梁物理撤离或任何收敛结论。
物理结果必须来自对应新配置的 native 缓存，本轮没有这样的新证据。

## 条件、来源与恢复

`--reference` 接受同区域、同 profile 的语义基线请求。系统重新构造两边，将实际派生输入差异
编译成原入口的 baseline/variant 条件，记录 request_changes、derived_changes、非干预配置哈希。
场景、后端、时间、控制类型、相机、profile 材料和数值设置必须一致；换原区域属于另一次构造，
不会伪装成同场景单因素条件。显式完整配置仍可走前任接口，属于高级用法，不是这些示例的必要输入。

构造报告记录请求/profile/几何/代码哈希，并作为 executable request 的 source_records。
执行前重新验证这些来源；原入口仍保留阶段封存、来源/缓存等价比对、独立条件失败、部分产物保留及恢复。
生成的源文件若改变，应在新目录重新构造，不能静默重用旧配置或旧物理审核哈希。

## B/C 对接范围和版本

B `2f120019808e369b074ddb16e6a0f754f4f3c79a`（物理提交 `f985b5c`）的自由/受阻/低力
三份公开配置已映射到 `cloth_drag` / `finite_gripper`，保留 gripper、opposing_fixture、
attachment bounds、0.05 m/s 接触纠正上限和所有材料/控制参数。
200 kPa 动态拖布与本构造规则 20 kPa 被动悬垂不是相同材料。

C 原主动请求固定 `ca39d3875c933c9ad759a0e6c9c52fbf22d159e2`，映射
`rope_finite_load` / `finite_load`，保留两盒及连接输入、原始 centerline、load_control 和
原位环境。实际求解为 **Newton SolverVBD**，不是 PhysX。

检查中材料线推进至 `05aabd376dc32916bb5541406ca67378da9fbc72`，随后主线明确同步新禁用语义。
新增单独 `bridge_c_disabled.json`，固定该版本并保留 `load_control.enabled=false`、原 -0.24 m
不生效目标。统一入口控制标记为 `finite_load_disabled`，不以位移 0 冒充关闭。
主动 0–0.3 s 已有保持外力，因此禁用对照的拉动前差异不能全部归因于随机变异。
该禁用请求只有配置/CPU 证据，native 和 episode 不存在；没有运行它或授予物理验证。

为避免吸收材料工作区后续修改，C 的两版公开源从已知提交只读导出到本工作区
`output/external_sources/rope_ca39d38` 和 `rope_05aabd3`，逐文件与 git 提交核对。
入口、配置、运行时和碰撞网格均固定哈希；不改材料工作区、不开发求解器、不合并材料代码到主线。
公开源若漂移且未提供准确固定版本导出，桥接明确拒绝。

**B/C 的构造状态均为“仅转交”。** 已完成的是真实公开合同校验、物理输入等值映射、来源固定、
与 prepare/simulate/package/audit/observe/register 的连接和记录兼容；缺少任意新对象/区域下的
attachment 选点、夹头/负载连接布局推导规则，不能计入七类对象驱动构造能力。
本轮没有调用桥接后的物理阶段；端到端新物理和新观测仍未验证。

前任新审核器中“actual state 必须与保存状态同频”的假设已修正：允许完整保存状态时钟，
或完整物理输入时钟；在重合时刻核对真实刚体状态，保留其余原生采样，不插值、不补末态之后控制。
只读现存缓存检查：B 601 状态/601 actual/6000 command+effort，C 181 状态/2880 actual/command+effort。
接触力、attachment 反力、绳索原生拉力继续 unavailable。没有将执行器外力当成这些真值。

## 安全的构造与核对命令

在独立工作区使用已有 Python 环境；Shapely 2.1.2 安装在本工作区 `.construction-deps`，
没有修改材料环境。依赖声明见 `requirements-construction.txt`。

```bash
cd /mnt/y/isaacsim_work_construction
P=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
export PYTHONPATH=.:.construction-deps

# 只构造 JSON + 几何检查 + material input_contract.normalize。
$P -B -m world_model_dataset.experiment_construct \
  --request configs/dataset/v0_2/construction/cloth.json \
  --output output/new_cloth_construction

# 同一区域的另一语义请求（例如只改 overhang_fraction）可加 --reference 基线请求。
# 输出目录必须是新目录；这不会调用 prepare/simulate/render。
$P -B -m world_model_dataset.experiment_construct \
  --request configs/dataset/v0_2/construction/cloth_more_overhang.json \
  --reference configs/dataset/v0_2/construction/cloth.json \
  --output output/new_cloth_condition_pair

$P -B -m world_model_dataset.experiment_material_bridge \
  --request configs/dataset/v0_2/construction/bridge_c_disabled.json \
  --output output/new_c_disabled_mapping

$P -B scripts/check_construction_examples.py --output output/new_construction_checks
$P -B -m unittest tests.test_experiment_construction \
  tests.test_experiment_api.ContractTests tests.test_experiment_integration_boundaries -v
```

已逐项检查上述入口：构造只调用 numpy/scipy/trimesh/Shapely 几何与 JSON；材料检查子进程
只导入 `input_contract.normalize` 及其合同依赖，没有导入或调用 native solver、material_entry.cli、
generate、prepare、simulate、package、preview。测试只选这些明确无物理/渲染的集合，
没有运行前任含 prepare 或梁观测审核的完整 NativeCacheTests。

独立复现 C 固定源码时，从主线共享对象库导出指定提交的 `experiments/material_response` 到新的
本工作区目录，再把请求中的 pinned_source 指向该目录；映射器会核对所有已跟踪 Python 源字节。
不能用任意目录冒充固定提交。

## 交付证据分级

机器可读核对摘要：[PHENOMENON_CONSTRUCTION_CHECKS.json](PHENOMENON_CONSTRUCTION_CHECKS.json)。

1. **构造代码已实现**：七类在上述适用范围内有实际几何/布局/约束规则，优先完成受限运动与悬垂后推广。
2. **配置与几何检查通过**：原场景七类各 baseline、尺寸变化、原区域重选、条件变化，共 28 配置；
   材料配置同时通过各自公开 native 输入合同，刚体通过实际 CPU recipe 编译。
   结果在 `output/construction_checks_delivery/results.json`；21 项相关测试通过。
3. **已有相近物理证据**：原 v7、被动布/梁/绳、B/C 缓存只能说明已有固定配置；未重新审核梁观测或补交付数字。
4. **新配置物理效果未验证**：没有新物理、渲染、收敛、训练准入或新数据目录。禁用 C 同样未验证。

旧检查失败目录原样保留；最终结果不掩盖超出构造范围的拒绝，也不把配置数量当作新增现象数量。

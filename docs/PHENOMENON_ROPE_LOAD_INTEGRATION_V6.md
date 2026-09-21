# C 绳索连接动态负载的主线采用（2026-09-22）

B 已在 v5 完成工程数据接入。本轮接入 C 的成功缓存，没有重新仿真、修改材料工作区、共享 schema/runner，或继续已暂停的模板工作。生成入口仍来自材料独立工作区，补丁未合并；本轮补齐的是主线缓存审核、诊断观测和统一读取。

## 来源与功能证据

来源 `Y:/isaacsim_work_rope_load`，分支 `feat/material-rope-load-20260922`，提交 `ca39d3875c933c9ad759a0e6c9c52fbf22d159e2`。采用 `output/rope_load_c/c_travel24_v1/episode`，保留 `c_smoke_v5/episode` 的 12 cm 目标未拉直结果，后者不计为成功样本。

原教师桌与相关完整对象网格保持原位。16 段 Newton cable，半径 3 mm、初始弧长约 0.419731 m，两只 6 cm 动态盒体质量分别 0.15/0.25 kg。前盒仅受有上限的 X 向阻抗外力；目标 −0.24 m、上限 1.5 N，0.3–1.1 s 拉动、1.1–1.6 s 保持、1.6–3 s 施力严格为零。源码循环通过 body_f 施加 COM 外力，每步保留碰撞及求解，没有以目标轨迹写入实际位姿。

主线独立核验两份缓存的状态、控制与实际执行器记录；重新计算端距/弧长比、起动时刻、位移、盒角穿透及连接间隙。成功缓存：

| 指标 | 结果 |
| --- | ---: |
| 端距/弧长比首次 ≥0.95 | 0.900 s |
| 最大端距/弧长比 | 0.999972 |
| 后盒相对落桌稳定状态沿 −X 超过 1 mm | 1.0167 s |
| 后盒末态相对稳定状态位移 | −22.18 mm |
| 前盒相对 t0 实际位移 | −206.53 mm |
| 内部连接最大间隙 / 端部连接最大间隙 | 0.1967 / 0.1970 mm |
| 前盒 / 后盒最低角相对桌面 | −0.463 / −1.232 mm |

目标与实际行程不同，前盒未被强制送达目标。两盒没有直接接触候选。双向 CPU 连接探针属于材料线已交付证据，主线没有重新运行探针。运动证据不能换算成拉力真值。

12/24 cm 的物理配置仅目标行程不同（另有描述文字变更），初始 native 状态、质量惯量、连接框架、碰撞形状和材料参数相同，原场景文件记录相同；拉动前最大位置差异 0.2694 mm 如实保留。这是一对已实现轨迹，不是 bit-exact 重复性或唯一反事实认证。

## 后端、表示与工程检查

实际后端为 **Newton 1.2.1 / Warp 1.13.0 SolverVBD**，Isaac 安装版本 `6.0.1-rc.7+release.42383.32955d8d.gl`；本 episode 不是 PhysX 求解。960 Hz、40 次迭代，60 Hz 保存状态，3 秒含首末态共 **181 帧**。每条命令、实际执行器状态、effort 流均 **2880 条**，记录每个物理步开始时的实际负载位姿/速度及外力。

主线逐帧检查全部段与负载的位姿、速度，以及几何载荷中的 body_q/body_qd/centerline/centerline_velocity，与 native 数组严格相等；世界 COM 独立重算允许 0.5 μm 绝对误差以兼容 float32 四元数运算。拓扑、连接参数、原生控制字段和时间步一致；相同保存时刻的实际执行器 trace 也与 native 状态一致。逐步核对力上限、无 Y/Z 力及力矩、释放阶段外力为零。

主线窗口接口现在允许实际执行器 trace 按物理频率存在于保存帧之间，与命令和 effort 相同；不插值、不降采样、不将目标位置替换实际状态。2.5–3 s 闭区间读取为 31 状态、480 命令、480 实际执行器记录、480 effort、6 观测；末态不补造下一步命令。

绳段仍保持原生胶囊/连接表示，两盒读取原生 BOX 半尺寸、局部形状变换和实际刚体位姿。没有将盒体误作绳段或改成一个软体表示。主线 26 项相关测试通过，材料公开入口 12 项合同检查通过；完整补丁在隔离的 `4b2449e` 基线导出目录中通过可应用检查，没有向 A/B 基线叠加不相容补丁。

## 交付与用途边界

[统一目录](../output/world_model_dataset/v0_2/phenomenon_rope_load_data_v6/catalog.json)含 **27 条 episode、13599 状态、771 帧观测**，四类数量为 12/7/5/3；保留旧布料替代历史。新 C 副本在 `phenomenon_rope_load_data_v6/delivery/c_travel24_v1/episode`，其源物理文件逐文件哈希保持相同，源 manifest 未变。

新增 31 帧 10 Hz、160×120 诊断 RGB、光轴深度及实例 ID，[关键帧](../output/world_model_dataset/v0_2/phenomenon_rope_load_data_v6/keyframes.png)覆盖 0、0.9、1.1、1.6、3 s。这些用于新接入抽查，不是完整材质视觉数据，也没有替代用户已接受的展示视频。

[采用审核](../output/world_model_dataset/v0_2/phenomenon_rope_load_data_v6/adoption_review.json)绑定成功/旧失败缓存、来源证据及固定配置状态数据政策哈希；[交付核验](../output/world_model_dataset/v0_2/phenomenon_rope_load_data_v6/delivery_audit.json)记录源文件保持和末段窗口读取。

工程结果为 `functional_cache_checked`。用途记录为 **`human_use_review_pending`**，建议方向为固定配置状态预测的 conditional 候选，并未授予 conditional 准入。已记录瞬时盒角穿透、连接间隙和运行间差异；未做收敛、重复性认证或实物标定。原生拉力、接触力和冲量均 unavailable，不支持这些量的监督；端距比例和起动时刻只作运动诊断。`training_admission=false`，没有训练、划分或发布。

## 复用入口

[采集配置](../configs/dataset/v0_2/phenomenon_rope_load_v6/collection.json)固定来源 manifest、审核哈希及相机。复用缓存时，在主线根目录执行，输出使用新目录：

```bash
PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
$PY -m world_model_dataset.phenomenon_collect \
  --batch configs/dataset/v0_2/phenomenon_rope_load_v6/collection.json \
  --output output/new_rope_load_data
$PY -m world_model_dataset.phenomenon_catalog \
  --index output/world_model_dataset/v0_2/phenomenon_cloth_gripper_data_v5/catalog.json \
  --index output/new_rope_load_data/index.json \
  --output output/new_rope_load_data/catalog.json --verify-streams
```

新物理运行使用材料线 `material_entry.py` 的 prepare → simulate → package 入口，准确参数见材料工作区 `experiments/material_response/scene_input/ROPE_LOAD_C_TRAVEL24.md`。新缓存需要重新审核，不沿用本次状态哈希的功能证据。可移植完整补丁为材料工作区 `output/rope_load_c/delivery_travel24/rope_load_c_full.patch`；底层生成代码尚未合入主线，不能把工程接入描述成单仓独立生成已完成。

A/B/C 的工程链路现均有交付；四类过程已有少量可读取样本，但不表示所有条件覆盖、数值质量和训练准入均已完成。模板扩展继续留给用户指定的支线，本轮不重复推进。

# 按现象生成的数据首批（2026-09-21）

后续进展见 [主线推进 v2](PHENOMENON_PROGRESS_V2.md)：两条稳定布料已用新 ID 接入当前目录，另补资产/材料复用。本文保留首批 14 条的历史状态，不改写旧布料的待复核标记。

本页按用户最新要求接替展示阶段优先级。已接受的视频保持原样；不再围绕视频打磨，不增加 split 或启动模型训练。目标是四类过程具备可重用生成入口、少量条件对照和共同读取链路。

## 首批设计

原 Classroom 静态几何保持原位。刚体与有限压板使用全部三个原场景几何组；布料与塑性使用完整原桌面／桌架；G01 绳索使用已交付的整个原管区对象网格。局部材料环境的范围显式记录，不冒充整个教室。每条都是新的物理轨迹；展示缓存不改写。

| 研究过程 | 目标／变量 | 两个条件 | 表示 |
| --- | --- | --- | --- |
| 整体运动 | 初始旋转对滚滑的影响 | 同一洋葱：Y 角速度 0／12 rad/s，平移同为 0.65 m/s | SDF256 刚体／源扫描网格 |
| 多体相互作用 | 改变参与数量 | 原有下层物体保持不变，3／4 个物体 | 每个刚体位姿、速度和原生接触流 |
| 几何受限 | 初始姿态决定通过 | 同一 22.5 cm 水壶平放／立放；同质量、摩擦、速度 | 刚体，COM 高度按原地面支承推导 |
| 体积材料响应 | 塑性冲击条件 | 同一红薯扫描试件，最低点距桌面 4／12 cm | Newton MPM 材料点、速度、Jp 等；无 Tet／原生表面 |
| 体积材料响应 | 有限加载、保持、真实卸载 | 同一弹性块和参考轨迹，压板力上限 50／150 N | PhysX 原生体积节点和 Tet；1 kg 压板实际状态及 effort |
| 薄片与布料 | 支承重叠程度 | 同一布片初始中心 X=2.73／2.83 m | 2925 个原生表面节点／5632 三角面 |
| 绳索与细长物 | 端部条件改变响应 | 同一 G01 绳中心线，自由／首整段固定 | 47 个原生胶囊段与 cable 连接；中心线另列为派生 |

以上 7 个目标的 14 条新实验均已生成，不是全覆盖本体或大矩阵。实际生成与失败记录在 `phenomenon_pilot_v1/batch.json` 和塑性修正批次中。目标结果不是筛选门槛；“生成”“记录完整”“物理质量认可”“训练准入”分开记录。两条布料明确标 `needs_stability_review`；其有限、可读状态不能证明桌沿接触已稳定。

## 可重用入口

`world_model_dataset/real_scene_batch.py` 保留既有资产列表、尺寸、姿态、速度、质量和材料输入，不改成逐件专用逻辑。材料侧统一使用 `scene_input/material_entry.py`，显式提供配置、runtime、mainline；不启动预览。有限压板复用已有原生体积求解和有质量、有限力阻抗控制器，将真实原桌面作为支承，不替换为隐藏实验台。

新主线 `phenomenon_pilot_batch.py` 读取 `backend=rigid/material/native` 的作业清单；一个作业失败后继续其他类别。已有成功输出以 manifest 哈希校验后跳过，部分失败目录保留；重新仿真必须使用新输出。历史 `phenomenon_batch.py` 完整保留，不改变其旧入口。

`configs/dataset/v0_2/phenomenon_pilot_v1/` 包含所有具体实验输入和 runtime。首轮塑性两条在准备前被严格配置接口拒绝：旧缓存带 `appearance_request`，它不属于物理输入。修正批次仅移除该展示字段，改用新 ID／目录；旧错误日志保留，没有把一次失败解释成物理质量不合格。

```bash
PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
$PY -m world_model_dataset.phenomenon_pilot_batch \
  --plan configs/dataset/v0_2/phenomenon_pilot_v1/generation_batch.json \
  --output output/new_phenomenon_pilot

# 已保存的塑性修正配置；不读取旧轨迹，不重新生成展示
$PY -m world_model_dataset.phenomenon_pilot_batch \
  --plan configs/dataset/v0_2/phenomenon_pilot_v1/retry_plastic.json \
  --output output/new_plastic_pilot

$PY -m world_model_dataset.phenomenon_collect \
  --batch output/new_phenomenon_pilot/batch.json \
  --output output/new_phenomenon_data
```

完整复现使用 `generation_batch.json` 的 14 条修正后输入；原 `batch.json` 和 `retry_plastic.json` 保留实际执行历史。允许范围不等于已经验证的连续参数域。每条新配置仍须独立检查缓存。

## 共同读取与记录完整性

共享 schema 接受 `physics_kind=surface`。`open_episode` 不再需要材料线 `SurfaceDraftEpisode` 补丁。`soft_geometries` 接受 volume／surface，`geometries` 可读绳索的段数组载体。两者都保留 NPZ 的原字段，验证哈希、字节数、时间／步号与数值有限性，不造 Tet、不把中心线当材料拓扑、不把 MPM 点强行重建为原生表面。

```python
from world_model_dataset.causal_loader import open_episode

ep = open_episode('/absolute/path/to/data/<episode_id>/episode')
states = ep.states()
controls = ep.controls()                 # 被动实验为空迭代器
observations = ep.observations()         # 同步 RGB、光轴深度、编号
surface = ep.soft_geometries('cloth')    # 布料：节点／速度／三角面
points = ep.soft_geometries('mpm_block') # MPM：材料点及内部量
cable = ep.geometries('segment_0')       # 原生段位姿／速度和派生中心线
topology = ep.soft_topology('segment_0') # cable 原生连接，非 Tet
```

不同示例需使用对应实例 ID，不应在一条 episode 上同时调用全部四种接口。有限压板使用原有 `controls()`、`actuator_states()`、`actuator_efforts()`；effort 是有上限的控制外力，不冒充接触反力。

`phenomenon_collect.py` 在新目录复制物理缓存，原样保留源 manifest 和源状态哈希，增加缓存审核、明确设计变量和 10 Hz／160×120 单机位诊断 RGB-D／分割。原始物理状态保留全记录速率。观测以原几何进行透视遮挡和深度计算，包含近平面裁剪；静态环境只栅格化一次。不会启动求解器，没有隐藏地面。

观测使用明确的诊断颜色，不是已接受展示的完整贴图外观。布料用原生三角面，体积弹性体用缓存的显示表面并保留权威 Tet 状态，绳索用原生碰撞胶囊三角化；MPM 以记录的初始粒子支持半径显示 glyph，深度是这些 glyph 的可见表面，不是连续体真实外表面。如此保留可读取链路，不将派生显示误标为物理真值。

数据副本只有在状态、初态、控制（适用时）、观测、标注、结果完整且默认 `open_episode(require_complete=True)` 实际读通之后才标 `lifecycle=completed`。该状态仅代表记录完整，`training_admission=false`；不授予数值收敛或全面碰撞正确性。缺失的布料接触力、绳拉力、软体冲量继续 unavailable。

## 实际交付与证据

数据目录：`output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/`。入口为 [index.json](../output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/index.json)，逐条检查和条件差异见 [verification.json](../output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/verification.json)，三时刻缓存抽查见 [keyframes.jpg](../output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/keyframes.jpg)。每条子目录包含 `episode/`、`cache_review.json` 和 `result.json`。

实际共 14 条、7 组对照：整体运动及多体 6 条、体积材料响应 4 条、布料 2 条、绳索 2 条。保存 6062 条状态、310 帧观测、10 条控制命令、1922 条执行体状态和 1920 条实际施力记录。全部默认核心读取通过，观测时刻与原生状态一致，派生 manifest 的 `capture_hz=10` 已逐条验证，源物理状态哈希不变。

有限压板的 50／150 N 条件分别达到最低高度 0.185461／0.172252 m，撤离后末态高度为 0.195770／0.195773 m；压板末态净空分别为 0.043383／0.043417 m。两条实际 effort 峰值等于各自上限；撤离阶段约 2.30 s 首次超过 4 mm 净空，证明实体执行体确实离开试件。该结果是压缩恢复对照，不能代替弯曲实验。

塑性 4／12 cm 落高的末态 Jp 均值分别为 0.963829／0.873733。它是原生材料内部量；重力和桌面支承仍存在，不把末态形状当作完全卸载后的塑性应变测量。

38 项相关测试通过，记录在 `output/world_model_dataset/v0_2/phenomenon_pilot_tests_final.log`。缓存与关键画面已抽查。两条布料仅记录完整，仍待独立稳定性复核；整批 `training_admission=false`。本次检查不宣称全面数值收敛，也不把诊断颜色观测称为完整材质观测。

## 缺口与接续

已经推进：有限压板压缩—保持—撤离，并从实际接触净空验证卸载。没有把软块压缩称为弯曲。

待接入：弹性弯曲回弹的实体夹具约束、有限夹头—布料双向耦合、绳索与动态负载连接／传力。具体原场景区域、坐标、规模、初态、执行时序、记录内容见 [材料需求交接](PHENOMENON_PILOT_MATERIAL_REQUEST.md)。材料三角网格输入已完成，不能继续拿它当阻塞。布料稳定性独立任务工作区 `/mnt/y/isaacsim_work_cloth_stability`，分支 `fix/cloth-edge-stability-20260921`，不涉及动态夹头；它正在核对原生速度尖峰及接触输入，尚未获得新缓存结果，不将网格朝向线索提前定为根因。主线 GPU 实验全部结束后已明确交还该任务。

黏弹性、流体、损伤与分离继续暂停。没有训练、split、额外展示汇编或全面数值研究。

# 可复用现象模板与条件对照 v4（2026-09-22）

材料线继续推进 B 布料拖动与 C 绳索负载传力。主线独立完成真实资产实验模板、两组新条件对照，以及跨物理表示的时间窗口读取；没有修改材料线工作区。

## 实际交付

[统一目录](../output/world_model_dataset/v0_2/phenomenon_templates_data_v4/catalog.json) 当前选用 **23 条 episode、11615 条状态、587 帧观测**，包括 v3 的 19 条及本轮 4 条。四类条数为整体运动 12、体积响应 7、布料 2、绳索 2。旧布料的两条替代关系继续保留。所有数据仍 `training_admission=false`，C3/C4 未整体完成，不新增 split、不训练。

每条新增记录均为 2 秒、481 个物理状态、21 帧 10 Hz 诊断 RGB/光轴深度/实例编号。沿用已接受的低成本方式，没有重做旧视频或生产完整贴图视觉训练集。原 Classroom 桌面、地板和椅架等导出网格原位参与碰撞；主体使用原扫描资产的 SDF 表示，没有改成单凸包。两组均为有限初始动量的自然演化，控制为 `none`。

| 对照 | 保持不变 | 实际结果 |
| --- | --- | --- |
| 洋葱低/高摩擦 | 0.105 m 尺寸、0.12 kg、初速 0.45 m/s、初始角速度零、位置/姿态、原桌面 | 静/动摩擦输入分别 0.05/0.03 与 0.9/0.7；2 秒 X 位移分别 0.3441/0.02043 m，两者均留在桌面上 |
| 同姿态水壶尺寸 | 直立姿态、0.55 kg、初速 2.2 m/s、初始 XY、摩擦、原椅架 | 最大尺寸 0.12 m 的水壶在约 0.554 s 整体越过 x=0.6 m 出口；0.225 m 水壶与原 `surroundings` 组产生 45 条接触点记录，停止在入口限制处，未通过 |

水壶两条在限制段均保持于图集采样的 0.2 m 宽带内，没有绕开椅架再被标为通过。实际初态高度约 0.0921/0.1727 m，最大尺寸不等于竖直高度。缩放保持质量，因此密度、惯量及贴地放置高度相应变化，记录为派生量，不称作相同实物材料的缩放实验。摩擦对照改变的是主体输入系数，环境系数不变；不将其当成经过测量的接触有效摩擦。

四条选用记录的原生接触最小 separation 约为 −0.385、−0.334、−0.745、−0.789 mm（低摩擦、高摩擦、小水壶、大水壶）。这只是缓存诊断，不授予连续碰撞或数值收敛结论。关键画面已抽查：[四条对照](../output/world_model_dataset/v0_2/phenomenon_templates_data_v4/keyframes.png)。小水壶在单视角中最少约 16 个可见像素，适合本轮诊断，不宣称精细外观监督。

## 模板入口

[设计配置](../configs/dataset/v0_2/phenomenon_templates_v4/design.json) 声明原场景基础配置、主体 ID、适用资产列表、共同初态及少量显式条件。`phenomenon_templates` 通过同一个 `real_scene_batch.recipe` 展开，不按资产写分支，不改环境。支持主体尺寸、质量或密度、位置/姿态、初始平移/旋转及摩擦/恢复系数；超出支持字段、非有限数值、非法尺寸、重复 ID 和环境修改会拒绝。

适用资产列表是调用者选定的实验范围，不是全库资产的物理准入。当前真实运行使用洋葱与水壶，旧批次仍提供同一底层入口接洋葱/红薯的证据；本轮资产列表展开与独立配置验证也有单元测试。

编译只运行 CPU 几何准备：检查场景/资产网格哈希，按原表面射线推导支承高度，输出预检、派生物理参数及生成清单。清单固定场景、资产、加载器、生成代码和基础配置的哈希；生成前及各作业启动前检查变动，发生变动时要求重新编译或使用新目录。预检不宣称任意资产的完整初始相交或扫掠净空已成立。

最终代码从空目录再次编译出的 4 份场景配置，与实际选用作业的配置逐文件哈希相同，见 [重编译核对](../output/world_model_dataset/v0_2/phenomenon_templates_v4/recompile_check.json)。最终固定清单位于 `phenomenon_templates_v4/compiled_final/batch.json`；没有为了这次重编译再跑物理。

```bash
PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
# 主线工作区内执行；每次编译/生成使用新的输出目录，启动前检查并行作业。
$PY -m world_model_dataset.phenomenon_templates \
  --design configs/dataset/v0_2/phenomenon_templates_v4/design.json \
  --output output/new_conditions/compiled
$PY -m world_model_dataset.phenomenon_pilot_batch \
  --plan output/new_conditions/compiled/batch.json --output output/new_conditions/physics
$PY -m world_model_dataset.rigid_template_review \
  --batch output/new_conditions/physics/batch.json --output output/new_conditions/pair_review.json
$PY -m world_model_dataset.phenomenon_collect \
  --batch output/new_conditions/physics/batch.json --output output/new_conditions/data
```

上述审核接受本轮两类成对条件，检查输入仅有声明字段变化、原场景网格相同，并测量轨迹、原生接触及出口位置。结果不符合设计意图时仍保留记录，不凭期望结果筛掉样本。其他现象的生成可继续使用原入口，不暗示该审核适用于任意现象。

## 保留的初次尝试与一次修正

最初沿用 0.65 m/s 初速运行摩擦对照；低摩擦样本滑出桌沿，落地发生约 13.1 mm 短暂负接触间隙，后段也离开原相机范围。随后为两条摩擦样本共同采用 0.45 m/s，重新生成这两条，让所需对照维持在原桌面内。没有改场景、单独干预低摩擦样本或扩大参数扫描。水壶对照未重跑。

初次物理与采集记录完整保留在 `phenomenon_templates_v4/physics/`、`phenomenon_templates_data_v4/index.json`，不计入当前 23 条目录。正式采用 [selected_index.json](../output/world_model_dataset/v0_2/phenomenon_templates_data_v4/selected_index.json)，引用原两条水壶及 `supported/` 中的新摩擦对照。最初 `pair_review.json` 的障碍分组计数使用了错误组名，已由明确区分 `support/room/surroundings` 的复核纠正；正式依据为 [selected_pair_review.json](../output/world_model_dataset/v0_2/phenomenon_templates_data_v4/selected_pair_review.json)。这些是审核/设计修正，不是已完成覆盖的额外扩量。

## 统一时间窗口读取

新增 `causal_window.read_window`，以闭区间选取真实保存的时间戳，不插值、不按声明的求解频率伪造帧。返回原生状态、形变数组、静态几何、控制命令、实际执行器记录和观测；保留标定索引与 unavailable 状态。刚体局部网格/原场景网格与带时间的形变数组分别处理。末态没有后续施力/命令时不会补造，命令目标不当成实际位姿。

```python
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.causal_window import read_window
ep = open_episode('/mnt/y/isaacsim_work/output/world_model_dataset/v0_2/phenomenon_beam_data_v3/beam_load_delivery/episode')
window = read_window(ep, 0.2, 0.6)
# window['states'], ['geometries']['Beam'], ['controls'],
# ['actuator_states'], ['actuator_efforts'], ['observations']
# 原生 topology、材料与连接继续通过 ep.soft_topology / ep.resolved_inputs 读取。
```

真实缓存检查在同一 0.2–0.6 s 窗口完成：刚体/梁各 97 状态，塑性 193，布料/绳索各 25；各有 5 个观测时刻。计数差异来自实际保存频率，不强行统一。塑性保持材料点及内部量，梁保持 Tet 节点，布料保持网格节点/速度，绳索保持段与连接表示。[窗口验证](../output/world_model_dataset/v0_2/phenomenon_templates_v4/window_review.json)记录了各自字段与数组形状。

23 项相关测试通过，见 [测试日志](../output/world_model_dataset/v0_2/phenomenon_templates_v4/tests.log)。[采用审核](../output/world_model_dataset/v0_2/phenomenon_templates_data_v4/adoption_audit.json)核对了新增源 manifest 未变、状态与派生观测副本一致、实际解析物理输入相同、每个观测时刻主体可见；统一目录逐条验证状态与观测流计数。

## 后续工作

主线的下一步是将同样的配置展开和输入固定机制接到已验证的塑性/梁材料入口，优先复用已有条件记录，按实际缺口补少量条件，不再叠加相似刚体样本。B/C 由材料线推进，交付后沿用现有状态/控制分离、缓存审核、低成本观测和目录接入流程。完整材质观测、规模化与用途准入各自登记，当前不靠反复渲染推进。

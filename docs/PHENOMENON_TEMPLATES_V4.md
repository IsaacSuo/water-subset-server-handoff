# 可复用现象模板与条件对照 v4（2026-09-22）

最新推进（2026-09-23）：入口截至 `7f84038` 的自动构造与校准能力已采用，见
[主线采用状态](PHENOMENON_CONSTRUCTION.md)。当前统一入口为 v10：保留 v9 的 45 条，新增绕接的 1 条仅物理状态记录，
共 46 条；包含明确标注的未达预期过程，数量不代表成功覆盖。B/C 和重排布局推导已接入，梁已有实际加载撤离证据，
拖布形成自由/受阻/低力匹配对照。绕接采用固定零面积面接收语义，复用材料线已有运行；没有新增观测。
下一缺口为塑性接触前网格速度组装的倒数分母来源，以及绳索实际求解阶段记录。
材料诊断 `91a72d9` / `d6902c7` 已缩小范围但未修复，详见主线采用状态；目录与用途资格不变。
流体仍在主线范围外，不阻塞现有四类过程。下文 v7 与 v4 记录保留为历史，不代表当前未完成项。

## 主线续接：条件采用 v7 与构造复用优先级（2026-09-22）

本节覆盖下文 v4 时的“当前”和后续队列，不改写历史证据。接手时主线为 `1c59689`，
入口支线为 `f6fe6a8`，A/B/C 来源仍为 `147bf4e` / `2f12001`（物理 `f985b5c`）/ `ca39d38`。
路线图与实现状态文件有他人未提交修改，本轮保留原样；本节承接它们已有的 C3/C4 目标，
不另立路线图。主线不承担入口构造器或求解器开发。

### 实际到达的位置

四类过程已有代表性物理证据：滚滑、多体重排、姿态/尺度约束通过、塑性冲击、加载卸载、
悬垂/拖布、被动绳索及松弛到拉直后负载运动。A 的加载/高位和 B 的自由/受阻/低力已经
形成初步对照；C 的 12/24 cm 记录支持行程差异下的实现轨迹比较，仍缺同配置无驱动基线。
这些足以推进有限范围的实验复用，不意味着现象目录穷尽、材料标定或数值收敛。

必须分别报告：后端过程支持、对象/区域到实验的构造、执行及统一读取、具体用途资格。
v4 的资产展开和支承高度推导是局部复用；七种实验入口主要补齐执行、输入绑定和数据工程。
调用者仍需给大量坐标、约束与时序，不能据此称完整实验生成已完成。B/C 尚未进入七种入口；
A/B/C 仅缓存与主线工程链路已接入，底层生成代码未全部合仓。原位三角网格不再是共同阻塞。

### 已采用的数据及对照关系

以 v6 全部 27 条为基线，采用入口支线已独立核实的四条新条件，
[当前统一目录 v7](../output/world_model_dataset/v0_2/phenomenon_conditions_data_v7/catalog.json)
共 **31 条、14887 状态、873 诊断观测**，四类为 **13/8/6/4**。
既有条目和布料替代历史不变；新增完整 episode 逐文件相同地复制到主线 delivery，未重跑物理。
[固定采用记录](../configs/dataset/v0_2/phenomenon_conditions_v7/adoption.json)
绑定源 manifest、审核、工作流、基线状态和目录哈希；复制与原生读取证据在 v7 的 `evidence/`。

| 新条件 → 已有基线 | 此次补充什么 | 必须保留的解释边界 |
| --- | --- | --- |
| `multibody_lower_drop` → `rearrange_count_3` | 同三体布局下一个物体低 40 mm 的初态对照 | 势能、接触时刻与后续重排均可变化；不是物体数量对照 |
| `plastic_yield_8000` → `plastic_drop_12_v2` | 屈服参数对冲击形变及原生 Jp 的响应 | 不从末态残余单独认定塑性，不声称真实材料辨识 |
| `cloth_offset_278` → `cloth_edge_273_stable_v1` | 支承重叠变化，补充悬垂条件 | 属于被动 20 kPa 配置，不与主动拖布 200 kPa 合称同材料对照 |
| `rope_both_ends` → `rope_end_first_clamped` | 末端边界条件改变 | 固定有限末段同时改变可动质量，不是拉力真值或负载传力对照 |

每组 API 非干预输入与场景哈希相同，API 基线状态哈希对应上表主线基线；这不授予唯一
反事实或 bit-exact 重复性资格。本轮只加密已有条件，不计新增现象。31 条均完整读取状态与
观测，新副本另读首/中/末原生窗口；继续 `training_admission=false`，不启动训练或扩 split。
诊断观测不等于带原材质的视觉数据。C 的 `human_use_review_pending` 保留，不自动提升用途。

### 下一阶段只推动三件事

1. **入口支线：先完成两类真正的构造复用。** 选择几何约束通过与布料悬垂，覆盖刚体和
   柔性两种构造问题；不一次扩七类。输入为适用对象描述、原场景/候选区域引用、现象条件
   （例如通过方向/姿态候选，或布的支承重叠比例/悬垂方向）和已批准数值 profile。
   由入口推导原支承、限制段与进出空间、对象落位/姿态、布面摆放及净空、必要原碰撞对象，
   输出可审阅的原生配置、每项推导依据、适用性失败原因及条件关系，继续用现有执行链路。
   调用者可选区域和条件，但不应重写整套世界坐标、布局和后端配置。首批限现有 SDF 刚体和
   已支持矩形布；不承诺任意扫描柔性体。真实资产碰撞表示与原场景变换保持原样。
   验证分别换一个适用对象/布尺寸、换一个有证据的原区域，另保留一个不适用案例；无需
   对象×区域全矩阵。不适用时返回具体几何原因，不能缩放环境或暗中改对象来凑成功。
   同一对照在推导后检查非干预输入；跨区域属于复用证据，不能冒充单因素配对。
   输出构造配置后至少跑少量独立实现，检查真实通过/受阻或悬垂轨迹；结果不符预期仍保留。
2. **入口与材料支线：接通 B/C，C 只补一条有价值的基线。** 入口负责主动实验请求到对应
   材料公开入口的映射和恢复/登记；材料 B/C 负责已选有限执行体、连接、实际控制流和能力
   限制。具体物理需求见[材料需求的当前增量](PHENOMENON_PILOT_MATERIAL_REQUEST.md)。
   先接 B 现有自由/受阻/低力，不重做已接受画面；C 优先同配置无驱动基线，保留旧 12 cm。
   不要求同场混合或全部代码先合仓。反力缺失不阻塞状态数据接口，也不能用 effort 冒充。
3. **主线：按过程与对照组织采用，保持 C3/C4 连续推进。** 本轮 v7 已完成条件整合；
   下一采用单位是“构造规则及其跨对象/区域证据”或“补齐解释缺口的对照”，不是更多条数。
   主线决定目标、变量和用途边界，按证据接入新缓存，分别记录构造、执行、数据与用途状态。
   若某支线受阻，继续独立构造案例或已有对照整理；不等待全部收尾，不把全面验收当日常统筹。

这些是供用户安排现有支线的具体需求，本轮未创建或联系其他 agent，也未修改其他工作区。
入口梁仍停在 `audit_complete / observe`；便携包最终证据未完成。交付脚本仍指向旧 v4，
后续应接收调用方给出的当前目录，保留已有 ID/替代历史；本轮已直接从 v6 构建 v7，未采用
会漏掉 B/C 的旧候选目录。上述工程收尾不替代构造复用，也不阻塞本轮条件采用。

---

以下为 v4 历史记录。

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

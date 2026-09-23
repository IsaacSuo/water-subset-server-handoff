# 现象请求构造层与材料公开入口对接

## 主线采用状态（2026-09-23，v10）

入口支线截至 `7f84038` 的构造、校准、审核与诊断代码已采用。对象/原区域/现象条件可以在各自声明的边界内
推导布局，B/C 不再仅转交完整配置。梁增加来源绑定的无加载探测，从自重变形推导压板路径；这会增加校准运行，
并非无仿真的通用几何公式。材料后端、真实资产和运行环境仍在外部工作区，不称全部生成能力已合仓。

当前统一入口为 [v10 catalog](../output/world_model_dataset/v0_2/phenomenon_wrap_data_v10/catalog.json)，
来源、逐例用途和校验记录见 [v10 adoption](../configs/dataset/v0_2/phenomenon_wrap_v10/adoption.json)；
前批采用记录保留于 [v9 adoption](../configs/dataset/v0_2/phenomenon_layout_v9/adoption.json)。
保留 v9 全部 45 条及替代历史，新增 1 条绕接物理轨迹，合计 46 条、22642 状态、1437 诊断观测，四类 18/11/10/7。
其中 45 条具有原有完整记录，绕接为显式 `record_scope=physics_only`：121 帧状态、零观测；不把它称为完整视觉样本。
源 manifest 仍为 draft 且逐字节保留。目录读取器只对显式状态条目放开观测要求，仍检查其状态/控制/结果记录及完整时段；
失败有限前缀不能借此进入完整运行目录。默认读取继续要求完整记录，窗口 CLI 根据目录 scope 读取。
旧碰撞未向前传播及两条梁未持续加载的记录也保留，
明确为 `negative_outcome_only`，不算对应现象的成功证明。无加载梁探测另存辅助证据，不计入现象 episode。
塑性非有限中止只保留诊断来源，不计入目录条数；绕杆旧 prepare 拒绝保留为历史，新固定后端已解锁实际证据。

绕接后端固定 `7011d17`，原布局与物理输入不变；复用材料线已完成的 960 步运行，入口及主线均未重跑。
完整原输入、114 个精确零面积面不参与碰撞的政策及原面/对象映射均随副本保留。
用途仅限本例单 U 绕接的定性说明，保留 0.160 mm 采样表面重叠、1.152 mm 最大连接间隙。
原杆为不闭合网格，采样距离、碰撞候选均不能证明帧间无穿透或接触反力。

| 现象 | 当前可表达的内容与边界 |
| --- | --- |
| 受限通过、布料悬垂 | 保留既有跨区域实例；不重复仿真 |
| 滚动与滚滑 | 平移与转动；不声明纯无滑滚动 |
| 多体重排 | 接触关系改变、滚落；不保证重新稳定 |
| 碰撞传播 | 修正支承姿态的两体正向速度传递；不是任意多体链保证 |
| 梁加载—保持—撤离 | 保存样本近接、额外下挠、撤离间隙及回弹；仍有抖动，无原生接触力 |
| 有限夹头拖布 | 同一 200 kPa 材料、同初态的自由/受阻/低力对照；与被动 20 kPa 不混同 |
| 被动绳索悬垂 | 固定首点、自由端下垂；不是张力真值 |
| 绳索有限负载 | 负载运动成立；轴向自旋合理性待材料线核查，质量准入挂起 |
| 原杆绕接 | 原杆参与碰撞候选，本例保持 U 形；物理状态可读，观测缺失、质量限制保留 |
| 塑性冲击 | 新盒体第140步最早观测异常在接触前网格速度组装；逆质量 Inf、速度 NaN，精确分母尚未捕获，未修复 |

新正例限于现象定性解释，拖布三组可作本次条件比较；不授予材料标定、数值收敛或训练资格。
新增条目的 `mainline_assessment` 明确标注解释范围与限制；既有条目用途不被本轮批量升级。
诊断观测和少量原材质抽查不等于完整材质视觉数据集。主线独立复算梁和拖布结果，验证复制哈希与统一窗口，
v9 复跑 55 项构造和执行接口检查；v10 另通过 24 项相关回归，独立重算 121 帧绕接几何、原面映射并读取首/中/末窗口。
既有 v9 流检查由未变的目录哈希绑定继承，未重复解码全部旧观测；未运行新物理或重做已接受画面。

**绳索解释修正：** 端部接负载的角向刚度/阻尼关闭，但绳段间存在非零角向约束，作用于完整相对旋转，
包含扭转。不能把旧记录中的零值概括为整条绳没有扭转约束。轴对称外观不显示自旋，角速度大也不直接等于能量大；
原因与合理性仍待材料诊断，不在主线猜定。

材料诊断更新：塑性 `91a72d9` 一次固定输入复现的 t0–139 有限前缀与前次逐位一致；
最早观测异常位于 `_compute_unconstrained_velocity` 输出，早于接触热启动及应力求解。
第140步局部网格节点 485/491 逆质量 Inf、速度 NaN；编号不是跨步固定节点，也不是粒子编号。
尚未捕获 `free_velocity` 的质量积分分母，不能将零质量写成已证实根因。
下一项限定为该实际调用前后的质量积分、速度积分及节点对应关系，不先改材料、尺寸、时步或加入质量下限。
证据：[塑性定位](../../isaacsim_work_plastic_diagnostics/experiments/material_response/scene_input/PLASTIC_BOUNDARY_LOCALIZATION.md)。

绳索 `d6902c7` 为离线时序审计，未新增物理。接触力和关节力均为接受姿态上的步末重新求值，
不是实际迭代冲量；t0 接触零值未采样，只是占位，不能用作首区间真实积分端点。
未发现接触容量溢出或已保存转矩的索引/叉乘错误；统计口径不匹配及步内信息缺失已确认，求解器故障尚未定位。
早期负载接触仅是自旋激发候选，尚未完成因果角冲量核算；旧/新缓存末段约126–133/72–75 rad/s的差异不算改善。
最小后续方案为前32步及指定差额窗口的实际求解阶段诊断；40次优化迭代不能当作40个物理子步累加力矩。
证据：[绳索时序审计](../../isaacsim_work_rope_diagnostics/experiments/material_response/scene_input/ROPE_TIMING_AUDIT.md)。
以上仅更新诊断认知，诊断模块未在本轮合入，v10目录、旧来源和用途资格均不变。

零面积面接收及绕接入口链路已接回。下一步由材料线捕获塑性倒数分母及绳索实际求解阶段信息；
入口线在后端交付后接回现有构造验证，不继续重跑已接入绕接。
主线维护对象/区域适用范围、条件比较和逐例用途，不另扩现象或参数矩阵。流体仍独立于本轮主线。
既有 C 禁用驱动缓存不因本次接入而消失，但尚不在本次 12 条新目录内，不能把旧基线冒充新布局的匹配对照。

以下为历史采用和交付记录；其中旧状态不覆盖本节。


## 历史主线采用状态（2026-09-22，v8）

主线已采用支线 `4441bdc`、`cca88b1`、`f8484b4` 的代码，分别对应本地提交
`e844424`、`8582a72`、`97055df`，未推送。采用定位为**有限范围的构造初版**，不是原七类目标全部完成。
受限运动和布料悬垂已各有一条跨区域的实际运行；其余规则保留各自有限构造证据，不借这两条结果
授予全部对象/区域组合的物理验证。多体重排构造、B/C 新对象与区域布局推导仍待入口线推进。

最新数据为[主线 v8 目录](../output/world_model_dataset/v0_2/phenomenon_construction_data_v8/catalog.json)：
33 条、15669 状态、945 诊断观测，四类 14/8/7/4。v7 的所有条目及替代历史保留；新增两条
完整副本位于 v8 `delivery/`。准确来源与采用边界见
[采用记录](../configs/dataset/v0_2/phenomenon_construction_v8/adoption.json)。
主线独立重读并重算两条轨迹结果，检查源哈希、复制一致性与统一窗口，复跑当前 27 项无仿真检查。
没有重跑物理或修改相机/材质；既有原材质抽查只作实验解释，不等于完整材质视觉数据集。

刚体结果支持“经过所选喉部”，没有椅架接触，不作为受阻或碰撞传力证据；布料支持“原边柜上悬垂”，
局部残余抖动仍保留。两条用途维持 `human_use_review_pending`，不自动训练准入。
自动相机遮挡已作为构造规则问题修复，旧失败观测和缓存均保留；相机重构没有改变物理输入。

构造/执行框架代码已进入主线；材料后端、真实资产、固定 C 源快照和运行环境仍分别引用外部来源，
不是单仓独立生成全部能力。依赖见 `requirements-construction.txt`；本轮主线检查通过 PYTHONPATH
使用入口工作区已有的 `.construction-deps`，没有修改材料环境。
流体保持主线范围外，用户另行安排材料线独立研究；本轮未联系材料线或更改其任务。

以下保留支线交付时的历史状态。历史“未合入主线”与 v7 指针不覆盖本节的采用状态。

**后续状态**：主线 `2b17ccf` 已采用初版及两个跨区域实际案例，目录为 v8、33 条。
新一阶段自动布局、实际运行和失败交付见 [AUTOMATIC_LAYOUT_COMPLETION.md](AUTOMATIC_LAYOUT_COMPLETION.md)。
重排、有限夹头拖布、有限负载连接已新增布局规则与代表运行；绕杆已实现有限规则但原网格被材料 prepare 拒绝。
下文保留初版历史合同与交付范围；其中“重排未完成”“B/C 仅转交”描述的是初版，不是当前新构造器状态。


后续两例实际执行见 [CONSTRUCTION_PHYSICS_TRIALS.md](CONSTRUCTION_PHYSICS_TRIALS.md)：
各运行一次新物理，刚体通过所选喉部，BlueWall 布料形成悬垂；相机构造问题已修复。
下文“构造初版”检查统计保留其历史阶段含义，不代表后续物理试验仍未执行。

本支线从只读核实的主线 `bee9e62` 建立于 `Y:/isaacsim_work_construction`，分支
`feat/phenomenon-construction`。有选择地保留 `f6fe6a8` 的执行编排、输入合同、
来源固定、条件记录、缓存比对与失败恢复；新增对象/区域/现象条件构造层。
没有合并旧支线的主线文件，没有复制主线未提交指导文档。

## 初版交付时目录与不变项（历史）

初版交付时目录为主线 `output/world_model_dataset/v0_2/phenomenon_conditions_data_v7/catalog.json`：
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

本次交付是**有实际能力的构造初版，尚未完成七类原目标，未按完整目标合入主线**。
原目标中的六类有以下有限规则；多体重排尚缺构造。另保留独立的多体碰撞传播能力，不能替代重排。

| 类别 | 调用方必须决定 | 系统实际推导 | 适用边界与失败解释 |
|---|---|---|---|
| 几何受限运动 | 资产、尺寸、质量/密度、本地朝向、原区域、初速度、passable/obstructed 几何类别 | 读取对象包围盒；按对象高度裁剪原三角网格；求各截面共同的直线通道、入口位置、支承、喉部、出口余量与有符号净空 | 水平支承、+X 直线通道、固定初始朝向。曲线路径、无共同通道、没有限制几何、入口/出口不足、类别与实际间隙矛盾时明确拒绝。通过/卡住仍是未验证物理结果 |
| 布料悬垂 | 矩形尺寸或已有矩形静止网格及显式尺度、20 kPa 被动 profile、区域、悬出比例 | 读取对象尺寸及拓扑；求实际水平支承边、支承面积比例、对象世界变换、接触间隙、生成矩形的 cells、悬空长度与下方预留空间 | 单连通近水平支承和直边；选择最接近世界 +X 的支承外向边，分析坐标变换不改变场景。已有网格须平展、无孔、无重叠、矩形边界；保留拓扑，不重采样。非矩形衣物/曲面/动态 attachment 不支持。支承、净空、区域或节点预算不符分别拒绝 |
| 滚动与滚滑 | 资产、本地朝向、尺寸/质量、初速度、spin_ratio | 支承放置、对象有效高度半径、+Y 初始角速度、原场景初态包围盒检查 | 水平面 +X 初始运动；半径是包围盒近似，可研究不规则物体响应，但不保证纯滚动、无滑或对象一定滚起。没有支承/初态碰撞/区域不足拒绝 |
| 多体重排（原目标，构造未完成） | 目前仍须提供完整布局及后端配置 | 仅保留旧完整配置执行入口，不自动推导重排初态 | 缺少考虑支承关系的不稳定堆放与沉降重排布局规则；构造请求返回 `construction_missing`，不转为碰撞传播 |
| 多体碰撞传播（额外保留） | 两个以上对象列表及各自尺寸/质量、相对间距、首体初速度 | 按实际尺寸排列 +X 碰撞链、间距、每体支承放置与下游余量 | `multibody_collision_propagation`，单排初速度传播。对象数不足、重复 ID、整排放不下、初态穿插或无完整支承拒绝；不计作多体重排完成 |
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
物理结果必须来自对应新配置的 native 缓存；构造初版阶段没有这样的新证据，后续两例见上方独立报告。

## 条件、来源与恢复

执行合同升级为 `phenomenon-experiment/2`：`control` 描述控制方式和启用状态，例如
`{"mode":"impedance_control","enabled":true}`；`actuation` 描述实体，例如
`{"kind":"finite_load","instance_ids":["front_load"]}`。被动对象为 `none` 和空实体列表。
同一有限负载禁用控制时只改变 enabled，实体身份不变。当前后端只支持其已有阻抗控制，
其他控制方式明确拒绝，不能因为字段分开便宣称支持新控制器。
v1 文件读取时迁移这两项元数据，原 native 物理输入保持等值；不改写旧文件、缓存或账本。
旧输出的来源哈希仍指向旧代码，更新后须重新构造到新目录，不能直接续用旧来源封存。

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
不生效目标。统一入口使用 `control.enabled=false`，实体仍为 `finite_load`，不以位移 0 冒充关闭。
主动 0–0.3 s 已有保持外力，因此禁用对照的拉动前差异不能全部归因于随机变异。
材料线随后完成 `d8a4c4b415ae283f2ed8d8903ed2274bb1ebd40c`：
`output/rope_load_c/c_disabled_v1/native` 与 `episode` 现已存在。入口线已只读核对交付哈希，
统一读取 181 状态及 2880 条 command/actual/effort，并逐条确认主动力、力矩为零且 target_active=false。
共同稳定参考窗口仍未通过，用途保持 human_use_review_pending，training_admission=false。
相对 `05aabd3`，交付版本修复 package 重复独占写入 annotations 的问题，附加审核脚本和证据；
求解器与 enabled 输入语义没有变化。现有主动 `ca39d38` 与禁用 `05aabd3` 入口绑定均未自动升级，
也未重新封装或重跑 C。若以后用新版封装，须显式选版本并重新固定来源，不能把旧封装缺陷静默替换掉。

为避免吸收材料工作区后续修改，C 的两版公开源从已知提交只读导出到本工作区
`output/external_sources/rope_ca39d38` 和 `rope_05aabd3`，逐文件与 git 提交核对。
入口、配置、运行时和碰撞网格均固定哈希；不改材料工作区、不开发求解器、不合并材料代码到主线。
公开源若漂移且未提供准确固定版本导出，桥接明确拒绝。

**B/C 的构造状态均为“仅转交”。** 已完成的是真实公开合同校验、物理输入等值映射、来源固定、
与 prepare/simulate/package/audit/observe/register 的连接和记录兼容；缺少任意新对象/区域下的
attachment 选点、夹头/负载连接布局推导规则，不能计入七类对象驱动构造能力。
这些规则由**入口线负责**，不能退回材料生成器。入口线须从新对象网格、原区域及现象条件
推导 B 的 attachment bounds、夹头初态、受阻夹具位置与拖动行程，以及 C 的绳端连接、
负载初态与连接坐标系，并检查支承、净空和连接兼容性。材料线提供合法 attachment/连接能力、
支持的约束与控制参数范围及输出语义；若这些能力缺失，应列出所需字段与合同缺口。
本轮没有实现上述新布局规则，不把固定公开配置映射称为构造。
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
  tests.test_experiment_api.ContractTests tests.test_experiment_integration_boundaries \
  tests.test_construction_revision -v
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

1. **构造代码已实现**：原目标中的六类有限规则及额外的碰撞传播；重排缺构造，B/C 仅转交。
2. **配置与几何检查通过**：下述少量真实身份变化案例、后端合同与 26 项无仿真测试通过。
   当前证据为 `output/construction_review_reuse_final/results.json`。先前 28 配置报告属于同一区域的
   参数敏感性检查，不能作为跨区域复用证据，也不再按它的旧标签声称多体重排已完成。
3. **已有相近物理证据**：原 v7、被动布/梁/绳、B/C 缓存只能说明已有固定配置；未重新审核梁观测或补交付数字。
4. **构造初版阶段的新配置物理效果未验证**：当时没有新物理、渲染、收敛、训练准入或新数据目录。
   后续仅两例获得本次缓存证据；C 的无驱动材料线缓存状态已在上文更新，其他新配置不据此升级。

旧检查失败目录原样保留；最终结果不掩盖超出构造范围的拒绝，也不把配置数量当作新增现象数量。

## 本次复用证据

请求位于 `configs/dataset/v0_2/construction/reuse/`。调用方没有填写对象世界坐标、连接布局或后端配置；
同类请求使用同一 profile。默认检查入口核对资产/支承来源哈希、原对象身份、实际输出变换及非干预参数。

| 对比 | 实际变化 | 检查与边界 |
|---|---|---|
| passage_jug → passage_potato | 不同源网格 `ph_jug_01` → `ph_sweet_potato`，均为 0.15 m | 网格哈希不同，区域/profile 不变；重新推导对象包围盒、通道、初态和净空，通过 CPU recipe |
| passage_potato → passage_second_row | 同一资产与尺寸，从原椅子区域换到另一排 | 原区域候选父对象 chair/chair.001/chair.014，另一排 chair.004/chair.005，两集合不交；共同通道宽约 0.358 → 0.441 m。候选身份不是实际接触证据 |
| cloth_rectangle → cloth_mesh | 生成矩形换为已有 2925 顶点、5632 三角形的矩形静止网格 | 源网格旋转约 15°，显式 scale=0.3，推导约 0.192×0.129 m；读取尺寸、保留拓扑、计算世界变换。仅证明矩形平展网格复用，不代表任意衣物 |
| cloth_mesh → cloth_sideboard | 同一网格与尺度，从 Classroom 教师桌换到 BlueWall 原边柜 Sideboard_01 | 原场景及支承网格哈希均不同；支承高度约 0.816 → 0.862 m，悬出比例均 0.3，支承比例均 0.7；材料、控制、时间和数值设置不变 |

已有布网格来源为 `phenomenon_cloth_stable_v1/cloth_edge_273_stable_v1/prepared/cloth.npz`，
SHA256 `1300c4e2210833f9479147afe43a4cf66e93e8619bc83b8df73d59bc514c1857`。
只读使用其静止几何，不把原缓存当作新摆放的物理证据。检查还确认：要求可通过区域变成受阻类别会拒绝；
边柜上改用原尺寸网格时因支承形状不满足而拒绝，不自动缩小对象或删除障碍。
原 8 cm 边界移动仍可通过 `--legacy-sensitivity` 单独复现，名称改为 `same_region_boundary_shift`。

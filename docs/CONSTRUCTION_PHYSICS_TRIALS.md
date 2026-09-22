# 两个构造请求的实际执行

本轮在 `Y:/isaacsim_work_construction` 完成两次新物理：另排椅子限制区域中的同一刚体资产，
以及 BlueWall 原边柜上的同一布网格。没有重跑基线、追加物理矩阵、运行 C、修改主线、登记或合并目录。
机器可读路径及哈希见 [CONSTRUCTION_PHYSICS_TRIALS.json](CONSTRUCTION_PHYSICS_TRIALS.json)。

## 分项结果

| 项目 | 刚体：passage_second_row | 布料：cloth_sideboard |
|---|---|---|
| 调用输入 | 已有 ph_sweet_potato、0.15 m、原区域、1 m/s、passable；既有 profile | 同一 2925 顶点静止网格、显式 scale=0.3、BlueWall 原边柜、悬出比例 0.3；既有被动 20 kPa profile |
| 配置检查 | 构造及实际 prepare 通过；保持原场景碰撞表示 | 构造、native 合同与实际 prepare 通过；保留原网格拓扑及全部供给碰撞组 |
| 实际执行 | PhysX，240 Hz、2 s，481 完整状态；一次新物理 | PhysX，1200 Hz、5 s，301 完整保存状态；一次新物理，保存逐节点位置/速度及拓扑 |
| 目标现象 | 实际通过所选喉部，未受阻；其后继续滚动并从区域 +X 端离开 | 有支承并形成下垂，未整体滑落；末段仍有局部残余运动 |
| 统一读取 | 状态、原生接触及 21 帧 RGB/depth/ID 诊断观测通过 | 状态、完整布网格与 51 帧 RGB/depth/ID 诊断观测通过 |
| 用途资格 | 固定配置状态预测候选，待主线用途审核；未自动授予训练准入 | 固定配置柔性状态预测候选，需保留局部抖动限制、单列用途审核；未自动授予训练准入 |

两者均无主动控制，统一控制流为空，不补造 command/effort。布料接触力、attachment 反力保持 unavailable。
未做时间/空间收敛、实物材料标定或 bit-exact 重复性研究；数据工程通过不等于这些结论通过。

刚体实测：原 chair.004/chair.005 限制区域的喉部 X=1.802734 m。
按每帧实际姿态变换原对象网格，前缘在 0.045833 s 到达喉部，后缘在 0.179167 s 越过；
对象横向包围盒全程在选定 Y 范围内。参考位置 X 从 1.714474 到 2.207053 m，
超过选区 X 上界 2.1 m 的后续滚动也完整保留，不裁成“成功片段”。记录到 2844 条 subject/room
原生接触，没有 subject/椅架接触；最小原生 separation 为 −0.231 mm。日志中的 PhysX material
face-index 警告原样保留；不由通过结果推断摩擦或冲击力已校准。喉部通过不证明所有其他障碍可通过。

布料实测：边柜支承面 Z=0.861611 m，初态布高出约 6 mm；0.033333 s 起超过半数节点
进入支承面 ±4 mm 的几何邻近带。末态约 69.23% 节点投影在原支承足迹内，布最低点 Z=0.804462 m，
低于台面约 57.15 mm。节点比例不是支承面积比例，也不是原生接触判定；没有从几何接近制造接触力。
支承投影内节点全程未低于 0.862183 m。全程最大网格边长/初始边长比约 1.0807，
峰值节点速度 1.515 m/s；末半秒峰值 0.0831 m/s，末态 RMS 0.00599 m/s。
末半秒相邻保存帧最大位移约 0.112 mm，能说明形状变化较小，不能据此抹掉速度抖动或称完全静止。

## 发现并修正的构造问题

原相机使用整个 ROI 的中心和最大跨度，刚体相机落在原场景遮挡物外侧。
物理成功，首次 observe 明确失败 `No observed subject`；原工作流、错误账本和不可见画面完整保留在
`output/construction_physics_v1/passage/execution/`。

修复的是构造规则：从已构造的刚体入口、选区出口与对象范围推导局部取景，检查原三角网格到初始中心
和路径目标点的视线；候选视线全部受阻时返回 `observation_visibility`。它检查初态视线，
不承诺整个运动过程都可见。新增测试覆盖原墙遮挡时拒用被遮挡视点。
调用者不需要补相机坐标，生成后也没有手工改世界坐标或后端物理配置。

用同一语义请求重新构造到 `passage_camera_v2/generated/`，通过新的
`--reuse-physics-cache` 显式声明本次旧缓存。执行 prepare 比对完整有效物理、原网格数组与
求解器依赖，确认等值后直接 package/audit/observe；**没有第二次刚体 simulate**。
修复后的 21 帧诊断均可读取，累计 subject 可见像素 23783。旧封存来源通过准备时复制的源文件核验，
不修改旧源码快照或旧报告去冒充最新代码。旧执行目录不应拿变化后的工作区代码直接 resume。

## 文件与复查

请求沿用已提交的 `configs/dataset/v0_2/construction/reuse/passage_second_row.json` 和
`cloth_sideboard.json`，对象、原区域、条件及 profile 未调参。完整交付根目录为
`Y:/isaacsim_work_construction/output/construction_physics_v1/`。

- 原生成配置与实际 native：`passage/`、`cloth/` 下的 generated 和 execution。
- 相机修复后的生成配置、物理缓存等值绑定及刚体最终统一 episode：`passage_camera_v2/`。
- 两例逐帧实测及来源核验：`reviews_v2/`；旧分析输出也保留。
- 原材质抽查：`appearance/passage/`、`appearance/cloth/`，各 3 张 480×320、4 samples 图片。
  直接读取新物理缓存、原 blend 场景与灯光；刚体读取真实 glTF 材质并核对外观/碰撞对齐。
  布料沿用公开回放器的织物外观与原生 UV，未改变物理网格。没有视频、补运动、隐藏原物件或灯光打磨。
- 27 项无仿真回归通过：`tests.log`。运行资源记录保存在各 execution；未终止其他作业。

`scripts/review_construction_physics.py` 可从请求、构造报告和已有工作流重读结果；`--output` 必须选新目录。
物理来源码若已更新，会核对准备阶段按哈希保存的历史源文件，不把最新文件冒充运行版本。
最终 episode 地址和全部交付文件哈希由机器可读摘要固定，主线可直接用 `open_episode` 读取。

## C 状态只读更新

`d8a4c4b` 的无驱动缓存已存在，独立核对交付列出的 native、封装、配置、执行及成对审核哈希，
并通过统一读取验证 181 状态、2880 条 command/actual/effort；全部主动力与力矩为零，target_active=false。
相对 `05aabd3` 的接口实现差异是 package 将禁用说明一次性写入，修复旧重复独占写入错误；
没有 solver/input 控制语义变更。共同稳定窗口仍未通过，不声称无驱动静止。
核对脚本为 `scripts/check_c_disabled_delivery.py`，结果为 `c_delivery_verified.json`。
旧 `ca39d38`/`05aabd3` 固定入口未升级，未重跑或重封装 C，未修改材料工作区。

多体重排构造、B/C 新布局推导仍是后续缺口；本次未追加这些内容，也未纳入流体。

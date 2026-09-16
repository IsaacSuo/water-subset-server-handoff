# 现成几何代表实验与 C4 刚体小子集

2026-09-17。本轮范围是扩大原位几何使用并交付一个内部开发子集，不是冻结完整数据集。
最新选择依据为 `Y:/scene_research_20260916/assessment_20260917/REPORT.md`；已同时考虑
本地warehouse、classroom及新增Blue Wall、Pawn Shop等。报告下调Blue Wall的通用价值，
但已有具体空带的缓存仍有效；本轮不因排名变化重跑它。

## 能力已实现

- `static_scene_source`：保留原世界坐标、实际三角形和相关周边碰撞，不用隐藏实验台。
- `native_scene_batch`：同一场景配方接收资产列表与条件配置，当前新批接受适用刚性球/块。
  既有真实资产入口仍保留；不声称已自动完成任意STL在任意原场景的姿态适配。
- C1有限执行器直接复用，滑动关节只限制运动轴，不替代接触反馈。
- `causal_subset`：从已有物理缓存生成/复用观测、按来源组分配split，并通过统一入口读取。
- 原生接触注释现在逐实际对象对保存；四体传播不会把一次两体接触误标成四体同时接触。

## 代表实验已完成

### Blue Wall 首例收尾

沿用原两条2 s缓存，1.2 m/s已有材质预览直接复用，只补0.8 m/s条件并拼接对照。
没有重做已完成的高速度渲染。运动、支承、前后遮挡已检查，仍保留约5.17 mm、
两个物理步超过2 mm的落地穿入。未因此调整时间步、碰撞参数或扫描材料。

[原材质并排预览](../output/world_model_dataset/v0_2/blue_wall_native01/comparison.mp4)。
静止后完全相同的缓存状态复用上一张图，但每个输出时刻仍有明确物理时间。

### 持续推动与原有几何受阻

目标是观察有限执行器在几何阻挡下的实际响应，不要求穿过不存在的狭缝。
选本地 `Y:/scenes/warehouse.blend`：原地面承载，正X侧已有墙脚金属结构阻挡。
保留可见原位7个网格（12,162个三角形），隐藏原对象按原设置保留不参与；
没有修改场景、另搭墙、加简化地面或替换金属基部。

- 必要补测：x=6.5–7.8 m、y约8 m的地面与横向路径；原地面z约−0.499997 m。
  墙脚金属表面x约7.94144 m，Wall1主体x约8.00644 m。实际先碰的是前者，保留二者几何。
- 实验体：20 cm方块、0.5 kg，初始中心x=7.1 m；推板8×24×24 cm、1 kg，x=6.7 m。
- 控制：0.2–2.6 s有限速度反馈力，增益20 N·s/m、力上限8 N；其后外力为0。
  初始双方静止；唯一条件变化是目标速度0.8/0.35 m/s。
- 观察：完整3 s、240 Hz状态/接触/实际施力；两条相机位置相同。

实际结果：高速在1.59583 s开始接触金属墙脚，方块COM停在x≈7.84144 m；
推板约移动1.0007 m，未到2 m关节上限。2.3–2.6 s施力100%处于8 N限幅，实际速度接近0。
慢速方块末x≈7.645 m，记录内没有碰到墙脚；不是失败样本，也不延长到它一定撞墙。
两条都有原生推板—物体接触，墙脚受阻一条保留物体—金属结构接触。

### 原地面四体碰撞传播

目标是观察局部初始运动如何依次传到多个物体。复用warehouse已测6×6 m区域，
不为增加场景名称重新接入同类大地面；Pawn Shop保留为确有需要时的替代。

- 四个直径20 cm、1 kg球，y=8 m，首球x=0；原地面直接承载。
- 初态首球vx=1.2 m/s，其余静止，正式时间轴控制为none。
- 物体摩擦0.05、恢复系数0.8；地面沿用指定材质，不从原纹理推断物性。
- 唯一布局参数变化是相邻表面净距6/26 cm，其余物体位置由它推导。
- 全程3 s；接触顺序、全部对象状态与继续运动均保留。

实际三次相邻接触首报时刻：近间距0.05417 / 0.11250 / 0.17917 s；
远间距0.22917 / 0.51667 / 0.90833 s。3 s末两组均仍运动；末球x约2.436/2.124 m，
仍在原测量窗口内，不称作最终静止。场景位姿固定，没有中途速度写入或删除参与体。

配方：`configs/dataset/v0_2/native_scene_batch02.json`。
原缓存及直接检查：`output/world_model_dataset/v0_2/native_scene_batch02/`。
每组遵循首条实验、直接审缓存、再补一个解释明确的条件，不跑参数矩阵。

[三组预览入口](../output/world_model_dataset/v0_2/native_scene_batch02/showcase/index.html)
（Blue Wall原材质、仓库两组中性材质）。仓库视频直接编码已有传感器帧，不再渲染。

## C4 首批观测完整数据：范围与读取

选择清单是可执行配置 `configs/dataset/v0_2/c4_rigid_micro01.json`，不是人工覆盖大表。
范围为10条刚体episode：简单碰撞/推动各一条、香蕉/椅子推动各一条、Blue Wall两条、
warehouse四条。数量来自现有互补缓存，未设置必须达到的总条数。

统一传感器规范：640×480、35 mm焦距、36 mm水平片幅、30 Hz、每条两个固定视角。
位姿按具体空间配置，在条件对照间保持一致；不是强行把不同房间套在同一世界相机坐标。
每帧保存RGB、以米计的相机光轴Z深度/有效掩码、物理参与体实例分割、物理步和时间。
每条同时保存原生逐物理步状态、接触点/法线/冲量、命令、实际执行器状态和受限外力。
无动作episode返回空控制迭代器，其控制primitive明确为none，不假造零力命令。

首批RTX传感器观测使用统一声明的中性材质，不迁移原blend贴图；物理原几何全部保留。
室内原墙/屋顶会遮住环境光，因此采用记录在 `observation_profile.json` 中的两处相机位置
补光（归一化SphereLight强度30000、半径0.15 m）。这是观测照明变化，不改变物理。
Blue Wall原材质Eevee展示与这批正式RGB-D分开，不把两种画面冒充同一个传感器记录。

过暗的首轮室内RGB保存在 `c4_rigid_micro01_unlit_diagnostic/`，**不属于交付子集**。
结构检查可通过不代表RGB可读；该问题在代表图直接检查时发现，补光单帧验证后重新生成
室内观测。原有两条C2观测和首轮已合格的香蕉/椅子观测复用，不重跑物理或重复制作。

来源分组在生成前写入 `sources_and_splits.json`：简单C2与资产推动为train，Blue Wall为
validation，warehouse为test。同一物理缓存、原场景、模板条件与原始资产谱系不跨组。
记录源manifest、状态、接触和解析输入哈希。不是随机按帧切分；不宣称这是正式几何OOD，
也不把十条数据当作足以评估模型的统计规模。后续新变体仍须继承这些来源组。

子集根目录：`output/world_model_dataset/v0_2/c4_rigid_micro01/`。
最终以根目录 `index.json` 与 `delivery_review.json` 为准；progress文件不是交付声明。

实际输出：10条episode，train/validation/test为4/2/4条；1,460张RGB与对应1,460份
深度/分割NPZ，5,770个物理状态时刻，36,201条原生接触点记录，5条控制命令、
2,880条执行器施力记录，目录约911 MiB。统一 `inspect` 已完整读过全部10条。
中断恢复时从空目录重新执行同一个build入口，复用9条完整观测并补最后一条，
复用帧索引完全一致；10条状态、接触、解析输入哈希均与原物理缓存一致。
这不是再跑一次确定性仿真，也未创建第二份无意义的全量重渲染。

首/中/末帧、双视角共1,512条几何射线检查：标签不匹配0，缺失交点0。
方块/环境组最大深度差约0.0043 mm，外部网格约0.278 mm。球体组用高分辨率球面
与RTX球原语三角化比较，中位差约2.4–2.5 mm，近轮廓射线最大12.84 mm；
保留此表示差异，不宣称渲染深度等于解析球面或原生烘焙碰撞面。详见
`geometry_observation_review.json`。27项相关测试通过。

```bash
# 从空输出目录复现：依赖清单里固定的原物理缓存，不重新仿真。
python3 -m world_model_dataset.causal_subset build \
  --spec configs/dataset/v0_2/c4_rigid_micro01.json \
  --output output/world_model_dataset/v0_2/c4_rigid_micro01_rebuild

# 可额外传 --observation-cache output/world_model_dataset/v0_2/c4_rigid_micro01
# 仅在物理状态和相机/照明profile匹配时复用观测。
python3 -m world_model_dataset.causal_subset inspect \
  --root output/world_model_dataset/v0_2/c4_rigid_micro01
```

```python
from world_model_dataset.causal_subset import CausalSubset
dataset = CausalSubset('output/world_model_dataset/v0_2/c4_rigid_micro01')
for item, episode in dataset.episodes(split='test'):
    states = episode.states()
    contacts = episode.contacts()
    controls = episode.controls()
    for metadata, frame in episode.observations():
        rgb, depth, mask = frame['rgb'], frame['depth_m'], frame['segmentation']
```

“复现”限于已有缓存复用与观测生产；并不声称物理重跑bit-exact。
软体、法线、运动矢量和正式模型基准不在这批交付内，整个C4没有因此关闭。
warehouse的原资产CC-BY来源已识别，但本地新增组件谱系待闭合；此子集只作内部开发，
不附带源blend再分发或不限商业公开许可承诺。本轮独立提交代码/配置/文档，不推送远端。

视觉限制：香蕉和椅子沿用既有推动相机，靠近推板时存在明显真实遮挡，不能解释为完整
轮廓观测。保留推板可见性与完整状态，不隐藏执行器；后续更清楚的形状展示可另设视角，
并继承同一来源组。当前子集只承诺声明模态与时间轴完整，不承诺每帧全表面可见。

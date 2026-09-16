# 首个现成场景原生几何接入：Blue Wall

2026-09-17。依据 `Y:/scene_research_20260916/REPORT.md` 第5节的工程接入顺序，
先选择 N02 Blue Wall，而不是按场景名称强行安排窄口实验。报告未找到已测、适合
0.2 m对象的贯通狭缝，本轮选择已有能力可直接实现的“支撑边缘 → 倾转 → 跌落”。

## 目标与条件

- 来源：Greg Zaal / Poly Haven，Blue Wall，包内 `info.txt` 与研究报告记录为 CC0。
  本轮复用已有证据，未重新作网络许可核查。
- 源文件：`/mnt/y/scene_research_20260916/downloads/blue_wall/blue_wall.blend`。
  SHA256：`7dfd3e3609140c1721bafd101f80e68051f7f23590df017ff3bf4e8b007b0477`。
- 必要几何：`Sideboard_01` 原有柜面与前缘，下方 `Room` 原有地面。
  柜面实测 z=0.8616107 m。选择植物/钟与杂志之间的空带，未移动任何原摆件。
- 系统：8 cm 刚性实验方块，密度700 kg/m³，对应质量0.3584 kg。
  这是新声明的实验物体，不是把此前20 cm资产静默缩小。
- 初态：COM=(-0.44,-0.17,0.9018107) m；初始方向不旋转；底面距实测支承面0.2 mm。
- 控制：none，仅在t0赋予朝柜前的初速度；重力由求解器积分，支撑不抽走、不改碰撞。
- 变化轴：0.8/1.2 m/s 初速度，其余输入相同。第一条自然停住后保留为未跌落条件，
  没有为得到跌落而修改柜子、接触参数或缓存。
- 观察：每条2 s、240 Hz，包含481个状态；短回放为固定近景，覆盖柜面与落地范围。

材料使用统一指定的静/动摩擦0.3/0.2、恢复系数0.1；不声称量测了原家具木材物性。
这轮所有环境对象固定，不模拟柜门、抽屉、植物或原摆件的动态响应。

## 最小接入改动

`causal_runner.prepare` 新增可选 `static_scene_source` 入口；其他物体入口不变。
仍使用v0.2合同、现有原生刚体运行器与统一loader，不新增schema。

1. Blender 5.0.1 禁自动脚本只读打开原场景，保留所有194个评估网格实例。
2. 应用原实例世界变换及原米制单位，按原渲染修改器设置评估并三角化。
   钟的细分修改器原来仅在渲染启用；导出时在内存里对齐该设置，记录在 `scene.json`。
3. 导出649,000个三角形，按柜体、房间、其余物件分成3个静态碰撞参与体；
   分组只为记录与加载，不删除或重塑几何。每个源对象保留名称、矩阵、顶点/面索引范围。
4. 静态三角碰撞使用 `approximation=none`，不做COM居中、凸包替换、填洞、加厚或减面。
5. 解析后的NPZ复制进episode；原生仿真只读episode内部几何，不依赖Blender运行。

源blend的SHA256前后相同，原始文件未保存或改写。已核对两条episode的三组网格
顶点和三角索引都与导出完全相同，整个时间轴环境位姿固定。

## 实际结果

| 初速度 | 实际结果 | 原生接触证据 |
| --- | --- | --- |
| 0.8 m/s | 向前移动约8.78 cm，轻微倾转后停在柜沿前；2 s末速度为0 | 631条柜体接触点记录；没有地面接触 |
| 1.2 m/s | 在柜沿倾转并跌落，最后停在地面；2 s末COM约(-0.43764,-0.46953,0.04) m | 柜体855条、地面324条；最后柜体报告0.400 s，首次地面报告0.725 s |

接触报告起止是有接触点报告的采样时刻，不等同于连续时间上的精确接触区间。
两条均只有solver step，没有中途写位姿/速度。更快一条最大速度约4.112 m/s。

已知数值限制：落地时源方块顶点对实测地面最大采样穿入约5.17 mm（0.720833 s），
超过2 mm共2个物理步，随后恢复；没有持续掉穿地面。保留事实，不把它描述为
高精度撞击样例，也不为本次开发接入反复调参。原柜面接触报告穿入很小。
地面测量前使用原Room三角形对全轨迹角点XY作向下射线确认，未用无限平面误计离场。

23项相关测试通过，包括静态开放三角面不被居中/翻面、拒绝动态误用、哈希核对，
以及原资产入口和因果episode回归。直接审核结果见 `cache_review.json`，不设置新增运行前门禁。

## 输出与复现

根目录：`output/world_model_dataset/v0_2/blue_wall_native01/`。

- `scene_geometry/scene.json`：原场景与每个实例的几何谱系。
- `representative/episode/`：首条0.8 m/s未跌落条件，目录名保留历史，不冒充跌落结果。
- `edge_speed12/episode/`：1.2 m/s跌落代表；含状态、原生接触点/法线/冲量、USD快照。
- `cache_review.json`：直接缓存检查与已知限制。
- `showcase_frames/`：原场景材质Eevee回放；曝光调整仅用于预览，不影响物理。
- `showcase.mp4`：完整2 s缓存半速播放，物理时间另有标注；不是正式RGB-D观测。
- `comparison.mp4`：0.8/1.2 m/s并排对照；复用已有高速预览，只补低速条件。
- `preview_check/`：首条试构图与曝光的中间图，不作为推荐展示。

命令（输出目录必须为新的目录）：

```bash
blender -b --factory-startup --disable-autoexec -t 8 \
  --python world_model_dataset/blend_scene_export.py -- \
  --blend /mnt/y/scene_research_20260916/downloads/blue_wall/blue_wall.blend \
  --output /mnt/y/isaacsim_work/output/world_model_dataset/v0_2/blue_wall_rebuild/scene_geometry \
  --support Sideboard_01 --floor Room

python3 -m world_model_dataset.native_scene_episode \
  --scene output/world_model_dataset/v0_2/blue_wall_rebuild/scene_geometry/scene.json \
  --output output/world_model_dataset/v0_2/blue_wall_rebuild/edge_speed12 --speed 1.2

blender -b --factory-startup --disable-autoexec -t 8 \
  --python world_model_dataset/blend_scene_replay.py -- \
  --blend /mnt/y/scene_research_20260916/downloads/blue_wall/blue_wall.blend \
  --episode /mnt/y/isaacsim_work/output/world_model_dataset/v0_2/blue_wall_rebuild/edge_speed12/episode \
  --output /mnt/y/isaacsim_work/output/world_model_dataset/v0_2/blue_wall_rebuild/showcase_frames
```

接入结论：已证明这个现成场景的实际柜体与地面能进入现有协议并产生代表物理轨迹，
不是只换渲染背景。不把这一条扩大为任意blend自动适配、所有材料已验证或正式数据集准入。
后续可以在这份固定几何上匹配合适真实资产或扩展初态，不随意改造原场景。

# 独立水体子集：Newton＋DFSPH 迁移

范围仅为独立水体子集。它不并入主线现象控制，也不承担布料、绳索、软体等其他系统的替换。已有外观资产及动作设计继续使用；更换的是水体求解和与它交互的刚体计算。

当前工作范围以 [CURRENT_WATER_SUBSET.md](CURRENT_WATER_SUBSET.md) 为准：先完成已有五个液面场景和三个主动驱动场景。水槽注水、山地岩壁落水后续纳入。五个液面场景已有布局与历史样片，并非待新建的扩展方向。八个场景的原分辨率输入已转换，完整动作诊断和最终验收分别记录，不以转换完成代替效果通过。

2026-09-24 补充：SPH_Project 已完成小水槽同条件比较，水波曲线与 CPU SPlisHSPlasH 接近，本机纯步进约快 6.4 倍，详见 [SPH_UNIFIED_COMPARISON.md](SPH_UNIFIED_COMPARISON.md)。GPU DFSPH＋Newton 适配器已运行到输出缓存，八个原场景均完成原分辨率的 1/30 秒流程检查。自由刚体的错误接触已通过用户批准的预生成 SDF 路径消除，0.2 秒接口中已核对入水扰动与水反力驱动刚体的闭环；完整动作及长时稳定性尚未验收。当前状态以 [GPU_DFSPH_NEWTON_WORKFLOW.md](GPU_DFSPH_NEWTON_WORKFLOW.md) 为准，下述 CPU 桥接仅为历史参考。

## 当前路线选择（2026-09-24）

用户明确的迁移目标是 SPH_Project GPU DFSPH＋Newton 通用水—刚体双向耦合。CPU SPlisHSPlasH 仅保留为参考和对照，不能作为迁移目标。此前这里写成“用户决定沿用 CPU、搁置 GPU”不符合当前明确指令，现予更正。当前 GPU 实现和状态见 [GPU_DFSPH_NEWTON_WORKFLOW.md](GPU_DFSPH_NEWTON_WORKFLOW.md)。

CPU 路线已完成简单浮体 3 秒双向耦合、短段时间步减半比较、边界坐标接口和原搅拌粗粒子 9 秒预览。基础运行未发现阻断问题，但静水仍有粒子残余运动；原搅拌 4 mm、694,674 粒子仅完成约 0.033 秒短测，完整片段与倒水、活塞尚未动态验收。继续沿用现有 `production_accepted=false` 状态，不能将选定 CPU 解释成上述未验收项目已经通过。

## 历史 CPU 桥接实现（参考保留）

- `coupled_scene/newton_dfsph/native`：原生桥接库，链接本机已有 SPlisHSPlasH 2.18.1；液体使用 DFSPH。
- `coupled_scene/newton_dfsph/bridge.py`：输入检查、稳定粒子编号、容量检查和明确的单进程生命周期。
- `runtime.py`：Newton 刚体质心速度与网格原点速度、质心力矩与原点力矩之间的转换。
- `coupling.py`：多刚体索引绑定、边界姿态核对、保留已有外力并累加液体作用力；显式区分自由刚体和给定动作的物体。
- `assets.py`：读取原 `assets.json` 和 `geometry_and_fill.npz`；保留三角形、局部顶点、原点、填充和资产哈希。
- `experiments/coupled_scenes/run_newton_water.py`：独立水体入口，复用 `active_drive.motion_state`，输出原有 `capture/manifest.json` 和 NPZ 合同。
- `run_active_pour_video.py --unaccepted-backend-preview`：原外观重建/渲染流程识别新后端；输出仍明确标注未验收。

本地使用随 Isaac 提供的 Newton 1.2.1、Warp 1.13.0，不升级全局环境。Newton 只解刚体；没有改用 MPM。DFSPH 所附带的 PBD 刚体模块只保存边界状态，不执行刚体步进。

当前原生构建为本机 Windows/MSVC 版本。服务器迁移还需要对应平台重新编译桥接库并配置 Newton/Warp 环境；不能直接复制本机 DLL 作为跨平台支持。

每个小步先把 Newton 的边界位姿和速度交给 DFSPH，再取得液体反作用力和力矩，最后推进 Newton 刚体。当前为显式顺序耦合，不包含迭代回滚。因此自由浮体、很轻物体、强挤压和大时间步仍需分别验收。

电机驱动的桨叶、倒水壶、活塞保留原运动学驱动。它们的反作用力被记录，不改变给定动作；自由刚体则需要将反作用力传回 Newton。两类对象不能混为同一种“双向效果”。

## 已接入的资产

| 资产 | 原粒子数/间距 | 当前接入状态 |
| --- | --- | --- |
| 搅拌 `active_02_stirring_assets_v2` | 694,674 / 4 mm | 原网格和填充转换；驱动最高速度固定 3.6 rad/s |
| 活塞 `active_03_piston_push_assets_v2` | 696,234 / 4 mm | 当前清单使用此原网格和填充；先前 3 mm 转换保留为历史记录 |
| 倒水 `active_pour_water_minus10_assets_v10`，来源为降低壶位后的 v7 外观 | 100,434 / 4 mm | 当前清单使用最新资产；先前 73,557 粒子转换保留为历史记录 |
| 五个液面场景 `surface_study_group01_v1` | 通常 2,450,448 / 4 mm；推板按两倍间距留空后 2,397,780 | 共用入口已接通原槽尺寸、装置与动作；动态和外观尚需验收 |

转换不等于验收，尤其不能把粗粒子预览当作原分辨率结果。运行报告和缓存均保持 `production_accepted=false`。

## 本机运行

在仓库根目录构建。源码和已编译参考库可以通过 CMake 的 `SRC`、`BLD` 路径覆盖；必须保持编译选项/ABI 一致。

```powershell
& 'C:/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe' -S coupled_scene/newton_dfsph/native -B output/coupled_scenes/newton_water_migration_20260923/build -G 'Visual Studio 17 2022' -A x64
& 'C:/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe' --build output/coupled_scenes/newton_water_migration_20260923/build --config Release
```

原分辨率搅拌入口（输出目录必须是新的；初始化后先通过静水窗口，才开始动作）：

```powershell
Y:/isaacsim/python.bat -u experiments/coupled_scenes/run_newton_water.py --assets output/coupled_scenes/active_02_stirring_assets_v2 --output output/coupled_scenes/newton_stirring_full --dll output/coupled_scenes/newton_water_migration_20260923/build/Release/reference_bridge.dll --seconds 10 --max-wall-seconds 43200
```

加入 `--prepare-only` 仅转换完整资产，不启动模拟。加入 `--preview --particle-stride 3 --hz 600` 可做 12 mm 调试预览；它明确跳过静水准入，只用于发现接入错误。默认 4 mm 正式入口不抽样、不删粒子、不注入速度。默认静水窗口沿用项目现有速度/水位限制，在 10 秒内未达到就失败；不会偷偷放宽。

首次细网格边界预计算较慢。后续可加 `--reuse-boundary-cache <先前成功运行目录>`：会核对桥接库、原网格内容、刚体初始变换和空间分辨率，复制已有体积图，并记录文件哈希。只改时间步可以复用；换几何或粒子/边界分辨率会拒绝复用。每次运行仍保留独立缓存，避免并发覆盖。

上述完整片段示例显式给出 12 小时上限，不代表片段已经运行或保证能在预算内验收。CLI 默认 30 分钟预算主要适合短测；本机 4 mm 短测实测 0.033 秒模拟需约 44 秒纯步进，完整离线片段可能是小时级工作。不得把短测速度直接当作长期恒定耗时预测。

原渲染入口对新后端须显式加 `--unaccepted-backend-preview` 并给出 `--simulation`。它仍核对原 blend 和几何哈希，不要求或伪造 PhysX 的涡量参数。正式采用之前，需要单独完成下述验收。

## 迁移验收顺序

1. 接口：非中心旋转轴、初始姿态、多边界顺序、质心力矩、原始粒子编号、坐标往返。
2. 自由浮体：至少数秒，不只看浮起一瞬；再做时间步减半比较。不能用运动学桨叶测试替代反作用力反馈测试。
3. 原搅拌：起转、3.6 rad/s 持续搅拌、停转后的余流；检查漏水、桨叶穿透、是否异常自激。
4. 原分辨率与原外观：粗预览通过后仍要检查 4 mm、原碗壁和原 Cycles 外观，记录运行成本。
5. 倒水/活塞：自由表面破碎、液体跨容器转移、窄缝和压力挤压另行验收。不能从搅拌通过推断它们通过。

已有参考结果用于离线对比和验收，不作为运行时目标曲线，也不驱动动作或参数分段。参考求解器接入以后同样要检查可见异常，不能因为它原先叫“参考器”就免验收。

## 当前范围与后续扩展

当前先处理静水、容器晃动、自由波浪传播、液面传播与反弹、受扰后的恢复、两容器倾倒／转移、搅拌、活塞推水八种已有场景。水槽注水和山地岩壁落水已确认后续需要纳入，本轮不扩展。其他运动类别仍是规划候选，不能当成已存在或已验收场景。每类保留有限、可验收的资产与动作组合，外观包仍各自独立。

## 尚未覆盖

目前适配器只支持单液体模型、固定步长 DFSPH 和 Bender 体积图边界。不支持发射/删除、进程内重新初始化、关节马达反馈控制、重启检查点和其他水体材料。复杂网格的体积图是离散近似，保留原三角形不意味着接触面精度自动相同。边界外包围盒/圆形边界计数仅为早期告警，不等价于完整的穿壁验收。

CPU 上的 DFSPH 加 CPU/GPU 状态交换成本仍明显高于原 GPU PhysX。是否适合作为离线有限子集后端，应以原场景完整片段的实际耗时和外观检查决定。保留 PhysX 原入口用于对照；当前没有宣称整个子集已切换验收完成。

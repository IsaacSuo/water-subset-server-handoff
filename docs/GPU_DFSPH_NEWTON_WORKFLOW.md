# GPU DFSPH＋Newton：独立水体流程

路线固定为 SPH_Project（`2a97e63dab680a2ee40b6a96b0f135062c6b1aaf`）CUDA DFSPH＋Newton 1.2.1。CPU SPlisHSPlasH 只作参考。刚体使用 Warp 1.13.0 CUDA；没有启用 Bullet 刚体步进，也不使用 Newton MPM。

## 实现

- `coupled_scene/gpu_dfsph/backend.py`：将上游独立的刚体积分接口替换成 Newton。支持固定边界、自由刚体、既定动作装置和多个物体绑定。原点／质心偏移通过 Newton 实际质心转换；水的力矩在同一世界质心坐标中传递。自由刚体接收反作用力，既定动作装置记录反力并维持指定动作。
- Newton 刚体网格现预生成 GPU 纹理 SDF，并缓存到 `output/coupled_scenes/gpu_newton_sdf_cache`。目标体素为液体间距的一半（本次 2 mm），高分辨率窄带为 ±20 mm，保留原网格、质量、质心与惯量。运行报告记录实际体素尺寸及每个刚体的 SDF 绑定，缺失绑定按基础接口错误处理。水—刚体仍通过 SPH_Project 固体边界粒子交换运动与反力，未替换成液体 SDF 边界。
- 上游 DFSPH 液体计算内核保留。密度最低两次迭代、误差单位换算等沿用已做过的统一比较设置，没有根据场景结果调参。上游在步末散度修正产生的反力仍于下一刚体小步消费，此步序没有暗中改动。
- `assets.py`：导入原网格、填充和编号，采样固体边界，记录求解局部坐标与原外观世界坐标的转换。纯数据导入复用旧辅助函数，不调用 CPU 液体求解器。
- `run_gpu_newton_water.py`：输出现有 `capture/manifest.json` 和 NPZ，记录粒子状态、刚体状态以及水反力和实际施加的反力。只检查 CUDA、有限值和编号完整性，没有静水／波幅／物理通过门槛。
- `run_gpu_water_workflow.py`：分别准备、模拟、渲染八个现有场景。失败保留日志并继续独立场景工作；不自动重试或修改参数。默认模拟长度仅一帧间隔，用于流程接通。
- 现有主动装置与液面渲染入口已增加 `gpu_dfsph_newton` 缓存识别；沿用原外观文件、相机和实际记录的装置姿态。

运行环境：`/home/fangsuo/venvs/sph-project-validation`。GPU 命令使用 `LD_LIBRARY_PATH=/usr/lib/wsl/lib`。未更改全局 Python 或上游 SPH_Project 文件。

## 当前记录

输出根目录：`output/coupled_scenes/gpu_newton_workflow_20260924`。原场景准备状态在 `prepare_status.json`。短接口运行在 `free_body_interface/simulation`。

第一次接口运行在初始化粒子重排处失败，尚未开始液体步进，也没有生成可用 GPU＋Newton 运动缓存。原因是新增适配器用 `super().reorder_particles()` 直接调用父类 Taichi 类内核，绕过 Taichi 的实例绑定，触发 `kernel_impl.py:1105` 的断言。这是本次适配层的调用错误，不能归因于液体物理结果。

用户批准后已修正：保留父类 `reorder_particles` 内核名称和实例调用路径，将扩展点移到 Python 层 `prepare_neighborhood_search`；原重排结束后，利用已有 `grid_ids_new` 排列同步稳定编号。原失败目录保留；同一个 0.2 秒短接口流程在 `free_body_interface/simulation_v2` 完成，输出 7 帧，耗时 49.78 秒。此处的完成仅表示执行与缓存完整，不表示双向耦合已验收。

此前手动查看 `simulation_v2` 缓存发现自由方块异常飞出。保留 `interface_recorded_states.png` 和原始接触记录 `initial_contacts.json`：方块与槽壁实际最近距离约 44 mm，顶点均不在槽壁材料内；未预生成 SDF 的网格距离查询路径却产生 3 个约 60 mm 的负间距接触。该次采样帧的水反力为零，不能证明水推动了自由刚体。用户随后批准使用预生成 SDF，修正结果见下节。

八场景的 `prepare_status.json`、`simulate_status.json`、`render_status.json` 均记录 8/8 完成。全部使用原 4 mm 输入，分别输出 t=0 和 t=1/30 秒的两帧缓存、两份重建水面和两张原外观渲染图；原视频编码规则输出 1/30 秒的一帧视频。人工逐项查看的是模拟后的 `frame_0001.png`，汇总为 `eight_scene_workflow_review.png`。五个液面场景中水面、槽体和相应装置位置衔接；倾倒初态可见壶内水面和接水容器；活塞与搅拌可见槽内水面和原装置。未见整场坐标错位或资产缺失，但初态短帧无法证明完整动作、波浪传播或恢复效果。多数既定动作从 2 秒开始，这次短流程尚未执行到动作段。

基础缓存检查已随运行完成（有限值、完整粒子编号、文件与源资产身份）；没有新增物理阈值或逐场景验收门槛。修改后的 Python 语法检查及 `git diff --check` 完成；SPH_Project 上游工作区保持干净。所有本轮运行均已结束，没有后台模拟继续执行。

粒子回收尚未接入；不将漏出率作为本阶段自动裁判，也不据此改动作。原 CPU 试验输出保留为历史数据，不计入 GPU 流程完成。

## 预生成 SDF 修正结果（用户批准后执行）

未实施此前的凸体分类方案。对原槽体和方块调用 `Mesh.build_sdf`，使用原网格生成有符号距离纹理；没有凸包化、改动作或调液体参数。需更正“网格与 SDF 二选一”的粗略描述：之前也使用有符号距离接触算法，但距离来自即时 BVH 网格查询；现在使用预生成 SDF 纹理。

先检查原位姿，不推进水体或刚体。`sdf_initial_sections.png` 显示槽底、槽壁为负距离实体，槽腔为正距离空腔；实际体素尺寸槽体约 1.92–1.94 mm、方块约 1.83 mm。`initial_contacts_sdf.json` 记录 38 个接触候选，全部正间距，最小约 44 mm，原先 3 条虚假穿透消失。候选数量不等于实际施加约束数量，XPBD 对正间距不施加穿透修正。

随后原封不动重跑同一个 0.2 秒接口流程，输出 `free_body_interface/simulation_sdf`，7 帧，耗时 12.85 秒，基础有限值和粒子编号检查完成。GPU 液体与 Newton CUDA 保持不变；SDF 缓存两文件约 4.65 MB，第二次构建复用缓存。

人工查看 `sdf_coupling_recorded_states.png` 和 `sdf_coupling_observations.json`：方块先下落，入水后挤开附近液体并引起局部运动；向上的水反力传入 Newton，方块减速、在约 0.167 秒转为回升，0.2 秒时竖直速度约 +0.116 m/s。0.1 秒采样水反力约 0.197 N，记录的实际施加力一致。记录帧中的方块最低点保持高于槽底，运动没有再次出现原来的异常飞出。这里的力是采样时刻的步进力，并非整帧平均力。

本次完成了这个短接口中刚体影响水、水反作用到自由刚体的闭环核对，没有新增物理通过阈值。尚未验证长时稳定性或八场景完整动作；八场景已有原外观输出属于此前未预生成 SDF 的短流程，本轮没有覆盖或冒充成 SDF 重跑结果。

## 八场景完整输出批次（运行中）

用户要求八场景都跑出后，已启动 `gpu_newton_sdf_full_20260924` 批次，复用此前原分辨率输入，使用当前预生成 SDF 路径。`run_gpu_water_workflow.py --phase run --full-duration --input-root ...` 按清单时长依次模拟、重建、原外观渲染和编码，失败保留现场并继续下一独立场景，不自动修正。清单时长合计 74 秒；不是此前每场景 1/30 秒的启动检查。实际完成状态看输出根目录的 `run_status.json`，当前不能当作八场景已完成。

完整时长实测预计需小时级至十几小时以上，已经向用户澄清完整时长或短流程的选择，未收到改变范围的回复时保持完整时长。已设置本任务自动跟进，只跟进该批次并在最终汇报后停用。

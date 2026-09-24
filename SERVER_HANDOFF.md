# 独立水体子集：服务器 Agent 交接

本文件是 2026-09-25 的最新交接状态，优先于旧实验文档中的“运行中”和旧 CPU 迁移描述。用户当前要求把工作传到服务器，由服务器 Agent 对接。本地保持暂停；本次交接没有重新运行模拟、改变场景或声称已在服务器验证。

## 目标与不可变范围

SPH_Project GPU DFSPH 负责水，Newton CUDA 负责刚体；Newton 原网格使用预生成 SDF。水—刚体通过 SPH 边界粒子双向传递运动与反力。自由刚体接受水反力；既定动作装置保持原驱动并记录反力。CPU SPlisHSPlasH 仅作参考，不是迁移目标。只做刚体，不做软体，不并入主线现象控制。

八场景按 configs/independent_water_subset.json：静水 6 秒、自由波浪传播 6 秒、传播与反弹 10 秒、受扰恢复 12 秒、容器晃动 10 秒、两容器转移 10 秒、活塞 10 秒、搅拌 10 秒。保持原 4 mm 粒子间距、原几何、原动作及原外观。搅拌已实现动作速度为 3.6 rad/s，勿按旧记录降低。水槽注水和山地岩壁落水是后续范围。

用户要求每次执行前先讲清楚计划，无需等待一般操作的确认；禁止静默执行。失败应保存日志并查因，不自行改几何、动作、液体参数、分辨率、回收规则或重跑失败任务。其他独立工作先继续，最后集中报告故障与修正方案，获用户批准后再改。不要建立物理阈值门槛；基础检查只筛运行和数据错误，画面用实际输出图组人工查看。

## 下载与文件身份

私有仓库：IsaacSuo/water-subset-server-handoff。使用有权限的 GitHub 身份。代码基于原仓库 8a00fe3；只覆盖水体相关新增代码与渲染接入改动，不包含原工作区其他未提交修改。

```bash
gh repo clone IsaacSuo/water-subset-server-handoff
cd water-subset-server-handoff
mkdir -p downloads
gh release download water-handoff-20260925 --repo IsaacSuo/water-subset-server-handoff --dir downloads
(cd downloads && sha256sum -c SHA256SUMS)
tar -xzf downloads/water-subset-assets-20260925.tar.gz
mkdir -p vendor
git clone downloads/SPH_Project-2a97e63.bundle vendor/SPH_Project
git -C vendor/SPH_Project checkout --detach 2a97e63dab680a2ee40b6a96b0f135062c6b1aaf
```

SPH_Project bundle 保留真实 Git 提交，因为现有 runner 会读取 rev-parse HEAD；不能用解压源码后伪造提交代替。其上游地址为 https://github.com/jason-huang03/SPH_Project.git 。Release 附有整个传输包的 SHA256SUMS；docs/WATER_SERVER_ASSET_MANIFEST.json 记录每份数据文件的大小和 SHA256。只做传输完整性检查，不作物理裁判。

资产包按仓库相对目录解压：

- output/coupled_scenes/gpu_newton_workflow_20260924/<case>/inputs：八场景已准备好的原分辨率 NPZ、边界 OBJ、元数据，合计约 659.5 MiB。
- registry 各 source 目录：原场景 JSON、主动装置几何和填充 NPZ、液面布局数据；同时包含八个原始 .blend。
- external_assets：八个 .blend 共用的 bryanston_park_sunrise_8k.exr。Blender 实际打开逐个检查后发现这是未打包的外部图像依赖。原 .blend 未重存，哈希保持不变。详情见 WATER_SERVER_BLEND_DEPENDENCIES.json 和资产清单。
- 少量历史图组和 SDF 接触记录，用于解释已完成到哪一步。没有打包庞大的旧逐帧缓存或重建网格，也没有把历史输出当作新的完整结果。

## 环境记录与代码入口

原环境 Python 3.12.3、Taichi 1.7.4、Newton 1.2.1、Warp 1.13.0；完整 pip freeze 在 configs/water-server-environment.freeze.txt。Blender 5.0.1，pysplashsurf 0.14.1.0，ffmpeg/ffprobe 用于编码。GPU 路线无需安装整个 Isaac Sim 或 CPU SPlisHSPlasH。pybullet 是上游导入依赖，不负责此路线的刚体步进。

主要文件：coupled_scene/gpu_dfsph/backend.py、assets.py；experiments/coupled_scenes/run_gpu_newton_water.py、run_gpu_water_workflow.py。newton_dfsph/assets.py 和 surface_assets.py 被当作纯数据辅助函数复用，该目录名称不代表执行 CPU 液体。

当前代码是本地 WSL + Windows 工具链的真实快照，**不是已经完成服务器平台适配的版本**。服务器 Agent 根据当地环境处理以下启动/路径适配，保持数值与场景不变：

1. run_gpu_newton_water.py 可通过 --upstream 指定 vendor/SPH_Project；run_gpu_water_workflow.py 尚未向子进程传该参数，默认值仍为 /mnt/y/tools/SPH_Project。批处理接入时需明确传递上游位置。
2. input.json、assets.json 与捕获报告内 source_blend/source_layout 等含 Y:/ 和 /mnt/y/isaacsim_work 的本机绝对路径。映射到新仓库位置，保存原输入副本与映射记录，几何数组和源 .blend 的哈希不变。
3. reconstruct_surface_snapshot.py 当前调用 Windows 的 pysplashsurf.exe，build_cabinet_liquid_surfaces.py::windows_path 也仅处理 Windows/WSL。服务器应调用本机重建工具，保留现有重建参数。
4. run_newton_surface_video.py 与 run_active_pour_video.py 通过 PowerShell 启动 D:/Program Files (x86)/Blender/blender.exe。服务器需换成当地 Blender 启动方式；保留原相机、材质、分辨率、采样与帧数规则。
5. .blend 内环境图路径是 Y:/scenes/HDRI/bryanston_park_sunrise_8k.exr。渲染打开文件后，将图像 filepath 在内存中映射到 external_assets 中对应文件并 reload，不重写原 .blend 破坏身份核对。现有渲染器选择 OptiX；按服务器实际支持适配设备选择，不擅自改变画面设置。
6. 原 WSL 的 LD_LIBRARY_PATH=/usr/lib/wsl/lib 仅用于该本地环境，不盲目复制到服务器。

单场景模拟入口示例（说明用，交接时未执行；输出必须是新目录）：

```bash
python experiments/coupled_scenes/run_gpu_newton_water.py \
  --input output/coupled_scenes/gpu_newton_workflow_20260924/surface_still/inputs \
  --output output/coupled_scenes/server_run/surface_still/simulation \
  --upstream "$PWD/vendor/SPH_Project" --seconds 6 --hz 1200
```

流程目标为模拟 → capture 状态缓存 → 重建 → 原外观渲染 → 视频。surface 系列渲染入口为 run_newton_surface_video.py，active 系列为 run_active_pour_video.py --simulation <已有GPU缓存> --unaccepted-backend-preview。不要漏掉 --simulation 而触发旧 PhysX 模拟路径。默认 workflow 是 1/30 秒短流程，完整任务必须使用 --full-duration。

## 实际完成度与暂停状态

八个场景此前仅各跑 1/30 秒，完成两帧缓存、重建和原外观对接。这批是预生成 SDF 修正之前的短流程，不能冒充当前 SDF 完整动作验收。

SDF 修正后，只完成了独立自由方块 0.2 秒接口验证：原几何 BVH 路径出现的假穿透消除，方块入水减速再回升，记录的水反力传入 Newton。它验证短时双向接口，不证明长时间稳定性。见已附 sdf_coupling_recorded_states.png、sdf_coupling_observations.json 等。

完整八场景批次 gpu_newton_sdf_full_20260924 在用户要求下已暂停。本地静水保存到 3.8/6 秒、115 帧；其余七场未开始。pause_state.json 为用户暂停记录，probe_report.json 的 running 是冻结前状态。本地进程由 SIGSTOP 暂停，自动跟进 gpu 同样 PAUSED；本次没有恢复它们。

**这不是可跨机器恢复的求解器检查点。** 现有输出只有采样状态，没有恢复 DFSPH 全部内部状态的入口；服务器无法直接从 3.8 秒无损续跑。若后续在服务器执行完整静水，应从原始输入起跑。不要宣称暂停的本地作业已完成，不要自动恢复本地或创建重复本地批次。

已知静水在原笔记本上非常慢；不要用未测量的服务器型号承诺加速倍数。当前交接仅做文件迁移，服务器执行、部署适配与结果检查由接手 Agent 接续。

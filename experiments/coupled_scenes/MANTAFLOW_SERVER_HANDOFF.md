# Mantaflow 倾倒修复：服务器烘焙交接（2026-10-01）

代码分支：`rental-gpu-rerun-20260927`。本次入口是 Blender Mantaflow CPU
液体求解，原壶按既定动画运动；不启动旧 GPU DFSPH/Newton 任务。
在收到的本次提交上建立干净 worktree，保留服务器已有工作树和运行任务。

## 固定输入与参数

- Blender **5.0.1**，其 Python 必须能导入 `numpy`、`openvdb`、`manta`。
  碰撞适配器使用该版本内部网格接口，其他版本会明确拒绝运行。
- 原设计 `01_container_transfer.blend` 和这次实际使用的初始水资产，获取方式见下。
  文件 SHA-256 见 [MANTAFLOW_INPUTS_20261001.json](MANTAFLOW_INPUTS_20261001.json)。
- 384 分辨率，实际网格 384×214×245，每格约 3.229 mm；液面网格倍率 1。
- 原壶、盆、台面网格；OpenVDB 有符号距离场每个实际求解子步更新。
  碰撞偏移 0，没有加厚或简化物理代理；Fractional Obstacles 关闭。
- CFL 1，子步 6–32，粒子带宽 30；原壶在 2.0 秒开始倾转，5.0 秒达到最大角度。
- 本次完整验证区间是 **1.8–3.8 秒，55–115 帧，共 61 帧，30 FPS**。
  在保持这组设置的前提下，根据服务器分配的 CPU 线程数选择 `-t`。

距离场半宽 4 格是存储带宽，不移动模型表面。内部临时标记只分配求解器字段，
其数据被原模型的距离场完全覆盖，烘焙后移除。适配器是 Python 运行时代码；
重新烘焙必须调用 `mantaflow_pour.py`，只在保存的 .blend 中点 Bake 不会安装此后端。

## 下载本次输入

已在现有 `water-handoff-20260925` Release 补充独立的约 6 MB 输入包及校验文件；
本次包包含原始 .blend、`assets.json`、`geometry_and_fill.npz` 的字节副本，
不包含历史烘焙缓存。旧 Release 的其他资产及原 `SHA256SUMS` 保持原样。

在本次代码 worktree 的根目录执行：

```bash
set -euo pipefail
mkdir -p downloads
gh release download water-handoff-20260925 \
  --repo IsaacSuo/water-subset-server-handoff \
  --pattern 'mantaflow-pour-inputs-20261001.*' --dir downloads
(cd downloads && sha256sum -c mantaflow-pour-inputs-20261001.sha256)
INPUT=output/coupled_scenes/mantaflow_inputs_20261001
test ! -e "$INPUT"
tar -xzf downloads/mantaflow-pour-inputs-20261001.tar.gz
```

资产 JSON 中的历史 `Y:` 和 `/mnt/y` 来源路径是溯源记录，保持字节不变。
下面的入口通过显式 `--blend`、`--assets` 读取本地文件，不依赖这些历史路径。
CPU 烘焙不需要 HDRI；Windows 最终渲染再指定原环境图。

## 服务器 CPU 烘焙与粒子验收

以下命令在仓库根目录运行。`BLENDER` 指向服务器实际 Blender 5.0.1，
`CPU_THREADS` 按任务获得的 CPU 配额设置；`OUT` 必须是不存在的新目录。
日志保存在输出目录旁，即使启动失败也能保留。

```bash
set -euo pipefail
BLENDER=blender
CPU_THREADS=16
INPUT=output/coupled_scenes/mantaflow_inputs_20261001
OUT=output/coupled_scenes/mantaflow_server_384_20261001
mkdir -p output/coupled_scenes

"$BLENDER" --background --factory-startup --python-exit-code 1 \
  --python-expr 'import bpy, numpy, openvdb, manta; assert bpy.app.version == (5,0,1); print("Mantaflow dependencies OK")'

"$BLENDER" --background -t "$CPU_THREADS" --python-exit-code 1 \
  --python experiments/coupled_scenes/mantaflow_pour.py -- \
  --blend "$INPUT/design/01_container_transfer.blend" \
  --assets "$INPUT/assets_4mm" --output "$OUT" \
  --start-seconds 1.8 --end-seconds 3.8 --resolution 384 --mesh-scale 1 \
  --collision-backend openvdb --collision-thickness 0 --effector-subframes 0 \
  --timesteps-min 6 --timesteps-max 32 --cfl 1 --particle-band-width 30 \
  2>&1 | tee "${OUT}_bake.log"

"$BLENDER" --background -t 2 --python-exit-code 1 \
  --python experiments/coupled_scenes/verify_mantaflow_particles.py -- \
  --output "$OUT" 2>&1 | tee "${OUT}_particles.log"
```

建议放入服务器独立 tmux/systemd 任务，记录外层退出码。失败时保留日志和缓存，
不要把 `bake_complete=true` 单独当作验收通过：后续验证也须完成且进程退出 0。

检查 `report.json` 的 Blender 版本、线程数、721 附近的实际子步数及完整缓存，
以及 `validation.json` 的倾转前漏水和逐帧水平集体积。
子步数是本次本地 61 帧记录，跨平台数值运行不要求恰好相同。
`particle_validation.json` 应有 `complete=true`、`all_frames_checked=true`、`passed=true`。
粒子检查覆盖壶底和盆底轮廓内、底面外侧 5 cm 区域，容差 1 mm；静止段另检查全局最低点。
`--frames 57` 可做局部诊断，写入 `particle_validation_partial.json`，不能代替完整验收。

回传完整 `OUT`（包括 `cache`、.blend 与所有 JSON），以及旁边两个日志。
缓存回到另一台机器后需要重新映射缓存和资产路径。

## Windows 渲染已有服务器缓存

将完整 `OUT` 复制到 Windows 后，使用下面入口；`--hdri` 指向原始
`bryanston_park_sunrise_8k.exr`。该图也在旧资产包的 `external_assets` 内。

```powershell
& 'D:\Program Files (x86)\Blender\blender.exe' --background -t 8 --python-exit-code 1 `
  --python experiments/coupled_scenes/render_mantaflow_keyframes.py -- `
  --output 'Y:\path\to\mantaflow_server_384_20261001' `
  --hdri 'Y:\scenes\HDRI\bryanston_park_sunrise_8k.exr'
```

入口检查完整验收、Windows OptiX GPU、五帧的缓存顶点数，然后只渲染
1.8、2.0、2.6、3.2、3.8 秒，原 `DesignView_00` 镜头、1280×960、64 samples。
不生成视频。添加 `--verify-only` 可只核对缓存与设备，不输出新图片。
保存的 `repaired_windows.blend` 使用当前 Windows 缓存路径。

## 已有验证与限制

- WSL 8 线程完整 61 帧烘焙约 2 小时 24 分钟。原几何和镜头检查通过。
  全部实际粒子底部检测无穿透；碰撞姿态逐帧最大顶点误差约 1.34e-7 m。
- 倾转前水平集体积最大漂移约 0.97%。末帧水平集体积较首帧增加约 **13.61%**；
  表面重建网格体积增加约 37%，其半径会影响薄流、水滴的体积估计。
  保留两种测量，不声称严格质量守恒，也未修改缓存来消除数字偏差。
- Windows 16 线程，1.8–2.1 秒共 10 帧，烘焙 1436.35 秒；粒子检查通过。
  剔除首帧后的相同 9 帧，Windows 平均 145.83 秒，原 WSL 136.39 秒，慢约 6.92%。
  平台和运行负载同时不同，这个结果不隔离线程收益，也不预测服务器加速比例。
- 发布前，新粒子验收入口已在 Windows 完整缓存及 Linux 的 1.87 秒帧检查；
  新缓存渲染入口已在 Windows OptiX 核对全部五帧，预检没有生成额外图片。
  租用服务器上的本次完整烘焙尚待执行。

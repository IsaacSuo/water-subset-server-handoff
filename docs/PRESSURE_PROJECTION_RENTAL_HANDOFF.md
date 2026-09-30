# 租用服务器：当前 B 压力求解重跑（2026-09-30）

当前代码分支：`rental-gpu-rerun-20260927`。本页优先于旧交接的运行状态与模型描述。
用户当前要求把代码同步到 GitHub，由租用服务器运行 B；本地不再启动模拟。

## 已完成与尚未完成

- A：修复 divergence 欠邻居粒子自身压力锁零，原输入到 4.1667 秒、126 帧和原外观视频已完成；撞槽仍有大量粒子飞散。
- A 全程接受 10,984 子步，其中 3,437 子步未满足原有最终收敛条件；176 次 CFL 回退，状态恢复核对通过。
- B：在 A 上加入所有边界局部残差及不收敛回退，只保存到 1.0 秒，随后子进程和队列都消失。
  没有捕获退出码；人工终止、会话清理等原因无法区分，不能声称是管理员终止。
- B 在中断前接受 4,620 子步，751 次压力相关回退，接受的子步全部收敛。它尚未到达倾倒撞槽，不能宣称问题已解决。

## 固定的 B 配置

- SPH_Project GPU DFSPH + Newton CUDA；FP32 求解；不改为 CPU SPlisHSPlasH。
- 原始倾倒输入 `output/coupled_scenes/gpu_newton_workflow_20260924/active_transfer/inputs`。
- 输入 NPZ SHA256：`849794f50617a5165086066ef1fee3d2ba3d87e205b33504d77a312c07b9c6cc`。
- 100,434 粒子，间距 4 mm、支持半径 8 mm，原几何、填充和动作。
- 完整 Akinci cohesion + curvature，系数 1.0；adhesion 为零。
- Bender2019 Volume Maps，非负累计压力 Jacobi，松弛 0.5，最终速度冲量只应用一次。
- 基础 1200 Hz；最小步长除数 **64**，下限 13.020833 µs。density/divergence 上限各 300 次，最少各 2/1 次。
- divergence 门槛为 20 个流体邻居，mask 在投影开始时冻结；自身压力始终锁零，但仍接受邻居压力。
- 所有 Volume Map 边界，包括静止接水槽，纳入局部最大残差；沿用已有容差。
- 不收敛最多减半 4 次，完整回滚重算两个投影；成功时步长上限 ×1.5 恢复。
- 耗尽压力回退或触及下限时，只可接受有限且满足 CFL 的子步，并明确记录未收敛。
  非有限数和 CFL 违规不会通过这个后备路径接受。

## 环境与输入

沿用服务器已装环境：Python 3.12、Taichi 1.7.4、Newton 1.2.1、Warp 1.13.0、NumPy、SciPy、trimesh、pybullet。
重建/视频需要 pysplashsurf 0.14.1.0、Blender、ffmpeg/ffprobe。
Volume Maps 使用 Newton/Warp 的 SDF 接口，按上述已验证版本运行。

旧资产包仍可用，无需上传本地历史缓存或 SDF/Volume Map 缓存。
初次启动会从原网格自动生成并缓存地图，首次运行额外耗时不能当作稳态仿真速度。
`vendor/SPH_Project` 必须保留实际 Git 仓库，提交为 `2a97e63dab680a2ee40b6a96b0f135062c6b1aaf`。

先激活已有 Python 环境，核对原输入：

```bash
sha256sum output/coupled_scenes/gpu_newton_workflow_20260924/active_transfer/inputs/input.npz
git -C vendor/SPH_Project rev-parse HEAD
PYTHONPATH=tests python -m unittest test_pressure_projection test_dfsph_reference test_volume_maps test_akinci2013
```

## B 完整短跑、重建、原外观视频

以下命令不依赖本地 A 或 baseline 输出。`--gpu 0` 指租用机器上的物理 GPU 编号；
多卡时只替换这个编号。输出必须是不存在旧运行结果的新目录。

```bash
python -u experiments/coupled_scenes/run_pressure_projection_ab.py \
  --case B --gpu 0 \
  --input output/coupled_scenes/gpu_newton_workflow_20260924/active_transfer/inputs \
  --output output/coupled_scenes/rental_pressure_B_20260930
```

队列从 t=0 的原输入运行到 4.1667 秒，之后自动重建并按原 .blend 的相机、材质和
画面设置渲染，再编码视频。不要提供 `--initial-frame`；中断的 1 秒缓存不是完整检查点。
若明确需要和同卡其他任务共享，增加 `--allow-shared-gpu`；这只跳过渲染前的空闲等待。
一张空闲租用卡不需要该参数。不要同时启动 A 或八场景批次。

请由服务器 agent 放入它自己的独立 tmux/systemd 任务，保存启动命令、外层退出码和日志。
队列记录子进程 PID、仿真返回码和渲染返回码，但无法在被 SIGKILL 时自行补写退出记录。

若先只做仿真，等效命令是：

```bash
CUDA_VISIBLE_DEVICES=0 python -u experiments/coupled_scenes/run_gpu_newton_water.py \
  --input output/coupled_scenes/gpu_newton_workflow_20260924/active_transfer/inputs \
  --output output/coupled_scenes/rental_pressure_B_sim_20260930 \
  --upstream vendor/SPH_Project \
  --seconds 4.166666666666667 --hz 1200 \
  --akinci-coefficient 1.0 --boundary-model volume_maps_bender2019 \
  --minimum-density-iterations 2 --minimum-divergence-iterations 1 \
  --minimum-dt-divisor 64 --local-residual-boundary-scope all \
  --pressure-convergence-retries 4 --diagnose-minimum-dt-failure \
  --solver-verification-window 3.5 4.166666666666667
```

## 交回内容与判读

提供 `status.json`、B 的 `probe_report.json`、`pressure_substeps.jsonl`、
`receiver_rebound_events.json`、`comparison.json`、`comparison_frames.csv` 和
`B_lock_zero_all_boundary_retry.mp4`。没有 A/baseline 缓存时比较文件只含 B，明确说明缺少对照。
逐子步记录迭代数、最终残差/容差、回退和未收敛接受标记；逐帧记录壁面第一层密度、能量和欠邻居数。

视频以实际 3.5–4.1667 秒撞击画面人工查看，不增加物理通过门槛。
离壁/入射速度比受压力传能、流向变化和采样影响，不能单独证明数值能量注入。
机械能没有计入 Akinci 表面势能，现有 motor-work 账本仍有 divergence 反力延迟消费的限制。
边界主导占比采用接水槽接触粒子子步口径，和旧首次分离粒子组的 45.3% 不能直接等同。
失败保留原日志和数据，先汇报原因及修正方案，不擅自改物理参数重跑。

# 现象数据主线推进 v2（2026-09-21）

后续更新：A 的原生梁 attachment 材料入口及两条加载/高位对照已接入主线，当前统一目录为 19 条、9691 状态、503 帧观测，见 [梁接入 v3](PHENOMENON_BEAM_INTEGRATION_V3.md)。下文保留 v2 当时的结果与失败探针历史，不代表 A 仍受阻。

用户已确认：展示阶段结束，按布料修正、现象缺口、入口复用、统一读取的顺序推进。当前继续 C3/C4，不启动训练或新 split。已接受的视频没有改写。

## 实际交付

本轮成功生成并完整封装 5 条：两条修正布料、两条扫描资产滚滑、一条塑性材料变化。另运行两条悬臂能力探针，均未达到加载—保持—卸载回弹目标，保留完整物理记录及失败诊断，不计入已完成现象覆盖。

当前 [统一目录](../output/world_model_dataset/v0_2/phenomenon_progress_data_v2/catalog.json) 为 **17 条、7769 条状态、421 帧诊断观测**。它引用首批的 12 条和本轮 5 条，显式用新布料替代目录中的旧两条；旧文件和旧索引均保留，替代关系在 `superseded` 中记录。四类条数为整体运动 8、体积响应 5、布料 2、绳索 2。

全部默认核心读取、状态/观测流及计数检查通过；物理状态哈希保持不变。11 项相关测试通过，日志为 `phenomenon_progress_data_v2/tests.log`。观测仍为 10 Hz、160×120、单视角诊断颜色 RGB/光轴深度/实例编号，不含完整材质贴图，不冒充正式视觉训练交付。布料、体积节点、塑性点和绳索各保留原生表示。`training_admission=false`。

## 布料修正已在主线采用

主线批量作业调用 `/mnt/y/isaacsim_work_cloth_stability/experiments/material_response/scene_input/material_entry.py`，采用上游 `ec7e904` 补丁；没有覆盖另一材料工作区。两条配置保留原初态及 X=2.73/2.83 m 对照，时长改为 5 秒；显式修正闭合桌面绕序，位置迭代 32→8。所有顶点、无向三角面、布片初态、材料和阻尼逐项核对未变。原桌架开放网格保持原样。

| 初始 X | 末半秒全布最大帧间位移 | 末半秒原生速度 RMS | 末半秒原生最大速度 |
| --- | ---: | ---: | ---: |
| 2.73 m | 0.103 mm | 0.00655 m/s | 0.04552 m/s |
| 2.83 m | 0.310 mm | 0.00584 m/s | 0.05039 m/s |

检查跟踪各自旧缓存所选的固定桌沿/悬垂节点身份，保留 1.5–2、2.5–3、4.5–5 秒局部指标。两个配置通过本次明确的局部稳定性检查；不授予连续物理收敛、实物标定或可信接触力。已看初末关键画面，见 [缓存抽查](../output/world_model_dataset/v0_2/phenomenon_progress_data_v2/cloth_keyframes.jpg)。

证据在 `phenomenon_cloth_stable_v1/review_273.json`、`review_283.json` 和 `adoption_geometry_audit.json`。采集器仅接受匹配该状态哈希的审核文件；别的布料不会自动解除待复核标记。复现用 `configs/dataset/v0_2/phenomenon_cloth_stable_v1/generation_pinned.json`，入口源文件哈希已固定；依赖变动会在生成前拒绝。实际执行的 `batch.json` 保持原样。

## 入口复用已有新增证据

- 同一个刚体入口分别接收洋葱与红薯扫描资产；两者最大尺寸同为 0.105 m、质量 0.12 kg、初速度 0.65 m/s、初始角速度零，摩擦及原场景相同。支承高度由网格自动导出，碰撞继续使用资产已有 SDF 表示。没有逐件逻辑。形状、归一化质量分布及支承高度随资产变化，不把它描述成单一几何标量反事实。
- 塑性使用同一红薯网格及 12 cm 落高，只将 `yield_pressure` 从 4000 改为 12000 Pa。末态 Jp 均值从首批的 0.873733 变为 0.995727；这是该固定仿真配置的材料内部量变化，不是经过标定的食品性质。

具体配置为 `configs/dataset/v0_2/phenomenon_reuse_v1/`，生成记录为 `output/world_model_dataset/v0_2/phenomenon_reuse_v1/batch.json`。

## 弯曲探针没有通过，不计完成

尝试利用现有轴对齐体积块表达 0.38×0.05×0.025 m 梁，在原桌沿 x=2.98013997 m 留 0.20 m 悬出。桌面为实际下支承，增加可见静态顶盖、后挡及侧轨形成接触夹具；没有冻结体积节点。0.2 kg 压头沿 Z 受 15 N 上限阻抗控制，原桌面和场景保持原位。

两次 4 秒记录各 961 状态：

| 配置 | 根部最大平移 | 加载命令期间最小采样间隙 | 结果 |
| --- | ---: | ---: | --- |
| E=0.3 MPa | 12.00 mm | 10.48 mm | 自重挠曲和根部滑移明显；未取得压头加载接触证据，不能把命令期间的下移称为主动加载 |
| E=3 MPa | 3.35 mm | 0.751 mm | 短暂接近接触，但只占加载命令时段约 8.6%；没有可靠保持和卸载回弹 |

两条末态压头净空约 84.8/68.6 mm；净空本身不能证明此前有完整加载过程。审核使用原生节点身份、根部刚体配准后的端部位移、截面中心轨迹和 Tet J，没有拿整体倾倒当弯曲。[原生截面轨迹](../output/world_model_dataset/v0_2/phenomenon_progress_data_v2/bending_probe_sections.png)已抽查。E=3 MPa 仅为一次有依据的修正；不继续扩大扫描。

路径：`phenomenon_bending_v1/` 和 `phenomenon_bending_stiffer_v1/`，各自含 `bending_review.json`。物理记录可用 `open_episode(..., require_complete=False)` 读取；未补诊断观测，不列入上述 17 条完整数据。下一步需要真实夹具连接/约束、针对自重形态的压头可达行程，以及实际加载保持证据；见更新的 [材料需求](PHENOMENON_PILOT_MATERIAL_REQUEST.md)。

## 其余接口缺口与推进顺序

当前材料公开入口仍只支持被动布料和无外接负载的绳索。CPU 合同探针确认动态 attachment 与 external loads 输入被拒绝，未启动求解器；证据为 `phenomenon_progress_data_v2/material_gap_audit.json`。这说明当前适配器缺失，并不说明底层引擎不可能实现。

1. 材料侧先补有限动态夹头—布料的双向连接、绳段—有限动态负载连接；按已有明确坐标/质量/时序各做一条完整 smoke。不同后端独立输出。
2. 弯曲使用实体夹具约束并检查实际压头接触/保持，解决上述失败后再增加对照。不得把静态几何接入再次列为阻塞。
3. 新能力成立后各加一条有解释力的条件变化，沿用统一读取与本次显式审核机制；当前无需大矩阵。
4. C3/C4 仍未整体完成，完整外观观测生产、规模化和正式用途准入分别处理。C5、黏弹性、流体、损伤和分离继续暂停。

## 调用入口

在主线 WSL 目录设置 `PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python`。

```bash
# 新运行使用新输出目录。旧缓存不覆盖。
$PY -m world_model_dataset.phenomenon_pilot_batch \
  --plan configs/dataset/v0_2/phenomenon_cloth_stable_v1/generation_pinned.json \
  --output output/new_cloth_batch

$PY -m world_model_dataset.cloth_stability_review \
  --episode output/new_cloth_batch/cloth_edge_273_stable_v1/episode \
  --reference output/world_model_dataset/v0_2/phenomenon_pilot_v1/cloth_edge_273/episode \
  --edge-x 2.98013997 --output output/new_cloth_batch/review_273.json
# 对 283 同样审核；然后按各自 ID 传入 --quality-evidence。
$PY -m world_model_dataset.phenomenon_collect \
  --batch output/new_cloth_batch/batch.json --output output/new_cloth_data \
  --ids cloth_edge_273_stable_v1 \
  --quality-evidence cloth_edge_273_stable_v1=output/new_cloth_batch/review_273.json

# 统一索引只记录显式替代，不移动或覆盖源缓存。
$PY -m world_model_dataset.phenomenon_catalog \
  --index output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/index.json \
  --index output/world_model_dataset/v0_2/phenomenon_progress_data_v2/index.json \
  --supersede cloth_edge_273=cloth_edge_273_stable_v1 \
  --supersede cloth_edge_283=cloth_edge_283_stable_v1 \
  --output output/world_model_dataset/v0_2/phenomenon_progress_data_v2/catalog.json \
  --verify-streams
```

# 原生梁材料入口的主线采用 v3（2026-09-21）

A 已完成材料入口与主线采集接入：原教师桌中的实体夹具约束、有限压头加载/保持/撤离，以及压头全程高位对照。主线只复用两份最终 delivery 缓存，没有重新仿真、移动原场景或重做旧展示视频。B 布料拖动、C 绳索向有限负载传递运动仍未完成，不能从 A 的固定夹具连接推定它们已接通。

## 统一数据与证据

[统一目录](../output/world_model_dataset/v0_2/phenomenon_beam_data_v3/catalog.json) 选用 **19 条 episode、9691 条状态、503 帧诊断观测**：整体运动 8、体积响应 7、布料 2、绳索 2。引用 v2 的 17 条及新增两梁，保留两条旧布料的显式替代历史；此前失败的弯曲探针仍不计入。默认完整读取和状态/观测流逐条验证，`training_admission=false`，没有新增 split 或启动训练。

两条梁各 4 秒，物理状态 240 Hz、961 帧；各有 960 条目标命令、961 条实际执行器状态、960 条实际施力。新增观测各 41 帧，10 Hz、160×120、同一标定相机，含 RGB、光轴深度及实例编号。外观为诊断颜色，不是完整材质视觉训练交付。梁观测使用原生 simulation Tet 的外边界，剔除内部共享面，保留原节点身份，不插值平滑；夹具、压头及原桌面/桌架保留，动态对象使用实际位姿。

- [加载/高位对照关键帧](../output/world_model_dataset/v0_2/phenomenon_beam_data_v3/beam_keyframes.png)：0、1.8、2.5、4 秒，分别检查初态、保持、撤离及末态。
- [采用核对](../output/world_model_dataset/v0_2/phenomenon_beam_data_v3/adoption_audit.json)：源 manifest 未变，源与副本状态、命令、实际执行器和施力记录相等，native/attachment 哈希相等，每个采样画面均可见梁、夹具及压头。
- [原缓存对照审核](../output/world_model_dataset/v0_2/phenomenon_beam_data_v3/pair_review.json)：绑定两条物理状态和 native 缓存哈希；不是数值收敛或材料标定证明。
- [测试日志](../output/world_model_dataset/v0_2/phenomenon_beam_data_v3/tests.log)：16 项相关测试通过，包含 Tet 外表面、实际压头位姿、证据归属、向量施力上限/缺失标记和目录替代历史。

## A 的物理结果与边界

成功配置 E=3 MPa，0.2 kg 压头、15 N 施力上限，目标下降 80 mm；不是此前失败探针的 0.3 MPa/35 mm 配置。可见 1 kg 夹具由原生 fixed joint 固定到世界，近端 196/1372 个体积节点通过 native attachment 连接夹具；其余梁节点自由响应。没有隐形冻结整梁或运动学压头。

加载与高位对照仅 `plate.schedule` 不同，初态、拓扑和材料一致。保持窗口 1.3–2 秒，梁端最后 20 mm 节点相对高位对照平均下移 **30.82 mm**；每个保持采样时刻均有压头底面近接触。撤离后 2.5–4 秒实际压头采样净空最小 **45.01 mm**，梁端平均距高位对照形态约 **0.857 mm**，即回到自重弯曲形态附近，并非回到初始水平。根部 attachment 节点最大位移 **0.0534 mm**，加载样本最小 Tet J **0.8152**，峰值实际施力 **3.692 N**。

主线近接触审核使用压头实际姿态的有向盒与 collision 节点距离；与上游轴对齐盒分类的节点占比不可直接等同。采样近接触不是连续碰撞或接触力真值。施力记录是受限控制器实际输入，不能当接触反力；接触反力、attachment 反力、执行器饱和标记继续明确 unavailable，整体体积角速度保留 null。体积节点/速度及拓扑仍为权威表示。

## 可复用入口与复现

材料实现位于 `/mnt/y/isaacsim_work_beam_attachment/experiments/material_response/scene_input/`，交付提交 `147bf4e`。说明见 [BEAM.md](../../isaacsim_work_beam_attachment/experiments/material_response/scene_input/BEAM.md)。同一 `kind=beam` 入口支持 `prepare → simulate → package`，梁尺寸/材料、夹具及压头时序为配置参数；当前支持轴对齐梁、固定实体夹具、Z 导向有限压头，不泛化为任意扫描弹性体或自由动态夹头。本轮没有合并材料补丁或更改材料工作区；可移植补丁已在材料基线做应用检查。

采用源为该工作区 `output/material_entry/beam_load_delivery/episode` 和 `beam_hold_delivery/episode`。主线 [缓存采集配置](../configs/dataset/v0_2/phenomenon_beam_v3/collection.json) 固定源 manifest 哈希、审核哈希及同一相机。它是缓存采集清单，不触发求解。

```bash
PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
# 在主线工作区运行；重新采集请选择新的输出目录。
$PY -m world_model_dataset.phenomenon_collect \
  --batch configs/dataset/v0_2/phenomenon_beam_v3/collection.json \
  --output output/new_beam_data

$PY -m world_model_dataset.phenomenon_catalog \
  --index output/world_model_dataset/v0_2/phenomenon_progress_data_v2/catalog.json \
  --index output/new_beam_data/index.json \
  --output output/new_beam_data/catalog.json --verify-streams
```

新仿真须先从材料公开入口产生新 episode，再用 `beam_pair_review --load ... --hold ... --output ...` 检查对应缓存并登记新哈希，不复用旧审核结论。统一读取继续使用 `open_episode(path)`、`geometries('Beam')`、`controls()`、`actuator_states()`、`actuator_efforts()` 和 `observations()`。

## 下一步

按 [材料需求 B/C](PHENOMENON_PILOT_MATERIAL_REQUEST.md) 依次推进有限动态夹头—布料双向连接与桌面滑移、绳索松弛到绷紧并驱动外接有限负载。每项先一条完整 smoke，再加一条有解释力的低力/余长对照；接口不足保留具体缺口，不阻塞其他类型。继续沿用原生状态、命令/实际状态分离、缓存检查和统一读取，不扩大矩阵或围绕视频打磨。C3/C4 仍在推进，C5、黏弹性、流体、损伤和分离继续暂停。

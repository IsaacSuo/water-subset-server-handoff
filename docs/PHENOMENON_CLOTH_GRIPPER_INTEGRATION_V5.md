# B 动态夹头拖布的主线采用（2026-09-22）

本轮暂停模板扩展，采用材料线已经验证并由用户观看正常速度预览的三份 B 缓存。主线没有重新仿真、改变物理参数或重做展示视频；材料工作区保持原样，补丁没有合并。

## 采用来源与结果

材料工作区 `Y:/isaacsim_work_cloth_gripper`，分支 `feat/material-cloth-gripper-20260921`；物理与诊断提交 `f985b5c`，预览提交 `2f12001`。来源分别为 `output/cloth_gripper_substeps/depen005_hold5s/episode`、`depen005_blocked5s/episode`、`depen005_low5s/episode`。

三条均为 5 秒，物理 1200 Hz、保存状态 120 Hz，各 601 状态/实际夹头记录、6000 命令和施力记录。cloth 保持 1785 个原生节点、3400 个三角面；有限动态夹头连接 18 个节点，质量 0.15 kg，不把目标当成实际运动。受阻条件增加可见反向夹具，低力条件仅将上限从 4 N 降至 0.2 N；原场景网格、初始布节点、拓扑和时间轴保持相同。

| 条件 | 夹头实际净行程 | 布节点平均 X 位移 | 4.5–5 s 保存峰速 | 同窗口最大节点帧移 |
| --- | ---: | ---: | ---: | ---: |
| 自由夹头，4 N | 213.787 mm | 204.404 mm | 0.2721 m/s | 1.976 mm |
| 反向夹具受阻，4 N | 44.101 mm | 18.948 mm | 0.0961 m/s | 0.549 mm |
| 低力，0.2 N | 0.947 mm | −0.416 mm | 0.0717 m/s | 0.519 mm |

主线对三条保存状态逐帧检查：节点位置/速度、三角拓扑、实际夹头/反向夹具位姿速度，与 native 一致；命令、实际施力及时间步也一致。受阻样本排除了导轨限位与夹头直接碰反向夹具；按实际姿态计算的包围盒仍有分离轴。功能结果支持有限夹头通过布料受到阻力，施力不是接触/attachment 反力测量。

[采用审核](../output/world_model_dataset/v0_2/phenomenon_cloth_gripper_data_v5/adoption_review.json)绑定三份状态、native 和 attachment 哈希，并引用原稳定性证据的路径和哈希。原始场景闭合桌面的显式绕序修正和开放桌架原样保留，不能描述为移动或改造场景。

## 数值与视觉状态分开

本轮采用配置是 `max_depenetration_velocity_m_s=0.05`，它限制接触纠正速度，**不裁剪布节点总速度**。保持 1200 Hz、8 次迭代、自碰撞和接触开启；运行时回读为 Isaac Sim 6.0.1 / PhysX 110.1.13。B 已采用面内 200 kPa，不能与原被动 dense 的 20 kPa 称为相同材料。

用户已认可正常速度预览的当前观感。主线把此事独立记录为用户视觉认可，未改写材料线较早证据中的“未视觉验收”历史文字，也未清除原数值质量警示。仍有轻微局部颤动；折层自接触与桌面支承下接触纠正放大局部回弹是证据支持的解释，不是已定位具体内部约束。粗网格影响观感仍是用户提出的可能原因，不是本轮已验证结论。

独立复跑与几何筛查使用材料线已交付证据，主线没有重复做全面数值研究。不能从保存帧未发现穿透推断连续时间零穿透，不能拿 120 Hz 与逐物理步 1200 Hz 峰值混比。接触和 attachment 反力继续 unavailable，执行器饱和标记缺失则保持 unavailable；根据施力是否达到上限计算的比例明确为派生诊断。`training_admission=false`，采集结果仍保留 `needs_stability_review` 标记。

## 主线数据与读取

[统一目录](../output/world_model_dataset/v0_2/phenomenon_cloth_gripper_data_v5/catalog.json)现含 **26 条 episode、13418 条状态、740 帧观测**；整体运动 12、体积响应 7、布料 5、绳索 2。增加 B 三条，不替代原来的两条被动布料。历史旧布料替代关系继续保留，C3/C4 未整体完成。

三条主线副本位于 `phenomenon_cloth_gripper_data_v5/delivery/`，每条新增 51 帧 10 Hz、160×120 诊断 RGB/光轴深度/实例编号；没有把它当完整材质视觉训练数据。现有几何回放已支持 B 的原生布面、实际动态夹头、反向夹具及原桌面/桌架，不需改共享 schema/runner，也不需修改渲染器。

首次复制在观测生成前中断，没有完成标记；不完整目录 `phenomenon_cloth_gripper_data_v5/depen005_hold5s/` 保留且不纳入索引。重试使用全新 `delivery/` 目录，日志为 `collection.log`。源 episode 未改写。

[诊断关键帧](../output/world_model_dataset/v0_2/phenomenon_cloth_gripper_data_v5/keyframes.png)抽查 0、1.5、3、5 秒，关注主体和夹具可见性及实际运动；它不替代用户已经查看的原正常速度预览。[交付核对](../output/world_model_dataset/v0_2/phenomenon_cloth_gripper_data_v5/delivery_audit.json)检查源 manifest 未变、物理记录相等、native/attachment 哈希相等、51 帧观测完整，并记录运行时实际数值配置。

本轮修正时间窗口接口的一项实际限制：控制/effort 允许按物理时钟记录在保存状态之间，不再要求每条都能找到同一时间的保存状态。窗口保留各流原始频率，不插值、不降采样、不补造末态之后的控制。三条 4.5–5 s 窗口分别有 61 状态、61 实际夹头记录、600 命令、600 施力及 6 帧观测。

主线 23 项相关测试通过，材料入口 12 项合同测试通过；完整 B 补丁在未修改的 A 基线 `147bf4e` 上 `git apply --check` 通过。本轮没有应用补丁，生成入口仍引用材料独立工作区。

## 重用调用

主线[采集清单](../configs/dataset/v0_2/phenomenon_cloth_gripper_v5/collection.json)固定源 manifest、审核文件哈希、条件和相机。以下仅复用缓存，输出必须采用新目录：

```bash
PY=/home/fangsuo/isaacsim_work_material_response/.venv-material/bin/python
$PY -m world_model_dataset.phenomenon_collect \
  --batch configs/dataset/v0_2/phenomenon_cloth_gripper_v5/collection.json \
  --output output/new_cloth_gripper_data
$PY -m world_model_dataset.phenomenon_catalog \
  --index output/world_model_dataset/v0_2/phenomenon_templates_data_v4/catalog.json \
  --index output/new_cloth_gripper_data/index.json \
  --output output/new_cloth_gripper_data/catalog.json --verify-streams
```

需要新物理时使用材料线 `material_entry.py`，输入选 `classroom_cloth_drag_depen005_trace5s.json` 及相应受阻/低力配置；新缓存需独立审核，不能沿用旧缓存的审核哈希。准确调用见材料线 `CLOTH_GRIPPER_SUBSTEPS.md`。主线 `cloth_gripper_adoption` 默认不授予视觉认可，只有确有用户观看并接受对应预览时才传 `--user-visual-acceptance`；缓存功能检查本身不能推出用户验收。

模板生成工作已暂停，由用户另起支线处理。C 绳索向有限负载传力继续由材料线推进；B 已完成本轮工程数据接入，后续不围绕其已接受画面反复打磨。数值质量、完整视觉数据和训练准入保持独立状态。

# 参数化物理资产：主线开发接入

2026-09-16。接收探索工作区`/home/fangsuo/isaacsim_work_flexible_explore`
的首批`physical-asset-source/1`，入口说明为该工作区的`docs/PHYSICAL_ASSET_LIBRARY.md`。
这是开发加载证据，不是正式资产准入，也没有合并探索测试台或修改探索目录。

## 最小改动

保持v0.2合同、C1控制、C2场景和episode路径不变。

1. `causal_runner.prepare`的既有mesh分支增加可选`physical_asset_source`来源。
   缺省仍走原来的STL注册表，不改历史香蕉多凸块或象形物SDF384配置。
2. `physical_asset_bridge.resolve_asset`在CPU准备阶段调用探索侧`load_variant`，
   复用尺寸、质量/密度、摩擦、恢复系数及完整惯量计算，不复制另一套质量算法。
   loader按SHA256固定，并将源码、参数、源资产JSON及来源哈希保存到resolved geometry。
3. 原生mesh作者补齐可选SDF subgrid、位深与三角缩减参数；原有只声明resolution的配方保持默认行为。
   此来源固定保留探索侧SDF256/6/16-bit/reduction=1，缺GPU则明确报错，不退回凸包。
4. 启动读回增加`com_pose_body_xyz_xyzw`，含局部COM平移和惯性主轴四元数；
   完整惯量仍使用原有原生tensor读回。物理profile在逐对象解析时复制，避免求得质量污染共享profile。

episode最终保存缩放后的完整网格、求得的质量、COM及完整惯量；无需原始STL。
native运行、缓存loader和回放不导入探索Python、不访问探索路径；这些路径只作为准备来源/溯源字段保存。
准备新尺寸/新参数的episode仍需要可用的探索包与对应loader。

没有导入探索的`physx.add_asset`：主线保留自己的actor树、接触回调、控制器和材质组合规则。
也没有把`asset.usda`当作原生烘焙文件导入；本次使用其对应的离线几何与参数加载入口。
本次未采集烘焙命中统计，不声称这四次主线启动一定命中缓存。没有清缓存或手工改烘焙环境。
跨机器、版本、尺寸或原生烘焙准备仍由探索侧维护。

## 使用入口

同一C2有限力推动配方批量换入四资产：

```bash
python3 -m world_model_dataset.physical_asset_smoke \
  --exploration-root /home/fangsuo/isaacsim_work_flexible_explore \
  --output output/world_model_dataset/v0_2/physical_assets_development02
```

支持`--assets banana chair`、`--size`、互斥的`--mass`/`--density`、`--prepare-only`。
输出必须为新目录，不覆盖历史结果。生成的每个`recipes/<asset>.json`也可直接交给既有`causal_runner`。
几何配置中的`physical_asset_source.size`还可改为`{"extents_m": [x,y,z]}`；
XYZ缩放只做了CPU入口与完整惯量一致性检查，不宣称其原生运动可靠。
输入物理profile必须明确选择质量或密度，resolved body同时记录实际推导的两者。

这次证明的是四个资产能批量进入C2推动加载流程，不代表P01–P12的每种几何约束都已自动适配，
也不代表任意外部STL都可直接使用。源轴保留，不推断语义朝上方向；脚底/最低顶点放到地面由主线准备逻辑处理。

## 已运行结果

目录：`output/world_model_dataset/v0_2/physical_assets_development01/`。
`index.json`索引四个episode，`integration_review.json`记录直接缓存检查。

条件：最大尺寸0.2 m、密度700 kg/m³、资产摩擦0.4/0.4、恢复系数0.1；
同一个主线C2动态推板，240 Hz，2 s，外力上限8 N，位置/速度不在t0后直接改写。
四条原生进程正常退出，各481帧；统一loader已读取状态、接触、命令与施力流。

| 资产 | 原网格三角数 | 原生质量kg | 峰值线速度m/s | 峰值角速度rad/s | 推板接触记录数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| banana | 43,622 | 0.19030 | 0.850 | 16.28 | 6,614 |
| elephant | 50,000 | 2.06762 | 0.592 | 5.01 | 4,120 |
| chair | 16,954 | 0.13821 | 0.810 | 5.12 | 2,160 |
| carrot | 39,398 | 0.17367 | 1.193 | 23.29 | 5,476 |

COM原生读回均在actor原点；完整惯量相对Frobenius误差最大约1.06e-7，质量相对误差小于4e-8。
没有再减面、补洞、换凸包；初始USD明确写入完整SDF配方。
上述接触记录数是逐步逐点报告数，不是独立碰撞事件数。

四条中原生已报告接触的最深负separation约0.315/0.762/0.082/0.966 mm；
这不是对整张网格或全部时间的精确穿透认证。胡萝卜有较明显转动和侧向移动，
不能把单条可运行结果升级成全部姿态/尖端接触可靠的结论。

与探索测试的关键区别保留且不隐瞒：

- 主线contact offset为2 mm；探索资产作者在0.2 m尺寸时使用0.4 mm。
- 主线摩擦组合为min、恢复系数组合为average；地面静/动摩擦为0.3/0.2。
  因而资产声明0.4/0.4不等于与地面的实际组合摩擦也是0.4。
- 有重力、地面和有限推板，不是探索的零重力108球测试；质量随几何体积变化，
  这四条也不是严格的单变量几何反事实。

当前没有调用正式finalize/发布准入，没有生成训练观测或视频。
13项既有相关测试和6项本地资产接入测试通过；后者缺探索交付时显式skip。
这里只验证了本次参数点的主线接入；0.5 kg、其他尺寸/密度、凹形通行和动态接触量程仍待探索侧继续。
香蕉旧Cylinder挂杆未复用。

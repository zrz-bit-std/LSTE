# 面向 LSTE 的语义导航架构调研

调研日期：2026-08-23

## 结论先行

不建议用一个端到端大模型替换 LSTE。它会把本项目已有且可解释的创新点
`task -> LSTE brain -> global_goal` 藏进不可审计的网络，同时在 RTX 3060 上无法稳定承担
实时安全控制。

更合理的方向是保留 LSTE 的任务理解与全局决策权，把执行层升级成**带语义记忆的导航
执行器**：语义模型决定“值得搜索什么区域”，几何地图决定“是否可达”，Navfn/TEB 决定
“怎样安全地到达”。这比让检测框直接抢占 TEB，或持续调 TEB 参数，解决的是更上游的
架构问题。

## 需要保留的 LSTE 创新

当前项目不是普通的 ObjectNav benchmark agent。必须保留并明确以下边界：

1. **LSTE brain 是唯一的任务语义入口。** 它将任务 JSON、文本 prompt、开放词汇感知
   和上下文综合成目标语义与 `global_goal`，不是由底层导航模型自行猜测任务。
2. **`/lste/final_goal` 的最终仲裁权属于 Goal Manager。** 任何新模块只能提交候选区域、
   可达性证据和置信度，不能绕开 Goal Manager 直接控制车辆。
3. **可观测、可复现实验是系统能力。** 每个目标事务、地图路线、检测证据和执行终态都应
   留在运行日志中；不能以“模型给出了动作”替代因果证据。
4. **开放词汇目标仍由当前 WeDetect/GroundingDINO 接口提供。** 后续可以替换检测器，
   但不应将视觉类别硬编码进导航栈。

因此，外部工作只能升级 LSTE 下游的“语义记忆、候选选择和局部执行”，而不是取代大脑。

## 当前问题的架构根因

现有链路是：

```text
LSTE brain / WeDetect -> Goal Manager -> final_goal -> Navfn + TEB
                                     ^
                         frontier、检测、失败事件同时进入
```

它的主要风险不是 TEB 不够会调参，而是不同时间尺度的信号共用一个二维点：

- 检测帧是高频、带噪的“证据”；
- frontier 是低频、基于地图的“搜索建议”；
- `MoveBaseAction` 是应当原子完成的“执行事务”；
- TEB 是 20 Hz 的局部轨迹优化器。

如果证据被直接翻译为新的点目标，就会发生目标频繁改写、动作抢占、轨迹重置和零速脉冲。
已有的 target track、路由承诺、失败锁和 action lifecycle 是第一步修正：它们把证据和
执行事务分开，但还没有持久记录“哪里看过、哪里更可能有目标”。

## 候选范式

| 路线 | 核心思想 | 对 LSTE 的价值 | 不作为主架构的原因 |
| --- | --- | --- | --- |
| VLFM | 用视觉语言模型给 frontier 打分，再由几何地图导航 | 最接近未知室内物体搜索；保留显式 occupancy map 和规划器 | 原项目基于 Habitat/Spot，视觉服务较重，需要封装为 ROS 节点 |
| VLMaps | 将 RGB-D 累积成可文本检索的稠密语义地图 | 目标不在当前视野时仍保有“最后一次看见的位置” | 主要用于离线/持续建图，细小杯子受地图分辨率和 CLIP 特征限制 |
| ConceptGraphs | 持久 3D 物体实例图和开放词汇查询 | 适合未来支持“桌上的蓝杯”等关系任务 | Grounded-SAM、LLaVA、3D 合并很重，3060 上不宜进入控制闭环 |
| ViNT / NoMaD | 用视觉导航 foundation policy 输出局部动作/轨迹 | 可以作为以后对比的学习型局部执行器 | 需要与本车、相机、动力学的适配数据；仍不提供全局几何可达性保证 |
| ViPlanner | 学习视觉局部导航轨迹 | 对复杂非结构化可通行性有价值 | 当前室内 lidar + costmap 已能表达安全约束；直接替换 TEB 的收益尚未证明 |
| Uni-NaVid | 视频 VLA 直接输出导航行为 | 研究展示价值高 | 官方公开说明为单张 A100 约 5 Hz，7B 模型，不适合作为 3060 上的实时安全控制器 |

### 关键资料与开源状态

- **VLFM**: *Vision-Language Frontier Maps for Zero-Shot Semantic Navigation*, ICRA 2024,
  DOI [10.1109/ICRA57147.2024.10610712](https://doi.org/10.1109/ICRA57147.2024.10610712)。
  [MIT 代码](https://github.com/rai-opensource/vlfm) 使用深度构建 occupancy map，再用 RGB
  和预训练视觉语言模型为 frontier 赋语义价值，并已在真实 Spot 上演示。
- **VLMaps**: *Visual Language Maps for Robot Navigation*, ICRA 2023,
  [arXiv:2210.05714](https://arxiv.org/abs/2210.05714)，
  [代码](https://github.com/vlmaps/vlmaps)。
- **ConceptGraphs**: *Open-Vocabulary 3D Scene Graphs for Perception and Planning*, ICRA 2024,
  DOI [10.1109/ICRA57147.2024.10610243](https://doi.org/10.1109/ICRA57147.2024.10610243)，
  [MIT 代码](https://github.com/concept-graphs/concept-graphs)。
- **ViNT**: *A Foundation Model for Visual Navigation*,
  [arXiv:2306.14846](https://arxiv.org/abs/2306.14846)；**NoMaD**: *Goal Masked Diffusion
  Policies for Navigation and Exploration*, ICRA 2024,
  DOI [10.1109/ICRA57147.2024.10610665](https://doi.org/10.1109/ICRA57147.2024.10610665)。两者与 GNM 的官方
  [MIT 实现](https://github.com/robodhruv/visualnav-transformer) 共用代码库。
- **ViPlanner**: *Visual Semantic Imperative Learning for Local Navigation*,
  [arXiv:2310.00982](https://arxiv.org/abs/2310.00982)，
  [代码](https://github.com/leggedrobotics/viplanner)。
- **Uni-NaVid**: RSS 2025, [论文](https://arxiv.org/abs/2412.06224)，
  [代码](https://github.com/jzhzhang/Uni-NaVid)。官方 README 说明它使用 7B 模型，评测时
  单张 A100 约 5 Hz。
- **L3MVN**: *Leveraging Large Language Models for Visual Target Navigation*, IROS 2023,
  DOI [10.1109/IROS55552.2023.10342512](https://doi.org/10.1109/IROS55552.2023.10342512)，
  [代码](https://github.com/ybgdgh/L3MVN)。它说明语言先验能改善搜索，但不应取代几何安全层。

## 推荐架构：LSTE-Semantic Navigation Executive

```text
任务 / LSTE brain
        |
        | 目标合同：类别、属性、任务约束、global_goal 候选
        v
语义导航执行器  <---- WeDetect 的目标证据、RGB-D/LiDAR、SLAM map
  |  1. object belief tracks（物体置信轨迹）
  |  2. semantic frontier memory（语义前沿记忆）
  |  3. action transaction manager（执行事务）
  v
可达候选集合：目标 approach manifold / 语义 frontier / 普通 frontier
        |
        | 仅把已验证、稳定的一项提交为 /lste/final_goal
        v
Navfn + TEB -> cmd_vel mux -> Pro3
```

### 1. 目标合同，而不是不断刷新的坐标

LSTE brain 输出的不是每帧 `(x, y)`，而是一个不可变 `mission_id` 的合同：

```json
{
  "mission_id": "task-42",
  "target_query": "blue mug",
  "global_goal": "find-and-approach",
  "success_predicate": "visible_close_and_reachable"
}
```

Goal Manager 为该合同维护目标轨迹、可达 approach pose、失败次数和地图进度。视觉帧只更新
belief，不直接改写 move_base action。这延续当前 `target_track_id` 的思想，但将其提升为
明确接口。

### 2. 语义前沿记忆，而不是纯几何最近 frontier

对每个可达 frontier `f`，执行器维护由目标语义、历史观察和 LSTE 上下文形成的价值：

```text
value(f) = P(target visible after reaching f | semantic memory, LSTE context)
           - path_cost(f) - risk(f) - revisited_penalty(f)
```

这不是要求立刻训练一个新网络。第一版可复用 VLFM 的“视觉语言相似度 + frontier”思想，
将当前相机的 CLIP/检测证据投影到已建地图附近，并把最优 frontier 作为**候选**交给
Goal Manager。路径距离、可达性和碰撞安全仍由现有 Navfn/costmap 验证。

### 3. `approach manifold`，而不是把物体中心当导航点

物体中心通常不可通行：杯子在桌上、点落在墙后、或目标很近但被障碍膨胀层覆盖。应从物体
估计位置周围生成一圈满足以下约束的 approach poses：

1. 在已知自由空间；
2. Navfn 有路；
3. 朝向目标并保留观察距离；
4. 代价最低且不与当前健康 action 产生无意义抢占。

这把“看到目标”转为“选择一个可验证的观察位姿”，是解决墙后目标、检测框抖动和频繁刹车
的架构手段。

### 4. 明确事务边界

每个 action 只有 `DISPATCHED -> ACTIVE -> SUCCEEDED/FAILED/CANCELLED` 一个生命期。
语义记忆可以高频更新，但只有以下事件可以改变执行权：

- 当前路线已成功完成；
- 当前路线经地图验证失效；
- 新候选的价值显著更高且当前 action 位于安全交接点；
- 目标合同完成或被 LSTE 替换。

这保留当前 bridge 的失败锁和 route continuation，但将“为什么可以改目标”变成可审计的
决策，而非一组位置/时间阈值。

## 分阶段实施建议

### 阶段 A：先完成执行器边界

- 保持 LSTE、gmapping、Navfn 和 TEB，不替换任何控制器。
- 完成 target track failure 到 frontier recovery 的事务闭环，并确保 frontier 从机器人
  当前位姿重新规划，而不是恢复陈旧 endpoint。
- 在日志中记录 `mission_id`、track ID、候选评分、拒绝原因、路由计划和 action terminal。

这是当前正在进行的工作，风险低，也能直接解决目标闪烁、错误重试和不必要停车。

### 阶段 B：引入语义前沿候选器

- 单独新建 ROS 节点，只订阅地图、相机、LSTE 任务合同和 robot pose；
- 输出 `/lste/semantic_frontier_candidates`，绝不直接发布 `/cmd_vel` 或 `/lste/final_goal`；
- Goal Manager 对它和现有几何 frontier 做 A/B：成功率、首次发现时间、总路径长度、
  action preemption、停顿时间。

应优先复用 VLFM 的思想和接口，而不是直接复制其 Habitat 工程。

### 阶段 C：再评估学习型局部执行器

仅当日志证明 TEB 在地图正确、目标合同稳定时仍系统性不能处理某类局部可通行性，才以
ViPlanner 或 ViNT/NoMaD 做独立对照。它们应先输出候选局部轨迹，经 costmap collision check
和 mux 安全层验证；不能直接覆盖安全控制。

## 现在不做的事

- 不以调 TEB 权重、速度或阈值作为主要方案；
- 不将 7B VLA 直接放入 3060 的低层控制闭环；
- 不删除 LSTE global goal，也不让第三方框架成为最终目标发布者；
- 不把“论文代码开源”误认为“已适配 Gazebo Pro3、ROS Noetic 和本项目传感器”。

## 评价标准

新架构只有在同一世界、同一任务、同一初始位姿下同时改善以下指标，才值得取代当前几何
frontier：目标首次发现时间、任务成功率、到达路径长度、有效 action 数、preemption 比率、
零速总时长、无障碍时的制动次数，以及可解释日志的完整性。

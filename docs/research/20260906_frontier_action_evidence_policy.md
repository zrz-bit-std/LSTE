# Place 内 Frontier 的证据决策架构

日期：2026-09-06  
状态：已实现纯逻辑切片，已通过回归测试；尚未完成办公楼正式 benchmark。

## 为什么要改

当前 Portal 选择已经使用“离散动作类别 + Pareto 证据”，但普通 Place 内的
ObservationWorkItem 仍会经过一个经验加权总分：信息量、结构量、路径长度和航向
惩罚被压成一个数。改变一个权重就可能改变“先看哪个 WorkItem”，这不是物理图
上的事实，而是调参结果。

这次切片把两类决定分开：

1. 图状态先决定动作是否合法：Place 所有权、WorkItem 是否仍未完成、Portal 是否
   已认证、Navfn 路由是否已验证、视点是否已覆盖。
2. 合法动作按离散类别排序：`viewpoint_retry`、`target_direction`、`adjacent`、
   `local`、`probe`。
3. 同一类别内只保留证据向量的 Pareto 前沿，不计算加权总分。
4. 前沿中的最终选择使用稳定的 WorkItem/Portal/栅格身份作为 tie-break，输入列
   表顺序和旧 `score` 不参与选择。

因此，路径更短不能把一个已完成 WorkItem 重新变成任务；信息更多也不能绕过未认证
门洞。旧标量仍保留给 baseline 和诊断，以保证实验可比较。

## 代码边界

纯逻辑决策位于
`src/lste_topo_access/scripts/global_frontier_frontier_decision.py`，不依赖 ROS。
它定义了 `FrontierActionCandidate`、`FrontierDecisionEvidence` 和
`FrontierActionDecision`。现有路线 tuple 通过 `candidate_from_route()` 适配，
不改变 ROS 接口和 Navfn/TEB 执行接口。

`place_portal_workitem` 与 `place_portal_workitem_strict` 的方法契约声明
`frontier_action_policy=event_pareto`。普通 `frontier`、距离去重和 `place_portal`
继续使用 `legacy_scalar`，不会被新策略悄悄改变。Global frontier 只在完整方法的
本地动作边界调用新选择器；Portal 事务和 TEB 仍由原模块负责。

当前证据字段是：

| 字段 | 来源 | 作用 |
| --- | --- | --- |
| `unknown_support` | frontier unknown 邻域 | 最大化可观察未知空间 |
| `work_item_novelty` | WorkItem 支持单元数量 | 保持观察任务 lineage |
| `target_relevance` | 目标方向动作类别 | 先满足任务相关观察 |
| `clearance` | 已通过安全 endpoint 验证 | 合法性事实，不是 reward |
| `path_cost` | Navfn/route 距离 | 同类动作的代价维度 |
| `turn_cost` | 当前切片为中性事实 | 未来可由测量的 route tangent 填充 |
| `risk` | 已通过硬约束后为中性事实 | 风险不能用分数抵消非法状态 |

`candidate_score()` 没有删除，因为它是历史 baseline 的实验组成部分；事件策略
不会读取它来选动作。每次选择另外发布 `frontier_action_selected`，包含类别、
可行数、Pareto 前沿大小、稳定 candidate id 和 decision epoch，便于回放。
`summarize_run.py` 会把这些事件独立汇总为类别计数、候选/可行/Pareto 总量和
策略一致性；它们不会和 Portal 的 `graph_action` 计数混在一起。

## 外部设计依据

这不是把某一篇工作的代码直接搬进 ROS 1，而是抽取它们共同的架构边界：

- Yamauchi，*A Frontier-Based Approach for Autonomous Exploration*，CIRA 1997：
  frontier 作为未知边界 baseline。
- Chaplot 等，*Learning to Explore using Active Neural SLAM*，ICLR 2020，
  [arXiv:2004.05155](https://arxiv.org/abs/2004.05155)：分层探索状态与局部执行器
  可以分离，支持保留 Navfn/TEB 的执行边界。
- Ramakrishnan 等，*Object Goal Navigation using Goal-Oriented Semantic
  Exploration*，CVPR 2020，[arXiv:2007.00643](https://arxiv.org/abs/2007.00643)：
  语义证据用于地图候选选择，而不是直接成为速度命令。
- Shah 等，*Vision-Language Frontier Maps for Zero-Shot Semantic Navigation*，
  ICRA 2024，[DOI](https://doi.org/10.1109/ICRA57147.2024.10610712)：语义价值
  应挂在 frontier/视点上；LSTE 保留更严格的物理 Portal 认证。
- STGPlanner，[arXiv:2412.13664](https://arxiv.org/abs/2412.13664)：以拓扑分支和
  状态转换组织探索，而不是只对瞬时点做贪心排序。
- GRID-FAST，[arXiv:2406.11635](https://arxiv.org/abs/2406.11635)：把区域、开口
  和未探索通路分开，支持 LSTE 的 Place/Portal/WorkItem 类型边界。
- SAP-Nav，[arXiv:2608.12707](https://arxiv.org/abs/2608.12707) 和 AECNav，
  [arXiv:2608.10817](https://arxiv.org/abs/2608.10817)：近期工作共同强调
  evidence-gated perception 与主动视点验证；LSTE 只允许这些证据请求合法的图动作，
  不让异步 detector 直接抢占运动控制。
- R2F，*Repurposing Ray Frontiers for LLM-free Object Navigation*，
  [arXiv:2603.08475](https://arxiv.org/abs/2603.08475)，[开源代码](https://github.com/Lab-RoCoCo-Sapienza/r2f)：
  方向条件的 ray frontier 可作为 WorkItem/Portal 的未知侧语义证据；当前只保留其
  方向假设，不把其仿真栈直接搬入 ROS 1。
- Clio，*Real-time Task-Driven Open-Set 3D Scene Graphs*，
  [arXiv:2404.13696](https://arxiv.org/abs/2404.13696)，[开源代码](https://github.com/MIT-SPARK/Clio)：
  任务条件的语义区域聚类适合作为 Place 上层 overlay，但不替代物理 Place/Portal
  身份。
- UIAP-OGN，*Uncertainty-Informed Active Perception for Open Vocabulary Object
  Goal Navigation*，[arXiv:2506.13367](https://arxiv.org/abs/2506.13367)，[代码](https://github.com/PRBonn/uiap-ogn)：
  可用于未来的语义 posterior/主动视角消歧；其现有实现仍含硬编码 UCB 参数，暂不
  直接接入，避免把一个新 scalar 再塞回当前系统。

上述工作提供设计启发，不构成 LSTE 已达到其论文指标的证据。

## 可证伪预测与实验臂

在相同 world、seed、SLAM、WeDetect、Navfn、TEB 和任务下，比较：

1. `frontier`：普通 frontier + 标量 baseline；
2. `frontier_distance_dedup`：距离去重 + 标量 baseline；
3. `place_portal`：Place/Portal + 标量 local selector；
4. `place_portal_workitem_legacy_rank`：Place/Portal/WorkItem + 旧 scalar，隔离
   selector 改造；
5. `place_portal_workitem_strict`：Place/Portal/WorkItem + event-Pareto，关闭
   branch-first；
6. `place_portal_workitem`：event-Pareto + branch-first 主方法。

主要假设是：事件决策会降低已完成 WorkItem 的再次派发和由权重变化造成的局部
来回切换，同时不破坏 Portal 认证和 covered transit。必须由自动化日志验证：

- 重复进入次数；
- 单 Place 有效 WorkItem 时间；
- resolved WorkItem 重复派发；
- Portal crossing 与未认证跳转；
- 覆盖率、语义目标成功率、路径长度、完成时间；
- 碰撞/失败率、停顿次数、角速度能量和方向反转。

当前 `src/lste_topo_access/test/test_frontier_action_decision.py` 覆盖类别优先、
Pareto 不可比较候选、稳定身份、非法 Portal、已完成 WorkItem、旧 score 不可影响
选择和方法契约。480 个现有 `lste_topo_access` 测试已通过。它们只证明决策不变量，
不证明 Level 4 已完成；
正式矩阵完成前不宣称方法优于 baseline。

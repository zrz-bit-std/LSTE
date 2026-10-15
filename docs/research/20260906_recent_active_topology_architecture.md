# 近期未知环境语义拓扑探索调研与 LSTE 架构建议

日期：2026-09-06  
状态：研究输入与下一步实现建议；不是 Level 4 完成结论。

## 1. 调研边界与证据规则

本次问题限定为：机器人没有预先完整地图，在办公楼这类有房间、走廊和门洞的环境中，寻找自然语言指定的物体；系统已有 2D SLAM、2D LiDAR、RGB 检测、Navfn 和 TEB。研究重点是减少重复进入和无意义停留，而不是重新训练一个控制器。

论文页面和摘要通过 arXiv API/HTML 在 2026-09-06 核验，代码状态通过对应 GitHub 仓库页面核验。arXiv 预印本的结果不能直接当作 LSTE 的实验结论。论文声称“代码将公开”但当前没有公开仓库的，标记为“不可直接依赖”。

## 2. 近期方法对照

| 方法 | 时间与来源 | 核心架构 | 代码/接入事实 | 对 LSTE 的启发 |
| --- | --- | --- | --- | --- |
| STGPlanner | ICRA 2025， [arXiv:2412.13664](https://arxiv.org/abs/2412.13664) | 用 skeleton 和拓扑分支表达复杂未知环境，减少在几何网格上反复选相近点 | [官方 ROS Noetic 代码](https://github.com/Haochen-Niu/STGPlanner) | 可作为未知办公楼拓扑探索基线；LSTE 进一步把门洞认证和物理 Place 身份写成事务 |
| CORE Planner | 2026-06， [arXiv:2606.29222](https://arxiv.org/abs/2606.29222) | 稀疏 visibility graph + Transformer contextual memory，使用历史上下文缓解局部最优 | [公开仓库](https://github.com/BBD00/core_planner) 当前明确是 PyTorch/Ray 训练代码，ROS 部署尚未发布 | 支持“稀疏持久结构 + 记忆”方向；不直接替换 ROS1 图规划器，也不把训练超参数伪装成架构创新 |
| FPAS | IROS 2026， [arXiv:2606.22838](https://arxiv.org/abs/2606.22838) | 按开放度自适应稀疏采样全局图，在窄通道保留连接性，并为死路保留回溯节点 | 论文页未发现作者运行代码 | 可作为未来几何图压缩消融；当前优先解决物理身份与证据闭环，不先引入采样阈值 |
| UNSEEN | 2026-06， [arXiv:2606.20755](https://arxiv.org/abs/2606.20755) | 将定位、稀疏建图和规划的不确定性统一到 receding-horizon 决策 | 论文描述为视觉-only、6 Hz 的独立系统，未找到可直接复用的 ROS1 实现 | 提醒我们把 SLAM/感知不确定性作为证据来源，而不是让瞬时地图标签直接改变长期 Place 身份 |
| UIAP-OGN | ECMR 2025， [arXiv:2506.13367](https://arxiv.org/abs/2506.13367) | 语义不确定性传感器模型、概率几何-语义地图和不确定性多臂 bandit | [MIT 代码](https://github.com/PRBonn/uiap-ogn)，依赖 GroundingDINO、MobileSAM、VLM/Habitat | “提示词不同导致语义不确定”应进入证据状态；不直接搬其 Habitat 和模型栈 |
| Active Semantic Perception | 2025， [arXiv:2510.05430](https://arxiv.org/abs/2510.05430) | 多层 scene graph（房间、物体、墙、窗），用 LLM 生成未观测区域假设，再按 waypoint 信息增益主动采样 | [公开代码](https://github.com/grasp-lyrl/active_semantic_perception)，ROS1，但需要先验 2D map、RGB-D、nvblox/Clio，仓库没有明确 license 元数据 | 最接近“语义图反过来决定观察”的实现参考；不能直接满足当前的无先验未知地图约束 |
| B-ActiveSEAL | 2025， [arXiv:2512.12194](https://arxiv.org/abs/2512.12194) | 将定位不确定性和地图不确定性耦合，使用信息论主动探索 | 当前未找到作者公开代码 | 可作为主动 SLAM 的理论参考；替换现有 SLAM 代价过大，不作为本阶段依赖 |
| R2F | 2026-03， [arXiv:2603.08475](https://arxiv.org/abs/2603.08475) | 把超出传感器范围的语义 ray 按方向存到 frontier region，语义证据持续积累，frontier 地图低频更新 | [官方 MIT 代码](https://github.com/Lab-RoCoCo-Sapienza/r2f)，Python 3.9/Habitat 0.3，RADIO + SigLIP，非 ROS | 说明“未知边界 + 方向”比单个瞬时 frontier 点稳定；当前 WorkItem 的 unknown-side normal 是轻量 2D 版本 |
| SCOUT | 2026-06， [arXiv:2606.06721](https://arxiv.org/abs/2606.06721) | PSGG 维护概率语义 scene graph，UGT 根据语义确定性、几何覆盖和代价选择下一视点 | 论文页未给出公开运行仓库；假设先验 2D occupancy map 和 RGB-D，实验使用经验阈值 | “语义地图必须反过来驱动采集”是正确方向；当前阶段只吸收证据闭环，不复制其阈值和 3D 管线 |
| OVIP-SG | 2026-08， [arXiv:2608.17633](https://arxiv.org/abs/2608.17633) | VLM 枚举场景词汇，LocateAnything + SAM2/CLIP 建立实例保持的 3D 图，并按功能划分搜索区域 | [公开代码](https://github.com/Agibot-Spatial-Intelligence/OVIP-SG)，仓库 API 未声明 license；依赖较重 | 对小物体、相似物体和实例合并很有参考价值；不能替代当前 2D Portal/TEB 执行层 |
| Concept-Guided Exploration | 2026-08， [arXiv:2608.23650](https://arxiv.org/abs/2608.23650) | Room 和 door 是异步 concept agents；房间概念提供几何/语义约束，门概念在墙边界中增量验证 | 当前未找到公开作者代码 | 直接支持“Place 和 Portal 是不同证据代理”的架构；LSTE 可用 ROS-free reducer 实现，而不引入分布式运行时 |
| AECNav | 2026-08， [arXiv:2608.10817](https://arxiv.org/abs/2608.10817) | evidence-gated perception、cluster-level log-odds、正/负证据和 active evidence acquisition | 摘要写明接收后公开代码，当前未找到公开仓库 | 是本阶段最有用的证据生命周期参考：检测缺失不应立刻抹掉目标，负证据也应有明确适用范围 |
| SAP-Nav | 2026-08， [arXiv:2608.12707](https://arxiv.org/abs/2608.12707) | Queryable Spatial-Semantic Representation + Active Viewpoint Verification；证据不足时主动换视角 | 摘要写明代码将公开，当前没有可依赖的公开仓库 | 目标重检应是独立动作，而不是普通 frontier 的分数加成；现有 `reinspect_target` 正是这一边界 |
| SSTG-Nav | 2026-08， [arXiv:2608.00527](https://arxiv.org/abs/2608.00527) | 可复用 metric-semantic topology，保留 source-aware 融合和多个可恢复 standoff | [公开 benchmark/评测代码](https://github.com/DaojiePENG/sstg-nav-bench)；README 明确完整机器人部署栈不公开 | 强调对象位置必须落到可达停靠点，并保留多个恢复视点；LSTE 的 WorkItem Attempt/route lease 可对应这一思想 |
| STEGNav | 2026-08， [arXiv:2608.28279](https://arxiv.org/abs/2608.28279) | 空间图同时放 target 和 occupancy-aware frontier，时间轴保留近期决策轨迹与已验证跨子任务结果 | 当前未找到公开作者代码 | 说明事件图不应只存当前状态，还要能回放“哪次证据导致动作”；现有 `EvidenceEventGraph` 是轻量起点 |
| CGFM-Nav | 2026-08， [arXiv:2608.29114](https://arxiv.org/abs/2608.29114) | 显式多模态关系图与连续 semantic-frontier field 联合，用图证据指导未匹配目标搜索 | 当前未找到公开作者代码 | 可作为未来“图节点 + 语义场”的扩展参考；不应先引入大模型或连续场，先验证图动作不变量 |
| RTNav | 2026-08， [arXiv:2608.26496](https://arxiv.org/abs/2608.26496) | 把推理延迟、异步环境推进和有限算力作为系统状态，而不是假定推理免费 | 当前未找到公开作者代码 | 支持把检测缓存、事件唤醒和 TEB 执行解耦；当前 3060 不应在每个控制周期重复运行高延迟语义推理 |
| vS-Graphs | IEEE RA-L 2026， [代码](https://github.com/snt-arg/visual_sgraphs) | 在 ORB-SLAM3 上加入墙、地面、房间、走廊等结构元素并优化 3D scene graph | 公开 ROS2 Jazzy 代码，需 RGB-D/视觉 SLAM，和当前 ROS1 2D 栈不同 | 可借鉴结构语义与 SLAM 解耦；不直接移植，作为未来定位/结构图升级候选 |
| HitMem | 2026-09， [arXiv:2609.00950](https://arxiv.org/abs/2609.00950) | 分层时间 3D memory，检测到物体位移时触发两阶段检索 | 当前未找到公开作者代码；论文刚提交 | 对动态办公环境有价值，但当前 benchmark 是静态 Gazebo，暂不把时间衰减加到主方法 |

## 3. 现有 LSTE 已覆盖与遗漏

现有研究文档已经覆盖 STGPlanner、GRID-FAST、UIAP-OGN、R2F、SAP-Nav、AECNav、SCOUT、OSG Navigator、UniGoal、Portal/WorkItem、event graph 和 route lease。它们已经形成了“长期拓扑身份、短期几何投影、控制器 lease”这条主线。

本次通过 arXiv API 和公开仓库页面追加核验了四个方向：CORE Planner
（[arXiv:2606.29222](https://arxiv.org/abs/2606.29222)）的仓库目前明确是
PyTorch/Ray 训练代码，ROS 部署尚未发布；FPAS
（[arXiv:2606.22838](https://arxiv.org/abs/2606.22838)）没有可直接复用的作者实现；
UNSEEN（[arXiv:2606.20755](https://arxiv.org/abs/2606.20755)）的视觉-only 传感器
假设不同于当前 2D LiDAR/SLAM 栈；FSD-VLN
（[arXiv:2607.08359](https://arxiv.org/abs/2607.08359)）是面向航空器的 fast-slow
DiT/VLN 控制架构。它们共同支持“慢语义/结构记忆与快执行解耦”，但没有一个要求
我们再增加一组速度、距离或置信度参数。

本次补齐了四类以前没有被集中说明的内容：

1. `Active Semantic Perception` 有公开 ROS1 代码，但依赖先验地图和 RGB-D，不能被误写成当前未知办公楼的直接替代方案；
2. `OVIP-SG` 公开了针对小物体和实例保持的实现，但 license 和部署负担需要单独审查；
3. `Concept-Guided Exploration` 给出了“异步 room/door 概念代理”的近期架构依据，支持把 Place 和 Portal 的证据职责分开；
4. 现有 `20260906_event_driven_evidence_graph.md` 的 STEGNav 链接已经修正为 `2608.28279`；之前的 `2608.28027` 实际不是 STEGNav。

还要如实记录一个实现缺口：当前 `EvidenceEventGraph.consume_wake()` 已经存在，但没有被规划 runtime 真正消费；它目前主要作为 status 的可回放投影。因此文档中“fast/slow”是已实现的状态边界和调度接口，不应宣传成已经完成的纯事件驱动运行时。

## 4. 建议的一个可落地创新点

### 4.1 Evidence-Contract Active Graph

建议将下一阶段方法命名为 **Evidence-Contract Active Graph（ECAG）**。它不是再加一个 frontier 权重，而是让每个图对象声明“还缺什么证据”，让动作声明“会产生什么证据”。

```text
Place P
  outstanding evidence: first_view | local_work(W) | target_verify

Portal Q
  outstanding evidence: source_view | crossing | destination_view

WorkItem W
  outstanding evidence: viewpoint_observation
  attempts: active | failed | succeeded

Graph action
  preconditions: durable identities + physical proof
  produces: one explicit evidence fact
```

规划顺序是有限状态和证据集合运算，而不是加权和：

1. 当前 Place 若有目标重检证据缺口，只能选择同一 Place 的 `reinspect_target`；
2. 没有目标重检时，当前 Place 的未解决 WorkItem 先获得合法局部视点；
3. 当前 Place 没有必须完成的局部证据时，选择未完成的 Portal source probe；
4. 已认证的 Portal 只能把机器人带到图中仍有证据缺口的 Place；已完成 Place 只能作为 transit；
5. 没有满足物理证据前置条件时返回 `hold`，而不是把任意 frontier 当作替代动作。

在同一个动作类别内，继续使用当前的 Navfn/costmap 可行性和确定性 Pareto 选择；动作类别之间不使用新的权重。TEB、Navfn、detector 仍然是执行和感知模块，不被图层替换。

### 4.2 为什么这具有方法特色

相关工作分别解决了其中一部分问题：AECNav 解决证据整合，SAP-Nav 解决主动验证，Concept-Guided Exploration 解决 room/door 概念约束，STEGNav 解决事件图记忆，SSTG-Nav 解决可达 standoff 记忆，R2F 解决方向条件的未知边界。ECAG 的项目特色是把它们压缩成一个 ROS1 可验证的动作契约：

- 语义证据可以要求观察，但不能创建 Place；
- frontier 可以提供几何 viewpoint，但不能自己宣称跨过 Portal；
- Portal 只能凭 source/destination 的物理证据改变 Place 图；
- route failure 只结束一个 Attempt，不结束 WorkItem；
- action terminal 不能代替 observation evidence；
- 一个 active route 由 durable action lease 所有，新的语义请求必须显式 preempt 或 defer。

这正好针对当前“反复进入重复房间、在同一房间停留太久、目标漏检后错误退出”的共同根因：动作的语义不再由暂时的 SLAM label 或一次 detector frame 决定。

### 4.3 证据事件例子

```text
route_selected(W7, P2)
    -> produces: viewpoint_attempt_started(W7, A3)

route_terminal(A3, success)
    -> does not resolve W7

frontier_endpoint_observed(W7, P2)
    -> resolves W7

portal_probe_started(Q4, P2)
    -> activates source_view(Q4)

portal_crossing_verified(Q4)
    -> activates destination_view(Q4)

destination_place_committed(Q4, P3)
    -> creates/enters P3; Q4 becomes crossed transit
```

其中 detector 的“没看到”只有在当前视点确实覆盖目标预期区域时才产生负证据；普通 detector 空帧不能删除 target WorkItem。这继承 AECNav 的证据思想，但保留当前 Goal Manager 的物理视点和任务版本边界。

## 5. 推荐代码边界

第一切片已经实现为一个 ROS-free 模块，不重写当前大节点：

```text
src/lste_topo_access/scripts/global_frontier_evidence_contract.py
```

建议接口：

```python
EvidenceRequirement(owner_kind, owner_id, kind, state)
EvidenceObservation(event, owner_kind, owner_id, evidence_kind, route_id, viewpoint_id)
EvidenceContractSnapshot(requirements, resolved, blocked)

apply_observation(snapshot, observation) -> snapshot
requirements_for(place, portal, work_item, target) -> tuple
action_for_gap(snapshot) -> GraphAction
```

实现边界如下：

- `global_frontier_graph_route_planner.py` 只读取未完成 requirement，继续返回离散图动作；
- `global_frontier_graph_route_gate.py` 验证 candidate 是否带有对应 Place/Portal/WorkItem 的证明；
- `global_frontier_graph_route_transaction.py` 在 commit 时写入 `produces_evidence`，不扩大 legacy route tuple 的几何职责；
- `global_frontier_event_graph.py` 将结构事件归约成 contract revision；
- `global_frontier_planning_runtime.py` 只在 idle 或 terminal 后消费 structural wake，active route 期间仍由现有 watchdog 观察物理进度；
- `global_frontier_portal_probe_ledger.py`、`global_frontier_target_observation_work.py` 和 `global_frontier_work_items.py` 继续拥有各自的生命周期，不合并成一个含糊的状态字段。

当前代码状态：`EvidenceRequirement`、`EvidenceObservation`、
`EvidenceContractSnapshot` 和 `EvidenceAction` 已在
`global_frontier_evidence_contract.py` 中实现；`GraphRoutePlan` 会自动携带
`required_evidence`/`produces_evidence`；`EvidenceEventGraph` 会归约显式证据事件并
在 status 中输出 `evidence_contract`。这只是契约和可回放状态切片，尚未把慢规划器
改成完全事件驱动，也没有因此宣称 Level 4 任务完成。

这一步不需要 MiniCPM、LLM、额外训练，也不需要改变 TEB 参数。已有的检测缓存和任务 JSON 仍可作为感知输入；ECAG 只决定下一项必须获得的证据。

## 6. 回归测试与已验证不变量

纯逻辑测试已经接入；当前需要持续保持以下不变量：

1. 同一个 `Place/Portal/WorkItem` 输入乱序事件，contract snapshot 和 action signature 必须相同；
2. `target_verify(P2)` 未完成时，任何 Portal probe/crossing candidate 都必须被 gate 拒绝；
3. 普通 route terminal 成功不能直接 resolve WorkItem，只有 `frontier_endpoint_observed` 才能 resolve；
4. route failure 只能结束 Attempt，下一次仍然产生同一个 WorkItem 的新 viewpoint requirement；
5. `source_view -> crossing -> destination_view` 的 Portal 三阶段中，不能提前创建 destination Place；
6. 已完成 Place 即使出现新 transient frontier，也不能产生新的 local WorkItem，只能作为 transit；
7. 同一结构事件重复发布不应产生第二次 wake；`consume_wake` 后图事实仍保留；
8. graph plan 为 `blocked` 时，selection 不得回退到普通 frontier；证据 revision 变化后才允许重新尝试；
9. target reinspection 的 legacy route 必须携带 `reinspect_target` 语义动作，同时保持同一 WorkItem viewpoint 几何；
10. 集成 fixture 覆盖“目标重检 pending + local WorkItem + Portal probe”时，最终选择必须是当前 Place 的 local viewpoint，并提交为 `reinspect_target`。

新增 `test_evidence_contract.py` 已覆盖：Portal probe 阶段优先级、pending probe 不饿死
local WorkItem、重复 observation 幂等、未知事件无效、证据事件乱序回放一致、route terminal
不等于 observation proof，以及图计划对证据契约的 JSON 投影。当前
`src/lste_topo_access/test` 全量为 607 项通过；RL 固定目标测试因当前 Python 环境缺少
`torch` 无法收集，这是环境依赖问题，与本切片无关。

## 7. 实验与论文证据

ECAG 应作为完整方法的新实验臂，与以下设置在同一 world、seed、SLAM、detector、Navfn 和 TEB 配置下比较：

- 普通 frontier；
- 距离去重 frontier；
- Place/Portal/WorkItem strict；
- Place/Portal/WorkItem + branch-first；
- Place/Portal/WorkItem + ECAG（主方法）。

除已有指标外，建议增加四个结构指标：

- `evidence_gap_reopens`：同一证据缺口被重新打开的次数；
- `proofless_candidate_rejections`：缺少 Place/Portal/WorkItem 证明而被拒绝的候选数；
- `action_class_switches`：route 尚未完成时动作类别被改变的次数；
- `structural_wake_count` 与 `reactive_tick_count`：衡量事件驱动层是否真的减少无效慢规划。

原有重复进入次数、单房间有效探索时间、WorkItem 重派发、覆盖率、语义目标成功率、路径长度、完成时间、碰撞/失败率和运动平顺性仍是主指标。所有结论必须来自自动化日志、配置、summary、拓扑验证和可重复视频；纯逻辑测试或一轮演示不能证明方法完成。

## 8. 实施顺序

1. 将已实现的 `EvidenceContract` 继续接入选择器的提交 gate，不改变控制器；
2. 把现有 `reinspect_target`、Portal probe 和 WorkItem Attempt 映射到 requirement，不增加新的 numeric knob；
3. 将 event graph 的 structural revision 接到 idle/terminal 的慢规划唤醒点，保留 active-route watchdog；
4. 在 Level 2/3 做拓扑 smoke，再在 Level 4 做完整矩阵；
5. 只有当 baseline、ablation 和主方法都有完整证据后，才写性能结论和论文初稿。

当前最重要的研究风险不是 TEB 参数，而是 contract 是否能在真实 SLAM 更新、异步 ROS 回调和 Portal arrival 延迟下保持单一动作所有权。这个风险应由回归测试和带时间戳事件日志回答，而不是靠更多阈值掩盖。

## References

- [STGPlanner](https://arxiv.org/abs/2412.13664) and [official code](https://github.com/Haochen-Niu/STGPlanner)
- [UIAP-OGN](https://arxiv.org/abs/2506.13367) and [official code](https://github.com/PRBonn/uiap-ogn)
- [Active Semantic Perception](https://arxiv.org/abs/2510.05430) and [official code](https://github.com/grasp-lyrl/active_semantic_perception)
- [B-ActiveSEAL](https://arxiv.org/abs/2512.12194)
- [R2F](https://arxiv.org/abs/2603.08475) and [official code](https://github.com/Lab-RoCoCo-Sapienza/r2f)
- [SCOUT](https://arxiv.org/abs/2606.06721)
- [OVIP-SG](https://arxiv.org/abs/2608.17633) and [official code](https://github.com/Agibot-Spatial-Intelligence/OVIP-SG)
- [Concept-Guided Exploration](https://arxiv.org/abs/2608.23650)
- [AECNav](https://arxiv.org/abs/2608.10817)
- [SAP-Nav](https://arxiv.org/abs/2608.12707)
- [SSTG-Nav](https://arxiv.org/abs/2608.00527) and [public benchmark](https://github.com/DaojiePENG/sstg-nav-bench)
- [STEGNav](https://arxiv.org/abs/2608.28279)
- [CGFM-Nav](https://arxiv.org/abs/2608.29114)
- [RTNav](https://arxiv.org/abs/2608.26496)
- [vS-Graphs and official code](https://github.com/snt-arg/visual_sgraphs)
- [HitMem](https://arxiv.org/abs/2609.00950)

# 持久图级路线规划器

更新时间：2026-09-06

状态：已实现的研究切片，尚未代表 Level 4 任务已经完成。

## 1. 要解决的结构性问题

原来的规划器每个周期都从当前 SLAM occupancy grid 的 frontier 重新开始。Place、Portal、WorkItem 虽然已经被记录，但主要被当作候选准入条件使用，没有一个接口回答：当前物理 Place 到哪个仍有义务的 Place，应该沿哪些已经认证的 Portal 前进？

这会导致 SLAM 更新后瞬时 frontier 变化，规划器重新选择相近房间边界；当目标义务在远处时，当前 Place 没有可用 frontier 也只能反复等待、回退，甚至把图误判为探索完成。这是长期拓扑记忆没有进入动作决策的问题，不是 TEB 参数问题。

## 2. 分层边界

```text
持久 Place/Portal/WorkItem 图
        |
        |  GraphRoutePlan：完整拓扑路径，只决定离散动作
        v
当前 SLAM 快照的几何适配器
        |
        |  当前地图中的第一条 Portal endpoint
        v
Navfn / TEB
```

`GraphRoutePlanner` 位于第一层。它不读取 occupancy array，不计算距离，不读取 detector confidence，也不修改 TEB 或 Navfn 参数。它只处理物理身份和证据状态。

第二层仍使用现有的 Portal selection、durable egress、Navfn reachability 和 route transaction。SLAM 改变时，只重新投影当前第一条边，不重新发明整条拓扑任务。

## 3. 图模型

### Place

Place 是一次物理空间身份，不是某一帧 SLAM component label。已观察的 Place 可以作为 transit，但不能自动重新成为探索目标。

### Portal

只有状态为 `crossed` 且具有 `source_place_id`、`destination_place_id` 的 Portal 才能作为已知 transit edge。跨过后可以沿两个方向 transit，因为反向运动是同一个物理门洞的合法离开。

`certified` 或 `selected` 但没有 destination 的 Portal 仍然是有效信息义务，但不能假造一个目标 Place。它在计划中表现为 `unbound_portal`，交给现有 Portal 几何层继续取得目的地证据。`failed` Portal 不进入普通图搜索。

### WorkItem 与 Portal probe

WorkItem 属于发现它的 Place。只有 `unresolved` 且没有活动 Attempt 的 WorkItem 才能成为当前可执行的 local observation action。与 Portal probe 绑定的 WorkItem 不会被重复当作普通 local work。

`pending` 和 `source_arrived` probe 是可以继续获取证据的义务；`active`、`destination_active` 仍是未完成义务，但不能被第二次并发派发。

## 4. GraphRoutePlan 契约

模块：`src/lste_topo_access/scripts/global_frontier_graph_route_planner.py`

```python
plan = planner.plan(
    current_place_id,
    places=region_memory.regions,
    portals=portal_hypothesis_ledger,
    work_items=place_work_items,
    probes=portal_probe_ledger,
    branch_first=True,
)
```

返回的不可变记录包含：

| 字段 | 含义 |
| --- | --- |
| `status` | `ready`、`complete` 或 `blocked` |
| `action` | `bootstrap_observation`、`observe_local_work`、`retry_viewpoint`、`probe_portal`、`cross_portal` 或 `hold` |
| `target_place_id` | 当前义务所在的 Place；未绑定门洞为 `None` |
| `obligation_kind` | `work_item`、`portal_probe`、`unbound_portal`、`unobserved_place` |
| `obligation_id` | 对应 WorkItem、probe 或 Portal 身份 |
| `portal_path` | 到目标义务的完整 Portal ID 序列 |
| `first_portal_id` | 本周期允许物化的第一条 Portal |
| `reason` | 可回放的离散决策原因 |

例如 `A -P1-> B -P2-> C` 且 C 有未完成 WorkItem 时，计划是：

```text
status=ready
action=cross_portal
target_place_id=C
portal_path=(P1, P2)
first_portal_id=P1
```

运行时只物化 P1。P2 不会提前发给控制器，等物理 crossing 和 destination Place commit 后重新规划。这样每一条图边都有独立物理证据，不会让一个长目标跨越多个未经确认的房间。

## 5. 不依赖调参的决策规则

1. 当前 Place 未完成第一次观察时，只允许 local observation 或 source-side probe；
2. `branch_first` 可以在当前 Place 已观察后优先前往未观察 Place，但不会跳过当前 Place 的 target WorkItem；
3. 没有本地义务时，通过已经 crossed 的 Portal 图搜索远端未观察 Place、未完成 WorkItem 或未完成 probe；
4. BFS 首先最小化 Portal hop 数，再按目标类别和 `(portal_id, place_id)` 稳定排序；
5. 已完成 Place 只能出现在到达未完成义务的 transit path 中；
6. 没有认证路径但仍有义务时返回 `blocked`，绝不返回 `complete`；
7. 所有输入都按身份排序处理，SLAM 消息或 ledger 插入顺序不会改变结果。

这里没有新增速度、距离、置信度、计时器或权重参数。距离和碰撞可行性仍由几何层、Navfn 和 TEB 负责。

## 5.1 持久图动作租约

前一轮运行暴露了一个新的架构问题：同一个物理 Place 中，某个 durable
probe 可能已经被图规划器选中，但它的当前 SLAM 投影暂时没有可执行的
frontier。若下一轮又用 `visible_*_ids` 重新选择，规划器会在这个 probe 和
另一个 local WorkItem 之间来回切换。两个候选都没有完成，系统却失去了
动作连续性。

现在 `GlobalFrontierGraphRouteAdapterMixin` 为 `ready` 计划维护一个短期
图动作租约：

```text
ready durable plan
    -> 当前快照无法物化
    -> lease(plan signature)
    -> 后续快照只重新投影同一个 identity
    -> candidate commit / route terminal / Place transition 时释放
```

这个租约不是时间阈值，也不是把失败次数调大。它表达的是所有权事实：
“当前动作尚未完成，因此不能被另一个瞬时可见候选抢走”。如果物理 Place
已经改变，租约会被识别为 stale 并重新规划；如果候选成功提交，租约也会
在同一个动作事务中释放。`graph_route_candidate_unavailable` 和
`graph_route_materialization_wait` 只在租约签名变化时记录，避免把同一个
等待状态误写成高频新决策。

这条边界应作为实验指标单独记录：租约保持时间、租约期间的 map epoch
数量、同一 durable identity 的最终提交率，以及租约期间是否发生不同
obligation 的 route dispatch。它可以直接检验“减少路线抖动”是否来自
状态表示，而不是来自控制器调参。

## 6. ROS 适配

模块：`global_frontier_graph_route_adapter.py`

适配器每次慢速规划周期：

1. 从现有 ledger 取只读快照；
2. 发布一次 `graph_route_plan_selected` 事件，记录完整 `portal_path` 和 `first_portal_id`；
3. 对当前边设置短期 identity preference；
4. 正向边交给现有 Portal selector，反向边交给 durable egress；
5. 只有当前地图、costmap 和 Navfn 都能物化 endpoint 后，才发布 `graph_route_edge_materialized`；
6. 仍返回原有 route tuple，不改变 ROS topic、move_base action 或 TEB 接口。

ROS 入口没有增加第二个控制器，也没有把图规划逻辑塞进 TEB bridge。GraphRoutePlanner 是慢速决策层，TEB 仍然是唯一运动控制器。

## 7. 回归测试

新增：

- `src/lste_topo_access/test/test_graph_route_planner.py`
- `src/lste_topo_access/test/test_graph_route_adapter.py`

覆盖多跳路径、failed edge、未绑定 Portal、branch-first 与 strict 差异、covered transit、无路径时的 `blocked`/`complete` 区分、环路终止、输入顺序稳定、规划器无副作用，以及适配器只保留第一条 Portal identity。

验证命令：

```bash
python3 -m pytest -q src/lste_topo_access/test/test_graph_route_planner.py
python3 -m pytest -q src/lste_topo_access/test/test_graph_route_adapter.py
python3 -m pytest -q src/lste_topo_access/test
```

此前切片记录为 `568 passed`；加入动作租约和目标事务回归后，当前
`lste_topo_access` 测试结果为 `597 passed`。这证明状态边界和输入输出
契约，不等于证明 Level 4 端到端任务已经成功。

## 8. 研究假设与下一步实验

在相同 world、seed、SLAM、detector、Navfn 和 TEB 配置下，图级规划应该减少当前 Place 没有 frontier 时的无效等待，减少已完成 Place 的重复进入，提高 transit 到远端未完成义务的成功率，并保持 resolved WorkItem redispatch 和 phantom Place 为零。

这些是可证伪假设，不是当前结论。下一步需要在 Level 2、3、4 上运行普通 frontier、距离去重 frontier、Place/Portal/WorkItem strict，以及完整的 graph route planner + branch-first。每组都要保存时间戳日志、summary、topology verifier 和视频，并报告重复进入、有效观察时间、WorkItem 重派发、Portal crossing、覆盖率、语义目标成功率、路径长度、完成时间、碰撞/失败率和运动平顺性。

## 9. 两阶段动作事务

图计划和地图候选不是两个可以互相覆盖的 planner，而是同一个动作的
prepare/materialize/commit 三个阶段。`global_frontier_graph_route_transaction.py`
保存这个边界：

```text
GraphRoutePlan(intent)
    -> 当前地图中的 WorkItem/Portal candidate
    -> GraphRoutePlan(committed) + route_selected
```

当前投影若只重新发现了同一 Place 的另一个 unresolved WorkItem，这是同一
类观察动作的身份细化，事务会显式更新 `obligation_id`；失败 viewpoint 只
能细化为同一 WorkItem 的 `retry_viewpoint`。从 local observation 升级到
`probe_portal` 或 `cross_portal` 必须携带门洞、跨 Place 和 Portal 身份等
证据，反向替换一律拒绝。适配器分别记录
`graph_route_plan_reconciled`、`graph_route_action_committed` 和
`graph_route_plan_mismatch`，所以实验回放能判断动作是否经过图层提交。

这项改造没有增加调参项，也不改变 Navfn/TEB 的控制接口；它把原来日志中
“计划是 local、执行却是 cross”的隐式替换变成可验证的状态转移。

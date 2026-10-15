# 目标视点选项账本：把局部控制失败与语义目标解耦

> 日期：2026-09-07  
> 状态：已实现第一版接口和回归测试；尚未完成 Gazebo benchmark，不能据此宣称导航性能已经提升。

## 1. 要解决的架构问题

原来的目标跟踪链路把下面三件事压在同一个失败状态里：

1. 目标身份：视觉模块认为某个 `target_track_id` 仍然代表任务目标；
2. 视点路线：从当前机器人位置到某个观察位置的 Navfn 路线是否可用；
3. 控制执行：TEB 是否真的走完了这一个短路线。

当 TEB 在一个视点前卡住时，旧路径直接发布 `target_route_failed`，
`TargetApproachTransaction` 变成 `failed`，并把控制权释放回全局 frontier。这样一个
局部动作失败就被错误解释成“目标不可达/目标任务失败”。这会造成：

```text
candidate_0 controller_failed
        |
        +--> target transaction failed
                 |
                 +--> release target room and return to frontier
```

在有桌子、墙角、窄门和视觉遮挡的办公楼里，这个错误尤其明显：目标可能仍在同一
个 Place 中，只是当前观察方向被障碍物挡住。

## 2. 方法边界：Target-owned Viewpoint Options

本次实现引入一个 ROS-free 的
`TargetViewpointAttemptLedger`。它不是新的控制器，也不通过增加速度、角速度或
超时参数修补 TEB；它改变的是状态表示和失败传播边界：

```text
TaskGoal / target_track_id
            |
            v
TargetApproachTransaction
            |
            v
TargetViewpointAttemptLedger
   + candidate_0: requested -> Navfn endpoint -> TEB attempt
   + candidate_1: requested -> Navfn endpoint -> TEB attempt
   + candidate_2: requested -> Navfn endpoint -> TEB attempt
            |
            v
       Navfn -> TEB
```

一个候选视点同时保留：

- `candidate_id`：由目标身份、目标生命周期 epoch 和视点梯级 index 构成；
- `requested_goal`：视觉几何希望到达的点；
- `validated_goal`：Navfn 真实返回并证明可达的 endpoint；
- `route_plan`：本次验证的计划证据；
- `map_epoch`：可选的地图快照身份；
- `attempt_id`：一次实际控制器租约的唯一身份；
- `failure_reason` 和历史事件。

因此 Navfn 的容差端点不再覆盖视觉目标点，TEB 的失败也不再覆盖语义目标身份。

## 3. 状态机和不变量

视点状态是：

| 状态 | 含义 | 能否再次 dispatch |
| --- | --- | --- |
| `route_pending` | 等待 TF、Navfn 或地图证据 | 可以，在规划证据到达后 |
| `route_rejected` | 当前地图快照没有证明路线 | 不能在同一快照盲重试 |
| `route_ready` | 已有请求点和 Navfn endpoint | 可以 |
| `active` | 已租给一个 TEB attempt | 不能并发抢占 |
| `arrived` | 到达视点，等待新的观察证据 | 不能重复进入同一视点 |
| `observed` | 该视点的观察边界已完成 | 不能 |
| `controller_failed` | 控制器在该选项上失败 | 不能 |

关键不变量：

1. `target_track_id` 不因一个视点失败而丢失；
2. 一个 `attempt_id` 只能关闭自己对应的 candidate；
3. 旧 candidate 的 failure 不能关闭当前 candidate；
4. 只有所有已知选项都关闭，才允许把 target transaction 置为 `failed`；
5. `route_pending` 不是失败，表示规划证据还不充分；
6. 新地图 epoch 可以重新验证 `route_rejected`，但不会复活
   `controller_failed`、`arrived` 或 `observed`；
7. 已完成的 target observation 仍然由原来的多帧 completion gate 决定，ledger
   不替代视觉完成证据。

这使系统从“失败计数器 + 超时”变成“带身份的局部动作选项”。没有新增 ROS 参数，
也没有改变 `/lste/final_goal`、`/lste/goal_intent` 和 TEB action 的公开接口。

## 4. 运行时消息边界

GoalManager 在 `target_segment_committed` 时把以下字段写入 goal intent 和 command：

```json
{
  "target_track_id": "task:yellow_cup:1",
  "target_viewpoint_candidate_id": "task:yellow_cup:1::epoch:1::viewpoint:0",
  "target_viewpoint_attempt_id": "...::attempt:1"
}
```

TEB bridge 在 action lifecycle 中复制 candidate/attempt 身份；发生
`target_route_failed` 时原样带回。GoalManager 只接受当前 track 和当前 attempt 的
失败，然后执行：

```text
candidate_0 active
    -> controller_failed
    -> keep TargetApproachTransaction active
    -> clear only executable target pose
    -> next timer selects candidate_1
```

只有 candidate ledger 报告 alternatives exhausted，才走原来的全局恢复路径。旧版
没有 candidate 字段时仍保留旧行为，便于 baseline 和兼容外部控制器。

## 5. 研究意义和可比较实验

这个设计借鉴了几个成熟方向的共同边界：层级动作/Option 把短期执行终态和高层
任务状态分开，主动感知把“从哪里看”作为有身份的 action，拓扑语义导航把物理
地点和局部观测任务作为长期状态。仓库已有的研究记录整理了 Clio、OneMap、OSG
Navigator、VLFM、SG-Nav、UniGoal、RayFronts 和 R2F 的相关联系：

- [20260905_place_portal_workitem_study.md](20260905_place_portal_workitem_study.md)
- [20260906_recent_active_topology_architecture.md](20260906_recent_active_topology_architecture.md)
- [20260906_portal_evidence_graph_architecture.md](20260906_portal_evidence_graph_architecture.md)

本模块的研究假设不是“某个阈值更好”，而是：

> 在相同感知、Navfn、TEB、速度和办公楼场景下，把局部观察路线表示为目标拥有的
> 多个有身份选项，能够降低因单个视点控制失败导致的错误目标放弃，并提高最终
> 语义目标到达率。

至少比较以下方法：

1. `legacy_target_failure`：一次视点失败直接释放目标；
2. `parallax_side_retry`：保留现有的固定两侧视差分支；
3. `target_viewpoint_options`：本 ledger 方法；
4. `place_portal_workitem + target_viewpoint_options`：完整上层拓扑方法。

固定相同 world、任务、起点、检测器和 TEB。记录：

- 单个目标的 candidate failure 数和 alternatives exhausted 数；
- 同一 `candidate_id` 重复 dispatch 次数，应为 0；
- 目标 track 被错误释放次数；
- 目标成功率、完成时间、路径长度和碰撞/失败率；
- 目标 Place 的重复进入次数；
- 平均有效观察时间和运动平顺性。

当前新增测试
`src/lste_topo_access/test/test_target_viewpoint_attempt_ledger.py` 已覆盖：

- candidate 0 失败后 candidate 1 可继续；
- 失败 candidate 不会再次 dispatch；
- 旧 attempt 不能关闭当前 attempt；
- requested endpoint 与 Navfn validated endpoint 同时保留；
- 同一目标重新观察时保留失败记忆；
- 全部候选失败后才报告 exhausted；
- 新 map epoch 只重新打开 route rejection。

## 6. 当前限制和下一步

当前版本是架构切入点，不是完整论文结果：

1. GoalManager 目前还没有从 Global Frontier 的完整 map snapshot 合同中填充
   `target_viewpoint_map_epoch`，所以运行时仍以 `None` 表示未知地图 epoch；
2. 视点梯级仍由现有的 target geometry 生成，尚未与 Place 内的
   `ObservationWorkItem` 做统一 lineage；
3. 还没有在 Level 2/3/4 benchmark 上完成三种方法的重复实验；
4. 需要在真实 Gazebo 日志中确认 TEB failure payload 的 candidate/attempt 身份
   与 action generation 始终一致。

因此在完整场景、基线、消融、日志和视频证据完成前，不能把本方法称为已解决导航
问题。下一阶段应优先做 candidate ledger 与 Place-owned WorkItem 的 identity
对齐，再运行 benchmark，而不是继续增加控制器阈值。

## 7. 20260907 最小仿真证据

在 `target_entry` 起点、Level 1、关闭自动 frontier 的短实验中，主动视差动作先
尝试左侧视点，Navfn 拒绝后选择右侧视点：

```text
target_parallax_viewpoint_selected  t=27.710s
target_follow_confirmed             t=28.297s
target_segment_committed             t=28.512s
target_segment_committed             t=31.111s
target_segment_committed             t=33.322s
target_task_completed                t=68.289s
```

该运行的控制器证据是 `move_base_aborts=0`、碰撞数为 0；最终机器人与 Gazebo
目标真值的距离约 1.77m，目标框在多个新帧中达到完成条件。它证明了“无视差时
主动获取一个侧向观察位”可以解除确认死锁，但不是 Level 4 完成证据。

同一轮早期实验曾把低残差但未标定的三角测量点直接用于 completion，机器人尚未
接近 Gazebo 真值目标就错误发布 `task_done`。因此当前生产状态明确关闭静态射线
点导航/完成（`target_hypothesis_navigation_enabled=false`，代码级实验开关），
三角测量只保留为诊断数据；正式路线仍由主动视差后的 bearing-only 目标段和多帧
视觉完成门控决定。未来若要启用静态点 standoff，必须先增加相机外参/目标真值无关
的独立标定回归和误差上界证据。

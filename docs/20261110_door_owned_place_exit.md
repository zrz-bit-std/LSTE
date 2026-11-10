# 20261110 进入 Place 的门洞退出义务

## 问题

`gp_branch_tree` 进入一个单门房间后，完成全部 Room WorkItem 就再也没有可选动作：

- `portal_entry` WI 属于父 Place，过门时已 `resolve`，不会留下“回去”的义务；
- `portal_probe_ledger` 只为未绑定目的地的门洞建 probe，过门绑定后为空；
- `place_dfs.ready_to_return()` 需要 graph planner 产生
  `dfs_place_and_children_complete_return_to_parent`，而该方法
  `graph_route_planner=False`；
- `select_durable_portal_egress()` 已经能构造反向路线，但入口守卫要求
  “DFS 返回 plan”或“covered-to-covered 有 graph progress”，两者都不成立，
  于是每轮发布 `durable_portal_egress_skipped`，机器人原地等 300 s。

2026-11-10 的 env04 运行里，P2 完成 19/19 个 WI 后卡在 `(2.39, 3.83)`，
`gp_branch_waiting` 408 次、`gp_branch_wait_expired` 5 次、
`durable_portal_egress_skipped` 397 次。

## 设计

进门 WI 保持“过门即成功”的语义，不改成出门。过门成功时，由**进入的 Place**
额外持有一条方向相反的门洞义务：

```
parent → child : portal_entry  (parent-owned, crossing resolves it)
child  → parent: portal_exit   (child-owned, reverse arrival resolves it)
```

同一条物理门洞边，两条不同所有权的义务。

### WorkItem 字段

`kind = "portal_exit"`，沿用 `PlaceWorkItemLedger`：

| 字段 | 含义 |
| --- | --- |
| `place_id` | 进入的 Place（subtree） |
| `portal_id` | 入口门洞 |
| `parent_place_id` | 父 Place |
| `exit_state` | `dormant / ready / returning / resolved / blocked` |
| `generation` | 该 Place 的访问序号，防止二次进入身份冲突 |
| `activated_at` / `dispatched_at` / `dispatch_count` | 生命周期与有界重试 |
| `x, y` | 门洞坐标（仅报告） |
| `anchor_xy` | 房间内侧点 |
| `normal_xy` | 由内指向门 |

### 状态机

```
dormant --(房间本地工作清空，连续 2 周期)--> ready
ready   --(durable portal egress 选中)-----> returning
returning --(反向到达父 Place)------------> resolved
returning --(派发超时, count < limit)-----> ready
ready/returning --(count >= limit)--------> blocked
```

`blocked` 保留 `state=unresolved` 但不再计入任何完成或进度判定，避免空转。

### 计数口径（关键）

| 计数 | `portal_exit` |
| --- | --- |
| `unresolved_observation_count` | 始终排除（不是房间覆盖） |
| `unresolved_count` | 始终排除（不是 progress 证据） |
| `unresolved_ids` | 仅 `ready/returning` 包含 |
| `close_place` / `close_scoped_place` | 不清理，只有 `resolve_place_exit` 能关闭 |

`place_progress` 额外输出 `exit_work_item_id / exit_state / exit_portal_id /
exit_parent_place_id / exit_dispatch_count`，但不影响 `unresolved_work_items`。

### 接点

| 阶段 | 位置 |
| --- | --- |
| 建账 | `global_frontier_portal_arrival_commit.commit_portal_arrival`（`dfs.enter` 之后） |
| 记录 | `portal["exit_work_item_id"]`，事件 `place_exit_created` |
| 激活 | `global_frontier_planning_snapshot._activate_place_exit_if_ready`，由 `_report_no_frontier_selection` 调用；两周期滞回 |
| 否决 | 目标 claim、目标观察待办、门洞认证进行中、pending frontier |
| 出口授权 | `global_frontier_portal_egress._egress_is_owned_portal_exit` 加入 covered-to-covered 守卫 |
| 派发 | 选中 egress 时 `mark_place_exit_dispatched` + `place_dfs.mark_returning` |
| 到达 | `_commit_semantic_room_return` 先 `resolve_place_exit`，再走原返回记账 |
| 失败 | 派发超时按 `dispatch_count` 重试，超限 `blocked` |
| 审计 | `audit_place_dfs` 新增 `place_exit_obligation_recorded`（仅新日志生效） |

### 事件

`place_exit_created`、`place_exit_ready`、`place_exit_dispatched`、
`place_exit_completed`、`place_exit_retry`、`place_exit_blocked`。

## 回归

- `test_place_work_items.py`：dormant 不计覆盖、激活后可派发、`close_place` 保留、
  blocked 不算 progress、generation 幂等、法向方向；
- `test_place_exit_obligation.py`：房间有工作时不激活、两周期激活、滞回重置、
  目标 claim / pending frontier 否决、超时重试后有界阻塞；
- `test_place_dfs.py`：`mark_returning` 必须是当前栈顶且匹配入口边；
- `test_portal_egress.py`：dormant 不授权、ready 只授权自己的 Place/门洞、
  returning 保持授权、blocked 撤销。

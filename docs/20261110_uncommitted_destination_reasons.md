# 20261110 未 commit 门洞的原因记录与可视化

## 问题

`place_dfs` 只保存物理进入过的 Place。一个已经认证、已经绑定目的地的门洞，
如果在选择阶段被拒绝，就会同时失去两类信息：

- 树里没有节点：看起来这扇门不存在；
- 账本里只剩 `selection_count=0`：为什么没进去，事后无法回答。

`last_rejection_reason` 只对未绑定目的地（`destination_place_id is None`）的门洞写入，
已绑定的门洞永远为空。2026-11-09 的 env04 运行里，P2/P4/P5 正是这种情况：
三个目的地都建立了，原因只存在于瞬态的 `portal_candidates_rejected` 事件里。

## 记录层

`PortalHypothesisLedger`（`global_frontier_portal_belief.py`）新增字段：

| 字段 | 含义 |
| --- | --- |
| `commit_state` | `observed` / `committed` |
| `defer_reason` | 最近一次未 commit 的原因 |
| `defer_ros` | 该原因发生时的 ROS 时间 |
| `defer_count` | 本 run 内被推迟的次数 |
| `defer_heading_delta_deg` | 航向门当时测得的路径切入角 |
| `defer_history` | 最近 8 条 `{ros, reason, heading_delta_deg}` |
| `heading_override_count` | 语义豁免已尝试次数（用于有界重试） |

`mark_deferred()` 与 `mark_unbound_rejected()` 语义分离：

- `mark_unbound_rejected()` 继续写 `rejected_map_epoch`，表示当前 map epoch 内不再考虑；
- `mark_deferred()` 只记录原因，不抑制，下一次决策仍会重新评估。

`select()` 和 `crossed()` 会清空 `defer_reason`，但保留 `defer_history` 供审计。

原因枚举：`heading`、`excluded_radius`、`frontier_rejected`、`costmap_unreachable`、
`sealed_reentry`、`not_preferred_portal`、`preferred_identity_mismatch`、
`hypothesis_missing`、`portal_destination_not_admissible`、`recertify_failed`、
`graph_action:<reason>`、`not_selected`、`selected_not_entered`。

## 上报层

`global_frontier_reporting.py`：

- `portal_hypothesis_report()` 增加上述字段；
- 新增 `place_dfs_report()`，在 `place_dfs` 快照旁增加 `observed` 数组：

```json
"place_dfs": {
  "stack": [1, 3, 6],
  "nodes": ["... physical DFS tree ..."],
  "observed": [
    {"portal_id": 3, "source_place_id": 1, "destination_place_id": 2,
     "commit_state": "observed", "reason": "heading",
     "heading_delta_deg": 178.2, "semantic_score": 0.73,
     "semantic_priority": 1, "selection_count": 0,
     "gate": [...], "destination": [...]}
  ]
}
```

`observed` 是兄弟字段，**不是** DFS 节点。把未进入的目的地塞进 `nodes` 会破坏
`enter/return/ready_to_return` 的不变量和 `audit_place_dfs` 的子节点完成判定。

## 绘图

`scripts/tools/plot_place_dfs.py`：

- 右侧 `Physically committed Place tree` 增加灰色虚线节点 `P# obs`，
  挂在对应 `source_place_id` 下，边上标注门洞 ID、语义分、拒绝原因与航向角；
- 新日志读 `place_dfs.observed`，旧日志自动从 `portal_hypotheses` 反推，保持兼容；
- `transformations` 说明“树节点仅代表物理进入的 Place，灰色节点是观测到但未 commit 的目的地”。

## 回归

`test_portal_belief.py`：推迟记录不写 `rejected_map_epoch`、选择/穿越清理原因、
历史长度上限 8。

`test_global_frontier_reporting.py`：`place_dfs_report()` 只把未进入的目的地放进
`observed`，已进入的 Place 不出现在该列表。

# 20261110 高语义门洞的航向豁免（B2）

## 问题

`_select_portal_transition` 对每个候选门洞先算
`route_heading_delta = |navfn 路径首段切线 − 当前 yaw|`，
超过 `heading_hard_limit`（115°）就直接丢弃。普通 frontier 在
`choose_valid_frontier` 里有 `heading_limit=None` 的“必要转向”兜底，
门洞路径没有这个兜底。

2026-11-09 的 env04 运行因此丢掉了 P2：

| portal | 目的地 | 语义分 | 航向偏差 | 结果 |
| --- | --- | --- | --- | --- |
| 3 | P2 | **0.73** | ≈178°（正后方） | 每次都被 `heading` 拒绝 |
| 4 | P4 | −0.69 | ≈178° | 拒绝（符合预期） |
| 6 | P5 | 0.18 | ≈151° | 拒绝（符合预期） |

同时 evidence 决策层只把“目标方向提示”算作 `task_relevant`，房间语义分
没有进入决策向量，所以即使放开航向门，P2 也可能输给更近的普通新房间。

## 设计

两条策略，都只在 `task_semantic_value` 能力（完整方法）下生效：

1. **航向豁免**：已认证、已绑定目的地、语义分 ≥ 阈值的门洞，允许超过
   115° 的航向门，由 TEB 先做原地 `pre_route_alignment` 再进入。
   `teb_turn_supervisor_turn_lifecycle.py` 对 `portal_transition` 动作在
   `|heading_error| > 75°` 时本来就会原地转向，因此不需要新的转向机制。
2. **类别提升**：语义分达标的门洞在 evidence 层计为 `target_relevant`，
   而不是 `new_place`，从而排在普通新房间之前。

低语义房间仍走原硬门，所以“只是没去过的房间”不会被强行掉头进入。

## 有界性

`PortalHypothesisLedger` 记录 `heading_override_count`。达到
`GLOBAL_FRONTIER_HEADING_SEMANTIC_OVERRIDE_MAX_ATTEMPTS`（默认 2）后，
该门洞本 run 内不再豁免，回到普通 `heading` 拒绝。这样窄处转不动时不会
无限重试，也不会静默丢失门洞——账本里仍保留 `defer_reason`。

## 参数

```yaml
GLOBAL_FRONTIER_HEADING_SEMANTIC_OVERRIDE_ENABLED: true
GLOBAL_FRONTIER_HEADING_SEMANTIC_OVERRIDE_MIN_SCORE: 0.5
GLOBAL_FRONTIER_HEADING_SEMANTIC_OVERRIDE_MAX_ATTEMPTS: 2
```

`MAX_ATTEMPTS: 0` 等价于关闭豁免（保持旧行为）。

## 行为预期

- 高语义门洞（P2，0.73）在一次原地转向后被选择并进入；
- 低语义门洞（P4 −0.69、P5 0.18）继续记录 `heading`，不进入；
- 每次豁免都会写 `portal_heading_override` 事件，包含门洞、语义分、航向角
  和第几次尝试，便于和 `place_dfs.observed` 对照。

## 风险

- 原地转向需要 `turn_min_clearance`（0.32 m）。窄走廊里可能转不动，
  此时由有界重试兜底，不会永久卡在该门洞。
- 语义分是房间级匹配分，不等于目标在房间内；进入 P2 后仍需检测确认。
- 探索顺序会改变：先探索 P2 后，后续 DFS 路径与历史运行不可直接比较。

## 回归

`test_portal_selection_identity.py`：

- 高语义 + 178° → 允许，且第 3 次被拒；
- 低语义 + 178° → 拒绝；
- 未绑定目的地 / 无 hypothesis_id → 拒绝；
- 关闭开关或 `MAX_ATTEMPTS=0` → 拒绝。

配置链路检查：`pipeline_defaults.yaml` → `load_pipeline_config.sh` →
`nodes/config/exploration.sh` → `nodes/windows/exploration.sh` /
`run_pipeline_tmux.sh` → `online_slam_frontier.launch` 四段都已接通。

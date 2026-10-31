# L 场景零命令停滞：根因、最小修复与回放回归

- 证据运行：`runtime/gp_branch_development/place_dfs_suite_20261030_033332`
- 指标目录：`runtime/targeted_navigation/t_junction_small/logs/20261030_033332`
- 主 bag：`semantic_l_junction_slam.bag`（1 576 741 690 B，SHA-256 `e3470eab…`）
- 诊断产物：`runtime/diagnostics/20261030_zero_command/{triage,costmap,samples,verdicts}.tsv`

## 1. 结论

零命令不是 mux 策略问题，也不是规划器与 bridge 的交接问题。

`planner_no_command` 由 C++ 插件在 `teb_.computeVelocityCommands()` 返回 `false` 时发布
（`src/lste_topo_access/src/persistent_navigation_plugins.cpp`）。rosout 中该区间有
**17 421 条** `TebLocalPlannerROS: trajectory is not feasible. Resetting planner...`
（首条 sim 1541.723）。因此直接原因是 **TEB 轨迹可行性检查拒绝全部候选轨迹**。

进一步量化后，可行性检查失败的物理前提是：**机器人自身位姿已在净空禁区之内**。

| 量 | 值 |
| --- | --- |
| 机器人中心到最近 `LETHAL_OBSTACLE` 栅格中心 | 0.32 – 0.45 m（110/110 份快照） |
| 判定阈值 `footprint 0.30 + min_obstacle_dist 0.22` | 0.52 m |
| 最近 `INSCRIBED` 栅格 | 0.10 m（相邻栅格） |
| 失败时刻位姿 | ≈ `(-2.03, 12.60)`，sim 1125 → 1894 仅移动 0.1 m（769 s） |

TEB 的每一条候选轨迹都从当前位姿起步。当前位姿本身违反可行性净空时，**任何轨迹（包括原地旋转）都不可能通过检查**，因此重试、等待、以及 move_base 的旋转恢复都无法脱困。

## 2. 为什么可以排除其它三类原因

| 假设 | 证据 | 判定 |
| --- | --- | --- |
| mux 策略 / 前向限制 | `cmd_vel_mux.block_reason = none`（110/110）；`governor_reason = not_forward` 与 `filter_reason = angular_deadband` 都出现在**输入已经是 0** 之后 | 下游症状，非原因 |
| planner↔bridge 交接 | 110/110 `teb_turn_supervisor.output_decision = planner_contract_boundary`，但该决策的输入 `planner_command_contract.state = 0` 由 C++ 插件产生；合同身份 `route_id/action_generation/map_epoch` 全部匹配 | 交接是结果，非原因 |
| 其它（里程计漂移等） | sim 1873 的 `/rbt_pose` 与 `/pro3/wheels_odom` 完全一致（`-2.034, 12.601, -0.597`） | 排除 |
| 反馈陈旧 | `teb_feedback_age_seconds ≥ 10 s`（106/110） | 因为 `teb_feedback_requires_fresh_planner_command=True`，规划器不出命令 → 反馈被持续 invalidate，是结果 |

补充：`/move_base/recovery_status` 在整个运行中没有消息。persistent 分支在 TEB 失败时恒 `return true`，
move_base 永远不会进入恢复流程，因此这是**永久死锁**而不是可自愈的瞬态。

## 3. 结构性缺口

- TEB 优化器约束：`min_obstacle_dist = 0.22 m` 作用在 footprint 边缘 → 中心距 ≈ 0.52 m。
- TEB 可行性检查：footprint 半径 0.30 m 的点不得落在代价 ≥ `INSCRIBED_INFLATED_OBSTACLE`(253) 的栅格，
  而 253 带本身又是 `robot_radius = 0.30 m` → 实际要求中心距 ≈ 0.60 m 以上。
- 两者相差 ≈ 0.08 m。机器人会在 0.52–0.60 m 之间被正常驱动，然后被自己的可行性检查判死。

## 4. 最小修复

改动只在 `persistent_navigation_plugins.cpp/.h`，不改速度上限、不改 footprint、不改净空、不放开倒车。

1. **原因精度**：TEB 失败时不再一律发布 `planner_no_command`。当机器人自身栅格的最近致命栅格
   小于 `footprint_radius + min_obstacle_dist` 时，发布 `planner_start_pose_infeasible`。
   这使后续运行无需离线重建即可区分“瞬态失败”与“起步位姿死锁”。

2. **有界净空逃逸**：同一状态下连续 `40` 个控制周期（约 2 s）仍不可行时，沿**实测净空严格单调不减**
   且末端净空更大的方向执行一次有界前向机动：
   - 先在原地转向目标方向（角速度 ≤ 0.30 rad/s，`linear.x = 0`，不变更前向约束）
   - 对准后以 0.08 m/s 前进，单次上限 0.45 m / 400 周期（约 20 s）
   - 任意时刻最近致命栅格比起始值下降超过 0.05 m 立即中止
   - 净空恢复即把控制权交回 TEB
   - 找不到任何改善方向时**保持零速**，不盲动

相关参数（均可在 launch 覆盖）：
`/move_base/persistent_start_pose_clearance`（默认 0.52）、
`persistent_clearance_escape`、`..._trigger_cycles`、`..._max_cycles`、
`..._linear_velocity`、`..._angular_velocity`、`..._distance`、`..._margin`。

## 5. 单测与回放回归

`src/lste_topo_access/test/test_zero_command_diagnosis.py`（22 项，全部通过）

分类器 `src/lste_topo_access/scripts/zero_command_diagnosis.py` 按上游优先判定：

- `planner_infeasible_start_pose`：规划器无命令 + 自身位姿净空不足
- `planner_infeasible_geometry`：规划器无命令 + 起步位姿净空充足（真正的轨迹生成失败）
- `bridge_handoff_boundary`：规划器有命令但被 supervisor 或 mux 前段丢弃
- `mux_blocked` / `forward_only_reverse_clamp` / `angular_deadband_only`：命令到达 mux 后被改写
- `planner_zero_command`：TEB 成功但输出低于阈值
- `no_fresh_feedback`：各段均零且反馈链路失效

**回放回归**：对封存的 110 份 failure 快照逐份判定，结果为
`planner_infeasible_start_pose` **110/110**，且每份的实测净空都严格小于 0.52 m
（`runtime/diagnostics/20261030_zero_command/verdicts.tsv`）。

## 6. 剩余风险

- 逃逸是新行为，只有真实 L 场景能验证其能否把机器人带出禁区；参数保守（0.08 m/s、0.45 m）。
- 若走廊宽度本身小于 TEB 的实际可行净空（本例约 1.2 m 走廊 vs ≈1.3 m 要求），逃逸只能脱困一次，
  系统仍会反复进入。根本解需要让全局规划与 TEB 的可行净空一致（Navfn 代价模型或终点/走廊净空门），
  属于独立的策略决策，本次未改动。

## 7. 实景验证（两轮 L）

### 7.1 第一轮 `place_dfs_suite_20261030_211917` / run `20261030_211918`

- 零命令死锁本身消失：`trajectory is not feasible` 从 17 421 次降到 26 次；
  失败快照从 110 份降到 8 份；机器人持续运动到 sim ≈1700。
- `planner_start_pose_infeasible` 正常记录（26 次），最长连续 10 个周期，未触发逃逸
  （阈值 40），说明短时贴障可以自行恢复。
- **残余停滞换成另一种形态**：sim 1450–1800 机器人在 `(-3.31, 13.94)` 原地打转，
  400 s 内累计偏航 27.03 rad、位移约 1.0 m。rosout 显示
  `TEB selected a persistent near-zero route command; rotating once toward Navfn tangent`
  与 `started stalled route reorientation` 每 8–12 s 重复一次。
  该位姿实测最近致命栅格 0.447 m（< 0.52 m），即 TEB 并未返回 false，而是给出
  **仅旋转**的可行解；原逃逸逻辑只在 `!command_available` 时触发，覆盖不到。

### 7.2 第二轮 `place_dfs_suite_20261030_220216` / run `20261030_220218`

- 已把逃逸触发扩展到“净空不足 + 平移分量为零”的只旋转情形（4 倍观测窗口），
  并把逃逸动作改为带前进分量的弧线，避免被转向监督器的近零替换覆盖。
- 该轮在 sim 108 起出现**第三种死锁**：全局 costmap 占用率从 3.0% 跳到 99.5%，
  `navfn_plan.length = 0.0`、`endpoint = null`，本地规划器长期停在 `terminal_hold`，
  `s1:planner_command` 计数自 sim ≈506 起完全不再增长，累计 215 份失败快照。
  该形态与本次修改的代码路径无关（`planner_clearance_stall` / `planner_clearance_escape`
  在该轮均未出现），但尚未定位到是 place scope 掩膜还是 mission 事务未刷新导致的
  terminal_hold 不释放。

### 7.3 审计结果

第一轮 wrapper 审计：`launch=1 recording=1 tree=1 lifecycle=1 dfs=1`，
但 `auth/auth_all/turn_raw/tsp/geometry/trajectory/native` 仍未通过。
**L 场景目前没有通过审计。**

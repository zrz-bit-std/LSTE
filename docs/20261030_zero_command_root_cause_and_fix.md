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

## 8. 第二轮死锁的定位与修复（`terminal_hold`）

### 8.1 证据

第二轮 `20261030_220218` 的合同消息显示：`s1:planner_command` **最后一次出现在 sim 97.8 s**，
之后全是 `terminal_hold`，并伴随每 ~9 s 一次的 `endpoint_reached`（共 82 次）、
`mission_boundary` 154 次、`move_base_failed_terminal` 72 次。

关键量化：快照中的 `teb_global_plan` 在 sim 173 / 365 / 587 / 806 四份里**完全相同**
（28 个位姿、长度恒 1.454 m），`navfn_plan.length = 0.0`、`endpoint = null`。

### 8.2 因果链

1. Navfn 在全局 costmap 上找不到路径，`makePlan` 失败；
2. `StreamingNavfnPlanner` 走“保留上一条有效路径”的分支，把**上一条路线**的缓存计划
   交给了**新的 mission goal**；
3. move_base 把它交给 `PersistentTebLocalPlanner::setPlan`；计划的终点就是机器人已到达的位置；
4. `teb_.isGoalReached()` 立刻为真 → 插件上报 `endpoint_reached` 并置 `terminal_hold`；
5. 图侧认为该路线已完成并下发下一条 → 回到第 1 步。

结果是机器人在 `(0.08, 2.80)` 静止 758 s、累计 215 份失败快照，而“失败”表象是每条路线
都“成功到达”。

### 8.3 修复

- `StreamingNavfnPlanner::makePlan`：仅在缓存计划的目标与当前请求目标一致
  （同 frame 且距离 ≤ `goal_epsilon_`）时才保留，否则返回失败，
  让 move_base/图侧真实地失败这条路线，而不是伪造到达。
- `PersistentTebLocalPlanner::computeVelocityCommands`：`endpoint_reached` 只在
  确实上报了终点时发布，否则发布 `planner_no_endpoint`。

### 8.4 冒烟验证（`place_dfs_suite_20261031_165651`，20 分钟）

- 假完成循环消失：失败快照 **215 → 15**，`planner_command` **994 → 1669**，
  `terminal_hold` **7746 → 359**，`endpoint_reached` **82 → 11**。
- move_base 恢复行为开始真正执行（快照中出现 `conservative_reset 0/4`），
  这在第二轮从未发生。
- **但冒烟门禁未通过**：合同消息在 sim 295.9 s 后完全停止，机器人停在
  `(-15.17, -1.88)` 直到 sim 913。全局前沿日志显示
  `no strict_clearance candidate within heading limit 115.0deg`、`active=False`、
  单周期 `2.07 s`。即导航层不再死锁，**探索层选不出候选而进入 idle**。

### 8.5 结论

第三次停滞属于探索/候选选择层，不属于本次导航修复的代码路径。
在它解决前跑满 60 分钟的 L 只会得到同样的 idle，因此**暂不启动满时长重跑**。
重跑工具已就绪（见下）。

## 9. 重跑工具

- `scripts/tests/targeted_navigation/l_stall_gate.py <suite_dir>`：把上述四道门禁
  （规划命令持续、机器人确实在移动、起步净空连续段有界、失败快照数有界）
  合并成一条命令，可在运行中随时执行。
- `scripts/tests/targeted_navigation/run_l_clean.sh`：干净环境启动器。
  预检会拒绝以下情况：残留 Gazebo/roslaunch/rosbag、套件端口被占用、
  插件库比源码旧、回归测试未通过。

## 10. 探索层候选饥饿（A）

结论：**不是 `strict_clearance`（0.52 m）刷掉了候选**。

- `required_clearance = 0.52`，实测 `forward_clearance = 1.83 / 2.25 / 2.30 m`；
- sim 293.76 `route_selected`：`candidate_count=11, feasible_count=11, pareto_front_count=6`；
- 系统记录到的拒绝原因只有 `no_reachable_observation_viewpoint`(39)、
  `viewpoint_already_attempted`(30)、`place_lifecycle_admission_failed`(22)，**没有任何 clearance 拒绝**。

真正冻结点：sim 295.77 `work_item_suspended` + `controller_lease_released`；sim 298.17
`portal_authentication_waiting_source_side_geometry`（node 5，branch 11，
`source_depth=+0.473`，即机器人在门洞目标侧）→ `withdraw_pending(11)`；
剩余分支 `[9,10,12]` 停在 `phase=watching`、`all_finished=false`，且永远起不来。
`watching` 期间调用方每轮都收回普通路线，于是 298.2→913 只余心跳。

修复：`JunctionDoorwayAuthentication.expire_unstartable()` 在
`/lste_global_frontier/portal_observation_start_timeout`（默认 30 s）内没有任何可启动
分支时，失败剩余分支并完成任务；调用方在收回路线之前调用它，并把树侧分支标记为
`authentication_state=failed`。

## 11. 物理停滞（B）：致命格编码不匹配

从封存包在 sim 261 直接比对同一格：

| 来源 | 值 |
| --- | --- |
| `/map`（SLAM 占据栅格） | **0（自由）** |
| `/lste/place_navigation_mask` | -1（未知） |
| `/move_base/global_costmap/costmap` | **100（致命）** |

Navfn 因此返回空路径（`length=0.0`, `endpoint=null`），move_base 中止目标并进入
`conservative_reset`，每条路线 4–8 s 就失败一次（25 次
`controller_stall_no_physical_progress`）。

根因：`global_frontier_planning_costmap.py` 的四套门禁全部按
`>= 253` 判定致命（costmap_2d 内部编码），而订阅到的
`/move_base/global_costmap/costmap` 是 **0–100 编码**（100=致命、99=内切、-1=未知）。
于是致命格 100 远低于阈值，`free = data < 253` 把障碍当成可通行，
可达性门禁、终点净空门禁、Navfn 启动探针全部失效。

修复：新增 `lethal_threshold()` / `lethal_mask()`，从栅格观测到的最大值推断编码
（峰值 ≤100 用 100，否则用 253），四个调用点共用。终点 3 格余量未改动。

## 12. 原地打转：转向被身份刷新掐断（C）

`20261031_172738` 在 `(-3.24, 13.98)` 从 sim ≈550 停到 931：207 s 内累计偏航
9.75 rad、位移 0.11 m，线性速度恒 0。

监督器日志：TEB 在距终点 1.45 m、航向误差 22–35° 时给出近零命令 → 启动
`stalled route reorientation`（目标 yaw ≈ −89.7°）→ **约 1 s 后以
`bridge_action_identity_changed completed=False` 释放**，36 次里 27 次如此。

核实两点：

1. 转向启动时上下文是 `goal_role="place_observation"`（owner 2 / src 2），
   0.1–1 s 后翻成 `goal_role="geometry_frontier"`（owner=None / src=None）；
2. `mission_id` 全程相同（`portal_semantic_test`），但 `graph_transaction_id`
   每 1–2 s 自增一次。

结论：不是新派发，而是**同一次派发被交替打上两种角色标签**；而续接谓词
`_continue_stalled_route_turn_for_handoff_locked` 要求两侧都是
`place_observation` 且 `owner_place_id>0`，因此
`stalled_route_turn_handoff_continued` 全程 0 次。

修复：改为按 **mission 身份 + 几何一致性** 判定同一次派发——角色白名单放宽到
`place_observation / place_observation_work / geometry_frontier`，
place/source place 仅在两侧都已知时才要求相等；终点 ≤2 m、目标 yaw 在容差内、
最多 8 次交接、非认证 portal 等原有边界全部保留。

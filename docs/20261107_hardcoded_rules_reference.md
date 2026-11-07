# LSTE 硬逻辑与写死规则参考

> 本文件汇总 `/home/zrz/work/lste_ws` 中运动控制、规划、感知打分、状态机、失败取证与评测里**写死的阈值、超时、魔法数字和状态转移条件**。
> 重点在「运动控制 + 规划」，感知/状态只做概括。
> 撰写时 Git 基线：`d5f1ff7`（2026-11-07 19:19）。

## 0. 读法约定（三类来源，数值优先级）

| 来源 | 含义 | 是否可改 |
| --- | --- | --- |
| **硬编码** | 直接写在 `.py` 里的常量，无 ROS 参数入口 | 改代码才能变 |
| **ROS 参数默认值** | `rospy.get_param("~name", 默认值)`，运行时可用参数服务器/launch 覆盖 | 不改代码可覆盖 |
| **pipeline_defaults.yaml** | 生产生效配置，经 `scripts/config/load_pipeline_config.sh` 转成 `CFG_*` 环境变量，再由 `run_all_tmux.sh` 注入 launch 参数 | 改 YAML + 重启即可 |

实际生效值以 **pipeline_defaults.yaml** 为准（它覆盖代码默认值）；表内会同时标注「YAML 生效值 / 代码默认值」以避免混淆。

---

## 1. 感知打分与状态（简要）

### 1.1 WeDetect 打分

每一帧检测结果被拆成三路得分，再加权合成总分：

```
S_total = 0.2 * S_target + 0.3 * S_env + 0.5 * S_ctx   （clamp 到 [0,1]）
```

| 分量 | 含义 | 关键写死规则 |
| --- | --- | --- |
| `S_target` | 目标本体置信度 | 取「短语包含目标名」或「token 匹配 ≥ 60%」的检测框中 **logit 最大值**，clamp [0,1]；无目标框 = 0 |
| `S_env` | 环境一致性 | 有效环境检测 < 2 个 → 直接 0；`S_env = logistic(pos_ratio) − 0.7 * logistic(neg_ratio)`，clamp [−1,1] |
| `S_ctx` | 上下文一致性（left/right 邻居） | 每个 ctx 词找最近邻检测框，`raw = 0.5*覆盖率 + 0.5*平均邻近度`；全未命中 → −0.4；2 个 ctx 只中 1 个 → 封顶 0.35 |

WeDetect 后端推理常量（`utils/wedetect_backend.py`）：`score_threshold=0.20`、`nms_iou=0.70`、`pre_nms_topk=3000`、`max_detections=100`、`variant=base`（输入 640×640）、`trt_max_classes=16`、label 中文映射表。

### 1.2 三状态状态机（`lste_state_node.py`）

状态：`PASS=0` / `SUSPICIOUS=1` / `LOCKED=2`（`EXHAUSTED=3` 已禁用）。

- **promising（有希望）**：`S_total≥0.45` 或 `S_target≥0.60` 或 `S_env≥0.35` 或 `S_ctx≥0.35` 或 `detected`。
- **进入 SUSPICIOUS**：近 10 帧窗口内有 ≥3 帧 promising；或已在 SUSPICIOUS 且未全低。全低（total<0.35 且 target<0.50 且 env<0.25 且 ctx<0.25）回 PASS。
- **进入 LOCKED**：近 2 帧里 ≥2 帧满足 `S_total≥0.6 且 S_target≥0.6 且 detected`。退出：连续 5 帧不满足即解锁。
- **subtype**：`Sus-A`（目标框 score≥0.4）→ `Sus-B`（left+right 上下文都检出且是不同框）→ 其余 `Sus-C`；subtype 切换需连续 3 帧确认。

### 1.3 观望期（两种）

| 观望期 | 触发 | 动作 | 结束 |
| --- | --- | --- | --- |
| **上下文观望** `ContextObservationPolicy` | 接近上下文目标停滞 ≥4s 或已到 0.55m | 原地按 `[0,30,-30,60,-60]°` 顺序扫头，每 2s 换一个角度 | 10s 后 abandon 或出现目标转 target |
| **近距目标观望** `TARGET_CLOSE_OBSERVING` | 已确认目标且框已足够大、完成证据未凑齐 | 保持当前位置（navigation_hold），等完成证据多帧投票 | 凑齐完成证据 / 证据过期 |

---

## 2. 运动控制（详细）

### 2.1 cmd_vel_mux：唯一的 `/cmd_vel` 出口（`lste_cmd_vel_mux_node.py`）

#### 前向碰撞调速器（governor，安全不变量）

对**所有**来源（teb/sappo/teleop）生效：

```
v_max = min(governor_max_speed, sqrt(2 * governor_decel * max(0, clearance - governor_min_gap)))
```

| 参数 | 值 | 生效时机 | 作用 |
| --- | --- | --- | --- |
| governor_decel | 0.60 m/s² | 前向速度 > 0 | 制动减速度 a |
| governor_min_gap | 0.45 m | 前向速度 > 0 | 停车后保留的安全间隙 |
| governor_max_speed | 0.50 m/s | 前向速度 > 0 | 前向速度硬顶 |
| 前向扇区 | ±25° | 每次激光扫描 | 只取正前方扇形内最近障碍算 clearance |
| 有效激光点 | > 0.01 m | 每次扫描 | 丢弃贴脸/无效点 |

`governor_reason` 枚举：`not_forward`（倒退不评估）/ `forward_scan_unavailable`（无有效 clearance）/ `governor_limited`（被限速）/ `within_governor_limit` / `not_evaluated`。

#### 输出门控（block_reason）

输出被阻止（`active=false`）时：`source_not_selected`（来源 ≠ 当前 mode）、`task_complete`（task_done）、`navigation_hold`、`hard_reset_hold`、`hard_reset`、`controller_mode_switch`。

#### TEB 前向-only 钳位与角速度滤波

| 规则 | 值 | 生效时机 | 作用 |
| --- | --- | --- | --- |
| forward_only_reverse_clamp | linear.x < 0 → 0 | `teb_forward_only=true` 且 TEB 输出负线速度 | 前向-only，倒退钳 0（防止倒退振荡） |
| teb_angular_deadband | 0.02 rad | `|ω| ≤ 0.02` | 角速度置 0，抑制走廊 zig-zag 噪声 |
| teb_angular_sign_switch_threshold | 0.12 rad | 角速度反向且 `|ω| < 0.12` | 判「微反向」→ micro-hold 或清零 |
| teb_micro_hold_limit | 4 个控制周期 | 微反向后 | 保持上一角速度 ≤4 周期，避免转弯口零速脉冲 |
| turn_hint 新鲜度 / 最小角速度 | 0.25 s / 0.05 rad | 前向钳位时 | 沿用 TEB 反馈里的转弯意图，避免「零→旋转」间隙 |

`filter_reason` 枚举：`angular_deadband` / `angular_micro_reversal_held` / `angular_micro_reversal_suppressed` / `forward_only_reverse_clamp` / `transaction_rejected` / `reset` / `none`。

#### TEB 命令合同校验（`on_teb_command_contract`）

非零命令必须携带有效身份，校验链依次检查（任一失败即拒绝并停机，`block_reason` 直接等于拒绝原因）：

`transaction_id>0` → 非陈旧（`transaction_id`、`command_sequence` 单调递增）→ `planner_transaction_id==transaction_id` → `route_id>0` → `planner_route_id>0` → `active_action` → `action_generation` 一致 → `active_transaction/route/generation` 一致 → `graph_transaction_id` 一致（graph 所有权时）→ `route_id==planner_route_id==active_route_id` → `map_epoch` 存在且与 planner/active 一致。

拒绝原因（部分）：`missing_transaction_id`、`stale_command_transaction`、`planner_transaction_mismatch`、`missing_route_id`、`route_id_mismatch`、`action_generation_mismatch`、`missing_graph_transaction_id`、`graph_transaction_mismatch`、`missing_map_epoch`、`map_epoch_mismatch` 等。

### 2.2 TEB 参数（`launch/teb_navigation.launch`）

| 参数 | 值 | 作用 |
| --- | --- | --- |
| max_vel_x | 0.60（YAML `TEB_MAX_LINEAR_SPEED=0.50` 生效） | 最大前向速度 |
| max_vel_x_backwards | 0.01（Noetic 最小值） | 反向下限（mux 仍拒绝负值） |
| max_vel_theta / acc_lim_theta | 0.80 rad/s / 1.30 rad/s² | 角速度 / 角加速度上限 |
| acc_lim_x | 1.10 m/s² | 线加速度上限 |
| xy_goal_tolerance | 0.50 m | 终点 XY 到达容差 |
| yaw_goal_tolerance | 3.14159 rad | 终点 yaw ≈ 永不要求（前端点到达即完成） |
| free_goal_vel | true | 到达目标不强停（task_done 的 mux 门做最终停车） |
| min_obstacle_dist / inflation_dist | 0.22 / 0.55 m | 障碍最小距离 / 膨胀距离 |
| footprint_model/radius | circular / 0.30 m | 圆形 footprint |
| obstacle_proximity_ratio / lower / upper | 0.40 / 0.35 / 1.20 m | 接近障碍渐进限速（1.2m 起 0.40 为地板） |
| dt_ref / dt_hysteresis | 0.25 / 0.12 s | 弹性带时间步长 |
| control_look_ahead_poses / prevent_near_goal | 5 / 3 | 前视位姿数 / 终点附近豁免数 |
| force_reinit_new_goal_dist / angular | 1.00 m / 0.78 rad | 超过则强制重建弹性带（热启动失效） |
| global_plan_viapoint_sep | 0.60 m | viapoint 间距（抑制栅格 zig-zag） |
| weight_viapoint / optimaltime / acc_lim_theta | 0.50 / 0.60 / 3.00 | 优化权重（acc_lim_theta 权重抑制左右摆） |
| weight_obstacle / inflation | 45.0 / 0.20 | 障碍 / 膨胀权重 |
| weight_kinematics_nh / forward_drive | 1000 / 1000 | 运动学（非完整 / 前向）硬权重 |
| feasibility_check_no_poses | 3 | 可行性检查位姿数 |
| enable_homotopy_class_planning / max_number_classes | true（YAML false）/ 5 | 多拓扑候选（左/右绕行） |
| penalty_epsilon | 0.0005 | 优化 penalty epsilon |

move_base 层：`controller_frequency=20Hz`、`planner_frequency=0`（事件驱动）、`planner_patience=5s`、`controller_patience=5s`、`oscillation_timeout=15s / oscillation_distance=0.25m`、`transform_tolerance=1.0s`。

代价地图：全局 `robot_radius=0.30 / inflation_radius=0.70 / cost_scaling=5.0 / static_map`；局部滚动窗 `8×8m / resolution=0.10 / robot_radius=0.30 / inflation_radius=0.55 / cost_scaling=3.0 / update 10Hz`，激光 `obstacle_range=8.0 / raytrace_range=10.0`。Navfn `allow_unknown=true`、`default_tolerance=0.20m`。

### 2.3 turn supervisor（`lste_teb_turn_supervisor`）

| 参数 | 值 | 作用 |
| --- | --- | --- |
| yaw_goal_tolerance / max_vel_theta / acc_lim_theta | 0.35 rad / 0.80 rad/s / 1.30 rad/s² | 转弯完成容差 / 限速 / 限加速度 |
| turn_start_confirm_duration | 0.50 s | 转弯目标需稳定该时长才开转 |
| turn_rotation_cap_factor | 1.6（单次至少 120°） | 旋转预算 = 初始误差 × factor，防绕满圈 |
| turn_settle_duration | 0.35 s | 转弯后保持航向，防 TEB 反向重锚 |
| pre_route_turn_threshold | 75°（钳 30–90°） | Navfn 入口航向误差超过则触发 pre-route 转弯 |
| SHARP_ENTRY_CONTINUITY_BLOCK_RAD | 90°（硬编码） | 入口航向误差 ≥90° 抑制连续性桥接 |
| turn_min_clearance | 0.30+0.02=0.32 m | 原地转弯前底盘内无障碍 |
| turn_scan_timeout | 0.50 s（硬编码） | 激光扫描新鲜度，超时禁转 |
| stalled_route_reorientation_delay / heading / linear / angular / min_distance | 0.80 s / 45° / 0.03 m/s / 0.15 rad/s / 1.00 m | 停滞时 TEB 反向入口 → 有界原地重定向 |
| trajectory_feedback_timeout / cap / scale | 0.35 / 0.60 s / 1.50 | 反馈新鲜度（EMA 自适应，封顶 0.60） |
| trajectory_continuity_min_forward / max_hold / terminal_radius | 0.05 m/s / 0.30 s / 0.75 m | 连续桥接门槛 / 最大保持 / 终点半径内不桥接 |

状态机：`PASS_THROUGH` ⇄ `TURNING`。进入 TURNING 需全部满足（mode 匹配、无 task_done/hold、动作已锁、intent 与 goal 匹配、确认窗口、扫描 clearance 通过）；退出原因（yaw 到达 / 旋转预算超 / 无进展 / clearance 丢失 / task_done / hold / 控制器切换 / 契约变化 / 生命周期失败）。输出决策枚举 `output_decision`：`inactive/turn/turn_clearance_release/move_base_recovery/planner_contract_boundary/pass_through/trajectory_continuity/forward_only_reorientation/turn_settle/task_done/navigation_hold/controller_mode`。

### 2.4 teb_goal_bridge（目标桥 / 看门狗 / 生命周期）

| 参数 | 值 | 作用 |
| --- | --- | --- |
| position_epsilon / yaw_epsilon | 0.15 / 0.08（bridge 实际） | 目标同点去重 |
| goal_min_update_interval / retry | 2.50 / 3.00 s | 高频重派发抑制 / 失败重试间隔 |
| handoff_distance | 1.00 m | 距目标 < 阈值才允许 stall 交接 |
| progress_timeout / progress_epsilon / yaw_epsilon | 12.0 s / 0.12 m / 0.12 rad | 无进展看门狗（平移或原地转都算进展） |
| handoff_min_interval | 4.0 s | 派发后保护期，至少 4s 才评估 stall |
| frontier_observation_completion_radius | 0.0（生产禁用） | 观察区提前完成（生产等待正常终端） |
| frontier_observation_completion_max_linear_speed / stationary_hold | 0.15 m/s / 0.10 s | 低速 + 持续时长才算观察完成 |
| teb_reorientation_*（linear/angular/yaw_progress/stagnation/max_extension） | 0.03 / 0.15 / 0.10 / 1.50 s / 8.0 s | 原地重定向的判定与预算 |
| prefetch admission（horizon / max_costmap_age / blocked_cost / service_timeout / endpoint_epsilon / lookahead_trigger） | 2.0 m / 0.50 s / 100 / 0.10 s / 0.20 m / 1.0 m | 预取后继路线的准入（Navfn+局部代价图一致才放行） |
| persistent curve handoff max heading | 65°（45–89°） | 曲线交接航向上限（超过走终端生命周期） |

硬编码常量：`ROUTE_LEASE_WATCHDOG_SECONDS=10s`（终端后无 ACK 强制释放路由）、`ACTION_TOMBSTONE_LIMIT=128`、`TRANSPORT_HANDOFF_TIMEOUT=15s`、`CONTROLLER_RESTART_MAX_ATTEMPTS=2`、`lifecycle_tick=0.2s`、`lifecycle_dispatch_timeout=120s`。

---

## 3. 规划（详细）

### 3.1 Navfn 验证与全局规划

| 规则 | 值 | 生效时机 | 作用 |
| --- | --- | --- | --- |
| Navfn allow_unknown | true | 全局规划 | 未知格高代价可通行（视觉目标在地图外时兜底） |
| Navfn default_tolerance | 0.20 m | 终点到达 | 终点容差 |
| target_route_validation | true | 视觉目标成为导航目标前 | 必须 Navfn 返回连通 plan 才接受 |
| target_route_validation_timeout / tolerance / period | 0.05 s / 0.20 m / 1.0 s | 视觉目标路由验证 | 验证服务超时 / 容差 / 周期 |
| GLOBAL_FRONTIER_NAVFN_EMPTY_WARMUP_SECONDS | 5.0 s | 热重启窗口 | 空响应重试而非拉黑分支 |
| GLOBAL_FRONTIER_NAVFN_STARTUP_PROBE_DISTANCE | 0.40 m | 首条 frontier 前 | Navfn 需能规划到 ≥0.4m 才认为 map 就绪 |
| planner_frequency | 0.0 | 全局规划 | 事件驱动，避免定时重建 TEB 弹性带产生零速脉冲 |

### 3.2 全局前沿打分与选择

| 权重/参数 | 值 | 作用 |
| --- | --- | --- |
| information 权重 | 0.035（硬编码） | 信息增益项 |
| path_cost 权重 | 0.12（硬编码） | 路径长度惩罚 |
| structure 权重 / radius / min_cells | 0.16 / 10 格 / 3 格 | 近墙结构偏置 |
| heading penalty | heading_weight×(1−cosΔ)，权重 2.5 | 软航向连续性（正后方最多 2×权重） |
| heading_hard_limit | 115° | 硬过滤（超出拒绝） |
| semantic_hint penalty | 0.35 × min(12, d) | 锁定目标被墙挡时，向目标方向软偏置 |
| info_radius | 8 格 | 信息计数邻域 |
| dead_end_check_distance / unknown_cells | 4.0 m / 15 格 | 死端检测（无未知判死端） |

### 3.3 图路线 planner（Place / Portal / WorkItem）

| 规则 | 值 | 作用 |
| --- | --- | --- |
| PLAN 枚举 | ready / complete / blocked | 每次规划结果 |
| 义务优先级 `_best_path` | target(0) > unobserved(1) > work_item(2) > probe(3) > unbound(默认9) | 多跳目的地选择（新颖性优先） |
| portal-entry 默认 priority | 10⁹ | 无 priority 时兜底排序 |
| SEMANTIC_ENTRY_PROJECTION_MAX_MISSES / COOLDOWN_EPOCHS | 5 / 10 | 入口 portal 连续投影失败 5 次进入 10 epoch 冷却 |
| CompletionGate | 4 类义务计数任一 >0 即不完成 | 完成门槛 |
| 完成恢复状态机 | searching/waiting/recovering/blocked/complete | 相同 signature 二次 audit 仍无进展 → blocked |

### 3.4 Portal（门洞）探测 / 认证 / 穿越

| 规则 | 值 | 生效时机 | 作用 |
| --- | --- | --- | --- |
| required_depth | 0.40 m | signed_depth ≥ 阈值 | entering → inside |
| return_radius | 0.75 m | 回程距 source 节点 | 判定回到路口 |
| entry_goal_margin / return_goal_margin | 0.60 / 0.60 m | 目标外延 | 越过门洞平面 |
| turn_tolerance | 0.40 rad | 认证转身 | 略大于 TEB 0.35 |
| source_side_proven margin | 0.05 m | 认证完成 | source_depth < −0.05 证明源侧 |
| portal_crossing_depth | max(0.05, 0.5×clearance) | 过门判定 | 车体中心过半 clearance |
| portal_crossing_evidence | source_signed<0 且 terminal_signed≥0（或双侧预观察） | 动作终止 | 物理穿越证据 |
| portal_match_radius | 0.30（复用×2） | 投影漂移复用 | 门漂 ≤2×match 才复用 |
| authentication_retry_budget | clamp(0.5×active_timeout, 8, 12) s | 认证故障重试 | 有界重试 |
| return_recovery 阶段1 / 阶段2 | retry≥3 且 ≥45s / retry≥6 且 ≥140s | 回程停滞 | return_recovery_blocked / final_blocked |
| return 控制器失败上限 | ≥4 次 | 连续失败 | 直接 final_blocked |
| settle_seconds | 1.0 s | 新 branch 后 | 静默稳定窗口 |
| destination_returns_into_source radius | 0.9 m | 建目的地前 | 自环抑制 |

PortalTransaction 状态机：`idle → source_probe → throat → destination_standoff → crossing_verified → place_commit → finish/aborted`。

### 3.5 Place 覆盖 / 离开 / 重访

| 规则 | 值 | 作用 |
| --- | --- | --- |
| completed_radius | 1.25 m | 已到达端点排除，抑制重选 |
| region_memory_radius | max(completed×2, approach×2) | Place 身份半径 |
| component_min_cells | 12 格 | 少于则不构成 Place |
| destination_was_visited radius | 0.9 m | 重访判定（自环抑制） |
| region_failure_limit | 2 | Place 停滞失败上限，超过放弃 |
| region_stagnation_timeout / delta | 20 s（YAML）/ ≥8 格信息增量 | 停滞判定 |
| place_wall_fallback_seconds / progress_radius | 45.0 s / 0.35 m | 首次观察长期缺失的存活兜底 |
| visited min_spacing / limit | 0.12 m / 6000 | 路径记录采样与上限 |

Place 状态：`open / ready_to_exit / dormant / suspended`；PlaceProgress 相位 `bootstrap / observe / exit / transit`。

### 3.6 WorkItem 生成 / 耗尽

| 规则 | 值 | 作用 |
| --- | --- | --- |
| merge_radius / viewpoint_merge_radius | 后者 ≤ 前者 ≥ 0.05 | 边界身份 / 视角去重分离 |
| _directions_compatible | dot > 0.0 | 法向相反不可继承 WI |
| _anchors_compatible | 锚点距 ≤ merge_radius | 锚点身份匹配 |
| reconcile 匹配质量 | anchor=2 / overlap=1 / 无锚点=0 | 弧匹配锚点优先 |
| viewpoint_is_available | 距 failed/blocked/rejection > viewpoint_merge_radius | 已试视角不复派 |

WorkItem 状态 `unresolved/resolved`；Attempt `active/failed/blocked/succeeded/suspended`。

### 3.7 路线 lease / 事务 / 所有权（超时）

| 规则 | 值 | 生效时机 | 作用 |
| --- | --- | --- | --- |
| GLOBAL_FRONTIER_ACTIVE_TIMEOUT | 60.0 s（代码默认 18） | 路线总时长最终护栏 | 仅作 STALL 之后的最后护栏 |
| GLOBAL_FRONTIER_STALL_TIMEOUT | 20.0 s（代码默认 8） | 路线无进展 | 停滞看门狗 |
| GLOBAL_FRONTIER_POST_TURN_STALL_TIMEOUT | 6.0 s | pre-route 转弯后未启动 | 启动看门狗 |
| GLOBAL_FRONTIER_UNREACHABLE_GRACE | 20.0 s | 地图整合短暂断连 | 保留最后端点 |
| GLOBAL_FRONTIER_REJECTED_TIMEOUT | 180.0 s | 路线真停滞 | 分支排除后可重试时长 |
| controller_failure_hold_seconds / terminal_wait | 20.0 / 12.0 s | 控制器持有失败 / 持久模式终局等待 | 保留路线 / 超时回退看门狗 |
| controller_release_ack_wall_seconds | 30.0 s | 等 release ACK | 防止永久门控 |
| endpoint_terminal_wait_radius | max(0.8, teb_tol+0.50) | 终点到达 | 终局包络 |
| goal_stagnation_limit / seconds | 3 / 25.0 s | 目标停滞 | 放弃目标 |
| rejected_timeout | 180.0 s | 路线被拒 | 排除候选时长 |
| progress_epsilon / odom_novel_cell_size | 0.12 m / 0.40 m | 看门狗 | 最小进展 / 独立进展证明 |

路线失败迁移（唯一合法顺序，`route_lease.py`）：`recovery → controller timeout → portal_edge → post_turn → active_timeout → stall`。失败权威：`global_watchdog` / `controller_terminal`。

所有权：`route_ownership = graph | observation`；observation 路线跳过 graph 高水位（`graph_transaction_id=0` 合法）；未知 payload 默认 graph（fail closed）。普通 frontier 不抢占已穿越/活跃的门洞事务（`decide_replan_lease`：已穿越/活跃→DEFER，否则→PREEMPT）。

### 3.8 证据合同（evidence contract）

- REQUIREMENT 状态：`pending / resolved / blocked`
- OWNER：`place / portal / portal_probe / work_item / target`
- EVIDENCE 十类：`place_view / work_item_open / work_item_viewpoint / portal_certified / portal_source_view / portal_crossing / portal_destination_view / target_verify / target_viewpoint / target_verified`
- gap 排序优先级：`crossing < destination_view < source_view < work_item_viewpoint < target_verify < place_view`；next_gap 优先级 `target < crossing < work_item < source_view`

---

## 4. 遥测 / 失败取证 / 生命周期

### 4.1 失败 episode（`navigation_metrics_failure_evidence.py`）

- pre_window=12s、post_window=12s、zero_velocity_limit=5s、no_progress_limit=8s；`failure_id="{run}-F{NNNN}"`，快照 `"{run}_failure_NNNN.json"`。
- 开 episode：`zero_velocity_stall`（|lin|≤0.05 且 |ang|≤0.03、goal_far≥0.80、持续≥5s）、`controller_output_gap`（≥5s）、`no_progress_stall`（有命令>0.08 但平移<0.04m 持续≥8s）、`frontier_route_stall`（≥8s 且 count≥3 或 target_failure_latched）、`target_route_failed`、边界强制（no_active_controller_route / handoff_timeout≥10s / controller_stall）。
- 可恢复触发（`frontier_route_stall / planner_materialization_stall / zero_velocity_stall`）在 post 窗口出现健康后继 → 丢弃为 context。
- 分类标签：`intentional_navigation_hold / planner_no_path / controller_stall / planner_materialization_stall / owner_transaction_race / recovery_exhausted / local_obstacle / tf_or_transform / unknown`，置信度 high/medium/low。
- 日志保留 15 天（retention_days）。

### 4.2 Watchdog（批实验 + 路由层）

- 批实验 `repeat_local_search.py`：原地停滞 50s / 半径 0.35m；总探索 600s；无 pose 60s 判 infra；launcher 非 0 判 infra；上报 30s、轮询 2s。
- success=3（默认，本批次新增成功，历史不计）；失败自动重开；infra 或可视化失败立即停；可视化重试 1 次、timeout 180s。

### 4.3 健康审计（`lste_health_audit.py`）

startup_grace=120s、check_interval=2s、cuda_interval=30s、node_ping_timeout=0.75s；critical_failures={cuda,ros_master} 不享宽限；fresh 阈值 detections=20s / final_goal=20s / prompts=15s / scores=20s（final_goal 在完成后豁免）；状态 healthy/starting/unhealthy。

### 4.4 tmux / 停止

会话 `lste-env / lste / lste-teleop / sappo-lste`；停止顺序 `lste-teleop → sappo-lste → lste → lste-env`；metrics 优雅停机 `timeout -k 1 3 rosnode kill /lste_navigation_metrics` 轮询 30×0.1s。

---

## 5. 数值速查（高频魔法数字）

| 量 | 值 | 含义 |
| --- | --- | --- |
| 机器人半径 / footprint | 0.30 m | 全局/局部/TEB 一致 |
| TEB min_obstacle_dist | 0.22 m | 障碍最小距离 |
| 净空 clearance（规划） | 0.62 m | 0.30 + 0.22 + SLAM 余量 |
| TEB xy 到达容差 | 0.50 m | 前端点完成包络 |
| 原地无进展（批实验） | 50 s / 0.35 m | 失败判定 |
| 目标跟随最小分数 / 框 | 0.20 / 0.01 | 进入跟随的门槛 |
| 目标完成最小分数 / 框 / 帧 | 0.40 / 0.05 / 3 帧 | 完成证据 |
| 上下文观望 sweep | 0,30,-30,60,-60° | 每 2s 换角度，10s 弃 |
| 检测置信阈值（WeDetect） | 0.20 | score_threshold |
| governor 参数 | 0.60 / 0.45 / 0.50 | decel / gap / max_speed |
| 前向-only | true | 倒退钳 0 |
| TEB 角速度死区 | 0.02 rad | mux 滤波 |
| Navfn 默认容差 | 0.20 m | 终点容差 |

---

*来源分层：代码内硬编码、ROS 参数默认值、`scripts/config/pipeline_defaults.yaml`（生产生效值）。以 YAML 为准，代码默认值仅作 fallback。*

# 未知环境在线 SLAM + Navfn + TEB 导航

## 目的

本文记录把固定目标实验中验证过的 `SLAM + Navfn + TEB` 控制链路接入正常
LSTE 未知环境任务的实现和运行证据。当前默认配置为 `LSTE_CONTROLLER: teb`，
TEB 是主力控制器；SA-PPO 仍保留为可热切换的对照控制器。

## 数据流

```text
LSTE task / WeDetect
        -> lste_goal_manager.py
        -> /lste/goal_intent (mission ownership)
        -> /lste/final_goal
        -> lste_teb_goal_bridge.py
        -> /move_base/goal (MoveBaseAction)
        -> move_base: NavfnROS + TebLocalPlannerROS
        -> /lste/cmd_vel/teb
        -> lste_cmd_vel_mux
        -> /cmd_vel -> Gazebo Pro3
```

探索期间，Goal Manager 的唯一全局目标发布者仍然是
`/lste/final_goal`。在没有可用视觉目标时，它使用
`/lste/global_frontier_goal` 从 gmapping 的在线 `/map` 中选择连通且有安全间隙的
frontier endpoint；视觉目标出现后，短距离的 target-follow goal 优先于 frontier。

### 各组件职责

- `online_slam_frontier.launch` 启动 gmapping 和在线 frontier 节点。frontier 节点只
  发布 `/lste/global_frontier_goal`，不会绕过 Goal Manager 发布最终目标。
- `teb_navigation.launch` 启动 `move_base`，全局规划器为 `navfn/NavfnROS`，局部
  规划器为 `teb_local_planner/TebLocalPlannerROS`。TEB 控制频率是 `20 Hz`，最大线
  速度由 `TEB_MAX_LINEAR_SPEED` 控制，当前为 `0.50 m/s`。
- 全局 Navfn 采用事件驱动生命周期（`planner_frequency=0`）：新导航事务、路线失效
  或 recovery 才会重建全局路径。在线 SLAM 仍持续更新 `/map`，局部 costmap 和 TEB
  仍以高频处理障碍物。固定 1 Hz 重算同一个路径会反复重置 TEB 的 timed elastic
  band，即使目标没有变化也会产生零速脉冲，因此不通过调 PID 或速度阈值来掩盖它。
- 生产路径不再把每个锐角拆成“先到过渡点、再去终点”的两个动作。frontier 节点把
  已经验证连通的路线终点作为唯一 mission endpoint，并把路线切线交给 Navfn/TEB；
  同一个 action 负责转向和前进。这样锐角是全局路径中的几何变化，而不是会插入零
  速度间隙的独立控制器事务。
- `lste_teb_turn_supervisor` 是执行状态适配器，不是第二个路径规划器。对生产
  `frontier_endpoint`，它订阅 `/move_base/NavfnROS/plan`，仅当全局路径首切线落后车头
  超过半圈时执行一次预转向，然后在同一个 action 内释放 TEB 命令；视觉 target 不会
  被它拦截。`frontier_turn_connector` 仍作为旧实验兼容保护：若外部实验显式发布旧
  类型，bridge 会等待 supervisor 的 `turn_completed`，不会因为 move_base 先报告 XY
  `SUCCEEDED` 就提前释放逻辑 action。只有显式关闭 `mission_endpoint_only` 的旧滚动
  地平线实验才会使用普通 `frontier_connector`。
- `lste_teb_goal_bridge.py` 将 odom 坐标系的 `/lste/final_goal` 转成标准
  `MoveBaseAction` goal。`/lste/goal_intent` 是 Goal Manager 在目标之前发布的
  机器可读 ownership 信号。生产配置把一个 MoveBaseAction 视为一个完整执行事务：
  普通 frontier/context 更新在健康 action 期间只合并最新值，不抢占当前轨迹；确认的
  视觉 target 才能按优先级启动受控交接。地图刷新不会自动升级成 action 替换，只有
  控制器切换、任务完成、路线失效或真正无进展恢复才使用显式 cancel。每个 action 的终态只发布一次到
  `/lste/teb_goal_terminal`，执行状态由 action client 回调提供，不再自行解释
  `/move_base/status` 的时间戳。
- 视觉 target 在 Goal Manager 提交段动作时只做一次 `odom -> map` 转换，之后该段的
  action identity 固定在 `map` 中。检测器的后续帧只能更新证据和下一段候选，不能因为
  `map -> odom` 的 SLAM 漂移而改变正在执行的目标。
- target action 有明确的失败终态：如果 feedback 在进度窗口内没有真实位移，或
  `move_base` 返回 `ABORTED/REJECTED`，bridge 发布一次
  `/lste/teb_goal_failure`（事件 `target_route_failed`）并取消该 action。它不会再对
  同一个 target 执行 `cancel -> retry` 循环。Goal Manager 将状态置为
  `TARGET_BLOCKED`，保留视觉证据但把控制权交回 frontier；只有一个 frontier action
  成功产生地图进展后，状态才回到 `TARGET_CANDIDATE`，重新验证目标路线。
- 视觉检测和导航执行之间还有一道路由承诺：Goal Manager 先把相机射线投影成
  一个候选 target approach pose，再调用同一个 `/move_base/NavfnROS/make_plan`
  服务验证当前 `map` 中是否存在全局路径。只有返回非空 plan，target intent 才能
  抢占 frontier action；空 plan 或 TF/服务暂不可用时，目标仍保存在视觉跟踪缓存中，
  当前 map-connected frontier action 继续执行。这样“看到了目标但目标点在墙后”不会
  被执行层解释成反复 cancel/retry。
- `lste_cmd_vel_mux_node.py` 仍是唯一 `/cmd_vel` 发布者。切换控制器不会重启
  Gazebo、SLAM 或检测器。

## 配置与启动

配置入口是 [`pipeline_defaults.yaml`](../../scripts/config/pipeline_defaults.yaml)：

```yaml
GLOBAL_FRONTIER_ENABLED: true
LSTE_CONTROLLER: teb         # 生产默认控制器
LEGACY_GP_FRONTIER_ENABLED: false  # 在线 SLAM frontier 已取代旧 GP frontier
TEB_MAX_LINEAR_SPEED: 0.50
DETECTOR: wedetect-large
# target ray must be connected by the live Navfn planner before it can own TEB
TARGET_ROUTE_VALIDATION: true
TEB_GOAL_FAILURE_TOPIC: /lste/teb_goal_failure
# 连续前沿交接与平顺度参数（详见 20260807_navigation_smoothness_optimization.md）
GLOBAL_FRONTIER_APPROACH_DISTANCE: 1.5
GLOBAL_FRONTIER_EARLY_HANDOFF_RADIUS: 2.00
# launch 层：allow_in_place_replacement=false,
# allow_route_continuation_replacement=true, transform_tolerance=1.0,
# force_reinit_new_goal_dist=5.0, global_plan_viapoint_sep=0.6, weight_viapoint=0.5,
# weight_acc_lim_theta=3.0, obstacle_proximity_upper_bound=0.9
```

验证 TEB 未知环境链路时，在工作区执行：

```bash
./scripts/lifecycle/run_all_tmux.sh
```

这会启动平级的 `lste-env`、`lste` 和 `lste-teleop` 会话。需要做 RL 对照时，重新
启动时显式使用 `LSTE_CONTROLLER=sappo`。

默认 TEB 启动不会创建旧的 `gp_subgoal` 进程或 `rviz_frontier` 窗口，因为在线
SLAM frontier 已提供探索 waypoint。旧链路仍可通过 `LEGACY_GP_FRONTIER_ENABLED: true`
显式恢复。

## 安全与更新策略

在线 frontier 只从已知自由空间开始 BFS，并对障碍物按 `GLOBAL_FRONTIER_CLEARANCE`
膨胀；它发布的 frontier endpoint 仍然位于已知、连通且有安全间隙的区域。Navfn
保留官方默认的 `allow_unknown=true`，把未知栅格当作高代价的最后 fallback。视觉
target 在进入 action bridge 之前还必须通过一次实时 Navfn `make_plan` 检查；如果
目标点落在墙后、膨胀障碍物内，或当前地图尚未连通，Goal Manager 会保留 frontier
任务而不发送这个 target action。目标证据不会丢失，后续地图更新后会重新验证。TEB
的 rolling lidar costmap、footprint 和 `min_obstacle_dist` 仍是实际碰撞约束，因此
这不是把未知空间当作无障碍物。

当前 action 的 feedback、Navfn 路线和 TEB 局部轨迹各自保持自己的生命周期；Goal
Manager 在 action 健康期间只保留最新的 mission intent。生产配置关闭通用的
`allow_in_place_replacement`，但开启更窄的
`allow_route_continuation_replacement`：只有同一张已验证 frontier 路线的相邻段，才会在
`early_handoff_distance`（2.0 m）prefetch 后，于 `maybe_segment_handoff_locked` 的
安全窗口内用 actionlib 原生新目标热交接。视觉目标、跨分支跳转和控制器切换仍等待
action 终态，因此不会把不同任务混成一个轨迹。bridge 会读取 TEB 的
`force_reinit_new_goal_dist`（5.0 m）和
`xy_goal_tolerance`，拒绝会重建 timed elastic band 或已经落入终态容差的跳转；这些是
TEB 的结构性边界，不是额外 PID 参数。见
[导航平顺度优化报告](20260807_navigation_smoothness_optimization.md)。

这条事件驱动规划边界和 action bridge 是同一个架构契约：地图观测的变化不会自动
升级成执行事务。只有 frontier 节点确认当前路线失效并发布 `route_invalidated`，或
Goal Manager 产生了更高优先级的 mission intent，才允许 Navfn/TEB 获得新的全局目标。

每次热交接会在 bridge status 和 metrics 中记录 `replacement=true` 以及
`replacement_kind`（生产路线续段通常是 `frontier_route_endpoint`）。这些事件与
`move_base_preemptions` 分开统计：actionlib 会把原生新目标报告为 PREEMPTED，但只要
没有伴随 stop/zero-velocity 脉冲，它就是同一路线的正常事务交接，而不是失败重试。
当前关键参数为：

```yaml
GLOBAL_FRONTIER_CLEARANCE: 0.52
GLOBAL_FRONTIER_FALLBACK_CLEARANCE: 0.30  # boundary detection only
GLOBAL_FRONTIER_APPROACH_DISTANCE: 1.0
GLOBAL_FRONTIER_MIN_PATH_DISTANCE: 1.2
GLOBAL_FRONTIER_LOOKAHEAD_DISTANCE: 4.0  # 兼容旧启动接口，终点模式不再截断路径
GLOBAL_FRONTIER_PERIOD: 1.0
GLOBAL_FRONTIER_UPDATE_RADIUS: 0.90
GLOBAL_FRONTIER_JUMP_DISTANCE: 2.0
```

Goal Manager 会把 frontier 更新转发给 TEB bridge，但不再用独立墙钟取消正在执行的
action。TEB bridge 会合并普通地图刷新；只有 action 反馈在 `progress_timeout` 内没有
真实进展时才 handoff。确认的视觉目标属于更高优先级意图，但必须先通过 Navfn
路由承诺才可以接管。`/lste/goal_arbitration` 会记录
`target_route_accepted`、`target_route_rejected`、`target_route_deferred`、
`target_route_held`、`target_route_failed` 和 `target_route_released`，用于把“感知
意图不可执行”、目标动作失败、地图进展后的重新授权与 TEB 局部避障分开统计。每个
target intent 还带有单调递增的 `target_epoch`；bridge 用它区分新证据和旧目标重放。
普通 frontier 不使用视觉目标的 retry 路径。
普通 frontier 不再依赖原地替换路径；它由 `move_base` 正常返回 `SUCCEEDED`，再由
终态回调提交下一个已验证 endpoint。bridge 对回调内再次 `send_goal` 做了异步调度，
避免 actionlib 仍处于 `DONE` 转换阶段时发生竞态。终态话题解决了“action 已完成但旧
frontier 仍被保留”的问题。

## 目标完成状态

目标完成不是由单个低质量检测框或状态机停留时间决定的。Goal Manager 要求：

1. 检测框分数和至少一个归一化尺寸达到近距离阈值；
2. 来自不同检测消息的近距离证据达到 `TARGET_DONE_MIN_FRESH_HITS`；
3. 证据保持时间达到 `TARGET_DONE_MIN_HOLD_TIME`。

确认后发生以下动作：

```text
/lste/task_done: True
        -> mux 发布一次零速度并锁定输出
        -> TEB bridge 取消当前 move_base action
        -> frontier 节点暂停，不再发布探索 waypoint
        -> Goal Manager 停止发布新的 final goal
```

收到新任务后，任务节点发布 `task_done=false`，各节点重新进入工作状态。

## 2026-08-06 未知环境验证

本次从干净状态执行了：

```text
./scripts/lifecycle/stop_all_tmux.sh
catkin_make --pkg lste_topo_access
LSTE_CONTROLLER=teb ./scripts/lifecycle/run_all_tmux.sh
```

运行配置：

| 项目 | 值 |
| --- | --- |
| world | `worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world` |
| task | `yellow_cup` |
| 初始位姿 | `(3.4, 0.0, 0.1, yaw=0)` |
| detector | WeDetect-Large TensorRT FP16 |
| global planner | `navfn/NavfnROS` |
| local planner | `TebLocalPlannerROS` |
| TEB 速度上限 | `0.50 m/s` |

运行证据：

- TEB 连续完成多个 frontier action，日志多次出现 `status=3 Goal reached`；
- 目标在视觉跟随阶段被确认，随后出现 `close target confirmed hits=3`；
- Goal Manager 在仿真时间约 `2253.279` 发布 `task_done`；
- TEB bridge 记录 `cancelled move_base: reason=task_done`；
- frontier 节点记录 `Global frontier paused: task_done=true`；
- 健康检查保持 `HEALTHY`，`task_done=true`；
- 完成后的后续观察中没有新的 final goal、TEB dispatch 或重复 observation hold 日志，
  车辆保持零速。

这证明当前 `LSTE brain -> online SLAM frontier -> Navfn -> TEB -> vehicle` 闭环在该
未知 Gazebo 场景中能够探索、发现黄色杯子并稳定停车。它证明的是 TEB 导航基线和
目标管理链路，不应表述为原始 SA-PPO policy 已经完成同一任务。

## 全局规划器 A/B 记录

2026-08-06 曾把 `base_global_planner` 临时替换为 ROS 自带的
`global_planner/GlobalPlanner`（quadratic potential + `GradientPath`）。第一段
路径的几何确实更连续，但在线 SLAM 更新到下一个 frontier 后连续返回
`Failed to get a plan`，触发四轮 recovery，车辆停在原地。由于这个结果降低了
探索可靠性，生产 launch 已恢复 `navfn/NavfnROS`；metrics 仍保留
`global_planner_plan` 字段，便于以后实现带 Navfn fallback 的组合插件后做严格对照。

## 已知限制

- frontier 选择依赖 gmapping 的在线地图和当前 lidar 可见范围；地图尚未覆盖的区域
  不会直接作为 Navfn 的通路。
- WeDetect 的视觉推理速度和小目标可见性仍决定目标跟随的最终耗时。
- `LSTE_CONTROLLER: teb` 是默认值，RL、TEB 和 teleop 可通过 mux 做对照；SA-PPO
  仍保留为可选实验，不再承担生产主链路。

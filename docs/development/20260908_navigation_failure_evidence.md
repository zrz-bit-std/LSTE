# Navigation Failure Evidence (2026-09-08)

## 结论

实验周期长不是单一的 TEB 参数问题，而是启动成本、未知空间探索成本和失败后没有立即结束语义事务叠加的结果。最后一轮可审计的 Level 4 运行是
`runtime/office_building_benchmark/logs/20260907_202912/`：它在约 16 秒开始第一条路线，约 495 秒发生目标路线 `ABORTED`，随后相同的 `reinspect_target` 相关身份被反复报告到约 776 秒。该轮没有完成任务。

一次失败以前需要人工同时翻阅 GoalManager、GlobalFrontier、TEB bridge 和 metrics 日志。旧日志有计数和单点事件，但没有一个对象能回答“这一次失败发生时机器人在哪里、目标/route 谁拥有控制权、Navfn 是否有路、TEB 选了什么、局部是否有障碍、失败后发生了什么”。

## 快速局部复现

完整办公楼矩阵默认需要视频作为论文证据；定位单个失败时不应为了一段录屏等待完整实验窗口。使用
`--stop-on-failure --no-video`（`--skip-video` 是同义参数）可以在首次关闭的
failure snapshot 后立即清理，并跳过屏幕录制。该覆盖只允许用于诊断模式，不会改变
`experiment_matrix.yaml` 的正式配置；`plan`、`*_experiment_record.json` 和每个 trial
条目都会写入 `video_override=disabled_by_cli`。例如：

```bash
python3 scripts/tests/office_building/run_experiment_suite.py \
  --phase topology_smoke --method place_portal_workitem \
  --level level_2 --stop-on-failure --no-video --execute
```

这个选项只减少视频进程和收尾开销，不能消除 Gazebo、SLAM 和导航节点的首次启动成本；
不能把它误认为复用地图或复用 ROS 进程。

## 已实现的证据链

`lste_navigation_metrics` 仍然是只读 observer，不会发布速度、取消 action 或改变规划策略。新增
`navigation_metrics_failure_evidence.py` 后，每个 metrics 进程在原有时间戳目录中写两个文件：

```text
YYYYMMDD_HHMMSS_navigation_metrics.log
YYYYMMDD_HHMMSS_failure_evidence.log
```

一次 episode 使用统一 ID：`<run_timestamp>-F0001`。`failure_started`、相关事件、`failure_snapshot_ready` 和专用快照日志都带这个 ID。环形缓冲默认保存失败前 12 秒、失败后 5 秒，采样周期 0.20 秒；只保存摘要，不保存原始图像或整张 costmap，因此不会把调试功能变成磁盘瓶颈。

关闭的 episode 还会在同一个运行目录落一个独立 JSON 现场文件：
`<run_timestamp>_failure_0001.json`。`failure_snapshot_ready` 和 benchmark
summary 都包含 `artifact_path`，因此定位一个失败只需读取这一个文件，不必从
数十万字的 metrics 行中手工截取 JSON。文件通过临时文件原子替换写入；写盘失败
只会留下日志告警，不会影响导航状态机。

现场 JSON 同时复制本次运行的不可变 `run_context`（world、初始位姿、任务定义、检测器、
控制器/TEB 配置和 Git 状态）。分析器优先读取这些独立文件，并只用
`failure_evidence.log` 补齐缺失 episode；失败定位不依赖 metrics 主日志大小。

每个快照样本包含：

- ROS 时间、墙钟时间、odom pose、当前 goal/frame、goal source、mission transaction；
- bridge/frontier/goal/move_base 的最近 route context（route、Place、Portal、WorkItem、target track）；
- 实际 `/cmd_vel`、TEB supervisor 输出、TEB planner 输出和 mux 状态；
- scan 最小距离、前方最小距离和障碍阈值；
- Navfn/global planner/TEB global/local plan 的点数、长度、endpoint，以及 Navfn remaining estimate；
- 以机器人为中心的 15x15 global/local costmap 窗口（frame、origin、resolution、中心 cell 和占用值）；
- TEB selected trajectory 摘要、move_base feedback/status、recovery 状态；
- state、hold、TF transform failure 计数和控制器状态。

从 2026-09-08 起，样本还包含 `channel_health`。它逐项记录 pose、goal、cmd_vel、
scan、Navfn、TEB feedback、两张 costmap、MoveBase feedback 和 route identity 是否在
失败边界真正可用、来源以及采样年龄。若 watchdog 先于独立 Navfn `Path` 回调到达，
metrics 会从同一 route 的 bridge/frontier 状态中的 `navfn_path_remaining` 或
`path_distance` 生成一个 `derived=true` 的 planner 摘要，并明确标注来源；这不是把
状态摘要冒充原始 Navfn 路径，而是保留回调竞争时已经存在的证据。若两者都不存在，
`channel_health` 会写出 `no_value_at_boundary`，调查者可以区分“没有路”和“观察者
尚未收到 Path”。

运行上下文还会把一次性解析的 `resolved_params` 写入 `run_context` 和 `run_start`，
因此单独打开 failure artifact 就能看到实际控制器、TEB、frontier 和 evidence 参数，
不必依赖 roslaunch 的屏幕输出或另一个参数转储文件。

另外，每个 failure sample 顶层都有紧凑的 `route_identity`，直接包含 route、Place、
Portal、WorkItem/obligation 等可用身份。完整的 `route_context` 仍保留用于回放，
但即使深层状态被 JSON 边界截断，调查者也不需要从 `<max-depth>` 反推失败动作。

## 自动触发和分类

以下事件会立刻开启 episode：

- move_base `ABORTED`、`REJECTED`、`LOST`、`RECALLED`；
- 未被已知 handoff 合同解释的 `PREEMPTED`；
- TEB bridge/target topic 的 route failure；
- GlobalFrontier 的持续 route-unavailable lease wait、execution terminal failure、Portal execution failure；
- GlobalFrontier 明确的 `route_invalidated` 失败原因（`stall`、`local_egress_stall`、
  `active_timeout`、`portal_edge_deadline`、`disconnected` 或 controller/recovery
  failure）；`frontier_observed_at_standoff` 是正常的观察边界，不会误报为失败；
- active route 持续零速度且路径清晰；
- TEB 给出前进命令但 pose 持续没有位移。
- TEB 仍在 `xy_goal_tolerance` 外却只给出反向微速度，而前向模式 mux 明确以
  `forward_only_reverse_clamp` 删除该线速度；这条 command-chain 缺口也会开启
  一个 `controller_output_gap` episode。

分类是保守的“候选解释排序”，不是用单个传感器武断下结论：

```text
planner_no_path       Navfn 空路、显式 no-path 或 TEB 没有可选轨迹
planner_materialization_stall endpoint 已完成，但 GlobalFrontier 选出的 successor 尚未物化
local_obstacle        scan/costmap 表明机器人被局部障碍限制
controller_stall      route 仍 active、路径清晰但没有有效运动
controller_output_gap TEB 只给反向微速度，mux 明确将其裁成 0，且尚未进入终点容差
tf_or_transform       pose 或 goal frame 无法变换
owner_transaction_race 最近发生 priority/handoff/replacement/owner 事件
recovery_exhausted    move_base recovery 已达到序列末端
unknown               当前证据不足，保留候选列表供离线复核
```

快照同时保存初始分类、结束时重新分类、候选列表和上下文时间线，所以后续的 terminal/recovery 回调可以修正“仅凭第一条日志”得到的错误归因。
最终分类会合并触发时和结束时的候选解释，保留置信度更高的证据；因此结束窗口内上下文已经变旧时，不会把触发时明确的 ownership race 退化成 `unknown`。快照额外包含 `classification_at_end`，可以区分触发瞬间和结束时的观察。

`causal_route` 是触发时分类器锁存的 route 身份，独立于后续 active lease。即使
GlobalFrontier 在发送 invalidation 后立即把 `active_route_id` 清为 0，artifact 的
`diagnosis.route_id`、`failure_snapshot_ready.route_id` 和 `causal_route` 仍指向实际
失败 route；嵌套的 graph plan 还会恢复 `graph_action`、WorkItem/Portal obligation
和 portal path。

特别注意 route recovery 的边界：`route_invalidated(reason=stall)` 先创建一个失败
episode；随后 bridge 发出的 `PREEMPTED` 仍按已知 recovery 合同计入
`route_recovery_preemptions`。两者共享同一个运行上下文，但不会再把真实 stall
隐藏在“正常抢占”计数后面。route、reason、目标、距离、进度信号和恢复字段会在
大状态消息被截断时被强制保留。

## 如何查看

每次定向 runner 或 office benchmark trial 结束后，都会自动生成一个合并报告：

```text
<run>_failure_report.json
<run>_failure_report.md
```

报告把启动就绪、实际导航窗口和清理耗时分开，并列出每个 failure ID 的 route、
主因、触发位姿/目标、证据完整性和独立现场文件。手工查看历史运行时使用短命令：

```bash
scripts/bin/navdiag runtime/targeted_navigation/t_junction_small/logs/<timestamp>
scripts/bin/navdiag runtime/office_building_benchmark/logs/<timestamp> --json
```

它只读已有日志，不启动 ROS/Gazebo；因此修改分类器或复核一次失败只需要毫秒到秒级，
不会重复等待整栋楼的探索过程。若报告显示 `evidence=INCOMPLETE`，先查看报告中的
`missing` 字段；若只显示 `truncated fields`，说明是历史 artifact 的序列化上限，
不影响其中已经锁存的 `causal_route` 和 trigger sample。

如果只修改了故障分类或报告逻辑，使用 `scripts/bin/navreplay` 重放固定的
T-junction failure fixture；它不启动 ROS/Gazebo，单次约 20 ms。只有要验证真实
传感器、costmap 或 TEB 行为时，才进入下面的定向 Gazebo runner。

启动一次正常实验后，直接查看本次目录：

```bash
python3 scripts/tools/analyze_navigation_metrics.py \
  runtime/office_building_benchmark/logs/<timestamp>/<timestamp>_navigation_metrics.log \
  --failures
```

输出会列出 failure ID、触发事件、分类、置信度、是否在进程结束前完成后窗口以及相关事件数；如果存在启动门禁故障，还会列出 `startup_failure` 的 ID、原因和 artifact 路径。需要完整指标时运行原命令；结果中的 `failure_evidence_log` 指向专用快照文件。
汇总中的 `technical_diagnosis` 是触发现场的因果 reducer 结果，和不可变 artifact
中当时生成的 `classification` 分开保留；升级分类规则后，历史现场也不会丢失更
具体的 route/command 诊断。
benchmark 的标准 `<run>_summary.json` 也会内嵌同一 failure ID 的精简触发/结束
现场，因此不需要再手工拼接多个节点日志。

启动阶段也有独立的证据边界。runner 先等待 metrics 文件，再等待
`navigation_readiness(ready=true)` 或首条真实 `route_command`；它不会把
“tmux 窗口已创建”当成导航已就绪。若在矩阵配置的
`startup_readiness_timeout_seconds`（当前为 45 秒）内仍没有就绪事件，会立即
停止本轮并写入：

```text
<run>_startup_failure.json
<run>_startup_failure.log
```

JSON 中保留最后的 readiness 状态、Navfn/costmap 门禁线索、最后一条 pose/goal/TEB/
mux/costmap sample、最近 bridge/mission 事件、metrics/global-frontier 日志路径和
实验配置；成功越过门禁时 lifecycle log 写入 `event=navigation_ready`，失败时写入
`event=startup_failure`。这类记录的 `outcome` 是 `startup_failure`，不会被计入导航
成功率，也不会消耗完整的 420/600 秒 trial timeout。

诊断实验可以在首个失败后自动结束，不必把剩余的 600 秒 trial timeout 全部耗掉：

```bash
python3 scripts/tests/office_building/run_experiment_suite.py \
  --phase topology_smoke --method place_portal_workitem \
  --execute --stop-on-failure
```

这个开关只改变 runner 的停止条件，不改变正式矩阵默认行为。它会等待
`failure_snapshot_ready`（包括失败前/后的环形窗口）后再调用统一 stop，记录
`failure_stop_id`、分类和实际 trial 时长。正式对比仍应使用完整 terminal event，
避免把“快速定位实验”混入成功率分母。

运行记录还会直接保存 `failure_stop_artifact_path`、失败 route、触发 pose/goal
和 command chain，脚本输出本身就能指出应打开哪个现场文件。

实验上限到达时还会写入：

```text
<run>_trial_end_diagnostic.json
```

这不是人为制造的 failure episode，也不会占用 metrics 的 `-F####` 序列；它有独立的
`<run>-T0001` 终止身份，表示“实验边界到达时的最后现场”。当 metrics 尚未满足持续
失败条件时，旧 runner 只留下 `outcome=timeout`，无法知道车是仍在前进、等待图物化，
还是已经出现无有效控制指令。新 artifact 会保存最后 16 条 bounded sample、最近的
route/bridge/goal 生命周期事件、active route 身份、Navfn/TEB plan 是否存在、move_base
状态、命令是否到达执行边界，以及距离进展、位移、零速度样本数和路线切换序列。
报告直接显示 `classification`（例如 `active_route_timeout`、`planner_no_path`、
`controller_output_gap` 或 `terminal_settle`）和 `progress.interpretation`。
其中这些字段仍是实验边界诊断，不等同于 metrics 的正式 `failure_snapshot`；这样正式
benchmark 的失败率不会被 runner 推断污染，但每次超时仍可直接定位。
该路径使用最多 1 MiB 的 metrics 尾部，避免最后几条大型 GlobalFrontier 事件把最后
sample 挤出诊断窗口。

对于 T-junction 这类局部物理故障，推荐使用专用的短入口：

```bash
scripts/tests/targeted_navigation/run_t_junction.sh --max-seconds 50
```

它会自动创建 `runtime/targeted_navigation/t_junction_small/logs/<timestamp>/`，
在同一个 launch 中启动 Gazebo、SLAM、GlobalFrontier、Navfn/TEB、mux 和 metrics，
通过 `readiness.sh` 直接确认必需节点和话题已经注册（`/map`、全局/局部
costmap、odom、scan、`/move_base`、GlobalFrontier 和 metrics），不再依赖
`roslaunch` 可能延迟刷新的 `map_ready=True costmap_ready=True` 控制台文本；
门禁通过后才开始计导航窗口，首个 `failure_snapshot_ready` 出现后自动结束。
可用 `--no-stop-on-failure` 保留完整窗口，或用 `--no-pre-route-turn` 做端点对齐
消融。每次运行都会额外生成 `<timestamp>_lifecycle.log` 和
`<timestamp>_summary.log`，不需要再手工启动 metrics。

如果启动门禁超时或 roslaunch 提前退出，runner 会在同一目录写入
`<timestamp>_startup_failure.json`。其中的 `checks` 保存失败边界的缺失节点/话题、
已注册清单、metrics 是否写入 `run_start` 以及 launcher 是否仍存活；`logs` 保存
launcher、metrics、lifecycle 的有限尾部。这样启动失败也有独立的 `failure_id` 和
可机器读取的现场，不会出现 lifecycle 写“超时”但 metrics 又显示半次成功动作的
矛盾记录。

若导航窗口到期但没有达到 metrics 的正式 failure trigger，定向 runner 还会写
`<timestamp>_trial_end_diagnostic.json`。它复用同一个 bounded reducer，保存最后的
route/bridge/MoveBase 状态、Navfn/TEB plan、命令链、样本窗口和最近事件，并将
`stall_candidate` 与正式 failure episode 分开；`outcome=timeout`、`stop_reason`
只描述实验边界，不会污染失败率。
其中还会检查“终点容差内的 terminal settle”和明确的
`forward_only_reverse_clamp`。前者标记为 `terminal_settle`，后者标记为
`controller_output_gap_candidate`，避免把正常到达误报成 stall，也避免把
“TEB 只给反向微速度、mux 删除线速度”隐藏在普通 timeout 里。

这条入口固定短诊断窗口（pre=4 s、post=1.5 s、采样=0.20 s），而正式办公楼
benchmark 仍使用矩阵中的证据窗口和 timeout。2026-09-08 的自动化运行
`20260908_075026` 从 runner 启动到清理约 76 s：readiness 约 29 s，导航约 45 s，
失败快照 post-window 1.5 s；失败 artifact 包含 route、goal、触发 pose、Navfn/TEB
计划、scan/costmap 窗口、mux command chain 和 local-egress recovery。该运行没有
残留 ROS/Gazebo 进程。

修复后的短跑 `20260908_092504` 在约 8 s 通过图状态门禁、执行 4 s 导航窗口并在
约 15 s 内清理完成；lifecycle 记录了 3 次缺失集合变化，最终没有启动失败 artifact，
且进程检查确认没有残留 Gazebo/ROS 节点。此前相同条件会因为等待缓冲 heartbeat
文本而耗尽整个 15/45 s readiness 上限。

在办公楼 `20260908_094702` 的 20 s smoke 中，最后样本距离终点 `0.4566 m`，
TEB 的终点容差为 `0.50 m`，且 bridge 正在等待 terminal。新的 reducer 将该现场
判为 `terminal_settle`；同一份旧规则会误报 `stall_candidate`。这类边界修正让
后续统计只把真正超出终点容差、且命令链确实断开的现场送入故障复现队列。

如果问题已经缩小到某一个终点，不要重新跑完整 T-junction。使用近端控制器入口：

```bash
scripts/tests/targeted_navigation/run_t_junction_endpoint.sh \
  --start 3.0 0.2 --goal 4.60 0.15 --max-seconds 20
```

它只启动 Gazebo、SLAM、move_base/TEB、mux 和 metrics，不启动 GlobalFrontier 或
GoalManager；机器人从给定近端位姿开始，直接把目标送进 TEB。最近一次验证从
`goal_published` 到 `failure_snapshot_ready` 约 11 秒，现场文件包含 TEB selected
velocity、原始 planner 命令、mux 输入/输出、局部/全局 costmap、scan 和终点距离。
`--allow-reverse` 可用于区分“TEB 本身没有前进意图”和“前向模式 mux 裁剪反向意图”。
该入口的日志位于 `runtime/targeted_navigation/t_junction_endpoint/logs/<timestamp>/`，
不会污染正式 office benchmark 的成功率分母。

## 失败目标的即时释放协议

2026-09-08 对历史 Level 4 运行暴露的长等待已经按消息所有权修复，而不是增加一个更短的
超时。旧运行在 `move_base=ABORTED` 后仍保留 target priority=2；GoalManager 发出的
frontier replan 因此被桥接层当成低优先级，且 replan 还带着
`target_room_claim=true, target_room_claim_release=false`。这会让同一个失败事务一直被
hold，历史样本从约 495 s 拖到约 776 s。

当前失败边界遵循下面的顺序：

```text
controller/Navfn failure
  -> bridge records target transaction/epoch/track tombstone
  -> bridge clears persistent target request and installed-target state
  -> bridge cancels a still-active persistent MoveBase lease
  -> bridge publishes target_route_failed
  -> GoalManager requests target_room_claim_release=true
  -> GlobalFrontier releases the claim (release wins over stale claim=true)
  -> a newer frontier route_id becomes the next controller owner
```

桥接层保留 `target_lease_tombstone_transaction_id`、epoch 和 track identity。旧消息即使
因为独立的 latched topic 而晚到，或携带比失败消息更大的 transport transaction，只要仍
属于同一个失败 track，就不能重新把 priority 改回 2；只有新的语义 track 或明确的新任务
才会重新获得 target ownership。C++ `StreamingNavfnPlanner` 同样保存失败序号，并把
`KIND_CLEAR` 转发给 `PersistentTebLocalPlanner`，所以 Python bridge、Navfn 和 TEB 不会各自
保留一份旧 target。

该协议的最小回归在
`src/lste_topo_access/test/test_persistent_target_plan_failure_release.py`，覆盖 persistent
plan failure、仍存活 action 的取消、frontier 接管、同一失败 track 的更大 transaction
重放，以及 legacy `/lste/goal_intent` 路径。C++ 清除传播的静态契约在
`test_persistent_planner_route_identity.py`。运行：

```bash
python3 -m pytest -q src/lste_topo_access/test/test_persistent_target_plan_failure_release.py \
  src/lste_topo_access/test/test_persistent_planner_route_identity.py
```

这类状态机修改不需要重新探索 Level 4。先用上述回归和 `scripts/bin/navreplay` 验证消息
顺序，再用 `run_t_junction_endpoint.sh` 或 `run_experiment_suite.py --stop-on-failure
--no-video` 做 20～90 秒定向现场；只有在失败现场不再出现 stale target hold 后，才重新
运行完整 benchmark。

### 局部障碍表示 A/B

近端终点的同一起点/目标（`3.0,0.2 -> 4.60,0.15`）做了三种配置：

| 配置 | 结果 | 证据 |
| --- | --- | --- |
| DBSMCCH 多边形 + 原始 costmap | 约 11 秒出现 `controller_output_gap`，实际位姿约 `3.91,0.10` | `runtime/targeted_navigation/t_junction_endpoint/logs/20260908_085404/` |
| DBSMCCH 多边形、关闭原始 costmap | 在 20 秒窗口内仍发生局部停滞，未形成稳定 terminal | `runtime/targeted_navigation/t_junction_endpoint/logs/20260908_090257/` |
| 原始 rolling costmap（转换器 disabled） | 约 12 秒 `SUCCEEDED` | `runtime/targeted_navigation/t_junction_endpoint/logs/20260908_090025/` |

完整 T-junction 短跑也重复了这个边界：原始 costmap 版本在 77 秒导航窗口完成
13/13 个动作、0 个 abort 和 0 个 failure episode（`20260908_090506`）；转换器版本
此前在同一类路线于约 33 秒进入 stall（`20260908_083529`）。因此当前生产默认已切到
`TEB_COSTMAP_CONVERTER_PLUGIN: disabled`。DBSMCCH 仍可通过
`T_JUNCTION_TEB_CONVERTER=costmap_converter::CostmapToPolygonsDBSMCCH` 或
runner 的默认诊断配置重现，作为论文中的 obstacle-representation 消融；这不是把
转换器从代码库删除，而是用实验结果选择更可靠的执行路径。

office benchmark runner 还记录 `phase_timings`，拆分 launcher、readiness、navigation、failure evidence、
cleanup 和 summary 的耗时。按 `Ctrl-C` 时，第一次中断会被记录为
`outcome=interrupted`，随后在屏蔽二次中断的窗口内执行统一 stop；因此不会因为人工
终止而丢失清理结果或把残留 Gazebo/tmux 带入下一轮实验。批量 runner 也会在该 trial
后停止，不会继续启动后续矩阵条件。

需要只复现一个局部条件时可以进一步过滤 level、seed 和 trial，并缩短诊断窗口：

```bash
python3 scripts/tests/office_building/run_experiment_suite.py \
  --phase topology_smoke --method place_portal_workitem \
  --level level_2 --seed 20260905 --trial-id 1 \
  --profile office_entry \
  --execute --stop-on-failure \
  --failure-pre-window 4 --failure-post-window 1.5
```

如果局部复现没有触发失败，也可以给诊断 trial 一个更短的硬上限，例如追加
`--max-trial-seconds 90`。这些选项只作用于诊断运行；不传时矩阵的正式 timeout
和证据窗口完全不变。

停止实验时，`scripts/bin/stopall` 会先通过 ROS 正常关闭
`/lste_navigation_metrics`，再清理 tmux 和 Gazebo。这样 `on_shutdown` 能写入
`run_stop` 和未完成 episode 的最终快照；若节点本身已经退出，则停止流程继续执行，不会阻塞实验清理。

## 为什么旧周期会拖长

1. 每次从干净进程启动，Gazebo、SLAM、costmap、Navfn、TEB、检测器都要重新等待 ready；这部分是固定启动成本。
   runner 将“launcher 进程组启动上限”（当前 180 秒）与“导航 readiness 上限”（当前
   45 秒）分开。前者防止 shell/节点启动死锁，后者防止节点都注册了却没有可执行
   route 的情况拖进整轮试验；两者都会在当前运行目录留下原因和日志指针。
2. Level 4 从未知区域开始，首次发现目标前必须完成多次 Place/Portal/WorkItem 观察和路线执行；这是探索成本。
3. 旧的失败 target lease 在某些运行中仍保持 `reinspect_target`，而 frontier 又因当前 SLAM snapshot 不能物化同一 durable identity，于是两层互相等待。`20260907_202912` 中同一 route 31 的 unavailable 事件持续约 284 秒就是这个证据。
   持久 Navfn 的 `target_plan_failed` 还可能只清掉请求、没有释放目标 owner，
   使 frontier 在下一次事务到来前继续等待。现在该事件会写入失败证据、设置目标
   的 semantic terminal boundary、释放 target lease，并让下一个 frontier transaction
   无需重启即可接管。
4. 旧版本 GoalManager timer 曾因 `target_close_since is None` 抛出 `TypeError`，导致后续目标完成/调度回调线程退出；当前代码保留空值保护，并有回归测试。

本轮针对性验证把同一条件拆成短 trial：`20260908_052215` 从启动到停止共
91.5 秒（launcher 24.4 秒、导航 60.0 秒、清理 6.9 秒）；`20260908_052413`
在约 97.2 秒触发失败，证据后窗口 1.58 秒，整轮 128.9 秒。两轮都在停止后
没有残留 ROS/Gazebo。后者的独立现场文件为
`runtime/office_building_benchmark/logs/20260908_052413/20260908_052413_failure_0001.json`。
这说明定位实验不必再等待完整 Level 4；先在同一 profile/seed 复现局部边界，
确认原因后再恢复正式长跑。

为减少 persistent stream 在终点附近的重复计算，endpoint-rooted transition BFS
现在按 `(active_route_id, endpoint, grid_shape, resolution, map_origin)` 绑定到当前
route lease。缓存只用于
successor 的拓扑/入口切向评分；每个候选仍用当前 costmap 和 Navfn 认证。普通
SLAM/costmap 更新以及重复的调度通知不会重复跑整张图；只有切换到新 route 或地图
尺寸变化才使缓存失效，并在 `transition_topology_cached` 事件中留下当前结构修订
号。这把长实验中“反复等待相同 successor topology”的成本压缩为每条路线一次
可审计计算，同时不放宽候选的实时可达性检查。

当前 target failure 路径已要求 controller failure 原子释放 target lease、清除旧 target goal，并以 `target_room_claim_release=True` 请求从当前 pose 重新规划。对应测试确认不会把失败目标重新当作有效 reinspection，也不会把 release request 错发成普通 target claim。

另外，最近的 Level 1 运行发现了一条独立的重入链：`Place 8 -> Place 9 -> Place 10 -> Place 8`。这不是 controller failure，而是 `branch_first` 在本地 WorkItem 尚未完成时提前跨 Portal。现在 graph planner 只允许 branch-first 绕过没有 durable viewpoint geometry 的候选；只要 WorkItem 仍有可重建的 `normal_xy`，就先完成当前 Place 的本地工作；如果该工作暂时不在当前 frontier，则返回 `current_place_obligation_not_materialized` 并等待重建。这是物理身份和证据状态的离散规则，不是再增加一个 dwell/timeout 参数，因此能减少无意义的离开再回访。

## 验证范围

新增 ROS-free 回归覆盖：

- failure ID 在一个 episode 内唯一，相关 trigger 不创建第二个 ID；
- planner/local obstacle/owner race/controller stall 的分类；
- pre-failure 样本进入快照，结束后 episode 可关闭；
- metrics sibling 安装路径包含新模块；
- target completion 在 close timestamp 被并发清空时仍安全完成。

短 ROS 集成 smoke（`runtime/navigation_failure_smoke_v2/logs/20260908_010837/`）还验证了两条边界：单个 `frontier_route_unavailable` 后紧接着 `route_command` 不会创建 failure episode；明确的 `NAVFN_NO_PATH` target failure 创建 `20260908_010837-F0001`，在约 1.1 秒后写出快照并分类为 `planner_no_path`。

这套证据链解决的是“失败后定位信息不足”和“失败是否持续”的可观测性问题；它不会把一轮 target-entry 隔离实验冒充完整 Level 4 成功，也不会在没有新实验数据时宣称导航已经完成。

`failure_evidence.log` 的每个快照还包含 `diagnosis`：它把 TEB selected
feedback、原始 planner 命令、turn supervisor 输出、mux 输入/输出和最终
`/cmd_vel` 放在同一条 command chain 中，并给出首个发生分歧的层。这样
“feedback 仍非零但 planner 已经因 endpoint terminal 输出零”会明确显示为
`planner_materialization_stall`/`stale_teb_feedback`，而不是笼统的
`controller_stall`。

从 2026-09-08 起，ring sample 还保留完整的 `teb_turn_supervisor` 状态、
`turn_phase`、目标 yaw 和 yaw error；`diagnosis.execution_phase` 会显示失败时
是否处于合法的原地转向。GlobalFrontier 同时把监督器的 route kind 纳入 watchdog
边界：`frontier_endpoint`、`portal_transition` 和 `local_egress` 的 `TURNING`
阶段计入物理进展，转向结束后才重新启用平移停滞判断。这样不会把“正在转向”误报
成 controller stall，也不会用增加全局 timeout 的方式掩盖问题。

前向模式下，TEB 用微小负线速度配合角速度表示 turn entry。mux 现在只在这个明确
的 turn-entry 语义存在时放行反向角速度，普通小幅左右反转仍按原规则抑制。恢复到
历史安全锚点的 `local_egress` 也进入同一个原子转向契约，先对齐恢复目标朝向再
执行前进，从而避免“目标在身后、负线速度被裁掉、角速度又被抖动过滤”的二次失败。

普通 frontier observation endpoint 现在沿当前 BFS 真实路径退回已有的
`frontier_approach_distance`，再交给 costmap/Navfn/TEB 认证。它不会把未知边界或墙面
本身当作 docking 点；这个 endpoint/standoff 几何改变只作用于候选编译，不改变
Place/Portal/WorkItem 身份，也不改变生产 TEB 的速度参数。

同日的 T-junction 对比给出了一个可复现的架构结论：关闭端点对齐时，route 3
会在 Navfn 急转处把 TEB 的近零线速度误认为普通平移停滞；打开
`teb_frontier_pre_route_alignment` 后，日志明确出现
`turn_started(turn_phase=pre_route_alignment)` 和 `turn_completed`，该阶段不再
触发 watchdog。剩余失败发生在另一条东侧终点 route，触发现场显示前方 scan
仍有约 1.1 m、TEB trajectory valid、距离终点约 0.66 m，随后约 1.5 s 内进入
`local_egress`。这说明端点转向误判已经被隔离，但终点 standoff/局部可执行性仍是
下一步需要单独验证的控制问题；不能把这次短实验当成 T-junction 或 Level 4 已完成。

桥接层的 endpoint 认证也遵循同一原则：frontier/egress 坐标是地图派生提示，
允许在线 SLAM 让它换到另一个 free cell；只有当报告点与当前 active Navfn
plan endpoint 重合时才接受这个漂移。Portal 和视觉 target 仍按原 action
identity 严格匹配，避免把“附近的另一个目标”误当成完成。

## 20260908 短重放结果

同一 Level 1 起点的短物理重放记录在
`runtime/office_building_benchmark/logs/20260908_014523/`。运行约 107 秒，进入
3 个物理 Place，未发生 Place 重入、MoveBase abort、碰撞或失败 WorkItem 重复
派发。4 次 `PREEMPTED` 都有明确的 frontier/Portal 路线恢复生命周期，记录为
`route_recovery_preemptions=4`；`unexpected_preemptions=0`，failure episode=0。
此前同一类路线切换会被误计为 3 个 ownership failure episode。

这次重放证明的是故障证据和 lease 生命周期的短场景行为，不是完整 Level 4
任务成功。完整 benchmark 仍需在固定版本、固定配置和自动超时下单独运行。

## 20260908 诊断运行结果

`runtime/office_building_benchmark/logs/20260908_045204/` 是一次短 smoke 的启动
验证：run directory 建立后约 24 秒出现首条真实 route，`persistent_stream` 的
TEB/StreamingNavfn 已进入 ACTIVE。随后手动停止以节省完整探索时间，因此该目录
不是成功/失败率样本；它只证明新的 readiness 边界没有再复现
`navfn_startup_probe_empty` 启动死锁。

`runtime/office_building_benchmark/logs/20260908_031619/` 是一次 Level 2
诊断运行。它捕获了一个稳定的 `planner_materialization_stall`：route 4 的
endpoint terminal 已收到，桥接层已释放 controller lease，但图层选出的 durable
WorkItem 还没有出现在当前 frontier 快照中。相同 route 的 14 次 unavailable
事件只生成一个 `20260908_031619-F0001`，后续窗口内的 pose、Navfn、TEB、mux
和 route identity 都在同一快照中。

## 20260908 异步 Navfn/后继路线修复

`20260908_052413` 现场曾把一个已经重建出的 WorkItem 67 错误判成
`selected_graph_obligation_not_executable_in_snapshot`。复核事件顺序后，候选实际
已经通过 GraphRoute gate 和 transaction；唯一未完成的步骤是精确 Navfn endpoint
验证。当时 `choose_valid_frontier` 返回了“验证 pending”，但 successor 选择层把
这个状态当成图物化失败，释放了刚结束的 controller route。

现在有两条明确的边界：

- Navfn worker 写入结果后主动请求 `navfn_validation_completed` wake；事件驱动模式
  不再依赖无界的“结果缓存非空”轮询，避免静态地图上的忙循环。
- `frontier_validation_pending` 只保留当前 graph plan lease，发布
  `reason=navfn_validation_pending`，不释放 controller；结果到达后重试同一个
  WorkItem/Portal identity。只有候选确实不能物化时，才进入
  `selected_graph_obligation_not_executable_in_snapshot`。

对应的 ROS-free 回归覆盖 WorkItem 67 的 gate/transaction、Navfn pending 不释放
route lease，以及 worker 完成事件唤醒。一次新的 Level 2 短运行记录在
`runtime/office_building_benchmark/logs/20260908_060750/`：启动约 24 秒，首条路线
约 11.7 秒仿真时间发出，没有启动门禁失败或 failure snapshot；诊断上限为 100 秒，
到达上限后正常清理。该运行证明的是后继调度链已恢复，不是完整 Level 4 成功率样本。

另一次 `--stop-on-failure` 运行（`20260908_030550`）在首个已关闭快照后约
5 秒自动结束，实际 trial 时长约 229 秒，而不是等待 420 秒 timeout。它的
post-window 保留了 route 9 的触发现场，随后 route 10 的恢复命令不会覆盖
触发时的 command-chain 证据。

## 当前实现核对（2026-09-08）

- 诊断 runner 现在可以用 `--method/--level/--seed/--trial-id` 只启动一个
  条件；`--profile` 仅在 `--stop-on-failure` 下可用，适合把机器人放到失败点
  附近做局部复现。
- `persistent_stream` 的启动门禁只要求 map/TF/costmap 连通；StreamingNavfn
  在首条真实 mission 上完成 Navfn 证明，不再用“尚未收到 mission 的空 probe”
  阻塞整个图规划。Navfn 异步空响应、服务错误或暂时不可用会重新排队一次
  event-driven wake，而不是永久停在 `waiting_for_navfn_probe`。
- runner 的 `startup_readiness_timeout_seconds` 与 `startup_timeout_seconds` 是两个
  不同合同：前者只覆盖 metrics 出现后的导航就绪等待，后者覆盖 launcher 进程组。
  启动死锁会生成 `<run>_startup_failure.json/.log`，并在汇总行中保留
  `startup_failure_id/reason/artifact_path`。
- T-junction 定向 runner 的门禁使用 ROS graph inventory，不读取 roslaunch 的
  heartbeat 文本；每次缺失集合变化会写入 lifecycle，超时会生成
  `targeted_navigation_startup_failure` artifact。该短路径的 readiness 判定和
  cleanup 已有 shell/JSON 回归覆盖。
- 失败窗口可以在诊断运行中显式覆盖（例如 pre=4 s、post=1.5 s），正式矩阵
  不传这些选项时仍使用原配置。
- `failure_snapshot_ready` 会携带 `artifact_path`、route、触发 pose/goal、
  command chain 和 metrics 时间；实验记录会原样保存这些字段。
- 884 个 `lste_topo_access` 回归和 8 个 `lste_core` 回归（1 个可选检测测试跳过）
  通过；四个 ROS-free 拓扑重放均在秒级通过。尚未用这组改动宣称 Level 4
  任务成功，完整 benchmark 仍需独立运行并纳入正式结果。

## 本轮快速验证

- `scripts/bin/navreplay` 重放固定 T-junction failure fixture，耗时约 20 ms，
  仍得到 `controller_stall / global_frontier / route=9`。
- `run_t_junction_endpoint.sh --goal 4.90 0.15 --max-seconds 25` 的实际周期为约
  16 s（就绪 8 s、导航 6 s、清理 2 s）。报告
  `runtime/targeted_navigation/t_junction_endpoint/logs/20260908_105446/` 将无图
  route ID 的控制器隔离场景标为 `controller_goal`，现场 pose、goal、Navfn/TEB/mux
  链完整，主因归为 `planner_no_path`。
- `run_t_junction.sh --max-seconds 60` 的一次失败周期约 53 s（就绪 8 s、导航
  43 s、清理 2 s），报告将 `20260908_104808-F0001` 绑定到 route 7；独立 artifact
  路径由报告直接给出。失败快照使用前后窗口，不再等待剩余的 600 s 正式上限。
- 完整回归：`python3 -m pytest -q src/lste_topo_access/test`，`932 passed`。

## 20260908 实验周期与失败收口修复

长周期不是一个单一的 TEB 参数问题，而是三个不同阶段叠加：

1. 启动阶段要拉起 Gazebo、SLAM、costmap、Navfn、TEB 和观测节点；就绪前的
   同名节点/残留进程会让 `move_base` 反复 respawn，原来可能一直耗到 readiness
   超时。
2. 正式 Level 2/4 运行从未知地图开始，路线、门洞和 WorkItem 要逐步认证；这段
   时间只有在研究探索策略时才有意义，不应拿它诊断一个局部终点故障。
3. 失败阶段过去没有统一的终止边界：失败目标可能继续持有 controller lease，图层
   又继续等待旧 transaction 或旧 route，导致额外数分钟的无效等待。

当前的针对性实验流程是：先用 `target_entry` 或 T-junction 小地图把机器人放到
失败点附近，只测试一个 ownership/规划假设；只有局部链路通过后才进入更大地图。
定向入口如下，默认要求完整的目标失败到 frontier 接管链路：

```bash
scripts/tests/targeted_navigation/run_t_junction_persistent_target.sh \
  --max-seconds 45 --startup-seconds 45 \
  --target-goal 100 100 --target-transaction-id 6900
```

该入口不附着 tmux，所有进程共享一个时间戳目录。`persistent_stream` 会显式把
`planner_frequency` 设为 20 Hz；endpoint-action 模式仍保持 0 Hz。原因是持久 action
不会因每个目标更新而重新发送 MoveBase goal，`StreamingNavfnPlanner` 必须被周期性
唤醒，才能在目标进入后快速产生 `target_plan_failed` 或 `target_plan_installed`。
快捷入口会先调用幂等的 `stopall` 清理标准 LSTE 会话；底层 runner 仍可用于需要
调用者自行管理 ROS 实例的 A/B 实验。

这次短实验的实测结果（运行目录在 `runtime/targeted_navigation/t_junction_small/logs/`）：

- 就绪约 8 s；目标失败约 0.05 s；目标 lease 释放约 0.05 s；frontier 接管约
  1.15 s；总周期约 13 s。
- 失败证据包含 `failure_id`、目标 transaction/track、旧 route、新 route、Place/
  WorkItem、Navfn、TEB、mux、scan、pose 和时间窗口；没有残留 ROS/Gazebo 进程。
- `target_route_failed` 且原因为 `persistent_navfn_target_unreachable` 时，报告首要
  分类为 `planner_no_path`。release/preempt 是后续收口动作，不再覆盖真正的规划原因。

修复了两个会造成长等待的协议缺口：

- replan callback 在清除图 route 时同步释放 `DecisionWakeScheduler` 的旧 route
  lease，否则调度器会永久认为旧路线仍占用执行权，后续 frontier wake 全被丢弃。
- target 与 frontier 共用旧 transaction 字段时，失败目标可能留下一个更大的数字，
  使合法的新 frontier 被误判为 stale。现在只有在匹配的 target-failure boundary
  仍处于 latch 状态、且新 frontier `route_id` 严格大于旧 route floor 时才允许这次
  跨域接管；普通旧目标和旧 frontier 仍被拒绝。

启动冲突也单独记录为 `*_startup_failure.json`，其中保留缺失节点、观察到的节点/话题、
launcher/metrics/lifecycle 尾部和启动配置。它与导航 failure snapshot 分开，避免把
“系统没启动好”误报成“TEB 导航失败”。正式 Level 4 仍使用完整 timeout 和视频/覆盖率
指标；本入口只用于秒级定位，不能替代完整 benchmark 结论。

## 20260908 诊断边界修正

Level 2 的一次 120 秒短实验显示，最后窗口可能同时包含两个不同 route：旧的
`frontier_endpoint` 已在目标容差内结束，随后图层启动了新的 `portal_transition`。如果
直接把窗口第一条样本和最后一条样本相减，会把“换了目标”误报成距离倒退。现在
`trial_end_diagnostic.json` 保留兼容的聚合字段，同时增加 `route_segments` 和
`active_route`；报告优先解释当前 route/transaction 的进度，并明确标出窗口跨越了多少
个目标。这样可以区分“当前路线仍在前进”和“同一路线无进展”，不需要重新启动仿真。

失败目标还有一个相反方向的竞态：Navfn 拒绝 target 后，frontier 可能在下一个 metrics
采样前安装 `bootstrap_observation`。失败 artifact 的 `causal_route` 和
`diagnosis_at_trigger` 现在锁存 `target_approach`，后续 frontier route 只保留在结束
现场和事件时间线中，不会覆盖失败动作的身份。事件时间线按契约字段压缩，保留
route/transaction/原因/Place/Portal/WorkItem，而不会把整张 graph 状态递归写入日志。

最后，metrics 的 post-window 可能在 runner 已请求 timeout 清理后才完成。如果清理后
发现了编号的 failure artifact，runner 会把实验记录从 `timeout` 更正为
`failure_snapshot`，并写入 `failure_snapshot_detected_after_cleanup` 生命周期事件，避免
同一次运行的 experiment record 和 failure report 给出互相矛盾的结论。

## 本轮验证（20260908 14:13）

`run_t_junction_persistent_target.sh` 的真实 probe 产生目录
`runtime/targeted_navigation/t_junction_small/logs/20260908_141316/`：启动门禁约 8 s，
目标失败和 lease 释放在首个导航秒内完成，清理约 2 s，总周期约 11 s。报告将现场归类
为 `planner_no_path / high`，锁存 route `1`、`target_approach`、transaction `900` 和
`probe:unreachable-wall`；独立 artifact 约 438 KB，事件时间线没有 `<max-depth>`，且
没有残留 ROS/Gazebo 进程。

同日的 Level 2 topology smoke（`20260908_135436`）在 24.729 s 启动后运行 120.081 s，
最后进入新 `portal_transition` route，车辆仍在移动但没有完成该路线，因此是
`active_route_timeout` 而不是失败快照。新报告同时给出 route-local progress，避免把
route 7 到 route 8 的目标切换误算成单一路线倒退。当前 ROS-free 与模块回归总计
`961 passed`。

`target_entry` 的 30 秒诊断运行 `20260908_142734` 还验证了 profile 隔离：不会启动
`/lste_global_frontier`，readiness 在首条 pose sample 即通过；目标在 Navfn 物化前被
拒绝时，报告标为 `pre_dispatch_planner_rejection`，因此缺少 move_base/TEB feedback
不会再被误报为损坏的失败现场。

## 20260908 快速周期与因果分类复核（15:09--15:24）

本轮针对“启动太慢、失败后仍要等完整窗口、失败结论不够具体”做了两类短实验，而不是再次运行整栋办公楼：

| 场景 | 启动就绪 | 导航窗口 | 清理 | 总周期 | 结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| T-junction 可达 endpoint，`20260908_150905` | 8 s | 14 s | 2 s | 24 s | `SUCCEEDED` 后立即停止 |
| T-junction 不可达 goal，`20260908_152347` | 8 s | 5 s | 2 s | 15 s | 首个 failure snapshot 后立即停止，`planner_no_path/high` |

两次运行都自动生成独立的 lifecycle、metrics、failure-evidence 和合并报告，且结束后没有残留 ROS/Gazebo 进程。不可达目标的现场明确包含：起点
`[3.0, 0.2]`、目标 `[100.0, 100.0]`、MoveBase goal ID、Navfn `poses=0`、TEB
`not_available`、TEB/mux/actuator 全链路零命令、scan 前方距离约 `1.93 m`、
`pre_dispatch_planner_rejection` 和完整的 pre/post sample window。

这次复核修正了一个容易误导调查的分类边界。旧规则看到 watchdog 的
`zero_velocity_stall` 就优先给出 `controller_stall`，即使 Navfn 没有路径、TEB 也从未
产生轨迹。现在分类器先检查是否存在“规划边界”：Navfn 为空、TEB 不是
`trajectory_valid`、selected/planner command 均为零。满足时首要分类为
`planner_no_path/high`，同时保留 `controller_stall/low` 作为“表面现象”的候选；只有
TEB 已有有效轨迹或命令确实到达执行边界时，零速度才会成为高置信度的控制器停滞。
因此报告中的 `classification`、`diagnosis.primary_cause` 和 `layer` 现在一致，不会
出现“顶层 controller stall、因果 diagnosis 却是 planner no path”的矛盾。

为防止在线回调顺序制造假进度，新的 MoveBase action 在安装 Navfn plan 前会清除上一
action 的 feedback pose、feedback frame、`last_feedback_pose_global` 和 TF failure
计数；Navfn plan 可以先到达并保留 endpoint 身份，待新 action feedback 到达后再计算
path progress。对应的 callback-order 回归测试覆盖了“旧 feedback -> 新 action -> 新
plan”和“late feedback projection”两个顺序。

验证命令：

```bash
python3 -m pytest -q src/lste_topo_access/test
# 966 passed
python3 -m pytest -q scripts/lifecycle/test
# 2 passed
bash scripts/bin/navreplay --step-delay 0
# fixed T-junction fixture, no ROS/Gazebo
```

所以目前“周期长”的可控部分已经变成秒级局部 runner；启动的约 8 秒和清理的约 2 秒
仍是拉起/关闭 Gazebo、SLAM、costmap 和 move_base 的物理成本，未知 Level 4 探索本身
仍然需要完整时间。正式 benchmark 不能用这些局部数字替代探索成功率；它们的作用是把
一个局部假设在进入 Level 4 前快速证伪或确认。

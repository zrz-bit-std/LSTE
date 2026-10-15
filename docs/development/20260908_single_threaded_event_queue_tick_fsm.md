# 单线程事件队列与 Tick FSM 重构总结

## 1. 文档信息

- 日期：2026-09-08
- 工作区：当前工作区目录
- 目标：将任务、路线和导航执行相关的异步状态管理收敛为 **Single-Threaded Event Queue + Tick FSM（Gather-Compute-Scatter）**。

## 2. 总体结论

本次没有重写 ROS、Navfn、TEB 或 move_base，也没有引入一个新的大型导航框架。现有 ROS 和导航组件继续负责消息传输、全局规划、局部规划和动作执行；项目新增了一层轻量的生命周期协调器，用来解决本项目特有的事务一致性、状态串行化、超时和 stale event 问题。

核心原则是：

```text
ROS/actionlib callback
        |
        | 只复制消息并构造 Event
        v
thread-safe inbox
        |
        | 固定频率 timer
        v
LifecycleManager.tick()
  Gather -> Compute -> Scatter
        |
        v
规划、动作发布、状态转移和超时处理
```

## 3. 主要改动

### 3.1 生命周期核心

新增 `src/lste_topo_access/scripts/lifecycle_manager.py`，提供：

- `State`：`IDLE`、`DISPATCHED`、`EXECUTION_DONE`、`SYNCING`、`FAILED`、`COMPLETED`。
- `EventType`：任务、地图、costmap、位姿、扫描、Navfn、actionlib、TEB 和 replan 等事件类型。
- 不可变 `Event` 数据结构。
- 基于有界 `queue.Queue` 的线程安全 inbox；默认容量为 1024，避免回调突发无限占用内存。
- 基于 UUID1 规范 60 位时间戳加 64 位身份位的复合全局事务号，数值顺序跟随 UUID 时间而不是 `UUID.int` 的字段布局。
- 旧事务事件立即丢弃。
- 新事务事件在 tick 内接管，避免跨节点消息被提前丢失。
- pose、scan、map、full costmap、TEB feedback 等连续状态按事务和流只保留最新样本。
- costmap delta 不跳过中间 patch；检测到 delta 合并时转为 resync 标记，丢弃不完整缓存并等待下一张 full costmap。
- `tick()` 每次最多处理 `max_events_per_tick` 个事件，之后仍执行 Compute 和 timeout 检查。
- `tick()` 的非重入保护，确保只有一个 FSM 计算线程。
- 状态进入时间和状态超时检查。
- `transition_to()`、`begin_transaction()` 和 `adopt_transaction()` 的 tick ownership 约束。
- 可选的 `compute_handler`，保证规划和动作输出也处于同一个 tick ownership window 内。
- `ClockProvider` 统一生命周期超时、反馈 freshness、route handoff、turn watchdog 和 costmap age；ROS 节点存活时读取 Gazebo `/clock`，未初始化或关闭时才回退到 monotonic。
- 机器人静止且没有显式失效时，`costmap_max_age` 超时不会丢弃仍有效的缓存；位姿发生有效移动或收到 delta resync 标记时才禁止复用并等待新 full costmap。

### 3.2 Global Frontier

相关文件：

- `scripts/global_frontier_lifecycle.py`
- `scripts/global_frontier_runtime_state.py`
- `scripts/global_frontier_planning_runtime.py`
- `scripts/global_frontier_planning_navfn.py`
- `scripts/global_frontier_terminal_replan.py`
- `scripts/global_frontier_ros_interfaces.py`

改动包括：

- 地图、costmap、pose、scan、task、detections、bridge status、terminal 和 replan 等 ROS 回调只入队。
- Navfn 后台线程不再直接写规划状态，结果通过 `NAVFN_RESULT` 事件回到 FSM。
- route terminal 后进入 `EXECUTION_DONE`，主动排入 `SYNC_REQUESTED`，随后进入 `SYNCING`。
- `SYNCING` 优先把仍在 `costmap_max_age` 内的 map/costmap 缓存作为带有状态进入时间的事件重新投递；缓存无效时才等待新的发布，超过配置的同步超时后进入 `FAILED`。
- 非 frontier 的 bridge heartbeat、target terminal 不会错误推进 Global Frontier 的事务号或触发 frontier 失败。
- live runtime 中不再启用旧的 planning lock、proposal gate 和 terminal drain timer；保留的兼容路径只服务于旧测试或旧组合方式。

### 3.3 Goal Manager

相关文件：

- `scripts/goal_manager_lifecycle.py`
- `scripts/goal_manager_input_callbacks.py`
- `scripts/goal_manager_frontier.py`
- `scripts/goal_manager_teb_callbacks.py`
- `scripts/goal_manager_goal_output.py`

改动包括：

- state、detections、scores、task、pose、camera、depth、frontier、controller mode、TEB terminal 等回调统一入队。
- 目标发布和 frontier replan 只在真正建立新生命周期时生成事务号，避免 5 Hz 重复目标刷新导致事务号无意义增长。
- 收到 Global Frontier 的外部事务号后，在 tick 内接管并沿用该事务号发布 goal intent 和 atomic goal command。
- 新任务、真正的目标切换、target terminal observation 和真正的 frontier replan 会建立新的生命周期边界。

### 3.4 TEB Goal Bridge

相关文件：

- `scripts/teb_goal_bridge_lifecycle.py`
- `scripts/teb_goal_bridge_mission_runtime.py`
- `scripts/teb_goal_bridge_ros.py`
- `scripts/teb_goal_bridge_action_terminal.py`
- `scripts/teb_goal_bridge_status.py`

改动包括：

- ROS topic 回调和 actionlib `active`、`feedback`、`done` 回调只入队。
- action dispatch、terminal 处理、handoff 和 mission runtime 在生命周期 tick 中执行。
- bridge status 增加 `lifecycle_transaction_id`。
- frontier terminal contract 增加完整生命周期事务号。
- action health、retry、feedback、costmap、route handoff 和状态年龄使用共享时钟，Gazebo 暂停时不会被宿主机时间推进。

### 3.5 TEB Turn Supervisor

相关文件：

- `scripts/teb_turn_supervisor_lifecycle.py`
- `scripts/teb_turn_supervisor_state.py`
- `scripts/teb_turn_supervisor_turn_lifecycle.py`

改动包括：

- intent、bridge status、goal、Navfn plan、pose、scan、TEB feedback、planner command、mode、task_done 等输入统一入队。
- 转向计算、速度输出、turn release 和状态发布均在 timer tick 的 Compute 阶段执行。
- supervisor status 增加 `lifecycle_transaction_id`。
- scan/planner command freshness、连续性窗口和 settle 窗口统一使用共享时钟。

## 4. 事务号传播

事务号通过以下边界传播：

- Global Frontier route command/status JSON。
- Goal Manager goal intent 和 atomic goal command JSON。
- TEB bridge status JSON。
- TEB turn supervisor status JSON。
- `FrontierExecutionTerminal.msg` 的 `lifecycle_transaction_id` 字段。

事务号使用复合 Python 大整数进行比较：高位是 UUID1 的规范 60 位时间戳，低位保留 64 位 UUID 身份作为唯一性 tie-breaker。由于完整事务号可能超过 ROS `uint64`，terminal message 中使用十进制字符串保存，避免截断或序列化溢出；进入 Python FSM 后再转换为整数比较。

## 5. 兼容与安装

ROS 节点入口仍保持薄层组合方式：

- `lste_global_frontier_node.py`
- `lste_goal_manager.py`
- `lste_teb_goal_bridge.py`
- `lste_teb_turn_supervisor.py`

`src/lste_topo_access/CMakeLists.txt` 已安装生命周期模块、`ClockProvider` 及其依赖模块，确保 devel-space 和 install-space 的启动行为一致。

## 6. 回归测试与验证

新增测试：

- `test/test_lifecycle_manager.py`
- `test/test_lifecycle_integration_layout.py`
- `test/test_global_frontier_lifecycle_sync.py`
- `test/test_lifecycle_failure_cleanup.py`
- `test/test_navfn_async_validation.py`（背压补充回归）
- `test/test_clock_provider_sim_time.py`
- `test/test_stationary_costmap_cache.py`

已执行并通过：

```bash
python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
# Ran 1005 tests ... OK

python3 -m py_compile src/lste_topo_access/scripts/*.py
catkin_make --pkg lste_topo_access -j2
catkin_make install -j2
git diff --check
```

额外静态审计结果：四个生命周期入口共检查 66 个 ingress callback，全部只入队；四个 timer 均调用 `LifecycleManager.tick()`，并在同一个 tick ownership window 内执行 Compute handler。红队回归覆盖 UUID 时间回绕、连续样本合并、delta resync、tick 预算、因果终端事件保留、Navfn 队列背压、延迟事务抑制、失败事务重置、route lease/action/turn 清理、仿真时钟暂停和静止状态下的缓存同步。install-space 中六个生命周期/时钟辅助模块均存在，terminal contract 也通过 ROS message 序列化验证。

真实 Gazebo smoke 使用 `lste_core/lab_with_pro3.launch`、`use_sim_time:=true` 和 `worlds/topo_test/lab_building.world`：暂停约 1.2 秒时生命周期保持 `DISPATCHED` 且 `timed_out=false`；恢复物理约 1.2 秒后仿真时间推进，0.5 秒生命周期超时转为 `FAILED`。本次 smoke 结束后已恢复 Gazebo 的暂停状态。

## 7. 红队审查进度

本轮按时间、背压、外部黑盒、分布式竞态和恢复退化五个维度进行“先红测、后加固”。本节只保留每个维度的当前结论，后续发现会更新同一行，不追加流水账。

| 维度 | 已证实风险与当前加固 | 证据状态 |
| --- | --- | --- |
| 时间与时钟 | UUID1 时间回拨由事务高水位处理；生命周期、缓存年龄、TEB action freshness、Turn Supervisor watchdog 统一走可注入 `ClockProvider`，ROS 节点存活时读取 `/clock`，ROS 未初始化/关闭时才 fallback 到 monotonic。Gazebo pause smoke 中 `paused_sim_delta=0`，宿主等待不触发 timeout；恢复后按仿真时间超时。 | `test_clock_provider_sim_time.py`、真实 Gazebo lifecycle smoke 已转绿 |
| 高频背压 | 连续 sample 不得挤掉因果终端；为因果事件预留队列容量，并分别统计 sample/causal 丢弃。 | `test_causal_event_is_not_evicted...` 已转绿 |
| 外部黑盒 | Navfn 结果投递被背压时转存 bounded result ledger 并释放 pending；delta 丢失则 resync；静止期间即使底层不再发布 full costmap，仍复用有效缓存，避免被动等待无界挂起。 | `test_rejected_lifecycle_result...`、`test_stationary_costmap_cache.py` 已转绿 |
| 分布式竞态 | 同一 inbox 已知更高事务时，低事务事件在 handler 前丢弃；远端 adopt 更新本地高水位。跨主机首次发起事务仍要求时钟同步，消息自身没有可推断的因果序列时不伪造顺序。 | `test_newer_transaction_supersedes...` 已转绿 |
| 恢复退化 | timeout/FAILED 保留一个 tick 的可观测失败，下一 tick 建立新事务；Global route lease、TEB action 和 turn state 在 FAILED 转移边界清理。 | `test_failed_timeout_resets...`、`test_lifecycle_failure_cleanup.py` 已转绿 |

## 8. 当前边界与后续工作

本次验证覆盖单元测试、结构测试、Python 编译、catkin 构建、install-space 安装、消息序列化，以及真实 Gazebo `/clock` pause/unpause lifecycle smoke。尚未在真实 Gazebo 中执行完整 SLAM/导航实验，因此以下内容仍应作为后续验证项：

1. 启动完整导航 launch，确认四个节点的 ROS callback 线程只产生队列事件。
2. 注入 route terminal、旧 terminal、target terminal 和超时场景，检查事务号及状态转移日志。
3. 验证真实 map/costmap 更新以及静止期间的缓存复用在 `SYNCING` 状态下能够完成同步并回到 `IDLE`。
4. 使用固定目标实验流程验证失败证据与生命周期事件在同一 run directory 中关联。
5. 在多机时钟明显倾斜、ROS `/clock` 重置和长期 Navfn service 不可用条件下运行完整导航短 smoke，确认外部时钟同步、retry 和 costmap 保鲜观测符合部署假设。

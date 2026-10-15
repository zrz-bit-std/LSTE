# 当前导航目标、系统状态与主要卡点

> 文档日期：2026-09-09  
> 适读对象：有移动机器人导航经验，但不了解本项目内部架构的人  
> 文档性质：当前状态说明和问题定位，不是项目完成声明

## 1. 先说结论

系统已经从“多个节点各自记录日志、失败后很难知道谁拥有路线”推进到了“路线有身份、失败有统一现场文件、部分生命周期竞态已经修复”。

但是，机器人还没有稳定完成“从未知办公楼开始探索，找到目标并接近目标”的完整闭环。最近的主要卡点也不是 TEB 速度参数，而是：

> 图层选出的下一条路线，有时没有被当前地图和代价地图物化成真正可执行的几何路线；系统随后继续保留这份旧路线意图，重复等待同一个不可执行的路线。

这会造成三个结果：

1. 机器人停止前进，但系统不一定马上把这次情况视为终止失败；
2. 同一个 Portal/route obligation 被反复重新检查；
3. 实验时间从几十秒膨胀到几分钟甚至更久，失败原因被大量重复日志淹没。

## 2. 我们最终要完成什么

目标是让机器人在固定、可复现的办公楼 benchmark 中完成以下任务：

```text
未知地图开始
  -> 在线 SLAM 建图
  -> frontier 发现未探索区域
  -> Place/Portal 图记录空间关系
  -> 图层决定下一项探索义务
  -> Navfn 验证全局可达性
  -> TEB 执行局部运动
  -> 检测器发现并确认目标
  -> 机器人接近目标并结束任务
```

正式验收条件如下。这里的“完成”必须由可复核的运行记录证明，不能由某一次看起来正常的短片段推断。

| 阶段或指标 | 验收要求 |
| --- | --- |
| `target_entry` | 3/3 完成 |
| Level 1 | 3/3 完成 |
| Level 2 | 至少 2/3 完成，不能错误锁定干扰物 |
| Level 3 | 至少 2/3 完成，必须实际经历死路、遮挡或路线恢复 |
| Level 4 | 固定种子连续 3 次完成，并在至少 3 个不同布局上达到 4/5 |
| 启动就绪 | 不超过 45 秒 |
| Level 4 导航窗口 | 单次不超过 900 秒 |
| 失败后释放 | terminal 或 route failure 后，旧 owner 在 10 秒内释放 |
| 运行安全 | 无碰撞、无明显失控 |
| 失败证据 | 每个正式失败都有独立 artifact 和明确分类 |

只有上表全部满足后，才可以停止修改核心架构。之后只能修复可复现缺陷、优化已经定义的指标或增加独立 holdout 场景。

## 3. Level 代表什么

- `target_entry`：不考察整栋楼探索，只验证机器人能否从近端位置合理接近目标。
- Level 1：办公楼骨架和基础房间，主要验证在线 SLAM、frontier、Navfn、TEB 能否串起来。
- Level 2：加入相似物体和语义干扰，主要验证目标识别、目标锁定和路线切换。
- Level 3：加入死路、遮挡、窄门或路线恢复，主要验证失败后能否选择新的有效路线。
- Level 4：从未知区域开始探索整个复杂办公楼，要求重复运行稳定，不能只在一个预设路线或一个特殊房间中成功。

当前最重要的差别是：短实验可以证明某个局部机制，但不能证明 Level 4 已完成。

## 4. 系统如何分工

可以把系统理解成几层。它们不是五个互相独立的程序，而是一条有所有权和事务边界的流水线。

### 4.1 感知和地图层

- 激光、里程计和在线 SLAM 产生当前地图。
- 目标检测器提供目标候选，但目标检测本身不应该直接控制机器人速度。
- frontier 是“已知区域和未知区域之间的边界”，表示还有哪里值得观察。

地图是不断变化的。机器人走动或 SLAM 更新后，同一个物理门口可能对应不同的栅格坐标。因此，不能只用 `(x, y)` 判断一个路线是否还是同一个路线。

### 4.2 持久拓扑层

这一层保存比单次地图快照更稳定的身份：

- **Place**：一个房间、走廊段或可辨认的空间区域。
- **Portal**：连接两个空间的门洞、通道或候选入口。
- **WorkItem**：某个 Place 中还没有完成的观察任务，例如某个方向的 frontier 或目标观察点。
- **PortalProbe**：为了确认门洞另一侧或门洞状态而执行的观察任务。

这些对象解决的是“这是同一个物理任务吗”，而不是“现在地图上具体走到哪个栅格”。

### 4.3 图路线层

图路线规划器先决定离散动作，例如：

- 观察当前房间的剩余区域；
- 探查一个 Portal；
- 穿过一个已经有证据支持的 Portal；
- 回到一个已知 Place 处理尚未完成的 WorkItem。

它只决定“下一项义务是什么”，不直接生成速度。选出的义务还必须经过当前地图和 costmap 的几何物化。

### 4.4 Navfn、TEB 和控制层

- Navfn 判断从当前位姿到候选终点有没有全局路径。
- TEB 在局部 costmap 中生成连续运动轨迹。
- controller/mux 决定哪一路速度命令最终到达机器人。
- bridge 负责把上层 mission/route 变成 MoveBase action，并处理 terminal、取消和释放。

因此，一条路线要真正执行，至少要同时满足：

```text
图层义务仍然有效
  + 当前地图能找到对应几何候选
  + Navfn 可达
  + TEB 能产生有效轨迹
  + bridge/owner 仍然拥有这条路线
  + mux 没有阻断最终速度
```

### 4.5 失败证据层

`lste_navigation_metrics` 是只读观察者。它保留失败前后的有限时间窗口，并将一次失败写成统一 ID，例如：

```text
20260909_072714-F0001
```

独立 JSON artifact 中会保存 pose、goal、route、Portal/WorkItem、Navfn/TEB、scan、costmap、cmd_vel、mux、TF、feedback 和 recovery 摘要，便于离线分析，不需要重新跑整栋楼。

## 5. 当前已经证明了什么

### 5.1 已完成或有直接证据的部分

1. 全局节点不再把普通 bridge 状态直接当作远端生命周期事务导入，减少了普通状态触发全局 `FAILED` 的竞态。
2. bridge terminal 现在需要匹配已经观测到的 dispatch contract，包括 route、action generation、transaction 和 route kind。
3. 旧运行中出现过“route command 后先变成 FAILED，稍后才 dispatch”的现象；修复后的隔离运行中没有再观察到这类提前失败。
4. 失败现场已经可以产生独立 artifact。最近运行目录中有 `F0001`、`F0002`、`F0003`，并且包含分类、诊断和事件时间线。
5. ROS/Gazebo 持久运行和 warm slice 已经存在。独立 runtime 观测中，graph 计划已经携带真实 `map_epoch`，不必每次都为一个局部问题重新等待完整启动过程。
6. 物化失败现在有明确的 `WAITING_MATERIALIZATION -> NEXT_ACTION` 边界；bridge 侧增加了 terminal 后 10 秒无新 route/ack 的 lease watchdog。
7. 静态检查和一次实际 runtime smoke 已通过；本轮没有反复执行单元测试，也没有修改测试来制造通过结果。
8. 第二次独立 Gazebo 运行中，`route_id=5`、`transaction_id=5`、`map_epoch=15` 在持久 endpoint terminal 后启动 watchdog；停止 Global Frontier 使下游不再 ack，10 秒仿真时间后成功产生 `route_lease_watchdog_expired`，并记录 owner/route intent 清理完成。

### 5.2 尚未证明的部分

以下事项目前都不能声称已经完成：

- `target_entry` 3/3；
- Level 1、2、3 的正式重复通过率；
- Level 4 固定种子连续 3 次完成；
- 三个不同布局达到 4/5；
- 启动就绪始终不超过 45 秒；
- 所有类型的 terminal/route failure 都在 10 秒内释放 owner（目前只实跑验证了持久 frontier endpoint 的无 ack 路径）；
- 正式运行无碰撞、无明显失控；
- 当前系统能在目标丢失、死路和路线恢复后稳定结束任务。

最近的 warm slice 结果是 `timeout`，不是成功。它证明了短窗口和日志链路可以工作，但没有证明机器人完成了任务。

## 6. 最近一次主要卡点：route 9

证据目录：

```text
runtime/office_building_benchmark/logs/20260909_072714/
```

### 6.1 发生了什么

route 9 的目标是 `(1.55, 2.85)`。机器人先到达了这个 frontier 的观察距离，随后上层图规划器认为应该继续处理 Portal 4，动作被表示为 `cross_portal`。

简化后的事件顺序如下：

```text
route 9 到达目标观察边界
  -> 图层选择 Portal 4 / cross_portal
  -> 当前地图没有可直接物化的 Portal 4 几何路线
  -> route owner 被释放，但 graph plan 仍被保留
  -> 下一轮仍检查 Portal 4
  -> 再次报告“当前快照没有这个 durable identity”
  -> 持续无进展后生成 F0001
```

F0001 的现场信息显示：

- route：9；
- graph action：`cross_portal`；
- obligation：Portal 4；
- 当前与目标距离约 3.27 m；
- 前方扫描距离约 4.09 m，不像是机器人正顶着近距离障碍；
- TEB 仍有 trajectory feedback，但 planner/mux 最终速度为 0；
- 失败分类为 `planner_no_path` 候选，详细诊断同时指出了 stale TEB feedback；
- 现场中 graph transaction 的 `map_epoch` 是 `null`。

这里的 `planner_no_path` 不是说已经证明 Navfn 永远没有路，而是说“图层选中的义务没有在当前快照中变成可执行路线”。`stale_teb_feedback` 是失败现场在控制反馈层观察到的另一个现象。两者可以同时存在，不能只看其中一个标签就断言应该调 TEB 参数。

### 6.2 已确认的架构原因与本轮修复

历史代码在候选收集的两个阶段使用了不带快照 epoch 的 `graph_prepare(None)`，导致实际 snapshot 已经有 epoch 34、35、36 等版本，但 transaction 记录成 `map_epoch=null`。后续系统无法判断这份 graph plan 是在哪个地图版本上产生的。

本轮已修复这条断点：

- [global_frontier_selection_planner.py](../../src/lste_topo_access/scripts/global_frontier_selection_planner.py#L163) 的两个规划阶段都显式传入当前 snapshot 的 `map_epoch`；
- [global_frontier_planning_selection.py](../../src/lste_topo_access/scripts/global_frontier_planning_selection.py#L300) 的 fallback materialization 也显式传入 epoch；
- [global_frontier_graph_route_adapter.py](../../src/lste_topo_access/scripts/global_frontier_graph_route_adapter.py#L670) 在 transaction 和 status 中保存该 epoch；
- 同一 graph obligation 连续物化失败时，adapter 记录负证据、清除旧 lease，并进入 `NEXT_ACTION`，planner 在当前 epoch 排除已经证明不可物化的 Portal/Probe/WorkItem；地图 epoch 变化后才重新允许尝试。

因此，`map_epoch=null` 是历史运行的证据，不是当前实现的预期状态。仍需在正式 benchmark runner 中复核所有 route 类型都能带出完整 epoch。

### 6.3 为什么这不是简单调参问题

如果把速度、加速度或 TEB 容差改小，最多只能改变机器人到达某个点的方式，不能解决以下事实：

- 图层选择的是 Portal 4；
- 当前快照没有可交给 Navfn/TEB 的对应候选；
- route owner 已经释放；
- graph plan 却没有按地图版本失效或重新规划。

这属于“离散路线意图”和“当前几何执行条件”之间的生命周期断裂。应该先修复 transaction、snapshot epoch 和 replan boundary，再判断是否存在真正的 TEB 或 costmap 问题。

## 7. 为什么实验会变长

### 7.1 启动成本

冷启动要拉起 ROS master、Gazebo、机器人支持节点、SLAM、costmap、Navfn、TEB、检测器、Goal Manager 和 metrics。持久运行已经降低了这部分成本；最近 warm slice 约 12 秒，说明启动复用方向是有效的。

但要注意：

```text
复用 ROS/Gazebo 进程 != 自动结束每个实验窗口
```

最近一次运行的短 slice 已经结束，但底层持久进程没有被实验编排器及时停止，后来继续积累 route 和失败事件，最长到约 8519 秒。这是实验生命周期控制问题，不应该被误认为机器人完成了一个超长 Level 4 任务。

### 7.2 未知区域本来就需要移动

Level 4 从未知地图开始，机器人必须逐步观察走廊、门洞和房间。每个 frontier 可能需要：

- 转向；
- 等待地图更新；
- Navfn 验证；
- TEB 局部执行；
- 到达观察距离；
- 重新发现新的 frontier 或 Portal。

这些是任务本身的必要时间，不能简单删掉，否则短实验就不能代表真实问题。

### 7.3 当前最浪费时间的部分：失败后循环等待

最近日志中，GlobalFrontier 的单次 planning cycle 通常约 1.5～2.5 秒，偶尔更长。单次计算并不一定严重，真正的问题是：

```text
没有可执行候选
  -> 保留旧 graph obligation
  -> 下一轮重复检查
  -> 再次没有候选
  -> 继续等待
```

如果没有明确的“同一地图 epoch 下只尝试一次”“跨 epoch 重新验证”“连续失败后释放并转入下一个可执行义务”的边界，这个循环就可以持续几分钟甚至更久。

## 8. 当前问题优先级

### P0：graph plan 没有携带可靠的 snapshot epoch（本轮已修复，待正式回归）

graph transaction 现在明确记录产生它的地图版本；短 runtime smoke 已观察到非空 epoch。还需要在正式 runner 中确认 route、Portal probe 和 target 相关路径的一致性。

### P0：不可物化路线没有明确的失效和重规划边界（本轮已实现，待重复验证）

现在需要验证并保持以下区分：

- 当前快照暂时没有候选，继续等待；
- 当前 epoch 已确认该 obligation 不可物化，记录负证据并换下一个义务；
- terminal 或 route failure 已经发生，旧 owner 必须在 10 秒内释放；
- 同一个 graph obligation 不能在没有新证据时无限重新发布。

短 runtime smoke 已出现 5 次物化负证据和 5 次 invalidation，并继续产生新的 `route_command`；这证明了短链路能够前进，但不等同于 Level 4 已完成。

### P0：实验 slice 和持久 runtime 的生命周期没有完全分离（runner 已支持，仍需实跑）

实验可以复用已经就绪的 ROS/Gazebo，但每个 slice 必须有自己的开始、结束和清理事件。达到两分钟上限后，应该结束本次 slice，而不是让同一个后台进程继续把结果混入下一次分析。

### P1：正式验收证据仍然不足

在 P0 问题解决前，直接跑 Level 4 只会重复产生长日志，不能有效回答系统是否变好。应先用短 warm slice 验证路线生命周期，再逐级恢复 Level 1 到 Level 4。

## 9. 下一步应该验证什么

下一次修改不应从调参数开始，而应只验证下面这条架构链：

```text
graph_route_plan_selected(map_epoch=N)
  -> candidate materialized / wait
  -> 如果当前 epoch 不可执行，写出明确负证据
  -> 清理旧 route owner
  -> 在新的地图 epoch 重新选择或换下一个 obligation
  -> 不再无限重复同一个 Portal/route

对持久执行的终端租约，还必须满足：

```text
persistent_*_terminal(transaction_id=T, map_epoch=N)
  -> route_lease_watchdog_armed(timeout=10s)
  -> 有 successor ack：watchdog_cancelled
  -> 无 successor ack：watchdog_expired
     -> owner_revoked=true
     -> route_intent_cleared=true
```
```

验证方式应保持短小：

1. 先做静态检查和已有的离线 failure artifact 分析；
2. 启动一次持久环境，只运行一个不超过 2 分钟的 warm slice；
3. 检查 `map_epoch`、route identity、owner release 时间和 graph replan 事件；
4. 只有这条链稳定后，才进入更高 Level 的实验；
5. 在没有新的证据前，不修改 TEB 速度、加速度、容差等边缘参数。

本轮独立 runtime 观测已经验证了 graph invalidation 能换到后续 route，也验证了无 ack 时 watchdog 能清理 bridge owner；这些运行没有通过正式 office benchmark runner 生成完整 trial record，因此只能算架构 smoke，不能算正式 benchmark 通过。

## 10. 术语表

| 术语 | 简单解释 |
| --- | --- |
| frontier | 已知地图和未知地图的边界，代表可能值得继续观察的地方 |
| Place | 房间或走廊等稳定的空间身份 |
| Portal | 连接两个空间的门洞或通道身份 |
| WorkItem | 某个空间中尚未完成的观察任务 |
| route | 当前要执行的一段几何路线及其生命周期身份 |
| owner / lease | 当前哪个模块有权持有、取消或替换 route |
| graph plan | 图层决定的离散动作，例如观察、探查或穿过 Portal |
| materialization | 把离散 graph plan 转换成当前地图中的具体目标点和路线 |
| snapshot | 某一时刻的地图、costmap、位姿和候选集合 |
| map epoch | 地图/拓扑快照的版本号，用于判断证据是否过期 |
| terminal | MoveBase 或 bridge 对当前 action 的最终结果 |
| failure artifact | 一次失败的独立 JSON 现场文件 |

## 11. 相关资料

- [20260908_navigation_failure_evidence.md](20260908_navigation_failure_evidence.md)：失败现场、分类器和 artifact 格式。
- [20260907_targeted_navigation_experiments.md](20260907_targeted_navigation_experiments.md)：短实验阶梯和晋级原则。
- [20260906_graph_route_planner.md](../research/20260906_graph_route_planner.md)：Place/Portal/WorkItem 图规划设计。
- [20260827_office_building_benchmark_plan.md](../testing/20260827_office_building_benchmark_plan.md)：办公楼场景和正式验收计划。
- [当前运行的 F0001 artifact](../../runtime/office_building_benchmark/logs/20260909_072714/20260909_072714_failure_0001.json)：route 9 的独立失败现场。

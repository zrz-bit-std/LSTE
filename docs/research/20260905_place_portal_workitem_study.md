# 未知办公楼语义导航：Place-Portal-WorkItem 研究方案

> 版本：2026-09-05
>
> 状态：方法与实验基础设施已开始实现；**尚未完成完整 benchmark，也没有论文结论**。

## 1. 可验证的研究问题

LSTE 负责把任务、语言和开放词汇感知转换为“寻找什么”的 `global_goal`。本研究不替换这个大脑，也不把 TEB 当成待反复调参的探索器。问题限定为：

> 在没有预建地图的办公楼中，在线 SLAM 的局部 frontier 会随观测和回环更新而移动、分裂或合并。如何让执行器记住**真实去过的场所和已完成的观察工作**，同时仍能穿过已完成房间到达新的门洞？

这会导出三个可以被反驳的问题。

| 编号 | 问题 | 可被否定的预测 |
| --- | --- | --- |
| RQ1 | 物理 Place 和记忆化、有向 Portal 是否比端点距离去重更能防止重复进入已完成房间？ | `frontier_distance_dedup` 的物理房间 re-entry 数不高于完整方法 |
| RQ2 | Place 内的 Observation WorkItem lineage 是否能避免 SLAM 前沿后移后再次派发同一观察方向？ | `place_portal` 的 resolved-descendant 或重复 WorkItem 派发数不高于完整方法 |
| RQ3 | Place graph 是否能在禁止重开房间的同时保留必要的 transit？ | 完整方法无法经已完成 Place 抵达一个新 Portal，或 transit 后重新激活当地 WorkItem |
| RQ4 | 上层 LSTE 语义目标与地点图约束是否能共同提升未知目标房间的到达成功率？ | 在相同 world、任务、感知栈和 TEB 下，完整方法没有更高成功率或更低代价 |

RQ1-RQ3 是本阶段的主要贡献边界。RQ4 需要更多任务和感知证据，当前不能宣称成立。

## 2. 方法：小而明确的长期状态

```text
LSTE TaskGoal/global_goal + WeDetect evidence
                    |
                    v
   Goal Manager (任务仲裁，不直接探索)
                    |
                    v
Place graph executive
  Place ---- DirectedPortal ---- Place
    |              |
    +-- ObservationWorkItem
                    |
                    v
online /map -> Navfn -> TEB -> robot
```

任务层首先形成稳定的 `TaskGoal` 合同。当前 ROS 消息仍使用已有的 `task_id`，
Goal Manager 同时根据任务的语义字段生成确定性的 `task_version`，并以
`mission_id`（默认等于 `task_id`）和版本共同标识一次任务。这样重复的 latched
任务消息不会重置记忆，而复用同一 `task_id` 但改变目标属性或上下文会被识别为新
mission。这个版本身份不改变 ROS 消息格式，也不与单次执行事务的
`transaction_id` 混用。

每次下发给 Navfn/TEB 的坐标还会携带一个稳定的 `goal_context`：
`geometry_frontier`、`place_observation`、`place_observation_work`、
`certified_portal_crossing`、`covered_place_transit` 或 `place_egress`。
它携带 TaskGoal 的 `mission_id/task_version`、Place/WorkItem 身份和（跨门时）
冻结的 odom 门洞证据；坐标本身仍可随在线地图重规划。这样日志能够区分“坐标变
了”“执行事务变了”和“任务换了”，控制器仍只执行一个普通的 Pose goal。

短期 `/map` 是 gmapping 对当前扫描的解释，允许随 SLAM 更新改变；长期状态以 odom/物理坐标保存，不能因为某个 frontier cell 改了就重新命名。

### 2.1 `Place`

`Place` 是一个真实可通行区域的持久身份，不是一个 frontier 点，也不是某一帧高 clearance 栅格连通分量。它拥有到达的观察锚点、状态和本地 WorkItem。起点所在 Place 允许在启动时建立；之后新 Place 只能由通过认证门洞的到达事务建立。

状态不变量：

1. `open` 的 Place 可以完成本地观察；
2. 本地观察完成后变为 `dormant`；
3. `dormant` Place 可以是图搜索路径的 transit 顶点；
4. `dormant` Place 永远不能再次产生本地 exploration endpoint 或 WorkItem；
5. 只有 LSTE 已确认的目标区域声明可以成为独立、显式记录的重入例外。

路线 `stall`、Navfn 断开和 move_base abort 都不是 Place closure 证据。它们只
会结束对应的 ViewpointAttempt，并保留失败原因供诊断；不能用“失败 N 次”把一
个尚有未知边界的 Place 改成 `dormant`。

### 2.2 `DirectedPortal`

一个 Portal 不是“跨分量的任何短路径”。它必须由占据栅格两侧的墙体支撑、可通行 throat 和源/目的侧方向共同认证。执行端点在门平面以外的目的侧；terminal 还必须给出 odom 中从源侧到目的侧的连续有符号距离证据。

这解释了为什么“走进同一个房间的另一个 SLAM 分量”不能被误认为新房间，也解释了为什么在已完成办公室中返回走廊是允许的、回头再探索办公室却不允许。

### 2.3 `ObservationWorkItem`

一个 WorkItem 是 Place 所拥有的 `frontier arc + unknown-side support`，不是一个临时 goal 点。当前地图上的 frontier 后退、更深或断裂时，若未知侧 support 与历史项重叠，它继承原 WorkItem ID。

WorkItem 的语义状态只有 `unresolved` 与 `resolved`。一次 Navfn/TEB 路线对应其下属的 `ViewpointAttempt`，后者独立记录 `active`、`failed`、`blocked` 或 `succeeded`。因此 `stall`、动作 abort 或某个安全视点不可达，只能结束该 Attempt，不能把未知侧观察任务标成完成或永久不可用。选择器会屏蔽同一个 odom 物理视点，改选同一 frontier arc 上的另一个可行视点；只有真实的端点观察/覆盖证据才能把 WorkItem 变为 `resolved`。已 `resolved` 的 descendant 永远不允许 dispatch。

这比“离上一个 goal 至少 N 米”强，因为 N 米无法判断两个不同位置是否是在看同一扇门后面，SLAM correction 也会改变 N 米的参照。

### 2.4 决策顺序

1. 在当前 `open` Place 选择尚未完成的 local WorkItem；
2. 当前 Place 已有观察锚点且本地工作耗尽时，只选择已认证的相邻 Portal；
3. 若路线必须穿过 `dormant` Place，它只能作为 `covered_transit`，不触发第二次 close；
4. Portal crossing 终态提交新的 Place；
5. Navfn/TEB 只负责验证和执行这一个已提交行动，不能重写 Place graph 状态。

实现入口和边界见：

| 模块 | 职责 |
| --- | --- |
| `global_frontier_place_memory*.py` | Place 的物理关联、状态与观察记忆 |
| `global_frontier_portal_*.py` | 门洞认证、跨越证据、到达提交与恢复 |
| `global_frontier_work_items.py` | WorkItem lineage 和生命周期 |
| `global_frontier_selection_context.py` | 每轮地图快照与持久状态的受控结合 |
| `global_frontier_candidate_*.py` | 从可行 frontier 生成、验证和选择动作 |

## 3. 可比较的方法组

方法名是 `GLOBAL_FRONTIER_METHOD` 的固定契约，不是多个参数的隐式组合。所有组固定 world、robot profile、LSTE task、WeDetect、gmapping、Navfn、TEB、速度和运行时日志格式。

| 方法 | 状态能力 | 角色 |
| --- | --- | --- |
| `frontier` | 仅当前地图的 information/path frontier | 普通 frontier baseline |
| `frontier_distance_dedup` | 普通 frontier + 已到达 endpoint 的欧氏半径排除 | 常见工程性 baseline |
| `place_portal` | 持久 Place + 认证 DirectedPortal，无 WorkItem lineage | 消融 WorkItem 的贡献 |
| `place_portal_workitem_legacy_rank` | Place + Portal + WorkItem，保留旧 scalar local selector | selector-only ablation |
| `place_portal_workitem` | Place + Portal + WorkItem | 完整方法 |
| `place_portal_workitem_strict` | Place + Portal + WorkItem + event-Pareto，关闭 branch-first | branch-first ablation |

`frontier` 不继承结构评分、航向偏置、语义 frontier hint、地点覆盖射线或门洞状态；否则它并非普通 baseline。所有组仍经过同样的 Navfn 可达性和 TEB 碰撞约束，这些是共同执行条件而非本方法的优势。

还应做、但尚未实现为可运行 arm 的消融：移除认证 Portal 限制、移除 `covered_transit`、移除 source-side 门洞观察。它们会有意破坏完整方法不变量，必须先明确如何记录物理 crossing evidence，不能只把一个布尔值关闭后称作公平比较。

## 4. Benchmark 与可复现协议

场景为 `office_building_v1` 的四个静态 level，详细几何和任务契约在 [20260827_office_building_benchmark_plan.md](../testing/20260827_office_building_benchmark_plan.md)。主比较使用 Level 2（语义办公家具）和 Level 3（死路、窄门、替代路线），固定 `primary` 起点和 `yellow_cup` 任务。

研究矩阵在 `scripts/tests/office_building/experiment_matrix.yaml`：

```bash
# 只显示将要运行的比较，不启动 Gazebo。
python3 scripts/tests/office_building/run_experiment_suite.py --phase topology_smoke

# 每个方法一个完全独立的 Level 2 进程重启。
python3 scripts/tests/office_building/run_experiment_suite.py --phase topology_smoke --execute

# Level 2、Level 3 各三次 process-restart repetitions。
python3 scripts/tests/office_building/run_experiment_suite.py --phase topology_main --execute

# 将 timestamped records 汇总为 JSON 与 CSV。
python3 scripts/tests/office_building/aggregate_experiment_results.py \
  --output-dir runtime/office_building_benchmark/results/office_place_portal_workitem_v1
```

`topology_*` phase 使用不存在于场景的 `purple stapler` 任务并以 `frontier_exhausted` 作为终止条件，隔离 RQ1-RQ3 的探索结构；`semantic_main` 才使用 `yellow_cup` 和 `task_done` 终止，评估 RQ4。这样探测器失败不会被误归因为 Place graph。每轮均通过 `GAZEBO_RANDOM_SEED -> gzserver --seed` 使用真实 Gazebo Classic seed，并把 seed 记入 lifecycle log；`trial_id` 只是 phase 内稳定编号。当前静态 world 没有随机布局，结果报告必须如实说明 seed 主要约束模拟器和未来随机传感器/动态物体，不能把重复结果虚构成布局泛化。

每次 trial 都必须：

1. 新建 `runtime/office_building_benchmark/logs/YYYYMMDD_HHMMSS/`；
2. 在 lifecycle log 中记录 method、trial id、world SHA-256、任务、起点、控制器、检测器和 Git revision；
3. 产生 `summary.json` 和 `topology_verification.json`；
4. 产生可回放的视频，并在 record 中写入 `video_status: recorded`、时长与
   字节数；若 X11/Gazebo/编码器不可用，明确记录 `missing` 或 `failed`，该
   trial 不得进入结果表；
5. 不把运行日志、视频或结果 CSV 提交到 Git。

录像从 Gazebo 真正可见的 X11 窗口采集，而非从 ROS topic 重新绘制。录制
停止时只中断 ffmpeg 子进程，等待 MP4 trailer 写完后才移动临时文件，并用
`ffprobe` 验证容器、时长和大小。2026-09-05 已做一次真实 Gazebo 窗口的
短时 smoke：生成了可解析的 4.95 s MP4。它只证明证据管线可用，**不是**
任何方法的 benchmark 结果。

## 5. 指标和证据要求

| 指标 | 当前来源 | 何时可报告 |
| --- | --- | --- |
| physical room re-entry | manifest room bounds + 连续 odom sample | 现在可报告 |
| single-Place effective work time | `work_item_dispatched -> work_item_settled` 的成功 Attempt ROS simulated-time 区间，按 durable Place ID 汇总 | 现在可报告；失败/阻塞 Attempt 单列，不冒充有效观察；未结束项为 `censored` |
| completed WorkItem 重复派发 | 在 `work_item_settled(resolved)` 之后再次 `work_item_dispatched`，并结合 lineage | 现在可报告；同一未完成 WorkItem 的替代视点尝试不算重复 |
| resolved descendant dispatch | 同上，目标为 0 | 现在可报告 |
| path length / completion time | navigation metrics odom / task_done | 现在可报告 |
| target success | task_done + 目标真值几何/感知证据 | 需在每个成功 trial 审核 |
| building coverage | manifest 固定的 architectural free-floor samples 经 `map <- odom` 投影到 `/map`，统计已知 cell | 现在可报告，字段为 `building_truth_fraction`；它不是随 map extent 改变的 `map_known_fraction` |
| collision rate | Pro3 base 的 benchmark-only Gazebo contact sensor，过滤 ground-plane 后按接触开始边沿计数 | 现在可报告，**仅**当 contact stream 已收到消息且 `collision_truth_status=measured` |
| smoothness | 直线路径角能量、forward sign flips、无解释制动 | 现在可报告 |

旧运行没有上述 observer 时，自动汇总仍会明确保留 `building_coverage: null` 或 `collision_events: null`，绝不把缺失资料转换为 0。`building_truth_fraction` 的分母是清单中 enabled room 和 main corridor 的内缩建筑地面，不是所有家具 mesh 的精确导航 footprint；因此它衡量的是可比较的建筑结构地图覆盖，而不是对每个桌椅边界的完美占据精度。达到一次任务成功也不够：至少应有完整矩阵、失败记录、视频和未缺失的关键真值指标。

`max_room_dwell_seconds` 仍然保留，因为它揭示“在一间物理房间滞留太久”的
用户可见失败。但它不能替代有效探索时间：其中混有走到门口、等待规划器、
调头和 transit。只有 `place_portal_workitem` 具有 WorkItem ownership，因而
只有这个方法的 `work_item_execution.status=measured` 可解释为 Place-owned
observation work；其他方法显示 `not_available` 是公平的能力差异，不是零秒。

## 6. 已有运行证据与负结果

`20260905_204815` 是完整方法的 Level 2 诊断运行，而非正式比较 trial：

- 运行期间通过 `covered_transit` 从已覆盖 `region_id=1` 走向新 Portal，并进入 `region_id=3`；
- 没有 resolved WorkItem descendant dispatch，也没有物理房间 re-entry；
- 但只到达两个 manifest 房间，最终 `frontier_exhausted`；
- 目标真值暴露帧存在而 WeDetect 匹配为 0，`task_done=false`。

它支持 RQ3 的一次运行级不变量，不支持 RQ4，更不能作为“系统完成”的证据。

2026-09-06 重新汇总已有的 6 个历史 record 后，方法契约审计保留了
`frontier`、`frontier_distance_dedup` 和 `place_portal` 各 1 个有效 Level 2
样本；3 个旧的 `place_portal_workitem` record 因缺少方法事件被明确排除，详见
`runtime/office_building_benchmark/results/office_place_portal_workitem_v1/`。
这不是完整对比结果，只是说明数据质量门槛已经在结果管线上生效。

## 7. 与已有工作的关系

本方法不是声称发明 frontier，也不是用一个新大模型替换导航。它吸收了下面工作的模块化思想，但关注一个不同的失败模式：在线 SLAM 变化下，物理 Place、门洞 crossing 证据和室内一次性 observation ownership 如何保持一致。

- B. Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, CIRA 1997：普通 frontier baseline 的起点。
- S. Chaplot et al., *Learning to Explore using Active Neural SLAM*, ICLR 2020, [arXiv:2004.05155](https://arxiv.org/abs/2004.05155)：学习/解析地图与分层规划并存的设计，支持保留 Navfn/TEB 的模块边界。
- S. K. Ramakrishnan et al., *Object Goal Navigation using Goal-Oriented Semantic Exploration*, CVPR 2020, [arXiv:2007.00643](https://arxiv.org/abs/2007.00643)：语义任务应在地图上的可达候选之间做选择，而不是直接把检测输出变成运动指令。
- N. Shah et al., *Vision-Language Frontier Maps for Zero-Shot Semantic Navigation*, ICRA 2024, [DOI](https://doi.org/10.1109/ICRA57147.2024.10610712)：未来语义价值层的参考；本阶段不复制其 Habitat 栈，也不把视觉语义当作越过未认证门洞的许可。
- Y. Huang et al., *Leveraging Large Language Models for Visual Target Navigation*, IROS 2023, [DOI](https://doi.org/10.1109/IROS55552.2023.10342512)：语言先验可影响搜索价值，但几何安全与状态可审计性仍应独立。

下一步调研应重点验证两件事：主动 doorway observation 的几何证据模型，以及带场所图的未知环境语义探索 benchmark；采用任何外部方法前，先确认开源实现、输入传感器、许可、计算预算和能否形成公平 arm。

## 8. 局限与停止条件

- Place 的结构解释目前来自 2D lidar/SLAM，不等于完整语义房间分割；
- 单层办公楼和单一杯子任务不足以说明泛化；
- 当前视觉检测失败可以掩盖探索的成功，必须分别报告；
- 当前 coverage truth 是 architectural free-floor samples，并不是 furniture mesh 级
  traversable mask；collision truth 是 benchmark-only base contact sensor。两者都
  已可测量，但报告必须带上它们的定义和 `status=measured`，不能泛化为真实
  机器人碰撞或完整可通行面积；
- 没有完整对比矩阵前，不得声称完整方法优于 baseline 或“可发 paper”。

只有当方法组、world/task/version、日志、自动汇总、视频和上述真值指标同时完整时，才进入论文的结果表、消融图和结论章节。

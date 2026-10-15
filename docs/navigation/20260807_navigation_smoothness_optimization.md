# 导航平顺度优化：目标频繁变动 / 直线摇晃 / 安全距离急刹车

## 一句话结论

在未知环境在线 SLAM + Navfn + TEB 导航链路上，通过三组结构性改动消除了目标
churn 循环、TF 外推导致的空旷处急刹车，并把直线段的航向修正幅度降下来；目标跟随后
动逻辑同时修复，使 LOCKED 的近距目标能可靠完成并停车。修复后单任务从"33 分钟循环
重试不完成"变为"约 2.6 分钟完成、0 重试、平均停车 0.29 s"。

## 基线数据（改前）

运行日志：`runtime/navigation/logs/20260807_123523`（旧 bridge、33 分钟未完成）

| 指标 | 值 | 含义 |
| --- | ---: | --- |
| move_base dispatch | 185 | 大量动作事务 |
| preempt | 165 | 几乎每个动作都被抢占 |
| retry (`retry_move_base_goal`) | 146 | 同一目标反复重发 |
| 平均停车时长 | 15.2 s | 大量长停顿 |
| 最大停车时长 | 274 s | 卡死 |
| task_done | 未完成 | 33 分钟没找到/接近目标 |

## 三个问题的根因

### 1. 目标频繁变动 / 动作 churn

- 旧 bridge 在目标动作非成功终止后，`dispatch_locked` 的 `retry_move_base_goal`
  路径会在 `goal_retry_interval` 后**重发同一个目标**，形成 100+ 次 cancel/retry 循环。
- 现 bridge 已用 `target_route_failed` latch 阻断目标重试；本次又补充确认了这条路径
  不会再触发（0 retries）。

### 2. 直线行驶左右摇晃

- TEB 把 Navfn 的全局路径当作 viapoint 紧贴（`global_plan_viapoint_sep=0.30`、
  `weight_viapoint=1.5`），而 Navfn 路径是逐栅格中心走出来的锯齿线。
- TEB 以 20 Hz 对 10 Hz 更新的 local costmap 反复重优化，每次扫描轻微改变带，
  产生一个控制周期左右的左/右修正。
- 全局路径航向被 `global_plan_overwrite_orientation=true` 强加到带上的每个姿态点，
  把锯齿直接复制成转向命令。

### 3. 频繁安全距离急刹车

两个相互独立的根因：

1. **move_base 顶层 `transform_tolerance` 默认 0.1 s 太小**。本仿真时钟比真实时间快
   ~13 倍，gmapping 离散发布 `map->odom`，计划时间戳比最新 TF 数据超前约 1 ms，
   导致 `transformGlobalPlan` 抛 extrapolation error，TEB 在**空旷处**直接输出 0 速度。
   这是"前方 3 米无障碍却突然刹停"的直接来源。
2. **每个 frontier endpoint 都是一个终止性 move_base action**。如果路线续段也只能
   等待 terminal，机器人到每个终点都会完整停车 -> 原地旋转 -> 再加速。现在生产策略
   关闭通用 `allow_in_place_replacement`，只开启语义更窄的
   `allow_route_continuation_replacement`，仅对同一路线的相邻段做热交接；真正的分支
   跳转仍等待 terminal，避免用平滑性掩盖错误的路线选择。

## 改动清单

### A. 连续前沿交接（消除到点停车）

- `src/lste_topo_access/scripts/lste_global_frontier_node.py`
  - 移除 `not self.mission_endpoint_only` 对 `prefetch_next_frontier` /
    `promote_prefetched_frontier` 的三处门控，使终点模式也能提前选取并 promote 下一分支。
- `src/lste_topo_access/launch/teb_navigation.launch`
  - `allow_in_place_replacement=false`、`allow_route_continuation_replacement=true`：bridge
    只对同一路线的 frontier 相邻段使用 actionlib 原生新目标热切换，目标接管和分支跳转
    仍由完整 action 终态驱动。
  - `force_reinit_new_goal_dist=5.00`：让 5 m 内的分支交接都走 hot-start，超出才重建 band。
- `src/lste_topo_access/launch/online_slam_frontier.launch`
  - `prefetch_distance=2.5`、`early_handoff_distance=2.0`：给 bridge 1.3 m 交接窗
    留出 0.7 m 跑道，让合法延续几乎总能在终点终止前就位。

### B. TF 外推急刹车

- `teb_navigation.launch` 增加顶层 move_base `transform_tolerance=1.0`
  （代价地图里的 `global_costmap/transform_tolerance` 不覆盖这个顶层查找）。
  大容差只延长 `waitForTransform` 等待，不使用过期数据，因此是安全的。

### C. 直线摇晃（TEB 平滑）

- `global_plan_viapoint_sep=0.30 -> 0.60`、`weight_viapoint=1.5 -> 0.5`：不再硬贴
  Navfn 逐格 viapoint。
- `weight_acc_lim_theta=1.5 -> 3.0`：加大角加速度惩罚，抑制左/右单周期修正。
- `weight_optimaltime=0.7 -> 0.6`、`control_look_ahead_poses=4 -> 5`：略放缓
  时间最小化并加长视野。
- 保留 `global_plan_overwrite_orientation=true`。测试过关闭它（让 TEB 自己定航向），
  结果机器人多处停死（最大停车 145 s）、前方转向能量上升，故保留路径切向。

### D. 目标跟随可靠性（LOCKED 后接近并停车）

- `lste_goal_manager.py` `on_dets`：`strong_detection`（score>=0.40 或 box>=0.05）
  不再因盒心在图像中漂移而重置候选 hits——机器人经过目标时盒心每帧可移动 0.3~0.45，
  旧 0.10 空间容差会让目标永远不被确认，机器人反复掠过。
- `lste_goal_manager.py` `maybe_publish_task_done`：close-target 计数不再被
  **单个**小盒帧重置，只在 close 证据超过 `max(detection_age, 2.0)` 秒缺席时复位，
  从而容忍 WeDetect 大模型的帧间闪烁，3 个近距帧即可完成。

### E. 指标与分析工具

- `lste_navigation_metrics.py` 新增：前进转向能量（rad/m，直线摇晃度量）、
  brake 按前方 clearance 分类（clear/near）、`forward_distance_m`。
- `scripts/tools/analyze_navigation_metrics.py` 新增：`--diagnose`（事件轨迹）、
  `--compare A B`（A/B 表）、`retry_dispatches`、`preemption_ratio`、`brakes/m`。

## 验证结果

最终运行：`runtime/navigation/logs/FINAL_20260807_171059/`（完成于约 155 s 仿真）。

### 改前（123523）vs 改后（171059）

| 指标 | 改前 | 改后 | 变化 |
| --- | ---: | ---: | --- |
| dispatch | 185 | 11 | -94% |
| retries | 146 | 0 | -100% |
| preemptions | 165 | 6 | -96% |
| stops | 45 | 7 | -84% |
| 平均停车 | 15.2 s | 0.29 s | -98% |
| 最大停车 | 274 s | 0.50 s | -99.8% |
| angular flips | 114 | 29 | -75% |
| task_done | 未完成 | 完成 | ✓ |

### 目标跟随修复前后（165810 卡死 vs 171059）

- 修复前：目标 `first_seen` 后 12 s 内 LOCKED，但 close 计数被盒闪烁反复清零，
  机器人反复 reacquire，477 s 未完成。
- 修复后：`first_seen -> done` 仅 12 s，target_route_failed=0，任务完成。

### 说明

- `brakes/m` 改后（0.45）高于改前（0.26）：改前任务从未完成，大部分时间在重试/停死；
  改后机器人完整跑完探索 + 目标接近，wall-approach 的渐进减速被计入 brake。绝对 brake
  数量下降（49 -> 26），且空阔处 3 m 无障碍刹停已消失。
- 各次运行探索路径不同（随机化程度有限的 frontier 打分仍会造成路径差异），
  因此对比采用按里程/时间归一化的速率。

## 使用

```bash
./scripts/lifecycle/run_all_tmux.sh
# 结束后分析最新运行
python3 scripts/tools/analyze_navigation_metrics.py runtime/navigation/logs/<latest>
# A/B
python3 scripts/tools/analyze_navigation_metrics.py --compare <runA> <runB>
# 事件轨迹（刹车/翻转/调度）
python3 scripts/tools/analyze_navigation_metrics.py --diagnose <latest>
```

## 遗留

- 靠近墙体的 frontier endpoint 仍有渐进减速的锯齿（obstacle_proximity 限速器与 TEB
  重优化交互）。已通过 `frontier_approach_distance=1.5`、`obstacle_proximity_upper=0.9`
  缓解，但探索本身必须贴近未知边界，完全消除会牺牲覆盖率。
- 直线段航向修正仍存在（flips/m≈0.32，改前≈0.55）。这是 local costmap 10 Hz 更新 +
  TEB 20 Hz 重优化的固有特性；更激进的平滑需在控制器输出加航向治理层，属于
  "掩盖来源"的取舍，未采纳。

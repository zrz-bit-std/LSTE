# Global Frontier 代码布局

## 目的

在线探索节点需要同时处理 ROS 回调、地图栅格、结构地点、候选评分和路线生命周期。为了让修改可以局部完成，代码按职责拆分；模块之间通过明确的数据对象和宿主节点方法交互。

## 模块职责

| 模块 | 负责内容 | 不负责内容 |
| --- | --- | --- |
| `lste_global_frontier_node.py` | ROS 初始化、订阅/发布、地图快照和路线生命周期编排 | 不直接实现规划、候选评分细节 |
| `global_frontier_planning_cycle.py` | 单次规划周期：地图上下文、BFS 路线图、候选选择和 terminal successor | 不创建 ROS 订阅者，不保存独立运行状态 |
| `global_frontier_activation.py` | 将已选候选提交为 active route，并发布选中/重规划状态 | 不计算候选评分，不执行 move_base action |
| `global_frontier_observation.py` | 拓扑组件、观察覆盖、地点状态和跨边界离开事务 | 不创建 ROS 订阅者，不修改 Navfn/TEB 参数 |
| `global_frontier_execution.py` | 活动路线进度、TEB watchdog、局部 egress、成功/失败终止处理 | 不选择新的 frontier，不创建 ROS 订阅者 |
| `global_frontier_planning.py` | 代价地图快照、Navfn 可达性验证、规划启动门控和路线预取 | 不创建 ROS 订阅者，不选择地点评分 |
| `global_frontier_selection.py` | 候选路线验证、地点动作分类、普通前沿选择、portal fallback | 不创建 ROS 订阅者，不保存独立运行状态 |
| `global_frontier_scoring.py` | 评分桶、new/revisit 优先级、语义追踪加权 | 不访问 ROS、地图或机器人状态 |
| `global_frontier_grid.py` | 膨胀、BFS、前沿掩码和栅格坐标转换 | 不决定候选优先级 |
| `global_frontier_topology.py` | 结构地点标签、portal 路径和闭合地点约束 | 不修改 Navfn/TEB 参数 |
| `global_frontier_place_memory.py` | 地点访问、观察增益、失败和 dormant 状态 | 不发布 ROS 路线 |
| `global_frontier_route_commands.py` | 把活动路线转换成稳定的 ROS 路线命令 | 不解释地图或选择地点 |
| `global_frontier_models.py` | 规划快照、候选和 watchdog 等数据契约 | 不执行副作用操作 |
| `global_frontier_planning_contract.py` | 慢规划提案的 generation、路线 lease 校验和原子提交边界 | 不选择目标、不发布速度或修改控制器 |

Goal Manager 采用同样的“主节点编排、策略模块负责单一职责”布局，具体拆分见 [`20260905_goal_manager_code_layout.md`](20260905_goal_manager_code_layout.md)。

## 运行调用链

```text
ROS timer
  -> GlobalFrontierPlanningCycleMixin.on_timer() (short ingress lock)
  -> global_frontier_planning_contract (snapshot token / proposal gate)
  -> lste_global_frontier_node 的 ROS 状态快照
  -> GlobalFrontierPlanningCycleMixin.build_frontier_planning_snapshot()
  -> GlobalFrontierObservationMixin 刷新地点拓扑和观察覆盖
  -> GlobalFrontierSelectionMixin.choose_valid_frontier()
  -> choose_frontier()
  -> global_frontier_grid / global_frontier_topology
  -> global_frontier_scoring
  -> GlobalFrontierPlanningMixin 的代价地图和 Navfn 可达性验证
  -> GlobalFrontierActivationMixin.activate_selected_frontier()
  -> 主节点保留 active route 状态
  -> GlobalFrontierExecutionMixin 监控执行和终止
  -> GlobalFrontierRouteCommandMixin 发布命令

候选扫描阶段不持有 `planning_lock`。执行终端通过 proposal gate 使旧快照失效，
只有仍然匹配当前 route lease 的提案才能进入 `activate_selected_frontier()`；
提交阶段才重新获取短时状态锁。
```

`GlobalFrontierSelectionMixin` 和 `GlobalFrontierRouteCommandMixin` 只读取宿主节点提供的状态与小型辅助方法，因此不会改变原有 ROS 话题和参数接口。

## 修改入口

- 修改评分公式或优先级：只改 `global_frontier_scoring.py`，先运行 `test_frontier_scoring.py`。
- 修改房间/走廊和 portal 判定：改 `global_frontier_topology.py`，补充拓扑测试。
- 修改候选筛选流程：改 `global_frontier_selection.py`，不要把评分公式复制回主节点。
- 修改代价地图、Navfn 验证或路线预取：改 `global_frontier_planning.py`，不要把规划服务调用塞回 ROS 回调。
- 修改规划周期的顺序、terminal successor 或无候选处理：改 `global_frontier_planning_cycle.py`。
- 修改候选提交、route id 初始化或选中状态日志：改 `global_frontier_activation.py`。
- 修改结构地点、观察 footprint 或地点离开提交：改 `global_frontier_observation.py`。
- 修改 ROS 话题、参数或生命周期：改 `lste_global_frontier_node.py`。
- 新增跨模块数据：先在 `global_frontier_models.py` 增加数据类，再传递对象，避免继续增长位置参数元组。

## 验证

```bash
python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
find src/lste_topo_access/scripts -maxdepth 1 -name '*.py' -print0 \
  | xargs -0 python3 -m py_compile
git diff --check
```

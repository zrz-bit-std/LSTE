# 20261105 路线所有权：图动作与观察工作项的双契约

## 问题

冷启动时全局前沿选择的是 **GP work item**（`place_observation_work`，无门洞、无图事务），
路线 `graph_transaction_id = 0`。但控制器链路上的三道闸都把“来源是
`global_slam_frontier` 且 `route_id > 0`”当成“图动作”，要求正的图事务：

1. `goal_manager_lifecycle._canonical_route_identity`（六字段一律 > 0）；
2. `goal_manager_goal_output._canonical_goal_identity_ready`（mission goal）；
3. `teb_goal_bridge_lifecycle._canonical_instruction_identity`（bridge 指令）。

再加上 `teb_goal_bridge_mission_input` 与 `lste_cmd_vel_mux_node` 的同类检查，
形成“没有图 ID 不能动 → 不动就看不到门洞 → 没有门洞就没有图 ID”的死锁。

## 设计

把路线分成两个所有权域，分类字段为 `route_ownership`：

| | `graph` | `observation` |
| --- | --- | --- |
| 触发 | portal probe / crossing、durable work-item 图动作 | GP work item、普通 frontier endpoint |
| 必须 | `graph_transaction_id > 0` + epoch/lifecycle/route | route/lifecycle/epoch + `work_item_id`/`attempt_id`（有 WI 时） |
| graph 高水位 | 参与 | 不参与，`0` 是合法哨兵 |
| 失效 | graph 回退/不匹配 | WI attempt 缺失、lifecycle/route 回退 |

未知 payload 默认 `graph`（fail closed），避免未升级的生产者绕过严格契约。

## 代码落点

- `src/lste_core/scripts/utils/route_ownership.py`：分类、观察身份、准入判定。
- `global_frontier_activation.py`：按动作类型设置 `active_route_ownership`，写入 dispatch contract。
- `global_frontier_reporting.py`：`goal_context` 与 route command 顶层携带所有权、WI/attempt。
- `goal_manager_lifecycle.py`：按类校验 canonical identity；观察路线跳过 graph 高水位。
- `goal_manager_frontier.py` / `goal_manager_goal_output.py`：mission goal 携带所有权；目标路线继承观察身份。
- `teb_goal_bridge_mission_input.py`：按类准入；观察路线校验 attempt。
- `teb_goal_bridge_lifecycle.py`：bridge 指令 identity 按类校验。
- `teb_goal_bridge_action_client.py` / `teb_turn_supervisor_*`：身份透传 + 超监督器按类比较。
- `lste_cmd_vel_mux_node.py`：TEB 命令按类比较 graph 身份。

## 测试

`src/lste_topo_access/test/test_route_ownership_contract.py`
（分类、GoalManager canonical identity、stale、bridge admission）

`src/lste_topo_access/test/test_route_ownership_bridge_chain.py`
（mission goal readiness、bridge instruction identity、目标路线继承）

覆盖：观察路线 graph=0 放行；图路线 graph=0 仍拒绝；观察 WI 缺 attempt 拒绝；
观察路线跳过 graph 高水位；图路线仍检查高水位；未知 payload fail closed；
目标路线继承观察身份；几何观察无 WI 也可放行。

## 已验证

- 33 个新增用例全部通过。
- 真实冷启动冒烟（`20261105_143807`）：机器人从 `(3.4, 0)` 移动到
  `(9.2, -0.3)`、`(6.4, 2.3)`、`(5.2, -0.1)`，bridge 正常
  `dispatched move_base action`，`missing_graph` / `missing_route` 拒绝计数为 0。
  该轮在 180 s 冒烟阈值下因后续 frontier 停顿结束（不是契约拒绝），未找到杯子。

# 20261104 黄杯搜索成功审计标准（简化版）

本文件定义 `env04_no_pro3.world` / `yellow_cup` 十轮重复实验的评测审计。
它与既有单元测试审计、`repeat_local_search.py` 自带统计相互独立：
pytest 只验证代码契约；本审计只看真实运行留下的指标日志、状态机记录和失败快照。

## 1. 判据

**成功 = 在时限内找到黄色杯子。**

- “找到”以指标日志出现 `event=task_done` 且 `value=true` 为准；
- 单轮总时限 **7200 s（2 h）**，从启动器开始计时，含启动、探索、停车与落盘；
- 卡死 **300 s（5 min）**：距最近位置锚点 0.35 m 内无平移进展超过 300 s 判失败，
  转动与里程计噪声不重置计时；
- 总时限或卡死任一超限，即该轮失败，即使之后才看到杯子；
- 完成策略仍启用 `TARGET_DONE_REQUIRE_LOCKED=true`，因此 `task_done`
  隐含感知状态机已进入 `LOCKED`；`/lste/state` 另有独立记录用于佐证，
  记录缺失时不改变成功结论，只在报告中标注。

不参与判定的观察项：门洞/portal 数量与位置、路径长度、停车次数、检测帧匹配率。
这些只作为运行质量记录写入报告，不影响成功与否。

## 2. 运行边界

| 项目 | 值 |
| --- | --- |
| World | `worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world` |
| Task | `model/Data_exchange/vlm_prompt/lab/yellow_cup.json`（`TASK_ID=yellow_cup`） |
| 起点 | `(3.4, 0, 0.1)`，yaw `0` |
| 目标 | `cup_yellow`，保存位姿 `(20.4843, -3.2836)` |
| 轮数 | 10 轮，每轮完整停止并重启 Gazebo 与业务节点 |

## 3. 每轮记录

- `attempt_result.json`：运行号、状态、失败原因、卡死位姿/锚点、总墙钟；
- `<run>_navigation_metrics.log`：启动配置、周期采样、目标事件、完成/停止汇总；
- `<run>_state.jsonl`：`/lste/state` 与 `/lste/task_done` 的独立记录；
- `<run>_failure_*.json`：指标节点写出的失败现场快照（若有）；
- `<run>_audit.json`：本审计的机器可读结论；
- `visualization/`：离线轨迹图与 `visualization_summary.json`。

## 4. 计时口径

| 字段 | 定义 |
| --- | --- |
| `startup_seconds` | 驱动脚本开始（含 stop/launch）到首个位姿采样 |
| `search_wall_seconds` | `run_start` 到 `task_done` 的墙钟差 |
| `search_sim_seconds` | 同上，仿真时钟差 |
| `acquire_latency` | `target_acquired.latency_seconds` |
| `acquire_to_confirm` | `target_follow_confirmed` 与 `target_acquired` 的 ros_time 差 |
| `confirm_to_done` | `task_done` 与 `target_follow_confirmed` 的 ros_time 差 |
| `stopped_total_seconds` | 所有 `stop_end.duration_seconds` 之和 |
| `stopped_max_seconds` | 上述最大值；与 300 s 卡死阈值分别统计 |
| `stall_*` | 失败快照中的停顿片段：起止、时长、位姿、主因 |

## 5. 轮次结论

| 结论 | 条件 |
| --- | --- |
| 成功 | `task_done=true` 且在 7200 s 内完成，且无 >300 s 卡死 |
| 失败 | 未出现 `task_done=true` |
| 超时失败 | 完成时间 >7200 s，或出现 >300 s 卡死 |
| 基础设施失败 | 启动失败、180 s 内无位姿、绘图/落盘失败 |

批次报告给出：成功次数、超时次数、失败原因分布、各阶段耗时与最长停顿。

## 6. 与既有规则的区别

- 不复用 `test_*` 断言，不把 pytest 通过当成实验成功；
- 不复用 `repeat_local_search.py` 的“三次成功”目标，本批次固定 10 轮；
- 不把检测到单帧目标框、`/goal` 到达或 `move_base SUCCEEDED` 当成成功；
- 门洞、路径、停车等只做观察记录，不作为成功条件；
- 失败快照只记录实际观测，不补造相机、深度或 TF 证据。

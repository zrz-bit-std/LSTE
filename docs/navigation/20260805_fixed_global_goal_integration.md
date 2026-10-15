# 固定 Global Goal 接入说明

## 目的

该模式用于把一个已知坐标直接接入正式的 LSTE global-goal 接口，验证
“目标发布 -> 控制器 -> 车辆”的闭环，而不让检测、状态机或 GP frontier 在
运行过程中改变终点。

它不是第二个 goal publisher。无论是正常大脑模式还是固定模式，唯一发布者
始终是 `lste_goal_manager.py`，唯一接口始终是：

```text
/lste/final_goal  (geometry_msgs/PoseStamped, frame_id=odom)
```

## 配置

编辑 `scripts/config/pipeline_defaults.yaml`：

```yaml
# brain: 正常 LSTE 决策；fixed: 固定坐标目标
GLOBAL_GOAL_SOURCE: fixed
FIXED_GLOBAL_GOAL_X: 12.95
FIXED_GLOBAL_GOAL_Y: -6.51
FIXED_GLOBAL_GOAL_YAW: 0.0
FIXED_GLOBAL_GOAL_PUBLISH_PERIOD: 1.0
FIXED_GLOBAL_GOAL_ALLOW_CLICK_OVERRIDE: true
```

坐标单位为米，坐标系是 `odom`。当前 Gazebo 仿真中 `odom` 与 world 原点一致，
因此 Gazebo 的 Shift + 左键目标可直接作为固定目标更新。真实机器人上若没有
确认这两个坐标系一致，应关闭 `FIXED_GLOBAL_GOAL_ALLOW_CLICK_OVERRIDE`。

切回原始大脑只需恢复：

```yaml
GLOBAL_GOAL_SOURCE: brain
```

改完配置后重启 LSTE 节点会话即可；不需要 catkin rebuild。

## 运行语义

`fixed` 模式不等待 `/rbt_pose`、检测结果、状态机或 GP 输出。Goal Manager 启动
后立即发布配置坐标，并按 `FIXED_GLOBAL_GOAL_PUBLISH_PERIOD` 重发同一目标，以便
控制器重启后重新订阅。它不会因为重发而产生新目标。

如果启用了点击更新，Shift + 左键会修改 Goal Manager 内存中的固定坐标，立即发布
新的 `/lste/final_goal`；下游控制器收到的是同一个正式接口上的一次真正目标变化。

## 已验证的闭环

固定目标对照通过 `rltest` 运行在独立 ROS/Gazebo 端口，避免干扰正常 LSTE：

| 项目 | 值 |
| --- | --- |
| 起点 | `(15.79, -8.45, -pi/2)` |
| global goal | `(12.95, -6.51)` |
| global-goal 发布者 | `lste_goal_manager.py`, `fixed` source |
| 轨迹执行 | 在线 SLAM + Navfn + TEB + frontier 临时子目标 |
| 本次运行日志 | `runtime/rl_fixed_goal_test/logs/20260805_200356/` |

运行日志记录：Goal Manager 持续发布固定终点；前沿管理器在 SLAM 确认最终点
属于机器人可安全连通区域后，于仿真时间 `2122.320` 切换到最终目标，随后在
`2128.799` 记录 `move_base reports the fixed final goal reached`。

这里验证的是当前“固定 global goal + TEB 基线”能够稳定抵达，不应把它表述为
原始 SA-PPO policy 已经解决绕墙问题。`CONTROLLER_METHOD: rl` 仍可通过同一
`/lste/final_goal` 进行独立对照。

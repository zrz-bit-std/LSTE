# GP Subgoal 业务流程

本文描述 `gp_subgoals_sim_topo.py` 在机器人运行时承担的业务流程，而不是
类关系或函数调用图。

```mermaid
flowchart TD
    lidar[雷达原始点云] --> projector[oc_srfc_proj]
    odom[轮速里程计] --> projector
    projector -->|/sph_pcl| gp[gp_subgoal]
    projector -->|/rbt_pose| gp

    state[/lste/state/] --> manager[Goal Manager]
    detections[/lste/detections/] --> manager
    depth[相机深度与内参] --> manager

    gp -->|全部 frontier 候选| frontiers[/lste/gp_frontiers/]
    frontiers --> manager

    gp -->|回退模式和回退目标| topo[/lste/access_topo/mode\n/lste/access_topo/backtrack_goal/]
    topo --> manager

    manager -->|唯一当前目标| final[/lste/final_goal/]
    final --> gp
    final --> vis[Detection/RViz visualization]

    final --> rl[SA-PPO 控制器]
    lidar --> scan[pointcloud_to_laserscan]
    scan -->|/pro3/rlscan| rl
    rl -->|/lste/cmd_vel/sappo| mux[速度 mux]
    teleop[键盘遥控] -->|/lste/cmd_vel/teleop| mux
    mode[/lste/controller_mode/] --> mux
    mux --> cmd[/cmd_vel/]
    cmd --> base[Gazebo 差速底盘]
    base --> odom
```

## GP Subgoal 做什么

`gp_subgoal` 接收局部球面雷达视图和机器人位姿。每一轮计算遵循：

```mermaid
flowchart LR
    input[最新 /sph_pcl 与 /rbt_pose]
    topo[推进已有回退或强制分支]
    gp[拟合局部 Sparse GP]
    predict[预测球面占据与不确定度]
    frontier[提取全部高不确定 frontier]
    memory[更新 anchor、路口分支和回退记忆]
    publish[发布 frontier 列表和调试点云]
    input --> topo --> gp --> predict --> frontier --> memory --> publish
```

GP 只回答："机器人附近哪些方向仍未知、并且可能通行？" 它不直接控制车辆。

## 谁决定最终方向

```mermaid
flowchart TD
    mode{Access-Topo 要求回退？}
    backtrack[采用已记录的回退目标]
    state{LSTE 状态和 subtype}
    explore[PASS 或 Sus-C：\n评估全部 GP frontier]
    target[Sus-A 或 LOCKED：\n跟随目标检测]
    context[Sus-B：\n跟随上下文物体对的中点]
    final[发布 /lste/final_goal]

    mode -->|yes| backtrack --> final
    mode -->|no| state
    state -->|PASS / Sus-C| explore --> final
    state -->|Sus-A / LOCKED| target --> final
    state -->|Sus-B| context --> final
```

`lste_goal_manager.py` 完成最后选择。它接收全部 frontier，但目标跟随、
上下文跟随或拓扑回退的优先级更高时，会忽略 frontier。

## Anchor 记忆

anchor 是机器人轨迹上的稀疏点，不是 GP 训练点。当前 profile 中，PASS
状态下离上一个 anchor 超过 1.5 m 时新建；Sus-C 则是超过 1.0 m。一个
anchor 附近出现多个稳定 frontier 峰时，被视作可能路口。已选择的分支标记
为已走过，未选择的分支保留为 `PENDING`，以便未来回退后继续探索。

## 控制器模式

默认执行 `./scripts/lifecycle/run_nodes_tmux.sh` 时，SA-PPO 直接订阅 `/lste/final_goal`，
结合 `/pro3/rlscan` 和 `/pro3/wheel_odom` 发布候选速度。SA-PPO 与键盘控制始终
并存，但发布到不同输入话题；速度 mux 根据 `/lste/controller_mode` 选择一个输入，
并作为 `/cmd_vel` 的唯一发布者。

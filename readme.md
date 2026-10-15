# LSTE：视觉目标搜索与自主导航

LSTE 在 Gazebo 场景中驱动 Pro3 机器人，根据任务中的目标描述自主探索、识别目标、接近并确认完成。本地已完成一次 `env04_no_pro3.world` 中寻找 `yellow_cup` 的端到端实验，成功记录为 **`20261014_221223`**。

本文记录复现过程、系统设计和实验操作，不展开依赖安装与环境搭建。所有命令默认在工作区根目录执行。

## 1. 复现过程与已完成的工作

参考环境为 `dh@10.1.83.236:~/dh_ws/lste_ws`，整个复现过程中远程仅作只读参考，修改与实验均在本地 `/home/zrz/work/lste_ws` 完成。当前开发分支为 `20260805_exploration_navigation_dev`。

复现按以下链路逐步打通：

1. **恢复仿真与传感器。** 完成本地工作区编译，补齐模型与提示词缓存，恢复激光雷达插件；确认 Gazebo 场景、Pro3、里程计、相机与激光雷达正常工作，并将点云转激光的输入对齐到 `/pro3/os_cloud_node/points`。
2. **恢复业务节点。** 修复 `lste_goal_manager.py` 不可执行导致的启动退出，修正遥控入口的 Python 调用，使目标管理、检测、探索、导航和控制节点能够一起启动。
3. **打通探索到目标跟随。** 使用在线 SLAM 地图与全局前沿探索，在目标出现后由 Goal Manager 接管搜索，生成经 Navfn 验证的目标接近动作，并交给 TEB 执行。
4. **修复闭环中的重复等待。** 同一目标丢失后再次确认时创建新的接近轮次，避免旧轮次已关闭的观察点导致立即耗尽；目标末端开始累计完成证据时优先保持当前观察位置，避免继续前进反复清空确认条件；修正持久导航动作在目标接近边界被错误启动释放看门狗的问题。
5. **反复运行并依据日志排障。** 按“同一位置持续无进展超过 3 分钟，记录位置与上下文后判定本轮失败，再重新运行”的规则推进，最终获得一次 `/lste/task_done=true` 的完整实验。
6. **保存并绘制实验结果。** 从成功实验的指标日志提取真实里程计轨迹与完成事件，从 world 和模型文件重建场景平面图，导出轨迹图、局部放大图、完成证据和搜索进程图。

这里的复现成果是一次完整成功实验；尚未由此建立多次运行的成功率或跨场景性能结论。

## 2. 系统有哪些设计

| 模块 | 主要职责与设计 |
| --- | --- |
| 仿真与传感器 | world 提供场景，启动脚本单独生成 Pro3；相机用于视觉检测，激光与里程计用于建图和导航。 |
| 任务与提示词 | 任务 JSON 描述目标、环境先验、关联物与负面线索；Prompt 节点优先复用任务缓存，缓存未命中时才启动 MiniCPM。 |
| 感知与状态 | 当前默认使用 WeDetect-Large、PyTorch 推理；结合目标、环境和上下文检测进行评分与状态判断，以多帧证据降低单帧误检影响。 |
| 建图与探索 | 在线 SLAM 发布 `/map`；全局探索根据地图组织区域、通道与探索工作项，选择可达前沿。当前默认关闭旧版 GP Frontier。 |
| 目标管理 | Goal Manager 仲裁探索、上下文观察与直接目标跟随；确认目标后管理接近轮次、可达观察点、丢失重检与完成证据。 |
| 路径与控制 | Navfn 验证全局可达性，TEB 负责局部轨迹与避障；当前使用 `persistent_stream` 执行架构，由控制切换节点和速度 mux 统一下发运动命令。 |
| 日志与评估 | Navigation Metrics 记录轨迹、任务事件、控制状态和失败快照；Gazebo 目标真值仅供旁路评估，不向搜索控制提供目标位置。 |

一次实验的主要数据流为：

```text
任务 JSON → 提示词缓存 / MiniCPM → WeDetect → 评分与状态
                                             ↓
激光 + 里程计 → 在线 SLAM → 全局前沿探索 → Goal Manager
                                             ↓
                                  Navfn → TEB → 速度 mux → Pro3
                                             ↓
                                多帧目标完成证据 → /lste/task_done

Navigation Metrics 旁路记录上述过程 → 实验日志 → 离线可视化
```

成功判据是 Goal Manager 发布 `/lste/task_done=true`。检测到一个目标框、出现 `/goal` 或单段导航到达，都不能单独代表搜索任务完成。当前成功记录的完成证据为 `terminal_track_continuity`：目标接近末端保留已确认轨迹的连续证据，累计独立检测消息与保持时间后完成任务。

## 3. 在哪里切换 world、任务和起点

统一修改 [scripts/config/pipeline_defaults.yaml](scripts/config/pipeline_defaults.yaml)。当前对应成功实验场景的主要值为：

```yaml
WORLD: worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world
TASK_JSON: model/Data_exchange/vlm_prompt/lab/yellow_cup.json
TASK_ID: yellow_cup

PRO3_SPAWN_X: 3.4
PRO3_SPAWN_Y: 0
PRO3_SPAWN_Z: 0.1
PRO3_SPAWN_YAW: 0

LSTE_CONTROLLER: teb
ONLINE_SLAM_ENABLED: true
GLOBAL_FRONTIER_ENABLED: true
LEGACY_GP_FRONTIER_ENABLED: false
DETECTOR: wedetect-large
WDETECT_RUNTIME: pytorch
```

`WORLD`、`TASK_JSON` 的相对路径以工作区根目录为基准。换场景时检查出生点是否处于可通行区域；位置单位为米，`YAW` 单位为弧度。默认流程在场景加载后生成机器人，应选用不内置 Pro3 的 world，避免重复生成。

更换寻找对象时同步修改 `TASK_JSON` 与 `TASK_ID`，JSON 内的 `task_parsed` 描述目标与搜索先验。若保留仿真评估，还需同步修改同一 YAML 中的 `TARGET_EVAL_GAZEBO_TARGET_MODEL`、`TARGET_EVAL_TARGET_LABELS` 和目标尺寸等评估项；它们不负责给机器人指路。

修改这些启动参数不需要重新编译。**切换 world、出生点，或开始一轮独立实验时，应完整停止再启动**，以重新建立机器人初始状态与在线地图；仅重启业务节点会保留 Gazebo 中机器人已有的位置。

启动器也支持 `PIPELINE_CONFIG` 指向实验 YAML：先读取默认配置，再用实验 YAML 覆盖同名项；已有同名 shell 环境变量优先于 YAML。

## 4. 怎么启动一次实验

### 4.1 从初始状态启动

确认上节参数后执行：

```bash
cd /home/zrz/work/lste_ws
export LSTE_WS="$PWD"
source scripts/config/pipeline_env.sh

# 停止上一轮，并让指标节点写入最终记录；首次启动也可执行
./scripts/lifecycle/stop_all_tmux.sh

# 启动场景、机器人、业务节点、探索与 TEB 导航
./scripts/lifecycle/run_all_tmux.sh
```

启动器创建 `lste-env`（ROS 与 Gazebo）、`lste`（业务与导航）和 `lste-teleop`（遥控备用）三个 tmux 会话。`yellow_cup` 命中提示词缓存时，MiniCPM 不启动，属于正常行为。完整启动入口会检查节点就绪；模型加载期间应等待检查结束。

直接再次执行 `run_all_tmux.sh` 可能复用已有会话，不能当作一轮全新实验。短命令 `runall` / `stopall` 若已可用，分别对应上述完整启停入口。

### 4.2 观察运行和完成信号

在另一个终端执行；各监控命令按需单独运行：

```bash
cd /home/zrz/work/lste_ws
source scripts/config/pipeline_env.sh

# 节点与窗口
rosnode list
tmux list-windows -t lste
tmux attach -t lste

# 查看任务、检测、当前目标与最终完成信号
rostopic echo /lste/task
rostopic echo /lste/detections
rostopic echo /goal
rostopic echo /lste/task_done

# 地图、前沿与导航的实时显示
rviz -d src/lste_topo_access/launch/gp_frontier.rviz
```

`tmux attach` 后按 `Ctrl+B` 再按 `D` 可退出查看而保持实验运行。重点关注 `goal`、`global_frontier`、`teb_nav`、`cmd_vel_mux` 和 `metrics` 窗口。

看到 `/lste/task_done` 输出 `data: true` 后，使用 `stop_all_tmux.sh` 结束实验，让指标节点落盘 `run_stop`。不要仅关闭终端来替代正常停止。

### 4.3 卡住时怎么处理

本次复现采用的规则是：机器人在同一位置持续无实质进展超过 **180 秒墙钟时间**，记录坐标、目标和对应日志，将这一轮标记为失败，停止后重新运行。正常观察等待与短时规划恢复不应立即当作整轮失败。

在停止前可保存或查看以下现场信息，并结合该轮 `sample`、失败快照中的 pose、route 和 command chain 分析：

```bash
tmux capture-pane -p -t lste:goal -S -120
tmux capture-pane -p -t lste:global_frontier -S -120
tmux capture-pane -p -t lste:teb_nav -S -120
rostopic echo -n 1 /pro3/wheel_odom
rostopic echo -n 1 /goal
```

这是实验判定与重试规则；`run_all_tmux.sh` 本身没有实现“180 秒卡死后自动重启直到成功”的外层循环。节点内部的规划/控制超时与该规则分别生效。

## 5. 跑完后到哪里看结果

每轮导航指标默认写到：

```text
runtime/navigation/logs/<YYYYMMDD_HHMMSS>/
├── <时间戳>_navigation_metrics.log   # 启动配置、周期采样、目标与完成事件、停止汇总
├── <时间戳>_failure_evidence.log     # 启用失败取证时的事件流
└── <时间戳>_failure_0001.json        # 出现失败片段时的独立现场快照
```

这不是所有进程标准输出的集合；节点实时输出还可从各 tmux 窗口查看。默认日志保留期为 15 天，需长期保存的实验应另行归档。`attempt_result.txt` 是此次复现整理的实验说明，不是启动器每轮必定生成的文件。

成功实验 [20261014_221223](runtime/navigation/logs/20261014_221223/) 的结果如下：

| 项目 | 结果 |
| --- | --- |
| 场景 / 任务 | `env04_no_pro3.world` / `yellow_cup` |
| 完成信号 / 证据 | `/lste/task_done=true` / `terminal_track_continuity` |
| 完成耗时 | 98.537 s 仿真时间；315.294 s 墙钟时间，以指标记录边界为准 |
| 完成时累计行程 | 32.6049 m |
| 完成时机器人位姿 | 约 `(20.2882, -7.7693, 1.3832)`，即里程计 `x, y, yaw` |
| 绘图采样 | 193 个周期样本 + 1 个完成快照，共 194 行 |
| 最后一次目标暴露片段的检测匹配 | 109 / 134 帧，仅表示该片段 |

该轮保留了一个 `planner_materialization_stall` 短时异常片段，记录位置约 `(20.1994, -12.5970)`，取证片段约 12.19 s；之后实验继续并完成，未触发上述 3 分钟整轮失败条件。成功不等于日志中完全没有恢复或异常事件。

## 6. 跑完后怎么生成可视化

### 6.1 重新生成本次成功轨迹图

绘图入口为 [scripts/tools/plot_world_run.py](scripts/tools/plot_world_run.py)，可在实验停止后离线运行，不需要启动 ROS 或 Gazebo：

```bash
cd /home/zrz/work/lste_ws
python3 scripts/tools/plot_world_run.py \
  runtime/navigation/logs/20261014_221223/20261014_221223_navigation_metrics.log \
  --world worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world \
  --output-dir runtime/navigation/logs/20261014_221223/visualization
```

脚本读取三类真实数据：指标 log 中的里程计与任务事件、world 中的碰撞几何与保存位姿，以及默认 `~/.gazebo/models` 下引用的 COLLADA 模型。模型放在其他位置时，可通过 `--model-root /实际模型根目录` 指定，支持重复传入。

生成四个面板：场景与时间渐变轨迹、目标区域放大、完成证据摘要、累计行程与事件时间线。输出位于该轮的 `visualization/`：

| 文件 | 用途 |
| --- | --- |
| `<时间戳>_env04_success_trajectory.png` | 350 dpi 图片，便于预览与插入文稿 |
| 同名 `.pdf`、`.svg` | 矢量输出，便于排版与后续编辑 |
| `trajectory_samples.csv` | 实际用于绘图的时间、位姿与累计行程样本 |
| `env04_scene_manifest.json` | 场景轮廓、来源、目标参考位置和缺失 mesh 列表 |
| `visualization_summary.json` | 完成信号、耗时、行程和输出路径等摘要 |

现成结果：[轨迹 PNG](runtime/navigation/logs/20261014_221223/visualization/20261014_221223_env04_success_trajectory.png)、[矢量 PDF](runtime/navigation/logs/20261014_221223/visualization/20261014_221223_env04_success_trajectory.pdf)、[矢量 SVG](runtime/navigation/logs/20261014_221223/visualization/20261014_221223_env04_success_trajectory.svg)。该次图的详细数据说明见 [visualization/README.md](runtime/navigation/logs/20261014_221223/visualization/README.md)。

### 6.2 新实验怎么使用

对于**同一 ENV04 黄杯任务、日志事件齐全且采用相同完成证据的成功轮次**，将上面命令中的日志路径和输出目录替换成新时间戳，`--world` 保持与实际运行文件一致。保留实验时所用 world 及模型版本，避免场景后来修改后与历史轨迹不匹配。

目前脚本专为本次 ENV04 成功实验编写：墙体模型名 `env04`、目标模型名 `cup_yellow`、局部放大范围、标题和部分完成证据文字写在脚本内，并要求日志包含 `task_done`、`task_completed`、`run_stop` 等事件。**切换其他 world、目标、完成策略，或绘制失败实验时，需要相应调整脚本；仅替换 `--world` 不足以保证图正确。** 可视化由上述命令手动生成，完整启动器不会自动出图。

### 6.3 图的解释边界

场景底图是 SDF 碰撞几何和模型投影重建的平面轮廓，不是本轮保存的 SLAM 栅格地图；家具使用投影凸包。轨迹来自记录的里程计采样，以直线连接且不做平滑。叠加假设 odom 与 world 轴向名义对齐，未记录完整 TF 流，因此不能据此计算独立标定的定位误差。

黄杯星标取自 world 保存状态的位置 `(20.4843, -3.28361)`；它是场景参考点，不是实时目标真值轨迹。缺失相机截图、深度或 TF 证据不会被绘图脚本补造；若有缺失 mesh，应先查看场景清单中的 `missing_meshes`。

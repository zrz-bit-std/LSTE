# LSTE 启动指南（SA-PPO、TEB、Teleop）

## 控制模式

`run_nodes_tmux.sh` 通过 `LSTE_CONTROLLER` 选择车辆控制器：

- `LSTE_CONTROLLER=teb`：默认自主链路。gmapping 在线建图，frontier 提供探索 waypoint，Navfn 计算全局路线，TEB 发布局部速度到 `/lste/cmd_vel/teb`。
- `LSTE_CONTROLLER=sappo`：SA-PPO 对照链路。LSTE 大脑发布 `/lste/final_goal`，SA-PPO 订阅该目标并发布 `/cmd_vel`。
- `LSTE_CONTROLLER=teleop`：调试模式，由键盘遥控发布 `/cmd_vel`。
- **SA-PPO 独立验证**：将 Ouster 点云转为激光 `/pro3/rlscan`，SA-PPO 根据手动目标 `/move_base/current_goal` 发布 `/cmd_vel`。

不要同时启动 `run_nodes_tmux.sh` 和独立验证脚本 `run_sappo_tmux.sh`：两者都会生成 Pro3，且可能同时发布 `/cmd_vel`。

## 启动前准备

进入工作区：

```bash
cd .
```

首次使用，或修改过 Gazebo GUI 取点插件、点云转激光包时，先编译 SA-PPO 依赖：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg gazebo_click_point pointcloud_to_laserscan
```

各启动脚本会自行加载 ROS Noetic 和工作区环境，无需在每个新终端重复 `source`。
SA-PPO 默认使用 `$HOME/miniconda3/envs/rlenvs/bin/python`；若环境位于其他位置，
启动时通过 `SAPPO_PYTHON=/绝对路径/python` 覆盖。

## 启动 LSTE + SA-PPO 自主链路

首次使用短命令时，在工作区根目录授权 `.envrc`：

```bash
direnv allow
```

之后进入工作区会自动设置 `LSTE_WS` 并提供 `runall`、`stopall`、`lste-env`；离开工作区后
这些环境设置会自动撤销。推荐使用一键启动命令：

```bash
runall
```

`runall` 只在后台创建 tmux 会话，不会自动进入任何会话；启动完成后会直接返回
当前终端。需要查看日志时再手动执行 `tmux attach -t lste-env`、
`tmux attach -t lste` 或 `teleop`。

完整路径形式仍然可用：

```bash
cd .
./scripts/lifecycle/run_all_tmux.sh
```

该脚本会依次启动并检查三个平级 tmux 会话：

| 会话 | 内容 |
| --- | --- |
| `lste-env` | ROS master、Gazebo server 和带操作插件的 Gazebo GUI |
| `lste` | LSTE 大脑、持续健康检查、点云转激光、速度 mux、控制器切换节点、SA-PPO，以及可选的在线 SLAM + Navfn + TEB |
| `lste-teleop` | 始终运行的键盘控制进程 |

脚本可重复执行：已运行的环境会被复用，缺失的会话会被创建，Gazebo GUI 插件
会被检查和修复。

### 分步启动

需要编辑世界、测试 Gazebo 插件或分步调试时，只启动 ROS 与 Gazebo 环境：

```bash
lste-env
```

`lste-env` 始终在后台返回当前终端，不会进入 tmux，也不会启动 `lste`、`lste-teleop`、
车辆、检测器、MiniCPM 或 SA-PPO。完整脚本形式
`./scripts/lifecycle/run_env_tmux.sh` 仍可用于需要直接附着环境会话的调试。

`lste-env` 包含：

| 窗口 | 内容 |
| --- | --- |
| `roscore` | ROS master |
| `world` | Gazebo 场景，初始不生成车辆 |

Gazebo GUI 是必需组件，启动脚本会始终加载 `libgazebo_click_point.so`。如果已有
`lste-env` 会话但 GUI 或插件未运行，再次执行 `./scripts/lifecycle/run_env_tmux.sh` 会自动
重建 GUI 客户端，不会重启 `gzserver` 或业务节点。

脚本会附着到该会话。确认 Gazebo 打开后，按 `Ctrl+B`，再按 `D`，从 tmux 分离回普通终端。

再启动大脑、车辆和默认的 TEB：

```bash
cd .
./scripts/lifecycle/run_nodes_tmux.sh
```

控制器默认读取 `pipeline_defaults.yaml` 中的 `LSTE_CONTROLLER: teb`，TEB 最大线速度为
`0.50 m/s`。SA-PPO、TEB 和 teleop 进程都会常驻运行，只有 mux 的输入源会改变。

需要做 SA-PPO 对照时，显式选择 SA-PPO：

```bash
LSTE_CONTROLLER=sappo ./scripts/lifecycle/run_nodes_tmux.sh
```

也可以把同样的变量加到一键启动命令：

```bash
LSTE_CONTROLLER=sappo ./scripts/lifecycle/run_all_tmux.sh
```

启动后，SA-PPO 和 teleop 可通过 Gazebo 插件或 `Ctrl+Shift+K` 热切换；TEB 可通过
`./scripts/lifecycle/switch_controller.sh teb` 选择。任何切换都不会重启 Gazebo、SLAM 或大脑。

脚本创建 tmux 会话 `lste`，只生成一辆 Pro3，并启动任务、VLLM、Goal Manager、VLM Prompt、开放词汇检测器、评分、状态、可视化、球面投影、在线 SLAM frontier、TEB、点云转激光和 SA-PPO。旧 GP frontier 和其 RViz 默认不启动；需要旧拓扑对照时，将 `LEGACY_GP_FRONTIER_ENABLED` 改为 `true`。

### 检测器选择

检测器不是同一个文件中的两个后端分支，而是两个独立 ROS 节点；两者只共享
`/lste/detections` 消息接口，因此评分、Goal Manager、SA-PPO 和 RViz 无需随模型切换而修改：

| 选择 | 独立节点文件 | 当前用途 |
| --- | --- | --- |
| `groundingdino` | `src/lste_core/scripts/lste_det_node.py` | 保留的原始 GroundingDINO 路径，独立加载 DINO 模型、执行两次推理。 |
| `wedetect` | `src/lste_core/scripts/lste_wedetect_det_node.py` | WeDetect 路径，当前配置为 WeDetect-Large TensorRT FP16。 |

启动器根据 `scripts/config/pipeline_defaults.yaml` 的唯一模型选择项 `DETECTOR` 选择其中一个
可执行文件，两个模型不会在同一个 Python 进程中导入、加载或运行。只需要修改这一行：

```yaml
DETECTOR: wedetect-large
```

可选值为 `groundingdino`、`wedetect-base`、`wedetect-large`。选择 WeDetect 后，启动器自动
匹配该型号的 checkpoint、XLM-RoBERTa、ONNX 与 TensorRT engine cache；不需要同时修改任何
`WDETECT_*` 路径。改完后重启 LSTE 节点会话即可生效。WeDetect 的 checkpoint、ONNX 与
TensorRT engine 会保留在 `model/WeDetect/`，DINO 的配置和权重也不会被改写或删除。

WeDetect 一次视觉前向同时处理目标和环境类别，类别文本向量只在任务 prompt 变化时计算一次。

首次使用 WeDetect 时下载官方源码和零样本权重（下载文件不纳入 Git）：

```bash
bash scripts/models/setup_wedetect.sh
```

该脚本只在 `dino` 环境补充 `mmengine==0.10.7`，用于读取官方 checkpoint 元数据；
不安装 MMCV/MMDetection，也不会升级 ROS/OpenCV 使用的 NumPy 1.26.4。

Large 的首次 TensorRT 启动会为 RTX 3060 构建 FP16 engine；后续直接复用
`model/WeDetect/deploy/trt_cache_large/` 中的缓存。无需改动 Goal Manager、评分、
可视化或 SA-PPO。

`WDETECT_TRT_MAX_CLASSES` 固定 TensorRT 的提示词容量，默认 16。任务可自由切换
prompt，只要类别数不超过该值就复用同一份 engine；只有提高该容量、修改视觉 ONNX、
更换 GPU 或删除缓存时才需要重新编译。

完整的架构、环境版本、缓存策略与实测结果见
[`20260723_wedetect_tensorrt_integration.md`](../perception/20260723_wedetect_tensorrt_integration.md)。

WeDetect 的中文类别映射、阈值、NMS 和 FP16 设置也位于同一配置文件。当前映射覆盖
`blue mug`、`yellow mug`、桌椅、显示器、文件夹、门、消防栓和灭火器；新类别可通过
`~wedetect_label_map` ROS 参数补充英文到中文的映射。

`lste_goal_manager.py` 是 `/lste/final_goal` 的唯一发布者。该消息类型为 `geometry_msgs/PoseStamped`，坐标系为 `odom`。SA-PPO 使用同一 `odom` 坐标系下的 `/pro3/wheel_odom` 计算车体局部目标，并输出 `/cmd_vel`。

如需绕过大脑、检测与 GP 来验证控制器，可在 `scripts/config/pipeline_defaults.yaml` 将 `GLOBAL_GOAL_SOURCE` 改为 `fixed`，并设置 `FIXED_GLOBAL_GOAL_X/Y/YAW`。固定模式仍由同一个 Goal Manager 发布 `/lste/final_goal`；详细配置、Gazebo 点击更新和已验证路径见 [固定 Global Goal 接入说明](../navigation/20260805_fixed_global_goal_integration.md)。

SA-PPO 在收到第一条 `/lste/final_goal` 之前只发布零速度。到达目标后，Goal Manager 对同一目标的周期性重发不会重新启动车辆；大脑发布位置发生变化的新目标时，控制器会重新开始运动。

### 3. 确认全局目标

另开终端执行：

```bash
cd .
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo /lste/final_goal
```

收到消息后应看到 `header.frame_id: "odom"` 和目标的 `pose.position.x/y`。若一直没有消息，先查看 `lste` 会话中的 `goal`、`oc_srfc` 和 `global_frontier` 窗口；旧的 `gp_frontier` 默认不会启动。Goal Manager 依赖机器人位姿、状态和相应感知结果。

确认目标与控制器已经连接：

```bash
rostopic info /lste/final_goal
rostopic info /pro3/rlscan
rostopic info /cmd_vel
```

预期 `/lste/final_goal` 的订阅者包含 `/StageEnv_0`，`/cmd_vel` 的发布者只有 `/StageEnv_0`。

常用 tmux 查看命令：

```bash
tmux attach -t lste-env
tmux attach -t lste
```

## 启动 LSTE 键盘调试模式

系统运行期间可在 SA-PPO 和键盘控制之间热切换，不会重启 Gazebo、Pro3 或任何
LSTE 大脑节点。推荐使用以下任一方式：

- 点击 Gazebo 左上角的 `RL / KB` 按钮。
- 在 Gazebo 窗口按 `Ctrl+Shift+K`。

切换完成后，鼠标位置会显示 `Switched to keyboard control` 或
`Switched to SA-PPO`；提示只在后端确认控制器已经切换后出现。

当前模式可通过以下话题确认：

```bash
rostopic echo /lste/controller_mode
```

进入键盘窗口：

```bash
teleop
```

命令行仍可作为备用方式：

```bash
./scripts/lifecycle/switch_controller.sh teleop
./scripts/lifecycle/switch_controller.sh sappo
```

SA-PPO 与 teleop 始终运行，并分别发布到 `/lste/cmd_vel/sappo` 和
`/lste/cmd_vel/teleop`。切换只改变速度 mux 的输入选择；两个控制器进程都不会
退出。`/cmd_vel` 始终只有 mux 一个发布者。
键盘控制位于独立的 `lste-teleop` tmux 会话，与 `lste` 和 `lste-env` 平级；切换
控制器不会重建另外两个会话。

## 启动 SA-PPO 独立验证

此模式用于验证策略、点云转激光和车辆控制，不启动 LSTE 大脑，也不会生成 `/lste/final_goal`。

确保 `lste` 大脑会话未运行后，执行：

```bash
cd .
SAPPO_SPEED=0.50 ./scripts/lifecycle/run_sappo_tmux.sh
```

若 `lste-env` 不存在，脚本会先创建它；随后创建 `sappo-lste` 会话：

| 窗口 | 内容 |
| --- | --- |
| `pro3` | 生成 Pro3 |
| `scan` | `/os_cloud_node/points` 转 `/pro3/rlscan` |
| `sappo` | 运行 `sappo_pure.py`，发布 `/cmd_vel` |

可在启动时覆盖速度倍率和默认目标：

```bash
SAPPO_SPEED=0.50 SAPPO_GOAL_X=10.5 SAPPO_GOAL_Y=6.0 ./scripts/lifecycle/run_sappo_tmux.sh
```

在 Gazebo 中按住左键可查看地面坐标；使用 `Shift + 左键` 将选点发布到 `/move_base/current_goal`。SA-PPO 收到新目标后会继续控制车辆。

确认 SA-PPO 链路：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic info /pro3/rlscan
rostopic info /move_base/current_goal
rostopic info /cmd_vel
```

## 自主链路数据流

```text
LSTE 大脑 -> /lste/final_goal -> SA-PPO
Ouster 点云 -> /pro3/rlscan -> SA-PPO -> /cmd_vel -> Gazebo 底盘
```

## 健康检查与 CUDA 无重启恢复

### 不要只看 tmux 窗口是否存在

业务进程退出后，启动脚本会在窗口中保留一个交互 Bash。因此，tmux 中
`pane_dead=0` 只表示 shell 仍在运行，不代表 VLLM、Prompt、DINO 或 SA-PPO
仍然健康。`runall` 会在 `lste:health` 窗口启动常驻进程
`lste_health_audit.py`，持续检查 CUDA、关键 ROS 节点、话题发布者和消息新鲜度。

健康状态由 sidecar 原子写入 `runtime/health/status.json`，不向 ROS graph
发布额外的健康话题。使用短命令读取当前结果：

```bash
health
health --json
```

`health` 输出适合人工查看的摘要并通过退出码表示状态：健康为 `0`，其他状态为
`1`。`health --json` 输出完整 sidecar 状态，其中包含
`missing_nodes`、`unreachable_nodes`、`missing_publishers`、`stale_topics`、
`cu_init` 和恢复建议。健康检查不仅读取 ROS master 注册信息，还会主动连接
每个关键节点的 XMLRPC 端点，避免已经退出但仍残留注册信息的节点被误判为健康。
启动宽限期内状态为 `starting`，全部链路首次就绪后变为 `healthy`；之后任一
关键项丢失会变为 `unhealthy` 并在 health 窗口记录错误。`cuInit != 0` 或 ROS master
不可达属于致命故障，不受启动宽限期影响，会立即标记为 `unhealthy`。健康检查只报告并
保存恢复步骤，不会未经确认停止仿真或执行需要 root 权限的 NVIDIA 驱动模块重载。

也可手动检查 ROS 节点和话题：

```bash
rosnode list
rostopic info /lste/prompts
rostopic info /lste/detections
rostopic info /lste/scores
rostopic info /lste/cmd_vel/sappo
```

正常链路应满足：

- `/lste_prompt_node` 发布 `/lste/prompts`；
- `/lste_det_node` 持续发布 `/lste/detections`；
- `/lste_score_node` 持续发布 `/lste/scores`；
- `/StageEnv_0` 发布 `/lste/cmd_vel/sappo`；
- `/lste_det_vis_node` 发布 `/lste/det_vis_image`。

可采样实际消息，而不是只检查话题名称：

```bash
rostopic echo -n 1 /lste/detections
rostopic echo -n 1 /lste/scores
rostopic hz /lste/det_vis_image
```

VLLM 只负责首次生成任务 prompt。结果按任务 JSON 内容、prompt 模板和模型标识
缓存到 `runtime/prompt_cache/`。同一任务再次 `runall` 时，prompt 节点直接发布缓存，
`vllm` 窗口显示 `Prompt cache hit; MiniCPM was not started.`，不会加载 MiniCPM。

任务内容、prompt 模板或模型标识改变后，缓存键自动变化，MiniCPM 会启动一次并生成
新缓存。成功发布 `/lste/prompts` 后，VLLM 随即停止并释放显存给 GroundingDINO 和
SA-PPO。因此端口 `8000` 关闭是正常行为；判断标准应是 `/lste_prompt_node` 和
`/lste/prompts` 是否存在。

需要对同一任务强制重新生成时，删除本地缓存后重新启动节点：

```bash
rm -rf runtime/prompt_cache
stopall
runall
```

### 识别 CUDA 驱动初始化故障

典型现象是 VLLM 报 `CUDA unknown error`，随后 Prompt 报连接失败，DINO
没有 prompt，RViz 只有相机画面而没有检测框。即使 `nvidia-smi` 正常，CUDA
计算通道仍可能已经失效，因为 `nvidia-smi` 使用的管理接口与 PyTorch 使用的
CUDA Driver API 不同。

先检查最底层的 CUDA 初始化：

```bash
python - <<'PY'
import ctypes
print("cuInit =", ctypes.CDLL("libcuda.so.1").cuInit(0))
PY
```

预期结果为 `cuInit = 0`。如果得到 `999`，再检查两个模型环境：

```bash
conda run -n minicpm python -c \
  'import torch; print(torch.cuda.is_available())'

conda run -n dino python -c \
  'import torch; print(torch.cuda.is_available())'
```

若二者都是 `False`，问题位于 NVIDIA CUDA 驱动状态，不是 LSTE、模型文件、
RViz 或某一个 Conda 环境。该问题可能在系统 suspend/resume 后出现。

### 不重启系统，重新加载 `nvidia_uvm`

先停止所有 LSTE/Gazebo/模型进程，避免 CUDA 模块仍被占用：

```bash
stopall
```

只重新加载 CUDA Unified Virtual Memory 模块，不卸载桌面正在使用的
`nvidia`、`nvidia_modeset` 或 `nvidia_drm`：

```bash
sudo modprobe -r nvidia_uvm
sudo modprobe nvidia_uvm
```

重新执行验证，两个条件都必须满足：

```bash
python - <<'PY'
import ctypes
print("cuInit =", ctypes.CDLL("libcuda.so.1").cuInit(0))
PY

conda run -n minicpm python -c \
  'import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))'

conda run -n dino python -c \
  'import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))'
```

预期输出是 `cuInit = 0`、两个环境均为 `True`，并显示实际 GPU 名称。确认后再
启动完整系统：

```bash
runall
```

等待模型加载完成后确认输出链路：

```bash
rosnode list | grep -E '/StageEnv_0|/lste_prompt_node|/lste_det_node|/lste_score_node'
rostopic echo -n 1 /lste/detections
rostopic echo -n 1 /lste/scores
```

如果 `modprobe -r nvidia_uvm` 报模块正在使用，先用以下命令找出残留 CUDA
进程并停止它们：

```bash
fuser -v /dev/nvidia-uvm
```

若 `nvidia_uvm` 成功重载后 `cuInit` 仍不是 `0`，才需要将重启系统作为最后
恢复手段。

## 停止与重启

一键停止完整系统：

```bash
stopall
```

完整路径形式为 `./scripts/lifecycle/stop_all_tmux.sh`。

脚本按 `lste-teleop`、`lste`、`lste-env` 的顺序停止控制器、大脑和仿真环境，
并清理可能残留的独立 `sappo-lste` 会话。脚本可重复执行。

停止 SA-PPO 独立验证：

```bash
tmux kill-session -t sappo-lste
tmux kill-session -t lste-env
```

集成模式中的 SA-PPO 位于 `lste` 会话，直接再次执行 `./scripts/lifecycle/run_nodes_tmux.sh` 即可重启。修改 Gazebo GUI 插件后必须重启 `lste-env`，因为插件仅在 `gzclient` 启动时加载。

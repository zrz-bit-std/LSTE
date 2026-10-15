# 视觉链路性能与框跟随

更新日期：2026-07-21

提交记录：`perf(vision): smooth detection visualization and Gazebo rendering`

## 目标

提高 Gazebo/RViz 画面流畅度，并让检测框在 GroundingDINO 两次推理之间持续跟随图像。保持现有启动方式不变：

```bash
./scripts/lifecycle/run_env_tmux.sh
./scripts/lifecycle/run_nodes_tmux.sh
```

## 性能策略

- Kinect Gazebo 相机更新率为 30 Hz。
- 场景关闭阴影，避免不必要的 Gazebo 渲染负担。
- `GAZEBO_MODEL_DATABASE_URI` 默认指向工作区的本地空模型数据库，避免 Gazebo Classic 访问已退役的公网模型数据库而阻塞启动。
- GroundingDINO 的常用推理间隔为 1.5 秒；`EXHAUSTED` 状态为 3 秒。DINO 与 Gazebo、RViz 共用 GPU；限制推理频率为渲染留下稳定时间，不改变相机的 30 Hz 图像发布。

这些值集中配置在 `scripts/config/pipeline_defaults.yaml`：

```yaml
DINO_MIN_INTERVAL: 1.5
DINO_INTERVAL_PASS: 1.5
DINO_INTERVAL_SUSPICIOUS: 1.5
DINO_INTERVAL_LOCKED: 1.5
DINO_INTERVAL_EXHAUSTED: 3.0
```

`runall` 启动时会自动读取该配置，不需要在命令行追加参数。

## GroundingDINO 实测帧率

测试环境为 NVIDIA GeForce RTX 3060，当前每帧执行两次 GroundingDINO 推理（目标 prompt 和环境 prompt）。

| 调度方式 | 平均结果间隔 | 平均结果率 | 平均 GPU 利用率 | 峰值 GPU 利用率 |
| --- | ---: | ---: | ---: | ---: |
| 原配置，最小间隔 3.0 秒 | 3.074 秒 | 0.325 FPS | - | - |
| 不限速 | 1.284 秒 | 0.779 FPS | 82.8% | 100% |
| 推荐配置，最小间隔 1.5 秒 | 1.588 秒 | 0.630 FPS | 68.9% | 99% |
| CUDA 自定义算子，最小间隔 1.5 秒 | 1.759 秒 | 0.569 FPS | 93.0% | 100% |

不限速时的 `0.779 FPS` 是当前实现测得的最高持续结果率。推荐使用 1.5 秒调度：相比不限速仅少约 19% 的结果率，但明显降低平均 GPU 占用，为 Gazebo 和 RViz 留出余量。

`Failed to load custom C++ ops. Running on CPU mode Only!` 是上游的误导性警告：未编译 `_C` 时，本项目的 fallback 仍通过 PyTorch 在 CUDA 上计算，并非整套模型退回 CPU。2026-07-21 已补齐 CUDA 12.1 开发头文件并成功编译、执行自定义 CUDA kernel，警告消失。不过，在 RTX 3060 与 Gazebo/RViz 共用 GPU 的端到端测试中，自定义算子没有提高当前工作负载的吞吐率，反而提高了 GPU 占用。上表保留两组数据，后续优化时应按完整 ROS 链路复测，不能只以扩展是否加载判断性能。

## 检测框策略

`lste_det_vis_node.py` 的跟踪模式由 `DETECTION_TRACKING_MODE` 控制，默认 `auto`。它根据
统一的 `DETECTOR` profile 解析后的后端选择策略：

- `groundingdino` 自动使用 `legacy`，即保留原有行为：每次 DINO 结果到达时，在对应历史图像上初始化 OpenCV CSRT 跟踪器，再重放缓冲帧追到当前画面。
- `wedetect` 自动使用 `associated`：同一图像戳的重复结果会忽略；标签相同且 IoU 足够的检测框会关联到已有 CSRT 跟踪器，并以 EMA 小幅校正，不会每帧清空重建。未匹配的新框才新建跟踪器。
- 可以显式设为 `legacy` 或 `associated`，例如 `DETECTION_TRACKING_MODE=legacy runall`。该配置只影响 `/lste/det_vis_image`，不会更改 `/lste/detections` 或控制逻辑。

两种模式都使用以下视觉跟踪能力：

- 跟踪计算在 1/4 分辨率、每 3 帧执行一次，显示图像仍按相机回调发布。
- CSRT 跟随框的位置和尺度。
- 对远处小物体或水平转动时 CSRT 丢失/卡住的情形，使用相邻图像的稀疏 Lucas-Kanade 光流估计全局二维画面平移，作为短时纯图像兜底。该过程不读取车辆位姿、`/rbt_pose` 或 TF。
- 框实际触及图像边界并持续向外运动时，会逐步显示为被裁剪的部分框；接近完全出画或持续外移约 1.2 秒后移除。若画面运动反向，退出过程取消。
- 光流和 CSRT 都不能继续支持时，框会移除；下一次 DINO 结果重新初始化跟踪。

## 原始检测帧显示

当 `DETECTION_VISUAL_TRACKING_ENABLED: false` 且
`DETECTION_DISPLAY_SYNC_MODE: detection_frame` 时，可视化不使用任何跟踪、插值或
预测。每个检测结果只绘制在产生该结果的原始相机帧上，因此快速转动时框不会因推理延迟
画到错误的新画面。代价是 `/lste/det_vis_image` 按检测结果到达时更新，不再保持相机帧率。
全局目标投影也使用该检测帧的 TF 时间戳，而不是最新机器人姿态，因此不会在转动时相对
同一张历史图像漂移。

## 关键参数

这些参数均为 `lste_det_vis_node` 的 ROS 私有参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `tracking_scale` | `0.25` | 跟踪使用的图像缩放比例 |
| `tracking_update_stride` | `3` | 每多少个相机帧更新一次跟踪 |
| `max_tracking_age` | `4.0` | 单次 DINO 结果允许被跟踪的最长秒数 |
| `tracking_flow_max_lost_frames` | `30` | CSRT 丢失时允许光流兜底的更新次数 |
| `tracking_edge_exit_frames` | `12` | 框触边向外移动后最多保留的更新次数 |

## 限制

CSRT 和光流输出的是轴对齐矩形。旋转、剧烈运动模糊、画面缺少纹理或物体被遮挡时，短时跟踪可能失效；DINO 的下一次推理会重新校正框的位置和大小。

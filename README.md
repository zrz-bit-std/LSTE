# LSTE：视觉目标搜索与自主导航

本仓库包含 ROS/Gazebo 工作区及视觉语言模型相关代码，已完成 ENV04 场景中寻找黄杯的本地端到端复现。

请阅读 **[复现流程与实验操作指南](readme.md)**，其中包含系统设计、启动实验、切换 world 与任务、日志分析和轨迹可视化命令。

当前工作区入口为 `scripts/lifecycle/run_all_tmux.sh`，统一任务配置位于 `scripts/config/pipeline_defaults.yaml`。

旧版 `Data_exchange/`、`GroundingDINO/`、`MiniCPM/` 等顶层目录随原 main 历史保留；当前 ROS 工作区使用 `model/` 下的对应模块，实验操作以新指南为准。

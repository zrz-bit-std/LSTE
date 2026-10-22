# GP 主导的入口与分支发现（开发验收中）

## 范围与参考边界

修改目标为 `/home/zrz/work/lste_ws`。旧工作区
`/home/zrz/work/lste-previous/lste_ws` 只读参考，未向其中写入代码。
参考旧 GP 的方向候选、稳定方向簇与路口分支记忆；不启用其强制转向或回退控制。

## 实现

- `gp_branch_discovery.py`：水平点云投影 LaserScan 的周期角度 GP；对预测自由距离按多尺度连续方向扇区生成候选。后验方差保存为证据质量，不把方差直接等同于可通行空间。
- `gp_exploration_tree.py`：稳定 node/branch 身份、临时候选到已确认路口的细化、父分支与子空间、回到已有空间的连接、分支状态与语义选择。
- `gp_branch_frontier.py`：接入在线 SLAM、采样时刻的传感器 TF、入口截面测量、固定节点的地图重投影、标准路线事务与语义检测证据。
- `SelectedFrontier.branch_id`：随候选传递分支身份，避免预取或重选覆盖本轮执行身份。
- 下游继续通过 Goal Manager、Navfn、TEB 和 mux。`route_command.branch_context` 保存分支上下文。

地图仅细化和关联 GP 候选；地图范围不足时保留真实传感器 GP 方向。所有执行目标仍需通过已知自由空间的严格膨胀图和既有导航验证。墙体开口检查作为可选 `wall_confirmed` 证据；无门框的分支不因此被拒绝。

入口几何含位置、方向、测得宽度、两侧边界与可通行状态。未知边界的宽度为 null、状态为 unknown，不编造墙体。对普通门洞记录狭窄截面；对走廊分支记录可重复关联的进入截面，不宣称存在门框。

语义评分使用任务目标、上下文和负面标签，经相机内参及采样时刻 optical→odom 变换关联到分支。重复检测帧使用各类最大置信度，不按消息次数无限累加。语义评分优先，语义相同时使用已有语义位置提示或路径距离；没有语义证据时不伪造语义分值。

## 运行状态含义

- pending：已记录、未探索；stable 需不同传感器帧和持续时间支持。
- active：当前执行分支。
- entered：实际里程计已达到入口目的侧深度，才创建或复用子空间。
- explored：沿该分支推进到已观测死路；新路口则记录后续连接。
- return：指向父空间的反向连接，供返回使用，不重复当作新房间。
- retry：执行失败后保留身份、允许重新选择。
- superseded：远处尚未分开的方向假设被近处更完整结构替代，仅保留历史，不计为有效分支。

## 验证入口

```bash
source /opt/ros/noetic/setup.bash
cd /home/zrz/work/lste_ws
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s src/lste_topo_access/test -p 'test_gp_*py'
T_JUNCTION_EXPLORATION_METHOD=gp_branch_tree scripts/tests/targeted_navigation/run_t_junction.sh --persistent --max-seconds 130 --startup-seconds 90 --no-stop-on-failure
scripts/tests/targeted_navigation/run_gp_junction_suite.sh
```

每轮保留 metrics、失败现场、运行配置以及 source/ 和 source_manifest.json；批次执行固定副本的 runner，避免 Bash 运行中修改脚本。验收脚本 `check_gp_tree.py` 只读真实 metrics，不使用 world 真值生成候选。

## 已有证据与尚待完成

- 新 GP/树/语义及旧 GP 布局相关 20 项检查通过；catkin_make 成功。
- T 字开发轮次 `20261021_153426` 已观察到北、东实际探索，入口分别约 (2.05,1.30)、(2.94,0.19)；后续又发现返回连接需复用父空间，已修正，需以新冻结批次重新确认。
- L 字开发轮次 `20261021_153647` 的日志检查通过。
- 第一批十字与旋转 T 字结果不合格，不作为成功证据：十字缺少地图远距候选回退且 runner 在修改中异常退出；旋转 T 字使用该修复前版本。
- 冻结重验批次：`runtime/gp_branch_development/junction_suite_20261021_154453/`，仍在运行。
- 全包回归仍有基线失败。在相同 ROS 环境下，当前运行 1039 项（49 failures、45 errors）；HEAD 基线运行 1024 项（51 failures、55 errors）；当前失败条目集合未出现基线之外的条目。这不等于全包测试通过。

当前生产默认探索模式尚未切换；完成冻结多场景验收与结果审查后再切换。不将上述开发进展等同于 goal 已完成。

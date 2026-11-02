# Place 内 TSP 与 DFS 实景复现（开发中，尚未验收完成）

所有命令在 bit1010 的 `/home/zrz/work/lste_ws` 执行。当前仍在优先完成 T 场景验证，
不能将旧 1024 成功包或此前 TSP 成功包当作本轮四场景通过证据。

## 单个或四个既有场景

先确保现有诊断已结束；以下测试使用同一个 ROS master，不应并行启动。

```bash
cd /home/zrz/work/lste_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
SEMANTIC_CASES=semantic_t_junction GP_SUITE_SECONDS=3600 \
  bash scripts/tests/targeted_navigation/run_place_dfs_suite.sh
```

去掉 `SEMANTIC_CASES` 可依次运行 T、L、十字、旋转 T。旋转 T 使用原套件同款
`semantic_t_junction_focused_rotated.world` 和原初始姿态。原四个 world 不因此改写。
默认独立 master 为 11673，可用 `PLACE_DFS_MASTER_URI` 显式覆盖；不继承 ROS setup 的默认 11311。
当前入口先通过 `check_semantic_junction_goal.py` 的五组检查再结束正常窗口，
也可能因启动失败或超时结束，因此必须查看最终 `summary.log`，不能仅看进程退出。

## 证据与判定

结果位于 `runtime/gp_branch_development/place_dfs_suite_<timestamp>/`。
每个场景保存 runner、原始 SLAM/TF/姿态 bag、GP 树、观望认证、语义生命周期、TSP、
DFS 与独立地图空间审计结果，以及 Portal 认证 PNG 和 Place/DFS PNG。
原始进程日志在 `runtime/targeted_navigation/t_junction_small/logs/<timestamp>/`。

DFS 审计检查真实跨门绑定、有效入口宽度、当前 Place 的 WI/TSP 归属、子节点优先、
完整 WI/子树完成后沿原入口回溯、禁止完成后重入，以及根 Place 原有 WI 未被遗弃。
内部子 Place 还必须有注册前的 distinct_destination_component SLAM 确认证据。
地图审计读取 bag 精确地图时间戳，用独立四连通 BFS 重建区域后核对实际派发视点。

原四个环境可能只有根节点加一层真实子 Place，所以每个原场景要求完整遍历其实际树，
不强行制造不存在的子节点。完整 DFS 能力仍需额外真实嵌套路口环境达到深度至少 3；
当前该实测证据尚未完成，不能用合成审计 fixture 替代。
审计器独立调用默认仍要求深度 3：

```bash
python3 scripts/tests/targeted_navigation/audit_place_dfs.py <metrics.log> \
  --output <dfs_audit.json>
python3 scripts/tests/targeted_navigation/audit_place_dfs_geometry.py <metrics.log> <slam.bag> \
  --output <geometry_audit.json>
```

`INCOMPLETE` 图和失败 JSON 属于诊断证据，不应包装成成功结果。

## 已定位的第五轮失败

`runtime/gp_branch_development/place_dfs_20261025/t_fifth_missing_width_replay.json`
使用第五轮原始 SLAM 与 TF 回放，证明 branch16 当时缺少 map width，原代码跳过空间确认。
用原 GP 扇区 reach/angular_width 的横向弦长恢复宽度后，源与进入点仍在同一已知自由连通域，
应拒绝 `known_free_bypass_within_current_place`。该轮错误三层树不算真实 DFS 成功。


## 补充真实嵌套 DFS 场景

`semantic_nested_place_dfs.world` 保留独立根 T，在东侧走廊 x=10m 再设一个真实 T，
共四个带物理家具的房间。原四场景不修改。几何检查已验证所有模型可解析、出生点与
四个房间入口在 0.40m 车身净空下连通；这只证明布局，不代表导航和 DFS 已通过。

```bash
python3 scripts/tests/targeted_navigation/generate_nested_place_dfs_world.py
SEMANTIC_CASES=semantic_nested_place_dfs GP_SUITE_SECONDS=3600 \
  bash scripts/tests/targeted_navigation/run_place_dfs_suite.sh
```

该场景自动要求真实 DFS 深度至少 3；不能以根加一层代替通过。


## Current development checkpoint (17:20)

This is still an incomplete development run, not a four-world success package.
Latest T suite: `runtime/gp_branch_development/place_dfs_suite_20261025_172016`.
Raw run: `runtime/targeted_navigation/t_junction_small/logs/20261025_172016`.
Dedicated master: `http://127.0.0.1:11677`.

Portal partitioning projects the measured finite gate section into each current
SLAM map. If a projected endpoint is inside known free space after local SLAM
wall correction, it can extend outward along the same section to the first
non-free cell, by at most half the measured opening width. If no boundary is
found in that range, the endpoint is unchanged and the bypass guard remains
active. Each scope records the measured endpoints, applied endpoints and
extension distances. This changes WI spatial ownership only; it does not add
a corridor/room classifier or modify the locked observation turn policy.

Actual tenth-run map replay: `runtime/gp_branch_development/place_dfs_20261025/t_tenth_refined_cut_replay.json`
(previous leak `[1]`, corrected leak `[]`, endpoint extension about 0.2m).
Independent tenth-run raw-bag audit: `t_tenth_raw_geometry_audit.json` under
the same checkpoint directory (PASS, including 5 inside-frontier records).
The tenth navigation itself remains incomplete/failed because the uncorrected
bypass guard blocked further WI dispatch; a geometry audit PASS is not a
whole-scene PASS.

Latest focused tests: 295 PASS plus one separate actual ROSbag audit regression.

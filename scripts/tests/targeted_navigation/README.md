# 语义房间场景 v2：T / L / 十字复现

实验室电脑：`ssh -4 zrz@bit1010.local`。工作区：`/home/zrz/work/lste_ws`。

## 新版场景

本版替换了只有短走廊的旧版场景。中央路口外有宽 3 米、长 12 米的走廊分支，西侧入口走廊长 14 米；各分支尽头通过宽 2.2 米的门口连入 8×8 米的封闭房间。家具使用实验室现有模型，保留真实 collision，不使用旧版 visual-only 副本。

| World（位于 `worlds/targeted_navigation/`） | 房间 | 物件实例数 |
| --- | --- | --- |
| `semantic_t_junction.world` | 西侧休息室、东侧办公室、北侧会议室 | 39 |
| `semantic_l_junction.world` | 西侧办公室、北侧茶水间 | 27 |
| `semantic_cross_junction.world` | 西侧休息室、东侧办公室、北侧会议室、南侧茶水间 | 50 |

物件复用现有 world 中的 desk、office_chair、monitor、keyboard、mouse、computer_case、cup、bookshelf、file_cabinet、file/file_box、conference_table、sofa、coffee_table、cafe_table、kitchen_chair、book、plant 等。每个房间至少 10 件物件，按房间主题布置，家具碰撞轮廓均位于房间内。

每个 world 同目录有：

- `<场景名>.json`：房间边界、门宽、走廊尺寸、起点、语义模型及位置。
- `<场景名>_layout.png`：带物件名称的布局图。
- `<场景名>_scene.png`：包含墙体和真实家具碰撞轮廓的完整场景概览。
- `<场景名>_geometry_result.json`：模型无缺失、家具在室内、每房间至少 10 件物件、从起点到每个房门内侧的 0.4 米净空路径连通性检查。

## 验证范围

运行当前 GP 候选发现 + 在线 SLAM 去重 + 门洞观望期 + persistent Navfn/TEB。验收中央路口的方向、稳定候选身份、逐分支进入、门内 frontier、固定方向转身、回到原 node 和观望期完成。

当前策略完成中央路口观望后停在 node 等待决策。**此测试不声称机器人已走遍远端所有房间**：远端真实房间已建模、物理家具已加载，并通过静态门口连通性检查；整图探索和 WeDeT 语义识别/打分不在本轮验收范围内。

## 运行准备

```bash
ssh -4 zrz@bit1010.local
bash
cd /home/zrz/work/lste_ws
source /opt/ros/noetic/setup.bash
```

代码改过需要构建时执行 `catkin_make`。仅复现已有构建可跳过。runner 会通过 `scripts/config/pipeline_env.sh` 加载当前工作区和 supporting workspace。

模型的 visual/collision 定义已内联进 world，但 mesh/material 仍引用 `~/.gazebo/models/` 的原始模型资源，不能只复制三个 world 就在新机器运行。旧参考工作区 `/home/zrz/work/lste-previous/lste_ws` 保持只读。

启动前检查是否已有其他实验占用 ROS/Gazebo：

```bash
pgrep -af 'gzserver|roslaunch|run_semantic_junction_suite.sh'
```

若有旧 suite，在原终端正常退出后再运行。**必须串行运行，两个 suite 不能共用同一 ROS master 和 Gazebo。**

## 重新生成 world 与静态几何检查

```bash
cd /home/zrz/work/lste_ws
python3 scripts/tests/targeted_navigation/generate_semantic_room_worlds.py
```

生成器会重写三个 world、JSON 和场景 PNG，并检查每个房间入口的静态连通性。几何检查失败会报错。模型需已安装；依赖已有 Python 3、NumPy、Matplotlib，不需额外安装 SciPy。

可选 SDF 检查：

```bash
export GAZEBO_MODEL_PATH="$PWD/worlds/targeted_navigation/models:$HOME/.gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_DATABASE_URI="file://$PWD/worlds/.gazebo_model_database"
for world in worlds/targeted_navigation/semantic_*.world; do
  timeout 30 gz sdf -k "$world"
done
```

## 全套与单场景运行

```bash
cd /home/zrz/work/lste_ws
GP_SUITE_SECONDS=900 T_JUNCTION_GUI=false \
  bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh
```

顺序 T → L → 十字；每个场景默认运行窗口 900 秒、启动就绪等待上限 90 秒，另有退出、审计和绘图开销。可用 `GP_SUITE_SECONDS=600` 调整运行窗口；时间不足导致未完成时，应看具体状态而非直接认定场景失败。需要 Gazebo GUI 时设 `T_JUNCTION_GUI=true`。

起点为 `(-3.5,0,yaw=0)`，中央 junction 为 `(0,0)`。长走廊/真实房间几何不会因运行窗口改变。建议在远端 tmux 内运行以支持 SSH 断开。

单场景：

```bash
SEMANTIC_CASES=semantic_t_junction GP_SUITE_SECONDS=900 T_JUNCTION_GUI=false \
  bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh

SEMANTIC_CASES=semantic_l_junction GP_SUITE_SECONDS=900 T_JUNCTION_GUI=false \
  bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh

SEMANTIC_CASES=semantic_cross_junction GP_SUITE_SECONDS=900 T_JUNCTION_GUI=false \
  bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh
```

`SEMANTIC_CASES` 也可设为加引号、空格分隔的多个场景名。

## 结果与通过标准

runner 第一行输出本轮目录：

```text
/home/zrz/work/lste_ws/runtime/gp_branch_development/semantic_junction_suite_YYYYMMDD_HHMMSS
```

```bash
RESULT_ROOT=/home/zrz/work/lste_ws/runtime/gp_branch_development/semantic_junction_suite_YYYYMMDD_HHMMSS
cat "$RESULT_ROOT/summary.log"
```

每个场景应出现 `launch=0 tree=0 auth=0 visual=0`；各 JSON 的 `passed` 应为 `true`。不要仅依据整个 shell 脚本的退出码判断通过，runner 会继续执行后续场景。

- `*_tree_result.json`：预期中央路口方向、无重复稳定方向、入口几何、实际进入及子空间、junction 身份。
- `*_authentication_result.json`：只对已确认 `junction_confirmed` 的路口审计。所有稳定候选均处理、真实重复启动计数为零、进入/门内 frontier/固定转向目标/回 node 证据完整、观望期完成、SLAM 去重审计存在。
- 父 node 的已知返回走廊标记为 `return`，不需要当成新门洞再次观望；中央方向数不等于新门洞认证数。
- `*_portal_authentication.png`：实际路线、frontier、门洞树和观望认证、SLAM 去重记录。全景房间及家具布局另看 world 同目录的 `*_scene.png`。
- `*_visualization.log`：绘图检查及资源缺失信息。

原始数据位于 summary 的 `run=` 目录；历史路径仍是 `runtime/targeted_navigation/t_junction_small/logs/<timestamp>/`，即使运行 L/十字也使用该目录名。目录包含 metrics、launcher、lifecycle、failure report 和失败快照。

## 已有 metrics 重新审计、绘图

```bash
cd /home/zrz/work/lste_ws
RUN_DIR=/home/zrz/work/lste_ws/runtime/semantic_rooms_v2/20261022_170952/semantic_t_junction/logs
STAMP=20261022_170952
METRICS="$RUN_DIR/${STAMP}_navigation_metrics.log"
OUT_DIR=/home/zrz/work/lste_ws/runtime/gp_branch_development/semantic_rooms_v2_recheck
mkdir -p "$OUT_DIR"

# T：中央方向为东、北、西。
python3 scripts/tests/targeted_navigation/check_gp_tree.py "$METRICS" \
  --headings 0 1.57079632679 3.14159265359 --center 0 0 \
  --output "$OUT_DIR/semantic_t_junction_tree_result.json"
python3 scripts/tests/targeted_navigation/check_portal_authentication.py "$METRICS" \
  --output "$OUT_DIR/semantic_t_junction_authentication_result.json"
python3 scripts/tools/plot_portal_authentication.py "$METRICS" \
  --world worlds/targeted_navigation/semantic_t_junction.world --route-zoom \
  --model-root "$HOME/.gazebo/models" --model-root "$PWD/worlds" \
  --output "$OUT_DIR/semantic_t_junction_portal_authentication.png"
```

L 的 headings 为 `1.57079632679 3.14159265359`；十字为 `0 1.57079632679 3.14159265359 -1.57079632679`，center 均为 `0 0`。相应替换 run/world/输出名。必须使用与 metrics 对应版本的 world 绘图，不能把旧短走廊 metrics 叠到新版房间几何上。

## 新旧结果区分

新版最终成功归档：`runtime/semantic_rooms_v2/20261022_170952/`。每个场景下有 `scene.png`、`authentication.png`、`checks.log`、`configuration.log` 和 `logs/`；总验收见 `summary.log`。

旧版短走廊结果 `semantic_junction_suite_20261022_150410` 仅为历史记录，不适用于新版几何。本轮失败尝试、重试产物及旧 world 临时副本已清理，成功原始日志集中在上述最终目录。

## 常见问题

- model/mesh 缺失：保留原 `~/.gazebo/models/` 模型资源；不要依赖已退役的在线 Gazebo 模型库下载。
- 候选或认证未完成：检查运行窗口、原始 failure report、auth JSON 的未完成分支，以及是否存在并行 ROS/Gazebo。
- Gazebo 异常退出：保留 launcher 和 failure report；可以对完整 metrics 独立审计/绘图，但绘图成功不代表行为通过，也不代表进程无异常。
- 仅有中央观望路线：这是当前停止等待决策策略的预期范围；远端房间完整几何可看 `*_scene.png` 和几何连通性 JSON。

## 长场景中补充的逻辑与回归

融合候选按已合并方向继续去重，junction 确认按独立方向计数；认证期间固定物理入口平面，并保持源 node 的路线所有权；返回遇到短暂无进展可在有界窗口内重试同一任务；物理返回完成时先释放旧 TEB lease、等待控制器确认，再推进下一支。审计只统计已确认 junction，并在形成唯一 ID 前检查重复启动。GP/树/认证回归 29 项、审计回归 2 项。动态验收包括完成后连续 30 秒停车。

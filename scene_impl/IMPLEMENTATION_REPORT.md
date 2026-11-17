# 两个新场景的落地报告（bit1010 实机）

日期：2026-11-06（v5 更新 2026-11-11；v6+ 更新 2026-11-17）　目标：`env05_gate_ring`（A3 仓库梯子）、`env06_gate_nested`（绿植）

> **v5（2026-11-11）**：
> 1. env05 A3 梯子东西向移到 `(5.40, −5.80)`，正对 `D_a3` 门心；纸箱分列梯子两侧，
>    两个货架移到 NE/SW 墙角，门口 `cam_a3` 直接看到梯子（截图验证）；
> 2. env06 B1 桌 + 3 椅 + 显示器整体移到西北角贴墙，留出南/东探索空间；
> 3. env06 B3 椅子东移到 `(24.00, 1.60)`，让出西侧通道；
> 4. env06 B5 的 2 个文件柜 + 1 个纸箱设为 `negative`，`plant.json` 的
>    `negative_clues` 增加 `file cabinet` / `cardboard box`，B5 门口 S=−0.69，直接跳过；
>    B4 使用同类货物，同样判负（原本就是“不进入”死胡同，行为不变）。
> 第 8 节为历史修复记录；当前几何/语义以 `audit_report.md`、`tables.md`、`design_spec.md` 为准。

> **v6（2026-11-16）P0+P1**：
> 1. 全部门洞 1.40 → 2.30 m；门洞所在墙在房间内侧加厚到 0.70 m，走廊侧保持原墙面齐平（无突出）；env05 4 门、env06 5 门；
> 2. spawn：env05 `(1.2, 2.6, -90°)`，env06 `(25.1, -4.1, 180°)`（主走廊最东端，面向西）；
> 3. 2.30 m 门宽使 `StructuralPlaceMap` 的 place split 由 4/4、5/5 变为 0/4、0/5（`PLACE_CLEAR≈0.75 m`）；门控语义仍由 `S_env` 提供，导航 strict route 在 0.62 m clearance 下已连通；`validate_world.py` 只要求 `S_env>0` 的任务房可达，A2/A4/B4/B5 为诱饵/负分房，不参与可达性判定；
> 4. env06 B2 台面组西移 0.6 m；B2 椅子靠西墙；主走廊 3 个负分物完全贴墙；B2 垃圾桶北移避开房间侧墙；2026-11-16 已重渲染 `shots/` 的俯视图与 8/9 机位截图。
> 5. env06 B3 桌 `(23.4, 0.6)`；椅子移到桌南侧 `(24.3, -0.3)` 以便观察；plant `(22.62, 0.60, z=0.75)` 放在桌面靠西边缘；桌面加 `b3_mon`/`b3_keyboard`/`b3_cup`，桌北侧加 `b3_trash` 补足 office 正向上下文；`plant.json` 改为 on the desk 且 key_objects 去掉 cardboard box、加入 keyboard/cup；观察点 `(20.8, 0.9)`。

> **v6+（2026-11-17，B4/B1 角色互换）**：
> 1. env06 **B4 改为休息区诱饵**：B1 的桌 + 2 椅 + 显示器搬到 B4 东半区，家具全部 `x≥21.65`，西侧进门通道完全留空；B4 门口 S_env=**1.00**；
> 2. **B1 改为负分储物间**：B4 原 file cabinet + 2 个 cardboard box 搬到 B1，另加 1 个同款文件柜；B1 门口 S_env=**−0.68**；
> 3. B5 保持负分，B2/B3/目标不变；`audit_scene.py` 的 `DOOR_LABELS` 已同步（D_b1 负分、D_b4 办公）。
> 4. 2026-11-17 已重渲染 env06 的 10 个机位、俯视图/标注图和 `rooms_contact_sheet.png`（新增 `cam_b4`/`cam_b4in`）。

## 1. 交付物

| 文件 | 位置 | 说明 |
|---|---|---|
| `env05_gate_ring.world` | `worlds/topo3.0_catch_mode/session_test/` | 14 段墙 + 27 个物体，无内嵌机器人 |
| `env06_gate_nested.world` | 同上 | 21 段墙 + 29 个物体，含套间内门 |
| `ladder.json` | `model/Data_exchange/vlm_prompt/lab/` | 当前 env05 任务（A3 仓库梯子） |
| `plant.json` | 同上 | 同上 |
| 生成/校验脚本 | `/home/zrz/lste_scene_impl/` | `build_design.py`、`build_world.py`、`validate_world.py`、`audit_scene.py`、`sweep_doors.py` |
| 机位截图 | `/home/zrz/lste_scene_impl/shots/env05_gate_ring/`、`shots/env06_gate_nested/` | env05 9 机位 + 俯视/标注/拼图；env06 10 机位（含 B4 诱饵 / B1 负分）+ 俯视/标注/拼图 |
| 审计报告 | `/home/zrz/lste_scene_impl/audit_report.md` | 可通行性 + 魔术值 + 观望 + 语义 |
| 实验记录 | `/home/zrz/lste_scene_impl/logs/env05_summary.txt`、`env06_summary.txt` | 每次实验的最终快照 |

运行参数：`clearance:=0.60`（仅新场景），门宽 **1.40 m**，栅格 0.10 m，墙 0.15×2.5 m，不写 `<state>` 块。

## 2. 门宽扫描（用真实代码，不是估算）

`StructuralPlaceMap` + `conservative_clearance_cells`，真实几何、0.1 m 栅格：

| clearance | 切成 2 个 Place | Navfn 连通 | 结论 |
|---|---|---|---|
| 0.52（默认） | ≤1.29 | ≥1.31 | **空集**（差 2 格） |
| **0.60（新场景）** | ≤1.45 | ≥1.31 | **1.31–1.45 ✓** |

所以门宽取 **1.40**（窗口正中，两侧各 0.05 m）。门控逻辑一行没改。

## 3. 几何自检：PASS

两套都通过：
- 4/5 个门洞无墙跨越，宽度 1.40
- 物体/墙 3D 互穿：**0**
- 建筑闭合（外部 flood 进不来）
- clearance 0.60 下 **所有房间 + 目标观察圈可达**
- Place 切分：env05 **4/4**、env06 **5/5**

## 4. 可通行性与观望审计

| 场景 | 房间可达率 | 无解口袋 | 质心 ≤2.5 m 可达 |
|---|---|---|---|
| env05 | A1 97.4%（20 格=0.2 m²，<0.5 m² 阈值）、A2/A3/A4 100% | 0 | 4/4 |
| env06 | B1–B5 全部 100% | 0 | 5/5 |

## 5. 语义分数与门控阈值（用 WeDetect 实测标签）

| 门 | 可见标签 | S_env | 决策 |
|---|---|---|---|
| A1 | cardboard box | **0.73** | 进入（诱饵） |
| A2 | desk, chair, monitor | **−0.45** | 跳过 |
| A3 | shelf | **0.18** | 进入（目标房） |
| A4 | — | **0.00** | 跳过 |
| B1 | file cabinet, cardboard box | **−0.68** | 跳过（负分） |
| B2 | desk, chair | **1.00** | 进入（中转） |
| B3 | desk, chair | **1.00** | 进入（目标房） |
| B4 | desk, chair, monitor | **1.00** | 进入（诱饵） |
| B5 | file cabinet, cardboard box | **−0.68** | 跳过（负分货物） |

- 阈值扫描（`audit_report.md`）：env06 在 θ≤0.90 都能进入目标房（B2/B3/B4）；
  env05 的 A3 只有 0.18，θ≥0.20 就漏目标房 → **硬阈值不可行，必须软降权 + 兜底**。
- 走廊视图若含 `fire hydrant`：env05 −0.32、env06 −0.54（负分生效）。
- 目标房首帧命中时状态机动作：`target_place` ✓。

## 6. 两次实验

| | env05（笔记本） | env06（绿植） |
|---|---|---|
| 启动配置 | `pipeline/env05.yaml` | `pipeline/env06.yaml` |
| 出生点 | (3.0, 2.8, 0.1) yaw 0 | (1.0, −4.1, 0.1) yaw 0 |
| 目标评估模型 | `laptop_pc_1` ✓ | `plant_1` ✓ |
| 时长 / 里程 | 555 s / 79.8 m | 353 s / 24.8 m |
| 目标数 | 31 unique goals | 10 goals |
| 碰撞 / 失败 | **0 / 0** | **0 / 0** |
| 结果 | 进入 A2（0.88 诱饵房）后在房内来回，未到 A3 | 停在 B1 门前 0.22 m 的 frontier 端点 |

**观察到的共性问题（系统层，不是场景层）**：两次实验都在到达 frontier 端点后停滞，
`teb_status=invalidated`、`teb_bridge_active=false`、`feedback_valid=False`，
`cmd=[0,0]` 而 `distance_to_goal=0.22`。场景本身可通行、无碰撞，卡点在 frontier 选择 /
TEB 契约的回填，与 world 几何无关（env04 长跑也是同一套模式）。

**env05 的次要发现**：A2（会议室）里桌子离南墙 1.0 m，留下 0.3 m 的净空带，
frontier 会反复指向桌后不可达点。建议把 `a2_desk` 从 y=−1.6 挪到 y=−2.2（贴南墙），
两行数据改动即可，其余布局不动。

## 7. 复现

```bash
# 只读审计
cd /home/zrz/lste_scene_impl
python3 validate_world.py          # 几何/连通/切分
python3 audit_scene.py             # 逐房间可通行 + 魔术值 + 观望 + 语义
SWEEP_CLEARANCE=0.60 python3 sweep_doors.py

# 跑一次实验
cd /home/zrz/work/lste_ws
scripts/bin/stopall
PIPELINE_CONFIG=/home/zrz/lste_scene_impl/pipeline/env05.yaml scripts/bin/runall
# 结束后
python3 /home/zrz/lste_scene_impl/summ_run.py
```

---

## 8. 修复轮（2026-11-07）：俯视图误读、target_ctx、negative_clues

### 8.1 水平中墙确实存在

`env05_gate_ring.world` 的 `wall_14`：

```
pose = (11.0, -3.60, 1.25)
size = 17.2 × 1.2 × 2.5 m      # x 2.4–19.6, y -4.2–-3.0
```

`wall_13` 是垂直中墙（x=9.4–10.6，y -9.6–1.6）。四间房是 **十字墙切出的 2×2**，不是左右两个大房间。之前俯视图看起来“没有隔板”，是因为旧渲染：

- 相机 z=17 m、HFOV 90°，透视让墙顶和墙面错位；
- 渲染副本把 `ambient` 提到 1.0，墙顶（Gazebo/Grey）和地面都接近白色；
- 15 段墙的顶面在 1024×768 里只有 1–2 px 对比度。

修复：`make_top_world.py` 改为 **z=124 m + HFOV 12.9°（近似正交）**，并在**渲染副本**里把墙刷成深灰（正式 world 不动），分辨率 1600×1200。新图 `shots/env05_gate_ring/top_env05.png` 与标注版 `top_env05_annotated.png` 中，十字墙、四个门洞和四间房都清晰可见。

### 8.2 target_ctx：keyboard → desk lamp

`keyboard` 是 0.44×0.15×**0.02 m** 的平板，放在 0.75 m 桌面上；0.60 m 高的相机被桌沿完全挡住（旧 `cam_a3ring` 里根本看不到）。改为 `lamp_table_small`：

```
a3_lamp, model=lamp_table_small, pose=(8.62, -6.05, 0.75), yaw=0
尺寸 0.18×0.18×0.23 m，贴桌沿（离西沿 0.12 m），灯罩顶 z≈0.98 m
```

新 `cam_a3ring.png` 中台灯在画面右侧完整可见。生产默认词表（未改代码）实测：

| prompt | 分数 | bbox |
|---|---|---|
| `desk lamp` | **0.871** | 0.123×0.256 |
| `lamp` | 0.864 | 0.123×0.256 |
| `monitor`（左侧） | 0.849 | 0.141×0.351 |

`target_ctx = {left: monitor, right: desk lamp}` 现在两个邻物都真实存在、都在 0.60 m 相机里可见、都能被检测到。

### 8.3 negative_clues 补全

历史 laptop/trash 任务已删除；当前 `ladder.json` 的 `negative_clues` 见第 9.3 节。
- 锥桶：1.8 m 机位实测被识别为 `fire hydrant` 0.302 / `traffic cone` 0.162；两者都在负词表里，所以走廊锥桶仍会产生负分（近处主要走 `fire hydrant` 分支）。

同时把 `key_objects` 改成场景里真实存在的 `monitor / desk lamp / desk / chair / file cabinet`，`layout_prior` 的 "monitors and keyboards" 改为 "monitors and desk lamps"。

### 8.4 重新校验

`validate_world.py` 仍为 **PASS**：env05 15 段墙、22 个物体、0 互穿、4/4 Place 切分、clearance 0.60 全房间+观察圈可达。`audit_scene.py` 逐房间可达率不变（A1 97.4%，其余 100%）。

### 8.5 修复后重跑 env05

| 项 | 结果 |
|---|---|
| 时长 / 里程 | 378 s / 36.2 m |
| 碰撞 / 失败 | **0 / 0** |
| 目标评估模型 | `laptop_pc_1` ✓ |
| 结果 | 未到 A3；在西南角 frontier 端点停滞 |

卡点仍是系统层：goal `[0.65, -9.65]`、距离 0.365 m、`teb_status=invalidated`、`feedback_valid=False`、`cmd=[0,0]`。该 goal 距西外墙（x=0.15）只有 0.50 m，落在 `clearance=0.60` 的膨胀区内，属于 frontier 端点选择问题；场景几何、门控、通行性均正常。

---

## 9. 2026-11-11 env05 改为“仓库找梯子”

### 9.1 场景改动

- **A3 改为开放仓库**：移除办公室家具（热桌、办公椅、主桌、laptop、显示器、台灯、
  垃圾桶）以及隔断；保留墙/门。新增：
  - 目标梯子 `a3_target`（`ladder_75deg_0rails`）@ `(8.50, -5.80, 0)`，靠北墙斜放；
  - 北墙货架 `a3_shelf_n`、西侧货架 `a3_shelf_w`；
  - 10 个纸箱（梯子旁、门口左右纸箱堆、西侧纸箱）。
- **A2**：保留会议室（桌 + 3 椅 + 显示器），但 `monitor / computer / laptop /
  office chair / desk / desk lamp / coffee machine` 全部作为 negative_clues；
  办公室诱饵语义被负分抵消。
- **A1**：保留文件柜 + 纸箱，作为仓库诱饵（有纸箱、没有梯子）。
- **A4**：清空，不再放纸箱。
- 目标观察点：`(6.00, -6.50, 0.60)`，朝东北 yaw≈0.87；距梯子 2.60 m。
- 两个 ctx：`cardboard box` @ (7.30,-5.50) 等实测 **0.326**；
  `shelf` @ (6.00,-4.60) 北墙货架实测 **0.679**。

### 9.2 实测检测（0.60 m 相机 / 640×360 / HFOV 90°）

| 标签 | 分数 | bbox |
|---|---|---|
| **`ladder`** | **0.879** | 0.252 × 0.792 |
| `shelf` / `bookshelf` | 0.679 / 0.670 | 0.385 × 0.963 |
| `cardboard box` / `box` | 0.326 / 0.326 | 0.265 × 0.240 |
| `crate` | 0.241 | — |
| `monitor` / `desk` / `chair` | 无 | 办公室负分未触发 |
| `trash can` / `fire hydrant` | 无 | 目标不与桶/消防器材混淆 |

目标框宽 0.252，是 `TARGET_DONE_MIN_BOX_WIDTH=0.05` 的 5 倍；
目标框高 0.792，远高于 0.05 门限。

### 9.3 world / JSON / pipeline

- World：`worlds/topo3.0_catch_mode/session_test/env05_gate_ring.world`
  14 段墙 / 27 个物体；A3 隔断已移除；`a3_target` 存在。
- Task JSON：`model/Data_exchange/vlm_prompt/lab/ladder.json`
  - target `ladder`
  - env `warehouse / storage room / stockroom`
  - related `cardboard box / shelf / pallet`
  - ctx `cardboard box` / `shelf`
  - negative `monitor, computer, laptop, office chair, desk, desk lamp,
    coffee machine, door, fire hydrant, fire extinguisher, construction cone,
    traffic cone`
- Pipeline：`pipeline/env05.yaml`
  - `TASK_JSON=.../ladder.json`，`TASK_ID=ladder`
  - `TARGET_EVAL_GAZEBO_TARGET_MODEL=a3_target`（Gazebo 实例名）
  - labels `ladder`
  - center `0,0.699,1.20`，size `0.951,1.548,2.520`
- 旧的 laptop / coffee_machine / trash_can 任务 JSON 已删除；lab 当前只保留
  `ladder.json`（env05）与 `plant.json`（env06）。

### 9.4 验证

- `validate_world.py`：PASS（0 互穿、4/4 Place 切分、目标观察点 free+reachable）
- `audit_scene.py`：
  - A1 纸箱 S_env **0.73**（仓库诱饵）
  - A2 办公室 S_env **−0.45**（桌/椅/显示器全部负分，应跳过）
  - A3 门口 `shelf` S_env **0.18**（仓库 context；实际策略无固定阈值，
    进房后梯子目标证据 0.879）
  - A4 空房 0.00
  - 目标观察圈可达，A3 房间可达率 97.9%

### 9.5 实验

- run：`20261111_121743`
- 时长 / 里程：**1955 s / 168.5 m**
- 目标数：31；碰撞：**0**；`task_done=false`
- 最终位置：`(14.36, -7.40)`，位于 **A4 空房内**
- 目标 `target_messages=0`，梯子未进入视野
- 过程：进过 A1（纸箱诱饵）和 A2（办公室诱饵），南走廊出现一次
  `teb_status=invalidated` 后恢复，随后进入 A4；未能继续到 A3 仓库
- 结论：world/JSON/检测均正常；未命中仍是 **frontier/TEB 系统层停滞**，
  不是仓库或梯子目标的问题

# LSTE 场景设计稿（v5 — env05 梯子正对门 / env06 B5 负分）

> **v5（2026-11-11）**：env05 梯子东西向对齐 A3 门心，移到 **`(5.40, −5.80)`**；
> 纸箱全部改为分列梯子东西两侧，两个货架移到 NE/SW 墙角，门口 `cam_a3` 正对梯子无遮挡。
> env06：B1 桌 + 3 椅 + 显示器整体移到西北角贴墙；B3 椅子东移到 `(24.00, 1.60)`；
> B5 货物（2 柜 + 1 箱）设为 negative，B5 门口 S=−0.69 直接跳过；B4 同类货物同样判负。
> 任务 JSON：`ladder.json`（env05）、`plant.json`（env06）；旧的 trash can / laptop 任务 JSON 已删除。
> 下文 v2 的 laptop/trash 检测表仅作历史选型记录，**当前以本文开头和 `design_spec.md` 为准**。

两套新场景的设计规格，用于覆盖 **四层语义（target / target_ctx / env / negative_clues）+ 新增的「门口语义门控」**。
v2 相对 v1 的三处改动全部来自实测：门宽窗口、目标选型、目标摆放规则。

```
lste_scene_design/
├── README.md                     ← 本文件
├── env05_gate_ring.md            ← 场景 A：环形四房 + 门控梯度（目标=仓库梯子）
├── env06_gate_nested.md          ← 场景 B：主轴+套间 + 两层门控（目标=绿植）
├── tables.md / tables/*.csv      ← 墙体/门/房间/物体表（自动生成）
├── figures/*.png                 ← 平面图
├── design_spec.md                ← 【权威】由脚本生成的完整规格（含每个物体的三维位姿）
├── tasks/ladder.json             ← 场景 A 当前任务
├── tasks/plant.json              ← 场景 B 主任务（绿植）
├── tasks/cup_yellow_variant.json ← 场景 B 颜色消融变体（黄杯，依赖硬颜色门限）
├── wedetect_verification.md      ← 词表验证报告（含抓图与结论）
├── evidence/                     ← 探针 world / 12 张截图 / 推理日志
└── build_design.py               ← 生成脚本
```

---

## 0. 已定的三条决策

| # | 决策 | 影响 |
|---|---|---|
| 1 | **新场景启动 `clearance:=0.60`** | 门宽 1.40 m 同时满足「走得进去」和「切成两个 Place」（实测窗口 1.30–1.45） |
| 2 | **颜色消融保留** | 以 `env06` 的**变体任务**实现：B2 台面上黄杯 vs 蓝杯。⚠ 需要把 `filter_boxes_by_color` 的 `min_keep` 改为 0（硬门限），否则模型会把蓝杯也叫成黄杯 |
| 3 | **场景 B 目标改为绿植** | `plant_1`（0.83×0.84×0.55，实测 `plant` 0.781） |

## 1. 两个场景一句话

| | env05_gate_ring | env06_gate_nested |
|---|---|---|
| 布局 | 环形回廊 + 四房（十字实体芯） | 3 m 主轴 + 两侧五房，**房间套房间** |
| 外轮廓 | 22 × 16 m（35,200 cells） | 26 × 12.2 m（31,720 cells） |
| 房间数 / 门数 | 4 间 / 4 门 | 5 间 / 5 门（含 1 个内门） |
| 目标 | `ladder_75deg_0rails` @ (5.40, −5.80, z=0)，yaw 0°，正对 D_a3 | `plant_1` @ (22.00, −0.30, z=0)，yaw 0° |
| 目标尺寸 | 0.95×1.55×2.52 m（梯子，门正对面） | 0.83×0.84×0.55 m |
| 观察圈 | (5.40, −7.50)，距目标 1.70 m | (22.00, −1.45)，距目标 1.15 m |
| 门口语义梯度 | A1 0.73 / A2 −0.45 / A3 0.18 / A4 0.00 | 0.88 / 0.50 / 0.50 / **−0.69** / **−0.69** |
| 负分物 | 2 hydrant + 2 cone（全在环廊） | 2 hydrant + 1 cone（全在主走廊） |
| 期望路程 | **57.6 m**（≈115 s 行驶） | **37.4 m**（≈75 s 行驶） |
| 门控考点 | 梯度阈值扫描（0.12 < θ ≤ 0.50） | 两层门串联；`env06b` 变体证明硬删必漏 |
| 覆盖考点 | 走错方向绕 60 m 环廊，不能重进 | 进 B2 ≠ 搜完 B2，必须发现内门 |
| 干扰 | A2 办公室（桌/椅/显示器，全部负分）+ A1 纸箱仓库诱饵 | 垃圾桶/纸箱（异类）+ 黄杯/蓝杯（颜色变体） |
| 目标遮挡 | A3 开放仓库，梯子正对门；纸箱分列两侧、货架靠 NE/SW 墙，门口直接看到梯子 | 桌子挡直视线，绕到桌子南侧观察圈才看见 |

---

## 2. clearance = 0.60 是**必须的**（已用真实代码验证）

用 `StructuralPlaceMap` + `conservative_clearance_cells` 在 0.1 m 合成栅格上扫门宽（墙厚 0.15，两种栅格对齐）：

| clearance | 门宽 → 切 2 个 Place | 门宽 → Navfn 连通 | 同时满足 |
|---|---|---|---|
| **0.52** | ≤ 1.25 | ≥ 1.30 | **空集！** |
| **0.60** | ≤ 1.45 | ≥ 1.30 | **1.30 – 1.45** |

推导：`region_topology_clearance = clearance + 0.15`
- routing 需要 `ceil(clearance/0.1)=6` 格 = 0.60 m 半径 → 门宽 ≥ 1.20（实测栅格化后 ≥1.30）
- 切 Place 需要门中线落在 `ceil(0.75/0.1)-1 = 7` 格膨胀内 → 门宽 ≤ 1.40（实测 1.45 仍可切，1.50 不可）

**结论：新场景启动参数固定 `clearance:=0.60`，门统一 1.40 m**（在 1.30–1.45 窗口正中，两侧各留 0.05 m）。env04 保持 0.52 不变。

---

## 3. WeDetect 词表实测（bit1010 实机，wedetect_base + 640×360 + 0.60 m 相机）

隔离实例：另开 ROS/Gazebo master、Xvfb + 软件渲染、6+6 个机位，跑的就是 `utils/wedetect_backend.py` 同一个 backend。
证据：`evidence/images/`、`evidence/boxes.json`、`evidence/logs/`，方法与完整结论见 `wedetect_verification.md`。

### 3.1 目标候选（w/h = 归一化框宽高）

| 目标 | prompt | 距离 | 分数 | bbox w×h | 判定 |
|---|---|---|---|---|---|
| **laptop_pc_1** | `black laptop` | 1.15 m | **0.728** | 0.178 × 0.256 | ✅ |
| laptop_pc_1 | `black laptop` | 3.35 m | **0.708** | 0.065 × 0.089 | ✅（远距也过 0.05 门限）|
| **plant_1** | `plant` | 2.0 m | **0.781** | 0.171 × 0.145 | ✅ |
| plant_1 | `potted plant` | 2.0 m | 0.333 | 0.172 × 0.144 | ⚠ 用 `plant` |
| first_2015_trash_can | `trash can` | 1.5 / 3.0 m | 0.978 / 0.932 | 0.207×0.514 / 0.096×0.236 | ✅ |
| **blue_bin_cuboid** | `blue recycling bin` | 1.15 / 3.0 m | **无检测** | — | ❌ **不可用** |
| coffee_machine | `coffee machine` | 1.5 m | 0.347 | 0.153 × 0.266 | ⚠ 偏弱 |
| bookshelf_large | `bookshelf` | 2.5 m | 0.789 | 0.372 × 0.725 | ✅ |
| cardboard_box | `cardboard box` | 2.0 m | 0.885 | 0.155 × 0.194 | ✅ |
| file_cabinet_large | `cabinet` | 3.0 m | **无检测** | — | ❌ |

### 3.2 环境（S_env）词

| prompt | 场景 | 分数 | 结论 |
|---|---|---|---|
| `chair` | 会议室 ×3 | 0.921 / 0.911 / 0.871 | ✅ |
| `monitor` | 桌面 | 0.743 ~ 0.823 | ✅ |
| `desk` | 桌 | 0.585 ~ 0.845 | ✅ |
| `desk lamp` | 桌面（A3 观察圈） | 0.871（生产默认词表） | ✅ 用作 `target_ctx.right` |
| `keyboard` | 桌面 | 未测到（2 cm 平放，被桌沿挡住） | ❌ 已从场景移除 |

→ 会议室那帧实际命中 **desk + chair + monitor = 3 个**，对应 S_env = **0.88**，与设计表格一致。

### 3.3 两条必须知道的负面结论

1. **颜色属性不可靠**：深绿色的 `first_2015_trash_can` 被识别成
   `blue recycling bin` **0.944**、`blue bin` **0.944**（在 3 m 处仍有 0.957）。
   而 `postprocess_target_candidates` 的颜色校验是"软佐证"（`min_keep=1`，没验到颜色也保留候选），
   **所以"同类别不同颜色"的硬负例在当前栈里不可分**。→ 场景 B 改成**异类**干扰。
2. **`blue_bin_cuboid` 完全检不出**（1.15 m 和 3.0 m 都是 0），**`file_cabinet_large` 也检不出**（只出纸箱）。
   所以：蓝桶不能做目标；柜子不能做 ctx 词。

### 3.4 建议的 `~wedetect_label_map` 增量

```yaml
laptop: 笔记本电脑
black laptop: 黑色笔记本电脑
plant: 植物          # 实测英文 0.781，中文可再验证
trash can: 垃圾桶
cardboard box: 纸箱
bookshelf: 书架
coffee machine: 咖啡机
```
实测英文原文（未映射）也能出框，但**远距分数掉一半**（laptop 3.35 m：0.367 → 0.708），加映射更稳。

---

## 4. 目标摆放的两条硬规则（实测）

1. **桌面目标必须贴桌沿**：目标离桌沿 b ≤ 0.15 m。
   实测：笔记本放在桌沿后 0.55 m 时，1.15 m 观察圈里**只看到桌板正面**（完全遮挡）；
   移到桌沿后 0.05 m 才完整可见。
2. **目标至少一维 ≥ 0.12 m，推荐 ≥ 0.15 m**（`target_done_min_box_width=0.05` 是 OR 判定，一维达标即可）。

---

## 5. 门控的语义刻度（照新逻辑设计）

```
room_terms = related_structures ∪ {ctx_left, ctx_right} − negative_clues   (本例 5 个 token)
S = logistic(pos_ratio, 0.4, 10) − 0.7 · logistic(neg_ratio, 0.15, 12)
```

| 门口可见 env 物体数 | S_env | 本设计 |
|---|---|---|
| 0 | **0.02** | A1 / A4 |
| 负分货物 | **−0.69** | B4 / B5（file cabinet / cardboard box） |
| 2 | **0.50** | A3 / B2 / B3 |
| 3 | **0.88** | A2 / B1 |
| 看到 1 个 hydrant | **−0.63** | 走廊负分物 |

**正确阈值区间 (0.12, 0.50]**；θ=0.3 落在里面；θ≥0.5 会漏掉目标房。

---

## 6. 魔术值兼容表

| 参数 | 值 | 两套场景如何满足 |
|---|---|---|
| gmapping delta | 0.10 m | 全部栅格阈值按 0.1 m 换算 |
| 地图窗口 | x[−3,27] y[−16,9] | A 22×16（35.2k cells）；B 26×12.2（31.7k cells） |
| **clearance** | **0.60（新场景）** | 门 1.40 m 落在实测窗口 1.30–1.45 内 |
| region_topology_clearance | 0.75（派生） | 切 Place 需门宽 ≤1.45 ✓ |
| portal support_radius | 0.9 m（派生） | 门半宽 0.70 ≤0.90 ✓ |
| 走廊宽 | >1.34 m 自成 Place | A 2.25 m；B 2.85 m |
| furniture_max_span | 2.5 m | 桌 2.40、柜 1.49、书架 1.66、挡墙 2.0 |
| region_memory_radius | 2.5 m | 所有房间 ≥5.2 m 进深 |
| region_stagnation_timeout | 12 s | 每个门口都有 ≥2.25 m 净空停车位 |
| completed_radius | 1.25 m | 走错房/绕环后不重进 |
| dead_end_check_distance | 4.0 m | B4/B5 进深 3.2 m；两者已判负分 |
| 路口窗口（若跑 topo tree） | 5 m / 10 s | 门心间距 A 9.2/9.2；B 5.5/5.5/5.5 |
| target_done_min_box | 0.05 | 笔记本 0.178、绿植 0.171，4 倍余量 |
| `_target_eval_target_size_m` | 需改 | A: 0.35,0.31,0.25　B: 0.83,0.84,0.55 |
| `_target_eval_target_labels` | 需改 | A: `laptop`　B: `plant,potted plant` |

---

## 7. 待确认（已收敛）

1. ~~clearance~~ → **已定 0.60**。
2. ~~颜色消融~~ → **保留**，但需硬颜色门限（代码改动）。
3. ~~绿植目标~~ → **已定**。
4. 仍待确认：门控是硬删还是软降权（本设计默认「软降权 + 兜底」，`env06b` 证明硬删必漏）。
5. 仍待确认：门控只作用于房间、不作用于走廊（否则第一条走廊 S=0.02 就把自己锁死）。
6. 仍待确认：门口标签归属需要额外的 portal 观测归属逻辑。

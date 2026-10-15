# 复杂办公楼 Gazebo 测试场景建设计划

> 文档版本：2026-08-27  
> 文档类型：测试环境设计与实施计划  
> 适用系统：LSTE + 在线 SLAM + Navfn + TEB + WeDetect  
> 设计目标：用一个具有真实办公楼结构和语义的 Gazebo 场景，验证系统从任务理解、目标发现、在线建图、全局探索、路径规划到目标接近的完整闭环。

> **文档定位**：本文件是办公楼 benchmark 的唯一计划和验收依据。第 1 至 14 节定义目标与设计，第 15 至 27 节记录当前实现和事实，第 28 节以后给出可直接执行的验收计划。静态文件“已存在”不等于端到端任务“已通过”。

> **当前配置快照（2026-09-06）**：仓库当前的四个 level 都是静态场景；
> `scripts/tests/office_building/scenarios.yaml` 和
> `worlds/benchmark/office_building_v1_manifest.yaml` 是 level/实体的运行时权威来源。
> 本文中关于 `dynamic_obstacle.py`、动态服务和旧 hash 的段落属于历史运行记录，
> 不代表当前 Level 4 配置；新的实验必须以当前 manifest、现场 SHA-256 和日志为准。

> **当前验收口径（2026-09-08）**：本文第 25～27 节保留 2026-08-27 的历史 benchmark
> 运行记录，不单独证明当前代码基线已经稳定完成 Level 4。完成生命周期与仿真时钟改动后，
> Level 4 需要重新执行；当前目标、重复运行门槛、失败证据要求和停止修改条件以本轮对话
> 中确认的最新目标为准。

## 0. 一页式总览

这份文档同时承担三项职责：

1. 定义要建设的办公楼场景，以及每个房间、门洞、家具和障碍为什么存在；
2. 定义如何把场景接入现有 LSTE 启动流程，并验证 SLAM、frontier、Navfn、TEB 和 WeDetect 的完整数据流；
3. 定义如何进行可复现的分级实验、记录日志、归类失败，并依据 Definition of Done 判断是否完成。

执行时严格按下面的顺序推进，不跳级：

```text
冻结场景规格
  -> 生成 world 和 manifest
  -> 对四个 level 做静态校验
  -> 停止旧进程并建立独立 timestamp 日志目录
  -> 启动 Level 1，验证 60 秒基础闭环
  -> 完成 Level 1 三次重复任务
  -> 完成 Level 2 语义家具和干扰物测试
  -> 完成 Level 3 死路、墙后目标和路线恢复测试
  -> 完成 Level 4 动态障碍测试
  -> 汇总指标、截图、轨迹和失败案例
  -> 审计 Definition of Done
  -> 仅提交源码、world、配置和文档
```

每个阶段的输入、输出和停止条件如下：

| 阶段 | 主要输入 | 必须产生的输出 | 未满足时的处理 |
| --- | --- | --- | --- |
| 规格冻结 | 房间布局、机器人 footprint、传感器范围、任务路线 | manifest、坐标表、level 开关 | 不启动 Gazebo，先修正规格 |
| 场景生成 | `generate_world.py`、模型目录、场景 level | 对应 level 的 world 文件 | 生成失败或 hash 不一致则停止 |
| 静态校验 | world、manifest、必需模型和坐标 | validator 通过、world hash | 不进入运行测试 |
| 启动闭环 | ROS、Gazebo、LSTE 配置 | timestamp 日志、节点/topic/TF 就绪 | 归类为 `infrastructure_failure` |
| Level 1 | 建筑骨架、少量家具、可达目标 | 3 次成功运行 | 先修复基础导航或目标接近 |
| Level 2 | 完整语义家具、相似目标 | 至少 2/3 成功 | 分离误检、抢占和局部控制问题 |
| Level 3 | 死路、遮挡、窄门、替代环路 | 至少 1 次成功恢复 | 检查 frontier、Navfn 和 goal ownership |
| Level 4 | 固定种子动态障碍 | 至少 1 次可复现演示 | 先验证动态服务，再分析 TEB |
| 归档验收 | 所有 timestamp 日志和证据 | metrics JSON、截图/录像、失败分析 | 不宣称 benchmark 完成 |

最小可执行入口为：

```bash
cd .
scripts/tests/office_building/run_office_building.sh generate
scripts/tests/office_building/run_office_building.sh validate level_1
scripts/tests/office_building/run_office_building.sh start level_1
scripts/tests/office_building/run_office_building.sh status
scripts/tests/office_building/run_office_building.sh summary
scripts/tests/office_building/run_office_building.sh stop
```

`start` 必须后台运行且不进入 tmux；每次运行都必须创建新的 `YYYYMMDD_HHMMSS` 日志目录。正式运行中不得临时修改生产 YAML、手动移动目标或删除障碍；需要定位单个问题时，必须标记为 `diagnostic_only`，不纳入通过次数。

## 1. 为什么需要一个复杂场景

当前 `env04_no_pro3.world` 可以验证办公环境中的基本导航，但它更接近一个固定实验地图，而不是完整的办公楼。它不能充分覆盖以下问题：

- 机器人是否能从大厅进入多个语义不同的房间；
- 在线 SLAM 是否能在未知区域逐步扩展地图；
- frontier 是否能选择真正有价值的未探索区域；
- Navfn 是否能绕过墙、桌子、死路和错误分支；
- TEB 是否能在走廊、门洞、转角和狭窄空间内连续行驶；
- 视觉目标被遮挡或暂时丢失后，系统是否能保持合理的全局路线；
- 目标在某个房间内时，系统能否先找到房间入口，再进行目标接近；
- 目标锁定后，路径切换是否会产生不必要的原地旋转、急停和反复改向。

因此，新场景不应是若干个孤立的直角走廊，而应当是一个完整的、具有多种空间语义的办公楼。所有测试因素都在同一个建筑中出现，但通过任务配置、起点和目标位置控制每次实验关注的变量。

## 2. 总体设计原则

### 2.1 一个建筑，多个语义空间

建立一套统一坐标体系的办公楼 world，四个 level 同时保留为独立文件：

```text
worlds/benchmark/office_building_v1_level_1_no_pro3.world
worlds/benchmark/office_building_v1_level_2_no_pro3.world
worlds/benchmark/office_building_v1_level_3_no_pro3.world
worlds/benchmark/office_building_v1_level_4_no_pro3.world
```

这个 world 包含大厅、主走廊、开放办公区、会议室、茶水间、打印区、储物间、经理办公室和一个带遮挡的目标房间。机器人每次从同一个或少数几个标准起点出发，目标则根据测试任务放在不同房间。

### 2.2 几何、语义和感知必须同时成立

房间不能只有名字。每个语义空间都要有与现实对应的布局和物体：

| 空间 | 应包含的真实语义 | 导航意义 | 感知意义 |
| --- | --- | --- | --- |
| 入口大厅 | 前台、沙发、等候区、公告牌 | 大空间起步、多个出口 | 视野开阔、目标容易被误检 |
| 主走廊 | 长走廊、墙面、多个门洞 | 长距离跟踪、方向连续性 | 目标通常不在走廊中央 |
| 开放办公区 | 成排办公桌、显示器、办公椅 | 障碍密集、通道不规则 | 适合寻找杯子、文件等小目标 |
| 会议室 | 大会议桌、多个座椅、白板 | 房间入口和绕桌路径 | 目标可能位于桌面边缘 |
| 茶水间 | 操作台、柜子、饮水机、杯子 | 狭窄区域和遮挡 | 多个相似杯子，考验开放词汇检测 |
| 打印区 | 打印机、纸箱、文件架 | 局部拥挤、短转弯 | 文件夹、纸盒等相似上下文物体 |
| 储物间 | 货架、箱子、窄入口 | 死路、低宽度、回退 | 目标可能完全不可见 |
| 经理办公室 | 独立办公桌、书柜、显示器 | 单入口房间、目标接近 | 目标和上下文物体组合出现 |
| 目标房间 | 目标物、遮挡物、侧向观察位置 | 先绕障碍再接近 | 验证目标丢失、重捕获和最终确认 |

### 2.3 不随机堆障碍物

障碍物必须有建筑逻辑：桌子靠墙、椅子围绕桌子、打印机靠近电源墙、会议桌位于会议室中心、货架沿储物间墙面排列。每个障碍都应能解释为什么机器人需要改变路线，而不是为了制造失败而随意放置。

### 2.4 保持实验可复现

静态场景不使用无控制的随机摆放。动态物体使用固定随机种子和预设轨迹。每次实验记录：

- world 文件及其 SHA-256；
- 场景版本和布局版本；
- 机器人初始位姿；
- 目标模型、任务 JSON 和目标坐标；
- 控制器、速度和关键参数；
- Git revision；
- 动态障碍物的种子和轨迹版本。

## 3. 办公楼的空间布局

### 3.1 建筑总体结构

建议使用一个近似矩形的单层办公楼，面积约为 `25 m x 22 m`。尺寸最终应根据 Pro3 的 footprint、激光雷达有效范围和 Gazebo 运行性能校准。建筑结构如下：

```text
┌────────────────────────────────────────────────────────────┐
│  会议室              主走廊                         茶水间 │
│  ┌───────────┐  ┌──────────────────────────────┐  ┌──────┐ │
│  │会议桌/椅  │  │                              │  │柜台  │ │
│  │白板       ├──┤        多个房间入口          ├──┤杯子  │ │
│  └───────────┘  │                              │  └──────┘ │
│                 │                              │           │
│  开放办公区     │                              │ 打印区    │
│  ┌───────────┐  │                              │ ┌───────┐ │
│  │桌列/显示器│  │                              │ │打印机 │ │
│  │椅子/文件  │  └──────────────┬───────────────┘ │纸箱   │ │
│  └───────────┘                 │                 └───────┘ │
│                                │                           │
│  经理办公室          大厅/前台  │       储物间/目标房间      │
│  ┌───────────┐       ┌─────────┴──────┐   ┌─────────────┐  │
│  │办公桌/书柜│───────│前台/沙发/公告牌│───│货架/遮挡目标│  │
│  └───────────┘       └────────────────┘   └─────────────┘  │
└────────────────────────────────────────────────────────────┘
```

上图是逻辑分区，不是最终精确尺寸。最终 world 应使用墙体、门洞和模型坐标明确表达同样的关系。

### 3.2 房间连接关系

建筑采用“主走廊 + 独立房间”的办公楼拓扑：

1. 大厅、开放办公区、打印区、储物间、休息区、会议室、经理办公室、茶水间和目标房间都各自拥有通向主走廊的命名门洞；
2. 墙体在门洞以外必须连续，房间不能因为两段墙之间的空隙而直接相通；
3. 只有开放办公区与打印区保留一扇标准宽度的员工内门，它是明确建模的服务连接，不是大面积开口；
4. 储物间和目标房间位于较深分支，需要经过自己的门洞或绕过遮挡才能进入。

应避免所有房间都只有一条直线连接。至少保留：

- 一个 T 字分叉；
- 一个 L 型转弯；
- 一个由走廊和“开放办公区 - 打印区”员工内门形成的小型可绕行环路；
- 一个看起来可走但实际是死路的分支；
- 一个必须从门洞进入的房间；
- 一个目标在墙后、但地图上存在绕行路线的区域。

这样才能观察系统是在“沿着目标方向盲走”，还是在利用地图找到可行路线。

## 4. 关键场景元素

### 4.1 主走廊

- 宽度应足以让 Pro3 正常通过，但不能宽到完全没有墙距约束；
- 走廊中段设置一个轻微转折，而不是无限长直线；
- 两侧有不同房间入口，入口宽度和深度略有差异；
- 其中一个入口通往死路或储物间，避免 frontier 只根据最近距离选择错误分支；
- 开放办公区与打印区的员工内门和两侧走廊门形成可验证的小型服务环路。

测试重点：地图扩展、路线连续性、走廊中心行驶、目标锁定后的路径接管。

### 4.2 门洞

至少设置三种门洞：

- 宽门洞：明显可通行，作为基础检查；
- 标准办公室门洞：需要 TEB 调整姿态后通过；
- 被家具部分遮挡的门洞：需要先接近并观察，不能从远处直接把目标点当作可达点。

门洞不能用不可见碰撞体制造。视觉模型和激光雷达看到的几何必须一致，否则无法判断失败来自算法还是仿真模型。
除 manifest 中明确列出的 `office_printer_staff_door` 外，房间之间没有直接通路；每个门洞均由连续墙体两侧的门框和上方 lintel 表达。

南侧 `south_entrance` 是一个例外且已明确记录：它保留门框和入口语义，但在 benchmark
中使用可见的关闭门。机器人标准起点位于建筑内部，Gazebo 建筑外是无限地面；如果入口
保持敞开，frontier 会把楼外当成无界的未探索区域，实验将失去“办公楼内部探索”的
边界。该关闭门不影响大厅到主走廊及各语义房间的内部路线，也不修改生产 world。

### 4.3 开放办公区

开放办公区是本场景最重要的复杂区域之一：

- 2 至 3 排办公桌；
- 桌面放置显示器、键盘、文件夹和杯子；
- 桌间保留一条主通道和若干较窄的侧通道；
- 椅子不要全部收齐，应有少量椅子伸入通道；
- 至少一个桌组靠近墙，形成目标被墙或桌面遮挡的视角；
- 真实目标旁边放置上下文物体，例如两个显示器或键盘。

测试重点：小物体检测、目标上下文判断、绕桌路径、局部障碍安全距离。

### 4.4 会议室

会议室中放置一张较大的会议桌、多个椅子、白板和墙面显示屏。目标可以有两种配置：

- 放在会议桌远端，需要绕桌接近；
- 放在椅子后方，只能从侧面看到。

测试重点：大障碍物绕行、房间入口选择和目标最终接近。

### 4.5 茶水间

茶水间应有柜台、吊柜、饮水机、垃圾桶和多个杯子。其中只有一个杯子是任务目标，其余杯子颜色或位置不同。

推荐目标布置：

- 目标黄色杯放在柜台边缘；
- 蓝色杯放在相邻位置，作为相似干扰物；
- 目标初始不在机器人视野内；
- 机器人需要从门洞进入后转向柜台。

测试重点：WeDetect 的开放词汇识别、相似目标区分、短暂遮挡和近距离停止。

### 4.6 打印区

打印区放置打印机、纸箱、文件架和回收箱，形成不规则但合理的局部障碍。纸箱可以占据一小段通道，使局部路线需要绕行。

测试重点：局部代价地图更新、TEB 连续绕行、障碍物安全距离和临时路线变化。

### 4.7 储物间和死路

储物间使用货架形成一个短死路，入口看起来足够吸引 frontier，但深入后无法继续。储物间内部不放任务目标。

测试重点：

- Navfn 是否能识别没有出口的区域；
- frontier 是否能在失败后转向其他分支；
- 机器人是否会卡在墙角；
- 是否出现反复进入和退出同一死路的行为。

### 4.8 目标房间

目标房间应位于地图较深处，目标工位位于入口后的第一观察区，而不是远端的单一幸运
视角。门框和侧向家具仍会造成遮挡，机器人需要先进入房间、调整观察位姿，再完成目标
接近；这样目标发现必须经历：

```text
探索未知区域
    -> 发现房间入口
    -> 通过 Navfn/TEB 进入房间
    -> 目标进入相机视野
    -> 多帧确认目标
    -> 选择可达观察点
    -> 接近并完成任务
```

## 5. 机器人起点、目标与任务设计

### 5.1 标准起点

首版固定一个主起点，建议位于大厅入口附近，朝向主走廊。之后可以增加两个备用起点：

- 大厅起点：测试全局探索；
- 开放办公区起点：测试局部导航和目标接近；
- 目标房间外起点：测试感知和最终路线，不用于评估 frontier 探索。

同一项对比实验必须固定起点和朝向。不能因为某个控制器失败就手动把机器人转到更有利的方向。

### 5.2 目标任务

首版只保留一个主任务，减少变量：

```yaml
task_id: yellow_cup
target_label: yellow cup
target_model: cup_yellow
```

黄色杯放在茶水间或目标房间中。蓝色杯、文件夹、显示器和键盘作为干扰和上下文物体。后续可以新增 `blue_mug`、`red_cup` 等任务，但必须使用独立任务 JSON，不能仅靠修改 world 中的模型名字伪造结果。

### 5.3 多种任务路线

同一个建筑至少定义三条逻辑任务路线：

1. 大厅 -> 主走廊 -> 茶水间 -> 黄色杯；
2. 大厅 -> 开放办公区 -> 目标房间 -> 黄色杯；
3. 大厅 -> 错误分支/储物间 -> 回退 -> 主走廊 -> 目标房间 -> 黄色杯。

这三条路线覆盖直接路线、障碍密集路线和需要恢复的路线；在同一个 level 的一批对比实验中，它们使用同一个已生成的 world，避免几何变量混入结果。

## 6. 难度分层

复杂场景不意味着每次都启用全部障碍。应通过场景配置控制启用的房间和障碍密度：

### Level 1：建筑骨架

- 墙、房间、主走廊、门洞；
- 不放动态障碍；
- 每个房间只放少量家具；
- 目标位置明显可达。

目的：验证 world、传感器、SLAM、Navfn、TEB 和目标球可正常工作。

### Level 2：语义办公环境

- 启用完整房间家具；
- 开放办公区增加桌列和显示器；
- 茶水间增加多个杯子；
- 目标从远处不可见，需要进入正确房间。

目的：验证感知、frontier 探索和多房间导航。

### Level 3：结构性挑战

- 启用死路、部分遮挡门洞和不规则障碍；
- 目标位于墙后或家具后；
- 至少有一个相似但错误的上下文区域。

目的：验证绕墙、回退、目标路线连续性和丢帧恢复。

### Level 4：演示级综合场景

- 启用全部房间和家具；
- 加入一到两个低速动态障碍；
- 使用完整自然语言任务；
- 进行一次从大厅到目标房间的端到端运行。

目的：生成汇报视频和检验系统整体表现。Level 4 的失败必须能回溯到 Level 1 至 Level 3 的单项实验。

## 7. 需要重点观测的系统行为

### 7.1 在线建图和 frontier

观察地图是否随机器人移动扩展，frontier 是否集中在未探索但可到达的房间入口，而不是反复选择已经访问的区域。

关键问题：

- 地图是否出现明显错位或重复墙体；
- frontier 是否被死路吸引；
- 房间入口被发现后是否能继续推进；
- 目标检测出现时，是否会不合理地抢占正在执行的前沿路线。

### 7.2 Navfn

Navfn 负责在当前地图中判断全局连通性。应记录：

- 规划是否有有效 path；
- 路径是否穿过未知或占用区域；
- 目标在墙后时是否被正确拒绝；
- 绕墙路线是否从正确的门洞进入；
- 死路失败后是否重新选择其他分支。

### 7.3 TEB

TEB 负责局部时空轨迹，不负责猜测未知房间。应关注：

- 走廊中是否保持连续前进；
- 转角处是否出现不必要的急停；
- 门洞前是否频繁左右摆动；
- 障碍物靠近时是否以安全方式减速；
- 目标锁定后是否突然替换为方向完全不同的短目标；
- 必须转向时是否能转向，非必要时是否避免原地旋转。

### 7.4 目标管理

Goal Manager 的测试重点是所有权和事务边界：

- frontier 路线执行期间，弱目标检测不能随意抢占；
- 确认目标后，目标路线必须通过 Navfn 可达性验证；
- 检测短暂丢失时不能立即启动破坏路线连续性的重捕获扫动；
- 目标路线被墙阻挡时，应释放到地图前沿，而不是反复重试同一点；
- 目标接近完成后，要等待多帧确认并停止。

## 8. 评测指标和成功标准

每次运行使用现有 navigation metrics 和目标评测日志。至少统计：

| 指标 | 说明 |
| --- | --- |
| task_success | 是否完成目标任务 |
| elapsed_time | 从启动到完成的时间 |
| path_length | 机器人实际行驶距离 |
| map_coverage | 任务结束时已探索地图比例 |
| navfn_success_rate | Navfn 规划成功次数和失败次数 |
| move_base_aborts | move_base abort 次数，应为 0 |
| move_base_preemptions | 非预期抢占次数，应尽量为 0 |
| turn_only_duration | 仅旋转、不前进的累计时间 |
| turn_only_events | 仅旋转事件次数 |
| stop_events | 急停和控制周期断档次数 |
| min_clearance | 最小障碍距离 |
| goal_changes | global goal 改变次数 |
| detector_lock_time | 首次检测到确认锁定的时间 |
| target_route_rejections | 目标路线被 Navfn 拒绝的次数 |
| target_task_completion | 目标近距离多帧确认是否成功 |

第一版场景的最低验收标准：

- Level 1 在 3 次重复运行中全部完成；
- Level 2 至少 3 次中完成 2 次；
- Level 3 能绕过死路或墙后目标，不出现无限重试；
- Level 4 能完成一次完整演示，失败时日志能指出具体阶段；
- 所有成功运行没有未解释的 `move_base_abort`；
- 目标锁定后的路线切换不会产生持续数秒的非必要原地旋转；
- 机器人不应长期贴墙或停在门洞前反复摆动。

## 9. 工程文件组织

建议新增以下结构，保留现有生产 world 不变：

```text
worlds/benchmark/
  office_building_v1_level_1_no_pro3.world
  office_building_v1_level_2_no_pro3.world
  office_building_v1_level_3_no_pro3.world
  office_building_v1_level_4_no_pro3.world
  office_building_v1_manifest.yaml
  models/
    office_desk/
    office_chair/
    meeting_table/
    printer_station/
    storage_shelf/
    cup_yellow/
    cup_blue/
    moving_obstacle/

scripts/tests/office_building/
  scenarios.yaml
  run_office_building.sh
  validate_office_building.py
  summarize_run.py

docs/testing/
  20260827_office_building_benchmark_plan.md
```

四份 `office_building_v1_level_*_no_pro3.world` 只保存建筑和环境模型。Pro3 由现有启动流程根据配置生成，避免 world 内置机器人和脚本生成机器人发生重复。

## 10. 启动和实验方式

不改变默认生产配置。测试时通过独立测试配置选择场景，例如：

```yaml
SCENARIO_ID: office_building_v1
WORLD: worlds/benchmark/office_building_v1_level_2_no_pro3.world
TASK_JSON: model/Data_exchange/vlm_prompt/lab/yellow_cup.json
TASK_ID: yellow_cup
PRO3_SPAWN_X: 2.0
PRO3_SPAWN_Y: 0.0
PRO3_SPAWN_YAW: 0.0
LSTE_CONTROLLER: teb
```

启动流程仍使用现有 `runall` / `stopall` 体系。新增脚本只负责：

- 选择 benchmark world；
- 检查模型、任务 JSON 和坐标；
- 为本次运行建立日志目录；
- 启动 Gazebo 和 LSTE 节点；
- 结束后汇总结果。

不把运行时日志、地图缓存、视频和临时生成文件提交到 Git。

## 11. 实施阶段

### 阶段 A：场景规格冻结

1. 确认建筑边界、房间尺寸和房间连接关系；
2. 测量 Pro3 footprint、激光雷达视场和最小安全通道宽度；
3. 确认目标房间和三条标准任务路线；
4. 写出 manifest 和坐标表；
5. 保留当前 `env04_no_pro3.world` 作为回归基线。

### 阶段 B：搭建建筑骨架

1. 创建墙体、地面、房间和门洞；
2. 确认 Gazebo 碰撞模型与视觉模型一致；
3. 确认 Pro3 能在大厅和主走廊正常生成；
4. 确认 `/pro3/wheel_odom`、`/pro3/rlscan`、相机和 `/map` 正常；
5. 确认 Navfn 能在 Level 1 建筑骨架中规划到各房间入口。

### 阶段 C：加入语义家具

1. 加入大厅、办公区、会议室、茶水间、打印区和储物间模型；
2. 加入显示器、杯子、文件夹等视觉物体；
3. 验证模型名称、材质和碰撞边界；
4. 验证 WeDetect 能发布目标检测；
5. 验证目标球或目标可视化不会影响实际导航。

### 阶段 D：加入结构性挑战

1. 启用死路和错误分支；
2. 启用部分遮挡门洞；
3. 设置墙后目标和侧向目标；
4. 重复执行三条标准任务路线；
5. 分析目标锁定、路线接管、丢帧和恢复日志。

### 阶段 E：动态障碍和端到端演示

1. 只加入一个低速动态障碍；
2. 使用固定轨迹和固定种子；
3. 验证 TEB 与安全距离策略；
4. 录制完整从大厅到目标房间的视频；
5. 生成场景结构图、轨迹图、指标表和失败案例说明。

## 12. 不纳入首版的内容

为了保证问题可定位，首版不加入：

- 多层楼梯和电梯；
- 复杂玻璃反射；
- 大量随机移动行人；
- 会改变地图拓扑的动态墙体；
- 过多装饰性模型；
- 同时存在多个需要完成的目标任务；
- 未经测量的极窄通道。

这些内容会显著增加仿真、感知和定位变量，应该在单层办公楼 benchmark 稳定后单独加入。

## 13. 最终交付物

完成后应得到：

1. 一个可由现有启动流程直接加载的办公楼 world；
2. 一份场景 manifest，包含房间语义、模型、坐标和任务路线；
3. 一份测试配置，可选择 Level 1 至 Level 4；
4. 一个场景校验脚本，启动前检查模型、目标和坐标；
5. 一个结果汇总脚本，读取 navigation metrics 并生成表格；
6. 至少一次完整端到端成功录像；
7. 至少一个失败案例及其日志分析；
8. 在 `docs/introduction.md` 中增加本文件索引。

## 14. 设计结论

我们要建设的不是“更多障碍物”，而是一个可解释的办公楼任务空间。它让机器人必须经历真实自主导航中的关键阶段：在大厅开始探索，沿走廊发现房间，在不同语义空间中识别目标上下文，绕过桌椅和墙体，通过门洞进入未知区域，在目标暂时不可见时保持全局路线，最后从可达观察点稳定接近目标。

这个场景足够复杂，可以暴露系统的真实问题；同时每个房间、分支和障碍都有明确作用，因此失败可以被定位、复现和修复，而不是只能归因于“环境太复杂”。

## 15. 当前实现清单（2026-08-27）

本节把计划与当前工作树逐项对齐。状态同时区分“文件已落地”和“真实 Gazebo 已验证”；
端到端结论只引用第 27 节中的 timestamp 日志和证据，不根据静态文件存在作推断。

| 组件 | 文件 | 当前状态 | 责任边界 |
| --- | --- | --- | --- |
| 场景生成器 | `scripts/tests/office_building/generate_world.py` | 已实现 | 按固定几何和模型坐标生成 world；不生成 Pro3 |
| Gazebo world | `worlds/benchmark/office_building_v1_level_1..4_no_pro3.world` | 已生成 | 四个独立、同时保留的 level 文件；建筑、碰撞、灯光、家具、目标和可选动态障碍 |
| 场景契约 | `worlds/benchmark/office_building_v1_manifest.yaml` | 已实现 | 房间、门、目标、起点、路线、必需模型、验收约束 |
| benchmark 配置 | `scripts/tests/office_building/office_building_config.yaml` | 已实现 | 通过 `PIPELINE_CONFIG` 覆盖测试专用 world、起点、TEB 和目标评估参数 |
| 难度选择 | `scripts/tests/office_building/scenarios.yaml` | 已实现 | `level_1` 至 `level_4` 的功能开关和动态障碍种子 |
| 启动入口 | `scripts/tests/office_building/run_office_building.sh` | 已实现，Level 1 启动闭环已 smoke test | `generate/validate/start/stop/status/summary`，后台启动，不 attach tmux |
| 动态障碍 | `scripts/tests/office_building/dynamic_obstacle.py` | 已实现，已验证（`20260827_231117`） | Level 4 中按固定三角波调用 `/gazebo/set_model_state`，记录每次调用和返回状态 |
| 指标汇总 | `scripts/tests/office_building/summarize_run.py` | 已实现，已验证 | 从 navigation metrics 日志提取任务、路径、停止、抢占和目标事件；完成后使用首次 `task_done=true` 快照 |
| 轨迹证据 | `scripts/tests/office_building/plot_benchmark_evidence.py` | 已实现，已生成 | 读取 metrics sample，叠加房间边界、起点、目标和多次运行轨迹 |
| 日志前缀器 | `scripts/tests/office_building/prefix_log.py` | 已实现 | 将 tmux pane 输出写成带时间戳、进程名和事件字段的独立 `.log` |
| 文档索引 | `docs/introduction.md` | 已更新 | 提供本计划入口 |

以下内容是已完成或仍需收尾的验证，不能仅凭静态文件存在作结论：

1. 已完成当前 ROS/Gazebo 环境的 world 加载、单次 Pro3 spawn、关键节点上线和四级静态校验；
2. 已完成 Level 1 至 Level 4 的代表性端到端运行，结果和失败阶段见第 27 节；
3. 已完成 Level 4 动态模型连续服务调用验证，失败次数为 0；
4. 已生成成功运行、可解释失败/恢复运行、Gazebo/RViz 截图和轨迹图；
5. 已在最终提交前执行 `git diff --check`，确认只提交源码、world、配置和文档。

### 15.1 已完成的启动 smoke test

2026-08-27 最新一次干净启动验证为 Level 1：

```bash
scripts/tests/office_building/run_office_building.sh start level_1
```

实际结果：

- world 重新生成并通过 Level 1 manifest 校验；
- Gazebo 与 `lste-env` 正常启动；
- Pro3 正常 spawn，Gazebo `/gazebo/model_states` 中只有一个 `pro3`；
- `lste` 节点窗口、在线 SLAM/frontier、Navfn/TEB、goal manager、metrics 均创建；
- `/pro3/wheel_odom`、`/pro3/rlscan`、`/map`、`/lste/final_goal`、`/move_base/status` 和 `/gazebo/model_states` 均已发现；
- TEB 已产生有效 Navfn/TEB 路径并发布速度指令；
- 本次 smoke test 本身不计入任务成功次数；目标杯端到端和动态障碍验证已在后续正式运行中完成，
  详见第 27.8 至 27.11 节。

本次 smoke test 只证明“启动闭环和基础 topic 存在”，不计入 Level 1 至 Level 4 的任务成功次数。正式验收仍以第 25 节和第 28 节为准。

## 16. 文件与数据流

### 16.1 启动前的数据流

```text
scenarios.yaml
       |
       v
run_office_building.sh start LEVEL
       |
       +--> generate_world.py --> office_building_v1_level_<n>_no_pro3.world
       |
       +--> validate_office_building.py --> world/manifest/model contract
       |
       +--> manifest.robot_profiles[profile] --> PRO3_SPAWN_X/Y/YAW
       |
       +--> office_building_config.yaml via PIPELINE_CONFIG
       |
       +--> existing lifecycle/run_all_tmux.sh
```

world 文件只描述仿真环境。机器人、传感器、ROS 节点和控制器仍由现有 LSTE 启动流程负责。这样做的原因是避免 world 中已有机器人与生命周期脚本再次 spawn，造成重名、重复传感器话题和 TF 冲突。

### 16.2 运行时的数据流

```text
Gazebo world
  |  /pro3/rlscan, /pro3/wheel_odom, camera topics
  v
online SLAM --> /map --> frontier manager --> exploration subgoal
                                      |
target detector --> goal manager ------+--> /lste/final_goal
                                                   |
                                                   v
                         Navfn global path --> move_base --> TEB --> /cmd_vel
                                                   |
                                                   v
                                     navigation metrics + lifecycle logs
```

数据流中的所有节点仍使用生产系统已有实现。benchmark 入口只注入场景配置和实验边界，不复制一份导航栈。

### 16.3 静态输入、运行时输出和禁止提交内容

| 类型 | 例子 | Git 策略 |
| --- | --- | --- |
| 静态输入 | world、manifest、YAML、Python、Shell、文档 | 应提交 |
| 运行时输出 | `runtime/**/logs/`、地图缓存、截图、录屏、临时 PID | 不提交 |
| 外部依赖 | `~/.gazebo/models`、ROS package、CUDA/TensorRT | 不复制进仓库；在 manifest 中校验 |
| 生成文件 | 由 `generate_world.py` 生成的确定性 world | 若作为 benchmark 固定版本使用，可提交；修改生成器后必须重新生成并校验 |

## 17. 精确执行顺序

每一个级别都遵循同一顺序，禁止跳过前置校验后直接把失败归因给导航算法。

### Step 0：准备环境

```bash
cd .
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果工作区使用 `.envrc`，先执行 `direnv allow`，但不得在 benchmark 脚本中静默改写生产 YAML。运行前记录：

```bash
git rev-parse HEAD
git status --short
echo "$GAZEBO_MODEL_PATH"
```

### Step 1：重新生成并校验场景

```bash
scripts/tests/office_building/run_office_building.sh generate
scripts/tests/office_building/run_office_building.sh validate level_1
scripts/tests/office_building/run_office_building.sh validate level_2
scripts/tests/office_building/run_office_building.sh validate level_3
scripts/tests/office_building/run_office_building.sh validate level_4
```

校验器必须报告 `PASS`。若失败，先修复 world、manifest 或本机模型安装，不启动 Gazebo。

### Step 2：启动 Level 1 骨架

```bash
scripts/tests/office_building/run_office_building.sh start level_1
```

脚本会停止旧 LSTE 运行，重新生成并校验 world，导出 `PIPELINE_CONFIG`、world、目标和起点环境变量，然后调用现有生命周期入口。它不会自动进入任何 tmux 会话。
显式导出的 `WORLD` 优先于 benchmark YAML 的 Level 2 默认值，因此 `start level_3` 会加载
`office_building_v1_level_3_no_pro3.world`，不会静默回退到其他 level。

### Step 3：检查启动闭环

```bash
tmux list-sessions
rosnode list
rostopic list | rg '/(pro3/wheel_odom|pro3/rlscan|map|lste/final_goal|move_base/status)$'
```

必须确认：

- Gazebo 只有一个 `pro3`；
- `/pro3/wheel_odom` 与 `/pro3/rlscan` 持续发布；
- `/map` 在机器人移动后增长；
- `/move_base/status` 存在；
- 目标管理或 frontier 节点没有重复启动；
- `lste`、`lste-env`、`lste-teleop` 的 tmux 生命周期符合现有启动约定。

### Step 4：执行单项任务路线

按 manifest 中的 `task_routes` 逐条测试。每条路线至少重复 3 次，固定 level、profile、任务 JSON 和 Git revision。不要在同一批次中同时修改 TEB 参数、检测器和 world，否则无法判断改动来源。

### Step 5：升级难度

只有 Level 1 达到验收标准，才进入 Level 2；只有 Level 2 的语义和目标流程稳定，才进入 Level 3；Level 4 只能在前三个级别的失败均可解释后执行。

### Step 6：停止并汇总

```bash
scripts/tests/office_building/run_office_building.sh summary
scripts/tests/office_building/run_office_building.sh stop
```

停止后不得删除本次 15 天保留期内的日志。汇总结果、Git revision、world hash 和运行目录写入实验记录或汇报材料，而不是写回生产配置。

## 18. 四个难度级别的具体开关

Level 的含义是“场景复杂度”。启动器每次选择 level 后都会调用确定性的
`generate_world.py --level LEVEL`，只重新生成该 level 对应的 world 文件；因此四个
level 共享同一个场景契约和模型坐标体系，但生成结果的家具、死路和动态障碍内容不同。
不能拿一个 level 的 world 冒充另一个 level 的实验输入。

| 级别 | 启用内容 | 主要问题 | 必须观察 |
| --- | --- | --- | --- |
| 1 | 建筑外壳、主走廊、目标房间入口、少量目标家具 | 基础连通性和传感器 | spawn、TF、SLAM、Navfn 到房间入口 |
| 2 | 全语义家具、办公桌列、会议室、茶水间、多杯子干扰 | 语义感知与家具绕行 | 目标锁定、路径接管、走廊中心、近距离停止 |
| 3 | 储物间死路、替代环路、墙后目标、窄门接近 | 回退与路线重选 | 不无限重试死路、不贴墙、不重复抢占 |
| 4 | Level 3 全部内容 + 一个固定动态障碍 | 综合演示与动态避障 | TEB 减速、最小间距、无异常急停、可复现轨迹 |

仓库同时保留四个 level 的独立 world 文件。它们分别使用第 27.1 节中的四个固定 hash；
`start LEVEL` 只会重新生成并加载对应的 `office_building_v1_level_<n>_no_pro3.world`，
不会覆盖其他 level 的文件。

## 19. 验证矩阵与证据要求

每次实验记录以下最小元数据：

```text
run_timestamp: YYYYMMDD_HHMMSS
scenario_id: office_building_v1
level: level_1|level_2|level_3|level_4
route_id: direct_corridor|office_context|dead_end_recovery
robot_profile: primary|office_entry|target_entry
task_id: yellow_cup
controller: teb
detector: wedetect-large
world_sha256: <actual hash>
git_revision: <commit>
dynamic_obstacle_seed: 20260827 or null
```

| 验证层 | 命令/观测 | 通过条件 | 证据 |
| --- | --- | --- | --- |
| 静态 XML | `validate ... --level LEVEL` | `PASS`，无缺模型、重复名、机器人重复定义 | 命令输出 + world hash |
| Gazebo | GUI/`/gazebo/model_states` | world 加载、光照正常、Pro3 只出现一次 | 截图或录屏 |
| 传感器 | `rostopic hz` | odom、scan、camera 持续发布，无明显断流 | 终端日志 |
| SLAM | `/map`、TF、RViz | 地图随运动增长，墙体不重复漂移 | RViz 截图/录屏 |
| Frontier | frontier/goal 日志 | 能发现未探索入口，死路后会换分支 | frontier log |
| Navfn | move_base status/path | 房间入口路径成功，墙后不可达时有记录 | navigation log |
| TEB | `/cmd_vel`、metrics | 连续速度、无长期摆动和非必要原地旋转 | metrics + trajectory |
| 视觉目标 | detections/final goal | 多帧确认后锁定，干扰杯不抢占 | detector/goal log |
| 方法契约 | `global_frontier_event` + `summary` | 至少观测到一个且只能观测到请求的方法；缺失或不一致的 trial 不进入方法均值 | summary + aggregation record |
| Level 4 静态扩展 | global frontier log + model states | 手推车、绿植、柜子和相似杯子实体存在且有碰撞模型 | frontier/world log |
| 任务结果 | `summary` | 指标完整，成功/失败原因可解释 | JSON 汇总 |

推荐保存的证据文件名：

```text
<run_timestamp>_gazebo.png
<run_timestamp>_rviz.png
<run_timestamp>_trajectory.csv
<run_timestamp>_metrics.json
<run_timestamp>_failure_notes.md
```

证据可以放在外部实验归档目录；若复制到仓库，必须确认体积和 `.gitignore` 规则，不得把 runtime 日志混入 Git。

## 20. 日志约定与生命周期

benchmark 复用工作区的日志规则：每次启动建立唯一的 `YYYYMMDD_HHMMSS` 父目录，每个进程只写自己的 `.log` 文件，文件名重复同一时间戳。至少应存在：

```text
runtime/.../logs/YYYYMMDD_HHMMSS/
  YYYYMMDD_HHMMSS_lifecycle.log
  YYYYMMDD_HHMMSS_navigation.log
  YYYYMMDD_HHMMSS_frontier_manager.log
  YYYYMMDD_HHMMSS_goal_manager.log
  YYYYMMDD_HHMMSS_dynamic_obstacle.log   # 仅 level_4
  YYYYMMDD_HHMMSS_navigation_metrics.log
```

生命周期日志必须在启动前写入解析后的 world、level、profile、控制器、速度、初始位姿、目标、Git revision 和 world hash，并记录 `start`、`process_ready`、`stop`、`exit_code`。每行至少包含时间戳、级别、进程名、事件名和关键参数。

停止条件分为三类：

1. **任务完成**：目标多帧确认、机器人到达停止条件，记录 `task_success` 后正常停止；
2. **可恢复失败**：单次规划失败、目标暂时丢失或动态障碍阻塞，保留运行继续观察恢复；
3. **基础设施失败**：Gazebo 崩溃、关键 topic 消失、TF 断裂或重复 spawn，立即停止并标为 `infrastructure_failure`。

日志保留 15 天。清理只能删除超过保留期的 timestamp 目录，不能按单个文件随意混删，不能把正式日志写入 `~/.ros/log` 作为唯一证据。

## 21. 失败定位决策树

```text
启动失败？
  |-- world XML/模型校验失败 -> 修复 generator/manifest/模型路径
  |-- Gazebo 启动但无 Pro3 -> 检查 lifecycle spawn 与 PIPELINE_CONFIG
  |-- 有 Pro3 但无 scan/odom -> 检查 robot plugin、topic remap、仿真时间
  |-- 有 scan/odom 但无 map -> 检查 SLAM、TF、时间同步
  |-- 有 map 但无 frontier -> 检查 frontier 开关和未知空间阈值
  |-- 有 goal 但无 path -> 检查 Navfn 可达性、门洞宽度、costmap
  |-- 有 path 但 cmd_vel 不连续 -> 检查 TEB、局部 costmap、速度限幅
  |-- 到房间但找不到目标 -> 检查相机、检测器、目标遮挡与任务 JSON
  |-- 目标频繁改动/原地转圈 -> 检查 goal ownership、锁定滞回、路线连续性
  |-- Level 4 才失败 -> 对比动态障碍服务日志和 Level 3 基线
```

每次修复只改变一个层次：先基础设施，再感知/建图，再全局规划，再局部控制，再任务管理。修复后必须重跑导致失败的最小级别，并至少做一次回归路线。

## 22. 参数与架构变更纪律

这个 benchmark 的目标是评估现有“在线 SLAM + frontier + Navfn + TEB + WeDetect”架构，不是通过大量参数把单次轨迹调出来。因此：

- 不修改 `scripts/config/pipeline_defaults.yaml` 的生产默认值；
- benchmark 专用覆盖只放在 `office_building_config.yaml`；
- 先记录现象和日志，再决定是否改参数；
- 若连续调参仍不能解决，优先检查 goal 所有权、状态机、地图/TF 和规划层级边界；
- 对 TEB 的速度、障碍距离、频率等改动必须附带前后指标；
- 控制器对比（TEB、DWA、RL）必须固定 world、起点、任务和重复次数；
- WeDetect 与 GroundingDINO 的对比必须固定相机输入、prompt、目标模型和评测距离。

## 23. 性能、资源和可复现性预算

Level 4 同时运行 Gazebo、SLAM、检测器、frontier、Navfn、TEB 和动态障碍节点。测试前记录：

```bash
nvidia-smi
free -h
df -h .
rostopic hz /pro3/rlscan /pro3/wheel_odom
```

若机器资源不足，优先按以下顺序降级：

1. 关闭 Level 4 动态障碍，回到 Level 3；
2. 关闭不参与当前问题的视觉节点，但保留导航基线；
3. 降低 Gazebo GUI 负载或使用无界面模式；
4. 减少证据录制分辨率；
5. 不得在未记录的情况下更换 world、删除模型或修改传感器频率。

每个结果必须能由 `run_timestamp + Git revision + world_sha256 + level + route_id` 唯一追溯。生成器是确定性的，修改生成器后必须重新运行 `generate` 并更新实际 hash；manifest 中的 `generated_at_build_time` 只表示由校验器现场计算 hash，不是跳过校验。

## 24. 回滚与安全边界

- 生产回归仍使用 `worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world`；benchmark 不覆盖它；
- 删除或停用 benchmark 时，只需停止 `run_office_building.sh` 启动的进程，不要删除生产模型和配置；
- 任何 world 生成错误都可以重新运行生成器，不能手工编辑生成结果后忘记同步生成器；
- 不把 Pro3 写入 benchmark world；
- 动态障碍节点只在 Level 4 窗口中运行，停止 benchmark 时必须一并结束；
- runtime 日志、PID、地图、录像和缓存不属于源码交付物；
- 真实测试前确认没有旧的 `lste`/Gazebo 进程，否则先执行 benchmark 的 `stop`。

## 25. 完成定义（Definition of Done）

只有同时满足以下条件，才能在汇报中称“办公楼 benchmark 完成”：

1. 生成器、world、manifest、配置、启动器、校验器和汇总器均在工作树中；
2. 四个 level 的静态校验全部通过；
3. Level 1 三次重复运行全部成功；
4. Level 2 三次至少成功两次，且目标干扰物没有被错误锁定；
5. Level 3 至少有一次真实绕过死路/墙后遮挡并成功到达目标的运行；
6. Level 4 至少有一次动态障碍运行，动态节点服务调用成功，失败可解释；
7. 关键 topic、TF、地图、Navfn path、TEB `/cmd_vel` 和目标事件均有日志或截图证据；
8. 汇总工具能从最新 metrics log 输出 JSON，且没有无效 glob、路径错误或空样本崩溃；
9. 文档记录已知限制、未完成项和下一步，不把静态实现误写成端到端验证完成；
10. Git 提交只包含源码、world、配置和文档，不包含 runtime 日志、缓存和大体积临时文件。

### 25.1 逐条验收审计（2026-08-27）

| DoD | 当前状态 | 证据或剩余动作 |
| --- | --- | --- |
| 1. 工程文件齐全 | 通过 | 生成器、world、manifest、配置、启动器、校验器、汇总器和轨迹绘图器均存在 |
| 2. 四级静态校验 | 通过 | `generate + validate level_1..level_4` 全部 `PASS`；hash 见第 27.1 节 |
| 3. Level 1 3/3 | 通过 | `20260827_221200`、`20260827_221850`、`20260827_222528` |
| 4. Level 2 至少 2/3 | 通过 | `20260827_223252`、`20260827_224125` 成功；`20260827_224900` 为可解释失败 |
| 5. Level 3 结构恢复 | 通过 | `20260827_225819` 完成 frontier stall、路线恢复和目标接近 |
| 6. Level 4 动态障碍 | 通过（有条件） | `20260827_231117` 完成；6787 次服务调用成功，2 次 abort 均有恢复日志 |
| 7. 关键运行证据 | 通过 | 每轮 timestamp 进程日志、metrics；Gazebo/RViz 截图和轨迹图见第 27.12 节 |
| 8. 汇总器健壮性 | 通过 | 5 个正式 metrics log 均能输出 JSON；未完成运行的完成时间为 `null` |
| 9. 限制和下一步 | 通过 | 第 12、23、30、32 节明确限制、失败分类和维护规则 |
| 10. Git 交付边界 | 通过 | `git diff --check` 已通过；最终 benchmark 提交仅包含源码、world、配置和文档，`runtime/` 未纳入 |

因此，场景、端到端 benchmark 验收事实和 Git 交付边界已经满足 DoD 1 至 10。
运行时日志仍保留在本机用于复核，但不属于提交内容。

## 26. 当前下一步（执行清单）

- [x] 在当前 ROS 环境运行四个 level 的 `validate`；
- [x] 执行一次 `start level_1`，确认 Gazebo、Pro3、传感器、SLAM、frontier、Navfn、TEB 和 goal manager；
- [x] 检查本次 timestamp 日志是否满足 `AGENTS.md` 的文件名、目录和字段规则；
- [x] 执行一次 `summary`，确认能读出 navigation metrics；
- [x] 放大 benchmark 专用黄色杯并重新确认目标评估尺寸，验证这是改善小目标可见性的场景修复，而不是放宽成功逻辑；
- [x] 根据当前 Kinect URDF 的实际传感器配置关闭 benchmark 的可选深度观测，避免把不存在的 depth topic 误报为感知失败；
- [x] 用 `target_entry` 隔离起点验证放大后的目标可以连续检测、锁定并完成接近；
- [x] 执行一次 `start level_4`，确认动态障碍物模型和 `/gazebo/set_model_state`（`20260827_231117`，6787/6787 成功）；
- [x] 补充 Gazebo/RViz 证据图、轨迹图和成功/失败分析；
- [x] 完成最终几何下 Level 1 三次真实任务运行（`20260827_221200`、`20260827_221850`、`20260827_222528`，3/3 成功）；
- [x] 完成 Level 2 三次运行（2/3 成功）、Level 3 死路恢复运行和 Level 4 动态障碍运行；
- [x] 把 Level 1 三次运行的 timestamp、结果、world hash 和关键失败/成功事件写入本节记录；
- [x] 根据真实 smoke test 更新本文件的“当前实现清单”和验收状态；
- [x] 在最终检查后统一提交 Git，继续保持 runtime 输出不入库（最终 benchmark 提交）。

## 27. 最新执行记录（2026-08-27）

本节记录计划执行过程中的可复核事实。它与第 25 节的完成定义分开：启动成功不等于任务成功，单次运行也不等于重复实验达标。

### 27.1 静态场景校验

以下命令已在当前工作区执行并全部通过；每次先生成并校验该 level 的独立 world 文件：

```bash
for level in level_1 level_2 level_3 level_4; do
  OFFICE_BUILDING_LEVEL="$level" \
    scripts/tests/office_building/run_office_building.sh generate
  scripts/tests/office_building/run_office_building.sh validate "$level"
done
OFFICE_BUILDING_LEVEL=level_1 \
  scripts/tests/office_building/run_office_building.sh generate
```

生成器对当前代码的预期 hash（每个 level 必须使用对应命令重新生成后再校验）为：

| level | world SHA-256 |
| --- | --- |
| `level_1` | `5c55979e53b2d029939f90d3e1c1fd3324cc02aca6c354e0f5b36d6e2b56da1b` |
| `level_2` | `6b9e2d314bf21ba568012142590a76252768a6af3d3dcbe0089cdc4dc4089d41` |
| `level_3` | `74b43250220fdc8107c52d646abfa608fe2c8b24b9c2834b21bf93967e886ce2` |
| `level_4` | `8b022c48d01a20a9e1551079a145e8395673b41fe15927cbadc7e1e8917c1c07` |

hash 是生成器、模型引用和文件内容的联合指纹。只要修改生成器、manifest、模型坐标、材质或 XML 排序，就必须重新生成并更新本表；实验日志始终以现场计算值为准。

### 27.2 干净 Level 1 启动验证

最近一次干净启动运行目录为：

```text
runtime/office_building_benchmark/logs/20260827_204532/
```

已确认：

- Gazebo、`lste-env`、`lste` 和 `lste-teleop` 会话建立，启动器不 attach 到任何会话；
- Pro3 只由生命周期流程生成一次；
- `/pro3/wheel_odom`、`/pro3/rlscan`、`/map`、`/lste/final_goal`、`/lste/detections`、`/move_base/status` 等关键接口出现；
- 在线 SLAM、frontier、Navfn、TEB、目标管理和 metrics 节点均注册；
- WeDetect-large 成功加载，MiniCPM prompt cache 正常工作；
- 健康审计最终记录 `status=healthy`、`failures=[]`、`cuda.ok=true`；
- metrics 汇总可正常输出 JSON，且本次没有 `move_base_abort`；
- 生命周期日志包含解析后的控制器、检测器、速度、起点、目标、world hash、Git revision、启动和停止事件；
- 本次只完成启动和导航过程验证，`task_done=false`，不计入 Level 1 任务成功次数。

本次运行暴露并修正了两个工程问题：检测器的 Python 文件名是 `lste_wedetect_det_node.py`，但 ROS 注册名是共享的 `/lste_det_node`；原目标桌和椅子又把目标接近点挤进了膨胀代价区。健康审计现在通过独立的 ROS 节点名参数检查检测器，目标房间则把桌组移到东侧，保留西侧可达观察通道。

### 27.3 当前完成度

截至本记录，场景实现、四级静态校验、启动闭环、日志汇总和代表性端到端实验均已完成。
Level 1 至 Level 4 的实际结果分别在第 27.8 至 27.11 节展开；Level 2 的第三轮失败被
保留为可解释负例，Level 4 的两次 abort 也有原始日志和恢复链路。后续只执行最终 Git
边界检查和归档维护，不再把这些已完成实验标记为待验证。

### 27.4 运行 `20260827_211228`：目标候选观察保持仍未完成任务

这轮运行使用 `level_1`、`primary` 起点、TEB、WeDetect-large 和 world hash
`9b3368850d7231cab419926dae4279857703f382e4fd74e0bf2f9c9c9fc32927`。
运行目录为：

```text
runtime/office_building_benchmark/logs/20260827_211228/
```

日志事实：

- 启动、Gazebo、Pro3、SLAM、frontier、Navfn、TEB 和检测器均上线；
- 首个弱目标候选在 ROS time `202.566 s` 出现，分数 `0.388`、框大小
  `0.012 x 0.014`，Goal Manager 进入最长 `8 s` 的候选观察保持；
- 第二个候选在 ROS time `223.356 s` 出现，分数 `0.503`、框大小
  `0.053 x 0.071`，但仍只有 `1/5` 个弱确认票；
- 运行期间没有出现 `target_follow_confirmed`、`target_close_confirmed` 或
  `task_done=true`，因此不能计入 Level 1 成功次数；
- `move_base_aborts=0`，但 frontier 仍发生多次 stall 释放和路线切换，说明导航基础设施没有崩溃，目标发现/持续观察与探索路线的协同仍未通过验收；
- 目标几何评估记录 `matched_detector_frames=2`，而深度话题的不可用计数持续增加；
  对照当前 Kinect URDF 可确认它只配置 RGB/CameraInfo，没有深度传感器，因此这不是
  WeDetect 的运行时错误。后续 benchmark 配置已将可选深度观测显式关闭，仍保留
  RGB 几何暴露和 detector 证据；
- 该轮应归类为 `failure_stage=perception` 与 `goal_management` 的联合诊断运行，
  不能只归因于 TEB。

后续最小复现顺序：固定同一个 world、起点和任务，先确认黄色杯在真实相机分辨率下
是否能连续产生候选帧，再检查深度话题和目标评估，最后才调整候选保持或 frontier
抢占策略。不得通过直接降低任务完成阈值把本轮运行改判为成功。

### 27.5 感知隔离运行 `20260827_212613`

为了把“目标不可见”与“全局探索没有走到目标房间”分开，使用 manifest 中的
`target_entry` 起点做了独立运行。该运行目录为：

```text
runtime/office_building_benchmark/logs/20260827_212613/
```

结果是 `diagnostic_only`，不计入 Level 1 的三次重复验收，但它证明了目标模型和
近距离接近链路本身可工作：

`target_entry` 是目标房间走廊侧的隔离起点（`[19.8, 13.8, 1.57079632679]`），
不是 manifest 中的目标真值坐标 `[21.6, 17.6]`。选择该 profile 会绕过正常的
frontier exploration；因此这类运行只能用于快速定位感知/接近链路，结果属于
diagnostic-only，不能作为最终 Level 4 benchmark evidence 或进入正式聚合。

- 第一帧候选分数 `0.619`、框大小 `0.037 x 0.044`；下一帧分数 `0.707`，
  中心偏移 `0.024`，达到 `target_follow_confirmed`；
- Goal Manager 连续提交 3 个可达观察段，最终在 `23.31,17.91` 附近停止；
- `target_close_confirmed` 和 `task_done=true` 均出现，耗时约 `25.2 s`；
- `move_base_aborts=0`，目标路线拒绝和非预期抢占均为 0；
- `min_scan_clearance=0.8911 m`，`turn_only_duration=0.119 s`；
- 配置中的 `target_eval_depth_enabled=false` 已在生命周期和 metrics 启动记录中解析
  生效，目标证据来自 RGB 检测、投影几何和接近状态。

这说明放大 benchmark 目标解决了远距离小目标的首要可见性问题；标准 `primary`
起点仍需重新运行，才能判断剩余失败是否来自 frontier 选择、地图覆盖或路线调度。

### 27.6 Goal Manager 两处状态机修复及最新运行快照

为解决“首帧检测出现但下一帧还没来，frontier 已经切换路线”和“接近目标时下一条
Navfn 射线被拒绝就清空目标跟踪”两个问题，`lste_goal_manager.py` 增加了两处有界的
状态机保护：

1. 首次弱目标检测会触发真实的 `navigation_hold`，冻结底盘并等待独立检测帧；确认
   目标后立即释放 hold，继续提交目标接近段。候选超时会自动释放，不会永久停车。
2. 已经进入近距离确认窗口后，如果下一条视觉射线被 Navfn 判定为 blocked，则保持当前
   位置和 close-vote，不立即清空 target track；只有近距离证据持续超时，才回退到语义
   frontier 恢复流程。

这两个修复只改变目标管理的事务边界，不修改生产默认 YAML，也不改变 TEB 的控制器
身份。验证时必须分别检查以下事件顺序：

```text
target_candidate_observation_hold
  -> target_follow_confirmed
  -> target_close_route_blocked_hold（若下一条路线不可达）
  -> target_close_confirmed
  -> target_task_completed
```

运行 `20260827_214140` 是第一处修复后的诊断运行：目标成功跟踪并完成 4 个视觉接近
段，close-vote 达到 `2/3`，但旧的路线释放逻辑仍清空了 target track；最终
`task_done=false`、`target_route_rejections=13`、`move_base_aborts=0`。该轮证明
“首帧观察保持”有效，但不计入 Level 1 成功次数。

运行 `20260827_214736` 已加载上述两处修复。快照显示 Gazebo、Pro3、SLAM、frontier、
Navfn、TEB 和 WeDetect 均正常，约 95 秒 ROS time 内没有 `move_base_abort`；截至快照
时尚未出现目标确认或 `task_done=true`，因此仍属于进行中的诊断运行，不能宣称 benchmark
通过。停止该运行后，必须用同一 timestamp 目录执行 `summary`，再把最终结果、失败阶段
和事件计数追加到本节。

### 27.7 Level 1 诊断运行 `20260827_215855`（旧目标几何）

在修复南侧开放入口导致的楼外 frontier 泄漏后，重新生成 Level 1 world 并执行了一次
任务运行。此时目标仍位于旧坐标 `(23.5, 19.0)`，尚未应用后续的入口可见漏斗布局。
运行目录为：

```text
runtime/office_building_benchmark/logs/20260827_215855/
```

现场配置和结果：

```yaml
level: level_1
profile: primary
controller: teb
detector: wedetect-large
world_sha256: 4d2dd811efcd22995790c1e124c6192a351a39143433a9ff3de83047d1c89ef8
initial_pose: [2.8, 11.0, 0.0]
target: [23.5, 19.0]
result: success_diagnostic_only
task_done_seconds: 187.946
path_length_m: 71.1292
move_base_aborts: 0
target_route_rejections: 0
target_segments_committed: 1
min_scan_clearance_m: 0.4198
evidence: 20260827_215855_gazebo.png
```

关键事件顺序为：`target_candidate_observation_hold` -> `target_follow_confirmed` ->
目标接近段提交 -> `target_close_confirmed` -> `task_done=true`。机器人最终位于目标
附近，任务完成后 `cmd_vel_mux` 进入 `task_complete` 并保持零速度。该轮只证明入口边界
修复后目标链路仍可完成，不计入最终几何的 Level 1 重复验收。

### 27.8 Level 1 最终几何重复验收

随后把目标工位移动到目标房间西侧入口后的可见漏斗区域 `(21.6, 17.6)`，并以新的
world hash `d1c9d42715334e70db8e4f60ad86764e9207eca10a9f782001465b95b9f8aa74` 固定几何。
在相同 `primary` 起点、TEB、WeDetect-large 和 `yellow_cup` 任务下完成了 3 次正式
重复运行：

| 次数 | run timestamp | 结果 | ROS 完成时间 | 路径长度 | goal changes | move_base aborts | 最小间距 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | `20260827_221200` | success | 216.642 s | 84.3142 m | 17 | 0 | 0.4254 m |
| 2 | `20260827_221850` | success | 304.546 s | 107.1304 m | 17 | 0 | 0.4346 m |
| 3 | `20260827_222528` | success | 224.336 s | 79.5552 m | 13 | 0 | 0.4244 m |

三次运行均满足：`target_follow_confirmed`、`target_close_confirmed`、`task_done=true`，
目标路线拒绝为 0，且没有 `move_base_abort`。第二次运行经历了 frontier stall 后的
路线恢复，仍成功进入目标房间并完成任务。三次独立 timestamp 目录均保留完整的进程
日志；因此 Level 1 的 3/3 重复性门通过，可以进入 Level 2。

### 27.9 Level 2 语义家具和干扰物运行

Level 2 固定最终几何、`primary` 起点、TEB、WeDetect-large、`yellow_cup` 任务和
world hash `12c161a61920cdb89b39e999c6d6e9b1ffcac6409b81ad0c9ba6a02766c3212b`，共执行
三轮。三轮都出现 `target_follow_confirmed`，成功轮还出现了
`target_close_confirmed` 和 `task_done=true`；没有日志表明蓝色杯或显示器被错误锁定。

| 轮次 | run timestamp | 结果 | 完成时间 | 路径长度 | goal changes | route rejections | aborts | 最小间距 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | `20260827_223252` | success | 302.117 s | 113.5656 m | 17 | 21 | 0 | 0.3913 m |
| 2 | `20260827_224125` | success | 251.684 s | 99.4289 m | 14 | 0 | 0 | 0.4157 m |
| 3 | `20260827_224900` | failure (可解释) | - | 109.2603 m | 25 | 9 | 0 | 0.4219 m |

第三轮不是基础设施失败：Gazebo、SLAM、frontier、Navfn 和 TEB 均持续运行，且
`move_base_aborts=0`。目标曾完成跟踪确认，但接近段连续被 Navfn 拒绝并释放，最终距离
目标 `7.6552 m`，没有进入 close-confirmed 状态。该轮归类为
`failure_stage=global_planning/goal_management`，保留为一个可复现的负例；因此 Level 2
按规定达到 `2/3` 成功门槛，而不是把失败轮删除或改判为成功。

### 27.10 Level 3 死路、墙后目标和路线恢复

运行 `20260827_225819` 使用 Level 3 world hash
`f41f3cee4441294a9e10d2a5b8da982f526bb179d158b1e512426ad0e3d0d4d2`。机器人先经历
frontier stall 和局部恢复，再释放死路路线并从其他 frontier 继续；日志中有
`move_base_recovery=2`，但没有 abort 或目标路线拒绝。

```yaml
run_timestamp: 20260827_225819
result: success
task_done_seconds: 371.223
path_length_m: 142.3788
goal_changes: 19
move_base_aborts: 0
move_base_recoveries: 2
target_route_rejections: 0
min_scan_clearance_m: 0.3953
target_follow_confirmed: true
target_close_confirmed: true
```

这轮满足 Level 3 的核心定义：不是把墙后目标直接当作可达点，而是在未知区域探索、
路线受阻和恢复后仍能到达目标房间并完成接近。

### 27.11 Level 4 动态障碍端到端运行

运行 `20260827_231117` 使用 Level 4 world hash
`9b63bba1d89987eeb7aff2fcf080948f049295abc97edc5ff9825560563fcc50` 和动态种子
`20260827`。动态节点以 10 Hz 调用 `/gazebo/set_model_state`，从
`x=8..15.5, y=11` 做固定三角波运动：

```yaml
run_timestamp: 20260827_231117
result: success
target_first_seen_seconds: 619.735
target_follow_confirmed_seconds: 629.329
target_close_confirmed_seconds: 654.796
task_done_seconds: 654.798
path_length_m: 177.769
goal_changes: 34
move_base_aborts: 2
move_base_recoveries: 9
speed_modulation_events: 38
min_scan_clearance_m: 0.4017
dynamic_service_calls: 6787
dynamic_service_failures: 0
```

两次 abort 分别发生在 ROS time `469.803 s` 和 `509.865 s`，原始错误均为
`Failed to find a valid plan. Even after executing recovery behaviors.`。随后 frontier
保留当前路线、执行 recovery 并选择新的可达分支，最终完成目标；因此它们是已解释且
可恢复的局部规划失败，不是未解释的系统崩溃。动态障碍日志的每次调用都包含计数、
elapsed、x/y、`success=True` 和服务返回消息，6787 次全部成功。

### 27.12 证据文件和可复核路径

当前已保存的有效证据如下。运行日志均在各自的 timestamp 父目录内，截图不作为唯一
结论，必须与对应 metrics 和生命周期日志一起查看：

| 证据 | 路径 | 用途 |
| --- | --- | --- |
| Level 1 Gazebo | `runtime/office_building_benchmark/logs/20260827_215855/20260827_215855_gazebo.png` | 目标房间、模型和机器人画面 |
| Level 2 Gazebo | `runtime/office_building_benchmark/logs/20260827_224900/20260827_224900_gazebo.png` | 复杂办公环境画面 |
| Level 2 RViz | `runtime/office_building_benchmark/logs/20260827_224900/20260827_224900_rviz.png` | 地图、代价地图和导航可视化 |
| 轨迹/平面图 | `runtime/office_building_benchmark/evidence/office_building_trajectories_20260827.png` | 房间语义、标准起点、目标和四轮轨迹对照 |
| Level 2 metrics | `runtime/office_building_benchmark/logs/20260827_223252/20260827_223252_metrics.json`、`20260827_224125_metrics.json`、`20260827_224900_metrics.json` | 三轮可机读指标汇总 |
| Level 3 metrics | `runtime/office_building_benchmark/logs/20260827_225819/20260827_225819_metrics.json` | 恢复运行指标 |
| Level 4 dynamic log | `runtime/office_building_benchmark/logs/20260827_231117/20260827_231117_dynamic_obstacle.log` | 动态服务逐次调用 |
| Level 4 metrics | `runtime/office_building_benchmark/logs/20260827_231117/20260827_231117_metrics.json` | 动态端到端指标汇总 |

绘图命令为：

```bash
python3 scripts/tests/office_building/plot_benchmark_evidence.py \
  --output runtime/office_building_benchmark/evidence/office_building_trajectories_20260827.png \
  runtime/office_building_benchmark/logs/20260827_223252/20260827_223252_navigation_metrics.log \
  runtime/office_building_benchmark/logs/20260827_224125/20260827_224125_navigation_metrics.log \
  runtime/office_building_benchmark/logs/20260827_225819/20260827_225819_navigation_metrics.log \
  runtime/office_building_benchmark/logs/20260827_231117/20260827_231117_navigation_metrics.log
```

轨迹图是汇报证据，不改变导航输入，也不参与任务成功判定。

当前汇总中的 `map_coverage` 为 `null`，原因是现有 metrics 节点没有发布统一的建筑
覆盖率真值，而不是地图没有增长。地图增长由 `/map`、frontier 日志和 RViz 画面核验；
如果后续需要覆盖率百分比，必须新增独立的地图评估器并在所有 level 固定同一算法，不能
用手工估计值回填历史运行。

## 28. 端到端验收执行计划

本节是实际执行时的主流程。每一次运行只能有一个明确的实验目的；不得在同一次运行中同时改变 world、检测器、TEB 参数和目标管理逻辑，否则失败无法归因。

### 28.1 阶段门（stage gates）

| 阶段门 | 必须完成的事实 | 进入条件 | 失败时动作 |
| --- | --- | --- | --- |
| G0 环境可用 | ROS、Gazebo、模型、磁盘和 CUDA 可用 | `validate` 全部通过，关键模型存在 | 修复依赖或模型路径，不启动任务 |
| G1 启动闭环 | Gazebo、Pro3、传感器、SLAM、frontier、Navfn、TEB 和检测器上线 | `start level_1` 后 topic/节点稳定至少 60 秒 | 按第 21 节决策树定位基础设施 |
| G2 建图导航 | 地图增长，机器人能通过标准门洞和主走廊 | Level 1 骨架运行无基础设施失败 | 先隔离 SLAM、TF、costmap 和 Navfn |
| G3 目标接近 | 目标多帧确认，路线可达，接近后稳定停止 | Level 1 目标任务成功 | 检查杯子尺寸、遮挡、相机、目标评估，不先放宽阈值 |
| G4 语义复杂度 | 多杯子干扰和家具绕行仍能完成 | Level 2 至少 3 次完成 2 次 | 分离感知误锁定、路线抢占和 TEB 局部问题 |
| G5 结构恢复 | 死路、墙后目标和替代路线能恢复 | Level 3 至少一次可解释恢复并到达 | 检查 frontier 选择、Navfn 可达性和 goal ownership |
| G6 动态演示 | 动态障碍服务和 TEB 避障可复现 | Level 4 至少一次完整运行 | 对比 Level 3，先验证动态节点再调整控制参数 |

任何阶段门未通过，都不能跳到更高 level 宣布“系统完成”。

### 28.2 G0：运行前检查

```bash
cd .
source /opt/ros/noetic/setup.bash
source devel/setup.bash
git rev-parse HEAD
git status --short
nvidia-smi
free -h
df -h .
for level in level_1 level_2 level_3 level_4; do
  OFFICE_BUILDING_LEVEL="$level" \
    scripts/tests/office_building/run_office_building.sh generate
  scripts/tests/office_building/run_office_building.sh validate "$level" || exit 1
done
OFFICE_BUILDING_LEVEL=level_1 \
  scripts/tests/office_building/run_office_building.sh generate
```

记录以下结果到实验记录：Git revision、工作树是否干净、模型路径、world hash、可用显存、可用磁盘和四个校验结果。若磁盘不足，先清理过期 runtime 日志和缓存，但不删除源码模型或未过期证据。

### 28.3 G1：启动和 60 秒稳定性检查

```bash
scripts/tests/office_building/run_office_building.sh start level_1
scripts/tests/office_building/run_office_building.sh status
tmux list-sessions
rosnode list
rostopic list | rg '/(pro3/wheel_odom|pro3/rlscan|map|lste/detections|lste/final_goal|move_base/status)$'
rostopic hz /pro3/wheel_odom /pro3/rlscan
```

60 秒内需要确认：Gazebo 只有一个 `pro3`；odom 和 scan 持续发布；`/map` 不断更新；TF 没有持续断裂；`/move_base/status` 存在；检测器和目标管理节点没有重复实例；启动器没有 attach 到 tmux。任何进程崩溃都记录为 `infrastructure_failure`，不得计入任务失败率。

### 28.4 G2/G3：Level 1 基线和目标任务

Level 1 只验证建筑骨架、主走廊、目标房间入口和最少家具。固定 `primary` 起点、`yellow_cup` 任务、TEB 和 WeDetect-large。每次运行：

1. 启动 `level_1`，记录 timestamp 和现场 world hash；
2. 等待地图、frontier 和 Navfn 就绪；
3. 观察机器人是否沿可达走廊前进，不手动干预目标；
4. 记录首次目标检测、目标锁定、目标路线接受、最终停止或失败；
5. 运行结束后先 `summary`，再 `stop`，保留整个 timestamp 目录；
6. 重复三次，三次都完成才通过 Level 1。

Level 1 的失败分类必须是以下之一：`startup`、`mapping`、`global_planning`、`local_control`、`perception`、`goal_management` 或 `target_evaluation`。只写“没走到”不算有效失败分析。

### 28.5 G4：Level 2 语义家具和干扰物

Level 2 开启开放办公区、会议室、茶水间、打印区和多杯子干扰。除 level 外不改变配置。至少执行三次：

- 一次直接走廊路线 `direct_corridor`；
- 一次办公区上下文路线 `office_context`；
- 一次重复最容易失败的路线，用于统计稳定性。

通过条件是至少完成 2/3，且不能把蓝色/绿色杯或显示器错误锁定为黄色杯。发现误锁定时，保存检测器原始日志和目标管理日志，不能只看最终轨迹。

### 28.6 G5：Level 3 死路和墙后目标恢复

Level 3 开启储物间死路、替代环路、遮挡目标和窄门接近。核心实验是 `dead_end_recovery`：

1. 验证 frontier 可以进入储物间入口；
2. 验证深入死路后不会无限重试同一目标；
3. 验证 Navfn/goal manager 能释放失败目标并选择仍未探索的分支；
4. 验证机器人最终从另一条路线到达目标房间；
5. 记录死路进入、规划失败、回退、重新选点和目标重捕获事件的时间顺序。

如果目标杯在目标房间内仍过小，先使用 benchmark 专用模型 scale 和真实相机分辨率修复可见性，再评估目标评估尺寸；禁止通过单纯放宽完成距离掩盖检测问题。

### 28.7 G6：Level 4 动态障碍

Level 4 在 Level 3 几何基础上启动一个固定种子的动态障碍。启动后必须分别验证：

```bash
rosservice list | rg '/gazebo/set_model_state'
rostopic echo -n 1 /gazebo/model_states
```

动态节点日志需记录模型名、seed、周期、轨迹边界、每次服务调用和返回状态。验收时检查模型确实移动、TEB 有减速或绕行行为、最小间距没有低于安全阈值，且障碍停止后机器人可以继续前进。动态障碍服务失败不能归因给 TEB。

### 28.8 每轮实验的固定顺序

```text
准备依赖
  -> 生成对应 level world
  -> 静态校验
  -> stop 旧进程
  -> start 新运行并创建 timestamp 日志目录
  -> 等待节点 ready
  -> 执行一条固定路线
  -> 采集 topic、metrics、截图和日志
  -> summary
  -> stop
  -> 归类结果
  -> 只有达到阶段门才升级 level
```

不得在运行中手工修改生产 YAML、移动目标杯、删除障碍或重启单个节点后把结果当作同一次正式实验。确需临时诊断时，必须标记为 `diagnostic_only`，不纳入通过次数。

## 29. 实验记录模板

每次正式运行在外部实验归档中建立一个与日志 timestamp 同名的记录。可以直接复制以下模板：

```yaml
run_timestamp: YYYYMMDD_HHMMSS
scenario_id: office_building_v1
level: level_1
route_id: direct_corridor
robot_profile: primary
task_id: yellow_cup
controller: teb
detector: wedetect-large
world_sha256: <sha256sum output>
git_revision: <git rev-parse HEAD>
git_dirty: false
dynamic_obstacle_seed: null
initial_pose: {x: 2.8, y: 11.0, z: 0.1, yaw: 0.0}
target_truth: {model: cup_yellow, x: 21.6, y: 17.6}
result: success|failure|diagnostic_only
failure_stage: none|startup|mapping|global_planning|local_control|perception|goal_management|target_evaluation
failure_signature: ""
log_dir: runtime/office_building_benchmark/logs/YYYYMMDD_HHMMSS
evidence:
  - YYYYMMDD_HHMMSS_gazebo.png
  - YYYYMMDD_HHMMSS_rviz.png
  - YYYYMMDD_HHMMSS_metrics.json
  - YYYYMMDD_HHMMSS_failure_notes.md
```

`failure_signature` 必须引用实际日志事件，例如 `target_route_rejected:12`、`move_base_abort:0`、`goal_changed:4` 或 `dynamic_service_error:1`。不要用主观描述替代可检索的事件名。

## 30. 结果判定、失败复现与修复纪律

### 30.1 结果判定

- `success`：任务完成条件满足，日志完整，关键指标没有未解释异常；
- `failure`：任务未完成，但基础设施和日志完整，能确定失败阶段；
- `infrastructure_failure`：Gazebo、ROS、TF、传感器或关键节点异常，单独统计，不混入控制器成功率；
- `diagnostic_only`：为定位问题的临时运行，不能改变任何 level 的通过计数。

### 30.2 最小复现原则

遇到失败时，按最小范围复现：

1. 保留失败运行的 world hash、Git revision、配置和日志；
2. 用同一个 level、route、起点和目标重复一次；
3. 只改变一个层次：基础设施 -> 感知/建图 -> Navfn -> TEB -> goal manager；
4. 修复后先重跑失败的最小 level，再重跑一条已通过的回归路线；
5. 将修复前后指标写入记录，不以单次“看起来好了”作为结论。

### 30.3 停止和清理

正式运行结束后执行：

```bash
scripts/tests/office_building/run_office_building.sh summary
scripts/tests/office_building/run_office_building.sh stop
```

确认 `stop` 后没有 benchmark 的 Gazebo、LSTE、动态障碍或 ROS 节点残留。日志目录保留 15 天；只清理过期的完整 timestamp 目录。源码、world、manifest、配置、脚本和文档不因清理 runtime 而改变。

## 31. 计划完成后的汇报材料

最终汇报必须至少包含：

1. 一张带房间语义标注的办公楼平面图；
2. Level 1 至 Level 4 的配置和 world hash 表；
3. 一张成功运行的 Gazebo/RViz 截图或录屏；
4. 一张机器人轨迹与目标路线图；
5. 一张指标表，包含成功率、耗时、路径长度、规划失败、急停、最小间距和目标锁定时间；
6. 一个失败案例，展示日志时间线、失败阶段和修复前后差异；
7. 明确列出尚未覆盖的多层建筑、玻璃反射、大量行人和多目标任务等限制。

汇报材料引用的每个数字都必须能回溯到一个 `run_timestamp` 目录和对应 Git revision，不能手工拼接不同 world 或不同配置的结果。

## 32. 文档维护规则

本文件同时是设计说明、执行手册和验收记录。为了避免计划与代码逐渐分叉，后续变更遵循以下规则：

1. **场景变更**：修改房间、墙体、门洞、家具、目标或动态障碍时，先修改
   `generate_world.py`、manifest 或 level 配置，再重新生成 world；同时更新对应的
   world SHA-256、场景图和受影响的验收级别。不要只手工修改生成后的 `.world`。
2. **启动变更**：修改启动脚本、环境变量、节点或日志目录规则时，更新第 9、10、16、
   17、20 节，并重新执行 G0/G1。启动命令必须能从干净 shell 复制执行。
3. **导航或感知变更**：每次只改变一个架构层，记录修改前后的 Git revision、参数、
   运行 timestamp 和指标；把结论追加到第 27 节或对应实验记录，不覆盖历史事实。
4. **验收状态**：只有实际运行产生完整日志和证据后，才能把第 26 节的复选框改为已完成。
   静态校验通过、节点启动或一次诊断成功，都不能代替重复任务验收。
5. **日志与证据**：正式日志、地图缓存、截图和录屏保留在 `runtime/` 或外部归档，不
   提交 Git；文档只引用其 timestamp、路径和摘要指标。日志目录按 `AGENTS.md` 规则保留
   15 天，清理时按完整 timestamp 目录删除。
6. **版本发布**：达到第 25 节 Definition of Done 后，在一个明确里程碑统一提交源码、
   world、manifest、配置、脚本和文档；提交前运行 `git diff --check`，并确认没有 runtime
   产物、模型权重或个人环境文件。

### 32.1 当前文档状态

截至 2026-08-27，本文件已完成计划、实现清单、执行步骤、日志规范、验收审计和实际
运行记录的整理。Level 1 已完成 3/3，Level 2 已完成 2/3，Level 3 和 Level 4 各有
一次成功的代表性端到端运行，动态服务、Gazebo/RViz 截图和轨迹证据也已归档。
第 25 节 DoD 的 1 至 10 项已经满足；最终 benchmark 提交只包含源码、world、配置和文档，
runtime 日志仍按规则保留但不入库。

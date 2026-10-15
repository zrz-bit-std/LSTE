# LSTE 文档索引

除固定索引 `docs/introduction.md` 外，`docs/` 下的文件名统一使用
`YYYYMMDD_主题.扩展名` 格式；Markdown 文档使用 `YYYYMMDD_主题.md`。日期表示
该文件在本次目录整理中的记录版本；正文中的更新日期仍保留各文档自身的技术
更新时间。

## operations

| 文档 | 内容 |
| --- | --- |
| [20260723_setup.md](operations/20260723_setup.md) | 工作区、依赖与环境搭建 |
| [20260723_startup_guide.md](operations/20260723_startup_guide.md) | 日常启动、controller 切换和 tmux 操作 |

## navigation

| 文档 | 内容 |
| --- | --- |
| [20260723_sappo_navigation.md](navigation/20260723_sappo_navigation.md) | LSTE global goal 到 SA-PPO 的现有集成 |
| [20260723_gp_subgoal_business_flow.md](navigation/20260723_gp_subgoal_business_flow.md) | GP subgoal 的业务流程 |
| [20260723_s_env_math.md](navigation/20260723_s_env_math.md) | S-env 的运行时数学说明 |
| [20260723_fixed_goal_navigation_debugging.md](navigation/20260723_fixed_goal_navigation_debugging.md) | 固定目标绕墙测试、故障定位和 TEB 基线 |
| [20260805_fixed_global_goal_integration.md](navigation/20260805_fixed_global_goal_integration.md) | 固定坐标接入正式 `/lste/final_goal`、配置与验证证据 |
| [20260806_online_slam_teb_navigation.md](navigation/20260806_online_slam_teb_navigation.md) | 未知环境在线 SLAM、frontier、Navfn、TEB 接入和端到端验证 |

## perception

| 文档 | 内容 |
| --- | --- |
| [20260723_visual_pipeline.md](perception/20260723_visual_pipeline.md) | 视觉链路、检测性能和框跟随 |
| [20260723_wedetect_tensorrt_integration.md](perception/20260723_wedetect_tensorrt_integration.md) | WeDetect TensorRT 接入与缓存 |

## research

| 文档 | 内容 |
| --- | --- |
| [20260723_open_vocabulary_detector_research.md](research/20260723_open_vocabulary_detector_research.md) | 开放词汇检测器的调研和选型 |
| [20260823_semantic_navigation_architecture_survey.md](research/20260823_semantic_navigation_architecture_survey.md) | 保留 LSTE global goal 的语义导航架构调研与实施建议 |
| [20260905_place_portal_workitem_study.md](research/20260905_place_portal_workitem_study.md) | Place-Portal-WorkItem 方法边界、基线、指标、复现实验和论文证据规则 |
| [20260906_task_conditioned_place_portal_belief.md](research/20260906_task_conditioned_place_portal_belief.md) | 任务条件语义信念、方向 WorkItem、主动门洞探测、持久 Portal 和目标射线验证架构 |
| [20260906_graph_first_architecture_innovation.md](research/20260906_graph_first_architecture_innovation.md) | 基于近期研究的图优先动作架构、实现边界与验证计划 |
| [20260906_portal_probe_value_pareto.md](research/20260906_portal_probe_value_pareto.md) | Portal probe 的硬约束、动作类别和 Pareto 选择层 |
| [20260906_place_phase_branch_transaction.md](research/20260906_place_phase_branch_transaction.md) | Place phase、Portal transaction、失败缓存失效和近期拓扑探索架构落地 |
| [20260906_branch_first_evidence_gated_exploration.md](research/20260906_branch_first_evidence_gated_exploration.md) | Branch-first 证据门控策略、事务身份校验、可证伪假设与消融指标 |
| [20260906_portal_decision_architecture.md](research/20260906_portal_decision_architecture.md) | 任务条件 Portal 决策层、无权重 Pareto 选择和外部架构依据 |
| [20260906_frontier_action_evidence_policy.md](research/20260906_frontier_action_evidence_policy.md) | Place 内 ObservationWorkItem 的事件类别、证据向量、Pareto 选择和实验边界 |
| [20260906_event_driven_evidence_graph.md](research/20260906_event_driven_evidence_graph.md) | Fast-Slow 事件图、可回放证据投影和目标/Portal 身份边界 |
| [20260906_graph_route_planner.md](research/20260906_graph_route_planner.md) | 持久 Place/Portal/WorkItem 图级路线规划器、第一条边物化和回归验证 |
| [20260906_controller_owned_route_lease.md](research/20260906_controller_owned_route_lease.md) | 持久执行中的控制器失败终端所有权与路线 lease 架构 |
| [20260906_recent_active_topology_architecture.md](research/20260906_recent_active_topology_architecture.md) | 2025-2026 未知环境语义拓扑探索调研、代码可用性和 ECAG 架构建议 |
| [20260906_durable_portal_action_compiler.md](research/20260906_durable_portal_action_compiler.md) | 持久 Portal 证据到 crossing route 的无 frontier 物化、preobserved crossing 和验证边界 |
| [20260907_target_viewpoint_option_ledger.md](research/20260907_target_viewpoint_option_ledger.md) | 目标拥有的视点选项账本：将局部 Navfn/TEB 失败与语义目标身份解耦 |
| [20260907_optimistic_slow_planning.md](research/20260907_optimistic_slow_planning.md) | 慢速图规划与快速执行的提案提交协议，避免长规划锁住终端事件 |

## development

| 文档 | 内容 |
| --- | --- |
| [20260905_global_frontier_code_layout.md](development/20260905_global_frontier_code_layout.md) | Global frontier 主节点与策略模块的职责划分 |
| [20260905_goal_manager_code_layout.md](development/20260905_goal_manager_code_layout.md) | Goal Manager 的检测、投影、路由和跟随模块划分 |
| [20260905_telemetry_code_layout.md](development/20260905_telemetry_code_layout.md) | 导航遥测与 TEB bridge 的模块边界和验证方式 |
| [20260907_targeted_navigation_experiments.md](development/20260907_targeted_navigation_experiments.md) | 从秒级最小故障回放逐步扩展到 Level 4 的实验阶梯 |
| [20260908_navigation_failure_evidence.md](development/20260908_navigation_failure_evidence.md) | 失败 episode、route stall 归因、目标 lease 释放和标准摘要 |
| [20260908_single_threaded_event_queue_tick_fsm.md](development/20260908_single_threaded_event_queue_tick_fsm.md) | 任务与路线生命周期的单线程事件队列、Tick FSM 实现和验证结果 |
| [20260909_current_navigation_blocker.md](development/20260909_current_navigation_blocker.md) | 面向导航人员的当前目标、系统状态、route 9 卡点和后续架构边界 |

## reporting

| 文档 | 内容 |
| --- | --- |
| [20260805_teacher_progress_report.md](reporting/20260805_teacher_progress_report.md) | 面向老师的阶段性进展汇报：接手边界、完成工作、真实实验结果与后续计划 |
| [20260805_project_delivery_report.md](reporting/20260805_project_delivery_report.md) | 阶段性交差报告：架构、控制器对比、真实截图与视频证据 |
| [20260725_fixed_goal_video_evidence.md](reporting/20260725_fixed_goal_video_evidence.md) | 固定目标控制器录像与日志索引 |

## testing

| 文档 | 内容 |
| --- | --- |
| [20260827_office_building_benchmark_plan.md](testing/20260827_office_building_benchmark_plan.md) | 复杂办公楼 Gazebo 场景、任务路线、难度分层与评测计划 |

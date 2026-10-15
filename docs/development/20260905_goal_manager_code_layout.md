# Goal Manager 代码布局

## 为什么拆分

`lste_goal_manager.py` 仍然是 ROS 节点的主编排器，但检测证据和 Navfn 服务事务各自有独立的生命周期。把这两块放在主文件中，会让订阅回调、目标仲裁和路由校验互相穿插，修改一个策略时很难判断影响范围。

现在主文件保留稳定的 ROS 方法名，新模块只接收宿主对象和消息，继续使用原来的参数、状态和发布接口，因此不会改变话题或参数契约。

## 模块职责

| 模块 | 负责内容 | 修改入口 |
| --- | --- | --- |
| `lste_goal_manager.py` | ROS 初始化、状态回调、目标仲裁和发布 | 需要改变节点生命周期或话题时 |
| `goal_manager_config.py` | 目标跟踪、导航、激光和诊断参数归一化 | 需要新增或调整 Goal Manager 参数时 |
| `goal_manager_detection.py` | 单帧检测过滤、目标 track、证据累计、图像射线平滑 | 需要改变检测确认或目标跟踪策略时 |
| `goal_manager_route_validation.py` | Navfn `make_plan` 请求、缓存、TF/服务失败状态和事件记录 | 需要改变视觉目标可达性判定时 |
| `goal_manager_projection.py` | 相机射线、LaserScan 裁剪和 TF 几何工具 | 需要改变坐标变换或传感器投影时 |
| `goal_manager_viewpoint.py` | 目标接近视点梯度的 Navfn 校验、连续性评分和选择 | 需要改变视觉目标候选点排序时 |
| `goal_manager_target_follow.py` | 目标跟随生命周期、终点重观测和探索回退 | 需要改变目标锁定或跟随状态机时 |

## 调用关系

```text
/lste/detections
  -> GoalManager.on_dets()
  -> goal_manager_detection.handle_detections(manager, message)
  -> 更新 target track / heading / observation hold

GoalManager.validate_target_route()
  -> goal_manager_route_validation.validate_target_route(manager, goal, now)
  -> Navfn make_plan
  -> True / False / None（三态结果）

GoalManager.commit_target_segment()
  -> goal_manager_viewpoint.select_target_viewpoint(manager, now)
  -> 视点梯度逐个校验 Navfn
  -> 选择连续性代价最低的可达终点

GoalManager.compute_goal()
  -> GoalManagerTargetFollowMixin.goal_from_target_follow()
  -> target route / close re-observation / frontier fallback
```

`True` 表示当前视觉目标点有可用路径，`False` 表示 Navfn 明确返回空路径，`None` 表示 TF 或服务尚未就绪。三态语义保持在路由模块中，主类只负责把结果接入目标仲裁。

主类初始化按三个阶段阅读：`_load_parameters()` 读取并归一化参数，状态缓存随后建立，`_setup_ros_interfaces()` 最后创建 publisher、subscriber 和 timer。新增参数时放入前一个阶段，不要把参数读取重新塞回 ROS 接口创建代码。

参数加载和几何工具通过 mixin 注入 GoalManager。这样保留了原有的
`self.method(...)` 调用方式，也避免在 ROS 主文件里混入大量与生命周期无关的
配置和数学细节。修改这些模块时，优先保持方法名和 `self` 属性契约稳定。

## 安装空间

这些模块会和 ROS 可执行脚本一起安装到 `CATKIN_PACKAGE_BIN_DESTINATION`。主脚本启动时把自身目录加入 `sys.path`，所以源码空间、devel 空间和 install 空间使用相同的导入方式。

当前共有七个 Goal Manager 文件：主编排器、配置 mixin、检测证据模块、路由校验模块、投影工具模块、视点选择模块和目标跟随 mixin。它们必须一起安装；只复制主脚本会在 install-space 启动时缺少导入模块。

## 验证

```bash
python3 -m py_compile \
  src/lste_topo_access/scripts/lste_goal_manager.py \
  src/lste_topo_access/scripts/goal_manager_config.py \
  src/lste_topo_access/scripts/goal_manager_detection.py \
  src/lste_topo_access/scripts/goal_manager_projection.py \
  src/lste_topo_access/scripts/goal_manager_route_validation.py \
  src/lste_topo_access/scripts/goal_manager_viewpoint.py \
  src/lste_topo_access/scripts/goal_manager_target_follow.py
python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
```

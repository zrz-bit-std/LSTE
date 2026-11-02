# Gazebo 崩溃定位与 L 优先复现

## 已证实的退出故障

完整硬件渲染 L 的 `20261029_174221` 已导航就绪并产出相机帧，退出时 GDB 捕获：

`ros::NodeHandle::ok()` ← `GazeboRosVelodyneLaser::laserQueueThread()`。

实际库是 `/home/zrz/work/lste-previous/pre_ws/devel/lib/libgazebo_ros_velodyne_laser.so`。实际析构先 shutdown/delete `nh_`，随后才 join，而后台线程循环仍读取 `nh_->ok()`，存在释放后访问。

最小修改：shutdown 后先检查 joinable 并 join，再 delete/null 句柄。激光范围、噪声、点云与物理参数未改变。原源/so、构建日志和修改后 SHA 位于 `runtime/gp_branch_development/place_dfs_parent_return_fix_20261028_0310/velodyne_shutdown_fix_20261029_1746/`。

## 启动渲染故障的证据范围

`20261029_173002` 的另一次 SIGSEGV 位于 OGRE `RenderSystem_GL::GLRenderSystem` / `RenderEngine::Load`，尚未加载世界。实际环境为 llvmpipe 软件渲染。独立世界初始化对照（含相同 ROS 系统插件）软件与硬件各五次均加载，未在该隔离测试复现；不能据此声称唯一原因已经证明。

当前复现使用已实测可用的 Intel iris 硬件路径：`DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=0`，保留完整相机/点云与 gdb 捕获。这是绕开已捕获软件路径的运行处理，仍以真实完整运行和正常退出验证稳定性。

## 修复后的重复验证

`l_full_startup_20261029_174638/result.json`：三个完整机器人/相机/语义节点启动均 navigation_ready，实际加载 iris 与修改后 Velodyne 库，三个相机 header 有至少两个递增时间戳，20 秒运行及正常清理无 SIGSEGV。记录分别为174638、174710、174742。传感帧时间另存 `l_startup_camera_advancing_time_evidence.json`。

4GB、八轮内存模式回读均匹配，只是短诊断阴性结果，不排除硬件问题。历史 metrics 随机坏字节原因仍未证明，旧记录不修补、不跳过。

## 当前验收顺序

当前仅 L 完整复现：`place_dfs_suite_20261029_175001`，ROS11754。先完成 L 的全部真实终局、严格认证/转向/native/身份几何/普通WI全轨迹/TSP/DFS/生命周期及图件，成功后才进入 T、旋转T、十字。三次启动短测不算整场成功；四场 goal 尚未完成。

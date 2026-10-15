"""One focused responsibility extracted from the legacy GP frontier node."""

import numpy as np
import rospy

class GpFrontierRuntimeMixin:
    def step(self):
        """节点主循环：从最新传感器缓存计算 frontier、拓扑和调试输出。

        每轮可按以下顺序理解：

        1. 等待 pose 与 sph_pcl；
        2. 推进旧的回退/强制分支状态；
        3. 点云下采样并训练 GP；
        4. 在球面网格预测占据与不确定度；
        5. 提取并发布全部 frontier；
        6. 更新拓扑快照和 RViz 点云；
        7. 保留旧子目标接口的直达/绕障分支。

        回调只覆盖最新输入，因此本循环可能对同一帧运行多次；循环频率由
        GP 计算耗时决定，而不是由 /sph_pcl 的发布频率直接限定。
        """
        while not rospy.is_shutdown():
            # [1] 没有位姿无法把相对方向放进 odom；没有球面点云无法训练 GP。
            if self.pose is None or self.pcl_arr is None:
                rospy.logwarn_throttle(5.0, "gp_subgoal: waiting for rbt_pose and sph_pcl ...")
                rospy.sleep(0.05)
                continue
            # [2] 启动初期 Goal Manager 尚未发布 final_goal 时，先给 GP 一个
            # “正前方 10 m”的内部评分参考。它不会直接驱动车辆。
            if (not self._start_goal_seeded) and (not self.final_goal_received) and self.pose is not None:
                try:
                    # Pose2D: x, y, theta（theta 为朝向）
                    forward_dist = 10.0
                    self.gl_x = self.pose.x + forward_dist * np.cos(self.pose.theta)
                    self.gl_y = self.pose.y + forward_dist * np.sin(self.pose.theta)
                    self.gl_yaw = self.pose.theta
                    self.gl_wrt_odom = np.array([self.gl_x, self.gl_y, 1], dtype="float32")
                    self._start_goal_seeded = True
                except Exception:
                    pass
            # [3] Access-Topo 先兑现上轮决定：检查回退是否到点、强制方向
            # 是否已走够距离。可能更新 /lste/access_topo/mode 或回退目标。
            self.check_backtrack_progress()
            self.update_forced_heading_progress()
            # [4] 构造 robot <-> odom 的二维变换，并计算到内部参考目标的距离。
            self.tf_rbt_2_odom()
            self.tf_odom_2_rbt()
            self.rbt2gl_error()


            # [5] 将最新 /sph_pcl 整理为稀疏训练样本：(theta, alpha) -> occupancy。
            self.downsample_pcl(self.pcl_arr)

            # [6] 每轮以当前局部观测创建 GP，并在固定球面网格预测：
            # gp_grd_oc 为占据均值，gp_grd_var 为模型不确定度。
            self.gp_nav_din = np.column_stack((self.pcl_thetas, self.pcl_alphas))
            self.gp_nav_dout = np.array(self.pcl_oc, dtype='float').reshape(-1, 1)

            self.gp_nav_fit(0.09, 0.11, 0.7, 10, 10,
                            0.005)  #(ls1, ls2, var, alpha, noise, noise_var)
            nav_grd_oc, nav_grd_var = self.gp_nav.model.predict_f(self.gp_grd)
            self.gp_grd_var = nav_grd_var.numpy()
            self.gp_grd_oc = nav_grd_oc.numpy()
            self.gp_grd_rds = self.oc_srfc_rds - self.gp_grd_oc

            # [7] 方差图 -> 多个 frontier；随后发布给 Goal Manager，并同步维护
            # 路口/分支/回退快照。Goal Manager 而非这里发布最终 final_goal。
            self.gp_nav_mask_thrshld()
            self.gp_grd_var_img()
            self.gp_nav_pkup_nav_pt()
            self.publish_frontiers()
            self.update_global_goal_periodic()  # 每隔 gl_update_period 秒基于当前 frontier 更新全局目标
            self.dump_access_topo()
            ####calculate navigation point in world frame zcy topo模式下不发布subgoal及可视化
            # self.gp_nav_xypts_actul_pcl()
            # self.gp_nav_pts_pcl()

            # [8] 将 GP 预测转换回 PointCloud2，仅用于 RViz/调试观察。
            self.gp_grd_oc_pcl()
            self.gp_grd_var_pcl()

            # [9] 以下是保留的旧子目标接口：判断内部参考目标方向的预测可行
            # 距离，并据此发布 gp_subgoal/gp_subgoal_os 等话题。当前 LSTE 主链
            # 主要使用第 [7] 步的 /lste/gp_frontiers + Goal Manager。
            # 将内部目标从 odom 转回机器人坐标，得到它对应的球面方向。
            self.gl_wrt_rbt = self.tf_2d_inv @ self.gl_wrt_odom

            # predict occupancy in direction of final goal using GP occupancy model
            self.gl_th = np.arctan2(self.gl_wrt_rbt[1], self.gl_wrt_rbt[0])
            self.gl_al = np.pi / 2
            gl_oc, gl_var = self.gp_nav.model.predict_f(
                np.array([self.gl_th, self.gl_al], dtype="float32").reshape(1, 2))
            gl_rds = self.oc_srfc_rds - gl_oc.numpy()
            # print("gl_var, var_thrshld: ", gl_var.numpy(), self.gp_nav_var_thrshld)
            # print("gl_dst_err, gl_rds: ", self.gl_dst_err, gl_rds)

            # 若预测的自由距离足以覆盖目标距离，旧接口认为可直接朝目标走。
            if (self.gl_dst_err < gl_rds):
                if not self.goal_published:
                    self.glbl_gl_pbl()  ## not sure if this correct situation
                    self.goal_published = False
                    ##print("Navigation Mode: FinalGoal ")
                else:
                    quit()

            # 否则旧接口发布推荐子目标和其余兴趣点；若拓扑标志禁止旧接口，
            # 则清空其可视化点并进入下一轮。
            else:
                #zcy
                self.publish_closed_targets()
                #zcy如果标志位为1，则启用topo，停止发送gpsubgoal
                if not self.change_flag:
                    print("false,change topo")
                    self.clear_points()
                    continue
                ##print("Navigation Mode: SubGoal ")
                self.gp_nav_xypts_actul_pcl()
                self.gp_nav_pts_pcl()
                self.gp_nav_glbl_gl_pbl()
                self.gp_interest_subgoal_publish()



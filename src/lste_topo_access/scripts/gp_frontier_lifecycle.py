"""One focused responsibility extracted from the legacy GP frontier node."""

import json
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from gp_frontier_common import wrap_angle

class GpFrontierLifecycleMixin:
    def dump_access_topo(self, force: bool = False):
        """周期性保存 topo tree 为 JSON，便于离线可视化和实验复盘。

        写入同一个 run_id 文件而不是每周期新建文件；下次重新启动节点才会
        生成新的 run_id 和新文件。该 JSON 是运行产物，不是启动配置。
        """
        if not self.topo_save_dir:
            return
        now = rospy.Time.now().to_sec()
        if (not force) and (now - self.last_topo_save_time < self.topo_save_period):
            return
        self.last_topo_save_time = now
        try:
            os.makedirs(self.topo_save_dir, exist_ok=True)
            data = {
                "stamp": now,
                "run_id": self.run_id,
                "run_start": self.run_start,
                "run_end": self.run_end,
                "test_name": self.test_name,
                "topo_tree_root": self.topo_tree_root,
                "pose": {"x": getattr(self.pose, "x", None), "y": getattr(self.pose, "y", None)},
                "access_mode": self.access_mode,
                "forced_heading_world": self.forced_heading_world,
                "forced_branch": self.forced_branch,
                "current_backtrack": self.current_backtrack,
                "backtrack_start_pose": self.backtrack_start_pose,
                "backtrack_path": self.backtrack_path,
                "backtrack_history": self.backtrack_history,
                "return_home_target": self.return_home_target,
                "current_profile": self.current_profile,
                "nodes": self.anchor_nodes,
                "pending_junctions": self.pending_junctions,
                "backtrack_stack": self.backtrack_stack,
                "state_events": self.state_events,
                "profile_events": self.profile_events,
                "goal_events": self.goal_events,
                "visited_events": self.visited_events,
            }
            with open(self.topo_save_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "dump_access_topo failed: %s", exc)

    def frontier_log(self, msg: str):
        """Append frontier-related log to file when enabled."""
        if not getattr(self, "frontier_log_enabled", False):
            return
        if not getattr(self, "frontier_log_file", None):
            return
        try:
            t = rospy.Time.now().to_sec() if rospy.rostime.is_initialized() else 0.0
            if getattr(self, "frontier_log_fh", None) is not None:
                self.frontier_log_fh.write(f"[{t:.3f}] {msg}\n")
            else:
                with open(self.frontier_log_file, "a", encoding="utf-8") as f:
                    f.write(f"[{t:.3f}] {msg}\n")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "gp_subgoal: frontier_log write failed: %s", exc)
            self.frontier_log_enabled = False

    def on_shutdown(self):
        """节点退出时强制落盘，确保单个 JSON 包含最终状态。"""
        self.run_end = rospy.Time.now().to_sec()
        self.dump_access_topo(force=True)
        # 关闭 frontier log，并恢复 stdout/stderr
        try:
            if self._stdout_orig is not None:
                sys.stdout = self._stdout_orig
            if self._stderr_orig is not None:
                sys.stderr = self._stderr_orig
        except Exception:
            pass
        try:
            if self.frontier_log_fh:
                self.frontier_log_fh.flush()
                self.frontier_log_fh.close()
        except Exception:
            pass

    #zcy
    # 切换模式，true为高斯开启发布，flase为topo，高斯subgoal停止发布
    def flag_cb(self, bool_msg):
        self.change_flag = bool_msg.data


    def zcypose_cb(self, rbt_pose_msg):
        self.pose = rbt_pose_msg
        # 初始化全局方向骨架：记录起始位姿，定义 4 个固定世界方向
        if not self.origin_set:
            self.origin_x = self.pose.x
            self.origin_y = self.pose.y
            self.origin_yaw = wrap_angle(self.pose.theta)
            self.headings = [
                self.origin_yaw,
                wrap_angle(self.origin_yaw + np.pi * 0.5),
                wrap_angle(self.origin_yaw + np.pi),
                wrap_angle(self.origin_yaw + np.pi * 1.5),
            ]
            self.dir_idx = 0
            self.last_switch_xy = (self.origin_x, self.origin_y)
            self.origin_set = True

        int_position = [int(self.pose.x), int(self.pose.y)]
        real_position = [self.pose.x, self.pose.y]
        if int_position not in self.int_visited_positions:
            self.int_visited_positions.append(int_position)
            self.visited_positions.append(real_position)
            print(real_position)
        self.ensure_anchor()

    def is_visited(self, point, threshold):
        for traj_pt in self.visited_positions:
            distance = np.sqrt((point[0] - traj_pt[0]) ** 2 + (point[1] - traj_pt[1]) ** 2)
            if distance < threshold:
                return True
        return False

    def publish_closed_targets(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "closed_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # 设置点的大小（直径）
        marker.scale.x = 0.2  # 根据需要调整
        marker.scale.y = 0.2  # 根据需要调整

        # 设置颜色为红色
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 透明度

        # 添加点到Marker消息中
        for point in self.visited_positions:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # 2D
            marker.points.append(p)

        # 发布Marker消息
        self.closed_marker_pub.publish(marker)

    """ @brief: to print all parameters"""

    def print_ros_param(self):
        print("######## OC SRFC ######")
        print("oc_srfc_rds                : ", self.oc_srfc_rds)
        print("######## Nav ########")
        print("gp_nav_indpts_sz    : ", self.gp_nav_indpts_sz)
        print("gap_k_dir        : ", self.gap_k_dir)
        print("gap_k_dst       : ", self.gap_k_dst)
        print("gp_nav_var_thrshld  : ", self.gp_nav_var_thrshld)
        print("gp_nav_goal_dst     : ", self.gp_nav_goal_dst)
        print("gp_nav_var_img_viz  : ", self.gp_nav_var_img_viz)
        print("gl_x         : ", self.gl_x)
        print("gl_y         : ", self.gl_y)
        print("##########################################")

    """ @brief: to initiate and train 2D SGP using the SGP2D class"""


"""One focused responsibility extracted from the legacy GP frontier node."""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from lste_msgs.msg import LsteFrontiers
from scipy.spatial import distance
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_from_euler

from gp_frontier_common import wrap_angle

class GpFrontierPublicationMixin:
    def publish_frontiers(self):
        """发布全量 frontier（theta_rel + area）供 GoalManager 聚合使用。

        不在这里决定车辆最终走哪条路：Goal Manager 会再根据 LSTE 状态、
        运动先验、激光安全距离和回退覆盖规则选出唯一 final_goal。
        """
        if self.gp_nav_frntr_cntrs is None:
            return
        try:
            thetas = np.array(self.gp_nav_frntr_cntrs).reshape(-1, 2)[:, 0]
        except Exception:
            return
        if thetas.size == 0:
            return
        areas_raw = getattr(self, "gp_nav_frntr_areas", None)
        if areas_raw is None:
            areas = np.ones_like(thetas, dtype=float)
        else:
            areas_raw = np.array(areas_raw).reshape(-1)
            if areas_raw.size != thetas.size:
                areas = np.ones_like(thetas, dtype=float)
                m = min(len(areas_raw), len(areas))
                areas[:m] = areas_raw[:m]
            else:
                areas = areas_raw

        msg = LsteFrontiers()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
        msg.theta_rel = [float(t) for t in thetas]
        msg.area = [float(a) for a in areas]
        self.frontiers_pub.publish(msg)

    def update_global_goal_periodic(self):
        """
        基于 4-方向锚定 + 分叉 commit 的内部全局目标更新。
        这里维护的是 GP 的评分参考目标，不等于 Goal Manager 对外发布的
        /lste/final_goal。

        - commit_active=True: 沿 commit_heading 前推固定距离 D。
        - 否则用 frontier 按四个骨架方向聚合投票，满足门槛后切换。
        """
        if self.pose is None or not self.origin_set:
            return
        now = rospy.Time.now().to_sec()

        # 非 commit 状态下：按照 gl_update_period 节流更新频率
        # （commit 阶段仍然每帧更新，方向固定，只是沿既定方向前推）
        if (not self.commit_active) and (now - self.last_gl_update_time < self.gl_update_period):
            return

        # 1) 分叉检测 + commit 触发
        if (not self.commit_active) and self.gp_nav_frntr_cntrs is not None and len(self.gp_nav_frntr_cntrs) > 0:
            thetas = np.array(self.gp_nav_frntr_cntrs).reshape(-1, 2)[:, 0]
            areas = np.array(getattr(self, "gp_nav_frntr_areas", []))
            if len(areas) != len(thetas):
                areas = np.ones_like(thetas)
            thr_angle = np.deg2rad(25.0)
            sep_angle = np.deg2rad(60.0)
            left_mask = thetas < -thr_angle
            right_mask = thetas > thr_angle
            if np.any(left_mask) and np.any(right_mask):
                theta_L = thetas[left_mask]
                theta_R = thetas[right_mask]
                area_L = areas[left_mask]
                area_R = areas[right_mask]
                idx_L = int(np.argmax(area_L))
                idx_R = int(np.argmax(area_R))
                best_theta_L = theta_L[idx_L]
                best_theta_R = theta_R[idx_R]
                best_area_L = area_L[idx_L]
                best_area_R = area_R[idx_R]
                if abs(best_theta_R - best_theta_L) > sep_angle and abs(best_theta_L) > thr_angle and abs(best_theta_R) > thr_angle:
                    if max(best_area_L, best_area_R) >= 1.25 * min(best_area_L, best_area_R):
                        theta_best = best_theta_L if best_area_L >= best_area_R else best_theta_R
                        # 将 commit 方向也投射到 4 个固定世界方向，保证全局目标始终落在骨架射线上
                        heading_world = wrap_angle(self.pose.theta + theta_best)
                        diffs_commit = [abs(wrap_angle(heading_world - h)) for h in self.headings]
                        k_commit = int(np.argmin(diffs_commit))
                        self.commit_active = True
                        self.commit_heading = self.headings[k_commit]
                        self.commit_start_xy = (self.pose.x, self.pose.y)
                        rospy.loginfo_throttle(5.0, "Commit to heading %.2f rad (dir %d)", self.commit_heading, k_commit)

        # 2) commit 模式：沿 commit_heading 前推
        D = 6.0
        if self.commit_active:
            dx = self.pose.x - self.commit_start_xy[0]
            dy = self.pose.y - self.commit_start_xy[1]
            progress = dx * np.cos(self.commit_heading) + dy * np.sin(self.commit_heading)
            if progress >= 3.0:
                self.commit_active = False
            else:
                self.gl_x = self.pose.x + D * np.cos(self.commit_heading)
                self.gl_y = self.pose.y + D * np.sin(self.commit_heading)
                self.gl_yaw = self.commit_heading
                self.gl_wrt_odom = np.array([self.gl_x, self.gl_y, 1], dtype="float32")
                self.last_switch_xy = (self.pose.x, self.pose.y)
                self.last_gl_update_time = now
                return

        # 3) 非 commit：frontier → 骨架方向选择
        if self.gp_nav_frntr_cntrs is None or len(self.gp_nav_frntr_cntrs) == 0:
            return
        areas = np.array(getattr(self, "gp_nav_frntr_areas", []))
        if areas.size == 0:
            areas = np.ones(len(self.gp_nav_frntr_cntrs))

        # 对外发布“最大 frontier 方向”供 Goal Manager 使用（相对机器人角度）
        try:
            best_idx = int(np.argmax(areas))
            if best_idx >= 0 and best_idx < len(self.gp_nav_frntr_cntrs):
                theta_best = float(self.gp_nav_frntr_cntrs[best_idx][0])
                area_best = float(areas[best_idx]) if best_idx < len(areas) else 0.0
                msg = Vector3Stamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_footprint"
                msg.vector.x = theta_best  # rad，相对机器人
                msg.vector.y = area_best
                msg.vector.z = 0.0
                self.frontier_dir_pub.publish(msg)
        except Exception:
            pass

        # 将所有 frontier 按最近的骨架方向聚合（只接受在扇区范围内的）
        sector_half = np.deg2rad(30.0)  # 扇区半宽，控制哪些 frontiers 参与投票
        sum_area = np.zeros(4, dtype=float)
        max_area = np.zeros(4, dtype=float)
        heading_weighted = np.zeros(4, dtype=float)

        for i, f in enumerate(self.gp_nav_frntr_cntrs):
            theta_rel = float(f[0])  # 相对机器人角
            heading_world = wrap_angle(self.pose.theta + theta_rel)
            diffs = [abs(wrap_angle(heading_world - h)) for h in self.headings]
            k = int(np.argmin(diffs))
            if diffs[k] > sector_half:
                continue  # 超出扇区范围的 frontiers 不参与该方向投票
            a = float(areas[i]) if i < len(areas) else 1.0
            sum_area[k] += a
            heading_weighted[k] += a * heading_world
            if a > max_area[k]:
                max_area[k] = a

        # 选票最多的骨架方向
        k_star = int(np.argmax(sum_area))
        if sum_area[k_star] <= 0:
            k_star = self.dir_idx  # 没有有效投票则保持当前方向
        # 该方向的加权平均朝向，用于一致性判断
        if sum_area[k_star] > 0:
            heading_f = wrap_angle(heading_weighted[k_star] / sum_area[k_star])
        else:
            heading_f = self.headings[k_star]

        # 门槛：角度一致性 + 行进距离
        diff_heading = abs(wrap_angle(heading_f - self.headings[k_star]))
        allow_switch = diff_heading < np.deg2rad(25.0)
        if self.last_switch_xy is None:
            traveled = 0.0
        else:
            dx = self.pose.x - self.last_switch_xy[0]
            dy = self.pose.y - self.last_switch_xy[1]
            traveled = np.hypot(dx, dy)
        if allow_switch and traveled >= 2.0 and k_star != self.dir_idx:
            self.dir_idx = k_star
            self.last_switch_xy = (self.pose.x, self.pose.y)

        heading = self.headings[self.dir_idx]
        self.gl_x = self.pose.x + D * np.cos(heading)
        self.gl_y = self.pose.y + D * np.sin(heading)
        self.gl_yaw = heading
        self.gl_wrt_odom = np.array([self.gl_x, self.gl_y, 1], dtype="float32")
        self.last_gl_update_time = now

    """ @brief:  publish recommended goal to DRL"""

    def gp_nav_glbl_gl_pbl(self):
        if self.gp_nav_actul_xy_gls is None or np.size(self.gp_nav_actul_xy_gls) == 0:
            return
        n = int(np.shape(self.gp_nav_actul_xy_gls)[0])
        if n <= 0:
            return
        if getattr(self, "chsn_gl_idx", None) is None:
            return
        if int(self.chsn_gl_idx) < 0 or int(self.chsn_gl_idx) >= n:
            return
        nav_xy_gl = self.gp_nav_actul_xy_gls[self.chsn_gl_idx].reshape(3, -1)
        yaw = np.arctan2(nav_xy_gl[1], nav_xy_gl[0])
        qtrn = quaternion_from_euler(0, 0, yaw)
        gl_msg = PoseStamped()
        gl_msg.pose.position.x = nav_xy_gl[0]
        gl_msg.pose.position.y = nav_xy_gl[1]
        gl_msg.pose.position.z = 0
        gl_msg.pose.orientation.x = qtrn[0]
        gl_msg.pose.orientation.y = qtrn[1]
        gl_msg.pose.orientation.z = qtrn[2]
        gl_msg.pose.orientation.w = qtrn[3]

        self.header.frame_id = "odom"
        gl_msg.header = self.header
        self.gp_rcmndd_subgl_pub.publish(gl_msg)
        print("recomended subgoal in odom frame: ", nav_xy_gl[0], nav_xy_gl[1])

        rbt_gl_msg = PoseStamped()
        rbt_gl_msg.pose.position.x = np.cos(self.pose.theta) * (gl_msg.pose.position.x - self.pose.x) + \
                                     np.sin(self.pose.theta) * (gl_msg.pose.position.y - self.pose.y)
        rbt_gl_msg.pose.position.y = - np.sin(self.pose.theta) * (gl_msg.pose.position.x - self.pose.x) + \
                                     np.cos(self.pose.theta) * (gl_msg.pose.position.y - self.pose.y)
        self.header.frame_id = "os_sensor"
        rbt_gl_msg.header = self.header
        self.gp_subgl_pub.publish(rbt_gl_msg)
        print("recomended subgoal in os_sensor frame: ", rbt_gl_msg.pose.position.x, rbt_gl_msg.pose.position.y)
        gp_nav_pcl = np.column_stack((rbt_gl_msg.pose.position.x, rbt_gl_msg.pose.position.y, 0, 1))
        self.header.frame_id = self.gp_nav_frame_id
        pc2 = point_cloud2.create_cloud(self.header, self.fields, gp_nav_pcl)
        self.gp_subgl_rviz.publish(pc2)
        # 同步发布到 LSTE 接口，供 PASS 态作为 frontier 使用
        self.lste_gp_frontier_pub.publish(gl_msg)

    #zcy
    def gp_interest_subgoal_publish(self):
        interest_count = self.gp_nav_gls_sz - 1
        if interest_count <= 0:
            rospy.loginfo("No interest subgoals to publish.")
            return

        # 获取剩余的候选点（排除已选择的子目标点）
        interest_points = self.gp_nav_actul_xy_gls.copy()
        interest_points = np.delete(interest_points, self.chsn_gl_idx, axis=0)

        # 临时列表用于存储当前帧检测到的点
        current_frame_points = []

        # 遍历剩余的候选点，逐个匹配并更新检测历史
        for idx, point in enumerate(interest_points):
            x, y, _ = point
            current_frame_points.append((x, y))
            matched = False
            for tracked_point in self.detected_interest_points:
                dist = distance.euclidean((x, y), (tracked_point['x'], tracked_point['y']))
                if dist < self.match_threshold:
                    tracked_point['count'] += 1
                    tracked_point['x'] = x  # 更新点的位置（可选）
                    tracked_point['y'] = y
                    matched = True
                    break
            if not matched:
                # 新检测到的点，初始化检测次数为1
                self.detected_interest_points.append({'x': x, 'y': y, 'count': 1})

        # 清理未在当前帧中检测到的点
        # 创建一个新列表，只保留当前帧检测到的点
        new_detected_interest_points = []
        for tracked_point in self.detected_interest_points:
            # 检查该点是否在当前帧的检测点中
            in_current_frame = False
            for (x, y) in current_frame_points:
                dist = distance.euclidean((tracked_point['x'], tracked_point['y']), (x, y))
                if dist < self.match_threshold:
                    in_current_frame = True
                    break
            if in_current_frame:
                new_detected_interest_points.append(tracked_point)
            else:

                pass
        self.detected_interest_points = new_detected_interest_points

        # 遍历检测历史，发布满足条件的点，连续出现3次以上才认为有效
        for tracked_point in self.detected_interest_points:
            if tracked_point['count'] >= 3:
                interest_gl_msg = PoseStamped()
                interest_gl_msg.pose.position.x = tracked_point['x']
                interest_gl_msg.pose.position.y = tracked_point['y']

                interest_gl_msg.header.frame_id = "odom"
                interest_gl_msg.header.stamp = rospy.Time.now()

                self.gp_interest_subgl_pub.publish(interest_gl_msg)
                print(f"Published filtered interest subgoal: x={tracked_point['x']}, y={tracked_point['y']}")


    """ @brief:  publish final goal"""


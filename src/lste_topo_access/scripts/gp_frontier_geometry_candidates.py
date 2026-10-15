"""One focused responsibility extracted from the legacy GP frontier node."""

import numpy as np
import rospy

from gp_frontier_common import angle_diff, wrap_angle

class GpFrontierGeometryAndCandidatesMixin:
    def tf_rbt_2_odom(self):
        if self.pose is None:
            self.tf_2d = None
            return
        self.tf_2d = np.array(
            [[np.cos(self.pose.theta), -np.sin(self.pose.theta), self.pose.x],
             [np.sin(self.pose.theta),
              np.cos(self.pose.theta), self.pose.y], [0, 0, 1]])

    """ @brief:  odom to robot 2D TF matrix"""

    def tf_odom_2_rbt(self):
        if self.pose is None:
            self.tf_2d_inv = None
            return
        cos_th = np.cos(self.pose.theta)
        sin_th = np.sin(self.pose.theta)
        self.tf_2d_inv = np.array(
            [[cos_th, sin_th, -self.pose.x * cos_th - self.pose.y * sin_th],
             [-sin_th, cos_th, self.pose.x * sin_th - self.pose.y * cos_th],
             [0, 0, 1]])

    """ @brief:  robot to odom 2D TF matrix"""

    def gls_in_odom_frame(self, gls):
        xy_gls = np.empty([0, 3])
        for gl in gls:
            xy_gls = np.vstack((xy_gls, self.tf_2d @ gl))
        # limit to map  border,  here consider  w=h (this part is not important)
        # xy_gls = np.clip(xy_gls, -self.map_2d_h / 2 + 1, self.map_2d_h / 2 - 1) 
        return xy_gls

    """ @brief:  calculate xy location in odom frame for subgoals (navigation points)"""

    def xy_actul_nav_gls(self):
        actul_gls_rds = self.gp_nav_goal_dst * np.ones(
            self.gp_nav_gls_sz, dtype='float32').reshape(-1, 1)
        x, y, z = self.convert_spherical_2_cartesian(
            self.gp_nav_pts.T[0].reshape(-1, 1),
            self.gp_nav_pts.T[1].reshape(-1, 1), actul_gls_rds)
        z = np.ones(self.gp_nav_gls_sz).reshape(-1, 1)
        actul_gls = np.hstack((x, y, z))
        self.gp_nav_actul_xy_gls = self.gls_in_odom_frame(actul_gls)

    """ @brief:  calculate the recommended subgoal (navigation point) based on cost function"""

    def gp_nav_pkup_nav_pt(self):
        """将 frontier 变为机器人周围的候选点，并选出旧接口的推荐点。

        当前 LSTE 主链会把“全部 frontier”交给 Goal Manager 统一选择；本函数
        仍会根据距内部全局目标的距离、偏离正前方的角度和拓扑强制窗口选出
        ``chsn_gl_idx``，以兼容旧的 gp_subgoal/gp_subgoal_os 发布接口。
        """
        self.gp_nav_pts = self.gp_nav_frntr_cntrs
        # print(f"gp_nav_frntr_cntrs={self.gp_nav_frntr_cntrs}")
        self.gp_nav_gls_sz = np.shape(self.gp_nav_pts)[0]
        # print("# goals: ", self.gp_nav_gls_sz)
        self.xy_actul_nav_gls()

        #zcy
        new_gp_nav_actul_xy_gls = []
        new_gp_nav_frntr_cntrs = []
        new_gp_nav_frntr_areas = []
        closed = []  # 用于存储阻塞的目标点

        for idx, subgoal in enumerate(self.gp_nav_actul_xy_gls):
            x, y, _ = subgoal
            theta_rel = None
            try:
                theta_rel = float(self.gp_nav_frntr_cntrs[idx][0])
            except Exception:
                pass
            in_forced_window = False
            if self.forced_heading_world is not None and theta_rel is not None:
                heading_world = wrap_angle(self.pose.theta + theta_rel)
                if angle_diff(heading_world, self.forced_heading_world) <= self.force_window_rad:
                    in_forced_window = True
                else:
                    continue  # 强制窗口下，非兴趣方向直接丢弃
            # 可行距离过滤：用 GP 预测该方向的可行半径（oc_srfc_rds - oc），过近则视为不可走
            clearance_ok = True
            try:
                if getattr(self, "enable_min_frontier_clearance", False) and self.min_frontier_clearance > 0.0 and self.gp_nav is not None:
                    oc_pred, _ = self.gp_nav.model.predict_f(
                        np.array([theta_rel, np.pi / 2], dtype="float32").reshape(1, 2)
                    )
                    rds = float(self.oc_srfc_rds - oc_pred.numpy())
                    if rds < float(self.min_frontier_clearance):
                        clearance_ok = False
            except Exception:
                clearance_ok = True
            if not clearance_ok:
                continue
            if (not in_forced_window) and self.is_visited([x, y], self.visited_thresh):
                closed_position = [int(x), int(y)]
                if closed_position not in self.closed:
                    closed.append(closed_position)
                continue
            new_gp_nav_actul_xy_gls.append(subgoal)
            new_gp_nav_frntr_cntrs.append(self.gp_nav_frntr_cntrs[idx])
            if idx < len(self.gp_nav_frntr_areas):
                new_gp_nav_frntr_areas.append(self.gp_nav_frntr_areas[idx])

        #对齐形状
        self.gp_nav_actul_xy_gls = np.array(new_gp_nav_actul_xy_gls)
        self.gp_nav_frntr_cntrs = np.array(new_gp_nav_frntr_cntrs)
        self.gp_nav_frntr_areas = np.array(new_gp_nav_frntr_areas)
        self.gp_nav_pts = self.gp_nav_frntr_cntrs
        self.gp_nav_gls_sz = len(new_gp_nav_actul_xy_gls)

        # 全部的候选目标点
        print(f"gp_nav_actul_xy_gls={self.gp_nav_actul_xy_gls}")
        self.frontier_log(f"candidates={self.gp_nav_actul_xy_gls.tolist()} areas={self.gp_nav_frntr_areas.tolist() if hasattr(self, 'gp_nav_frntr_areas') else []}")

        # 回到起点途中：方案 A——不再因新 frontier 打断，全部忽略
        if self.return_home_target is not None and self.gp_nav_gls_sz > 0:
            # 直接丢弃 frontier，继续回家
            self.gp_nav_actul_xy_gls = np.array([])
            self.gp_nav_frntr_cntrs = np.array([])
            self.gp_nav_frntr_areas = np.array([])
            self.gp_nav_pts = self.gp_nav_frntr_cntrs
            self.gp_nav_gls_sz = 0
            return

        if len(self.gp_nav_actul_xy_gls) == 0:
            now = rospy.Time.now().to_sec()
            # mode2 免回退窗口：给强制阶段至少 5 秒探索 frontier
            if self.access_mode == 2 and self.forced_heading_world is not None:
                if self.mode2_start_time is not None and (now - self.mode2_start_time) < 5.0:
                    return
            if self.no_frontier_since is None:
                self.no_frontier_since = now
                if self.pose is not None:
                    self.no_frontier_start_pose = {"x": self.pose.x, "y": self.pose.y}
                self.no_frontier_start_anchor = self.ensure_anchor(force=False)
            # 无 frontier 连续 >=T 触发回退
            if self.no_frontier_since is not None:
                if (now - self.no_frontier_since) >= self.no_frontier_trigger_sec:
                    if self.no_frontier_start_pose is not None and self.backtrack_start_pose is None:
                        self.backtrack_start_pose = dict(self.no_frontier_start_pose)
                    if self.backtrack_start_anchor is None:
                        self.backtrack_start_anchor = self.no_frontier_start_anchor
                    if not self.backtrack_recording:
                        self.backtrack_path = []
                        if self.backtrack_start_anchor is not None:
                            self.backtrack_path.append(self.backtrack_start_anchor)
                        self.backtrack_recording = True
                    self.enter_backtrack()
            return
        # 恢复有 frontier，清除计时
        self.no_frontier_since = None
        self.no_frontier_start_pose = None
        self.no_frontier_start_anchor = None
        if not self.backtrack_recording:
            # 事件未触发回溯，清空起点信息
            self.backtrack_start_pose = None
            self.backtrack_start_anchor = None
            self.backtrack_path = []

        # Access commit 阶段：只保留“每个 PENDING 分支方向最近的一个 frontier”
        if self.access_mode == 2 and self.pose is not None:
            pend_headings = []
            if self.forced_branch is not None:
                node = self._get_node(self.forced_branch[0])
                if node:
                    for br in node.get("branches", []):
                        if br.get("status") == "PENDING" and br.get("heading_world") is not None:
                            pend_headings.append(float(br["heading_world"]))
            if pend_headings and self.gp_nav_frntr_cntrs is not None and self.gp_nav_frntr_cntrs.size > 0:
                try:
                    thetas_rel = np.array(self.gp_nav_frntr_cntrs).reshape(-1, 2)[:, 0]
                except Exception:
                    thetas_rel = np.array([])
                if thetas_rel.size > 0:
                    headings_world = [wrap_angle(float(self.pose.theta) + float(th)) for th in thetas_rel]
                    keep_idx = set()
                    for ph in pend_headings:
                        best = None
                        for idx, hw in enumerate(headings_world):
                            diff = angle_diff(hw, ph)
                            if best is None or diff < best[0]:
                                best = (diff, idx)
                        if best is not None:
                            keep_idx.add(best[1])
                    if keep_idx:
                        keep_idx = sorted(list(keep_idx))
                        self.gp_nav_actul_xy_gls = self.gp_nav_actul_xy_gls[keep_idx]
                        self.gp_nav_frntr_cntrs = self.gp_nav_frntr_cntrs[keep_idx]
                        try:
                            if len(self.gp_nav_frntr_areas) >= len(self.gp_nav_frntr_cntrs):
                                self.gp_nav_frntr_areas = self.gp_nav_frntr_areas[keep_idx]
                        except Exception:
                            self.gp_nav_frntr_areas = np.array([])
                        self.gp_nav_pts = self.gp_nav_frntr_cntrs
                        self.gp_nav_gls_sz = len(keep_idx)

        gap_to_gl_dst = np.sqrt((self.gl_y -
                                 self.gp_nav_actul_xy_gls.T[1]) ** 2 +
                                (self.gl_x - self.gp_nav_actul_xy_gls.T[0]) ** 2)

        self.gap_utlty_fun = self.gap_k_dst * (
                self.gp_nav_goal_dst + gap_to_gl_dst) + self.gap_k_dir * abs(
            self.gp_nav_frntr_cntrs.T[0]) ** 2
        print("gap_dir: ", abs(self.gp_nav_frntr_cntrs.T[0]))
        print("gap_to_gl_dst : ", gap_to_gl_dst)
        print("self.gap_utlty_fun: ", self.gap_utlty_fun)

        self.chsn_gl_idx = self.gap_utlty_fun.argmin(axis=0)
        self.gp_nav_pt = self.gp_nav_pts[self.chsn_gl_idx]
        print("recommended subgoal id and direction: ", self.chsn_gl_idx, self.gp_nav_pt[0])
        try:
            chosen_theta_rel = float(self.gp_nav_pt[0])
        except Exception:
            chosen_theta_rel = None
        if chosen_theta_rel is not None:
            self.create_interest_from_frontiers(chosen_theta_rel)
        self.closed.extend(closed)


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility goals from legacy local frontier and context observations."""

import math
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from lste_msgs.msg import LsteDetection
from goal_manager_modes import CATCH_TARGET_MODE, EXPLORE_SUS_C_MODE
from goal_manager_target_utils import wrap_angle

class GoalManagerLegacyGoalsMixin:
    def goal_from_ctx_follow(self, now: float) -> Optional[PoseStamped]:
        if self.ctx_start_time is None:
            self.ctx_start_time = now

        # ctx 期间检测到 target：直接切到 target 模式
        if self.pick_best_target() is not None:
            self.effective_mode = CATCH_TARGET_MODE
            self.pub_access_mode.publish(String(data=self.effective_mode))
            self.ctx_start_time = None
            return self.goal_from_target_follow(now)

        goal = None
        left_det, right_det = self.pick_ctx_pair()
        if left_det is not None and right_det is not None:
            # 用两框中心的中点方向
            mid_det = LsteDetection()
            mid_det.cx = 0.5 * (float(left_det.cx) + float(right_det.cx))
            mid_det.cy = 0.5 * (float(left_det.cy) + float(right_det.cy))
            mid_det.w = mid_det.h = 0.0
            detection_stamp = self.latest_dets.header.stamp
            heading_world = self.det_heading_world(
                mid_det,
                detection_stamp
                if detection_stamp and detection_stamp.to_sec() > 0.0
                else rospy.Time(0),
            )
            if heading_world is not None:
                dist = self.clip_distance(heading_world, self.follow_context_step_distance)
                if dist is not None:
                    x = self.latest_pose.x + dist * math.cos(heading_world)
                    y = self.latest_pose.y + dist * math.sin(heading_world)
                    old_distance = float("inf")
                    old_heading = None
                    if self.ctx_last_goal is not None and self.latest_pose is not None:
                        old_distance = math.hypot(
                            self.ctx_last_goal.pose.position.x - self.latest_pose.x,
                            self.ctx_last_goal.pose.position.y - self.latest_pose.y,
                        )
                        old_heading = self.yaw_from_pose(self.ctx_last_goal)
                    heading_delta = (
                        abs(wrap_angle(heading_world - old_heading))
                        if old_heading is not None else float("inf")
                    )
                    # Keep a context inspection segment stable while the robot
                    # is approaching it.  Replacing it on every detector/timer
                    # tick makes the target move with the robot and produces
                    # the same steering chase that previously affected TEB.
                    can_refresh = (
                        self.ctx_last_goal is None
                        or old_distance <= self.ctx_goal_reached_radius
                        or heading_delta >= self.ctx_goal_heading_update_threshold
                    )
                    if can_refresh and (now - self.ctx_last_update) >= self.follow_min_update_period:
                        if self.follow_heading_alpha and old_heading is not None:
                            heading_world = self._slerp_yaw(
                                old_heading, heading_world, self.follow_heading_alpha
                            )
                            x = self.latest_pose.x + dist * math.cos(heading_world)
                            y = self.latest_pose.y + dist * math.sin(heading_world)
                        goal = self.make_goal_pose((x, y, 0.0), heading_world)
                        self.goal_source = "ctx_pair_follow"
                        self.ctx_last_goal = goal
                        self.ctx_last_update = now
                        rospy.loginfo(
                            "GoalManager: context segment refreshed old_dist=%.2f "
                            "heading_delta=%.1fdeg goal=(%.2f,%.2f)",
                            old_distance,
                            math.degrees(heading_delta) if math.isfinite(heading_delta) else float("inf"),
                            x,
                            y,
                        )
                    elif self.ctx_last_goal is not None:
                        goal = self.ctx_last_goal
                        self.goal_source = "ctx_pair_cached"

        # ctx 窗口过期：回到探索
        if (now - self.ctx_start_time) >= self.follow_ctx_lost_timeout:
            if goal is None:
                goal = self.ctx_last_goal
            rospy.loginfo_throttle(2.0, "GoalManager: ctx window %.1fs expired, switch to explore_sus_c_mode",
                                   self.follow_ctx_lost_timeout)
            self.effective_mode = EXPLORE_SUS_C_MODE
            self.pub_access_mode.publish(String(data=self.effective_mode))
            self.ctx_start_time = None
            self.ctx_cooldown_until = now + self.context_follow_cooldown
            rospy.loginfo(
                "GoalManager: context observation complete; global coverage cooldown %.1fs",
                self.context_follow_cooldown,
            )
            return goal if goal is not None else self.goal_from_frontiers_prior()

        if goal is None and self.ctx_last_goal is not None:
            self.goal_source = "ctx_cached"
            return self.ctx_last_goal
        if goal is None:
            return self.goal_from_frontiers_prior()
        return goal

    def goal_from_frontier(self, period_mode: str) -> Optional[PoseStamped]:
        if self.latest_frontier is None:
            rospy.logwarn_throttle(5.0, "GoalManager: frontier info not available")
            return None
        theta_rel = float(self.latest_frontier.vector.x)
        heading_world = float(self.latest_pose.theta) + theta_rel
        dist = self.clip_distance(heading_world, self.forward_dist)
        if dist is None:
            return None
        x = self.latest_pose.x + dist * math.cos(heading_world)
        y = self.latest_pose.y + dist * math.sin(heading_world)
        return self.make_goal_pose((x, y, 0.0), heading_world)

    def goal_from_frontiers_prior(self) -> Optional[PoseStamped]:
        """PASS/Sus-C：使用全量 frontier + 运动先验投影到 4 固定方向后选取目标。"""
        if self.headings is None:
            if self.start_pose:
                self.init_headings(self.start_pose)
            elif self.latest_pose:
                self.init_headings(self.latest_pose)
        if self.headings is None:
            rospy.logwarn_throttle(5.0, "GoalManager: headings not initialized")
            return None
        if self.latest_pose is None:
            return None
        msg = self.latest_frontiers
        if msg is None or len(msg.theta_rel) == 0:
            rospy.logwarn_throttle(5.0, "GoalManager: gp_frontiers not available")
            return None

        thetas_rel = list(msg.theta_rel)
        areas_raw = list(msg.area) if msg.area else [1.0] * len(thetas_rel)
        if len(areas_raw) < len(thetas_rel):
            areas_raw += [1.0] * (len(thetas_rel) - len(areas_raw))
        areas = [max(0.0, float(a)) for a in areas_raw[: len(thetas_rel)]]

        total_area = sum(areas)
        if total_area <= 0:
            return None
        # 运动方向（世界系）
        dir_move = None
        prior_strength = 0.0
        if self.latest_cmd_vel is not None:
            vx = float(self.latest_cmd_vel.linear.x)
            speed = abs(vx)
            if speed >= self.v_min and self.latest_pose is not None:
                dir_move = wrap_angle(self.latest_pose.theta if vx >= 0 else self.latest_pose.theta + math.pi)
                if self.v_scale > 1e-3:
                    prior_strength = min(1.0, max(0.0, (speed - self.v_min) / self.v_scale))
                else:
                    prior_strength = 1.0

        scores = [0.0, 0.0, 0.0, 0.0]
        sigma = self.front_sigma if self.front_sigma > 1e-3 else 0.35
        front_thr = self.front_deg
        back_thr = math.pi - self.back_deg
        base_heading = float(self.latest_pose.theta)

        for theta_rel, area in zip(thetas_rel, areas):
            heading_world = wrap_angle(base_heading + float(theta_rel))
            a_norm = area / total_area
            weight = math.pow(a_norm, self.area_power)
            # 运动先验权重（投影前）
            if dir_move is not None:
                delta = abs(wrap_angle(heading_world - dir_move))
                if delta <= front_thr:
                    w_move = 1.0 + prior_strength * (self.w_front_max - 1.0)
                elif delta >= back_thr:
                    w_move = 1.0 - prior_strength * (1.0 - self.w_back_min)
                else:
                    w_move = 1.0
            else:
                w_move = 1.0
            vote = weight * w_move
            for idx, Hk in enumerate(self.headings):
                diff = abs(wrap_angle(heading_world - Hk))
                proj = math.exp(-0.5 * (diff / sigma) ** 2)
                scores[idx] += vote * proj

        # 前进时限制后半平面（可选）
        allow_mask = [True] * 4
        if self.forward_only and dir_move is not None and abs(float(self.latest_cmd_vel.linear.x)) > self.v_gate:
            for idx, Hk in enumerate(self.headings):
                if math.cos(wrap_angle(Hk - base_heading)) < 0:
                    allow_mask[idx] = False
            if not any(allow_mask) and any(s > self.min_allowed_score for s in scores):
                allow_mask = [True if s > self.min_allowed_score else False for s in scores]

        best_idx = None
        best_val = -1.0
        second_val = -1.0
        for idx, s in enumerate(scores):
            if not allow_mask[idx]:
                continue
            if s > best_val:
                second_val = best_val
                best_val = s
                best_idx = idx
            elif s > second_val:
                second_val = s
        if best_idx is None:
            return None

        # 滞回：优势不够则保持上次方向
        selected_idx = best_idx
        if self.last_dir_idx is not None and best_idx != self.last_dir_idx:
            margin = second_val * (1.0 + self.switch_margin)
            if best_val < margin:
                best_idx = self.last_dir_idx
                best_val = scores[best_idx]
        self.last_dir_idx = best_idx
        self.goal_source = "frontier_vote"
        self.frontier_debug = "n=%d selected=%d raw_best=%d scores=[%s]" % (
            len(thetas_rel), best_idx, selected_idx,
            ",".join("%.3f" % score for score in scores),
        )

        heading_world = self.headings[best_idx]
        dist = self.clip_distance(heading_world, self.forward_dist)
        if dist is None:
            return None
        x = self.latest_pose.x + dist * math.cos(heading_world)
        y = self.latest_pose.y + dist * math.sin(heading_world)
        return self.make_goal_pose((x, y, 0.0), heading_world)



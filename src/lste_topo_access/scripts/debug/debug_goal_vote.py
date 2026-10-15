#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试工具：实时打印 PASS/Sus-C 投票过程。
- 订阅：/rbt_pose (Pose2D), /cmd_vel (Twist), /lste/gp_frontiers (LsteFrontiers), /lste/final_goal (PoseStamped)
- 逻辑：复用 lste_goal_manager 的参数和公式，打印 4 个固定方向的得分、运动先验、frontier 列表。
"""
import math
import rospy
from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from lste_msgs.msg import LsteFrontiers


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_q(q) -> float:
    # q: geometry_msgs/Quaternion
    return math.atan2(2 * (q.w * q.z), 1 - 2 * (q.z * q.z))


class DebugGoalVote:
    def __init__(self):
        rospy.init_node("debug_goal_vote")

        # 读取与 goal_manager 一致的参数（使用相同默认值）
        gp = rospy.get_param
        self.front_sigma = math.radians(float(gp("~front_sigma_deg", 25.0)))
        self.area_power = float(gp("~area_power", 1.0))
        self.v_min = float(gp("~v_min", 0.05))
        self.v_scale = float(gp("~v_scale", 0.3))
        self.front_deg = math.radians(float(gp("~front_deg", 60.0)))
        self.back_deg = math.radians(float(gp("~back_deg", 60.0)))
        self.w_front_max = float(gp("~w_front_max", 1.5))
        self.w_back_min = float(gp("~w_back_min", 0.2))

        self.pose = None
        self.cmd = None
        self.frontiers = None
        self.goal = None

        rospy.Subscriber("/rbt_pose", Pose2D, self.cb_pose, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=1)
        rospy.Subscriber("/lste/gp_frontiers", LsteFrontiers, self.cb_frontiers, queue_size=1)
        rospy.Subscriber("/lste/final_goal", PoseStamped, self.cb_goal, queue_size=1)

        rospy.loginfo("debug_goal_vote started, waiting for topics...")
        self.loop()

    def cb_pose(self, msg: Pose2D):
        self.pose = msg

    def cb_cmd(self, msg: Twist):
        self.cmd = msg

    def cb_frontiers(self, msg: LsteFrontiers):
        self.frontiers = msg

    def cb_goal(self, msg: PoseStamped):
        self.goal = msg

    def loop(self):
        rate = rospy.Rate(2.0)  # 2 Hz
        while not rospy.is_shutdown():
            if not (self.pose and self.cmd and self.frontiers and self.goal):
                rate.sleep()
                continue
            self.print_vote()
            rate.sleep()

    def print_vote(self):
        pose = self.pose
        cmd = self.cmd
        fr = self.frontiers
        goal = self.goal

        base = wrap(float(pose.theta))
        yaw_goal = wrap(yaw_from_q(goal.pose.orientation))
        headings = [
            yaw_goal,
            wrap(yaw_goal + math.pi * 0.5),
            wrap(yaw_goal + math.pi),
            wrap(yaw_goal + math.pi * 1.5),
        ]

        thetas = list(fr.theta_rel)
        areas_raw = list(fr.area) if fr.area else [1.0] * len(thetas)
        if len(areas_raw) < len(thetas):
            areas_raw += [1.0] * (len(thetas) - len(areas_raw))
        areas = [max(0.0, float(a)) for a in areas_raw[: len(thetas)]]
        total = sum(areas) if len(areas) > 0 else 1.0

        vx = float(cmd.linear.x)
        speed = abs(vx)
        dir_move = None
        prior_strength = 0.0
        if speed >= self.v_min:
            dir_move = wrap(base if vx >= 0 else base + math.pi)
            if self.v_scale > 1e-6:
                prior_strength = min(1.0, max(0.0, (speed - self.v_min) / self.v_scale))
            else:
                prior_strength = 1.0

        sigma = self.front_sigma if self.front_sigma > 1e-6 else 0.35
        scores = [0.0, 0.0, 0.0, 0.0]
        for tr, a in zip(thetas, areas):
            hw = wrap(base + float(tr))
            a_norm = a / total if total > 1e-9 else 0.0
            weight = math.pow(a_norm, self.area_power)
            w_move = 1.0
            if dir_move is not None:
                delta = abs(wrap(hw - dir_move))
                if delta <= self.front_deg:
                    w_move = 1.0 + prior_strength * (self.w_front_max - 1.0)
                elif delta >= math.pi - self.back_deg:
                    w_move = 1.0 - prior_strength * (1.0 - self.w_back_min)
            vote = weight * w_move
            for i, H in enumerate(headings):
                diff = abs(wrap(hw - H))
                proj = math.exp(-0.5 * (diff / sigma) ** 2)
                scores[i] += vote * proj

        rospy.loginfo("\n--- debug_goal_vote ---")
        rospy.loginfo("pose.theta=%.3f  cmd_vx=%.3f  dir_move=%s  prior=%.2f",
                      base, vx, f"{dir_move:.3f}" if dir_move is not None else "None", prior_strength)
        rospy.loginfo("frontiers: %s",
                      ", ".join([f"(d={tr:+.3f},a={a:.0f})" for tr, a in zip(thetas, areas)]))
        rospy.loginfo("headings : %s",
                      ", ".join([f"{h:+.3f}" for h in headings]))
        rospy.loginfo("scores   : %s",
                      ", ".join([f"{s:.4f}" for s in scores]))
        rospy.loginfo("current final_goal yaw=%.3f", yaw_goal)


if __name__ == "__main__":
    DebugGoalVote()

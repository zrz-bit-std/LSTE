#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lste_score_node
- 订阅 /lste/task 与 /lste/detections
- 调用 score_adapter（封装 calc_s_target / calc_s_env / calc_s_ctx / aggregate_target_score）
- 发布 /lste/scores（LsteScores）
"""

from __future__ import annotations

import rospy
from std_msgs.msg import Header

from lste_msgs.msg import LsteDetections, LsteScores, LsteTask
from utils import score_adapter


class ScoreNode:
    def __init__(self):
        rospy.init_node("lste_score_node")
        self.current_task: LsteTask | None = None

        self.weights = self._load_weights()
        self.env_params = self._load_env_params()

        self.pub = rospy.Publisher("/lste/scores", LsteScores, queue_size=10)
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.sub_detections = rospy.Subscriber("/lste/detections", LsteDetections, self.on_detections, queue_size=10)

        self.allow_task_mismatch = bool(rospy.get_param("~allow_task_mismatch", False))
        rospy.loginfo("lste_score_node started.")

    def _load_weights(self):
        defaults = dict(score_adapter.DEFAULT_WEIGHTS)
        defaults["target"] = float(rospy.get_param("~w_target", defaults["target"]))
        defaults["env"] = float(rospy.get_param("~w_env", defaults["env"]))
        defaults["ctx"] = float(rospy.get_param("~w_ctx", defaults["ctx"]))
        return defaults

    def _load_env_params(self):
        defaults = score_adapter.DEFAULT_ENV_PARAMS
        return score_adapter.EnvScoreParams(
            lambda_neg=float(rospy.get_param("~lambda_neg", defaults.lambda_neg)),
            pos_midpoint=float(rospy.get_param("~pos_midpoint", defaults.pos_midpoint)),
            pos_steepness=float(rospy.get_param("~pos_steepness", defaults.pos_steepness)),
            neg_midpoint=float(rospy.get_param("~neg_midpoint", defaults.neg_midpoint)),
            neg_steepness=float(rospy.get_param("~neg_steepness", defaults.neg_steepness)),
        )

    def on_task(self, msg: LsteTask):
        self.current_task = msg
        rospy.loginfo("lste_score_node cached task_id=%s", msg.task_id)

    def on_detections(self, msg: LsteDetections):
        if self.current_task is None:
            rospy.logwarn_throttle(5.0, "lste_score_node waiting for /lste/task before scoring.")
            return
        if (
            not self.allow_task_mismatch
            and self.current_task.task_id
            and msg.task_id
            and msg.task_id != self.current_task.task_id
        ):
            rospy.logwarn_throttle(
                5.0,
                "Detection task_id=%s mismatches cached task_id=%s; skip scoring.",
                msg.task_id,
                self.current_task.task_id,
            )
            return
        try:
            result = score_adapter.compute_scores(
                self.current_task,
                msg,
                weights=self.weights,
                env_params=self.env_params,
            )
        except Exception as exc:
            rospy.logerr("Failed to compute scores: %s", exc)
            return
        self.publish_scores(msg, result)

    def publish_scores(self, detections: LsteDetections, result: score_adapter.ScoreResult):
        msg = LsteScores()
        msg.header = detections.header if isinstance(detections.header, Header) else Header()
        if not msg.header.stamp:
            msg.header.stamp = rospy.Time.now()
        msg.task_id = detections.task_id or (self.current_task.task_id if self.current_task else "")
        msg.detected = bool(result.detected)
        msg.s_target = float(result.s_target)
        msg.s_env = float(result.s_env)
        msg.s_ctx = float(result.s_ctx)
        msg.s_total = float(result.s_total)
        msg.pos_ratio = float(result.pos_ratio)
        msg.neg_ratio = float(result.neg_ratio)
        msg.ctx_coverage = float(result.ctx_coverage)
        msg.ctx_proximity = float(result.ctx_proximity)
        self.pub.publish(msg)
        rospy.loginfo_throttle(
            2.0,
            "Scores published: target=%.3f env=%.3f ctx=%.3f total=%.3f detected=%s",
            msg.s_target,
            msg.s_env,
            msg.s_ctx,
            msg.s_total,
            msg.detected,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    ScoreNode().spin()

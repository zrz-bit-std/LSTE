#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lste_state_node
- 订阅 /lste/task 与 /lste/scores
- 维护 PASS / SUSPICIOUS / LOCKED / EXHAUSTED 状态机（占位版 EXHAUSTED 不依赖 topo）
- 发布 /lste/state
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Deque, Optional

import rospy
from std_msgs.msg import Header

from lste_msgs.msg import LsteScores, LsteState, LsteTask, LsteDetections


STATE_PASS = 0
STATE_SUSPICIOUS = 1
STATE_LOCKED = 2
STATE_EXHAUSTED = 3


@dataclass
class ScoreSample:
    stamp: float
    s_total: float
    s_target: float
    s_env: float
    s_ctx: float
    detected: bool


class StateNode:
    def __init__(self):
        rospy.init_node("lste_state_node")

        # ---- thresholds / parameters (default符合之前设计，可通过 ROS 参数覆盖) ----
        gp = rospy.get_param
        self.promising_total = float(gp("~promising_total_thresh", 0.45))
        self.promising_target = float(gp("~promising_target_thresh", 0.60))
        self.promising_env = float(gp("~promising_env_thresh", 0.35))
        self.promising_ctx = float(gp("~promising_ctx_thresh", 0.35))
        # 细分 subtype 的检测阈值
        self.sus_a_score_thresh = float(gp("~sus_a_score_thresh", 0.4))

        # LOCKED is a temporal confirmation, not a one-frame confidence test.
        # Small real targets often have modest per-frame confidence but remain
        # spatially consistent for many frames.  ``lock_enter_min_hits`` lets
        # a few dropped frames pass without promoting a transient false hit.
        self.lock_enter_total = float(gp("~lock_total_enter", 0.6))
        self.lock_enter_target = float(gp("~lock_target_enter", 0.6))
        self.lock_enter_frames = int(gp("~lock_enter_frames", 2))
        self.lock_enter_min_hits = max(
            1,
            min(
                self.lock_enter_frames,
                int(gp("~lock_enter_min_hits", self.lock_enter_frames)),
            ),
        )

        self.lock_exit_total = float(gp("~lock_total_exit", 0.60))
        self.lock_exit_target = float(gp("~lock_target_exit", 0.60))
        self.lock_exit_unstable = int(gp("~lock_exit_unstable_frames", 5))

        self.susp_window = int(gp("~suspicious_window", 10))
        self.susp_min_hits = int(gp("~suspicious_min_hits", 3))
        self.susp_exit_total = float(gp("~susp_exit_total", 0.35))
        self.susp_exit_target = float(gp("~susp_exit_target", 0.50))
        self.susp_exit_env = float(gp("~susp_exit_env", 0.25))
        self.susp_exit_ctx = float(gp("~susp_exit_ctx", 0.25))
        # Context labels can alternate for adjacent detector frames. Require a
        # short run of the new subtype before Goal Manager is allowed to change
        # its interpretation of the same scene.
        self.subtype_switch_min_hits = max(
            1, int(gp("~subtype_switch_min_hits", 3))
        )

        self.exh_window = int(gp("~exhausted_window", 30))
        self.exh_min_scores = int(gp("~exhausted_min_scores", 30))
        self.exh_total_max = float(gp("~exhausted_total_max", 0.25))
        self.exh_target_max = float(gp("~exhausted_target_max", 0.40))
        self.exh_min_duration = float(gp("~exhausted_min_duration", 90.0))
        self.exh_min_no_promise = float(gp("~exhausted_min_no_promise", 30.0))

        self.allow_task_mismatch = bool(gp("~allow_task_mismatch", False))
        self.republish_interval = float(gp("~republish_interval", 1.0))

        # ---- stateful buffers ----
        self.current_state = STATE_PASS
        self.current_subtype = ""
        self.current_task_id: str = ""
        self.first_score_time: Optional[float] = None
        self.last_promising_time: Optional[float] = None
        self.total_scores_count = 0
        self.locked_unstable_count = 0
        self.pending_subtype = None
        self.pending_subtype_hits = 0
        self.latest_dets: Optional[LsteDetections] = None
        self.latest_task_msg: Optional[LsteTask] = None
        self.scores_short: Deque[ScoreSample] = collections.deque(maxlen=self.susp_window)
        self.lock_samples: Deque[ScoreSample] = collections.deque(maxlen=self.lock_enter_frames)
        self.scores_long: Deque[ScoreSample] = collections.deque(maxlen=self.exh_window)
        self.last_published_time = 0.0

        # ---- ROS I/O ----
        self.pub = rospy.Publisher("/lste/state", LsteState, queue_size=10, latch=True)
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.sub_scores = rospy.Subscriber("/lste/scores", LsteScores, self.on_scores, queue_size=20)
        self.sub_dets = rospy.Subscriber("/lste/detections", LsteDetections, self.on_detections, queue_size=5)

        rospy.loginfo("lste_state_node started.")

    # ----------------- Callbacks -----------------
    def on_task(self, msg: LsteTask):
        if not self.current_task_id or msg.task_id != self.current_task_id:
            self.reset_task(msg.task_id)
            rospy.loginfo("State reset for new task_id=%s", msg.task_id)
        self.latest_task_msg = msg

    def on_detections(self, msg: LsteDetections):
        self.latest_dets = msg

    def on_scores(self, msg: LsteScores):
        if not self.allow_task_mismatch and self.current_task_id:
            if msg.task_id and msg.task_id != self.current_task_id:
                rospy.logwarn_throttle(
                    5.0, "lste_state_node: task mismatch scores(%s) vs cached(%s)", msg.task_id, self.current_task_id
                )
                return
        if not self.current_task_id:
            self.current_task_id = msg.task_id or self.current_task_id

        stamp = msg.header.stamp.to_sec() if isinstance(msg.header, Header) and msg.header.stamp else rospy.Time.now().to_sec()
        sample = ScoreSample(
            stamp=stamp,
            s_total=float(msg.s_total),
            s_target=float(msg.s_target),
            s_env=float(msg.s_env),
            s_ctx=float(msg.s_ctx),
            detected=bool(msg.detected),
        )
        self.ingest_sample(sample)
        self.evaluate_state(sample)

    # ----------------- Core logic -----------------
    def reset_task(self, task_id: str):
        self.current_task_id = task_id
        self.current_state = STATE_PASS
        self.current_subtype = ""
        self.first_score_time = None
        self.last_promising_time = None
        self.total_scores_count = 0
        self.locked_unstable_count = 0
        self.pending_subtype = None
        self.pending_subtype_hits = 0
        self.scores_short.clear()
        self.lock_samples.clear()
        self.scores_long.clear()
        self.last_published_time = 0.0
        self.latest_dets = None
        self.latest_task_msg = None
        self.publish_state(rospy.Time.now().to_sec())

    def ingest_sample(self, sample: ScoreSample):
        self.total_scores_count += 1
        self.scores_short.append(sample)
        self.lock_samples.append(sample)
        self.scores_long.append(sample)
        if self.first_score_time is None:
            self.first_score_time = sample.stamp
        if self.is_promising(sample):
            self.last_promising_time = sample.stamp

    def is_promising(self, sample: ScoreSample) -> bool:
        return (
            sample.s_total >= self.promising_total
            or sample.s_target >= self.promising_target
            or sample.s_env >= self.promising_env
            or sample.s_ctx >= self.promising_ctx
            or sample.detected
        )

    def evaluate_state(self, sample: ScoreSample):
        now = sample.stamp
        prev_state = self.current_state
        prev_subtype = self.current_subtype

        # 1) LOCKED handling
        locked_now = self.handle_locked(sample)
        if locked_now:
            self.current_state = STATE_LOCKED
            self.current_subtype = ""
            self.maybe_publish(now, prev_state, prev_subtype)
            return

        # 2) EXHAUSTED handling
        # 目前暂时禁用 EXHAUSTED 状态：
        # - 不再进入 STATE_EXHAUSTED
        # - 若原本会从 EXHAUSTED 恢复，也直接走后面的 SUSPICIOUS/PASS 逻辑
        # （后续若要恢复 EXHAUSTED，只需把下面两段逻辑改回原来的判断）
        # if self.current_state == STATE_EXHAUSTED and self.is_promising(sample):
        #     self.current_state = STATE_SUSPICIOUS
        #     self.current_subtype = self.pick_suspicious_subtype(sample)
        #     self.maybe_publish(now, prev_state, prev_subtype)
        #     return
        #
        # if self.should_exhaust(now):
        #     self.current_state = STATE_EXHAUSTED
        #     self.current_subtype = "Exhausted-LowScore"
        #     self.maybe_publish(now, prev_state, prev_subtype)
        #     return

        # 3) SUSPICIOUS handling
        if self.should_suspicious():
            self.current_state = STATE_SUSPICIOUS
            candidate_subtype = self.pick_suspicious_subtype(sample)
            if (
                prev_state == STATE_SUSPICIOUS
                and prev_subtype
                and candidate_subtype != prev_subtype
                and candidate_subtype != "Sus-A"
            ):
                if self.pending_subtype == candidate_subtype:
                    self.pending_subtype_hits += 1
                else:
                    self.pending_subtype = candidate_subtype
                    self.pending_subtype_hits = 1
                if self.pending_subtype_hits < self.subtype_switch_min_hits:
                    candidate_subtype = prev_subtype
                else:
                    self.pending_subtype = None
                    self.pending_subtype_hits = 0
            else:
                self.pending_subtype = None
                self.pending_subtype_hits = 0
            self.current_subtype = candidate_subtype
            self.maybe_publish(now, prev_state, prev_subtype)
            return

        # 4) PASS
        self.current_state = STATE_PASS
        self.current_subtype = ""
        self.pending_subtype = None
        self.pending_subtype_hits = 0
        self.maybe_publish(now, prev_state, prev_subtype)

    # ---- Helpers for locked / suspicious / exhausted ----
    def handle_locked(self, sample: ScoreSample) -> bool:
        """Return True if we should remain in or enter LOCKED."""
        # Already locked: check exit conditions
        if self.current_state == STATE_LOCKED:
            if (
                sample.s_total >= self.lock_exit_total
                and sample.s_target >= self.lock_exit_target
                and sample.detected
            ):
                self.locked_unstable_count = 0
                return True
            self.locked_unstable_count += 1
            if self.locked_unstable_count < self.lock_exit_unstable:
                return True
            # Fall through to downgrade if unstable for too long

        # Try to enter locked
        if len(self.lock_samples) < self.lock_enter_frames:
            return False
        recent = list(self.lock_samples)
        confirmed_hits = sum(
            s.s_total >= self.lock_enter_total
            and s.s_target >= self.lock_enter_target
            and s.detected
            for s in recent
        )
        if confirmed_hits < self.lock_enter_min_hits:
            return False
        self.locked_unstable_count = 0
        return True

    def should_suspicious(self) -> bool:
        if len(self.scores_short) == 0:
            return False
        hits = 0
        for s in self.scores_short:
            if self.is_promising(s):
                hits += 1
        if hits >= self.susp_min_hits:
            return True

        # Exit to PASS if window all low
        low_all = all(
            s.s_total < self.susp_exit_total
            and s.s_target < self.susp_exit_target
            and s.s_env < self.susp_exit_env
            and s.s_ctx < self.susp_exit_ctx
            for s in self.scores_short
        )
        return not low_all and self.current_state == STATE_SUSPICIOUS

    def pick_suspicious_subtype(self, sample: ScoreSample) -> str:
        # 按检测结果细分：target>阈值 -> Sus-A；同时命中左右 ctx -> Sus-B；其余默认为 Sus-C
        if self.has_target_detection():
            return "Sus-A"
        if self.has_ctx_pair():
            return "Sus-B"
        return "Sus-C"

    def has_target_detection(self) -> bool:
        if self.latest_dets is None:
            return False
        for det in self.latest_dets.target_dets:
            try:
                if float(det.score) >= self.sus_a_score_thresh:
                    return True
            except Exception:
                continue
        return False

    def has_ctx_pair(self) -> bool:
        if self.latest_dets is None or self.latest_task_msg is None:
            return False
        left_term = (self.latest_task_msg.ctx_left or "").strip().lower()
        right_term = (self.latest_task_msg.ctx_right or "").strip().lower()
        if not left_term or left_term == "none" or not right_term or right_term == "none":
            return False
        all_dets = list(self.latest_dets.env_dets) + list(self.latest_dets.target_dets)
        left_hits = [
            det for det in all_dets
            if left_term in (det.label or "").lower()
        ]
        right_hits = [
            det for det in all_dets
            if right_term in (det.label or "").lower()
        ]
        # ``yellow_cup.json`` uses ``monitor`` for both sides.  A single
        # monitor bounding box must not satisfy both semantic slots: it is one
        # observation, not an inferred pair.  The Goal Manager applies the
        # same distinct-detection rule when it calculates a context midpoint.
        return any(left is not right for left in left_hits for right in right_hits)

    def should_exhaust(self, now: float) -> bool:
        if self.total_scores_count < self.exh_min_scores:
            return False
        if len(self.scores_long) < self.exh_window:
            return False
        if self.first_score_time is None:
            return False
        if now - self.first_score_time < self.exh_min_duration:
            return False
        if self.last_promising_time is not None and now - self.last_promising_time < self.exh_min_no_promise:
            return False
        all_low = all(
            s.s_total < self.exh_total_max and s.s_target < self.exh_target_max and not s.detected
            for s in self.scores_long
        )
        return all_low

    # ----------------- Publishing -----------------
    def maybe_publish(self, now: float, prev_state: int, prev_subtype: str):
        if (
            self.current_state != prev_state
            or self.current_subtype != prev_subtype
            or (now - self.last_published_time) >= self.republish_interval
        ):
            self.publish_state(now)

    def publish_state(self, now: float):
        msg = LsteState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.from_sec(now)
        msg.task_id = self.current_task_id
        msg.state = int(self.current_state)
        msg.subtype = str(self.current_subtype)
        self.pub.publish(msg)
        self.last_published_time = now
        rospy.loginfo_throttle(2.0, "State=%d subtype=%s task_id=%s", msg.state, msg.subtype, msg.task_id)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    StateNode().spin()

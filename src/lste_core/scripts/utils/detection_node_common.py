#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS plumbing shared by the independently implemented detector nodes."""

import json
import threading
import time
from typing import Iterable, Sequence

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from lste_msgs.msg import LsteDetection, LsteDetections, LstePrompts, LsteState, LsteTask
from utils import detector as det_utils
from utils.target_evidence import preserve_raw_target_evidence


STATE_PASS = 0
STATE_SUSPICIOUS = 1
STATE_LOCKED = 2
STATE_EXHAUSTED = 3


def task_to_task_parsed(msg: LsteTask) -> dict:
    if msg.raw_json:
        try:
            data = json.loads(msg.raw_json)
            return data.get("task_parsed", data)
        except Exception as exc:
            rospy.logwarn("Failed to parse raw_json in LsteTask: %s", exc)
    return {
        "target": {"name": msg.target_name, "attributes": list(msg.target_attributes)},
        "env": {
            "env_type_prior": list(msg.env_type_prior),
            "related_structures": list(msg.env_related_structures),
        },
        "obj_related": {
            "key_objects": list(msg.obj_key_objects),
            "negative_clues": list(msg.obj_negative_clues),
        },
        "target_ctx": {"left": msg.ctx_left, "right": msg.ctx_right},
    }


class DetectionNodeBase:
    """Common ROS lifecycle; model-specific inference lives in each node file."""

    detector_name = "detector"

    def __init__(self):
        rospy.init_node("lste_det_node")
        self.interval_pass = float(rospy.get_param("~interval_pass", 1.5))
        self.interval_suspicious = float(rospy.get_param("~interval_suspicious", 1.5))
        self.interval_locked = float(rospy.get_param("~interval_locked", 1.5))
        self.interval_exhausted = float(rospy.get_param("~interval_exhausted", 3.0))
        self.min_inference_interval = float(rospy.get_param("~min_inference_interval", 1.5))
        # A model can finish its initial load after Gazebo has already advanced
        # several seconds.  Publishing that queued image would turn an old
        # camera ray into a new navigation command, so bound source-frame age
        # independently of inference cadence.  Set <= 0 only for offline use.
        self.max_source_image_age = float(rospy.get_param("~max_source_image_age", 1.0))
        # Inference can run at 10 Hz. Keep evidence at a useful diagnostic
        # cadence without turning a long search into gigabytes of repeated
        # identical JSON records. Outcome transitions always bypass this cap.
        self.perception_decision_period = max(
            0.0, float(rospy.get_param("~perception_decision_period", 1.0))
        )
        self._last_perception_decision_wall = 0.0
        self._last_perception_decision_event = ""
        self._suppressed_perception_decisions = 0
        self.current_task = None
        self.task_parsed = None
        self.current_prompts = None
        self.current_state = STATE_PASS
        self.last_det_time = 0.0
        self._infer_lock = threading.Lock()
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_topic = rospy.get_param("~image_topic", "/kinect/hd/image_color_rect")
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.sub_prompts = rospy.Subscriber("/lste/prompts", LstePrompts, self.on_prompts, queue_size=1)
        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)
        self.sub_state = rospy.Subscriber("/lste/state", LsteState, self.on_state, queue_size=1)
        self.pub = rospy.Publisher("/lste/detections", LsteDetections, queue_size=5)
        # The final detection message intentionally contains only accepted
        # boxes. Publish a compact companion record for every inference so a
        # failed search can distinguish model rejection from later filtering
        # or a stale source frame without recording camera images to disk.
        self.perception_decision_pub = rospy.Publisher(
            "/lste/perception_decision", String, queue_size=20
        )

    def on_task(self, msg: LsteTask):
        self.current_task = msg
        self.task_parsed = task_to_task_parsed(msg)
        rospy.loginfo("Received /lste/task task_id=%s", msg.task_id)

    def on_prompts(self, msg: LstePrompts):
        self.current_prompts = msg
        rospy.loginfo("Received /lste/prompts for task_id=%s", msg.task_id)

    def on_state(self, msg: LsteState):
        self.current_state = int(msg.state)

    def on_image(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if image is None or getattr(image, "size", 0) == 0:
                raise ValueError("empty image")
            self.latest_image = (msg.header, image)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Failed to convert Image: %s", exc)
            self.latest_image = (msg.header, None)

    def publish_empty_dets(self, header=None, reason: str = ""):
        self._publish(header, "", [], [], [], [], [])
        if reason:
            rospy.logwarn_throttle(1.0, "Publish empty detections (%s)", reason)

    def publish_perception_decision(
        self,
        header,
        event: str,
        prompt_a: str = "",
        prompt_b_terms=(),
        force: bool = False,
        **details
    ):
        """Publish one auditable outcome for an attempted detector inference."""
        now_wall = time.monotonic()
        same_outcome = str(event) == self._last_perception_decision_event
        if (
            not force
            and same_outcome
            and self.perception_decision_period > 0.0
            and now_wall - self._last_perception_decision_wall
            < self.perception_decision_period
        ):
            self._suppressed_perception_decisions += 1
            return
        stamp = 0.0
        sequence = None
        frame_id = ""
        if isinstance(header, Header):
            stamp = header.stamp.to_sec()
            sequence = int(header.seq)
            frame_id = str(header.frame_id)
        now = rospy.Time.now().to_sec()
        payload = {
            "event": str(event),
            "detector": self.detector_name,
            "task_id": self.current_task.task_id if self.current_task else "",
            "source_image_seq": sequence,
            "source_image_stamp": round(stamp, 6),
            "source_image_age_seconds": round(max(0.0, now - stamp), 4) if stamp > 0.0 else None,
            "source_image_frame": frame_id,
            "state": int(self.current_state),
            "prompt_a": str(prompt_a),
            "prompt_b_terms": [str(term) for term in prompt_b_terms],
            "suppressed_same_outcome_count": self._suppressed_perception_decisions,
        }
        payload.update(details)
        self.perception_decision_pub.publish(
            String(data=json.dumps(payload, sort_keys=True, separators=(",", ":")))
        )
        self._last_perception_decision_wall = now_wall
        self._last_perception_decision_event = str(event)
        self._suppressed_perception_decisions = 0

    @staticmethod
    def _candidate_summary(scores, labels, limit: int = 3):
        pairs = []
        for score, label in zip(DetectionNodeBase._as_list(scores), labels):
            pairs.append({"label": str(label), "score": round(float(score), 4)})
        return sorted(pairs, key=lambda item: item["score"], reverse=True)[:limit]

    def _publish(self, header, prompt_a, prompt_b_terms, target_boxes, target_scores, target_labels,
                 env_boxes, env_scores=None, env_labels=None):
        msg = LsteDetections()
        msg.header = header if isinstance(header, Header) else Header(stamp=rospy.Time.now())
        if not msg.header.stamp:
            msg.header.stamp = rospy.Time.now()
        msg.task_id = self.current_task.task_id if self.current_task else ""
        msg.prompt_a = prompt_a
        msg.prompt_b_terms = list(prompt_b_terms)
        self._append_detections(msg.target_dets, target_boxes, target_scores, target_labels)
        self._append_detections(msg.env_dets, env_boxes, env_scores or [], env_labels or [])
        self.pub.publish(msg)
        rospy.loginfo_throttle(
            5.0, "Published /lste/detections via %s: %d target boxes, %d env boxes (task_id=%s)",
            self.detector_name, len(msg.target_dets), len(msg.env_dets), msg.task_id,
        )

    @staticmethod
    def _append_detections(destination, boxes, scores, labels):
        for index, box in enumerate(boxes):
            det = LsteDetection()
            det.label = str(labels[index]) if index < len(labels) else ""
            det.score = float(scores[index]) if index < len(scores) else 0.0
            try:
                det.cx, det.cy, det.w, det.h = [float(value) for value in box]
            except Exception:
                det.cx = det.cy = det.w = det.h = 0.0
            destination.append(det)

    def current_interval(self) -> float:
        interval = {
            STATE_PASS: self.interval_pass,
            STATE_SUSPICIOUS: self.interval_suspicious,
            STATE_LOCKED: self.interval_locked,
            STATE_EXHAUSTED: self.interval_exhausted,
        }.get(self.current_state, self.interval_pass)
        return max(self.min_inference_interval, interval)

    def ready(self) -> bool:
        return (
            self.current_task is not None
            and self.task_parsed is not None
            and self.current_prompts is not None
            and self.latest_image is not None
            and (not self.current_prompts.task_id or self.current_prompts.task_id == self.current_task.task_id)
        )

    def should_detect(self) -> bool:
        now = time.time()
        if now - self.last_det_time < self.current_interval():
            return False
        self.last_det_time = now
        return True

    @staticmethod
    def _as_list(values) -> list:
        if values is None:
            return []
        try:
            return values.cpu().tolist()
        except AttributeError:
            return values.tolist() if hasattr(values, "tolist") else list(values)

    def _postprocess(self, image_source, target_boxes, target_scores, target_labels,
                     env_boxes, env_scores, env_labels):
        required_colors = det_utils.extract_color_terms(
            (self.task_parsed.get("target") or {}).get("attributes", [])
        )
        if required_colors and target_boxes is not None and len(target_boxes) > 0:
            target_boxes, target_scores, target_labels = det_utils.postprocess_target_candidates(
                image_source,
                target_boxes,
                target_scores,
                target_labels,
                required_colors,
                ratio_threshold=0.02,
            )
        if target_boxes is not None and len(target_boxes) > 1:
            target_boxes, target_scores, target_labels = det_utils.nms_iou(
                target_boxes, target_scores, target_labels, threshold=0.9
            )
        if target_boxes is not None and len(target_boxes) > 0:
            # Color is an optional corroborating observation, not a second
            # semantic detector. ``postprocess_target_candidates`` preserves
            # model candidates when the tiny/occluded crop has no reliable
            # color pixels; applying ``validate_color_by_phrase`` here would
            # turn valid WeDetect candidates into empty target messages.
            target_boxes, target_scores, target_labels = det_utils.keep_top_confidence_detection(
                target_boxes, target_scores, target_labels
            )
        if env_boxes is not None and target_boxes is not None and len(env_boxes) > 0 and len(target_boxes) > 0:
            env_boxes, env_scores, env_labels = det_utils.filter_boxes_by_overlap(
                env_boxes, env_scores, env_labels, target_boxes, threshold=0.5
            )
        if env_boxes is not None and len(env_boxes) > 0:
            env_boxes, env_scores, env_labels = det_utils.validate_color_by_phrase(
                image_source, env_boxes, env_scores, env_labels, ratio_threshold=0.02,
                blur_ksize=3, dilate_iter=1,
            )
        if env_boxes is not None and len(env_boxes) > 1:
            env_boxes, env_scores, env_labels = det_utils.nms_iou(
                env_boxes, env_scores, env_labels, threshold=0.9
            )
        return target_boxes, target_scores, target_labels, env_boxes, env_scores, env_labels

    def run_detection(self):
        if not self._infer_lock.acquire(blocking=False):
            rospy.logwarn_throttle(1.0, "Skip frame: inference busy")
            return
        try:
            if self.latest_image is None:
                return
            header, image_bgr = self.latest_image
            if image_bgr is None:
                self.publish_perception_decision(header, "empty_image")
                self.publish_empty_dets(header, "empty image frame")
                return
            prompt_a = self.current_prompts.prompt_a
            prompt_b_terms = list(self.current_prompts.prompt_b_terms)
            if not prompt_a or not prompt_b_terms:
                self.publish_perception_decision(
                    header, "empty_prompt", prompt_a, prompt_b_terms
                )
                self.publish_empty_dets(header, "empty prompt")
                return
            try:
                raw_result = self.infer(image_bgr, prompt_a, prompt_b_terms)
                inference_metadata = self.inference_metadata()
                raw_target_boxes, raw_target_scores, raw_target_labels = raw_result[1:4]
                raw_env_boxes, raw_env_scores, raw_env_labels = raw_result[4:7]
                result = self._postprocess(*raw_result)
                (
                    target_boxes,
                    target_scores,
                    target_labels,
                    target_evidence_fallback,
                    target_evidence_fallback_reason,
                ) = preserve_raw_target_evidence(
                    raw_target_boxes,
                    raw_target_scores,
                    raw_target_labels,
                    result[0],
                    result[1],
                    result[2],
                )
                result = (
                    target_boxes,
                    target_scores,
                    target_labels,
                    result[3],
                    result[4],
                    result[5],
                )
            except Exception as exc:
                rospy.logerr_throttle(1.0, "%s exception: %s", self.detector_name, exc)
                self.publish_perception_decision(
                    header,
                    "inference_exception",
                    prompt_a,
                    prompt_b_terms,
                    exception=type(exc).__name__,
                )
                self.publish_empty_dets(header, "exception: %s" % type(exc).__name__)
                return
            stamp = header.stamp.to_sec() if isinstance(header, Header) else 0.0
            age = rospy.Time.now().to_sec() - stamp if stamp > 0.0 else 0.0
            if self.max_source_image_age > 0.0 and age > self.max_source_image_age:
                rospy.logwarn_throttle(
                    1.0,
                    "Discard %s result from stale image: age=%.2fs limit=%.2fs",
                    self.detector_name,
                    age,
                    self.max_source_image_age,
                )
                self.publish_perception_decision(
                    header,
                    "stale_source_image",
                    prompt_a,
                    prompt_b_terms,
                    model_target_candidates=len(raw_target_boxes),
                    model_env_candidates=len(raw_env_boxes),
                    model_env_top=self._candidate_summary(raw_env_scores, raw_env_labels),
                    published_target_candidates=len(result[0]),
                    published_env_candidates=len(result[3]),
                    published_env_top=self._candidate_summary(result[4], result[5]),
                    max_source_image_age_seconds=self.max_source_image_age,
                    force=bool(inference_metadata.get("small_object_tile_search_ran")),
                    **inference_metadata,
                )
                return
            target_boxes, target_scores, target_labels, env_boxes, env_scores, env_labels = result
            raw_target_count = len(raw_target_boxes)
            target_count = len(target_boxes)
            outcome = (
                "model_no_target_candidate"
                if raw_target_count == 0
                else "target_published_from_raw_evidence"
                if target_evidence_fallback
                else "target_removed_by_postprocess"
                if target_count == 0
                else "target_published"
            )
            self.publish_perception_decision(
                header,
                outcome,
                prompt_a,
                prompt_b_terms,
                model_target_candidates=raw_target_count,
                model_env_candidates=len(raw_env_boxes),
                model_target_top=self._candidate_summary(raw_target_scores, raw_target_labels),
                model_env_top=self._candidate_summary(raw_env_scores, raw_env_labels),
                published_target_candidates=target_count,
                published_env_candidates=len(env_boxes),
                published_target_top=self._candidate_summary(target_scores, target_labels),
                published_env_top=self._candidate_summary(env_scores, env_labels),
                target_evidence_fallback=bool(target_evidence_fallback),
                target_evidence_fallback_reason=target_evidence_fallback_reason,
                force=bool(inference_metadata.get("small_object_tile_search_ran")),
                **inference_metadata,
            )
            self._publish(
                header, prompt_a, prompt_b_terms,
                self._as_list(target_boxes), self._as_list(target_scores), [str(v) for v in target_labels],
                self._as_list(env_boxes), self._as_list(env_scores), [str(v) for v in env_labels],
            )
        finally:
            self._infer_lock.release()

    def spin(self):
        rate = rospy.Rate(max(0.1, float(rospy.get_param("~spin_hz", 10.0))))
        while not rospy.is_shutdown():
            if self.ready() and self.should_detect():
                self.run_detection()
            rate.sleep()

    def infer(self, image_bgr, prompt_a: str, prompt_b_terms: Sequence[str]):
        raise NotImplementedError

    def inference_metadata(self):
        """Return detector-specific, JSON-safe observability fields.

        A detector may use a conditional second pass without changing the
        common ROS message contract. The fields are diagnostic only and are
        published with the perception decision, never consumed as control.
        """
        return {}

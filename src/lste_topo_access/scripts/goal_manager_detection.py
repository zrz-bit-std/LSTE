"""Detector evidence state machine used by GoalManager.

The ROS-facing class keeps the callback name and subscription contract, while
this module owns the long-lived target-track bookkeeping for one detector frame.
"""

from dataclasses import dataclass
import math

import rospy

from goal_manager_target_geometry import (
    MIN_TRANSLATION_BASELINE_M,
    supports_stable_static_target,
    supports_static_target,
)
from target_ray_hypothesis import estimate_target_point


# Confirmation is an observation problem, not a frame-count problem. A small
# low-confidence box must be seen from a genuinely different viewpoint before
# it can acquire navigation ownership. These are geometric evidence bounds,
# independent of detector score tuning.
VIEWPOINT_BASELINE_M = 0.20
VIEWPOINT_BASELINE_YAW_RAD = math.radians(15.0)


@dataclass(frozen=True)
class DetectionEvidenceContext:
    """Values shared by the candidate, heading, and confirmation stages."""

    det: object
    score: float
    box_size: float
    now: float
    projection_stamp: object
    image_stamp_seconds: float
    center_delta: float
    candidate_average_score: float


def record_detector_frame(manager, msg):
    """Record one detector message for the confirmed-target loss gate.

    Empty ``LsteDetections`` messages are meaningful after a reached target
    segment. They are intentionally counted separately from
    ``target_observation_epoch``, which only counts target-bearing frames used
    to project a visual ray.
    """
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    try:
        stamp_key = (
            int(getattr(stamp, "secs", 0)),
            int(getattr(stamp, "nsecs", 0)),
            int(getattr(getattr(msg, "header", None), "seq", 0)),
        )
    except (TypeError, ValueError):
        stamp_key = None
    if (
        stamp_key is not None
        and stamp_key == getattr(manager, "target_detector_last_stamp", None)
        and any(stamp_key)
    ):
        return False
    manager.target_detector_last_stamp = stamp_key
    manager.target_detector_frame_epoch = int(
        getattr(manager, "target_detector_frame_epoch", 0)
    ) + 1
    gate = getattr(manager, "target_observation_gate", None)
    if gate is None or not gate.active:
        return True
    detector = manager.target_detection_for_track(msg)
    visible = False
    if detector is not None:
        try:
            visible = (
                float(detector.score) >= float(manager.target_follow_min_score)
                and max(float(detector.w), float(detector.h))
                >= float(manager.target_follow_min_box_size)
            )
        except (AttributeError, TypeError, ValueError):
            visible = False
    try:
        now = rospy.Time.now().to_sec()
    except AttributeError:
        now = 0.0
    gate.observe(
        getattr(manager, "target_track_id", ""),
        manager.target_detector_frame_epoch,
        visible,
        now,
        viewpoint_key=_target_viewpoint_key(manager),
    )
    return True


def _target_viewpoint_key(manager):
    """Quantize physical camera viewpoints for absence verification.

    The key is used only to establish parallax/viewpoint diversity. Its scale
    is the existing detector confirmation baseline, not a tunable navigation
    threshold and never affects target geometry or route selection.
    """
    pose = getattr(manager, "latest_pose", None)
    if pose is None:
        return None
    try:
        return (
            int(round(float(pose.x) / VIEWPOINT_BASELINE_M)),
            int(round(float(pose.y) / VIEWPOINT_BASELINE_M)),
            int(round(float(pose.theta) / VIEWPOINT_BASELINE_YAW_RAD)),
        )
    except (AttributeError, TypeError, ValueError, ZeroDivisionError):
        return None


def handle_detections(manager, msg):
    """Process one detector frame without publishing a navigation goal."""
    manager.latest_dets = msg
    if getattr(manager, "target_done_policy", "visual_target") in (
        "target_room_entry",
        "target_context",
    ):
        # Doorway-entry acceptance is a topology probe. Visual target tracks
        # and context-only completion are intentionally observational here;
        # letting a box claim a target route would divert the explorer before
        # the completion timer can evaluate the declared acceptance contract.
        return
    # Once the task has reached its terminal state, keep recording the
    # latest detector message for diagnostics but do not refresh target
    # follow/observation-hold state.
    if manager.task_done_published:
        return
    det = manager.target_detection_for_track(msg)
    if det is None:
        return
    score = float(det.score)
    box_size = max(float(det.w), float(det.h))
    if score < manager.target_follow_min_score or box_size < manager.target_follow_min_box_size:
        return

    now = rospy.Time.now().to_sec()
    evidence = _record_candidate(manager, msg, det, score, box_size, now)
    if evidence is None:
        return
    if not _update_target_heading(manager, evidence):
        return
    _apply_confirmation_policy(manager, evidence)


def _record_candidate(manager, msg, det, score, box_size, now):
    """Update target identity and candidate votes for one new frame."""
    image_stamp = msg.header.stamp
    image_stamp_seconds = image_stamp.to_sec() if image_stamp else 0.0
    # A zero stamp is an explicitly legacy/externally produced message.
    # Real detector output carries its original camera header and must be
    # projected using historical TF.
    projection_stamp = (
        image_stamp if image_stamp_seconds > 0.0 else rospy.Time(0)
    )
    source_stamp = _detector_source_identity(manager, msg)
    # A detector result may be observed by the Goal Manager several times
    # before the next inference completes.  It is evidence only once;
    # reusing it must not move the target goal again.
    if manager.target_last_detection_stamp == source_stamp:
        return
    manager.target_last_detection_stamp = source_stamp
    manager.target_observation_epoch += 1
    if not manager.target_blocked:
        manager.target_execution_state = "TARGET_CANDIDATE"
    within_window = (
        manager.target_candidate_last_seen is not None
        and now - manager.target_candidate_last_seen <= manager.target_follow_confirm_window
    )
    if not within_window:
        manager.target_candidate_hits = 0
        manager.target_candidate_score_sum = 0.0
        manager.target_candidate_anchor_cx = None
        manager.target_candidate_anchor_cy = None
        manager.target_candidate_viewpoint_anchor_odom = None
        manager.target_candidate_viewpoint_anchor_yaw = None
        manager.target_candidate_viewpoint_translation = 0.0
        manager.target_candidate_viewpoint_yaw_delta = 0.0
        manager.target_candidate_viewpoint_diverse = False
        manager.target_ray_history = []
    _update_viewpoint_baseline(manager, within_window)
    center_delta = float("inf")
    if (
        within_window
        and manager.target_candidate_anchor_cx is not None
        and manager.target_candidate_anchor_cy is not None
    ):
        center_delta = math.hypot(
            float(det.cx) - manager.target_candidate_anchor_cx,
            float(det.cy) - manager.target_candidate_anchor_cy,
        )
        # Image-center motion is expected when the base moves around a small
        # object. Track identity is established by semantic compatibility and
        # source-stamped ray geometry; center_delta remains diagnostic only.
    if manager.target_candidate_source_stamp != source_stamp:
        manager.target_candidate_hits += 1
        manager.target_candidate_score_sum += score
    manager.target_candidate = manager.clone_detection(det)
    if not manager.target_track_label:
        manager.target_track_label = (det.label or "").strip().lower()
        manager.target_track_sequence += 1
        manager.target_track_id = "%s:%s:%d" % (
            manager.current_task_id or "task",
            manager.target_track_label or "target",
            manager.target_track_sequence,
        )
        # The first target track creates a mission-level room-observation
        # obligation.  Its release is emitted only by the explicit loss/clear
        # lifecycle, so a transient context-only frame cannot silently let the
        # graph abandon the Place or leave an unresolved ledger entry behind.
        manager.target_candidate_room_claim_requested = True
        # A candidate frame owns a Place-level observation obligation before
        # it owns motion. The same gate can therefore require an active
        # reinspection when a weak target disappears before confirmation.
        observation_gate = getattr(manager, "target_observation_gate", None)
        if observation_gate is not None:
            observation_gate.confirm(
                manager.target_track_id,
                int(getattr(manager, "target_detector_frame_epoch", 0)),
                now,
            )
        # A target track is the unit of evidence for a visual approach.
        # Emit its creation explicitly so telemetry never combines the
        # discovery of one image-track with the completion of another.
        manager.publish_goal_arbitration(
            "target_track_started",
            target_track_id=manager.target_track_id,
            label=manager.target_track_label,
            score=round(float(score), 4),
            box=[round(float(det.w), 4), round(float(det.h), 4)],
            center=[round(float(det.cx), 4), round(float(det.cy), 4)],
        )
    manager.target_candidate_last_seen = now
    # A direct target box is already a room-level lead, even before the
    # second frame promotes it to a navigation track.  Keep its timestamp
    # separately from monitor/context state so a missing monitor cannot
    # make the subsequent candidate timeout look like "no target was ever
    # seen" and release the room immediately.
    manager.target_last_seen = now
    manager.target_candidate_source_stamp = source_stamp
    manager.target_candidate_anchor_cx = float(det.cx)
    manager.target_candidate_anchor_cy = float(det.cy)
    candidate_average_score = manager.target_candidate_score_sum / max(
        1, manager.target_candidate_hits
    )

    # A single detector frame is evidence, not a mission decision. In
    # particular, it must not request a frontier replan or acquire a room
    # lease: one weak false positive previously changed the exploration route
    # and kept the vehicle in an otherwise complete room. Confirmation below
    # is the only boundary at which visual evidence may influence control.

    return DetectionEvidenceContext(
        det=det,
        score=score,
        box_size=box_size,
        now=now,
        projection_stamp=projection_stamp,
        image_stamp_seconds=image_stamp_seconds,
        center_delta=center_delta,
        candidate_average_score=candidate_average_score,
    )


def _detector_source_identity(manager, msg):
    """Return a per-message identity even for legacy zero-stamped inputs.

    Some replay/fake publishers leave ``stamp`` and ``seq`` at zero. Treating
    that tuple as a real identity drops every frame after the first one. The
    input callback already advances ``target_detector_frame_epoch`` for each
    received message, so it is the preferred receipt identity; direct policy
    tests and old callers use a local receipt counter instead.
    """
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    try:
        stamp_key = (
            int(getattr(stamp, "secs", 0)),
            int(getattr(stamp, "nsecs", 0)),
            int(getattr(header, "seq", 0)),
        )
    except (TypeError, ValueError):
        stamp_key = (0, 0, 0)
    if any(stamp_key):
        return stamp_key
    frame_epoch = int(getattr(manager, "target_detector_frame_epoch", 0) or 0)
    if frame_epoch <= 0:
        frame_epoch = int(
            getattr(manager, "target_detector_receipt_epoch", 0) or 0
        ) + 1
        manager.target_detector_receipt_epoch = frame_epoch
    return ("receipt", frame_epoch)


def _update_viewpoint_baseline(manager, within_window):
    """Track whether candidate votes come from different robot viewpoints."""
    pose = manager.latest_pose
    if pose is None:
        return
    current = (float(pose.x), float(pose.y))
    current_yaw = float(pose.theta)
    if not within_window or manager.target_candidate_viewpoint_anchor_odom is None:
        manager.target_candidate_viewpoint_anchor_odom = current
        manager.target_candidate_viewpoint_anchor_yaw = current_yaw
        return
    anchor = manager.target_candidate_viewpoint_anchor_odom
    translation = math.hypot(current[0] - anchor[0], current[1] - anchor[1])
    yaw_delta = abs(_wrap_angle(
        current_yaw - float(manager.target_candidate_viewpoint_anchor_yaw)
    ))
    manager.target_candidate_viewpoint_translation = max(
        float(manager.target_candidate_viewpoint_translation), translation
    )
    manager.target_candidate_viewpoint_yaw_delta = max(
        float(manager.target_candidate_viewpoint_yaw_delta), yaw_delta
    )
    manager.target_candidate_viewpoint_diverse = (
        manager.target_candidate_viewpoint_translation >= VIEWPOINT_BASELINE_M
        or manager.target_candidate_viewpoint_yaw_delta >= VIEWPOINT_BASELINE_YAW_RAD
    )


def _update_target_heading(manager, evidence):
    """Project and smooth the detector ray at its camera exposure time."""
    det = evidence.det
    projection_stamp = evidence.projection_stamp
    image_stamp_seconds = evidence.image_stamp_seconds
    # Smooth the normalized image centre and then transform that single
    # filtered ray into odom.  Limit the per-frame heading change as a
    # second guard against a one-frame false association while allowing a
    # genuine turn to converge over a few detector frames.
    if manager.target_filtered_cx is None:
        manager.target_filtered_cx = float(det.cx)
        manager.target_filtered_cy = float(det.cy)
    else:
        alpha = manager.target_bbox_alpha
        manager.target_filtered_cx += alpha * (float(det.cx) - manager.target_filtered_cx)
        manager.target_filtered_cy += alpha * (float(det.cy) - manager.target_filtered_cy)
    filtered_det = manager.clone_detection(det)
    filtered_det.cx = manager.target_filtered_cx
    filtered_det.cy = manager.target_filtered_cy
    raw_heading = manager.det_heading_world(det, projection_stamp)
    filtered_heading = manager.det_heading_world(filtered_det, projection_stamp)
    if filtered_heading is not None:
        if manager.target_filtered_heading is None:
            manager.target_filtered_heading = filtered_heading
        else:
            delta = _wrap_angle(filtered_heading - manager.target_filtered_heading)
            delta = max(
                -manager.target_max_heading_step,
                min(manager.target_max_heading_step, delta),
            )
            manager.target_filtered_heading = _wrap_angle(
                manager.target_filtered_heading + manager.target_heading_alpha * delta
            )
        manager.target_last_heading = manager.target_filtered_heading
        manager.target_last_heading_source_stamp = image_stamp_seconds
        observation_origin = None
        origin_from_tf = False
        origin_at_stamp = getattr(manager, "det_origin_world", None)
        if callable(origin_at_stamp):
            observation_origin = origin_at_stamp(projection_stamp)
            origin_from_tf = observation_origin is not None
        if observation_origin is None and manager.latest_pose is not None:
            # Legacy/replay fixtures may not expose a camera TF origin. Keep
            # the historical fallback, but mark the source in the event so a
            # run can distinguish time-consistent geometry from a degraded
            # latest-pose estimate.
            observation_origin = (
                float(manager.latest_pose.x),
                float(manager.latest_pose.y),
            )
        if observation_origin is not None:
            manager.target_last_observation_odom = observation_origin
            rays = list(getattr(manager, "target_ray_history", []))
            rays.append((
                manager.target_last_observation_odom,
                (
                    math.cos(float(manager.target_last_heading)),
                    math.sin(float(manager.target_last_heading)),
                ),
            ))
            manager.target_ray_history = rays[-6:]
            estimate = estimate_target_point(manager.target_ray_history)
            if estimate is not None:
                manager.target_hypothesis_xy = estimate.point_xy
                manager.target_hypothesis_residual = estimate.residual
                manager.target_hypothesis_ray_count = estimate.ray_count
                manager.publish_goal_arbitration(
                    "target_geometry_hypothesis_updated",
                    target_track_id=manager.target_track_id,
                    point_odom=[
                        round(float(estimate.point_xy[0]), 4),
                        round(float(estimate.point_xy[1]), 4),
                    ],
                    residual=round(float(estimate.residual), 4),
                    ray_count=int(estimate.ray_count),
                    ray_origin_source=(
                        "source_stamped_camera_tf"
                        if origin_from_tf
                        else "latest_pose_fallback"
                    ),
                )
    elif raw_heading is not None and manager.target_filtered_heading is None:
        manager.target_filtered_heading = raw_heading
        manager.target_last_heading = raw_heading
        manager.target_last_heading_source_stamp = image_stamp_seconds
    elif raw_heading is None:
        # Never transform a historical image ray with current TF. During a
        # turn that turns latency into a false world-space target.
        rospy.logwarn_throttle(
            1.0,
            "GoalManager: ignoring target evidence without %s camera TF "
            "(image_stamp=%.6f)",
            "source-stamped" if image_stamp_seconds > 0.0 else "latest",
            image_stamp_seconds,
        )
        return
    return True


def _apply_confirmation_policy(manager, evidence):
    """Apply target confirmation and close-range observation-hold policy."""
    det = evidence.det
    score = evidence.score
    box_size = evidence.box_size
    now = evidence.now
    image_stamp_seconds = evidence.image_stamp_seconds
    center_delta = evidence.center_delta
    candidate_average_score = evidence.candidate_average_score
    # Do not build a new navigation goal here.  This callback receives
    # asynchronous detector frames while TEB is optimizing its current
    # band.  Re-projecting the image ray from the robot pose at this point
    # changes the action goal during motion and creates an avoidable
    # brake/turn pulse.  ``goal_from_target_follow`` commits the next
    # segment only after the current action is terminal.

    high_quality = score >= max(0.40, manager.target_done_min_score) or box_size >= 0.05
    viewpoint_diverse = bool(manager.target_candidate_viewpoint_diverse)
    ray_geometry_supported = supports_static_target(
        getattr(manager, "target_ray_history", [])
    )
    # A weak detector score must be supported by redundant, translationally
    # observable geometry.  The old path used ``target_follow_confirm_hits``
    # (two frames) and accepted pure yaw as viewpoint diversity; a false box
    # could therefore acquire navigation ownership before depth was observable.
    stable_geometry_supported = supports_stable_static_target(
        getattr(manager, "target_ray_history", [])
    )
    translation_diverse = (
        float(getattr(manager, "target_candidate_viewpoint_translation", 0.0))
        >= MIN_TRANSLATION_BASELINE_M
    )
    # Strong detections may use the existing two-frame track contract, but
    # still need the estimator-backed geometry guard.  Their close image
    # evidence provides identity; no uncalibrated point is used for routing.
    weak_quality_confirmed = (
        manager.target_candidate_hits >= manager.target_follow_weak_confirm_hits
        and candidate_average_score >= manager.target_follow_weak_min_average_score
        and translation_diverse
        and stable_geometry_supported
    )
    high_quality_confirmation = (
        high_quality
        and manager.target_candidate_hits >= manager.target_follow_confirm_hits
        and viewpoint_diverse
        and ray_geometry_supported
    )
    if high_quality_confirmation or weak_quality_confirmed:
        if not manager.target_follow_confirmed:
            rospy.loginfo(
                "GoalManager: target follow confirmed hits=%d avg_score=%.3f "
                "score=%.3f box=(%.3f,%.3f) center_delta=%.3f",
                manager.target_candidate_hits,
                candidate_average_score,
                score,
                float(det.w),
                float(det.h),
                center_delta if math.isfinite(center_delta) else float("nan"),
            )
            manager.publish_goal_arbitration(
                "target_follow_confirmed",
                target_track_id=manager.target_track_id,
                hits=int(manager.target_candidate_hits),
                average_score=round(float(candidate_average_score), 4),
                score=round(float(score), 4),
                box=[round(float(det.w), 4), round(float(det.h), 4)],
                center=[round(float(det.cx), 4), round(float(det.cy), 4)],
                confirmation=(
                    "strong_detection"
                    if high_quality
                    else "weak_consistent_detection"
                ),
                source_image_stamp=round(float(image_stamp_seconds), 6),
                projection_tf_mode=(
                    "source_stamp"
                    if image_stamp_seconds > 0.0
                    else "latest_for_unstamped_message"
                ),
                viewpoint_diverse=viewpoint_diverse,
                viewpoint_translation=round(
                    float(manager.target_candidate_viewpoint_translation), 3
                ),
                viewpoint_yaw_delta=round(
                    float(manager.target_candidate_viewpoint_yaw_delta), 4
                ),
                ray_geometry_supported=bool(ray_geometry_supported),
                stable_geometry_supported=bool(stable_geometry_supported),
                translation_diverse=bool(translation_diverse),
                weak_confirm_hits_required=int(
                    manager.target_follow_weak_confirm_hits
                ),
                target_ray_count=len(getattr(manager, "target_ray_history", [])),
            )
        manager.target_follow_confirmed = True
        manager.target_reinspection_pending = False
        manager.target_last_seen = now
        approach_transaction = getattr(
            manager, "target_approach_transaction", None
        )
        if approach_transaction is not None:
            started = approach_transaction.begin(
                manager.target_track_id,
                now,
                target_epoch=int(
                    getattr(manager, "target_track_sequence", 0) or 0
                ),
            )
            approach_transaction.observe(
                manager.target_track_id,
                now,
                close=manager.target_detection_is_close(det),
            )
            if started:
                manager.publish_goal_arbitration(
                    "target_approach_transaction_started",
                    target_track_id=manager.target_track_id,
                    state=approach_transaction.snapshot().state,
                )
        observation_gate = getattr(manager, "target_observation_gate", None)
        if observation_gate is not None:
            observation_gate.confirm(
                manager.target_track_id,
                int(getattr(manager, "target_detector_frame_epoch", 0)),
                now,
            )
        # When a confirmed target is already large enough in the image,
        # the safe behaviour is to observe from this pose until the
        # multi-frame completion contract resolves.  Do not release the
        # candidate hold merely because its second frame confirmed the
        # target track: that used to send the base toward the desk, lose
        # the cup, and let a flickering monitor state release the room.
        if manager.target_detection_is_close(det) and manager.target_completed_segments < 1:
            hold_until = now + manager.target_follow_confirm_window
            extended = hold_until > manager.target_observation_hold_until
            manager.target_observation_hold_until = max(
                manager.target_observation_hold_until, hold_until
            )
            manager.target_execution_state = "TARGET_CLOSE_OBSERVING"
            manager.set_navigation_hold(True, "direct_close_confirmation")
            if not manager.target_direct_close_hold_reported:
                manager.target_direct_close_hold_reported = True
                manager.publish_goal_arbitration(
                    "target_direct_close_observation_hold",
                    target_track_id=manager.target_track_id,
                    score=round(float(score), 4),
                    score_threshold=round(
                        float(manager.target_close_score_threshold()), 4
                    ),
                    box=[round(float(det.w), 4), round(float(det.h), 4)],
                    required_hits=int(manager.target_done_min_fresh_hits),
                    hold_seconds=round(
                        float(manager.target_follow_confirm_window), 3
                    ),
                    hold_extended=bool(extended),
                )
        elif (
            manager.navigation_hold_active
            and not manager.target_terminal_reobserve_pending
        ):
            # Confirmation is now an explicit visual-track transaction;
            # let the next timer tick validate and commit its Navfn/TEB
            # approach segment instead of waiting for the full candidate
            # timeout.
            manager.set_navigation_hold(False, "target_follow_confirmed")
    else:
        rospy.loginfo_throttle(
            2.0,
            "GoalManager: target candidate hits=%d/%d avg_score=%.3f "
            "score=%.3f box=(%.3f,%.3f) center_delta=%.3f",
            manager.target_candidate_hits,
            manager.target_follow_weak_confirm_hits,
            candidate_average_score,
            score,
            float(det.w),
            float(det.h),
            center_delta if math.isfinite(center_delta) else float("nan"),
        )

    # Observation is deliberately decoupled from motion.  The detector
    # keeps contributing fresh evidence while the committed TEB segment is
    # running; only maybe_publish_task_done() can stop the vehicle after
    # the independent close-target completion contract is satisfied.


def _wrap_angle(angle):
    """Normalize an angle without coupling this module to GoalManager."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

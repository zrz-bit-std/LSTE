"""Structured log formatting and target-session attribution.

Navigation metrics has several independent ROS callback families.  They all
write the same line protocol, and target latency must be attributed to a
stable ``(task_id, target_track_id)`` pair.  This mixin keeps those two
cross-cutting concerns out of the node composition root.
"""

import datetime
import json

import rospy


class NavigationMetricsLoggingMixin:
    """Provide the telemetry line writer and target lifecycle ledger."""

    def _write(self, level, event, **fields):
        # Earlier event records omitted simulated time, making diagnostic
        # traces appear at t=0 despite their wall-clock timestamps. Preserve
        # an explicitly supplied value and stamp every other event here.
        active_failure = getattr(self, "active_failure", None)
        if isinstance(active_failure, dict):
            fields.setdefault("failure_id", active_failure.get("failure_id"))
        fields.setdefault("ros_time", round(rospy.Time.now().to_sec(), 3))
        payload = json.dumps(fields, sort_keys=True, separators=(",", ":"), default=str)
        line = "%s level=%s process=%s event=%s data=%s\n" % (
            datetime.datetime.now().isoformat(timespec="milliseconds"),
            level,
            self.process_name,
            event,
            payload,
        )
        try:
            self.stream.write(line)
        except (AttributeError, ValueError):
            pass

    @staticmethod
    def _target_session_key(payload):
        """Return the only identity allowed for target lifecycle latency."""
        task_id = str(payload.get("task_id", "")).strip()
        target_track_id = str(payload.get("target_track_id", "")).strip()
        if not task_id or not target_track_id:
            return None
        return (task_id, target_track_id)

    def _record_target_lifecycle_locked(self, event, payload):
        """Persist identity-bound target lifecycle evidence.

        ``/lste/task_done`` is a Bool and therefore cannot identify a target.
        GoalManager's arbitration stream contains the task/track pair, so only
        that stream may contribute to end-to-end target metrics.
        """
        field_by_event = {
            "target_track_started": "first_seen_ros",
            "target_follow_confirmed": "follow_confirmed_ros",
            "target_approach_terminal": "approach_terminal_ros",
            "target_close_confirmation_started": "close_started_ros",
            "target_close_confirmed": "close_confirmed_ros",
            "target_task_completed": "task_completed_ros",
        }
        timestamp_field = field_by_event.get(event)
        if timestamp_field is None:
            return
        key = self._target_session_key(payload)
        if key is None:
            self._write(
                "WARN",
                "target_lifecycle_unattributed",
                lifecycle_event=event,
                task_id=payload.get("task_id"),
                target_track_id=payload.get("target_track_id"),
            )
            return
        now = rospy.Time.now().to_sec()
        session = self.target_lifecycle_sessions.setdefault(
            key,
            {
                "task_id": key[0],
                "target_track_id": key[1],
                "first_seen_ros": None,
                "follow_confirmed_ros": None,
                "approach_terminal_ros": None,
                "close_started_ros": None,
                "close_confirmed_ros": None,
                "task_completed_ros": None,
            },
        )
        if session[timestamp_field] is None:
            session[timestamp_field] = now
        session["last_event_ros"] = now
        if event == "target_task_completed":
            self.target_lifecycle_last_completed_key = key
        record = dict(payload)
        record.pop("event", None)
        record.pop("task_id", None)
        record.pop("target_track_id", None)
        self._write(
            "INFO",
            "target_lifecycle",
            lifecycle_event=event,
            task_id=key[0],
            target_track_id=key[1],
            session_started_seconds=(
                None
                if session["first_seen_ros"] is None
                else round(session["first_seen_ros"] - self.start_ros, 3)
            ),
            event_seconds=round(now - self.start_ros, 3),
            **record
        )

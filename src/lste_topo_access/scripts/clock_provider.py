#!/usr/bin/env python3
"""Clock selection shared by lifecycle and map-age logic."""

import math
import time

try:
    import rospy
except ImportError:  # pragma: no cover - pure Python tooling fallback
    rospy = None


class ClockProvider:
    """Return ROS simulation time when live, otherwise an injected fallback."""

    def __init__(
        self,
        fallback=None,
        ros_initialized=None,
        ros_shutdown=None,
        ros_now=None,
    ):
        self._fallback = fallback or time.monotonic
        self._ros_initialized = ros_initialized or self._default_ros_initialized
        self._ros_shutdown = ros_shutdown or self._default_ros_shutdown
        self._ros_now = ros_now or self._default_ros_now

    @staticmethod
    def _default_ros_initialized():
        if rospy is None:
            return False
        try:
            # ``rostime.is_initialized()`` remains false while Gazebo is
            # paused at sim time zero. The ROS node is nevertheless live and
            # zero is the correct simulation timestamp, not a fallback cue.
            return bool(rospy.core.is_initialized())
        except Exception:
            try:
                return bool(rospy.rostime.is_initialized())
            except Exception:
                return False

    @staticmethod
    def _default_ros_shutdown():
        if rospy is None:
            return True
        try:
            return bool(rospy.is_shutdown())
        except Exception:
            return True

    @staticmethod
    def _default_ros_now():
        return float(rospy.Time.now().to_sec())

    def now(self):
        if self._ros_initialized() and not self._ros_shutdown():
            try:
                value = float(self._ros_now())
                if math.isfinite(value):
                    return value
            except Exception:
                pass
        return float(self._fallback())

    __call__ = now


def now_for(owner):
    """Resolve the lifecycle clock for a mixed-in runtime owner."""
    manager = getattr(owner, "lifecycle_manager", None)
    if manager is not None:
        method = getattr(manager, "now", None)
        if callable(method):
            return float(method())
    provider = getattr(owner, "time_provider", None)
    if provider is not None:
        method = provider if callable(provider) else getattr(provider, "now", None)
        if callable(method):
            return float(method())
    return ClockProvider().now()


__all__ = ["ClockProvider", "now_for"]

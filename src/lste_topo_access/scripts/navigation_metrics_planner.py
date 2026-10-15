"""Facade for planner telemetry in the LSTE navigation observer.

TEB feedback, move_base lifecycle feedback, and path analysis have independent
event streams.  Keep their implementation in sibling modules while preserving
the original mixin imported by the observer composition root.
"""

from navigation_metrics_planner_move_base import NavigationMetricsMoveBaseMixin
from navigation_metrics_planner_paths import NavigationMetricsPathMixin
from navigation_metrics_planner_teb_feedback import NavigationMetricsTebFeedbackMixin


class NavigationMetricsPlannerMixin(
    NavigationMetricsPathMixin,
    NavigationMetricsMoveBaseMixin,
    NavigationMetricsTebFeedbackMixin,
):
    """Combine read-only planner telemetry concerns for the observer node."""


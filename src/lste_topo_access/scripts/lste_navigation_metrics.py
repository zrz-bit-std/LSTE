#!/usr/bin/env python3
"""Passive, structured navigation telemetry for a live LSTE run.

This executable deliberately has no navigation policy.  It initializes the
ROS node, composes narrowly scoped observer modules, and lets those modules
record the goal, controller, planner, and perception evidence needed to audit
one run.
"""

import sys
from pathlib import Path

import rospy


# ``rosrun`` may execute a catkin-generated relay from ``devel/lib``.  Add the
# source/install sibling directory explicitly so the composed modules resolve
# in both devel-space and install-space launches.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from navigation_metrics_action_lifecycle import NavigationMetricsActionLifecycleMixin
from navigation_metrics_benchmark_truth import NavigationMetricsBenchmarkTruthMixin
from navigation_metrics_callbacks import NavigationMetricsCallbacksMixin
from navigation_metrics_command_quality import NavigationMetricsCommandQualityMixin
from navigation_metrics_detection import NavigationMetricsDetectionMixin
from navigation_metrics_execution_events import NavigationMetricsExecutionEventsMixin
from navigation_metrics_failure_evidence import NavigationMetricsFailureEvidenceMixin
from navigation_metrics_goal_events import NavigationMetricsGoalEventsMixin
from navigation_metrics_logging import NavigationMetricsLoggingMixin
from navigation_metrics_observer_state import NavigationMetricsObserverStateMixin
from navigation_metrics_planner import NavigationMetricsPlannerMixin
from navigation_metrics_pose import NavigationMetricsPoseMixin
from navigation_metrics_ros_interfaces import NavigationMetricsRosInterfacesMixin
from navigation_metrics_runtime import NavigationMetricsRuntimeMixin
from navigation_metrics_setup import NavigationMetricsSetupMixin
from navigation_metrics_snapshot import NavigationMetricsSnapshotMixin
from navigation_metrics_target_eval import NavigationMetricsTargetEvaluationMixin
from navigation_metrics_target_state import NavigationMetricsTargetStateMixin


class NavigationMetrics(
    NavigationMetricsSnapshotMixin,
    NavigationMetricsBenchmarkTruthMixin,
    NavigationMetricsCallbacksMixin,
    NavigationMetricsRosInterfacesMixin,
    NavigationMetricsCommandQualityMixin,
    NavigationMetricsActionLifecycleMixin,
    NavigationMetricsExecutionEventsMixin,
    NavigationMetricsFailureEvidenceMixin,
    NavigationMetricsPlannerMixin,
    NavigationMetricsTargetEvaluationMixin,
    NavigationMetricsDetectionMixin,
    NavigationMetricsGoalEventsMixin,
    NavigationMetricsLoggingMixin,
    NavigationMetricsPoseMixin,
    NavigationMetricsObserverStateMixin,
    NavigationMetricsTargetStateMixin,
    NavigationMetricsSetupMixin,
    NavigationMetricsRuntimeMixin,
):
    """Composition root for the read-only navigation metrics node."""

    def __init__(self):
        self._initialize_metrics()


def main():
    NavigationMetrics()
    rospy.spin()


if __name__ == "__main__":
    main()

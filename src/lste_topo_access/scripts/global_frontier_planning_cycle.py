"""Composition point for one online-frontier planning cycle.

The timer runtime, snapshot construction, and terminal successor policy are
separate so a change to selection behavior cannot obscure the cycle's error
boundary or the data used to create a planning snapshot.
"""

from global_frontier_planning_runtime import GlobalFrontierPlanningRuntimeMixin
from global_frontier_planning_selection import GlobalFrontierPlanningSelectionMixin
from global_frontier_planning_snapshot import GlobalFrontierPlanningSnapshotMixin


class GlobalFrontierPlanningCycleMixin(
    GlobalFrontierPlanningRuntimeMixin,
    GlobalFrontierPlanningSnapshotMixin,
    GlobalFrontierPlanningSelectionMixin,
):
    """Compose runtime, snapshot, and successor-selection behavior."""


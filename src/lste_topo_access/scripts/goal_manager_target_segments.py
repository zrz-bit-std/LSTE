"""Compatibility facade for the visual-target segment policy modules.

Callers continue to import :class:`GoalManagerTargetSegmentsMixin`; the
implementation is intentionally divided by route policy, commit transaction,
and frontier-recovery responsibility.
"""

from goal_manager_target_recovery import GoalManagerTargetRecoveryMixin
from goal_manager_target_route_continuity import GoalManagerTargetRouteContinuityMixin
from goal_manager_target_segment_commit import GoalManagerTargetSegmentCommitMixin
from goal_manager_target_utils import wrap_angle
from goal_manager_viewpoint import select_target_viewpoint


class GoalManagerTargetSegmentsMixin(
    GoalManagerTargetSegmentCommitMixin,
    GoalManagerTargetRecoveryMixin,
    GoalManagerTargetRouteContinuityMixin,
):
    """Compose the visual-target segment policy without changing its API."""

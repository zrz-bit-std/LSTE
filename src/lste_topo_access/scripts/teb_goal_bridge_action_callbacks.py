"""Compatibility façade for MoveBase callbacks in the TEB goal bridge."""

from teb_goal_bridge_action_feedback import TebGoalBridgeActionFeedbackMixin
from teb_goal_bridge_action_terminal import TebGoalBridgeActionTerminalMixin


class TebGoalBridgeActionCallbacksMixin(
    TebGoalBridgeActionTerminalMixin,
    TebGoalBridgeActionFeedbackMixin,
):
    """Expose the established callback mixin while keeping lifecycles focused."""

    pass

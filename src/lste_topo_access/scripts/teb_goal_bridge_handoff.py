"""Route-handoff composition for the TEB action bridge."""

from teb_goal_bridge_handoff_segment import TebGoalBridgeHandoffSegmentMixin
from teb_goal_bridge_handoff_stall import TebGoalBridgeHandoffStallMixin


class TebGoalBridgeHandoffMixin(
    TebGoalBridgeHandoffStallMixin,
    TebGoalBridgeHandoffSegmentMixin,
):
    """Assemble stale-action ownership and safe segment replacement policy."""


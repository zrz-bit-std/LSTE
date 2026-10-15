"""Mission lifecycle composition for the TEB action bridge.

The public mixin name remains stable for the node composition. Each incoming
event family is implemented in a focused sibling so edits stay local.
"""

from teb_goal_bridge_mission_control import TebGoalBridgeMissionControlMixin
from teb_goal_bridge_mission_input import TebGoalBridgeMissionInputMixin
from teb_goal_bridge_mission_runtime import TebGoalBridgeMissionRuntimeMixin
from teb_goal_bridge_persistent_target_result import (
    TebGoalBridgePersistentTargetResultMixin,
)


class TebGoalBridgeMissionLifecycleMixin(
    TebGoalBridgeMissionInputMixin,
    TebGoalBridgeMissionRuntimeMixin,
    TebGoalBridgeMissionControlMixin,
    TebGoalBridgePersistentTargetResultMixin,
):
    """Assemble mission input, runtime, control, and target-result callbacks."""


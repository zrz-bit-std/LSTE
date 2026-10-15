"""Facade for the legacy GP frontier topology layer.

The comparison node keeps its original mixin name, while the individual
topology concerns live in small modules that can be edited independently.
"""

from gp_frontier_anchor_graph import GpFrontierAnchorGraphMixin
from gp_frontier_backtrack import GpFrontierBacktrackMixin
from gp_frontier_forced_branch import GpFrontierForcedBranchMixin
from gp_frontier_junction_logging import GpFrontierJunctionLoggingMixin


class GpFrontierTopologyMixin(
    GpFrontierBacktrackMixin,
    GpFrontierForcedBranchMixin,
    GpFrontierAnchorGraphMixin,
    GpFrontierJunctionLoggingMixin,
):
    """Compose anchor memory, diagnostics, return routing, and commitment."""

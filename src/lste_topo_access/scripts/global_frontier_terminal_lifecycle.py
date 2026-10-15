"""Facade for the successful-action terminal lifecycle.

The terminal transition is split by responsibility. Keeping this facade
preserves the explorer's public composition point while making each part small
enough to review and edit independently.
"""

from global_frontier_terminal_prefetch import GlobalFrontierTerminalPrefetchMixin
from global_frontier_terminal_recording import GlobalFrontierTerminalRecordingMixin
from global_frontier_terminal_replan import GlobalFrontierTerminalReplanMixin
from global_frontier_terminal_validation import GlobalFrontierTerminalValidationMixin
from global_frontier_terminal_connector import GlobalFrontierTerminalConnectorMixin


class GlobalFrontierTerminalLifecycleMixin(
    GlobalFrontierTerminalConnectorMixin,
    GlobalFrontierTerminalReplanMixin,
    GlobalFrontierTerminalRecordingMixin,
    GlobalFrontierTerminalValidationMixin,
    GlobalFrontierTerminalPrefetchMixin,
):
    """Compose terminal validation, evidence recording, and replanning."""

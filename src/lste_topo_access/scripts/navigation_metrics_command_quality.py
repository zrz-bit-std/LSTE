"""Compatibility facade for read-only command-stream navigation telemetry.

The concrete policies live in focused sibling modules.  Keep this name stable
because the composition root imports it directly and downstream launch setups
may import it as well.
"""

from navigation_metrics_command_events import NavigationMetricsCommandEventsMixin
from navigation_metrics_command_stream import NavigationMetricsCommandStreamMixin


class NavigationMetricsCommandQualityMixin(
    NavigationMetricsCommandStreamMixin,
    NavigationMetricsCommandEventsMixin,
):
    """Compose command event classification with final-command accounting."""


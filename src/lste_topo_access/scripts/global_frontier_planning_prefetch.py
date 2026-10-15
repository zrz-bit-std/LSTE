"""Compatibility façade for the global-frontier successor cache lifecycle."""

from global_frontier_prefetch_activation import GlobalFrontierPrefetchActivationMixin
from global_frontier_prefetch_selection import GlobalFrontierPrefetchSelectionMixin
from global_frontier_prefetch_validation import GlobalFrontierPrefetchValidationMixin


class GlobalFrontierPlanningPrefetchMixin(
    GlobalFrontierPrefetchSelectionMixin,
    GlobalFrontierPrefetchValidationMixin,
    GlobalFrontierPrefetchActivationMixin,
):
    """Retain the established planning API while separating cache stages."""

    pass

"""Composition facade for simulator-only navigation target evaluation.

The public mixin name is intentionally stable.  Each evaluator concern lives
in a focused sibling module so geometry, depth decoding, and episode metrics
can change independently without growing one opaque observer implementation.
"""

from navigation_metrics_target_eval_configuration import (
    NavigationMetricsTargetEvaluationConfigurationMixin,
)
from navigation_metrics_target_eval_depth import NavigationMetricsTargetEvaluationDepthMixin
from navigation_metrics_target_eval_episodes import NavigationMetricsTargetEvaluationEpisodesMixin
from navigation_metrics_target_eval_projection import NavigationMetricsTargetEvaluationProjectionMixin


class NavigationMetricsTargetEvaluationMixin(
    NavigationMetricsTargetEvaluationConfigurationMixin,
    NavigationMetricsTargetEvaluationProjectionMixin,
    NavigationMetricsTargetEvaluationDepthMixin,
    NavigationMetricsTargetEvaluationEpisodesMixin,
):
    """Stable public surface for Gazebo-only target-evaluation helpers."""


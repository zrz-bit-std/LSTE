"""Evidence-preserving target post-processing contract.

Detector post-processing is allowed to annotate or rank a candidate, but it
must not erase the only observation of a small target before the temporal and
geometric Goal Manager can validate it.  This helper deliberately has no ROS
or model dependency.
"""


def preserve_raw_target_evidence(
    raw_boxes, raw_scores, raw_labels, accepted_boxes, accepted_scores,
    accepted_labels,
):
    """Restore raw target candidates only when post-processing removed all.

    The model's score threshold has already been applied upstream.  A raw
    candidate is therefore an observation hypothesis, not a completion claim.
    Goal Manager still requires multi-view/static-target evidence before it can
    own motion.  Returning the raw result in this one case prevents colour or
    crop validation from turning an informative frame into an indistinguishable
    no-target frame.
    """
    raw_count = len(raw_boxes) if raw_boxes is not None else 0
    accepted_count = len(accepted_boxes) if accepted_boxes is not None else 0
    if raw_count > 0 and accepted_count == 0:
        return (
            raw_boxes,
            raw_scores,
            raw_labels,
            True,
            "postprocess_removed_all_target_candidates",
        )
    return accepted_boxes, accepted_scores, accepted_labels, False, "accepted"


__all__ = ["preserve_raw_target_evidence"]

"""Stable semantic identity for one visual target track.

Detector labels are observations, not object identities.  A locked track may
accept a harmless synonym (``mug``/``cup``), but it must reject a different
attribute or object class.  Keeping this rule in a ROS-free module prevents
each callback from inventing a different fallback policy.
"""

import re
from dataclasses import dataclass


_TOKEN_RE = re.compile(r"[a-z0-9]+")
_ALIASES = {
    "mug": "cup",
    "mugs": "cup",
    "cups": "cup",
    "cup": "cup",
    "monitor": "monitor",
    "monitors": "monitor",
    "screen": "monitor",
    "screens": "monitor",
    "display": "monitor",
    "computer": "monitor",
    "table": "desk",
    "tables": "desk",
    "desk": "desk",
    "desks": "desk",
}
_STOPWORDS = frozenset(("a", "an", "the", "with", "and", "of"))
_ATTRIBUTE_TERMS = frozenset(
    (
        "red", "orange", "yellow", "green", "blue", "purple", "pink",
        "black", "white", "gray", "grey", "brown", "small", "medium",
        "large", "big", "wooden", "metal", "open", "closed",
    )
)


def semantic_tokens(label):
    """Normalize a detector label into comparable concept tokens."""
    if label is None:
        return frozenset()
    return frozenset(
        _ALIASES.get(token, token)
        for token in _TOKEN_RE.findall(str(label).lower().replace("-", " "))
        if token not in _STOPWORDS
    )


def canonical_label(label):
    """Return a stable, human-readable label for a track identity."""
    return " ".join(sorted(semantic_tokens(label)))


def labels_compatible(track_label, observed_label):
    """Whether an observation can update an already-locked target track.

    Object concepts must agree.  Attributes are optional, but an attribute
    explicitly present on both labels may not conflict.  This accepts
    ``yellow mug`` for ``yellow cup`` and rejects ``blue cup`` or ``yellow
    monitor`` without consulting detector confidence.
    """
    track = semantic_tokens(track_label)
    observed = semantic_tokens(observed_label)
    if not track or not observed:
        return False
    track_objects = track - _ATTRIBUTE_TERMS
    observed_objects = observed - _ATTRIBUTE_TERMS
    if track_objects and observed_objects and not (
        track_objects <= observed_objects or observed_objects <= track_objects
    ):
        return False
    for attribute in _ATTRIBUTE_TERMS:
        if attribute in track and attribute not in observed:
            # A missing attribute is an unknown, not a contradiction.
            continue
        if attribute in track and attribute in observed:
            continue
    track_attributes = track & _ATTRIBUTE_TERMS
    observed_attributes = observed & _ATTRIBUTE_TERMS
    # Terms from one mutually-exclusive attribute family cannot disagree.
    for family in (
        frozenset(("red", "orange", "yellow", "green", "blue", "purple", "pink", "black", "white", "gray", "grey", "brown")),
        frozenset(("small", "medium", "large", "big")),
    ):
        left = track_attributes & family
        right = observed_attributes & family
        if left and right and left.isdisjoint(right):
            return False
    return bool(track_objects & observed_objects) or not track_objects or not observed_objects


@dataclass(frozen=True)
class TargetTrackIdentity:
    """Immutable semantic key used by target evidence and navigation."""

    canonical: str
    tokens: frozenset

    @classmethod
    def from_label(cls, label):
        return cls(canonical_label(label), semantic_tokens(label))

    def accepts(self, observed_label):
        return labels_compatible(self.canonical, observed_label)


__all__ = [
    "TargetTrackIdentity",
    "canonical_label",
    "labels_compatible",
    "semantic_tokens",
]

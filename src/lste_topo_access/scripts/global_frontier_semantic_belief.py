"""Task-conditioned semantic belief for topological exploration.

The occupancy grid is a short-lived geometric observation.  This module keeps
the complementary long-lived question explicit: *which physical Place has
evidence relevant to the current task?*  It is deliberately ROS-free so the
state transition and the action policy can be tested without a running graph.

The policy is qualitative rather than a weighted score.  A Place is either
still in its bootstrap observation, has target evidence, has task-context
evidence, or should expand through a certified portal to seek an unobserved
Place.  This makes the semantic contribution an architectural decision and
keeps detector confidence and map resolution out of the exploration policy.
"""

import json
import re
from collections import Counter
from dataclasses import dataclass


_TOKEN_RE = re.compile(r"[a-z0-9]+")

# These aliases are intentionally small and domain-oriented.  Detection
# backends use labels such as ``mug`` and ``computer monitor`` while task JSON
# often says ``cup`` and ``monitor``; treating them as one concept avoids a
# backend-specific policy branch.
_ALIASES = {
    "mugs": "cup",
    "mug": "cup",
    "cups": "cup",
    "cup": "cup",
    "computer": "monitor",
    "display": "monitor",
    "screens": "monitor",
    "screen": "monitor",
    "monitors": "monitor",
    "monitor": "monitor",
    "tables": "desk",
    "table": "desk",
    "desks": "desk",
    "desk": "desk",
    "seats": "chair",
    "chairs": "chair",
    "chair": "chair",
    "keyboards": "keyboard",
    "keyboard": "keyboard",
    "folders": "folder",
    "folder": "folder",
    "cans": "can",
    "can": "can",
}

_STOPWORDS = frozenset(
    (
        "a",
        "an",
        "and",
        "at",
        "by",
        "for",
        "from",
        "in",
        "near",
        "of",
        "on",
        "or",
        "the",
        "to",
        "with",
    )
)


def semantic_tokens(value):
    """Return normalized concept tokens for one task or detector label."""
    if value is None:
        return frozenset()
    raw = str(value).strip().lower()
    if not raw:
        return frozenset()
    tokens = []
    for token in _TOKEN_RE.findall(raw.replace("-", " ")):
        if token in _STOPWORDS:
            continue
        tokens.append(_ALIASES.get(token, token))
    return frozenset(tokens)


def _terms(values):
    result = set()
    for value in values or ():
        result.update(semantic_tokens(value))
    return frozenset(result)


@dataclass(frozen=True)
class SemanticTaskIntent:
    """Immutable semantic contract derived from one task version."""

    task_id: str
    task_version: str
    target_terms: frozenset
    context_terms: frozenset
    negative_terms: frozenset

    @property
    def active(self):
        return bool(self.task_id and (self.target_terms or self.context_terms))


def task_intent_from_message(task, task_version=""):
    """Convert an ``LsteTask``-shaped object into a stable semantic intent."""
    if task is None:
        return SemanticTaskIntent("", "", frozenset(), frozenset(), frozenset())

    task_id = str(getattr(task, "task_id", "") or "").strip()
    target_name = getattr(task, "target_name", "")
    target_attributes = list(getattr(task, "target_attributes", []) or [])
    target_terms = set(semantic_tokens(target_name))
    target_terms.update(_terms(target_attributes))

    context_values = []
    context_values.extend(list(getattr(task, "env_related_structures", []) or []))
    context_values.extend(list(getattr(task, "env_type_prior", []) or []))
    context_values.extend(list(getattr(task, "obj_key_objects", []) or []))
    context_values.extend(
        value
        for value in (
            getattr(task, "ctx_left", ""),
            getattr(task, "ctx_right", ""),
        )
        if value
    )
    # The ROS message keeps the complete task JSON for provenance. Use the
    # richer, optional priors when present without changing the public message
    # definition (typical locations, layout prior, and search strategy).
    raw_json = str(getattr(task, "raw_json", "") or "").strip()
    if raw_json:
        try:
            parsed = json.loads(raw_json)
            parsed = parsed.get("task_parsed", parsed)
            target = parsed.get("target", {}) if isinstance(parsed, dict) else {}
            env = parsed.get("env", {}) if isinstance(parsed, dict) else {}
            related = parsed.get("obj_related", {}) if isinstance(parsed, dict) else {}
            context_values.extend(target.get("typical_locations", []) or [])
            context_values.extend(env.get("layout_prior", []) or [])
            if related.get("search_strategy"):
                context_values.append(related["search_strategy"])
        except (TypeError, ValueError, json.JSONDecodeError, AttributeError):
            # Structured fields above remain authoritative when provenance is
            # unavailable or the raw task was not valid JSON.
            pass
    context_terms = set(_terms(context_values))
    negative_terms = _terms(getattr(task, "obj_negative_clues", []) or [])
    # A negative clue must not accidentally become a positive context signal
    # when a task repeats the same noun in a different field.
    context_terms.difference_update(negative_terms)
    return SemanticTaskIntent(
        task_id=task_id,
        task_version=str(task_version or ""),
        target_terms=frozenset(target_terms),
        context_terms=frozenset(context_terms),
        negative_terms=negative_terms,
    )


def matched_terms(label, terms):
    """Return the task concepts supported by one detector label."""
    return frozenset(semantic_tokens(label) & set(terms or ()))


def semantic_place_action(
    intent, evidence, place_observed, local_work_pending=True,
):
    """Return the topology action class for one physical Place.

    This is a finite-state policy.  It has no distance, confidence, or time
    thresholds, and therefore cannot become another parameter-tuning loop.
    """
    if not intent.active or not place_observed:
        return "bootstrap_observation"
    if evidence is None:
        evidence = {}
    if int(evidence.get("target_hits", 0)) > 0:
        return "target_place"
    # Generic office context (desk/monitor/chair) is a weak prior shared by
    # many rooms. It may guide the first observation, but after that first
    # evidence-bearing pass it must not pin the graph to the room forever.
    # ``observations == 1`` is a lifecycle fact, not a tuned confidence or
    # distance threshold. Only task-specific target evidence can keep a
    # covered Place as the semantic owner thereafter.
    if (
        int(evidence.get("context_hits", 0)) > 0
        and local_work_pending
        and int(evidence.get("observations", 0)) <= 1
    ):
        return "context_place"
    return "expand_unobserved_portal"


class SemanticPlaceBelief:
    """Per-task semantic evidence indexed by durable physical Place ID."""

    def __init__(self, intent=None):
        self.intent = intent or SemanticTaskIntent(
            "", "", frozenset(), frozenset(), frozenset()
        )
        self._places = {}

    def reset(self, intent):
        """Start a new task version without carrying semantic evidence over."""
        self.intent = intent or SemanticTaskIntent(
            "", "", frozenset(), frozenset(), frozenset()
        )
        self._places = {}

    def observe(self, place_id, labels, now=0.0, target_labels=None):
        """Accumulate labels observed while physically inside ``place_id``."""
        try:
            place_id = int(place_id)
        except (TypeError, ValueError):
            return None
        if place_id <= 0:
            return None
        record = self._places.setdefault(
            place_id,
            {
                "place_id": place_id,
                "observations": 0,
                "context_hits": 0,
                "target_hits": 0,
                "negative_hits": 0,
                "context_terms": Counter(),
                "target_terms": Counter(),
                "negative_terms": Counter(),
                "labels": Counter(),
                "last_observed_at": None,
            },
        )
        target_label_texts = {
            str(label or "").strip().lower() for label in (target_labels or ())
        }
        for label in tuple(labels or ()):
            text = str(label or "").strip()
            if not text:
                continue
            record["labels"][text.lower()] += 1
            target = text.lower() in target_label_texts
            context_matches = matched_terms(text, self.intent.context_terms)
            target_matches = matched_terms(
                text,
                self.intent.target_terms,
            )
            negative_matches = matched_terms(
                text,
                self.intent.negative_terms,
            )
            if context_matches:
                record["context_hits"] += 1
                for term in context_matches:
                    record["context_terms"][term] += 1
            # ``LsteDetections`` already separates target_dets from env_dets.
            # Preserve that contract: an environment label such as "other
            # cups" must not claim the target merely because it shares the
            # noun "cup" with the task.
            if target:
                record["target_hits"] += 1
                for term in target_matches:
                    record["target_terms"][term] += 1
            if negative_matches:
                record["negative_hits"] += 1
                for term in negative_matches:
                    record["negative_terms"][term] += 1
        record["observations"] += 1
        record["last_observed_at"] = float(now)
        return self.evidence(place_id)

    def evidence(self, place_id):
        """Return JSON-safe evidence for one Place."""
        try:
            place_id = int(place_id)
        except (TypeError, ValueError):
            return None
        record = self._places.get(place_id)
        if record is None:
            return None
        return {
            "place_id": int(record["place_id"]),
            "observations": int(record["observations"]),
            "context_hits": int(record["context_hits"]),
            "target_hits": int(record["target_hits"]),
            "negative_hits": int(record["negative_hits"]),
            "context_terms": sorted(record["context_terms"]),
            "target_terms": sorted(record["target_terms"]),
            "negative_terms": sorted(record["negative_terms"]),
            "labels": sorted(record["labels"]),
            "last_observed_at": record["last_observed_at"],
        }

    def all_evidence(self):
        """Return a copy suitable for diagnostics and experiment summaries."""
        return {
            int(place_id): self.evidence(place_id)
            for place_id in sorted(self._places)
        }

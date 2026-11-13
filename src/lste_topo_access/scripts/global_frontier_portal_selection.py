"""Explicit portal action selection for the online place graph."""

import math

from global_frontier_portal_admission import GlobalFrontierPortalAdmissionMixin
from global_frontier_graph_executive import choose_graph_action
from global_frontier_portal_decision import (
    PortalDecisionCandidate,
    PortalDecisionEvidence,
    portal_action_category,
    select_portal_transition as select_evidence_portal_transition,
)
from global_frontier_topology import semantic_hint_is_forward


class GlobalFrontierPortalSelectionMixin(GlobalFrontierPortalAdmissionMixin):
    """Choose a validated doorway action from the Place graph.

    Strict methods call this after local work is clear. The branch-first method
    may call it earlier for a certified edge to an unobserved Place; the source
    Place is then suspended so its WorkItems remain resumable.
    """

    def _refresh_or_certify_portal_hypothesis(
        self, request, source, portal_gate_xy, destination_xy, *, create=True,
    ):
        """Keep a durable Portal ID when its map projection is rehydrated.

        ``create=False`` is used while a snapshot-local candidate is still
        passing physical admission.  A rejected covered-room candidate must
        not leave a durable Portal/probe behind as a side effect of merely
        being inspected.
        """
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        source_place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or source_place_id is None:
            return None
        map_to_physical = request.map_to_physical_xy
        physical_gate = (
            portal_gate_xy
            if map_to_physical is None
            else map_to_physical(portal_gate_xy[0], portal_gate_xy[1])
        )
        physical_destination = (
            destination_xy
            if map_to_physical is None
            else map_to_physical(destination_xy[0], destination_xy[1])
        )
        preferred_portal_id = getattr(self, "graph_route_portal_id", None)
        try:
            preferred_portal_id = (
                None if preferred_portal_id is None else int(preferred_portal_id)
            )
        except (TypeError, ValueError):
            preferred_portal_id = None
        # A crossed Portal defines a durable doorway band.  If the current
        # Place is that edge's destination, a fresh structural candidate at
        # the same physical gate is the reverse/transit side of the doorway,
        # even when SLAM chose the opposite jamb or flipped its normal.  Reject
        # it before certification so it cannot become a Place->same-Place
        # self-loop.
        crossed_gate_query = getattr(
            ledger, "crossed_gate_for_destination", None,
        )
        if callable(crossed_gate_query) and physical_gate is not None:
            memory = getattr(self, "region_memory", None)
            gate_radius = getattr(
                memory, "portal_entry_match_radius", None,
            )
            candidate_direction = (
                None
                if physical_destination is None
                else (
                    float(physical_destination[0]) - float(physical_gate[0]),
                    float(physical_destination[1]) - float(physical_gate[1]),
                )
            )
            try:
                crossed_gate = crossed_gate_query(
                    source_place_id,
                    physical_gate,
                    gate_radius=gate_radius,
                    candidate_direction_xy=candidate_direction,
                )
            except TypeError:
                # Keep injected ledgers with the pre-directional method
                # signature source-compatible while production uses the
                # signed wide-gate admission.
                crossed_gate = crossed_gate_query(
                    source_place_id,
                    physical_gate,
                    gate_radius=gate_radius,
                )
            if crossed_gate is not None:
                reports = getattr(
                    self, "_portal_hypothesis_suppression_reports", None,
                )
                if reports is None:
                    reports = set()
                    self._portal_hypothesis_suppression_reports = reports
                signature = (
                    int(source_place_id),
                    int(crossed_gate["id"]),
                )
                if signature not in reports:
                    reports.add(signature)
                    self.publish_status(
                        "portal_hypothesis_suppressed",
                        source_place_id=int(source_place_id),
                        portal_id=int(crossed_gate["id"]),
                        gate=[
                            round(float(physical_gate[0]), 3),
                            round(float(physical_gate[1]), 3),
                        ],
                        reason=(
                            "same_wide_gate_as_crossed_portal_is_transit"
                        ),
                    )
                return None
            # A Place identity can split at a doorway after a loop closure,
            # so the directional destination query above may not know that
            # this is the same already-crossed physical gate.  Use the strict
            # physical gate band as a canonical duplicate check, but preserve
            # an explicitly selected crossed graph edge for rehydration.
            crossed_at_gate = getattr(
                ledger, "crossed_gate_at_physical_gate", None,
            )
            if callable(crossed_at_gate) and physical_gate is not None:
                crossed_gate = crossed_at_gate(physical_gate)
                if crossed_gate is not None:
                    try:
                        preferred_matches = (
                            preferred_portal_id is not None
                            and int(crossed_gate.get("id", -1))
                            == int(preferred_portal_id)
                        )
                    except (TypeError, ValueError, AttributeError):
                        preferred_matches = False
                    if not preferred_matches:
                        reports = getattr(
                            self, "_portal_hypothesis_suppression_reports", None,
                        )
                        if reports is None:
                            reports = set()
                            self._portal_hypothesis_suppression_reports = reports
                        signature = (
                            int(source_place_id),
                            int(crossed_gate["id"]),
                        )
                        if signature not in reports:
                            reports.add(signature)
                            self.publish_status(
                                "portal_hypothesis_suppressed",
                                source_place_id=int(source_place_id),
                                portal_id=int(crossed_gate["id"]),
                                gate=[
                                    round(float(physical_gate[0]), 3),
                                    round(float(physical_gate[1]), 3),
                                ],
                                reason="same_physical_gate_as_crossed_portal",
                            )
                        return None
        hypothesis_id = getattr(source, "hypothesis_id", None)
        if hypothesis_id is not None:
            hypothesis = ledger.get(hypothesis_id)
            if hypothesis is None:
                return None
            try:
                valid_source = int(hypothesis["source_place_id"]) == int(
                    source_place_id
                )
            except (KeyError, TypeError, ValueError):
                valid_source = False
            bound_state = str(hypothesis.get("state", "")).strip().lower()
            allow_bound_graph_edge = (
                preferred_portal_id is not None
                and int(hypothesis.get("id", -1)) == preferred_portal_id
                and bound_state == "crossed"
            )
            # A bound doorway that this snapshot deliberately re-exposed is a
            # legitimate exit even though the graph did not name it as the
            # preferred crossed edge. It still has to pass portal admission,
            # the graph action, and the physical route checks below; once it
            # is crossed the record is no longer re-exposed.
            allow_bound_portal_reuse = (
                bound_state in ("certified", "selected")
                and int(hypothesis.get("id", -1))
                in (getattr(self, "_bound_portal_source_ids", None) or ())
            )
            if (
                not valid_source
                or (
                    hypothesis.get("destination_place_id") is not None
                    and not (
                        allow_bound_graph_edge or allow_bound_portal_reuse
                    )
                )
                or bound_state not in ("certified", "selected", "crossed")
            ):
                return None
            refresh = getattr(ledger, "refresh_projection", None)
            if refresh is not None:
                refresh(
                    hypothesis["id"],
                    map_gate_xy=portal_gate_xy,
                    map_destination_xy=destination_xy,
                    now=request.now,
                    task_version=str(
                        getattr(self, "current_task_version", "") or ""
                    ),
                )
            self._ensure_portal_hypothesis_probe(
                request, hypothesis, portal_gate_xy, destination_xy,
            )
            return hypothesis

        # A map update can move the destination projection far enough that the
        # normal gate+destination matcher no longer recovers this crossed edge.
        # When the slow planner has selected a preferred physical identity,
        # reuse it after checking the source and gate evidence instead of
        # minting a replacement Portal.
        if preferred_portal_id is not None:
            preferred = ledger.get(preferred_portal_id)
            if isinstance(preferred, dict):
                try:
                    same_source = int(preferred.get("source_place_id")) == int(
                        source_place_id
                    )
                except (TypeError, ValueError):
                    same_source = False
                preferred_gate = preferred.get("physical_gate")
                if preferred_gate is None:
                    preferred_gate = preferred.get("map_gate")
                gate_for_match = physical_gate if physical_gate is not None else portal_gate_xy
                try:
                    gate_matches = (
                        preferred_gate is not None
                        and gate_for_match is not None
                        and math.hypot(
                            float(preferred_gate[0]) - float(gate_for_match[0]),
                            float(preferred_gate[1]) - float(gate_for_match[1]),
                        ) <= float(getattr(ledger, "match_radius", 0.30)) * 2.0
                    )
                except (IndexError, TypeError, ValueError):
                    gate_matches = False
                if same_source and gate_matches and str(
                    preferred.get("state", "")
                ).strip().lower() in ("certified", "selected", "crossed"):
                    self._ensure_portal_hypothesis_probe(
                        request, preferred, portal_gate_xy, destination_xy,
                    )
                    return preferred

        if not create:
            return None

        hypothesis, created = ledger.certify(
            source_place_id,
            physical_gate,
            physical_destination,
            now=request.now,
            task_version=str(getattr(self, "current_task_version", "") or ""),
            map_gate_xy=portal_gate_xy,
            map_destination_xy=destination_xy,
        )
        if hypothesis is None or str(
            hypothesis.get("state", "")
        ).strip().lower() == "failed":
            if hypothesis is not None:
                self.publish_status(
                    "portal_hypothesis_rejected",
                    portal_id=int(hypothesis["id"]),
                    source_place_id=int(source_place_id),
                    reason="portal_negative_evidence",
                )
            return None
        if created:
            self.publish_status(
                "portal_hypothesis_certified",
                portal_id=int(hypothesis["id"]),
                source_place_id=int(source_place_id),
                gate=[
                    round(float(portal_gate_xy[0]), 3),
                    round(float(portal_gate_xy[1]), 3),
                ],
                destination=[
                    round(float(destination_xy[0]), 3),
                    round(float(destination_xy[1]), 3),
                ],
            )
        self._ensure_portal_hypothesis_probe(
            request, hypothesis, portal_gate_xy, destination_xy,
        )
        return hypothesis

    def _ensure_portal_hypothesis_probe(
        self, request, hypothesis, portal_gate_xy, destination_xy,
    ):
        """Bind an unbound Portal hypothesis to one source-side probe."""
        if not isinstance(hypothesis, dict):
            return None
        if hypothesis.get("destination_place_id") is not None:
            return None
        ensure = getattr(self, "register_portal_hypothesis_probe", None)
        if ensure is None:
            return None
        return ensure(
            request,
            int(hypothesis["id"]),
            portal_gate_xy,
            destination_xy,
        )

    def _note_portal_deferral(
        self, portal_id, reason, now=None, heading_delta_deg=None, detail=None,
    ):
        """Persist one non-commit reason and return the event note tuple.

        The reason is written to the durable Portal ledger and mirrored onto
        the GP observation branch that owns the same doorway, so an observed
        room that was never committed keeps an explanation after the map and
        the route that produced it are gone.  The caller still appends the
        returned tuple to its per-cycle rejection list, so the status event and
        the durable record can never disagree.
        """
        try:
            portal_id = None if portal_id is None else int(portal_id)
        except (TypeError, ValueError):
            portal_id = None
        reason = str(reason or "").strip()
        if portal_id is not None and reason:
            ledger = getattr(self, "portal_hypothesis_ledger", None)
            mark = getattr(ledger, "mark_deferred", None)
            if callable(mark):
                mark(
                    portal_id,
                    reason,
                    now=0.0 if now is None else now,
                    heading_delta_deg=heading_delta_deg,
                    detail=detail,
                )
            self._mirror_portal_deferral_to_branch(
                portal_id, reason, now=now,
                heading_delta_deg=heading_delta_deg,
            )
        return (portal_id, reason)

    def _mirror_portal_deferral_to_branch(
        self, portal_id, reason, now=None, heading_delta_deg=None,
    ):
        """Copy the deferral onto the GP branch that owns this Portal."""
        tree = getattr(self, "gp_exploration_tree", None)
        branches = getattr(tree, "branches", None)
        if not branches:
            return
        try:
            portal_id = int(portal_id)
        except (TypeError, ValueError):
            return
        for branch in branches.values():
            try:
                if int(branch.get("canonical_portal_id", -1)) != portal_id:
                    continue
            except (TypeError, ValueError):
                continue
            branch["defer_reason"] = reason
            branch["defer_ros"] = 0.0 if now is None else float(now)
            branch["defer_count"] = int(branch.get("defer_count", 0)) + 1
            branch["defer_heading_delta_deg"] = heading_delta_deg

    def portal_turn_needs_positive_room(self, route_heading_delta, room_score):
        """Return True when a turn into a doorway lacks the evidence for it.

        Driving straight needs no evidence.  Turning beyond the soft limit into
        a room does: a negative or unknown doorway is skipped so the corridor
        ahead keeps being explored (env04 mid junction), while a room with at
        least one positive environment term may still be turned into.
        """
        try:
            delta = abs(float(route_heading_delta))
        except (TypeError, ValueError):
            return False
        try:
            soft_limit = math.radians(float(
                getattr(self, "room_turn_soft_limit_deg", 60.0) or 0.0))
        except (TypeError, ValueError):
            soft_limit = math.radians(60.0)
        if delta <= soft_limit:
            return False
        try:
            min_score = float(
                getattr(self, "heading_semantic_override_min_score", 0.15) or 0.0)
        except (TypeError, ValueError):
            min_score = 0.15
        if room_score is None:
            return True
        try:
            return float(room_score) < min_score
        except (TypeError, ValueError):
            return True

    def _portal_forward_alternatives(self, request, portal_sources):
        """Return ``(straight_available, positive_available)`` for this pass.

        The turn-evidence gate is relative: a negative doorway is skipped only
        when a better alternative exists at the same junction, either

        * another doorway that can be taken without a large turn (the corridor
          ahead is still usable), or
        * another doorway whose room already has positive environment evidence
          (env04 lower junction: east lab 0.182 vs west hydrant -0.69, both
          ~90 deg, so the straight rule alone would still enter the west room).

        When neither holds the gate yields.  Corridors themselves often score
        negative (a nearby door or fire hydrant), and refusing the last way
        forward parks the robot (run 20261113_181729 stopped at y=-3.98 with
        the only south doorway at -0.538).
        """
        try:
            soft_limit = math.radians(float(
                getattr(self, "room_turn_soft_limit_deg", 60.0) or 0.0))
        except (TypeError, ValueError):
            soft_limit = math.radians(60.0)
        try:
            min_score = float(getattr(
                self, "heading_semantic_override_min_score", 0.15) or 0.0)
        except (TypeError, ValueError):
            min_score = 0.15
        straight = False
        positive = False
        for source in portal_sources.values():
            try:
                endpoint_row = source.endpoint_row
                endpoint_col = source.endpoint_col
                portal_x, portal_y = self.cell_xy(
                    request.message, endpoint_row, endpoint_col,
                )
            except (AttributeError, TypeError, ValueError):
                continue
            if portal_x is None or portal_y is None:
                continue
            if request.excluded and any(
                    math.hypot(portal_x - old_x, portal_y - old_y)
                    < self.completed_radius
                    for old_x, old_y in request.excluded):
                continue
            try:
                if self.frontier_is_rejected(portal_x, portal_y, request.now):
                    continue
            except (AttributeError, TypeError, ValueError):
                continue
            if request.validation is not None:
                try:
                    if self.candidate_costmap_distance(
                            request.validation, portal_x, portal_y) is None:
                        continue
                except (AttributeError, TypeError, ValueError):
                    continue
            if not positive:
                ledger = getattr(self, "portal_hypothesis_ledger", None)
                hypothesis_key = getattr(source, "hypothesis_id", None)
                if ledger is not None and hypothesis_key is not None:
                    record = ledger.get(hypothesis_key) or {}
                    try:
                        score = float(record.get("semantic_score"))
                    except (TypeError, ValueError):
                        score = None
                    if (score is not None and math.isfinite(score)
                            and score >= min_score
                            and not record.get("exploration_suppressed")):
                        positive = True
            if not straight:
                try:
                    delta, _penalty = self._route_heading_penalty(
                        request,
                        (endpoint_row, endpoint_col),
                        (portal_x, portal_y),
                    )
                    delta = abs(float(delta))
                except (AttributeError, TypeError, ValueError):
                    delta = None
                if delta is not None and delta <= soft_limit:
                    straight = True
            if straight and positive:
                break
        return straight, positive

    def _retire_negative_room_entry(self, record, portal_id, room_score, now):
        """Retire a doorway that was inspected and still looks negative.

        Deferring is enough while the inside of the room is still unknown: the
        approach window may still reveal a chair or desk that turns the score
        positive.  Once the doorway authentication completed (the robot stood
        inside the doorway and looked around) and the score is still below the
        turn floor, the doorway is a deliberate policy skip.  Leaving its
        Portal-entry WorkItem unresolved would make the source Place look
        unfinished forever, so the DFS suspends and re-enters that Place in a
        loop (run 20261113_180426: place 3 every ~9 s, 160 identical notes).
        """
        if not isinstance(record, dict):
            return None
        if str(record.get("gp_authentication_state", "")).strip().lower() != (
                "completed"):
            return None
        try:
            portal_id = int(portal_id)
        except (TypeError, ValueError):
            return None
        record["exploration_suppressed"] = True
        record["policy_skip_reason"] = "negative_room_turn"
        record["policy_skip_ros"] = float(now)
        work = getattr(self, "place_work_items", None)
        abandon = getattr(work, "abandon_portal_entry", None)
        retired = None
        if callable(abandon):
            retired = abandon(
                portal_id, now, "negative_room_turn",
                resolution="policy_skipped_negative_room",
                item_id=record.get("semantic_entry_work_item_id"),
            )
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "portal_entry_policy_skipped",
                portal_id=portal_id,
                semantic_score=(
                    None if room_score is None else round(float(room_score), 4)
                ),
                entry_work_item_id=(
                    None if retired is None else retired.get("id")
                ),
                reason="negative_room_turn",
            )
        return retired

    def _portal_room_score_ranking(self, source_place_id):
        """Return the two highest certified room scores of one source Place."""
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if ledger is None or source_place_id is None:
            return None, None
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return None, None
        snapshot = getattr(ledger, "snapshot", None)
        if not callable(snapshot):
            return None, None
        scores = []
        for record in snapshot():
            if not isinstance(record, dict):
                continue
            try:
                if int(record.get("source_place_id", -1) or -1) != source_place_id:
                    continue
            except (TypeError, ValueError):
                continue
            if record.get("destination_place_id") is None:
                continue
            if record.get("exploration_suppressed"):
                continue
            try:
                score = float(record.get("semantic_score"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(score):
                scores.append(score)
        scores.sort(reverse=True)
        if not scores:
            return None, None
        return scores[0], (scores[1] if len(scores) > 1 else None)

    def _portal_semantic_heading_override(
        self, source, route_heading_delta, now=0.0, runner_up_score=None,
    ):
        """Authorize one high-value doorway to be entered after a turn.

        The heading bound exists so the forward-only controller is never asked
        to drive a route whose first tangent is behind the robot.  A doorway
        whose destination room already matches the task is worth the in-place
        turn that the turn supervisor performs for ``portal_transition``
        actions anyway (``pre_route_alignment`` above 75 degrees).

        Returns the durable Portal record when the override is authorized,
        otherwise ``None``.  The per-doorway attempt counter keeps a failed
        turn from being retried forever and is part of the audit record.
        """
        if not getattr(self, "heading_semantic_override_enabled", False):
            return None
        if not getattr(self, "task_semantic_value_enabled", False):
            return None
        try:
            max_attempts = int(
                getattr(self, "heading_semantic_override_max_attempts", 0)
            )
        except (TypeError, ValueError):
            max_attempts = 0
        if max_attempts <= 0:
            return None
        try:
            delta = abs(float(route_heading_delta))
        except (TypeError, ValueError):
            return None
        if delta > math.radians(180.5):
            return None
        portal_id = getattr(source, "hypothesis_id", None)
        if portal_id is None:
            return None
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        getter = getattr(ledger, "get", None)
        if not callable(getter):
            return None
        record = getter(portal_id)
        if not isinstance(record, dict):
            return None
        if record.get("destination_place_id") is None:
            return None
        try:
            score = float(record.get("semantic_score"))
        except (TypeError, ValueError):
            return None
        if not math.isfinite(score):
            return None
        try:
            min_score = float(
                getattr(self, "heading_semantic_override_min_score", 0.15)
            )
        except (TypeError, ValueError):
            min_score = 0.15
        if score < min_score:
            return None
        try:
            min_margin = float(
                getattr(self, "heading_semantic_override_min_margin", 0.40)
            )
        except (TypeError, ValueError):
            min_margin = 0.40
        try:
            runner_up = -1.0 if runner_up_score is None else float(runner_up_score)
        except (TypeError, ValueError):
            runner_up = -1.0
        if score - runner_up < min_margin:
            return None
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "portal_heading_margin",
                portal_id=int(portal_id),
                score=round(score, 4),
                runner_up=round(runner_up, 4),
                margin=round(score - runner_up, 4),
                required=round(min_margin, 4),
                attempt=int(record.get("heading_override_count", 0) or 0),
            )
        attempts = int(record.get("heading_override_count", 0) or 0)
        # One authorized turn needs several decision passes to materialize a
        # route (fresh map/TF, Navfn, graph transaction).  Counting every pass
        # burned both attempts in under a second in run 20261110_184912 and
        # withdrew the override before the route existed.  Hold the attempt
        # budget for a bounded window so the count measures real retries.
        try:
            hold = max(
                0.0,
                float(
                    getattr(
                        self,
                        "heading_semantic_override_hold_seconds",
                        8.0,
                    )
                ),
            )
        except (TypeError, ValueError):
            hold = 8.0
        last = record.get("heading_override_ros")
        try:
            last = None if last is None else float(last)
        except (TypeError, ValueError):
            last = None
        try:
            now = float(now)
        except (TypeError, ValueError):
            now = 0.0
        if now > 0.0 and last is not None and 0.0 <= now - last < hold:
            return record
        if attempts >= max_attempts:
            return None
        record["heading_override_count"] = attempts + 1
        record["heading_override_ros"] = now
        return record

    def _select_context_portal_transition(
        self, request, context, portal_sources,
    ):
        """Select a portal, deriving an adjacent edge when needed."""
        if (
            not request.allow_portal_transitions
            or request.allowed_region_tiers is not None
        ):
            return None
        self._populate_adjacent_portal_sources(
            request, context, portal_sources,
        )
        return self._select_portal_transition(request, portal_sources)

    def _select_portal_transition(self, request, portal_sources):
        """Choose one safe doorway action when ordinary work is exhausted."""
        portal_best = None
        portal_best_score = -float("inf")
        portal_best_hypothesis = None
        portal_best_class = None
        portal_class_rank = {"covered_transit": 0, "new_place": 1}
        evidence_candidates = []
        evidence_routes = {}
        # The full graph method uses a discrete, task-conditioned decision
        # layer. Baseline arms retain their historical score policy so the
        # benchmark isolates the architectural contribution.
        use_evidence_decision = bool(
            getattr(self, "task_semantic_value_enabled", False)
        )
        self.last_portal_sources = len(portal_sources)
        self.last_portal_candidates_seen = len(portal_sources)
        self.last_portal_executable_candidates = 0
        self.last_portal_hard_rejected = 0

        rejected_notes = []
        # Lazily computed: (straight, positive) alternatives among the other
        # candidates.  A negative side room is only skipped when one exists.
        forward_alternatives = None
        # Same-junction score ranking, used by the "clearly better" heading
        # override.  A room that only wins a tie is not worth a U-turn.
        room_best, room_second = self._portal_room_score_ranking(
            getattr(self, "current_physical_place_id", None))
        preferred_portal_id = getattr(self, "graph_route_portal_id", None)
        try:
            preferred_portal_id = (
                None if preferred_portal_id is None else int(preferred_portal_id)
            )
        except (TypeError, ValueError):
            preferred_portal_id = None

        for source in portal_sources.values():
            # Candidate collection must be observational until the graph-owned
            # identity is known to match.  In particular, do not certify or
            # bind an unrelated doorway while a durable Portal action is
            # waiting for projection/materialization.
            if preferred_portal_id is not None:
                try:
                    source_portal_id = getattr(source, "hypothesis_id", None)
                    source_portal_id = (
                        None
                        if source_portal_id is None
                        else int(source_portal_id)
                    )
                except (TypeError, ValueError):
                    source_portal_id = None
                if source_portal_id != preferred_portal_id:
                    rejected_notes.append(self._note_portal_deferral(
                        source_portal_id, "not_preferred_portal", request.now,
                    ))
                    continue
            frontier_row, frontier_col = source.frontier_row, source.frontier_col
            # Keep the labelled core for admission and place identity, while
            # sending the robot only to a short point beyond the actual door.
            # A raw structural core can begin deep inside an unlabelled door
            # approach; treating it as a portal endpoint would recreate the
            # long cross-room action this topology layer is designed to avoid.
            identity_row, identity_col = source.identity_row, source.identity_col
            endpoint_row, endpoint_col = source.endpoint_row, source.endpoint_col
            portal_x, portal_y = self.cell_xy(
                request.message, endpoint_row, endpoint_col,
            )
            if request.excluded and any(
                math.hypot(portal_x - old_x, portal_y - old_y)
                < self.completed_radius
                for old_x, old_y in request.excluded
            ):
                self.last_portal_excluded += 1
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "excluded_radius",
                    request.now,
                ))
                continue
            if self.frontier_is_rejected(portal_x, portal_y, request.now):
                self.last_portal_rejected += 1
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "frontier_rejected",
                    request.now,
                ))
                continue

            costmap_distance = self.candidate_costmap_distance(
                request.validation, portal_x, portal_y,
            )
            if request.validation is not None and costmap_distance is None:
                # A portal endpoint encodes a physical proof. Replacing it by
                # the first Navfn-reachable point on a remote frontier route
                # can place TEB inside its success ball at the gate or cross
                # another portal. Keep this edge pending until a fresh map
                # yields a valid certified endpoint instead.
                self.last_portal_costmap_rejected += 1
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "costmap_unreachable",
                    request.now,
                ))
                continue

            portal_gate_xy = self.cell_xy(
                request.message, source.gate_row, source.gate_col,
            )
            if (
                portal_gate_xy is not None
                and self.sealed_portal_reentry(
                    portal_gate_xy,
                    (portal_x, portal_y),
                    map_to_physical_xy=request.map_to_physical_xy,
                ) is not None
            ):
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "sealed_reentry",
                    request.now,
                ))
                continue

            path_distance = (
                costmap_distance
                if costmap_distance is not None
                else request.steps[endpoint_row, endpoint_col]
                * request.message.info.resolution
            )
            score_path_distance = (
                float(request.steps[endpoint_row, endpoint_col])
                * request.message.info.resolution
                if request.score_path_from_steps
                else path_distance
            )
            route_heading_delta, heading_penalty = self._route_heading_penalty(
                request,
                (endpoint_row, endpoint_col),
                (portal_x, portal_y),
            )
            room_score = None
            ledger = getattr(self, "portal_hypothesis_ledger", None)
            hypothesis_key = getattr(source, "hypothesis_id", None)
            if ledger is not None and hypothesis_key is not None:
                record = ledger.get(hypothesis_key) or {}
                try:
                    room_score = float(record.get("semantic_score"))
                except (TypeError, ValueError):
                    room_score = None
            if self.portal_turn_needs_positive_room(
                    route_heading_delta, room_score):
                if forward_alternatives is None:
                    forward_alternatives = self._portal_forward_alternatives(
                        request, portal_sources)
                straight_available, positive_available = forward_alternatives
                if straight_available or positive_available:
                    # Going straight needs no evidence; turning into a room
                    # does, but only while the corridor ahead is still usable.
                    self.last_portal_negative_turn_rejected = int(
                        getattr(self, "last_portal_negative_turn_rejected", 0)
                    ) + 1
                    rejected_notes.append(self._note_portal_deferral(
                        hypothesis_key, "negative_room_turn", request.now,
                        heading_delta_deg=math.degrees(route_heading_delta),
                        detail={"semantic_score": room_score},
                    ))
                    self._retire_negative_room_entry(
                        record, hypothesis_key, room_score, request.now,
                    )
                    continue
                # Every candidate needs a large turn, so refusing this one
                # would park the robot at the end of a corridor.  Corridors
                # often score negative themselves (a door or fire hydrant is
                # nearby), so evidence may only demote a side room, never
                # block the last way forward.
                self.last_portal_negative_turn_last_resort = int(
                    getattr(self, "last_portal_negative_turn_last_resort", 0)
                ) + 1
            if (
                request.max_heading_delta is not None
                and route_heading_delta is not None
                and route_heading_delta > request.max_heading_delta
            ):
                runner_up = None
                if room_score is not None and room_best is not None:
                    runner_up = (
                        room_second
                        if abs(room_score - room_best) < 1e-9 else room_best
                    )
                override = self._portal_semantic_heading_override(
                    source, route_heading_delta, request.now,
                    runner_up_score=runner_up,
                )
                if override is None:
                    self.last_portal_heading_rejected += 1
                    rejected_notes.append(self._note_portal_deferral(
                        getattr(source, "hypothesis_id", None),
                        "heading",
                        request.now,
                        heading_delta_deg=math.degrees(route_heading_delta),
                    ))
                    continue
                self.last_portal_heading_overrides = int(
                    getattr(self, "last_portal_heading_overrides", 0)
                ) + 1
                override_signature = (
                    int(override.get("id", 0)),
                    round(math.degrees(route_heading_delta), 1),
                )
                if override_signature != getattr(
                    self, "_portal_heading_override_signature", None
                ):
                    self._portal_heading_override_signature = override_signature
                    self.publish_status(
                        "portal_heading_override",
                        portal_id=int(override["id"]),
                        source_place_id=override.get("source_place_id"),
                        heading_delta_deg=round(
                            math.degrees(route_heading_delta), 2
                        ),
                        semantic_score=float(override.get("semantic_score")),
                        attempt=int(
                            override.get("heading_override_count", 0) or 0
                        ),
                    )

            information = self.frontier_information(
                request.unknown, frontier_row, frontier_col,
            )
            # A portal action terminates in the first labelled core of its
            # adjacent place. Retain that identity now; after the matching
            # action terminal it becomes a durable physical-arrival fact.
            # Without it, the next cycle starts in an unowned room and can
            # later re-enter the same physical space through another doorway.
            component = (
                None
                if request.components is None
                else self.topology_component_evidence(
                    request.message,
                    request.components,
                    identity_row,
                    identity_col,
                    steps=request.steps,
                )
            )
            # Refresh/reuse the durable identity before admission so a
            # transient component cannot reclassify a known destination.
            # Existing identities may be refreshed before admission so a
            # preferred graph edge remains bound to its physical gate.  A new
            # candidate, however, is only an observation until the covered-
            # destination and reverse-egress checks below accept it.
            hypothesis = self._refresh_or_certify_portal_hypothesis(
                request,
                source,
                portal_gate_xy,
                (portal_x, portal_y),
                create=False,
            )
            if preferred_portal_id is not None:
                if (
                    preferred_portal_id is not None
                    and (
                        hypothesis is None
                        or int(hypothesis.get("id", -1)) != preferred_portal_id
                    )
                ):
                    # The slow graph layer selected a specific durable edge.
                    # A different snapshot-local doorway is not an equivalent
                    # substitute; let the caller retain the plan and wait for
                    # that physical edge to be reprojected.
                    rejected_notes.append(self._note_portal_deferral(
                        getattr(source, "hypothesis_id", None),
                        "preferred_identity_mismatch",
                        request.now,
                    ))
                    continue
            if hypothesis is None and getattr(source, "hypothesis_id", None) is not None:
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "hypothesis_missing",
                    request.now,
                ))
                continue
            if not self.portal_destination_is_admissible(
                request,
                portal_gate_xy,
                (portal_x, portal_y),
                component,
                information,
                hypothesis=hypothesis,
            ):
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "portal_destination_not_admissible",
                    request.now,
                ))
                continue
            if hypothesis is None:
                hypothesis = self._refresh_or_certify_portal_hypothesis(
                    request,
                    source,
                    portal_gate_xy,
                    (portal_x, portal_y),
                )
                if hypothesis is None:
                    rejected_notes.append(self._note_portal_deferral(
                        getattr(source, "hypothesis_id", None),
                        "recertify_failed",
                        request.now,
                    ))
                    continue
            certify_probe = getattr(self, "certify_portal_probe", None)
            if certify_probe is not None and hypothesis is not None:
                certify_probe(
                    request,
                    portal_gate_xy,
                    (portal_x, portal_y),
                    int(hypothesis["id"]),
                    request.now,
                )
            # Portal actions use the same graph executive as local candidates.
            # Covered transit remains gated by unresolved local work, while
            # branch-first may admit a certified edge to a novel Place.
            source_place_id = getattr(self, "current_physical_place_id", None)
            source_region = None
            region_memory = getattr(self, "region_memory", None)
            if source_place_id is not None and region_memory is not None:
                source_region = region_memory.by_id(source_place_id)
            local_work_pending = False
            context = getattr(self, "last_selection_context", None)
            pending_method = getattr(self, "context_has_pending_local_work", None)
            if pending_method is not None:
                local_work_pending = bool(pending_method(context))
            target_work_pending = bool(
                getattr(context, "target_observation_work_pending", False)
            )
            graph_action = choose_graph_action(
                graph_ready=request.components is not None,
                source_place_id=source_place_id,
                source_place_state=(
                    None if source_region is None
                    else source_region.get("state")
                ),
                source_place_observed=bool(
                    source_region is not None
                    and int(source_region.get("endpoint_observations", 0)) > 0
                ),
                place_hops=1,
                portal_certified=True,
                work_item_required=bool(
                    getattr(self, "work_item_memory_enabled", False)
                ),
                local_work_pending=local_work_pending,
                must_complete_local_work=bool(
                    # Crossing is an outward graph action. The Place-owned
                    # WorkItem phase must be complete before it is admitted;
                    # semantic target claims only add constraints to that
                    # invariant.
                    getattr(self, "work_item_memory_enabled", False)
                ),
                portal_branch_priority=bool(
                    getattr(self, "branch_first_enabled", False)
                ),
                target_work_pending=target_work_pending,
                portal_destination_unobserved=(
                    getattr(self, "last_portal_destination_class", None)
                    == "new_place"
                ),
            )
            self.last_graph_action_kind = graph_action.kind
            self.last_graph_action_reason = graph_action.reason
            if not graph_action.executable:
                self.last_graph_policy_rejections = int(
                    getattr(self, "last_graph_policy_rejections", 0)
                ) + 1
                rejected_notes.append(self._note_portal_deferral(
                    getattr(source, "hypothesis_id", None),
                    "graph_action:%s" % str(graph_action.reason),
                    request.now,
                ))
                continue
            # This counter is deliberately incremented only after route,
            # costmap, portal identity, and graph-action checks. A structural
            # source that merely existed in the map is not a recovery path.
            self.last_portal_executable_candidates += 1
            # Reuse is an executed policy decision only after every physical
            # admission gate has passed.  Emitting this event earlier made a
            # rejected projection look like a valid portal reuse in benchmark
            # summaries.
            if getattr(source, "hypothesis_id", None) is not None:
                self.publish_status(
                    "portal_hypothesis_reused",
                    portal_id=int(hypothesis["id"]),
                    source_place_id=int(hypothesis["source_place_id"]),
                    goal=[round(float(portal_x), 3), round(float(portal_y), 3)],
                    frame=str(
                        getattr(
                            getattr(request.message, "header", None),
                            "frame_id",
                            "map",
                        ) or "map"
                    ),
                )
            structure = self.frontier_structure(
                request.occupied, frontier_row, frontier_col,
            )
            score = self._frontier_candidate_score(
                information,
                structure,
                score_path_distance,
                (portal_x, portal_y),
                heading_penalty,
                request.semantic_hint,
            )
            action_class = getattr(
                self, "last_portal_destination_class", "new_place"
            )
            if use_evidence_decision:
                # A room whose measured contents match the task is itself a
                # target-relevant information action.  Without this term the
                # evidence layer only sees the direction-to-target hint and a
                # high-semantic doorway is downgraded to a plain new place,
                # losing to a nearer but semantically empty room.
                room_score = None
                if isinstance(hypothesis, dict):
                    try:
                        room_score = float(hypothesis.get("semantic_score"))
                    except (TypeError, ValueError):
                        room_score = None
                room_task_relevant = bool(
                    room_score is not None
                    and math.isfinite(room_score)
                    and room_score
                    >= float(
                        getattr(
                            self,
                            "heading_semantic_override_min_score",
                            0.5,
                        )
                    )
                )
                task_relevant = bool(
                    room_task_relevant
                    or (
                        getattr(request, "semantic_pursuit", False)
                        and semantic_hint_is_forward(
                            request.robot_map,
                            request.semantic_hint,
                            (portal_x, portal_y),
                        )
                    )
                )
                destination_unobserved = action_class == "new_place"
                decision_category = portal_action_category(
                    task_relevant=task_relevant,
                    destination_unobserved=destination_unobserved,
                    information_gain=information,
                )
                candidate_id = (
                    int(hypothesis["id"])
                    if hypothesis is not None
                    else (int(identity_row), int(identity_col))
                )
                decision_candidate = PortalDecisionCandidate(
                    candidate_id=candidate_id,
                    category=decision_category,
                    evidence=PortalDecisionEvidence(
                        task_relevance=1.0 if task_relevant else 0.0,
                        novelty=1.0 if destination_unobserved else 0.0,
                        information_gain=float(information),
                        # The route has already passed the hard costmap and
                        # portal-admission checks. Keep these explicit neutral
                        # facts until a planner supplies measured values.
                        clearance=1.0,
                        path_cost=float(path_distance),
                        risk=0.0,
                    ),
                    tie_break_key=(
                        candidate_id,
                        round(float(portal_x), 3),
                        round(float(portal_y), 3),
                    ),
                )
                evidence_candidates.append(decision_candidate)
                # Store the actual route candidate immediately. The previous
                # loop-local ``portal_best`` is unrelated to this candidate.
                evidence_routes[id(decision_candidate)] = (
                    (
                        endpoint_row,
                        endpoint_col,
                        portal_x,
                        portal_y,
                        path_distance,
                        information,
                        structure,
                        score,
                        component,
                        1,
                        "portal_transition",
                        portal_gate_xy,
                        None,
                        None,
                        0,
                        None,
                        False,
                        graph_action.kind,
                        graph_action.reason,
                    ),
                    hypothesis,
                    action_class,
                )
                continue
            rank = portal_class_rank.get(action_class, 0)
            best_rank = portal_class_rank.get(portal_best_class, -1)
            if rank < best_rank or (rank == best_rank and score <= portal_best_score):
                continue
            portal_best = (
                endpoint_row,
                endpoint_col,
                portal_x,
                portal_y,
                path_distance,
                information,
                structure,
                score,
                component,
                1,
                "portal_transition",
                portal_gate_xy,
                None,
                None,
                0,
                None,
                False,
                graph_action.kind,
                graph_action.reason,
            )
            portal_best_score = score
            portal_best_hypothesis = hypothesis
            portal_best_class = action_class

        signature = tuple(rejected_notes)
        if rejected_notes and signature != getattr(
            self, "_portal_rejection_signature", None
        ):
            self._portal_rejection_signature = signature
            self.publish_status(
                "portal_candidates_rejected",
                candidates_seen=int(self.last_portal_candidates_seen),
                executable_candidates=int(self.last_portal_executable_candidates),
                rejections=[
                    {"portal_id": portal_id, "reason": reason}
                    for portal_id, reason in rejected_notes
                ],
                destination_class=getattr(
                    self, "last_portal_destination_class", None
                ),
                graph_action_kind=getattr(self, "last_graph_action_kind", None),
                graph_action_reason=getattr(
                    self, "last_graph_action_reason", None
                ),
                local_work_pending=bool(
                    callable(
                        getattr(self, "context_has_pending_local_work", None)
                    )
                    and self.context_has_pending_local_work(
                        getattr(self, "last_selection_context", None)
                    )
                ),
                source_place_id=getattr(self, "current_physical_place_id", None),
            )
        elif not rejected_notes:
            self._portal_rejection_signature = None

        if use_evidence_decision and evidence_candidates:
            decision = select_evidence_portal_transition(evidence_candidates)
            selected = decision.selected
            self.last_portal_decision = (
                None
                if selected is None
                else {
                    "category": decision.category,
                    "candidate_count": len(decision.feasible) + len(decision.rejected),
                    "feasible_count": len(decision.feasible),
                    "pareto_front_count": len(decision.pareto_front),
                    "selected_candidate_id": selected.candidate_id,
                    "rejected": [
                        {
                            "candidate_id": rejection.candidate.candidate_id,
                            "reasons": list(rejection.reasons),
                        }
                        for rejection in decision.rejected
                    ],
                }
            )
            if selected is not None:
                portal_best, portal_best_hypothesis, portal_best_class = (
                    evidence_routes[id(selected)]
                )
                portal_best_score = float(portal_best[7])
                self.last_portal_destination_class = portal_best_class
                self.publish_status(
                    "portal_decision_selected",
                    category=decision.category,
                    candidate_count=len(decision.feasible) + len(decision.rejected),
                    feasible_count=len(decision.feasible),
                    pareto_front_count=len(decision.pareto_front),
                    candidate_id=selected.candidate_id,
                    evidence={
                        name: round(float(getattr(selected.evidence, name)), 4)
                        for name, _direction in (
                            ("task_relevance", "max"),
                            ("novelty", "max"),
                            ("information_gain", "max"),
                            ("clearance", "max"),
                            ("path_cost", "min"),
                            ("risk", "min"),
                        )
                    },
                )

        if portal_best is not None:
            self.last_frontier_region_tier = "portal"
            self.last_selected_place_hops = 1
            self.last_portal_endpoint = [int(portal_best[0]), int(portal_best[1])]
            if portal_best_hypothesis is not None:
                selected_hypothesis = getattr(
                    self, "portal_hypothesis_ledger", None
                ).select(portal_best_hypothesis["id"], now=request.now)
                if selected_hypothesis is not None:
                    # Preserve the durable Portal identity in the route
                    # contract.  Coordinates remain snapshot-local; this
                    # trailing field is what lets the graph transaction
                    # verify that the selected edge is the planned edge.
                    portal_best = tuple(portal_best) + (
                        int(selected_hypothesis["id"]),
                    )
                    self.last_portal_hypothesis_id = int(
                        selected_hypothesis["id"]
                    )
                    self.publish_status(
                        "portal_hypothesis_selected",
                        portal_id=int(selected_hypothesis["id"]),
                        source_place_id=int(
                            selected_hypothesis["source_place_id"]
                        ),
                        goal=[
                            round(float(portal_best[2]), 3),
                            round(float(portal_best[3]), 3),
                        ],
                        action_class=portal_best_class,
                    )
        self.last_portal_hard_rejected = max(
            0,
            int(self.last_portal_candidates_seen)
            - int(self.last_portal_executable_candidates),
        )
        return portal_best

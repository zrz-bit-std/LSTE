"""ROS-facing adapter for the pure durable graph route planner.

This boundary translates a discrete graph intent into the existing portal
selection/egress helpers.  It never computes geometry or publishes a velocity
command; Navfn and TEB remain the only motion authorities.
"""

from dataclasses import replace

import rospy

from global_frontier_graph_route_planner import (
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_RETRY_VIEWPOINT,
    PLAN_READY,
)


_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES = 4
_GRAPH_ROUTE_MATERIALIZATION_MAX_EPOCH_SPAN = 2
_GRAPH_ROUTE_MATERIALIZATION_EXCLUSION_EPOCHS = 8
_GRAPH_ROUTE_MATERIALIZATION_INVALIDATED_MAX = 128
_GRAPH_ROUTE_MATERIALIZATION_TRACKERS_MAX = 128
_GRAPH_ROUTE_FSM_SEARCHING = "SEARCHING"
_GRAPH_ROUTE_FSM_PLANNING = "PLANNING"
_GRAPH_ROUTE_FSM_WAITING = "WAITING_MATERIALIZATION"
_GRAPH_ROUTE_FSM_EXECUTING = "EXECUTING"
_GRAPH_ROUTE_FSM_NEXT_ACTION = "NEXT_ACTION"
from global_frontier_graph_route_transaction import (
    GraphRouteActionTransaction,
    candidate_action,
    materialize_graph_route_action,
)


class GlobalFrontierGraphRouteAdapterMixin:
    """Materialize only the first edge of a durable graph route."""

    @staticmethod
    def _graph_snapshot_map_epoch(snapshot):
        """Read the immutable map epoch used by graph negative evidence."""
        for value in (
            getattr(
                getattr(getattr(snapshot, "map_context", None), "components", None),
                "epoch",
                None,
            ),
            getattr(
                getattr(getattr(snapshot, "route_graph", None), "validation", None),
                "epoch",
                None,
            ),
        ):
            if value is None:
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                return value
        return None

    @staticmethod
    def _normalise_graph_map_epoch(value):
        """Normalize an optional map epoch without inventing provenance."""
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _transition_graph_route_fsm(
        self, state, *, transaction_id=0, map_epoch=None, reason=""
    ):
        """Record the graph executor's explicit lifecycle state."""
        previous = str(
            getattr(self, "graph_route_fsm_state", _GRAPH_ROUTE_FSM_SEARCHING)
            or _GRAPH_ROUTE_FSM_SEARCHING
        )
        state = str(state or _GRAPH_ROUTE_FSM_SEARCHING)
        if previous == state:
            return False
        self.graph_route_fsm_state = state
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "graph_route_fsm_transition",
                previous_state=previous,
                current_state=state,
                transaction_id=int(transaction_id or 0),
                map_epoch=map_epoch,
                reason=str(reason or ""),
            )
        if state == _GRAPH_ROUTE_FSM_NEXT_ACTION:
            rospy.logwarn(
                "Global frontier graph FSM -> NEXT_ACTION "
                "transaction_id=%s epoch=%s reason=%s",
                int(transaction_id or 0),
                map_epoch,
                str(reason or ""),
            )
        return True

    @staticmethod
    def _graph_route_materialization_obligation_key(plan):
        """Return a stable key for one durable graph obligation.

        The plan signature includes map and planner-reason fields and is
        therefore unsuitable for a failure budget.  Prefer the planner's
        explicit topology key, while retaining a compatibility fallback for
        older injected plan objects.
        """
        if plan is None:
            return None
        key_method = getattr(plan, "durable_obligation_key", None)
        if callable(key_method):
            try:
                key = key_method()
            except (TypeError, ValueError):
                key = None
            if key is not None:
                return tuple(key)

        action = str(getattr(plan, "action", "") or "")
        obligation_kind = str(
            getattr(plan, "obligation_kind", "") or ""
        )
        try:
            obligation_id = int(getattr(plan, "obligation_id", 0) or 0)
        except (TypeError, ValueError):
            obligation_id = 0
        if obligation_id > 0:
            return ("obligation", obligation_kind or action, obligation_id)
        try:
            portal_id = int(getattr(plan, "first_portal_id", 0) or 0)
        except (TypeError, ValueError):
            portal_id = 0
        if portal_id > 0:
            return ("portal", portal_id)
        path = []
        for value in getattr(plan, "portal_path", ()) or ():
            try:
                value = int(value)
            except (TypeError, ValueError):
                continue
            if value > 0:
                path.append(value)
        return (
            "logical",
            action,
            obligation_kind,
            getattr(plan, "current_place_id", None),
            getattr(plan, "target_place_id", None),
            tuple(path),
        )

    def _graph_route_materialization_identity(
        self, *, plan=None, map_epoch=None, transaction=None
    ):
        """Build one complete identity envelope for graph/handoff events."""
        transaction = transaction or getattr(
            self, "graph_route_action_transaction", None
        )

        def integer(value, default=0):
            try:
                return int(value or default)
            except (TypeError, ValueError):
                return int(default)

        def optional_integer(value):
            if value is None:
                return None
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        active_route_id = integer(getattr(self, "active_route_id", 0))
        released_route_id = integer(
            getattr(self, "last_released_route_id", 0)
        )
        route_id = max(active_route_id, released_route_id)
        graph_transaction_id = integer(
            getattr(transaction, "graph_transaction_id", 0)
        )
        transaction_id = integer(
            getattr(transaction, "transaction_id", 0)
        )
        if transaction_id <= 0:
            transaction_id = integer(
                getattr(self, "last_released_route_transaction_id", 0)
            )
        action_generation = integer(
            getattr(self, "active_action_generation", 0)
        )
        if (
            action_generation <= 0
            and released_route_id > 0
            and released_route_id == route_id
        ):
            action_generation = integer(
                getattr(self, "last_released_route_action_generation", 0)
            )
        lifecycle_getter = getattr(
            self, "route_lifecycle_transaction_id", None
        )
        lifecycle_transaction_id = integer(
            lifecycle_getter() if callable(lifecycle_getter) else getattr(
                getattr(self, "lifecycle_manager", None),
                "current_transaction_id",
                0,
            )
        )
        if lifecycle_transaction_id <= 0:
            lifecycle_transaction_id = integer(
                getattr(
                    self,
                    "last_released_route_lifecycle_transaction_id",
                    0,
                )
            )
        transaction_epoch = optional_integer(
            getattr(transaction, "map_epoch", None)
        )
        resolved_map_epoch = optional_integer(map_epoch)
        if resolved_map_epoch is None:
            resolved_map_epoch = transaction_epoch
        if resolved_map_epoch is None:
            resolved_map_epoch = optional_integer(
                getattr(self, "last_map_epoch", None)
            )
        origin_epoch = optional_integer(
            getattr(transaction, "origin_epoch", None)
        )
        if origin_epoch is None:
            origin_epoch = resolved_map_epoch
        projection_epoch = optional_integer(
            getattr(transaction, "projection_epoch", None)
        )
        if projection_epoch is None:
            projection_epoch = optional_integer(
                getattr(self, "last_graph_snapshot_epoch", None)
            )
        if projection_epoch is None:
            projection_epoch = resolved_map_epoch
        return {
            "route_id": route_id,
            "transaction_id": transaction_id,
            "action_generation": action_generation,
            "lifecycle_transaction_id": lifecycle_transaction_id,
            "graph_transaction_id": graph_transaction_id,
            "map_epoch": resolved_map_epoch,
            "origin_epoch": origin_epoch,
            "projection_epoch": projection_epoch,
        }

    def _graph_plan_matches_active_route_contract(
        self, plan=None, transaction=None
    ):
        """Return whether a graph plan owns the current controller route.

        A durable successor can be prepared while the predecessor's persistent
        action is still running.  The mutable graph transaction then describes
        the successor, while ``active_route_dispatch_contract`` remains the
        only immutable record for the controller owner.  Do not infer
        ownership from route recency or geometry; release is allowed only when
        both graph transaction identities are present and equal.
        """
        if plan is None and transaction is None:
            transaction = getattr(self, "graph_route_action_transaction", None)
        if transaction is None:
            return False
        try:
            active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        except (TypeError, ValueError):
            active_route_id = 0
        if active_route_id <= 0 or getattr(self, "active_frontier", None) is None:
            return False
        active_contract = getattr(self, "active_route_dispatch_contract", None)
        if not isinstance(active_contract, dict):
            return False

        def positive(value):
            try:
                value = int(value or 0)
            except (TypeError, ValueError):
                return 0
            return value if value > 0 else 0

        active_graph_transaction_id = positive(
            active_contract.get("graph_transaction_id")
        )
        candidate_graph_transaction_id = positive(
            getattr(transaction, "graph_transaction_id", 0)
        )
        if active_graph_transaction_id <= 0 or candidate_graph_transaction_id <= 0:
            return False
        return candidate_graph_transaction_id == active_graph_transaction_id

    def _graph_route_materialization_tracker(
        self, obligation_key, create=False
    ):
        """Return the bounded failure bucket for one durable obligation."""
        if obligation_key is None:
            return None
        try:
            key = tuple(obligation_key)
        except TypeError:
            return None
        trackers = getattr(
            self, "graph_route_materialization_trackers", None
        )
        if not isinstance(trackers, dict):
            if not create:
                return None
            trackers = {}
            self.graph_route_materialization_trackers = trackers
        tracker = trackers.get(key)
        if tracker is None and create:
            tracker = {
                "miss_count": 0,
                "epochs": [],
                "last_observation": None,
                "first_epoch": None,
                "last_epoch": None,
                "last_event": None,
            }
            trackers[key] = tracker
            while len(trackers) > _GRAPH_ROUTE_MATERIALIZATION_TRACKERS_MAX:
                oldest = next(iter(trackers))
                if oldest == key and len(trackers) > 1:
                    oldest = next(
                        candidate
                        for candidate in trackers
                        if candidate != key
                    )
                trackers.pop(oldest, None)
        return tracker

    def _load_graph_materialization_tracking(self, obligation_key):
        """Expose one obligation bucket through legacy diagnostic fields."""
        if obligation_key is None:
            self.graph_route_materialization_miss_signature = None
            self.graph_route_materialization_miss_count = 0
            self.graph_route_materialization_epochs = []
            self.graph_route_materialization_last_event = None
            self.graph_route_materialization_obligation_key = None
            self.graph_route_materialization_last_observation = None
            self.graph_route_materialization_first_epoch = None
            self.graph_route_materialization_last_epoch = None
            return None
        key = tuple(obligation_key)
        # A first miss must create a persistent bucket.  Returning a temporary
        # empty dict here would make the increment visible only to this call,
        # so the next map epoch would restart the durable failure budget.
        tracker = self._graph_route_materialization_tracker(key, create=True)
        tracker = tracker if isinstance(tracker, dict) else {}
        self.graph_route_materialization_miss_signature = key
        self.graph_route_materialization_miss_count = int(
            tracker.get("miss_count", 0) or 0
        )
        self.graph_route_materialization_epochs = list(
            tracker.get("epochs", ()) or ()
        )
        self.graph_route_materialization_last_event = tracker.get(
            "last_event"
        )
        self.graph_route_materialization_obligation_key = key
        self.graph_route_materialization_last_observation = tracker.get(
            "last_observation"
        )
        self.graph_route_materialization_first_epoch = tracker.get(
            "first_epoch"
        )
        self.graph_route_materialization_last_epoch = tracker.get(
            "last_epoch"
        )
        return tracker

    def _reset_graph_materialization_tracking(self, obligation_key=None):
        """Forget one obligation's miss bucket after success/invalidation."""
        if obligation_key is None:
            obligation_key = getattr(
                self, "graph_route_materialization_obligation_key", None
            )
        if obligation_key is not None:
            try:
                trackers = getattr(
                    self, "graph_route_materialization_trackers", {}
                )
                if isinstance(trackers, dict):
                    trackers.pop(tuple(obligation_key), None)
            except TypeError:
                pass
        self.graph_route_materialization_miss_signature = None
        self.graph_route_materialization_miss_count = 0
        self.graph_route_materialization_epochs = []
        self.graph_route_materialization_last_event = None
        self.graph_route_materialization_obligation_key = None
        self.graph_route_materialization_last_observation = None
        self.graph_route_materialization_first_epoch = None
        self.graph_route_materialization_last_epoch = None

    def _target_observation_materialization_deferred(self, place_id):
        """Return whether target reinspection is deferred at one Place.

        The target WorkItem remains unresolved until an explicit completion or
        release event.  This separate state only records that the last
        reinspection attempt had no executable viewpoint, so an unresolved
        target cannot force the same empty graph transaction forever.
        """
        try:
            place_id = int(place_id)
        except (TypeError, ValueError):
            return False
        if place_id <= 0:
            return False
        deferred = getattr(
            self, "target_observation_materialization_deferred_places", ()
        )
        if not isinstance(deferred, (set, tuple, list)):
            return False
        return place_id in deferred

    def _defer_target_observation_materialization(self, plan, reason):
        """Park an unmaterializable target observation without completing it."""
        if str(getattr(plan, "obligation_kind", "") or "") != "target_observation":
            return None
        try:
            place_id = int(getattr(plan, "current_place_id", 0) or 0)
        except (TypeError, ValueError):
            place_id = 0
        if place_id <= 0:
            return None
        deferred = getattr(
            self, "target_observation_materialization_deferred_places", None
        )
        if not isinstance(deferred, set):
            deferred = set()
            self.target_observation_materialization_deferred_places = deferred
        already_deferred = place_id in deferred
        deferred.add(place_id)
        if not already_deferred:
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "target_observation_materialization_deferred",
                    place_id=place_id,
                    action=str(getattr(plan, "action", "") or ""),
                    obligation_kind="target_observation",
                    reason=str(reason or "materialization_exhausted"),
                    target_obligation_preserved=True,
                    next_policy="continue_other_local_or_graph_work",
                )
        return place_id

    def _clear_target_observation_materialization_deferred(
        self, place_id=None, reason="new_target_evidence"
    ):
        """Reopen target reinspection only after explicit new evidence."""
        deferred = getattr(
            self, "target_observation_materialization_deferred_places", None
        )
        if not isinstance(deferred, set):
            return False
        if place_id is None:
            cleared = bool(deferred)
            deferred.clear()
        else:
            try:
                place_id = int(place_id)
            except (TypeError, ValueError):
                return False
            cleared = place_id in deferred
            deferred.discard(place_id)
        if cleared:
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "target_observation_materialization_reopened",
                    place_id=place_id,
                    reason=str(reason or "new_target_evidence"),
                )
        return cleared

    def _remember_graph_route_materialization_exclusion(
        self, map_epoch, kind, identity
    ):
        """Exclude one failed identity only from the current map epoch."""
        epoch = self._normalise_graph_map_epoch(map_epoch)
        try:
            identity = int(identity)
        except (TypeError, ValueError):
            return False
        if epoch is None or identity <= 0:
            return False
        exclusions = getattr(
            self, "graph_route_materialization_exclusions", None
        )
        if not isinstance(exclusions, dict):
            exclusions = {}
            self.graph_route_materialization_exclusions = exclusions
        bucket = exclusions.setdefault(epoch, {})
        bucket.setdefault(str(kind), set()).add(identity)
        if len(exclusions) > _GRAPH_ROUTE_MATERIALIZATION_EXCLUSION_EPOCHS:
            for old_epoch in sorted(exclusions)[: -_GRAPH_ROUTE_MATERIALIZATION_EXCLUSION_EPOCHS]:
                exclusions.pop(old_epoch, None)
        return True

    def _graph_route_materialization_exclusions(self, map_epoch):
        """Return epoch and run-scoped exclusions for graph planning."""
        epoch = self._normalise_graph_map_epoch(map_epoch)
        if epoch is None:
            epoch_bucket = {}
        else:
            epoch_bucket = getattr(
                self, "graph_route_materialization_exclusions", {}
            ).get(epoch, {})
        portal_ids = set(epoch_bucket.get("portal_ids", set()))
        work_item_ids = set(epoch_bucket.get("work_item_ids", set()))
        probe_ids = set(epoch_bucket.get("probe_ids", set()))

        # Epoch-scoped negative evidence prevents an immediate retry.  An
        # exhausted durable obligation must also be excluded from the next
        # graph plan, even after a new map epoch creates a fresh bucket.
        raw_invalidated = getattr(
            self, "graph_route_materialization_invalidated_obligations", ()
        )
        if not isinstance(raw_invalidated, (set, tuple, list)):
            raw_invalidated = ()
        invalidated = {
            tuple(key)
            for key in raw_invalidated
            if isinstance(key, (tuple, list)) and key
        }
        mapped_keys = set()
        identity_map = getattr(
            self, "graph_route_materialization_invalidated_identity_map", {}
        )
        if isinstance(identity_map, dict):
            for raw_key, linked in identity_map.items():
                key = tuple(raw_key) if isinstance(raw_key, (tuple, list)) else None
                if key not in invalidated or not isinstance(linked, dict):
                    continue
                mapped_keys.add(key)
                for value in linked.get("portal_ids", ()):
                    try:
                        value = int(value)
                    except (TypeError, ValueError):
                        continue
                    if value > 0:
                        portal_ids.add(value)
                for value in linked.get("probe_ids", ()):
                    try:
                        value = int(value)
                    except (TypeError, ValueError):
                        continue
                    if value > 0:
                        probe_ids.add(value)
                for value in linked.get("work_item_ids", ()):
                    try:
                        value = int(value)
                    except (TypeError, ValueError):
                        continue
                    if value > 0:
                        work_item_ids.add(value)

        for key in invalidated:
            if key in mapped_keys:
                continue
            if not isinstance(key, (tuple, list)) or not key:
                continue
            if key[0] == "portal" and len(key) >= 2:
                try:
                    portal_ids.add(int(key[1]))
                except (TypeError, ValueError):
                    continue
                continue
            if key[0] != "obligation" or len(key) < 3:
                continue
            try:
                identity = int(key[2])
            except (TypeError, ValueError):
                continue
            kind = str(key[1] or "")
            if kind == "work_item":
                work_item_ids.add(identity)
            elif kind == "portal_probe":
                probe_ids.add(identity)
                # Older state only persisted the durable key.  In the graph
                # plan representation a source probe's first portal is the
                # same physical edge when no explicit link was persisted.
                portal_ids.add(identity)
            elif kind in {"portal_edge", "unbound_portal"}:
                portal_ids.add(identity)
        return (
            tuple(sorted(portal_ids)),
            tuple(sorted(work_item_ids)),
            tuple(sorted(probe_ids)),
        )

    def _remember_graph_route_materialization_invalidation(self, plan):
        """Keep an exhausted obligation out of the next graph-plan pass."""
        key = self._graph_route_materialization_obligation_key(plan)
        if key is None:
            return None
        invalidated = getattr(
            self, "graph_route_materialization_invalidated_obligations", None
        )
        if not isinstance(invalidated, set):
            invalidated = set()
            self.graph_route_materialization_invalidated_obligations = invalidated
        invalidated.add(tuple(key))
        while len(invalidated) > _GRAPH_ROUTE_MATERIALIZATION_INVALIDATED_MAX:
            invalidated.pop()
        identity_map = getattr(
            self, "graph_route_materialization_invalidated_identity_map", None
        )
        if not isinstance(identity_map, dict):
            identity_map = {}
            self.graph_route_materialization_invalidated_identity_map = identity_map

        def positive(value):
            try:
                value = int(value)
            except (TypeError, ValueError):
                return None
            return value if value > 0 else None

        linked = {
            "portal_ids": set(),
            "probe_ids": set(),
            "work_item_ids": set(),
        }
        obligation_kind = str(getattr(plan, "obligation_kind", "") or "")
        obligation_id = positive(getattr(plan, "obligation_id", None))
        first_portal_id = positive(getattr(plan, "first_portal_id", None))
        if obligation_kind == "portal_probe" and obligation_id is not None:
            linked["probe_ids"].add(obligation_id)
            # PortalProbe and Portal IDs are separate identity domains. Resolve
            # the physical doorway from the durable probe ledger before
            # populating the planner exclusion set; using the probe ID as a
            # Portal ID is only a compatibility fallback for old records.
            probe_ledger = getattr(self, "portal_probe_ledger", None)
            get_probe = getattr(probe_ledger, "get", None)
            probe_record = (
                None if not callable(get_probe) else get_probe(obligation_id)
            )
            linked_portal_id = (
                None
                if not isinstance(probe_record, dict)
                else positive(probe_record.get("portal_id"))
            )
            if linked_portal_id is not None:
                linked["portal_ids"].add(linked_portal_id)
            elif first_portal_id is not None:
                linked["portal_ids"].add(first_portal_id)
        elif obligation_kind == "work_item" and obligation_id is not None:
            linked["work_item_ids"].add(obligation_id)
        elif obligation_kind in {"portal_edge", "unbound_portal"}:
            if obligation_id is not None:
                linked["portal_ids"].add(obligation_id)
        if first_portal_id is not None and obligation_kind != "portal_probe":
            linked["portal_ids"].add(first_portal_id)
        identity_map[tuple(key)] = linked
        while len(identity_map) > _GRAPH_ROUTE_MATERIALIZATION_INVALIDATED_MAX:
            identity_map.pop(next(iter(identity_map)))
        return tuple(key)

    def _record_graph_route_negative_evidence(
        self, plan, map_epoch, reason, miss_count, epoch_span
    ):
        """Persist epoch-scoped negative evidence before dropping the plan."""
        epoch = self._normalise_graph_map_epoch(map_epoch)
        obligation_key = self._graph_route_materialization_obligation_key(plan)
        transaction = getattr(self, "graph_route_action_transaction", None)
        identity = self._graph_route_materialization_identity(
            plan=plan, map_epoch=epoch, transaction=transaction
        )
        negative = {
            **identity,
            "obligation_key": (
                None if obligation_key is None else list(obligation_key)
            ),
            "action": str(getattr(plan, "action", "") or ""),
            "obligation_kind": str(
                getattr(plan, "obligation_kind", "") or ""
            ),
            "obligation_id": getattr(plan, "obligation_id", None),
            "first_portal_id": getattr(plan, "first_portal_id", None),
            "reason": str(reason or ""),
            "miss_count": int(miss_count),
            "epoch_span": int(epoch_span),
        }
        portal_id = getattr(plan, "first_portal_id", None)
        portal_action = str(getattr(plan, "action", "") or "")
        portal_obligation = str(
            getattr(plan, "obligation_kind", "") or ""
        )
        portal_ledger = getattr(self, "portal_hypothesis_ledger", None)
        get_portal = getattr(portal_ledger, "get", None)
        if (
            portal_id is not None
            and callable(get_portal)
            and (
                portal_action == "cross_portal"
                or portal_obligation in {"portal_edge", "unbound_portal"}
            )
        ):
            try:
                portal_id = int(portal_id)
            except (TypeError, ValueError):
                portal_id = None
            record = None if portal_id is None else get_portal(portal_id)
            if (
                isinstance(record, dict)
                and record.get("destination_place_id") is None
                and str(record.get("state", "")).strip().lower()
                in {"certified", "selected"}
            ):
                mark_rejected = getattr(
                    portal_ledger, "mark_unbound_rejected", None
                )
                if callable(mark_rejected):
                    marked = mark_rejected(
                        portal_id,
                        epoch,
                        "portal_unmaterializable_at_epoch",
                    )
                    negative["portal_negative_evidence"] = {
                        "portal_id": portal_id,
                        "map_epoch": epoch,
                        "reason": "portal_unmaterializable_at_epoch",
                        "state": (
                            None if marked is None else marked.get("state")
                        ),
                    }
        if (
            str(getattr(plan, "action", "") or "") == ACTION_PROBE_PORTAL
            and str(getattr(plan, "obligation_kind", "") or "")
            == "portal_probe"
            and getattr(plan, "obligation_id", None) is not None
        ):
            probe_ledger = getattr(self, "portal_probe_ledger", None)
            park = getattr(probe_ledger, "mark_projection_unavailable", None)
            if callable(park):
                parked = park(
                    plan.obligation_id,
                    map_epoch=epoch,
                    reason="probe_unmaterializable_at_epoch",
                )
                negative["probe_negative_evidence"] = {
                    "probe_id": plan.obligation_id,
                    "map_epoch": epoch,
                    "reason": "probe_unmaterializable_at_epoch",
                    "state": None if parked is None else parked.get("state"),
                }
                if self._remember_graph_route_materialization_exclusion(
                    epoch, "probe_ids", plan.obligation_id
                ):
                    negative["probe_negative_evidence"][
                        "excluded_in_map_epoch"
                    ] = True
        if epoch is not None:
            obligation_kind = str(
                getattr(plan, "obligation_kind", "") or ""
            )
            obligation_id = getattr(plan, "obligation_id", None)
            if (
                obligation_kind == "work_item"
                and self._remember_graph_route_materialization_exclusion(
                    epoch, "work_item_ids", obligation_id
                )
            ):
                negative["work_item_negative_evidence"] = {
                    "work_item_id": int(obligation_id),
                    "map_epoch": epoch,
                    "reason": "work_item_unmaterializable_at_epoch",
                    "excluded_in_map_epoch": True,
                }
            if (
                portal_id is not None
                and (
                    portal_action == ACTION_CROSS_PORTAL
                    or portal_obligation in {
                        "portal_edge",
                        "unbound_portal",
                    }
                )
                and self._remember_graph_route_materialization_exclusion(
                    epoch, "portal_ids", portal_id
                )
            ):
                negative.setdefault("portal_negative_evidence", {}).update(
                    {
                        "excluded_in_map_epoch": True,
                        "map_epoch": epoch,
                    }
                )
        self.last_graph_route_materialization_negative_evidence = negative
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "graph_route_materialization_negative_evidence",
                **negative,
            )
        return negative

    def _invalidate_graph_route_for_materialization(
        self, plan, map_epoch, reason, miss_count, epoch_span
    ):
        """Invalidate one exhausted graph plan and enter NEXT_ACTION."""
        obligation_key = self._remember_graph_route_materialization_invalidation(
            plan
        )
        target_deferred_place = self._defer_target_observation_materialization(
            plan, reason
        )
        transaction = getattr(self, "graph_route_action_transaction", None)
        transaction_id = (
            0 if transaction is None else int(transaction.transaction_id)
        )
        epoch = self._normalise_graph_map_epoch(map_epoch)
        if epoch is None and transaction is not None:
            epoch = self._normalise_graph_map_epoch(transaction.map_epoch)
        try:
            active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        except (TypeError, ValueError):
            active_route_id = 0
        active_route_present = bool(
            active_route_id > 0
            and getattr(self, "active_frontier", None) is not None
        )
        active_graph_contract_match = bool(
            self._graph_plan_matches_active_route_contract(
                plan=plan, transaction=transaction
            )
        )
        controller_release_allowed = bool(
            not active_route_present or active_graph_contract_match
        )
        self._record_graph_route_negative_evidence(
            plan, epoch, reason, miss_count, epoch_span
        )
        identity = self._graph_route_materialization_identity(
            plan=plan, map_epoch=epoch, transaction=transaction
        )
        # The graph obligation may expire after the controller has already
        # released the physical route that brought the robot to this Place.
        # Keep that predecessor contract explicit instead of making the later
        # graph failure look like it released the newer materialization
        # transaction.  This is especially important for a bound Portal:
        # graph transaction N can fail after the controller ACK for the
        # predecessor route, while the actual successor is dispatched later.
        predecessor_contract = None
        active_contract = getattr(self, "active_route_dispatch_contract", None)
        if isinstance(active_contract, dict):
            predecessor_contract = dict(active_contract)
        if predecessor_contract is None:
            released_route_id = int(
                getattr(self, "last_released_route_id", 0) or 0
            )
            plan_route_id = int(getattr(plan, "route_id", 0) or 0)
            if released_route_id > 0 and (
                plan_route_id <= 0 or released_route_id == plan_route_id
            ):
                predecessor_contract = {
                    "route_id": released_route_id,
                    "route_kind": str(
                        getattr(self, "last_released_route_kind", "") or ""
                    ),
                    "transaction_id": int(
                        getattr(self, "last_released_route_transaction_id", 0)
                        or 0
                    ),
                    "lifecycle_transaction_id": int(
                        getattr(
                            self,
                            "last_released_route_lifecycle_transaction_id",
                            0,
                        )
                        or 0
                    ),
                    "action_generation": int(
                        getattr(self, "last_released_route_action_generation", 0)
                        or 0
                    ),
                    "graph_transaction_id": int(
                        getattr(
                            self, "last_released_route_graph_transaction_id", 0
                        )
                        or 0
                    ),
                    "map_epoch": getattr(
                        self, "last_released_route_map_epoch", None
                    ),
                    "origin_epoch": getattr(
                        self, "last_released_route_origin_epoch", None
                    ),
                    "projection_epoch": getattr(
                        self, "last_released_route_projection_epoch", None
                    ),
                }
        if isinstance(predecessor_contract, dict):
            predecessor_contract = {
                key: value
                for key, value in predecessor_contract.items()
                if value not in (None, "")
            }
            if int(predecessor_contract.get("route_id", 0) or 0) > 0:
                if controller_release_allowed:
                    self.pending_graph_successor_predecessor_contract = dict(
                        predecessor_contract
                    )
                identity["predecessor_contract"] = dict(predecessor_contract)
                identity["predecessor_route_id"] = int(
                    predecessor_contract["route_id"]
                )
        invalidation = {
            **identity,
            "obligation_key": (
                None if obligation_key is None else list(obligation_key)
            ),
            "action": str(getattr(plan, "action", "") or ""),
            "obligation_kind": str(
                getattr(plan, "obligation_kind", "") or ""
            ),
            "obligation_id": getattr(plan, "obligation_id", None),
            "first_portal_id": getattr(plan, "first_portal_id", None),
            "miss_count": int(miss_count),
            "epoch_span": int(epoch_span),
            "max_misses": _GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES,
            "max_epoch_span": _GRAPH_ROUTE_MATERIALIZATION_MAX_EPOCH_SPAN,
            "reason": str(reason),
            "active_route_graph_contract_match": active_graph_contract_match,
            "controller_release_allowed": controller_release_allowed,
            "active_route_retained": bool(
                active_route_present and not controller_release_allowed
            ),
        }
        if target_deferred_place is not None:
            invalidation["target_observation_materialization_deferred"] = True
            invalidation["target_observation_place_id"] = int(
                target_deferred_place
            )
        if isinstance(predecessor_contract, dict):
            invalidation["predecessor_contract"] = dict(predecessor_contract)
        self.last_graph_route_materialization_invalidation = invalidation
        self.graph_route_materialization_invalidated = True
        next_state = (
            _GRAPH_ROUTE_FSM_NEXT_ACTION
            if controller_release_allowed
            else _GRAPH_ROUTE_FSM_EXECUTING
        )
        self._transition_graph_route_fsm(
            next_state,
            transaction_id=transaction_id,
            map_epoch=epoch,
            reason=(
                str(reason)
                if controller_release_allowed
                else "successor_%s_active_route_retained" % str(reason)
            ),
        )
        # Invalidation is a controller ownership boundary, not only a graph
        # bookkeeping event. Release the local frontier lease before asking
        # the Bridge to cancel the corresponding persistent action so the next
        # planning pass can allocate a strictly newer route identity.
        release_frontier = getattr(self, "release_active_frontier", None)
        old_route_id = int(getattr(self, "active_route_id", 0) or 0)
        if (
            controller_release_allowed
            and
            callable(release_frontier)
            and old_route_id > 0
            and getattr(self, "active_frontier", None) is not None
        ):
            released = release_frontier(
                discard_prefetch=True,
                preserve_place_departure=False,
                # The bridge must acknowledge this forced physical release
                # before the graph is allowed to submit a successor route.
                queue_successor=False,
                await_controller_release_ack=True,
            )
            invalidation.update(
                {
                    "controller_release_requested": True,
                    "controller_release_route_id": old_route_id,
                    "controller_release_result": bool(released),
                }
            )
        elif active_route_present:
            invalidation.update(
                {
                    "controller_release_requested": False,
                    "controller_release_route_id": active_route_id,
                    "controller_release_result": False,
                    "controller_release_suppressed": True,
                    "controller_release_suppression_reason": (
                        "successor_graph_contract_not_active"
                    ),
                }
            )
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "graph_route_materialization_invalidated",
                **invalidation,
                portal_path=list(getattr(plan, "portal_path", ()) or ()),
                next_state=next_state,
                controller_lease=(
                    "release" if controller_release_allowed else "held"
                ),
            )
        rospy.logwarn(
            "Global frontier invalidating graph route "
            "route_id=%s transaction_id=%s action_generation=%s "
            "graph_transaction_id=%s map_epoch=%s origin_epoch=%s "
            "projection_epoch=%s misses=%d epoch_span=%d obligation=%s "
            "portal=%s reason=%s",
            identity["route_id"],
            identity["transaction_id"],
            identity["action_generation"],
            identity["graph_transaction_id"],
            identity["map_epoch"],
            identity["origin_epoch"],
            identity["projection_epoch"],
            int(miss_count),
            int(epoch_span),
            getattr(plan, "obligation_id", None),
            getattr(plan, "first_portal_id", None),
            str(reason),
        )
        publish_unavailable = getattr(
            self, "_publish_graph_route_unavailable", None
        )
        if callable(publish_unavailable):
            publish_unavailable(
                plan,
                "graph_route_materialization_expired",
                map_epoch=epoch,
                controller_release_allowed=controller_release_allowed,
            )
        scheduler = getattr(self, "decision_wake_scheduler", None)
        route_id = max(
            int(getattr(self, "active_route_id", 0) or 0),
            int(getattr(self, "last_released_route_id", 0) or 0),
        )
        if scheduler is not None and controller_release_allowed:
            advanced = scheduler.finish_route(
                route_id,
                "graph_route_materialization_invalidated",
            )
            if not advanced:
                scheduler.request("graph_route_materialization_invalidated")
        self.last_planning_wall = 0.0
        self.last_graph_route_plan = None
        self.last_graph_route_plan_signature = None
        self.graph_route_action_transaction = None
        self.graph_route_portal_id = None
        self.graph_route_probe_id = None
        self.graph_route_work_item_id = None
        self.graph_route_work_item_filter = None
        self._clear_graph_route_plan_lease("graph_route_materialization_expired")
        self._reset_graph_materialization_tracking(
            self._graph_route_materialization_obligation_key(plan)
        )
        return True

    def _graph_route_plan_for_active_work_item(self, work_item_id=None):
        """Return the committed graph plan for one active WorkItem lease.

        Execution failure handling clears the mutable WorkItem attempt before
        releasing the controller route.  The graph transaction is the durable
        identity that must survive that cleanup, so resolve the plan from the
        committed transaction first and use the last plan as a compatibility
        fallback.  A plan from another obligation must never be reused merely
        because its route happened to be the latest one.
        """
        if not getattr(self, "graph_route_planner_enabled", False):
            return None
        try:
            work_item_id = int(
                getattr(self, "active_work_item_id", None)
                if work_item_id is None
                else work_item_id
            )
        except (TypeError, ValueError):
            return None
        if work_item_id <= 0:
            return None
        transaction = getattr(self, "graph_route_action_transaction", None)
        candidates = (
            getattr(transaction, "plan", None),
            getattr(self, "last_graph_route_plan", None),
        )
        seen = set()
        for plan in candidates:
            if plan is None or id(plan) in seen:
                continue
            seen.add(id(plan))
            if getattr(plan, "status", None) != PLAN_READY:
                continue
            if str(getattr(plan, "obligation_kind", "") or "") != "work_item":
                continue
            try:
                planned_id = int(getattr(plan, "obligation_id", 0) or 0)
            except (TypeError, ValueError):
                continue
            if planned_id == work_item_id:
                return plan
        return None

    def _record_graph_route_activation_rejection(
        self, selection=None, map_epoch=None, reason="activation_rejected"
    ):
        """Feed a pre-route activation reject into the durable miss budget.

        Portal leases are reserved before a frontier route is installed.  A
        rejected reservation therefore has no active controller route for the
        execution-failure path to observe, but it still invalidates the
        current graph materialization attempt.  Keep the graph identity until
        the normal miss/epoch budget either permits a later projection or
        canonically excludes the obligation.
        """
        if not getattr(self, "graph_route_planner_enabled", False):
            return False

        def positive(value):
            try:
                value = int(value)
            except (TypeError, ValueError):
                return None
            return value if value > 0 else None

        probe = getattr(selection, "portal_observation_probe", None)
        selected_probe_id = positive(getattr(probe, "probe_id", None))
        active_probe_id = positive(
            getattr(self, "active_portal_probe_id", None)
        )
        selected_work_item_id = positive(
            getattr(selection, "work_item_id", None)
        )
        transaction = getattr(self, "graph_route_action_transaction", None)
        candidates = (
            getattr(transaction, "plan", None),
            getattr(self, "last_graph_route_plan", None),
        )
        plan = None
        seen = set()
        for candidate in candidates:
            if candidate is None or id(candidate) in seen:
                continue
            seen.add(id(candidate))
            if getattr(candidate, "status", None) != PLAN_READY:
                continue
            kind = str(getattr(candidate, "obligation_kind", "") or "")
            obligation_id = positive(getattr(candidate, "obligation_id", None))
            if kind == "portal_probe":
                expected = selected_probe_id or active_probe_id
                if expected is not None and obligation_id == expected:
                    plan = candidate
                    break
            elif (
                kind == "work_item"
                and selected_work_item_id is not None
                and obligation_id == selected_work_item_id
            ):
                plan = candidate
                break
        if plan is None:
            return False

        if map_epoch is None and selection is not None:
            component = getattr(selection, "component", None)
            map_epoch = (
                component.get("epoch")
                if isinstance(component, dict)
                else getattr(component, "epoch", None)
            )
        if map_epoch is None:
            map_epoch = getattr(transaction, "map_epoch", None)
        if map_epoch is None:
            map_epoch = getattr(self, "last_graph_snapshot_epoch", None)

        record_miss = getattr(
            self, "_record_graph_route_materialization_miss", None
        )
        if not callable(record_miss):
            return False
        exhausted = bool(
            record_miss(
                plan,
                map_epoch,
                "graph_route_activation_rejected:%s" % str(reason or "reject"),
            )
        )
        if not exhausted:
            retain_lease = getattr(self, "_retain_graph_route_plan_lease", None)
            if callable(retain_lease):
                retain_lease(plan, "graph_route_activation_rejected")
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            identity = self._graph_route_materialization_identity(
                plan=plan, map_epoch=map_epoch, transaction=transaction
            )
            publish(
                "graph_route_activation_rejection_recorded",
                **identity,
                obligation_key=list(
                    self._graph_route_materialization_obligation_key(plan)
                ),
                obligation_kind=str(getattr(plan, "obligation_kind", "") or ""),
                obligation_id=getattr(plan, "obligation_id", None),
                reason=str(reason or "activation_rejected"),
                exhausted=exhausted,
            )
        return True

    def _record_active_work_item_materialization_failure(
        self, plan=None, map_epoch=None, reason="active_work_item_route_failure"
    ):
        """Feed an active WorkItem route failure into canonical exclusion.

        The controller route has already been released by the caller before
        this method is invoked.  That ordering lets the common invalidation
        path publish the durable graph event without cancelling the same
        action twice.  Until the normal miss/epoch budget is exhausted, retain
        the same plan lease so a later map epoch can provide the next piece of
        negative evidence instead of silently switching obligations.
        """
        if plan is None:
            plan = self._graph_route_plan_for_active_work_item()
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        if str(getattr(plan, "obligation_kind", "") or "") != "work_item":
            return False
        return self._record_active_durable_materialization_failure(
            plan=plan,
            map_epoch=map_epoch,
            reason=reason,
        )

    def _graph_route_plan_for_active_durable_obligation(self):
        """Return the committed durable plan owned by the active route.

        WorkItem failures used to have a dedicated bridge into the graph miss
        tracker, while PortalProbe and Portal edge failures stopped at their
        semantic ledgers.  Resolve the committed transaction before mutable
        route cleanup so every durable obligation type can share the same
        canonical exclusion path.
        """
        if not getattr(self, "graph_route_planner_enabled", False):
            return None
        work_item = self._graph_route_plan_for_active_work_item()
        if work_item is not None:
            return work_item

        def positive(value):
            try:
                value = int(value)
            except (TypeError, ValueError):
                return None
            return value if value > 0 else None

        active_probe_id = positive(
            getattr(self, "active_portal_probe_id", None)
        )
        active_portal_id = positive(
            getattr(self, "last_portal_hypothesis_id", None)
        )
        transaction = getattr(self, "graph_route_action_transaction", None)
        candidates = (
            getattr(transaction, "plan", None),
            getattr(self, "last_graph_route_plan", None),
        )
        seen = set()
        for plan in candidates:
            if plan is None or id(plan) in seen:
                continue
            seen.add(id(plan))
            if getattr(plan, "status", None) != PLAN_READY:
                continue
            kind = str(getattr(plan, "obligation_kind", "") or "")
            if kind not in {"portal_probe", "portal_edge", "unbound_portal"}:
                continue
            obligation_id = positive(getattr(plan, "obligation_id", None))
            first_portal_id = positive(getattr(plan, "first_portal_id", None))
            if kind == "portal_probe" and active_probe_id is not None:
                if obligation_id != active_probe_id:
                    continue
            if kind in {"portal_edge", "unbound_portal"} and active_portal_id is not None:
                if obligation_id not in {None, active_portal_id} and (
                    first_portal_id != active_portal_id
                ):
                    continue
            return plan
        return None

    def _record_active_durable_materialization_failure(
        self, plan=None, map_epoch=None, reason="active_durable_route_failure"
    ):
        """Feed an active durable route failure into canonical exclusion."""
        if plan is None:
            plan = self._graph_route_plan_for_active_durable_obligation()
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        if str(getattr(plan, "obligation_kind", "") or "") not in {
            "work_item", "portal_probe", "portal_edge", "unbound_portal",
        }:
            return False
        record_miss = getattr(
            self, "_record_graph_route_materialization_miss", None
        )
        if not callable(record_miss):
            return False
        exhausted = bool(
            record_miss(
                plan,
                map_epoch,
                "active_durable_route_failure:%s" % str(reason or "failure"),
            )
        )
        if not exhausted:
            retain_lease = getattr(self, "_retain_graph_route_plan_lease", None)
            if callable(retain_lease):
                retain_lease(plan, "active_durable_route_failure")
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "active_durable_materialization_failure_recorded",
                obligation_key=list(
                    self._graph_route_materialization_obligation_key(plan)
                ),
                obligation_kind=str(getattr(plan, "obligation_kind", "") or ""),
                obligation_id=getattr(plan, "obligation_id", None),
                action=str(getattr(plan, "action", "") or ""),
                reason=str(reason or "active_durable_route_failure"),
                exhausted=exhausted,
            )
        return exhausted

    def _record_graph_route_materialization_miss(
        self, plan, map_epoch=None, reason="graph_route_materialization_wait"
    ):
        """Count materialization misses and force a bounded next action."""
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        obligation_key = self._graph_route_materialization_obligation_key(plan)
        if obligation_key is None:
            return False
        transaction = getattr(self, "graph_route_action_transaction", None)
        transaction_epoch = (
            None if transaction is None else transaction.map_epoch
        )
        epoch = self._normalise_graph_map_epoch(map_epoch)
        if epoch is None:
            epoch = self._normalise_graph_map_epoch(transaction_epoch)

        # The budget belongs to the durable obligation, not to whichever
        # obligation happened to be selected immediately before it.  Loading a
        # different key must therefore switch the diagnostic view without
        # deleting the previous key's history.
        tracker = self._load_graph_materialization_tracking(obligation_key)
        if tracker is None:
            return False
        epochs = list(tracker.get("epochs", ()) or ())
        observation_key = (obligation_key, epoch)
        observation_is_new = observation_key != tracker.get("last_observation")
        if observation_is_new:
            tracker["miss_count"] = int(tracker.get("miss_count", 0) or 0) + 1
            if epoch is not None and epoch not in epochs:
                epochs.append(epoch)
            tracker["last_observation"] = observation_key
            if tracker.get("first_epoch") is None:
                tracker["first_epoch"] = epoch
            tracker["last_epoch"] = epoch
        tracker["epochs"] = epochs
        epoch_span = (
            0
            if len(epochs) < 2
            else max(epochs) - min(epochs)
        )
        self._load_graph_materialization_tracking(obligation_key)
        miss_count = int(tracker.get("miss_count", 0) or 0)
        identity = self._graph_route_materialization_identity(
            plan=plan, map_epoch=epoch, transaction=transaction
        )
        self._transition_graph_route_fsm(
            _GRAPH_ROUTE_FSM_WAITING,
            transaction_id=identity["transaction_id"],
            map_epoch=epoch,
            reason=str(reason),
        )
        event_key = (obligation_key, epoch, str(reason))
        if event_key != tracker.get("last_event"):
            tracker["last_event"] = event_key
            self.graph_route_materialization_last_event = event_key
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "graph_route_materialization_miss",
                    **identity,
                    obligation_key=list(obligation_key),
                    action=str(getattr(plan, "action", "") or ""),
                    obligation_kind=str(
                        getattr(plan, "obligation_kind", "") or ""
                    ),
                    obligation_id=getattr(plan, "obligation_id", None),
                    first_portal_id=getattr(plan, "first_portal_id", None),
                    miss_count=miss_count,
                    observed_epochs=list(epochs),
                    epoch_span=int(epoch_span),
                    max_misses=_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES,
                    max_epoch_span=_GRAPH_ROUTE_MATERIALIZATION_MAX_EPOCH_SPAN,
                    reason=str(reason),
                )
        rospy.logwarn(
            "Global frontier graph materialization miss "
            "route_id=%s transaction_id=%s action_generation=%s "
            "graph_transaction_id=%s map_epoch=%s origin_epoch=%s "
            "projection_epoch=%s count=%d epoch_span=%d obligation=%s "
            "portal=%s reason=%s",
            identity["route_id"],
            identity["transaction_id"],
            identity["action_generation"],
            identity["graph_transaction_id"],
            identity["map_epoch"],
            identity["origin_epoch"],
            identity["projection_epoch"],
            miss_count,
            int(epoch_span),
            getattr(plan, "obligation_id", None),
            getattr(plan, "first_portal_id", None),
            str(reason),
        )
        if (
            miss_count >= _GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES
            or epoch_span >= _GRAPH_ROUTE_MATERIALIZATION_MAX_EPOCH_SPAN
        ):
            return self._invalidate_graph_route_for_materialization(
                plan,
                epoch,
                "graph_route_materialization_expired",
                miss_count,
                epoch_span,
            )
        return False

    @staticmethod
    def _positive_ids(values):
        """Normalize a snapshot candidate-ID collection for lease checks."""
        if values is None:
            return None
        if isinstance(values, dict):
            values = values.keys()
        try:
            iterator = iter(values)
        except TypeError:
            iterator = iter((values,))
        result = set()
        for value in iterator:
            try:
                value = int(value)
            except (TypeError, ValueError):
                continue
            if value > 0:
                result.add(value)
        return result

    def _graph_snapshot_projection_is_current(self, map_epoch=None):
        """Return whether the current map has completed identity projection."""
        if not bool(
            getattr(self, "last_graph_snapshot_projection_complete", False)
        ):
            return False
        projected_epoch = getattr(self, "last_graph_snapshot_epoch", None)
        if map_epoch is None or projected_epoch is None:
            return False
        try:
            return int(projected_epoch) == int(map_epoch)
        except (TypeError, ValueError):
            return str(projected_epoch).strip() == str(map_epoch).strip()

    def _graph_snapshot_projection_ids(self):
        """Return the identity sets produced by the current map snapshot."""
        return (
            self._positive_ids(
                getattr(self, "last_graph_snapshot_visible_work_item_ids", ())
            )
            or set(),
            self._positive_ids(
                getattr(self, "last_graph_snapshot_visible_probe_ids", ())
            )
            or set(),
            self._positive_ids(
                getattr(self, "last_graph_snapshot_visible_portal_ids", ())
            )
            or set(),
        )

    def _graph_plan_identity_is_visible(
        self, plan, visible_work_item_ids=None, visible_probe_ids=None,
        visible_portal_ids=None,
    ):
        """Check that a ready plan names an identity in this map snapshot."""
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        action = str(getattr(plan, "action", "") or "")
        obligation_kind = str(
            getattr(plan, "obligation_kind", "") or ""
        )
        if action in {
            ACTION_BOOTSTRAP,
            ACTION_OBSERVE_LOCAL_WORK,
            ACTION_RETRY_VIEWPOINT,
        }:
            ids = self._positive_ids(visible_work_item_ids)
            obligation = self._positive_ids(
                (getattr(plan, "obligation_id", None),)
            )
            return bool(
                ids is not None
                and obligation
                and next(iter(obligation)) in ids
            )
        if action == ACTION_PROBE_PORTAL and obligation_kind == "portal_probe":
            ids = self._positive_ids(visible_probe_ids)
            obligation = self._positive_ids(
                (getattr(plan, "obligation_id", None),)
            )
            return bool(
                ids is not None
                and obligation
                and next(iter(obligation)) in ids
            )
        if action in {ACTION_PROBE_PORTAL, ACTION_CROSS_PORTAL}:
            ids = self._positive_ids(visible_portal_ids)
            portal = self._positive_ids(
                (getattr(plan, "first_portal_id", None),)
            )
            return bool(
                ids is not None
                and portal
                and next(iter(portal)) in ids
            )
        if action == "reinspect_target":
            return bool(
                self._positive_ids(visible_work_item_ids)
                or self._positive_ids(visible_probe_ids)
                or self._positive_ids(visible_portal_ids)
            )
        return False

    def _graph_plan_can_reuse_with_candidates(
        self, plan, visible_work_item_ids=None, visible_probe_ids=None,
        visible_portal_ids=None,
    ):
        """Keep a lease while its identity or miss budget remains durable.

        A route lease protects an obligation from transient SLAM churn.  It
        must not freeze a fresh plan that candidate collection has shown cannot
        be materialized while another same-place identity is executable. Once
        the plan has a recorded miss, however, the missing identity is the
        evidence being measured and the lease must survive its empty snapshot.
        None means projection has not run; an empty set means projection
        completed and found no executable identity.
        """
        if plan is None:
            return False
        action = str(getattr(plan, "action", "") or "")
        if not self._graph_plan_probe_phase_matches_durable_state(plan):
            return False
        # Once a ready obligation has recorded a miss, its lease is the
        # durable retry boundary. Candidate visibility is deliberately not a
        # requirement in this state: an empty projection is the fact being
        # counted, and releasing the lease would replace the obligation before
        # its miss budget can expire.
        tracked_key = getattr(
            self, "graph_route_materialization_obligation_key", None
        )
        current_key = self._graph_route_materialization_obligation_key(plan)
        if (
            tracked_key is not None
            and current_key is not None
            and tuple(tracked_key) == tuple(current_key)
            and int(
                getattr(self, "graph_route_materialization_miss_count", 0)
                or 0
            ) > 0
            and not bool(
                getattr(self, "graph_route_materialization_invalidated", False)
            )
        ):
            return True
        if action in {"bootstrap_observation", "observe_local_work", "retry_viewpoint"}:
            ids = self._positive_ids(visible_work_item_ids)
            obligation = self._positive_ids((getattr(plan, "obligation_id", None),))
            obligation = next(iter(obligation), None)
        elif (
            action == ACTION_PROBE_PORTAL
            and str(getattr(plan, "obligation_kind", "") or "")
            == "portal_probe"
        ):
            ids = self._positive_ids(visible_probe_ids)
            obligation = self._positive_ids((getattr(plan, "obligation_id", None),))
            obligation = next(iter(obligation), None)
        elif action in {ACTION_PROBE_PORTAL, ACTION_CROSS_PORTAL}:
            ids = self._positive_ids(visible_portal_ids)
            obligation = self._positive_ids(
                (getattr(plan, "first_portal_id", None),)
            )
            obligation = next(iter(obligation), None)
        else:
            return True
        if ids is None:
            return True
        return obligation is not None and obligation in ids

    def _graph_plan_probe_phase_matches_durable_state(self, plan):
        """Reject a probe lease after its evidence transaction changes phase.

        A source-side probe finishing is a durable state transition, not map
        churn.  Reusing the old source plan after that transition makes the
        materializer construct a destination candidate against a source
        contract.  The resulting mismatch is avoidable and can cause an
        unnecessary controller handoff.  Keep legacy plans/ledgers without
        phase provenance permissive; production records carry the state needed
        to make the boundary explicit.
        """
        if plan is None or str(getattr(plan, "action", "") or "") != "probe_portal":
            return True
        if str(getattr(plan, "obligation_kind", "") or "") != "portal_probe":
            return True
        expected = str(getattr(plan, "portal_probe_phase", "") or "").strip().lower()
        if not expected:
            return True
        ledger = getattr(self, "portal_probe_ledger", None)
        get_record = getattr(ledger, "get", None)
        if not callable(get_record):
            return True
        try:
            record = get_record(getattr(plan, "obligation_id", None))
        except (TypeError, ValueError):
            return True
        if not isinstance(record, dict):
            return True
        state = str(record.get("state", "") or "").strip().lower()
        if expected == "source":
            # ``active`` is the source-side execution state.  A destination
            # attempt has its own explicit ``destination_active`` state.
            if state == "awaiting_projection":
                state = str(
                    record.get("projection_resume_state", "") or ""
                ).strip().lower()
            return state in {"pending", "active"}
        if expected == "destination":
            if state == "awaiting_projection":
                state = str(
                    record.get("projection_resume_state", "") or ""
                ).strip().lower()
            return state in {"source_arrived", "destination_active"}
        return True

    def _publish_graph_route_unavailable(
        self, plan, reason, map_epoch=None, controller_release_allowed=True
    ):
        """Publish a controller handoff when a ready graph route has no edge.

        ``graph_route_plan_lease_active`` protects the durable obligation from
        snapshot churn, but it does not own the already-running MoveBase
        action.  Once a ready plan has been checked against the current
        snapshot and no candidate can materialize, the controller must receive
        an explicit release record.  Keep the selected route identity in that
        record so a bridge cannot infer a substitute from the goal coordinates.
        """
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        # A pre-admitted successor is still a controller handoff path.  This
        # helper is reserved for the stronger case where the graph edge has no
        # materialized successor at all; let the normal terminal promotion
        # consume the cached successor instead.
        if getattr(self, "prefetched_frontier", None) is not None:
            return False
        try:
            route_id = max(0, int(getattr(self, "active_route_id", 0) or 0))
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            # There is no controller lease to release during startup or after
            # a mission has explicitly stopped. Keep this a planning-only
            # diagnostic in that state.
            return False
        pending_release_lookup = getattr(
            self, "_pending_controller_release_for_route", None
        )
        if callable(pending_release_lookup):
            pending_release = pending_release_lookup(route_id)
        else:
            pending_release = getattr(
                self, "pending_controller_lease_release", None
            )
        try:
            pending_release_route_id = int(
                pending_release.get("route_id", 0) or 0
            ) if isinstance(pending_release, dict) else 0
        except (TypeError, ValueError):
            pending_release_route_id = 0
        last_released_route_id = int(
            getattr(self, "last_released_route_id", 0) or 0
        )
        release_pending_for_route = bool(
            (
                getattr(self, "awaiting_controller_lease_release_ack", False)
                and pending_release_route_id == route_id
            )
            or (
                pending_release_route_id == route_id
                and isinstance(pending_release, dict)
            )
            or (
                bool(
                    getattr(
                        self, "last_released_route_controller_pending", False
                    )
                )
                and last_released_route_id == route_id
            )
        )
        release_already_settled = bool(
            getattr(self, "active_frontier", None) is None
            and not release_pending_for_route
            and last_released_route_id == route_id
        )
        # A materialization expiry can follow an explicit release emitted by
        # ``_invalidate_graph_route_for_materialization``. Keep the
        # route-unavailable notification for GoalManager, but do not ask the
        # Bridge to release the same predecessor a second time. The route
        # identity is required so a settled predecessor cannot suppress a
        # release for a newer active route.
        force_release = bool(
            controller_release_allowed
            and str(reason) == "graph_route_materialization_expired"
            and not release_pending_for_route
            and not release_already_settled
        )
        signature = (
            plan.signature(),
            route_id,
            str(reason),
            bool(controller_release_allowed),
        )
        if signature == getattr(
            self, "last_graph_route_unavailable_signature", None
        ):
            return False
        self.last_graph_route_unavailable_signature = signature
        publish = getattr(self, "publish_status", None)
        if publish is None:
            return False
        if map_epoch is None:
            transaction = getattr(self, "graph_route_action_transaction", None)
            map_epoch = None if transaction is None else transaction.map_epoch
        identity = self._graph_route_materialization_identity(
            plan=plan, map_epoch=map_epoch
        )
        identity["route_id"] = route_id
        predecessor_contract = getattr(
            self, "pending_graph_successor_predecessor_contract", None
        )
        if isinstance(predecessor_contract, dict):
            predecessor_route_id = int(
                predecessor_contract.get("route_id", 0) or 0
            )
            if predecessor_route_id > 0:
                identity["predecessor_contract"] = dict(predecessor_contract)
                identity["predecessor_route_id"] = predecessor_route_id
        publish(
            "frontier_route_unavailable",
            **identity,
            successor_route_id=0,
            controller_lease=(
                "release"
                if controller_release_allowed
                else "held_until_terminal"
            ),
            route_unavailable=True,
            controller_release_allowed=bool(controller_release_allowed),
            active_route_retained=bool(not controller_release_allowed),
            force_controller_lease_release=force_release,
            controller_release_already_requested=release_pending_for_route,
            controller_release_already_settled=release_already_settled,
            action=str(plan.action),
            obligation_kind=str(plan.obligation_kind),
            obligation_id=plan.obligation_id,
            portal_path=list(plan.portal_path),
            first_portal_id=plan.first_portal_id,
            target_place_id=plan.target_place_id,
            reason=str(reason),
        )
        return True

    def _retain_graph_route_plan_lease(self, plan, reason="materialization_wait"):
        """Keep one ready durable action across transient map snapshots.

        A missing frontier projection is not evidence that the durable
        obligation changed.  The lease makes that distinction explicit and
        prevents the next planning cycle from selecting a different obligation
        merely because its cells happen to be visible first.
        """
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return False
        self.graph_route_plan_lease_active = True
        self.graph_route_plan_lease_signature = plan.signature()
        transaction = getattr(self, "graph_route_action_transaction", None)
        self._transition_graph_route_fsm(
            _GRAPH_ROUTE_FSM_WAITING,
            transaction_id=(
                0 if transaction is None else int(transaction.transaction_id)
            ),
            map_epoch=(None if transaction is None else transaction.map_epoch),
            reason=str(reason),
        )
        return True

    def _clear_graph_route_plan_lease(self, reason=""):
        """Release the durable action lease at an explicit lifecycle boundary."""
        del reason
        self.graph_route_plan_lease_active = False
        self.graph_route_plan_lease_signature = None

    def _graph_route_plan_lease_matches_current(self, plan):
        """Reject a stale lease after a physical Place transition."""
        if plan is None:
            return False
        current = getattr(self, "current_physical_place_id", None)
        planned = getattr(plan, "current_place_id", None)
        if current is None or planned is None:
            return True
        try:
            return int(current) == int(planned)
        except (TypeError, ValueError):
            return False

    def _graph_route_plan_projection_matches_current(self, plan, map_epoch=None):
        """Require a reused plan to retain the current snapshot lineage.

        A durable lease may survive a transient projection miss, but a route
        transaction created from an older completed projection must not be
        reused as the next route after a newer snapshot has completed.  The
        route geometry and its projection identity are one immutable contract;
        reusing only the durable obligation would otherwise publish a current
        outer event with an older transaction epoch.
        """
        if plan is None:
            return False
        current_epoch = self._normalise_graph_map_epoch(map_epoch)
        if current_epoch is None:
            current_epoch = self._normalise_graph_map_epoch(
                getattr(self, "last_graph_snapshot_epoch", None)
            )
        if current_epoch is None:
            return True
        transaction = getattr(self, "graph_route_action_transaction", None)
        planned_epoch = self._normalise_graph_map_epoch(
            None if transaction is None else transaction.projection_epoch
        )
        if planned_epoch is None:
            planned_epoch = self._normalise_graph_map_epoch(
                getattr(plan, "map_epoch", None)
            )
        # Compatibility fixtures may carry no provenance at all.  Production
        # transactions always carry projection_epoch or plan.map_epoch.
        if planned_epoch is None:
            return True
        return planned_epoch == current_epoch

    def _install_graph_route_preference(self, plan):
        """Install the selected identity without creating a new transaction."""
        self.graph_route_portal_id = None
        self.graph_route_probe_id = None
        self.graph_route_work_item_id = None
        self.graph_route_work_item_filter = None
        if plan is None or plan.status != PLAN_READY:
            return
        if (
            plan.action in {
                ACTION_BOOTSTRAP,
                ACTION_OBSERVE_LOCAL_WORK,
                ACTION_RETRY_VIEWPOINT,
            }
            and plan.obligation_kind == "work_item"
        ):
            try:
                work_item_id = int(plan.obligation_id)
            except (TypeError, ValueError):
                work_item_id = 0
            if work_item_id > 0:
                # The graph plan is the single durable owner. Candidate
                # construction must materialize this identity only; building
                # every local WorkItem candidate is both redundant and costly
                # once a long-running map has accumulated many boundaries.
                self.graph_route_work_item_id = work_item_id
                self.graph_route_work_item_filter = "selected"
                return
        # A non-local graph plan (Portal/probe/target) does not need local
        # WorkItem rehydration. Raw frontier and dedicated Portal collectors
        # still run so explicit graph promotions can be materialized.
        self.graph_route_work_item_filter = "none"
        first_id = plan.first_portal_id
        if first_id is None:
            return
        try:
            first_id = int(first_id)
        except (TypeError, ValueError):
            return
        if plan.action == ACTION_PROBE_PORTAL and plan.obligation_kind == "portal_probe":
            self.graph_route_probe_id = first_id
        else:
            self.graph_route_portal_id = first_id

    def _set_graph_route_preference(
        self, map_epoch=None, visible_work_item_ids=None,
        visible_probe_ids=None, visible_portal_ids=None, stage_only=False,
    ):
        """Replan after ledger reconciliation and install one edge identity.

        Selection calls this hook only after the current frontier snapshot has
        reconciled its WorkItems and Portal probes.  Keeping the preference
        installation separate from geometry materialization gives the slow
        graph layer a consistent view of the durable ledgers.
        """
        plan = self._plan_durable_graph_route(
            map_epoch=map_epoch,
            visible_work_item_ids=visible_work_item_ids,
            visible_probe_ids=visible_probe_ids,
            visible_portal_ids=visible_portal_ids,
            commit=not stage_only,
        )
        if stage_only:
            # Do not let events emitted during candidate collection inherit a
            # completed route's stale graph plan.  The staged identity lives
            # only in the preference fields until the final pass commits it.
            self.last_graph_route_plan = None
            self.last_graph_route_plan_signature = None
            self.graph_route_action_transaction = None
        self._install_graph_route_preference(plan)
        return plan

    def _plan_durable_graph_route(
        self, map_epoch=None, visible_work_item_ids=None,
        visible_probe_ids=None, visible_portal_ids=None, commit=True,
    ):
        if not getattr(self, "graph_route_planner_enabled", False):
            self.last_graph_route_plan = None
            self.graph_route_portal_id = None
            self.graph_route_probe_id = None
            self.graph_route_work_item_id = None
            self.graph_route_work_item_filter = None
            self.graph_route_action_transaction = None
            return None
        planner = getattr(self, "graph_route_planner", None)
        current_place_id = getattr(self, "current_physical_place_id", None)
        if planner is None or current_place_id is None:
            self.graph_route_work_item_id = None
            self.graph_route_work_item_filter = None
            return None
        resume_place_id = getattr(self, "pending_graph_resume_place_id", None)
        try:
            if resume_place_id is not None:
                resume_place_id = int(resume_place_id)
        except (TypeError, ValueError):
            resume_place_id = None
        if resume_place_id is not None and resume_place_id == int(current_place_id):
            self.pending_graph_resume_place_id = None
            self.pending_graph_resume_portal_id = None
            self.pending_graph_resume_destination_place_id = None
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "graph_place_resume_completed",
                    place_id=int(current_place_id),
                    reason="covered_portal_source_reached",
                )
            resume_place_id = None
        work_ledger = getattr(self, "place_work_items", None)
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        portal_ledger = getattr(self, "portal_hypothesis_ledger", None)
        target_work = getattr(self, "target_observation_work", None)
        if visible_work_item_ids is None:
            context = getattr(self, "last_selection_context", None)
            visible_work_item_ids = (
                None
                if context is None
                else getattr(context, "work_item_details", None)
            )
        target_work_pending = bool(
            target_work is not None
            and callable(getattr(target_work, "has_pending", None))
            and target_work.has_pending(current_place_id)
        )
        # The reinspection event is authoritative even while the target
        # WorkItem is still crossing the asynchronous GoalManager/global-
        # frontier boundary. Once bound, the ledger independently preserves
        # the same obligation across detector gaps.
        target_pending = bool(
            target_work_pending
            or getattr(self, "target_reinspection_pending", False)
        )
        if target_pending and self._target_observation_materialization_deferred(
            current_place_id
        ):
            # Preserve the unresolved target WorkItem for completion accounting,
            # but do not force a route whose current snapshot has already
            # proved that no safe reinspection viewpoint exists. New target
            # evidence explicitly reopens this policy above.
            target_pending = False
        (
            excluded_portal_ids,
            excluded_work_item_ids,
            excluded_probe_ids,
        ) = self._graph_route_materialization_exclusions(map_epoch)
        map_epoch = self._normalise_graph_map_epoch(map_epoch)
        if map_epoch is None:
            map_epoch = max(
                1,
                int(
                    getattr(self, "last_map_epoch", None)
                    or getattr(self, "topology_component_epoch", 1)
                    or 1
                ),
            )
        plan = planner.plan(
            current_place_id,
            places=getattr(getattr(self, "region_memory", None), "regions", ()),
            portals=portal_ledger,
            work_items=work_ledger,
            probes=probe_ledger,
            branch_coverage=getattr(
                self, "directional_branch_coverage", None
            ),
            visible_work_item_ids=visible_work_item_ids,
            visible_probe_ids=visible_probe_ids,
            visible_portal_ids=visible_portal_ids,
            branch_first=bool(getattr(self, "branch_first_enabled", False)),
            target_observation_pending=target_pending,
            resume_place_id=resume_place_id,
            structural_boundary_first=bool(
                getattr(self, "branch_first_enabled", False)
            ),
            map_epoch=map_epoch,
            excluded_portal_ids=excluded_portal_ids,
            excluded_work_item_ids=excluded_work_item_ids,
            excluded_probe_ids=excluded_probe_ids,
        )
        if getattr(plan, "map_epoch", None) != map_epoch:
            plan = replace(plan, map_epoch=map_epoch)
        if not commit:
            # This is the read-only first phase of snapshot planning.  It
            # supplies the candidate collector with one durable identity
            # preference, but leaves the previous committed plan/transaction
            # untouched until all current-snapshot candidates are reconciled.
            return plan
        self.last_graph_route_plan = plan
        transaction_id = int(
            getattr(self, "graph_route_action_transaction_sequence", 0)
        ) + 1
        self.graph_route_action_transaction_sequence = transaction_id
        self.graph_route_action_transaction = GraphRouteActionTransaction(
            transaction_id=transaction_id,
            plan=plan,
            map_epoch=map_epoch,
            # The durable plan is born from the same snapshot that supplies
            # the current projection. Keep both names explicit on the
            # transaction so downstream consumers never have to infer lineage
            # from the mutable topology counter.
            origin_epoch=map_epoch,
            projection_epoch=(
                getattr(self, "last_graph_snapshot_epoch", None)
                or map_epoch
            ),
        )
        self._transition_graph_route_fsm(
            _GRAPH_ROUTE_FSM_PLANNING,
            transaction_id=transaction_id,
            map_epoch=map_epoch,
            reason="graph_route_plan_selected",
        )
        if map_epoch is None:
            log_error = getattr(rospy, "logerr", None)
            if not callable(log_error):
                log_error = getattr(rospy, "logwarn", None)
            if callable(log_error):
                log_error(
                    "Global frontier graph route transaction missing "
                    "map epoch transaction_id=%s epoch=None",
                    transaction_id,
                )
        signature = plan.signature()
        if signature != getattr(self, "last_graph_route_plan_signature", None):
            self.last_graph_route_plan_signature = signature
            publish = getattr(self, "publish_status", None)
            if publish is not None:
                plan_fields = plan.as_dict()
                # ``GraphRoutePlan.as_dict`` already contains map_epoch. Build
                # the event payload once so the reporting boundary cannot
                # receive the same keyword through both ``**plan_fields`` and
                # an explicit argument.
                plan_fields.update({
                    "transaction_id": transaction_id,
                    "map_epoch": map_epoch,
                })
                publish(
                    "graph_route_plan_selected",
                    **plan_fields,
                )
        return plan

    def _prepare_graph_route_action(
        self, snapshot, *, visible_work_item_ids=None, visible_probe_ids=None,
        visible_portal_ids=None, reuse_existing_plan=False, stage_only=False,
        map_epoch=None,
    ):
        """Reserve a planner-selected edge for the next selection pass.

        The returned candidate is only used for a durable reverse egress whose
        current map no longer exposes a normal frontier.  Forward edges and
        source-side probes retain a preferred identity and are materialized by
        the ordinary portal selector on the same snapshot.
        """
        if map_epoch is None:
            map_epoch = self._graph_snapshot_map_epoch(snapshot)
        # A live snapshot must first pass through the current candidate
        # projection. The dry-run path (snapshot=None) is used only after
        # selection has explicitly recorded that projection.
        if (
            snapshot is not None
            and not self._graph_snapshot_projection_is_current(map_epoch)
        ):
            return getattr(self, "last_graph_route_plan", None), None
        existing = getattr(self, "last_graph_route_plan", None)
        lease_active = bool(
            getattr(self, "graph_route_plan_lease_active", False)
        )
        lease_identity_is_present = self._graph_plan_can_reuse_with_candidates(
            existing,
            visible_work_item_ids=visible_work_item_ids,
            visible_probe_ids=visible_probe_ids,
            visible_portal_ids=visible_portal_ids,
        )
        projection_lineage_is_current = (
            self._graph_route_plan_projection_matches_current(
                existing, map_epoch=map_epoch
            )
        )
        if (
            reuse_existing_plan
            and existing is not None
            and lease_active
            and lease_identity_is_present
            # A cleared lease does not make a plan current.  Portal arrival
            # clears the old lease exactly when physical Place ownership
            # changes; reusing that plan would dispatch reverse transit from
            # the previous Place before its new local ledger is reconciled.
            and self._graph_route_plan_lease_matches_current(existing)
            and projection_lineage_is_current
        ):
            # Candidate visibility is a transient projection, not a durable
            # ownership boundary. Once a plan has a lease, keep its identity
            # through Navfn/materialization waits even if this pass did not
            # project that WorkItem.
            plan = existing
            self._install_graph_route_preference(plan)
        else:
            if reuse_existing_plan:
                if not projection_lineage_is_current:
                    reason = "stale_plan_projection_lineage"
                    publish = getattr(self, "publish_status", None)
                    if callable(publish):
                        transaction = getattr(
                            self, "graph_route_action_transaction", None
                        )
                        publish(
                            "graph_route_plan_rebase_required",
                            route_id=int(
                                getattr(self, "active_route_id", 0) or 0
                            ),
                            graph_transaction_id=int(
                                getattr(
                                    transaction,
                                    "graph_transaction_id",
                                    0,
                                )
                                or 0
                            ),
                            previous_projection_epoch=(
                                None
                                if transaction is None
                                else transaction.projection_epoch
                            ),
                            current_projection_epoch=(
                                self._normalise_graph_map_epoch(map_epoch)
                            ),
                            reason=reason,
                        )
                else:
                    reason = (
                        "stale_or_missing_plan"
                        if lease_identity_is_present
                        else "leased_obligation_not_in_current_candidates"
                    )
                self._clear_graph_route_plan_lease(reason)
            plan = self._set_graph_route_preference(
                map_epoch=map_epoch,
                visible_work_item_ids=visible_work_item_ids,
                visible_probe_ids=visible_probe_ids,
                visible_portal_ids=visible_portal_ids,
                stage_only=stage_only,
            )
        if plan is None or plan.status != PLAN_READY:
            self._clear_graph_route_plan_lease("plan_not_ready")
            return plan, None
        # ``None`` is the deliberate dry-run used after frontier reconciliation
        # and before score-pool selection.  It installs the graph identity but
        # does not attempt to consume a map snapshot twice.
        if snapshot is None:
            return plan, None
        first_id = plan.first_portal_id
        if first_id is None:
            return plan, None
        try:
            first_id = int(first_id)
        except (TypeError, ValueError):
            return plan, None

        if plan.action == ACTION_PROBE_PORTAL and plan.obligation_kind == "portal_probe":
            probe_ledger = getattr(self, "portal_probe_ledger", None)
            probe = (
                None
                if probe_ledger is None
                else probe_ledger.get(first_id)
            )
            if probe is not None:
                probe_state = str(
                    probe.get("state", "") or ""
                ).strip().lower()
                blocked_epoch = self._normalise_graph_map_epoch(
                    probe.get("projection_blocked_map_epoch")
                )
                current_epoch = self._normalise_graph_map_epoch(map_epoch)
                projection_wait_current = (
                    probe_state == "awaiting_projection"
                    and (
                        current_epoch is None
                        or blocked_epoch is None
                        or current_epoch <= blocked_epoch
                    )
                )
                self.graph_route_probe_id = first_id
                if projection_wait_current:
                    # The same map snapshot cannot provide new geometry. Keep
                    # the durable lease and let the miss tracker observe a
                    # later epoch instead of switching obligations here.
                    candidate = None
                else:
                    select_probe = getattr(
                        self, "select_durable_portal_probe", None
                    )
                    candidate = (
                        None
                        if select_probe is None
                        else select_probe(snapshot)
                    )
                if candidate is not None:
                    if not self._commit_graph_route_candidate(
                        plan, candidate, source="durable_probe"
                    ):
                        return plan, None
                    self.graph_route_probe_id = None
                    self._publish_graph_route_materialized(
                        self.last_graph_route_plan, candidate
                    )
                    return plan, candidate
                refreshed_probe = probe_ledger.get(first_id)
                if isinstance(refreshed_probe, dict) and str(
                    refreshed_probe.get("state", "") or ""
                ).strip().lower() == "awaiting_projection":
                    # A projection miss is a retryable observation of this
                    # durable obligation. The next epoch must reach this same
                    # branch so the miss budget can expire deterministically.
                    retain_lease = getattr(
                        self, "_retain_graph_route_plan_lease", None
                    )
                    if callable(retain_lease):
                        retain_lease(plan, "probe_projection_unavailable")
                    return plan, None
                return plan, None

        # An unbound hypothesis or a crossed edge is selected by the portal
        # layer.  A certified unbound edge may be materialized directly from
        # physical Portal memory even when its frontier projection vanished;
        # reverse traversal remains owned by durable egress.
        self.graph_route_portal_id = first_id
        record = getattr(
            getattr(self, "portal_hypothesis_ledger", None), "get", lambda _id: None
        )(first_id)
        current = getattr(self, "current_physical_place_id", None)
        reverse = bool(
            isinstance(record, dict)
            and current is not None
            and record.get("destination_place_id") is not None
            and int(record.get("destination_place_id")) == int(current)
            and int(record.get("source_place_id", -1)) != int(current)
        )
        if reverse:
            select_egress = getattr(self, "select_durable_portal_egress", None)
            candidate = None if select_egress is None else select_egress(snapshot)
            if candidate is not None:
                if not self._commit_graph_route_candidate(
                    plan, candidate, source="durable_egress"
                ):
                    return plan, None
                self.graph_route_portal_id = None
                self._publish_graph_route_materialized(
                    self.last_graph_route_plan, candidate
                )
                return plan, candidate
            # Reverse egress is a graph-owned edge just like forward crossing.
            # If its snapshot-local materializer reports a miss, keep the
            # durable plan leased and let the common bounded miss tracker own
            # negative evidence, invalidation, release, and successor gating.
            unavailable = getattr(
                self, "last_durable_portal_egress_unavailable", None
            )
            unavailable_epoch = (
                None
                if not isinstance(unavailable, dict)
                else unavailable.get("map_epoch")
            )
            current_epoch = self._graph_snapshot_map_epoch(snapshot)
            try:
                same_epoch = (
                    isinstance(unavailable, dict)
                    and int(unavailable.get("portal_id")) == int(first_id)
                    and unavailable_epoch is not None
                    and current_epoch is not None
                    and int(unavailable_epoch) == int(current_epoch)
                )
            except (TypeError, ValueError):
                same_epoch = (
                    isinstance(unavailable, dict)
                    and str(unavailable.get("portal_id")) == str(first_id)
                    and str(unavailable_epoch) == str(current_epoch)
                )
            if same_epoch:
                reason = str(
                    unavailable.get("reason")
                    or "portal_egress_materialization_unavailable"
                )
                record_miss = getattr(
                    self, "_record_graph_route_materialization_miss", None
                )
                exhausted = False
                if callable(record_miss):
                    exhausted = bool(
                        record_miss(
                            plan,
                            current_epoch,
                            "portal_egress_materialization_unavailable:%s"
                            % reason,
                        )
                    )
                if exhausted:
                    return None, None
                retain_lease = getattr(
                    self, "_retain_graph_route_plan_lease", None
                )
                if callable(retain_lease):
                    retain_lease(plan, "portal_egress_%s" % reason)
                publish = getattr(self, "publish_status", None)
                if callable(publish):
                    identity = self._graph_route_materialization_identity(
                        plan=plan,
                        map_epoch=current_epoch,
                        transaction=getattr(
                            self, "graph_route_action_transaction", None
                        ),
                    )
                    publish(
                        "graph_route_materialization_retry_deferred",
                        **identity,
                        previous_portal_id=first_id,
                        reason=reason,
                        retry_on_new_map_epoch=True,
                    )
                return plan, None
        if (
            isinstance(record, dict)
            and record.get("destination_place_id") is None
            and str(record.get("state", "")).strip().lower()
            in ("certified", "selected")
        ):
            select_crossing = getattr(
                self, "select_durable_portal_crossing", None,
            )
            candidate = (
                None
                if select_crossing is None
                else select_crossing(snapshot)
            )
            if candidate is not None:
                if not self._commit_graph_route_candidate(
                    plan, candidate, source="durable_crossing"
                ):
                    return plan, None
                self.graph_route_portal_id = None
                self._publish_graph_route_materialized(
                    self.last_graph_route_plan, candidate
                )
                return plan, candidate
            # The crossing materializer can prove that this physical Portal
            # cannot be represented in the current map/costmap snapshot. The
            # egress helper records epoch-scoped negative evidence. Keep the
            # durable graph plan leased and let the miss tracker decide when
            # this obligation is exhausted; one local projection miss must not
            # silently replace the graph obligation.
            unavailable = getattr(
                self, "last_durable_portal_crossing_unavailable", None
            )
            unavailable_epoch = (
                None if not isinstance(unavailable, dict)
                else unavailable.get("map_epoch")
            )
            current_epoch = self._graph_snapshot_map_epoch(snapshot)
            try:
                same_epoch = (
                    unavailable is not None
                    and int(unavailable.get("portal_id")) == int(first_id)
                    and unavailable_epoch is not None
                    and current_epoch is not None
                    and int(unavailable_epoch) == int(current_epoch)
                )
            except (TypeError, ValueError):
                same_epoch = (
                    isinstance(unavailable, dict)
                    and str(unavailable.get("portal_id")) == str(first_id)
                    and str(unavailable_epoch) == str(current_epoch)
                )
            if same_epoch:
                reason = str(
                    unavailable.get("reason") or
                    "portal_materialization_unavailable"
                )
                # Keep the state transition at the graph/action boundary as
                # well as in the geometry helper. Test adapters and older
                # materializers may only report the reason; marking here
                # prevents another geometry attempt on this map epoch.
                mark_rejected = getattr(
                    getattr(self, "portal_hypothesis_ledger", None),
                    "mark_unbound_rejected",
                    None,
                )
                if callable(mark_rejected):
                    mark_rejected(first_id, current_epoch, reason)
                record_miss = getattr(
                    self, "_record_graph_route_materialization_miss", None
                )
                exhausted = False
                if callable(record_miss):
                    exhausted = bool(
                        record_miss(
                            plan,
                            current_epoch,
                            "portal_materialization_unavailable:%s" % reason,
                        )
                    )
                if exhausted:
                    return None, None
                retain_lease = getattr(
                    self, "_retain_graph_route_plan_lease", None
                )
                if callable(retain_lease):
                    retain_lease(plan, "portal_%s" % reason)
                publish = getattr(self, "publish_status", None)
                if publish is not None:
                    identity = self._graph_route_materialization_identity(
                        plan=plan,
                        map_epoch=current_epoch,
                        transaction=getattr(
                            self, "graph_route_action_transaction", None
                        ),
                    )
                    publish(
                        "graph_route_materialization_retry_deferred",
                        **identity,
                        previous_portal_id=first_id,
                        reason=reason,
                        retry_on_new_map_epoch=True,
                    )
                return plan, None
        return plan, None

    def _commit_graph_route_candidate(self, plan, candidate, *, source="selector"):
        """Commit one candidate through the single graph-action boundary."""
        transaction = getattr(self, "graph_route_action_transaction", None)
        if transaction is None:
            return True
        validation_candidate = candidate
        # Older injected adapters returned a short tuple from the durable
        # probe/egress helpers.  Preserve that public tuple while supplying
        # the graph action to the transaction validator; production routes
        # already carry the explicit action fields.
        if candidate_action(candidate) is None and str(source).startswith("durable"):
            route = list(tuple(candidate or ()))
            while len(route) <= 18:
                route.append(None)
            route[17] = str(transaction.plan.action)
            route[18] = "legacy_durable_candidate"
            validation_candidate = tuple(route)
        elif candidate_action(candidate) == "cross_portal" and len(
            tuple(candidate or ())
        ) <= 19:
            # The live portal selector records the chosen durable ID even for
            # older route tuples that predate the trailing identity field.
            # Supply that identity to validation without changing the public
            # tuple returned to activation.
            portal_id = getattr(self, "last_portal_hypothesis_id", None)
            try:
                portal_id = int(portal_id)
            except (TypeError, ValueError):
                portal_id = None
            if portal_id is not None and portal_id > 0:
                route = list(tuple(candidate))
                while len(route) <= 18:
                    route.append(None)
                route.append(portal_id)
                validation_candidate = tuple(route)
        materialization = materialize_graph_route_action(
            plan,
            validation_candidate,
            branch_first=bool(getattr(self, "branch_first_enabled", False)),
        )
        if not materialization.accepted:
            publish = getattr(self, "publish_status", None)
            if publish is not None:
                publish(
                    "graph_route_plan_mismatch",
                    transaction_id=int(transaction.transaction_id),
                    map_epoch=transaction.map_epoch,
                    source=str(source),
                    planned_action=str(plan.action),
                    candidate_action=str(materialization.candidate_action),
                    planned_first_portal_id=plan.first_portal_id,
                    candidate_portal_id=materialization.candidate_portal_id,
                    planned_obligation_id=plan.obligation_id,
                    candidate_work_item_id=materialization.candidate_work_item_id,
                    reason=str(materialization.reason),
                )
            return False

        self.last_graph_route_plan = materialization.plan
        self.last_graph_route_plan_signature = materialization.plan.signature()
        self.graph_route_action_transaction = transaction.commit(materialization)
        self._reset_graph_materialization_tracking()
        self._transition_graph_route_fsm(
            _GRAPH_ROUTE_FSM_EXECUTING,
            transaction_id=int(transaction.transaction_id),
            map_epoch=transaction.map_epoch,
            reason="graph_route_candidate_committed",
        )
        # The graph transaction is committed before Navfn confirms the
        # candidate and before activation installs the controller route. Keep
        # the durable identity leased across that interval; activation clears
        # it only after the route lifecycle exists.
        retain_lease = getattr(self, "_retain_graph_route_plan_lease", None)
        if callable(retain_lease):
            retain_lease(
                materialization.plan, "candidate_committed_until_activation"
            )
        publish = getattr(self, "publish_status", None)
        if publish is not None:
            identity = self._graph_route_materialization_identity(
                plan=materialization.plan,
                map_epoch=transaction.map_epoch,
                transaction=transaction,
            )
            identity["transaction_id"] = int(transaction.transaction_id)
            if materialization.reconciled:
                publish(
                    "graph_route_plan_reconciled",
                    transaction_id=int(transaction.transaction_id),
                    map_epoch=transaction.map_epoch,
                    source=str(source),
                    planned_action=str(plan.action),
                    committed_action=str(materialization.plan.action),
                    planned_obligation_id=plan.obligation_id,
                    committed_obligation_id=materialization.plan.obligation_id,
                    planned_first_portal_id=plan.first_portal_id,
                    committed_first_portal_id=materialization.plan.first_portal_id,
                    reason=str(materialization.reason),
                )
            publish(
                "graph_route_action_committed",
                **identity,
                source=str(source),
                action=str(materialization.plan.action),
                obligation_id=materialization.plan.obligation_id,
                portal_id=materialization.candidate_portal_id,
                reconciled=bool(materialization.reconciled),
                reason=str(materialization.reason),
            )
            # Portal/probe materializers publish this richer event at their
            # physical-edge boundary below. Local WorkItem actions have no
            # such helper, so record their completed graph materialization at
            # the common transaction commit point as well.
            if materialization.plan.action not in {
                ACTION_PROBE_PORTAL,
                ACTION_CROSS_PORTAL,
            }:
                self._publish_graph_route_materialized(
                    materialization.plan, candidate
                )
        return True

    def _publish_graph_route_materialized(self, plan, candidate):
        publish = getattr(self, "publish_status", None)
        if publish is None:
            return
        route = getattr(candidate, "route", candidate)
        route = tuple(route or ())
        identity = self._graph_route_materialization_identity(plan=plan)
        publish(
            "graph_route_edge_materialized",
            **identity,
            status=plan.status,
            action=plan.action,
            target_place_id=plan.target_place_id,
            obligation_kind=plan.obligation_kind,
            obligation_id=plan.obligation_id,
            portal_path=list(plan.portal_path),
            first_portal_id=plan.first_portal_id,
            portal_probe_phase=str(
                getattr(plan, "portal_probe_phase", "") or ""
            ),
            route_kind=(route[10] if len(route) > 10 else None),
            goal=(
                None
                if len(route) <= 3
                else [round(float(route[2]), 3), round(float(route[3]), 3)]
            ),
        )


__all__ = ["GlobalFrontierGraphRouteAdapterMixin"]

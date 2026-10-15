#!/usr/bin/env python3
"""Single-threaded ingress boundary for GoalManager."""

from copy import deepcopy
import json

import rospy
from std_msgs.msg import Bool

from experiment_reset_contract import decode_reset_request, publish_reset_ack
from lifecycle_manager import EventType, LifecycleManager, State
from goal_manager_frontier import GoalManagerFrontierMixin
from goal_manager_goal_output import GoalManagerGoalOutputMixin
from goal_manager_input_callbacks import GoalManagerInputCallbacksMixin
from goal_manager_scheduling import GoalManagerSchedulingMixin
from goal_manager_teb_callbacks import GoalManagerTebCallbacksMixin


class GoalManagerLifecycleMixin:
    """Gather ROS facts, then run GoalManager policy from one timer tick."""

    def _initialize_lifecycle_manager(self):
        gp = rospy.get_param
        allow_local_transactions = not bool(
            getattr(self, "global_frontier_enabled", False)
        )
        self.lifecycle_manager = LifecycleManager(
            event_handler=self._handle_lifecycle_event,
            transition_handler=self._on_lifecycle_transition,
            timeout_handler=self._on_lifecycle_timeout,
            # Online Global Frontier routes must adopt their upstream
            # lifecycle identity.  The diagnostic direct-target profile has
            # no upstream route owner, so it gets one local lifecycle epoch.
            allow_local_transactions=allow_local_transactions,
            timeouts={
                State.DISPATCHED: max(
                    1.0, float(gp("~lifecycle_dispatch_timeout", 120.0))
                ),
            },
        )
        if allow_local_transactions:
            self.lifecycle_manager.begin_transaction(initial_state=State.IDLE)
        self._lifecycle_event_transaction_id = None
        self._last_hard_reset_id = None
        self._last_hard_reset_transaction_id = 0
        self._hard_reset_count = 0

    def lifecycle_state(self):
        return self.lifecycle_manager.current_state

    def lifecycle_transaction_id(self):
        manager = getattr(self, "lifecycle_manager", None)
        if manager is None:
            return 0
        latest = getattr(manager, "latest_transaction_id", None)
        if callable(latest):
            return int(latest() or 0)
        return int(getattr(manager, "current_transaction_id", 0) or 0)

    def _enqueue_lifecycle_event(self, event_type, payload=None):
        if event_type == EventType.GLOBAL_FRONTIER_COMMAND:
            identity, reason = self._canonical_route_identity(payload)
            if reason:
                self._record_canonical_rejection(reason, identity)
                return False
            stale_reason = self._canonical_route_stale_reason(identity)
            if stale_reason:
                self._record_canonical_rejection(stale_reason, identity)
                return False
            transaction_id = identity["lifecycle_transaction_id"]
        elif event_type == EventType.GLOBAL_FRONTIER_STATUS:
            # Status is an audit stream. Only the route_command topic is
            # allowed to advance the GoalManager's canonical lifecycle.
            transaction_id = int(
                getattr(self.lifecycle_manager, "current_transaction_id", 0) or 0
            )
        else:
            transaction_id = self._external_transaction_id(payload)
            if transaction_id is not None and self.lifecycle_manager.is_stale_transaction(
                transaction_id
            ):
                self._record_canonical_rejection(
                    "lifecycle_transaction_id_below_high_water",
                    {"lifecycle_transaction_id": transaction_id},
                )
                return False
        return self.lifecycle_manager.enqueue_type(
            event_type,
            deepcopy(payload),
            transaction_id=transaction_id,
        )

    @staticmethod
    def _external_transaction_id(message):
        try:
            value = int(getattr(message, "lifecycle_transaction_id", 0) or 0)
        except (TypeError, ValueError):
            value = 0
        if value > 0:
            return value
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None
        try:
            value = int(payload.get("lifecycle_transaction_id", 0) or 0)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    @staticmethod
    def _canonical_route_identity(message):
        """Decode the identity Global Frontier owns for one route command."""
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return None, "malformed_route_command"
        if not isinstance(payload, dict) or payload.get("event") != "route_command":
            return None, "unsupported_route_command"

        def positive(name):
            try:
                value = int(payload.get(name, 0) or 0)
            except (TypeError, ValueError):
                return 0
            return value if value > 0 else 0

        identity = {
            "lifecycle_transaction_id": positive("lifecycle_transaction_id"),
            "route_id": positive("route_id"),
            "graph_transaction_id": positive("graph_transaction_id"),
            "map_epoch": positive("map_epoch"),
            "origin_epoch": positive("origin_epoch"),
            "projection_epoch": positive("projection_epoch"),
        }
        for name, value in identity.items():
            if value <= 0:
                return identity, "missing_%s" % name
        return identity, ""

    def _canonical_route_stale_reason(self, identity):
        """Return a reason when a route command is below a local high water."""
        if not isinstance(identity, dict):
            return "missing_canonical_identity"
        lifecycle = self.lifecycle_manager
        if identity["lifecycle_transaction_id"] < lifecycle.transaction_high_water_mark():
            return "lifecycle_transaction_id_below_high_water"
        high_water = (
            ("route_id", "canonical_route_high_water"),
            ("graph_transaction_id", "canonical_graph_high_water"),
            ("map_epoch", "canonical_map_epoch_high_water"),
            ("origin_epoch", "canonical_origin_epoch_high_water"),
            ("projection_epoch", "canonical_projection_epoch_high_water"),
        )
        for field, attribute in high_water:
            if identity[field] < int(getattr(self, attribute, 0) or 0):
                return "%s_below_high_water" % field
        return ""

    def _record_canonical_rejection(self, reason, identity=None):
        """Record and discard an invalid or stale upstream route command."""
        identity = identity if isinstance(identity, dict) else {}
        event = (
            "stale_rejected"
            if "below_high_water" in str(reason)
            else "canonical_identity_rejected"
        )
        publish = getattr(self, "publish_goal_arbitration", None)
        if callable(publish):
            publish(
                event,
                reason=str(reason),
                received_lifecycle_transaction_id=int(
                    identity.get("lifecycle_transaction_id", 0) or 0
                ),
                received_route_id=int(identity.get("route_id", 0) or 0),
                received_graph_transaction_id=int(
                    identity.get("graph_transaction_id", 0) or 0
                ),
                received_map_epoch=int(identity.get("map_epoch", 0) or 0),
                received_origin_epoch=int(
                    identity.get("origin_epoch", 0) or 0
                ),
                received_projection_epoch=int(
                    identity.get("projection_epoch", 0) or 0
                ),
            )
        rospy.logwarn(
            "GoalManager discarded canonical route command: event=%s reason=%s "
            "lifecycle=%s route=%s graph=%s map=%s",
            event,
            reason,
            identity.get("lifecycle_transaction_id", 0),
            identity.get("route_id", 0),
            identity.get("graph_transaction_id", 0),
            identity.get("map_epoch", 0),
        )

    # Gather-only ROS callbacks.
    def on_state(self, message):
        return self._enqueue_lifecycle_event(EventType.STATE_UPDATED, message)

    def on_dets(self, message):
        return self._enqueue_lifecycle_event(
            EventType.DETECTIONS_UPDATED, message
        )

    def on_scores(self, message):
        return self._enqueue_lifecycle_event(EventType.SCORES_UPDATED, message)

    def on_task(self, message):
        return self._enqueue_lifecycle_event(EventType.TASK_UPDATED, message)

    def on_pose(self, message):
        return self._enqueue_lifecycle_event(EventType.POSE_UPDATED, message)

    def on_map(self, message):
        """Queue map snapshots used to version target-route validation."""
        return self._enqueue_lifecycle_event(EventType.MAP_UPDATED, message)

    def on_cam_info(self, message):
        return self._enqueue_lifecycle_event(
            EventType.CAMERA_INFO_UPDATED, message
        )

    def on_depth(self, message):
        return self._enqueue_lifecycle_event(EventType.DEPTH_UPDATED, message)

    def on_frontier(self, message):
        return self._enqueue_lifecycle_event(EventType.FRONTIER_UPDATED, message)

    def on_frontiers(self, message):
        return self._enqueue_lifecycle_event(
            EventType.FRONTIERS_UPDATED, message
        )

    def on_controller_mode(self, message):
        return self._enqueue_lifecycle_event(
            EventType.CONTROLLER_MODE_UPDATED, message
        )

    def on_scan(self, message):
        return self._enqueue_lifecycle_event(EventType.SCAN_UPDATED, message)

    def on_cmd_vel(self, message):
        return self._enqueue_lifecycle_event(EventType.CMD_VEL_UPDATED, message)

    def on_access_mode(self, message):
        return self._enqueue_lifecycle_event(
            EventType.ACCESS_MODE_UPDATED, message
        )

    def on_access_goal(self, message):
        return self._enqueue_lifecycle_event(
            EventType.ACCESS_GOAL_UPDATED, message
        )

    def on_fixed_goal_click(self, message):
        return self._enqueue_lifecycle_event(
            EventType.FIXED_GOAL_UPDATED, message
        )

    def on_global_frontier(self, message):
        return self._enqueue_lifecycle_event(
            EventType.GLOBAL_FRONTIER_UPDATED, message
        )

    def on_global_frontier_command(self, message):
        return self._enqueue_lifecycle_event(
            EventType.GLOBAL_FRONTIER_COMMAND, message
        )

    def on_global_frontier_status(self, message):
        return self._enqueue_lifecycle_event(
            EventType.GLOBAL_FRONTIER_STATUS, message
        )

    def on_teb_goal_terminal(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TEB_GOAL_TERMINAL, message
        )

    def on_teb_goal_failure(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TEB_GOAL_FAILURE, message
        )

    def on_teb_goal_bridge_status(self, message):
        # Bridge status carries the bridge's own lifecycle identity.  It is an
        # acknowledgement for this node, not a remote transaction boundary, so
        # keep the GoalManager transaction identity local while still routing
        # the callback through the single timer-owned event queue.
        return self.lifecycle_manager.enqueue_type(
            EventType.TEB_BRIDGE_STATUS, deepcopy(message)
        )

    def on_task_done(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TASK_COMPLETED, message
        )

    def on_hard_reset(self, message):
        """Queue the reset so goal ownership is cleared by the lifecycle tick."""
        return self._enqueue_lifecycle_event(EventType.RESET, deepcopy(message))

    def on_timer(self, event):
        return self.lifecycle_manager.tick(
            compute_handler=lambda _now: GoalManagerSchedulingMixin.on_timer(
                self, event
            )
        )

    def publish_goal(self, goal, force_republish=False):
        result = GoalManagerGoalOutputMixin.publish_goal(
            self, goal, force_republish=force_republish
        )
        return result

    def request_global_frontier_replan(self, reason, **fields):
        return GoalManagerFrontierMixin.request_global_frontier_replan(
            self, reason, **fields
        )

    def _handle_lifecycle_event(self, event):
        self._lifecycle_event_transaction_id = None
        if event.type == EventType.RESET:
            return self._apply_hard_reset(event)
        if event.type == EventType.GLOBAL_FRONTIER_COMMAND:
            identity, reason = self._canonical_route_identity(event.payload)
            if reason:
                self._record_canonical_rejection(reason, identity)
                return None
            if int(event.transaction_id or 0) != identity[
                "lifecycle_transaction_id"
            ]:
                self._record_canonical_rejection(
                    "event_lifecycle_transaction_mismatch", identity
                )
                return None
            # LifecycleManager has already adopted a newer event before this
            # handler runs. Keep the exact wire ID for GoalManager output;
            # downstream nodes must see the same canonical value.
            self._lifecycle_event_transaction_id = int(event.transaction_id)
            self.global_frontier_lifecycle_transaction_id = int(
                identity["lifecycle_transaction_id"]
            )
            self.canonical_lifecycle_high_water = max(
                int(getattr(self, "canonical_lifecycle_high_water", 0) or 0),
                identity["lifecycle_transaction_id"],
            )
            self.canonical_route_high_water = max(
                int(getattr(self, "canonical_route_high_water", 0) or 0),
                identity["route_id"],
            )
            self.canonical_graph_high_water = max(
                int(getattr(self, "canonical_graph_high_water", 0) or 0),
                identity["graph_transaction_id"],
            )
            self.canonical_map_epoch_high_water = max(
                int(getattr(self, "canonical_map_epoch_high_water", 0) or 0),
                identity["map_epoch"],
            )
        handlers = {
            EventType.STATE_UPDATED: GoalManagerInputCallbacksMixin.apply_state,
            EventType.DETECTIONS_UPDATED: GoalManagerInputCallbacksMixin.apply_dets,
            EventType.SCORES_UPDATED: GoalManagerInputCallbacksMixin.apply_scores,
            EventType.POSE_UPDATED: GoalManagerInputCallbacksMixin.apply_pose,
            EventType.MAP_UPDATED: GoalManagerLifecycleMixin._apply_map_epoch,
            EventType.CAMERA_INFO_UPDATED: GoalManagerInputCallbacksMixin.apply_cam_info,
            EventType.DEPTH_UPDATED: GoalManagerInputCallbacksMixin.apply_depth,
            EventType.FRONTIER_UPDATED: GoalManagerInputCallbacksMixin.apply_frontier,
            EventType.FRONTIERS_UPDATED: GoalManagerInputCallbacksMixin.apply_frontiers,
            EventType.CONTROLLER_MODE_UPDATED: GoalManagerInputCallbacksMixin.apply_controller_mode,
            EventType.SCAN_UPDATED: GoalManagerInputCallbacksMixin.apply_scan,
            EventType.CMD_VEL_UPDATED: GoalManagerInputCallbacksMixin.apply_cmd_vel,
            EventType.ACCESS_MODE_UPDATED: GoalManagerInputCallbacksMixin.apply_access_mode,
            EventType.ACCESS_GOAL_UPDATED: GoalManagerInputCallbacksMixin.apply_access_goal,
            EventType.FIXED_GOAL_UPDATED: GoalManagerInputCallbacksMixin.apply_fixed_goal_click,
            EventType.GLOBAL_FRONTIER_UPDATED: GoalManagerFrontierMixin.apply_global_frontier,
            EventType.GLOBAL_FRONTIER_COMMAND: GoalManagerFrontierMixin.apply_global_frontier_command,
            EventType.GLOBAL_FRONTIER_STATUS: GoalManagerFrontierMixin.apply_global_frontier_status,
            EventType.TEB_GOAL_TERMINAL: GoalManagerTebCallbacksMixin.apply_teb_goal_terminal,
            EventType.TEB_GOAL_FAILURE: GoalManagerTebCallbacksMixin.apply_teb_goal_failure,
            EventType.TEB_BRIDGE_STATUS: GoalManagerTebCallbacksMixin.apply_teb_bridge_status,
        }
        if event.type == EventType.TASK_UPDATED:
            GoalManagerInputCallbacksMixin.apply_task(self, event.payload)
            return None
        if event.type == EventType.TASK_COMPLETED:
            self.task_done_published = bool(event.payload.data)
            return State.COMPLETED if bool(event.payload.data) else State.IDLE
        handler = handlers.get(event.type)
        if handler is None:
            return None
        try:
            handler(self, event.payload)
            return None
        finally:
            self._lifecycle_event_transaction_id = None

    def _apply_hard_reset(self, event):
        """Clear target evidence and goal ownership for one warm slice."""
        request = decode_reset_request(event.payload)
        if request is None:
            return None
        reset_id = request["reset_id"]
        if reset_id == self._last_hard_reset_id:
            publish_reset_ack(
                self.pub_hard_reset_ack,
                "lste_goal_manager",
                request,
                state=State.IDLE.value,
                transaction_id=int(
                    getattr(self, "_last_hard_reset_transaction_id", 0) or 0
                ),
                duplicate=True,
                active_goal=False,
                route_owner=False,
                goal_command_id=0,
            )
            return None

        task = deepcopy(getattr(self, "latest_task", None))
        previous_goal = bool(getattr(self, "last_goal", None))
        previous_goal_command_id = int(getattr(self, "goal_command_id", 0) or 0)
        new_transaction_id = self.lifecycle_manager.hard_reset(
            event=event,
            now=rospy.Time.now().to_sec(),
            transaction_id=request.get("transaction_id"),
        )
        self._initialize_runtime_state(rospy.get_param)
        # ``goal_command_id`` is the mission transaction sequence consumed by
        # the persistent planner.  A warm reset clears semantic ownership, but
        # the planner intentionally retains its transaction high-water mark
        # to reject late commands from the previous slice.  Keep issuing
        # strictly newer mission IDs across that reset instead of rewinding
        # the distributed contract to zero.
        self.goal_command_id = previous_goal_command_id
        if task is not None:
            GoalManagerInputCallbacksMixin.apply_task(self, task)
        # Both are latched safety gates. Publish explicitly even when the
        # freshly initialized booleans already happen to be false.
        self.pub_task_done.publish(Bool(data=False))
        self.pub_navigation_hold.publish(Bool(data=False))
        self._last_hard_reset_id = reset_id
        self._last_hard_reset_transaction_id = int(new_transaction_id)
        self.canonical_lifecycle_high_water = int(new_transaction_id)
        self.canonical_route_high_water = 0
        self.canonical_graph_high_water = 0
        self.canonical_map_epoch_high_water = 0
        self.canonical_origin_epoch_high_water = 0
        self.canonical_projection_epoch_high_water = 0
        self._hard_reset_count += 1
        self.publish_goal_arbitration(
            "hard_reset_complete",
            reset_id=reset_id,
            reason=request["reason"],
            previous_goal_active=previous_goal,
            previous_goal_command_id=previous_goal_command_id,
            state=State.IDLE.value,
            hard_reset_count=int(self._hard_reset_count),
        )
        publish_reset_ack(
            self.pub_hard_reset_ack,
            "lste_goal_manager",
            request,
            state=State.IDLE.value,
            transaction_id=new_transaction_id,
            active_goal=bool(getattr(self, "last_goal", None)),
            route_owner=False,
            goal_command_id=int(getattr(self, "goal_command_id", 0) or 0),
            task_preserved=task is not None,
        )
        return None

    def _apply_map_epoch(self, message):
        """Record map timing without inventing a downstream map epoch."""
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        try:
            self.navigation_map_last_stamp = float(stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            self.navigation_map_last_stamp = 0.0

    def _on_lifecycle_transition(self, previous, current, event):
        publish = getattr(self, "publish_goal_arbitration", None)
        if callable(publish):
            publish(
                "lifecycle_transition",
                previous_state=previous.value,
                current_state=current.value,
                transaction_id=int(self.lifecycle_transaction_id()),
                transition_event=(
                    None if event is None else event.type.value
                ),
            )

    def _on_lifecycle_timeout(self, event):
        publish = getattr(self, "publish_goal_arbitration", None)
        if callable(publish):
            publish(
                "lifecycle_timeout",
                state=event.payload.get("state"),
                timeout=event.payload.get("timeout"),
                transaction_id=int(event.transaction_id),
            )

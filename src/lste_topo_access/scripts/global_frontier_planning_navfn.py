"""Navfn startup readiness and exact endpoint validation for frontiers."""

import math
import queue
import threading

import numpy as np
import rospy
from nav_msgs.srv import GetPlanRequest
from lifecycle_manager import EventType
from clock_provider import now_for


class GlobalFrontierPlanningNavfnMixin:
    """Keep slow Navfn service calls outside the graph-planning critical path."""

    # ``make_plan`` is synchronous and can block while Navfn rebuilds after a
    # costmap update. A single daemon worker preserves request ordering and
    # bounds CPU/service pressure without holding ``planning_lock``.
    _NAVFN_QUEUE_LIMIT = 8
    _NAVFN_RESULT_LIMIT = 32

    def _request_navfn_validation_wake(self, reason="navfn_validation_retry"):
        """Wake event-driven deliberation after a consumed result."""
        if getattr(self, "lifecycle_manager", None) is not None:
            return
        scheduler = getattr(self, "decision_wake_scheduler", None)
        request = getattr(scheduler, "request", None)
        if callable(request):
            request(str(reason or "navfn_validation_retry"))

    def _ensure_navfn_validation_worker(self):
        """Start the bounded worker lazily so pure fixtures stay lightweight."""
        if getattr(self, "_navfn_validation_worker", None) is not None:
            return
        self._navfn_validation_lock = threading.Lock()
        self._navfn_validation_queue = queue.Queue(
            maxsize=self._NAVFN_QUEUE_LIMIT
        )
        self._navfn_validation_pending = set()
        self._navfn_validation_results = {}
        self._navfn_validation_worker = threading.Thread(
            target=self._navfn_validation_worker_loop,
            name="lste-navfn-validation",
            daemon=True,
        )
        self._navfn_validation_worker.start()

    @staticmethod
    def _navfn_validation_number(value, digits=2):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return round(value, digits)

    def _navfn_validation_key(self, robot_map, goal_xy, frame_id):
        """Return an endpoint identity stable across tiny odometry updates."""
        return (
            str(frame_id or "map").strip().lstrip("/") or "map",
            self._navfn_validation_number(robot_map[0], 1),
            self._navfn_validation_number(robot_map[1], 1),
            # Navfn and the SLAM/costmap projections are quantized at the map
            # resolution. Keeping two decimal places made a harmless one-cell
            # reprojection look like a brand-new identity every cycle.
            self._navfn_validation_number(goal_xy[0], 1),
            self._navfn_validation_number(goal_xy[1], 1),
        )

    def _navfn_validation_request_payload(
        self, robot_map, goal_xy, frame_id, resolution,
    ):
        return {
            "robot_map": (float(robot_map[0]), float(robot_map[1])),
            "goal_xy": (float(goal_xy[0]), float(goal_xy[1])),
            "frame_id": (
                str(frame_id or "map").strip().lstrip("/") or "map"
            ),
            "resolution": float(resolution),
            "submitted_wall": now_for(self),
            "transaction_id": int(
                self.lifecycle_transaction_id()
            ),
        }

    def _navfn_validation_result_ready(self):
        """Report whether an asynchronous result can wake deliberation."""
        lock = getattr(self, "_navfn_validation_lock", None)
        results = getattr(self, "_navfn_validation_results", None)
        if lock is None or results is None:
            return False
        with lock:
            return bool(results)

    def _navfn_validation_worker_loop(self):
        """Run potentially slow service calls without blocking ROS callbacks."""
        while True:
            try:
                key, payload = self._navfn_validation_queue.get(timeout=0.25)
            except queue.Empty:
                try:
                    if rospy.is_shutdown():
                        return
                except Exception:
                    pass
                continue
            try:
                try:
                    result = self._call_navfn_validation_service(payload)
                except Exception as exc:
                    # Service/proxy doubles and ROS transport failures may use
                    # different exception classes. Keep the worker alive.
                    result = {
                        "kind": "error",
                        "state": "service_error",
                        "error": str(exc),
                        "completed_wall": now_for(self),
                    }
                completed_wall = now_for(self)
                lifecycle = getattr(self, "lifecycle_manager", None)
                if lifecycle is not None:
                    accepted = lifecycle.enqueue_type(
                        EventType.NAVFN_RESULT,
                        {
                            "key": key,
                            "payload": payload,
                            "result": result,
                            "completed_wall": completed_wall,
                        },
                        transaction_id=payload.get("transaction_id"),
                    )
                    if not accepted:
                        # A bounded lifecycle inbox may be full of causal
                        # events. Preserve the worker fact in the bounded
                        # result ledger and release the identity so the next
                        # Compute phase can consume or retry it.
                        self._store_navfn_validation_result(
                            key, payload, result, completed_wall
                        )
                        rospy.logwarn_throttle(
                            5.0,
                            "Global frontier retained Navfn result after "
                            "lifecycle queue backpressure goal=(%.2f,%.2f)",
                            payload["goal_xy"][0],
                            payload["goal_xy"][1],
                        )
                else:
                    self._store_navfn_validation_result(
                        key, payload, result, completed_wall
                    )
                rospy.loginfo(
                    "Global frontier Navfn async validation completed "
                    "goal=(%.2f,%.2f) state=%s elapsed=%.3fs",
                    payload["goal_xy"][0],
                    payload["goal_xy"][1],
                    result.get("state", "reachable" if result.get("reachable") else "empty"),
                    now_for(self) - payload["submitted_wall"],
                )
                # The result is now available, but the planning timer may be
                # event-driven and have already returned after observing the
                # pending request. Wake it from the same completion boundary;
                # otherwise a static map can leave a valid candidate parked in
                # ``_navfn_validation_results`` until an unrelated map event.
                self._request_navfn_validation_wake(
                    "navfn_validation_completed"
                )
            finally:
                self._navfn_validation_queue.task_done()

    def _call_navfn_validation_service(self, payload):
        """Issue one immutable GetPlan request and return raw result data."""
        try:
            rospy.wait_for_service(
                self.navfn_make_plan_service,
                timeout=self.navfn_make_plan_timeout,
            )
        except (rospy.ROSException, rospy.ROSInterruptException) as exc:
            return {
                "kind": "error",
                "state": "unavailable",
                "error": str(exc),
                "completed_wall": now_for(self),
            }

        request = GetPlanRequest()
        request.start.header.frame_id = payload["frame_id"]
        request.start.header.stamp = rospy.Time.now()
        request.start.pose.position.x = payload["robot_map"][0]
        request.start.pose.position.y = payload["robot_map"][1]
        request.start.pose.orientation.w = 1.0
        request.goal.header.frame_id = payload["frame_id"]
        request.goal.header.stamp = request.start.header.stamp
        request.goal.pose.position.x = payload["goal_xy"][0]
        request.goal.pose.position.y = payload["goal_xy"][1]
        request.goal.pose.orientation.w = 1.0
        # Validate the exact action endpoint. A nonzero tolerance could return
        # a nearby pose that TEB cannot use as the published mission point.
        request.tolerance = 0.0
        try:
            response = self.navfn_service(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            return {
                "kind": "error",
                "state": "service_error",
                "error": str(exc),
                "completed_wall": now_for(self),
            }
        reachable = bool(response.plan.poses)
        endpoint = None
        if reachable:
            point = response.plan.poses[-1].pose.position
            endpoint = (float(point.x), float(point.y))
        return {
            "kind": "response",
            "reachable": reachable,
            "endpoint": endpoint,
            "resolution": payload["resolution"],
                "completed_wall": now_for(self),
        }

    def _store_navfn_validation_result(
        self, key, payload, result, completed_wall=None,
    ):
        """Commit one worker fact at the lifecycle tick boundary."""
        del payload
        with self._navfn_validation_lock:
            self._navfn_validation_pending.discard(key)
            if len(self._navfn_validation_results) >= self._NAVFN_RESULT_LIMIT:
                oldest = next(iter(self._navfn_validation_results), None)
                if oldest is not None:
                    self._navfn_validation_results.pop(oldest, None)
            self._navfn_validation_results[key] = (
                now_for(self) if completed_wall is None else float(completed_wall),
                result,
            )

    def _consume_navfn_validation_result(self, key, payload):
        """Apply one worker result without performing another RPC."""
        with self._navfn_validation_lock:
            entry = self._navfn_validation_results.get(key)
            if entry is None:
                return None, False
            _completed_wall, result = entry
            self._navfn_validation_results.pop(key, None)

        if result.get("kind") == "error":
            self.navfn_last_validation_state = str(
                result.get("state", "service_error")
            )
            rospy.logwarn_throttle(
                5.0,
                "Global frontier Navfn validation %s goal=(%.2f,%.2f): %s",
                self.navfn_last_validation_state,
                payload["goal_xy"][0],
                payload["goal_xy"][1],
                result.get("error", "unknown error"),
            )
            return None, True

        now_wall = float(result.get("completed_wall", now_for(self)))
        if self.navfn_first_response_wall is None:
            self.navfn_first_response_wall = now_wall
        reachable = bool(result.get("reachable", False))
        endpoint = result.get("endpoint")
        if reachable and endpoint is not None:
            self.navfn_last_plan_endpoint = endpoint
            endpoint_error = math.hypot(
                float(endpoint[0]) - payload["goal_xy"][0],
                float(endpoint[1]) - payload["goal_xy"][1],
            )
            if endpoint_error > max(
                0.01, 1.01 * float(payload["resolution"])
            ):
                self.navfn_last_validation_state = "endpoint_offset"
                rospy.logwarn(
                    "Global frontier rejected Navfn offset endpoint goal=(%.2f,%.2f) "
                    "plan_end=(%.2f,%.2f) error=%.3fm",
                    payload["goal_xy"][0],
                    payload["goal_xy"][1],
                    endpoint[0],
                    endpoint[1],
                    endpoint_error,
                )
                return False, True
            self.navfn_last_validation_state = "reachable"
            if self.navfn_first_nonempty_response_wall is None:
                self.navfn_first_nonempty_response_wall = now_wall
                rospy.loginfo(
                    "Global frontier Navfn validation is operational: first "
                    "non-empty plan goal=(%.2f,%.2f)",
                    payload["goal_xy"][0],
                    payload["goal_xy"][1],
                )
            return True, True

        self.navfn_last_plan_endpoint = None
        if self.navfn_first_nonempty_response_wall is None:
            warmup_age = now_wall - self.navfn_first_response_wall
            if warmup_age < self.navfn_empty_warmup_seconds:
                self.navfn_last_validation_state = "bootstrap_empty"
                rospy.loginfo_throttle(
                    1.0,
                    "Global frontier waits for Navfn's first usable plan: "
                    "empty response age=%.2fs/%.2fs goal=(%.2f,%.2f)",
                    warmup_age,
                    self.navfn_empty_warmup_seconds,
                    payload["goal_xy"][0],
                    payload["goal_xy"][1],
                )
                return None, True
            self.navfn_last_validation_state = "bootstrap_timeout"
            rospy.logwarn_throttle(
                3.0,
                "Global frontier Navfn startup probe still empty after %.2fs "
                "goal=(%.2f,%.2f)",
                warmup_age,
                payload["goal_xy"][0],
                payload["goal_xy"][1],
            )
            return False, True

        warmup_age = now_wall - self.navfn_first_response_wall
        if warmup_age < self.navfn_empty_warmup_seconds:
            self.navfn_last_validation_state = "warmup_empty"
            rospy.loginfo_throttle(
                1.0,
                "Global frontier waits for Navfn costmap warmup: empty plan "
                "age=%.2fs/%.2fs goal=(%.2f,%.2f)",
                warmup_age,
                self.navfn_empty_warmup_seconds,
                payload["goal_xy"][0],
                payload["goal_xy"][1],
            )
            return None, True
        self.navfn_last_validation_state = "unreachable"
        rospy.logwarn(
            "Global frontier rejected Navfn-empty candidate goal=(%.2f,%.2f) "
            "robot=(%.2f,%.2f)",
            payload["goal_xy"][0],
            payload["goal_xy"][1],
            payload["robot_map"][0],
            payload["robot_map"][1],
        )
        return False, True

    def startup_probe_goal(self, validation, map_frame):
        """Return one nearby connected costmap cell in the SLAM map frame."""
        if validation is None:
            return None
        message, data, steps = validation
        resolution = float(message.info.resolution)
        if resolution <= 0.0:
            return None
        desired_steps = max(
            1, int(math.ceil(self.navfn_startup_probe_distance / resolution))
        )
        candidates = np.argwhere((steps >= 1) & (data < 253))
        if candidates.size == 0:
            return None
        distances = np.abs(
            steps[candidates[:, 0], candidates[:, 1]] - desired_steps
        )
        row, col = candidates[int(np.argmin(distances))]
        goal = self.cell_xy(message, int(row), int(col))
        costmap_frame = (
            (message.header.frame_id or "map").strip().lstrip("/") or "map"
        )
        map_frame = (map_frame or "map").strip().lstrip("/") or "map"
        if costmap_frame != map_frame:
            goal = self.transform_xy(map_frame, costmap_frame, goal[0], goal[1])
        return goal

    def navigation_stack_is_ready(self, robot_map, map_frame, now):
        """Gate mission selection on a usable online-SLAM navigation chain."""
        if self.navigation_stack_ready:
            return True
        validation = self.cached_costmap_steps(robot_map, now)
        if validation is None:
            state = "waiting_for_global_costmap_connectivity"
            if state != self.navigation_readiness_state:
                self.navigation_readiness_state = state
                rospy.loginfo("Global frontier startup gate: %s", state)
                self.publish_status("navigation_readiness", ready=False, state=state)
            return False
        if getattr(self, "persistent_execution", False):
            # StreamingNavfnPlanner has no geometric mission before the graph
            # selects its first route. Probing that placeholder can return an
            # empty plan and deadlock the event-driven startup. The
            # costmap-connected pose is the infrastructure gate here; the
            # first real mission is the Navfn/TEB route proof.
            self.navigation_stack_ready = True
            self.navigation_readiness_state = "ready_persistent_stream"
            rospy.loginfo(
                "Global frontier startup gate: ready for persistent_stream "
                "from map/TF/costmap; defer Navfn proof to first mission"
            )
            self.publish_status(
                "navigation_readiness",
                ready=True,
                state="ready_persistent_stream",
                validation="first_mission_route",
                navfn_probe="deferred",
            )
            return True
        probe = self.startup_probe_goal(validation, map_frame)
        if probe is None:
            state = "waiting_for_costmap_probe_cell"
            if state != self.navigation_readiness_state:
                self.navigation_readiness_state = state
                rospy.loginfo("Global frontier startup gate: %s", state)
                self.publish_status("navigation_readiness", ready=False, state=state)
            return False
        reachable = self.navfn_goal_reachable(robot_map, probe, map_frame)
        if reachable is not True:
            state = "waiting_for_navfn_probe"
            if reachable is False:
                state = "navfn_probe_empty_after_warmup"
            if state != self.navigation_readiness_state:
                self.navigation_readiness_state = state
                rospy.loginfo(
                    "Global frontier startup gate: %s probe=(%.2f,%.2f)",
                    state,
                    probe[0],
                    probe[1],
                )
                self.publish_status(
                    "navigation_readiness",
                    ready=False,
                    state=state,
                    probe=[round(float(probe[0]), 3), round(float(probe[1]), 3)],
                )
            return False
        self.navigation_stack_ready = True
        self.navigation_readiness_state = "ready"
        rospy.loginfo(
            "Global frontier startup gate passed: Navfn probe=(%.2f,%.2f)",
            probe[0],
            probe[1],
        )
        self.publish_status(
            "navigation_readiness",
            ready=True,
            state="ready",
            probe=[round(float(probe[0]), 3), round(float(probe[1]), 3)],
        )
        return True

    def navfn_goal_reachable(self, robot_map, goal_xy, frame_id):
        """Validate one endpoint without blocking the graph planner.

        ``None`` means validation is pending/unavailable and the caller should
        retry on a later planning snapshot. ``True`` and ``False`` retain the
        original exact-endpoint contract; the current costmap BFS remains an
        independent live safety gate.
        """
        if not self.navfn_plan_validation:
            self.navfn_last_validation_state = "disabled"
            return True
        self._ensure_navfn_validation_worker()
        frame = str(frame_id or "map").strip().lstrip("/") or "map"
        resolution = (
            float(self.map_msg.info.resolution)
            if getattr(self, "map_msg", None) is not None
            and self.map_msg.info.resolution > 0.0
            else 0.10
        )
        key = self._navfn_validation_key(robot_map, goal_xy, frame)
        payload = self._navfn_validation_request_payload(
            robot_map, goal_xy, frame, resolution,
        )
        now = now_for(self)
        with self._navfn_validation_lock:
            cached = self._navfn_validation_results.get(key)
            pending = key in self._navfn_validation_pending
            if cached is None and not pending:
                self._navfn_validation_pending.add(key)
                try:
                    self._navfn_validation_queue.put_nowait((key, payload))
                except queue.Full:
                    self._navfn_validation_pending.discard(key)
                    self.navfn_last_validation_state = "queue_full"
                    rospy.logwarn_throttle(
                        5.0,
                        "Global frontier Navfn validation queue is full; "
                        "deferring goal=(%.2f,%.2f)",
                        goal_xy[0],
                        goal_xy[1],
                    )
                    return None
                rospy.loginfo(
                    "Global frontier Navfn async validation submitted "
                    "goal=(%.2f,%.2f)",
                    goal_xy[0],
                    goal_xy[1],
                )
        if cached is None:
            self.navfn_last_validation_state = "pending"
            return None
        result, consumed = self._consume_navfn_validation_result(key, payload)
        if not consumed:
            self.navfn_last_validation_state = "pending"
            return None
        if result is None:
            # A consumed bootstrap-empty, unavailable, or service-error fact
            # must not strand event-driven persistent_stream before its first
            # mission. The next cycle resubmits this stable identity.
            self._request_navfn_validation_wake()
        return result

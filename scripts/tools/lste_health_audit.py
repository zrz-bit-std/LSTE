#!/usr/bin/env python3
"""Continuously audit the live LSTE ROS graph and CUDA driver state."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import xmlrpc.client
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, Iterable, List, Set, Tuple

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from lste_msgs.msg import LsteDetections, LstePrompts, LsteScores
from std_msgs.msg import Bool


DEFAULT_NODES = [
    "/gazebo",
    "/gp_subgoal",
    "/lste_cmd_vel_mux",
    "/lste_controller_switch",
    "/lste_det_node",
    "/lste_det_vis_node",
    "/lste_goal_manager",
    "/lste_prompt_node",
    "/lste_score_node",
    "/lste_state_node",
    "/lste_task_node",
    "/oc_srfc_proj",
    "/pro3/robot_state_publisher",
    "/pro3_pointcloud_to_laserscan",
    "/StageEnv_0",
    "/teleop_twist_keyboard_reset",
]

DEFAULT_PUBLISHER_TOPICS = [
    "/kinect/hd/image_color_rect",
    "/lste/cmd_vel/sappo",
    "/lste/cmd_vel/teleop",
    "/lste/det_vis_image",
    "/lste/detections",
    "/lste/final_goal",
    "/lste/prompts",
    "/lste/scores",
    "/lste/state",
    "/lste/task",
    "/pro3/rlscan",
]

DEFAULT_FRESH_TOPICS = {
    "/lste/detections": 20.0,
    "/lste/final_goal": 20.0,
    "/lste/prompts": 15.0,
    "/lste/scores": 20.0,
}

FRESH_TOPIC_TYPES = {
    "/lste/detections": LsteDetections,
    "/lste/final_goal": PoseStamped,
    "/lste/prompts": LstePrompts,
    "/lste/scores": LsteScores,
}

# Goal Manager intentionally stops publishing a new global goal after the
# target has been confirmed.  This is a successful terminal state, not a
# stale-data failure.  The latched /lste/task_done flag tells the audit when
# this exception applies.
COMPLETION_OPTIONAL_TOPICS = {"/lste/final_goal"}


class _TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout: float) -> None:
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


def _string_list_param(name: str, default: Iterable[str]) -> List[str]:
    value = rospy.get_param(name, list(default))
    if not isinstance(value, list):
        rospy.logwarn("%s must be a list; using defaults", name)
        return list(default)
    return [str(item) for item in value if str(item).strip()]


def _fresh_topic_param() -> Dict[str, float]:
    value = rospy.get_param("~fresh_topics", dict(DEFAULT_FRESH_TOPICS))
    if not isinstance(value, dict):
        rospy.logwarn("~fresh_topics must be a dictionary; using defaults")
        return dict(DEFAULT_FRESH_TOPICS)
    result: Dict[str, float] = {}
    for topic, seconds in value.items():
        try:
            result[str(topic)] = max(1.0, float(seconds))
        except (TypeError, ValueError):
            rospy.logwarn("Ignoring invalid freshness threshold %r=%r", topic, seconds)
    return result or dict(DEFAULT_FRESH_TOPICS)


def _cuda_probe() -> Tuple[bool, int, str]:
    code = (
        "import ctypes; "
        "print(ctypes.CDLL('libcuda.so.1').cuInit(0))"
    )
    try:
        completed = subprocess.run(
            [sys.executable, "-c", code],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, -1, str(exc)

    output = completed.stdout.strip()
    try:
        result = int(output.splitlines()[-1])
    except (IndexError, ValueError):
        detail = completed.stderr.strip() or output or "cuInit produced no result"
        return False, -1, detail
    return result == 0, result, completed.stderr.strip()


class HealthAudit:
    def __init__(self) -> None:
        rospy.init_node("lste_health_audit")
        self.started_at = time.monotonic()
        self.startup_grace = max(0.0, float(rospy.get_param("~startup_grace", 120.0)))
        self.check_interval = max(0.5, float(rospy.get_param("~check_interval", 2.0)))
        self.cuda_interval = max(5.0, float(rospy.get_param("~cuda_interval", 30.0)))
        self.node_ping_interval = max(5.0, float(rospy.get_param("~node_ping_interval", 15.0)))
        self.node_ping_timeout = max(0.1, float(rospy.get_param("~node_ping_timeout", 0.75)))
        self.healthy_log_interval = max(5.0, float(rospy.get_param("~healthy_log_interval", 30.0)))
        default_status_file = Path(os.environ.get("LSTE_WS", Path.cwd())) / "runtime/health/status.json"
        self.status_file = Path(rospy.get_param("~status_file", str(default_status_file)))
        self.expected_nodes = _string_list_param("~expected_nodes", DEFAULT_NODES)
        self.publisher_topics = _string_list_param(
            "~publisher_topics", DEFAULT_PUBLISHER_TOPICS
        )
        self._apply_pipeline_profile()
        self.fresh_topics = _fresh_topic_param()

        self.last_messages: Dict[str, float] = {}
        self.last_cuda_check = 0.0
        self.last_node_ping = 0.0
        self.cuda_ok = False
        self.cuda_result = -1
        self.cuda_detail = "not checked"
        self.unreachable_nodes: List[str] = []
        self.ready_once = False
        self.task_done = False
        self.last_signature = ""
        self.last_healthy_log = 0.0
        self.last_payload: Dict[str, object] = {}

        self.master = rosgraph.Master(rospy.get_name())
        self.subscribers = [
            rospy.Subscriber(
                topic,
                FRESH_TOPIC_TYPES.get(topic, rospy.AnyMsg),
                self._message_callback,
                callback_args=topic,
                queue_size=1,
            )
            for topic in self.fresh_topics
        ]
        self.task_done_subscriber = rospy.Subscriber(
            "/lste/task_done", Bool, self._task_done_callback, queue_size=1
        )
        rospy.on_shutdown(self._write_stopped)
        print(
            "[health] sidecar started: grace=%.1fs interval=%.1fs status=%s"
            % (self.startup_grace, self.check_interval, self.status_file),
            flush=True,
        )

    def _apply_pipeline_profile(self) -> None:
        """Remove inactive controller/legacy expectations from the audit.

        The health sidecar is shared by TEB, teleop, and SA-PPO launches. A
        static list that always requires the SA-PPO and legacy GP nodes makes a
        healthy TEB benchmark report ``starting`` forever, even when every
        node that the selected architecture needs is alive.
        """
        mode = str(rospy.get_param("~controller_mode", "")).strip().lower()
        legacy_gp = str(rospy.get_param("~legacy_gp_frontier_enabled", "true")).strip().lower() in (
            "1", "true", "yes", "on"
        )
        detector_node = str(rospy.get_param("~detector_node", "")).strip()
        if mode in ("teb", "teleop"):
            self.expected_nodes = [node for node in self.expected_nodes if node != "/StageEnv_0"]
        if not legacy_gp:
            self.expected_nodes = [node for node in self.expected_nodes if node != "/gp_subgoal"]
        if detector_node:
            self.expected_nodes = [node for node in self.expected_nodes if node != "/lste_det_node"]
            if detector_node not in self.expected_nodes:
                self.expected_nodes.append(detector_node)
        if mode in ("teb", "teleop", "sappo"):
            active_topic = "/lste/cmd_vel/%s" % mode
            self.publisher_topics = [
                topic
                for topic in self.publisher_topics
                if topic not in ("/lste/cmd_vel/teb", "/lste/cmd_vel/sappo", "/lste/cmd_vel/teleop")
            ]
            self.publisher_topics.append(active_topic)

    def _message_callback(self, _message: rospy.AnyMsg, topic: str) -> None:
        self.last_messages[topic] = time.monotonic()

    def _task_done_callback(self, message: Bool) -> None:
        self.task_done = bool(message.data)

    def _system_state(self) -> Tuple[Set[str], Set[str], str]:
        try:
            publishers, subscribers, services = self.master.getSystemState()
        except Exception as exc:
            return set(), set(), str(exc)

        nodes: Set[str] = set()
        publisher_topics: Set[str] = set()
        for topic, names in publishers:
            if names:
                publisher_topics.add(topic)
                nodes.update(names)
        for _, names in subscribers:
            nodes.update(names)
        for _, names in services:
            nodes.update(names)
        return nodes, publisher_topics, ""

    def _update_cuda(self, now: float) -> None:
        if now - self.last_cuda_check < self.cuda_interval:
            return
        self.cuda_ok, self.cuda_result, self.cuda_detail = _cuda_probe()
        self.last_cuda_check = now

    def _ping_node(self, node_name: str) -> bool:
        try:
            uri = rosgraph.Master(rospy.get_name()).lookupNode(node_name)
            proxy = xmlrpc.client.ServerProxy(
                uri,
                transport=_TimeoutTransport(self.node_ping_timeout),
                allow_none=True,
            )
            code, _, _ = proxy.getPid(rospy.get_name())
            return int(code) == 1
        except Exception:
            return False

    def _update_node_liveness(self, now: float, registered_nodes: Set[str]) -> None:
        if now - self.last_node_ping < self.node_ping_interval:
            return
        candidates = [node for node in self.expected_nodes if node in registered_nodes]
        unreachable: List[str] = []
        with ThreadPoolExecutor(max_workers=8) as executor:
            futures = {executor.submit(self._ping_node, node): node for node in candidates}
            for future in as_completed(futures):
                node = futures[future]
                try:
                    if not future.result():
                        unreachable.append(node)
                except Exception:
                    unreachable.append(node)
        self.unreachable_nodes = sorted(unreachable)
        self.last_node_ping = now

    def _write_status(self, payload: Dict[str, object]) -> None:
        self.status_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.status_file.with_name(
            "%s.tmp.%d" % (self.status_file.name, os.getpid())
        )
        temporary.write_text(
            json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary), str(self.status_file))

    def _write_stopped(self) -> None:
        payload = dict(self.last_payload)
        payload.update(
            {
                "status": "stopped",
                "ok": False,
                "wall_time": time.time(),
                "sidecar_pid": os.getpid(),
            }
        )
        try:
            self._write_status(payload)
        except OSError:
            pass

    def _audit(self) -> None:
        now = time.monotonic()
        self._update_cuda(now)
        nodes, publisher_topics, master_error = self._system_state()
        self._update_node_liveness(now, nodes)

        missing_nodes = sorted(set(self.expected_nodes) - nodes)
        missing_publishers = sorted(set(self.publisher_topics) - publisher_topics)
        stale_topics = []
        message_ages: Dict[str, object] = {}
        for topic, threshold in self.fresh_topics.items():
            last_seen = self.last_messages.get(topic)
            if last_seen is None:
                message_ages[topic] = None
                if not (self.task_done and topic in COMPLETION_OPTIONAL_TOPICS):
                    stale_topics.append(topic)
                continue
            age = max(0.0, now - last_seen)
            message_ages[topic] = round(age, 3)
            if age > threshold and not (
                self.task_done and topic in COMPLETION_OPTIONAL_TOPICS
            ):
                stale_topics.append(topic)

        failures = []
        if master_error:
            failures.append("ros_master")
        if not self.cuda_ok:
            failures.append("cuda")
        if missing_nodes:
            failures.append("nodes")
        if self.unreachable_nodes:
            failures.append("unreachable_nodes")
        if missing_publishers:
            failures.append("publishers")
        if stale_topics:
            failures.append("stale_topics")

        within_grace = now - self.started_at < self.startup_grace
        # CUDA driver initialization and ROS master reachability cannot become
        # healthy merely by waiting for normal node startup. Do not hide these
        # terminal failures behind the ordinary startup grace window.
        critical_failures = {"cuda", "ros_master"}
        if not failures:
            self.ready_once = True
            status = "healthy"
        elif critical_failures.intersection(failures):
            status = "unhealthy"
        elif within_grace and not self.ready_once:
            status = "starting"
        else:
            status = "unhealthy"

        payload = {
            "status": status,
            "ok": status == "healthy",
            "wall_time": time.time(),
            "sidecar_pid": os.getpid(),
            "uptime_seconds": round(now - self.started_at, 3),
            "startup_grace_seconds": self.startup_grace,
            "task_done": self.task_done,
            "failures": failures,
            "cuda": {
                "ok": self.cuda_ok,
                "cu_init": self.cuda_result,
                "detail": self.cuda_detail,
            },
            "missing_nodes": missing_nodes,
            "unreachable_nodes": self.unreachable_nodes,
            "missing_publishers": missing_publishers,
            "stale_topics": stale_topics,
            "message_age_seconds": message_ages,
        }
        if not self.cuda_ok:
            payload["cuda_recovery"] = [
                "stopall",
                "sudo modprobe -r nvidia_uvm",
                "sudo modprobe nvidia_uvm",
                "verify cuInit == 0, then runall",
            ]

        encoded = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        self.last_payload = payload
        try:
            self._write_status(payload)
        except OSError as exc:
            print("[health] ERROR cannot write status file: %s" % exc, flush=True)

        signature = json.dumps(
            {
                "status": status,
                "failures": failures,
                "missing_nodes": missing_nodes,
                "unreachable_nodes": self.unreachable_nodes,
                "missing_publishers": missing_publishers,
                "stale_topics": stale_topics,
                "cu_init": self.cuda_result,
            },
            sort_keys=True,
        )
        changed = signature != self.last_signature
        if status == "unhealthy" and changed:
            print("[health] UNHEALTHY %s" % encoded, flush=True)
        elif status == "starting" and changed:
            print("[health] STARTING %s" % encoded, flush=True)
        elif status == "healthy" and (
            changed or now - self.last_healthy_log >= self.healthy_log_interval
        ):
            print("[health] HEALTHY %s" % encoded, flush=True)
            self.last_healthy_log = now
        self.last_signature = signature

    def run(self) -> None:
        while not rospy.is_shutdown():
            cycle_started = time.monotonic()
            self._audit()
            elapsed = time.monotonic() - cycle_started
            time.sleep(max(0.0, self.check_interval - elapsed))


if __name__ == "__main__":
    HealthAudit().run()

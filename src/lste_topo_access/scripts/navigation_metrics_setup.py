"""Node setup for ``NavigationMetrics``."""

import sys
import threading
import time

import rospy
import tf


def _private_remap_value(argv, parameter_name):
    """Return a private ROS remap value without YAML type coercion.

    ``rospy.init_node`` parses ``_name:=value`` remappings with
    ``yaml.safe_load`` before this node can call ``get_param``.  Timestamp
    names such as ``20260908_072000`` therefore become a large Python integer
    and fail when uploaded through XML-RPC.  Keep this tiny parser string-only
    so the value can be restored after node initialization.
    """
    prefix = "_%s:=" % str(parameter_name)
    value = None
    for argument in argv:
        if isinstance(argument, str) and argument.startswith(prefix):
            # Match rospy's remapping dictionary: a later duplicate wins.
            value = argument[len(prefix):]
    return value


def _remove_private_remap(argv, parameter_name):
    """Remove one private remap from the argv passed to ``rospy.init_node``."""
    prefix = "_%s:=" % str(parameter_name)
    return [
        argument
        for argument in argv
        if not (
            isinstance(argument, str) and argument.startswith(prefix)
        )
    ]


def _init_metrics_node(argv=None):
    """Initialize metrics while preserving ``run_timestamp`` as text."""
    raw_argv = list(sys.argv if argv is None else argv)
    requested_timestamp = _private_remap_value(raw_argv, "run_timestamp")
    init_argv = _remove_private_remap(raw_argv, "run_timestamp")
    rospy.init_node("lste_navigation_metrics", argv=init_argv)
    if requested_timestamp is not None:
        rospy.set_param("~run_timestamp", str(requested_timestamp))


class NavigationMetricsSetupMixin:
    """Create shared resources, then delegate focused setup work."""

    def _initialize_metrics(self):
        # ``run_timestamp`` is an identifier, never a numeric ROS parameter.
        # Remove its command-line remap before rospy's YAML/XML-RPC upload,
        # then restore it as an explicit string after initialization.  The
        # normal launcher passes only ``run_directory``; this also makes a
        # manually started metrics node safe and keeps old commands working.
        _init_metrics_node()
        self.lock = threading.RLock()
        self.process_name = "lste_navigation_metrics"
        self.log_root = self._resolve_path(
            rospy.get_param("~log_dir", "runtime/navigation/logs")
        )
        self.retention_days = max(
            1, int(rospy.get_param("~retention_days", 15))
        )
        # Benchmark launchers may allocate the timestamp directory before the
        # node starts so every process in one invocation shares one parent.
        # Normal production runs keep the historical auto-created directory.
        requested_run_dir = str(rospy.get_param("~run_directory", "")).strip()
        requested_timestamp = str(rospy.get_param("~run_timestamp", "")).strip()
        if requested_run_dir:
            self.log_dir = self._resolve_path(requested_run_dir)
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.run_timestamp = requested_timestamp or self.log_dir.name
        else:
            self.run_timestamp, self.log_dir = self._create_run_dir()
        self.log_path = self.log_dir / (self.run_timestamp + "_navigation_metrics.log")
        # A targeted application-node restart may reuse the same warm run
        # directory. Preserve the existing evidence in that case; a newly
        # allocated directory is still initialized as a fresh log.
        log_mode = "a" if self.log_path.exists() else "w"
        self.stream = self.log_path.open(log_mode, encoding="utf-8", buffering=1)
        # This parameter is written by the benchmark stop boundary.  Remove a
        # value left by an older process so a normal/manual shutdown cannot be
        # mistaken for the next trial's timeout.
        try:
            rospy.delete_param("/lste_navigation_metrics/termination_reason")
        except (KeyError, rospy.ROSException):
            pass

        self.start_wall = time.monotonic()
        self.start_ros = rospy.Time.now().to_sec()
        self.task_id = str(rospy.get_param("~task_id", "")).strip()
        self.execution_architecture = str(
            rospy.get_param("~execution_architecture", "unknown")
        ).strip().lower() or "unknown"
        self.persistent_execution = self.execution_architecture == "persistent_stream"
        self.pose = None
        self.goal = None
        self.goal_frame = "odom"
        self.last_goal_frame = "odom"
        self.goal_message = None
        self.tf_listener = tf.TransformListener()
        self.distance_transform_failures = 0
        self._initialize_benchmark_truth()
        self._initialize_benchmark_collision_truth()
        self._initialize_target_evaluation_state()
        self._initialize_navigation_observer_state()
        # Capture immutable run metadata once.  Failure artifacts can then be
        # inspected in isolation without reopening the multi-megabyte metrics
        # log to recover the world, pose, task, and controller contract.
        self.run_context = self._run_start_context()
        # Resolve the controller/planner contract once and retain it in the
        # standalone run context. ``_write_run_start`` reuses this snapshot so
        # diagnostics do not wait for the parameter server a second time.
        self.run_context["resolved_params"] = self._resolved_startup_params()
        # Keep the high-volume failure snapshots separate from the compact
        # benchmark stream. Both files belong to this metrics process and the
        # same timestamped run directory, so a failure ID can be followed
        # without searching ~/.ros/log.
        self.failure_log_path = self.log_dir / (
            self.run_timestamp + "_failure_evidence.log"
        )
        self._open_failure_evidence_log()
        self._write_run_start()
        self._setup_ros_interfaces()

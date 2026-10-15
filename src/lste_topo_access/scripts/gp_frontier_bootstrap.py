"""Bootstrap responsibilities for the legacy GP frontier node.

The historical GP node is still available for comparison experiments, but its
node constructor used to combine mutable state, ROS wiring, parameter loading,
and optional log redirection.  Keeping those phases explicit makes changes to
one concern less likely to alter another startup contract.
"""

import os
import sys
from time import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Vector3Stamped
from lste_msgs.msg import LsteFrontiers, LsteState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String, UInt8
from visualization_msgs.msg import Marker

from gp_frontier_common import resolve_topo_path


class GpFrontierBootstrapMixin:
    """Own initialization phases without changing legacy topic names."""

    def _initialize_sensor_cache(self):
        """Create attributes needed before a subscriber callback can run."""
        self.pose = None
        self.pcl_arr = None
        self.header = Header()
        self.header.seq = 0
        self.header.stamp = None
        self.header.frame_id = "odom"

    def _setup_core_ros_interfaces(self):
        """Connect raw GP inputs and the original visual/debug publishers."""
        self.rbt_pose_sub = rospy.Subscriber(
            "rbt_pose", Pose2D, self.pose_cb, queue_size=1
        )
        self.sph_pcl_sub = rospy.Subscriber(
            "sph_pcl", PointCloud2, self.sph_pcl_cb, queue_size=1
        )
        self.gp_var_pub = rospy.Publisher("gp_nav_var", PointCloud2, queue_size=1)
        self.gp_oc_pub = rospy.Publisher("gp_nav_oc", PointCloud2, queue_size=1)
        self.gp_nav_pts_pub = rospy.Publisher(
            "gp_nav_pts", PointCloud2, queue_size=1
        )
        self.gp_actul_xy_subgls_pub = rospy.Publisher(
            "gp_nav_actul_xy_gls", PointCloud2, queue_size=1
        )
        self.gp_rcmndd_subgl_pub = rospy.Publisher(
            "gp_subgoal", PoseStamped, queue_size=1
        )
        self.gp_subgl_pub = rospy.Publisher(
            "gp_subgoal_os", PoseStamped, queue_size=1
        )
        self.gp_subgl_rviz = rospy.Publisher(
            "gp_subgl_rviz", PointCloud2, queue_size=1
        )
        self.lste_gp_frontier_pub = rospy.Publisher(
            "/lste/gp_frontier", PoseStamped, queue_size=1
        )

    def _initialize_access_topology_state(self):
        """Initialize persistent Access-Topo state before its callbacks bind."""
        self.visited_positions = []
        self.int_visited_positions = []
        self.closed = []
        self.change_flag = True
        self.final_goal_received = False
        self.lste_state = 0
        self.lste_subtype = ""
        self.run_id = int(time())
        self.run_start = rospy.Time.now().to_sec()
        self.run_end = None
        self.state_events = []
        self.profile_events = []
        self.goal_events = []
        self.visited_events = []
        self.active_mode = "unknown"
        self.vanish_distance = None
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.headings = []
        self.dir_idx = 0
        self.last_switch_xy = None
        self.commit_active = False
        self.commit_heading = 0.0
        self.commit_start_xy = (0.0, 0.0)
        self.detected_interest_points = []
        self.match_threshold = 0.5

        self.load_access_topo_config()
        self.max_pending_per_node = getattr(self, "max_pending_per_node", 2)
        self.junction_cooldown_anchors = getattr(
            self, "junction_cooldown_anchors", 4
        )
        self.last_junction_anchor = None
        self.anchor_nodes = []
        self.anchor_last_xy = None
        self.anchor_last_id = None
        self.backtrack_stack = []
        self.current_backtrack = None
        self.backtrack_start_pose = None
        self.backtrack_start_anchor = None
        self.backtrack_path = []
        self.commit_goal = None
        self.backtrack_history = []
        self.no_frontier_since = None
        self.no_frontier_start_pose = None
        self.no_frontier_start_anchor = None
        self.backtrack_recording = False
        self.return_home_target = None
        self.access_mode = 0
        self.forced_heading_world = None
        self.forced_start_xy = None
        self.forced_branch = None
        self.mode2_start_time = None
        self.cluster_track = []
        self.pending_junctions = []
        self.junction_post_finalize_cooldown = getattr(
            self, "junction_post_finalize_cooldown", 4
        )
        self.last_finalized_anchor = None
        self.multi_peak_since = None
        self.force_window_rad = np.deg2rad(self.force_window_deg)
        self.cluster_eps_rad = np.deg2rad(self.cluster_eps_deg)
        self.junction_peak_min_sep_rad = np.deg2rad(
            getattr(self, "junction_peak_min_sep_deg", 30.0)
        )
        self.junction_exit_confirm_sec = getattr(
            self, "junction_exit_confirm_sec", 0.5
        )
        self.junction_decision_dist = getattr(
            self, "junction_decision_dist", self.anchor_step_dist
        )
        self.junction_same_dir_gate_rad = np.deg2rad(
            getattr(self, "junction_same_dir_gate_deg", 30.0)
        )
        self.junction_finalize_dist = getattr(
            self,
            "junction_finalize_dist",
            self.junction_decision_dist * 1.5,
        )
        self.junction_finalize_timeout = getattr(
            self, "junction_finalize_timeout", 6.0
        )
        self.junction_max_pending = 2
        if not getattr(self, "topo_save_file", None):
            self.topo_save_file = os.path.join(
                self.topo_save_dir, "access_topo_%s.json" % self.run_id
            )
        self.last_topo_save_time = 0.0

    def _setup_access_topology_ros_interfaces(self):
        """Connect Access-Topo, LSTE coordination, and frontier interfaces."""
        self.zcyrbt_pose_sub = rospy.Subscriber(
            "rbt_pose", Pose2D, self.zcypose_cb, queue_size=1
        )
        self.vanish_sub = rospy.Subscriber(
            "/vanish_point", Pose, self.vanish_cb, queue_size=1
        )
        self.gp_interest_subgl_pub = rospy.Publisher(
            "interest_subgoal", PoseStamped, queue_size=1
        )
        self.closed_marker_pub = rospy.Publisher(
            "closed_markers", Marker, queue_size=1
        )
        self.change_flag_pub = rospy.Publisher("change_flag", Bool, queue_size=1)
        self.change_flag_sub = rospy.Subscriber(
            "change_flag", Bool, self.flag_cb, queue_size=1
        )
        self.backtrack_goal_pub = rospy.Publisher(
            "/lste/access_topo/backtrack_goal", PoseStamped, queue_size=1
        )
        self.access_mode_pub = rospy.Publisher(
            "/lste/access_topo/mode", UInt8, queue_size=1
        )
        self.final_goal_sub = rospy.Subscriber(
            "/lste/final_goal", PoseStamped, self.final_goal_cb, queue_size=1
        )
        self.lste_state_sub = rospy.Subscriber(
            "/lste/state", LsteState, self.lste_state_cb, queue_size=1
        )
        self.active_mode_sub = rospy.Subscriber(
            "/lste/access_topo/active_mode", String, self.active_mode_cb, queue_size=1
        )
        self.frontier_dir_pub = rospy.Publisher(
            "/lste/gp_frontier_dir", Vector3Stamped, queue_size=1
        )
        self.frontiers_pub = rospy.Publisher(
            "/lste/gp_frontiers", LsteFrontiers, queue_size=1
        )

    def _load_gp_runtime_configuration(self):
        """Resolve GP parameters and allocate the fixed spherical grid."""
        self.oc_srfc_rds = rospy.get_param("~oc_srfc_rds", 5.0)
        self.pcl_skp = rospy.get_param("~pcl_skp", 3)
        self.pose = None
        self.pcl_arr = None
        self.org_unq_thetas = None
        self.pcl_unq_thetas = None
        self.pcl_thetas = None
        self.pcl_alphas = None
        self.pcl_rds = None
        self.pcl_oc = None
        self.pcl_sz = None
        self.gp_grd = None
        self.gp_grd_w = None
        self.gp_grd_h = None
        self.gp_grd_ths = None
        self.gp_grd_als = None
        self.gp_grd_oc = None
        self.gp_grd_rds = None
        self.gp_grd_var = None
        self.sample_gp_nav_grid()
        self.gp_nav_pt = None
        self.gp_nav_pts = None
        self.gap_utlty_fun = None
        self.gp_nav_frntr_cntrs = None
        self.gp_nav_frntr_areas = None
        self.gp_nav_indpts_sz = rospy.get_param("~gp_nav_indpts_sz", 400)
        self.gp_nav_var_thrshld = rospy.get_param("~gp_nav_var_thrshld", 0.03)
        self.gap_k_dir = rospy.get_param("~gap_k_dir", 4.0)
        self.gap_k_dst = rospy.get_param("~gap_k_dst", 5.0)
        self.gp_nav_goal_dst = rospy.get_param("~gp_nav_goal_dst", 5.0)
        self.gp_nav_var_img_viz = rospy.get_param("~gp_nav_var_img_viz", False)
        self.gp_nav_var_viz = rospy.get_param("~gp_nav_var_viz", 5.0)
        self.gl_update_period = rospy.get_param("~gl_update_period", 5.0)
        self.gl_update_forward_dist = rospy.get_param("~gl_update_forward_dist", 10.0)
        self.last_gl_update_time = rospy.Time.now().to_sec() - self.gl_update_period
        self.gl_x = rospy.get_param("~gl_x", -4.0)
        self.gl_y = rospy.get_param("~gl_y", -16.0)
        self.gl_yaw = rospy.get_param("~gl_yaw", 0.0)
        self.gl_wrt_odom = np.array([self.gl_x, self.gl_y, 1], dtype="float32")
        self.gp_nav_frame_id = "os_sensor"
        self.gp_nav_var_pblsh = True
        self.gp_nav_xypts = None
        self.goal_published = False
        self.map_2d_h = 500
        self.map_2d_w = 500
        self.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        self._start_goal_seeded = False

    def _configure_frontier_log(self):
        """Optionally mirror legacy GP console output to one frontier log."""
        self.frontier_log_enabled = rospy.get_param("~frontier_log", False)
        self.frontier_log_dir = rospy.get_param(
            "~frontier_log_dir", resolve_topo_path("topo_tree/frontier_log")
        )
        self.frontier_log_file = None
        self.frontier_log_fh = None
        self._stdout_orig = None
        self._stderr_orig = None
        if not self.frontier_log_enabled:
            return
        try:
            self.frontier_log_dir = os.path.expanduser(self.frontier_log_dir)
            os.makedirs(self.frontier_log_dir, exist_ok=True)
            self.frontier_log_file = os.path.join(
                self.frontier_log_dir, "frontier_%s.log" % self.run_id
            )
            rospy.loginfo(
                "gp_subgoal: frontier_log enabled, writing to %s",
                self.frontier_log_file,
            )
            self.frontier_log_fh = open(
                self.frontier_log_file, "a", encoding="utf-8", buffering=1
            )
            self.frontier_log_fh.write("# frontier_log run_id=%s\n" % self.run_id)
            self._stdout_orig = sys.stdout
            self._stderr_orig = sys.stderr
            sys.stdout = _TeeStream(sys.stdout, self.frontier_log_fh)
            sys.stderr = _TeeStream(sys.stderr, self.frontier_log_fh)
        except Exception as exc:
            rospy.logwarn("gp_subgoal: frontier_log disabled (init failed: %s)", exc)
            self.frontier_log_enabled = False
            if self.frontier_log_fh is not None:
                self.frontier_log_fh.close()
            self.frontier_log_fh = None
            self.frontier_log_file = None


class _TeeStream:
    """Mirror stdout/stderr to a timestamped GP log while retaining the console."""

    def __init__(self, main_stream, log_stream):
        self._main = main_stream
        self._log = log_stream

    def write(self, data):
        try:
            self._main.write(data)
        except Exception:
            pass
        if not self._log:
            return
        try:
            for chunk in data.splitlines(True):
                stamp = rospy.Time.now().to_sec()
                if stamp <= 0.0:
                    stamp = time()
                if chunk.endswith("\n"):
                    content = chunk[:-1]
                    newline = "\n"
                else:
                    content = chunk
                    newline = ""
                if content.strip():
                    self._log.write("[%.3f] %s%s" % (stamp, content, newline))
                elif newline:
                    self._log.write("[%.3f]%s" % (stamp, newline))
            self._log.flush()
        except Exception:
            pass

    def flush(self):
        try:
            self._main.flush()
        except Exception:
            pass
        try:
            self._log.flush()
        except Exception:
            pass

#!/usr/bin/env python3
"""ROS entry point for the live LSTE detection visualization.

The node intentionally contains only ROS lifecycle and shared display state.
Tracking, overlay rendering, score panels, and goal projection are kept in
their own modules so they can evolve independently.
"""

import copy
import threading
from collections import deque

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D, PoseStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
import tf2_ros

from det_vis_goal_projection import DetectionGoalProjectionMixin
from det_vis_overlay import DetectionOverlayMixin
from det_vis_score_panel import DetectionScorePanelMixin
from det_vis_tracking import DetectionTrackingMixin
from lste_msgs.msg import LsteDetections, LsteScores, LsteState, LsteTask


class DetectionVisualizer(
    DetectionTrackingMixin,
    DetectionOverlayMixin,
    DetectionScorePanelMixin,
    DetectionGoalProjectionMixin,
):
    """Subscribe to perception state and publish the annotated camera frame."""

    def __init__(self):
        rospy.init_node("lste_det_vis_node")
        self._initialize_message_state()
        self._load_visualization_config()
        self._initialize_tracking_state()
        self._initialize_goal_projection()
        self._create_ros_interfaces()
        self._log_startup()

    def _initialize_message_state(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None
        self.display_detections = None
        self.latest_scores = None
        self.latest_task = None
        self.latest_state = None
        self.task_done = False
        self.robot_pose = None

    def _load_visualization_config(self):
        self.image_topic = rospy.get_param("~image_topic", "/kinect/hd/image_color_rect")
        self.detections_topic = rospy.get_param("~detections_topic", "/lste/detections")
        self.output_topic = rospy.get_param("~output_topic", "/lste/det_vis_image")
        self.scores_topic = rospy.get_param("~scores_topic", "/lste/scores")
        self.state_topic = rospy.get_param("~state_topic", "/lste/state")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.draw_labels = bool(rospy.get_param("~draw_labels", True))
        self.font_scale = float(rospy.get_param("~font_scale", 0.65))
        self.line_thickness = int(rospy.get_param("~line_thickness", 2))
        self.camera_fov_deg = float(rospy.get_param("~camera_fov_deg", 60.0))
        self.display_sync_mode = self._display_sync_mode()
        self.display_history_size = max(1, int(rospy.get_param("~display_history_size", 90)))
        self._load_tracking_config()
        self.camera_info_topic = self._resolved_camera_info_topic(self.camera_info_topic)

    def _display_sync_mode(self):
        requested = str(rospy.get_param("~display_sync_mode", "latest_frame")).lower()
        if requested in ("latest_frame", "detection_frame"):
            return requested
        rospy.logwarn("Unknown display_sync_mode=%s; using latest_frame", requested)
        return "latest_frame"

    def _load_tracking_config(self):
        self.tracking_enabled = bool(rospy.get_param("~tracking_enabled", True))
        self.detector_backend = str(rospy.get_param("~detector_backend", "groundingdino")).lower()
        requested_mode = str(rospy.get_param("~tracking_mode", "auto")).lower()
        if requested_mode == "auto":
            self.tracking_mode = "associated" if self.detector_backend == "wedetect" else "legacy"
        elif requested_mode in ("legacy", "associated"):
            self.tracking_mode = requested_mode
        else:
            rospy.logwarn("Unknown tracking_mode=%s; using legacy", requested_mode)
            self.tracking_mode = "legacy"
        self.associated_iou_threshold = float(rospy.get_param("~associated_iou_threshold", 0.25))
        self.associated_measurement_alpha = float(
            rospy.get_param("~associated_measurement_alpha", 0.35)
        )
        self.associated_track_ttl = float(rospy.get_param("~associated_track_ttl", 0.8))
        self.tracking_scale = float(rospy.get_param("~tracking_scale", 0.25))
        self.tracking_history_size = int(rospy.get_param("~tracking_history_size", 90))
        self.max_tracking_lag = float(rospy.get_param("~max_tracking_lag", 3.0))
        self.max_tracking_age = float(rospy.get_param("~max_tracking_age", 4.0))
        self.tracking_update_stride = max(1, int(rospy.get_param("~tracking_update_stride", 3)))
        self.tracking_edge_touch_pixels = float(rospy.get_param("~tracking_edge_touch_pixels", 2.0))
        self.tracking_edge_exit_frames = max(1, int(rospy.get_param("~tracking_edge_exit_frames", 12)))
        self.tracking_edge_exit_speed_scale = float(
            rospy.get_param("~tracking_edge_exit_speed_scale", 0.65)
        )
        self.tracking_edge_min_visible_fraction = float(
            rospy.get_param("~tracking_edge_min_visible_fraction", 0.05)
        )
        self.tracking_flow_enabled = bool(rospy.get_param("~tracking_flow_enabled", True))
        self.tracking_flow_min_points = max(4, int(rospy.get_param("~tracking_flow_min_points", 12)))
        self.tracking_flow_max_lost_frames = max(
            1, int(rospy.get_param("~tracking_flow_max_lost_frames", 30))
        )

    def _resolved_camera_info_topic(self, configured_topic):
        if configured_topic:
            return configured_topic
        if "/" in self.image_topic:
            return f"{self.image_topic.rsplit('/', 1)[0]}/camera_info"
        return "/camera/camera_info"

    def _initialize_tracking_state(self):
        self._frame_history = deque(maxlen=max(1, self.tracking_history_size))
        self._display_frame_history = deque(maxlen=self.display_history_size)
        self._display_image = None
        self._display_image_header = None
        self._trackers = []
        self._tracking_started_at = None
        self._tracking_frame_count = 0
        self._previous_tracking_image = None
        self._last_associated_detection_stamp = None
        self._tracking_lock = threading.Lock()

    def _initialize_goal_projection(self):
        self.palette = {
            "target": {"edge": (48, 72, 255), "fill": (48, 72, 255)},
            "env": {"edge": (0, 204, 255)},
            "ctx": {"edge": (72, 201, 176)},
            "neg": {"edge": (140, 140, 140)},
        }
        self.global_goal_point = None
        self.global_goal_frame = "odom"
        self.camera_model = PinholeCameraModel()
        self.has_camera_info = False
        self.camera_frame_id = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _create_ros_interfaces(self):
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)
        self.sub_detections = rospy.Subscriber(
            self.detections_topic, LsteDetections, self.on_detections, queue_size=1
        )
        self.sub_scores = rospy.Subscriber(self.scores_topic, LsteScores, self.on_scores, queue_size=1)
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.sub_state = rospy.Subscriber(self.state_topic, LsteState, self.on_state, queue_size=1)
        self.sub_task_done = rospy.Subscriber("/lste/task_done", Bool, self.on_task_done, queue_size=1)
        self.sub_robot_pose = rospy.Subscriber("/rbt_pose", Pose2D, self.on_robot_pose, queue_size=1)
        self.sub_camera_info = rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.on_camera_info,
            queue_size=1,
        )
        self.sub_global_goal_pose = rospy.Subscriber(
            "/lste/final_goal",
            PoseStamped,
            self.on_global_goal_pose,
            queue_size=1,
        )

    def _log_startup(self):
        rospy.loginfo(
            "lste_det_vis_node started. image_topic=%s detections_topic=%s scores_topic=%s "
            "output_topic=%s tracking_mode=%s display_sync_mode=%s",
            self.image_topic,
            self.detections_topic,
            self.scores_topic,
            self.output_topic,
            self.tracking_mode,
            self.display_sync_mode,
        )

    def on_image(self, message):
        self.latest_image = message
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("cv_bridge failed to convert Image: %s", exc)
            return
        if image is None or image.size == 0:
            return
        with self._tracking_lock:
            tracking_image = self._tracking_image(image)
            self._frame_history.append((self._stamp_ns(message.header), tracking_image))
            if self.display_sync_mode == "detection_frame":
                self._display_frame_history.append((self._stamp_ns(message.header), message.header, image))
            self._tracking_frame_count += 1
            if self._tracking_frame_count % self.tracking_update_stride == 0:
                self._update_trackers(tracking_image, image.shape[1], image.shape[0])
        self.try_publish(image)

    def on_detections(self, message):
        self.latest_detections = message
        with self._tracking_lock:
            if self.tracking_mode == "associated":
                self._associate_trackers(message)
            else:
                self._start_trackers(message)
            if self.display_sync_mode == "detection_frame":
                self._select_detection_frame(message)
        if self.display_sync_mode == "detection_frame":
            self.try_publish()

    def on_scores(self, message):
        self.latest_scores = message

    def on_task(self, message):
        self.latest_task = message
        self.task_done = False

    def on_state(self, message):
        self.latest_state = message

    def on_task_done(self, message):
        self.task_done = bool(getattr(message, "data", False))

    def on_camera_info(self, message):
        try:
            self.camera_model.fromCameraInfo(message)
            self.has_camera_info = True
            self.camera_frame_id = message.header.frame_id or self.camera_frame_id
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to load camera info: %s", exc)
            self.has_camera_info = False

    def on_robot_pose(self, message):
        self.robot_pose = message

    def on_global_goal_pose(self, message):
        self.global_goal_point = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        self.global_goal_frame = message.header.frame_id or "odom"

    def try_publish(self, image=None):
        output_header = None
        if self.display_sync_mode == "detection_frame":
            with self._tracking_lock:
                if self._display_image is None or self._display_image_header is None:
                    return
                image = self._display_image.copy()
                output_header = self._display_image_header
        if self.latest_image is None:
            return
        if image is None:
            try:
                image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            except CvBridgeError as exc:
                rospy.logwarn("cv_bridge failed to convert Image: %s", exc)
                return
        output_header = output_header or self.latest_image.header
        with self._tracking_lock:
            detections = copy.deepcopy(self.display_detections or self.latest_detections)
        annotated = self.draw_detections(
            image.copy(),
            detections,
            self.latest_scores,
            self.latest_state,
            output_header,
        )
        try:
            output = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("cv_bridge failed to convert cv image back to ROS Image: %s", exc)
            return
        output.header = output_header
        self.pub.publish(output)

    def spin(self):
        rospy.spin()


def main():
    try:
        DetectionVisualizer().spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

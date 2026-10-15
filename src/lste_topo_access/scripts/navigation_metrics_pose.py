"""Pose and Gazebo ground-truth callbacks for navigation telemetry.

The telemetry node is an observer.  This module maintains its pose cache and,
when enabled, a short camera/Gazebo alignment history used only to evaluate
detector recall.  None of these callbacks publishes navigation commands.
"""

import math
import time

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class NavigationMetricsPoseMixin:
    """Maintain frame-aware robot pose and optional evaluation calibration."""

    def _pose_xy_in_frame_locked(self, frame):
        """Return the latest odom pose expressed in ``frame``."""
        if self.pose is None:
            return None
        target_frame = (frame or "odom").strip().lstrip("/") or "odom"
        if target_frame == "odom":
            return self.pose
        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = "odom"
        stamped.pose.position.x = float(self.pose[0])
        stamped.pose.position.y = float(self.pose[1])
        stamped.pose.orientation.z = math.sin(0.5 * float(self.pose[2]))
        stamped.pose.orientation.w = math.cos(0.5 * float(self.pose[2]))
        try:
            self.tf_listener.waitForTransform(
                target_frame, "odom", rospy.Time(0), rospy.Duration(0.02)
            )
            transformed = self.tf_listener.transformPose(target_frame, stamped)
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            self.distance_transform_failures += 1
            rospy.logwarn_throttle(
                3.0,
                "Navigation metrics cannot transform pose odom -> %s: %s",
                target_frame,
                exc,
            )
            return None
        yaw = euler_from_quaternion([
            transformed.pose.orientation.x,
            transformed.pose.orientation.y,
            transformed.pose.orientation.z,
            transformed.pose.orientation.w,
        ])[2]
        return (
            float(transformed.pose.position.x),
            float(transformed.pose.position.y),
            float(yaw),
        )

    def on_odom(self, message):
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        position = message.pose.pose.position
        with self.lock:
            self.last_odom_wall = time.monotonic()
            self.pose = (position.x, position.y, yaw)
            self.target_eval_odom_pose = {
                "position": (float(position.x), float(position.y), float(position.z)),
                "orientation": (
                    float(orientation.x), float(orientation.y),
                    float(orientation.z), float(orientation.w),
                ),
            }
            xy = (position.x, position.y)
            if self.last_pose_xy is not None:
                self.path_length += math.hypot(
                    xy[0] - self.last_pose_xy[0], xy[1] - self.last_pose_xy[1]
                )
            self.last_pose_xy = xy

    def on_pose2d(self, message):
        with self.lock:
            if self.pose is None:
                self.last_odom_wall = time.monotonic()
                self.pose = (message.x, message.y, message.theta)

    def on_target_eval_camera_info(self, message):
        """Capture only camera calibration required by the observer."""
        if not self.target_eval_enabled:
            return
        fx, fy = float(message.K[0]), float(message.K[4])
        if message.width <= 0 or message.height <= 0 or fx <= 0.0 or fy <= 0.0:
            return
        frame = self.target_eval_camera_frame or (
            message.header.frame_id or ""
        ).strip().lstrip("/")
        if not frame:
            return
        with self.lock:
            self.target_eval_camera = {
                "frame": frame,
                "width": int(message.width),
                "height": int(message.height),
                "fx": fx,
                "fy": fy,
                "cx": float(message.K[2]),
                "cy": float(message.K[5]),
            }

    def on_gazebo_model_states(self, message):
        """Record a short world/odom calibration history for evaluation only."""
        if not self.target_eval_enabled:
            return
        with self.lock:
            if self.target_eval_odom_pose is None:
                return
            try:
                target_index = message.name.index(self.target_eval_target_model)
                robot_index = message.name.index(self.target_eval_robot_model)
            except ValueError:
                return
            target_pose = message.pose[target_index]
            robot_pose = message.pose[robot_index]
            self.target_eval_states.append({
                "ros_time": rospy.Time.now().to_sec(),
                "wall_time": time.monotonic(),
                "target_world": {
                    "position": (
                        float(target_pose.position.x), float(target_pose.position.y),
                        float(target_pose.position.z),
                    ),
                    "orientation": (
                        float(target_pose.orientation.x), float(target_pose.orientation.y),
                        float(target_pose.orientation.z), float(target_pose.orientation.w),
                    ),
                },
                "robot_world": {
                    "position": (
                        float(robot_pose.position.x), float(robot_pose.position.y),
                        float(robot_pose.position.z),
                    ),
                    "orientation": (
                        float(robot_pose.orientation.x), float(robot_pose.orientation.y),
                        float(robot_pose.orientation.z), float(robot_pose.orientation.w),
                    ),
                },
                "odom": dict(self.target_eval_odom_pose),
            })
            self.target_eval_last_model_wall = self.target_eval_states[-1]["wall_time"]

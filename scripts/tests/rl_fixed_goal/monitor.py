#!/usr/bin/env python3
"""Persist comparable telemetry for each isolated fixed-goal navigation run."""

import csv
import datetime
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Monitor:
    def __init__(self):
        self.pose = None
        self.goal = None
        self.command = Twist()
        self.controller_status = "awaiting controller"
        self.subgoal = None
        self.scan_minimum = float("nan")
        self.scan_forward_minimum = float("nan")
        self.map_stats = (0, 0, 0, float("nan"), float("nan"))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.8))
        trace_dir = Path(rospy.get_param("~log_dir", "/tmp/rl_fixed_goal_traces"))
        trace_dir.mkdir(parents=True, exist_ok=True)
        started = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trace_path = trace_dir / ("run_" + started + ".csv")
        self.trace_file = self.trace_path.open("w", newline="")
        self.trace = csv.writer(self.trace_file)
        self.trace.writerow((
            "ros_time", "x", "y", "yaw", "goal_x", "goal_y", "distance",
            "local_goal_x", "local_goal_y", "cmd_linear_x", "cmd_angular_z",
            "scan_minimum", "scan_forward_minimum", "subgoal_x", "subgoal_y",
            "map_free_cells", "map_occupied_cells", "map_unknown_cells",
            "map_resolution", "map_stamp", "controller_status",
        ))
        self.trace_file.flush()

        rospy.Subscriber("/pro3/wheel_odom", Odometry, self.on_odom, queue_size=1)
        goal_topic = rospy.get_param("~goal_topic", "/rl_fixed_goal_test/final_goal")
        rospy.Subscriber(goal_topic, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.on_command, queue_size=1)
        rospy.Subscriber("/rl_fixed_goal_test/controller_status", String, self.on_controller_status, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_subgoal, queue_size=1)
        rospy.Subscriber("/pro3/rlscan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.report)
        rospy.on_shutdown(self.close)
        rospy.loginfo("RL fixed-goal trace: %s", self.trace_path)

    def on_odom(self, message):
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        position = message.pose.pose.position
        self.pose = (position.x, position.y, yaw)

    def on_goal(self, message):
        self.goal = (message.pose.position.x, message.pose.position.y)

    def on_command(self, message):
        self.command = message

    def on_controller_status(self, message):
        self.controller_status = message.data

    def on_subgoal(self, message):
        self.subgoal = (message.pose.position.x, message.pose.position.y)

    def on_scan(self, message):
        ranges = [distance for distance in message.ranges if math.isfinite(distance)]
        self.scan_minimum = min(ranges) if ranges else float("nan")
        forward = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if math.isfinite(distance) and abs(angle) <= math.radians(20.0):
                forward.append(distance)
        self.scan_forward_minimum = min(forward) if forward else float("nan")

    def on_map(self, message):
        cells = message.data
        self.map_stats = (
            cells.count(0), cells.count(100), cells.count(-1),
            message.info.resolution, message.header.stamp.to_sec(),
        )

    def close(self):
        if not self.trace_file.closed:
            self.trace_file.close()

    def report(self, _event):
        if self.pose is None or self.goal is None:
            return
        x, y, yaw = self.pose
        goal_x, goal_y = self.goal
        distance = math.hypot(goal_x - x, goal_y - y)
        dx = goal_x - x
        dy = goal_y - y
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        recovery_turn = local_x < -0.5
        recovery_direction = "right" if local_y < 0.0 else "left"
        subgoal_x, subgoal_y = self.subgoal if self.subgoal is not None else (float("nan"), float("nan"))
        self.trace.writerow((
            rospy.Time.now().to_sec(), x, y, yaw, goal_x, goal_y, distance,
            local_x, local_y, self.command.linear.x, self.command.angular.z,
            self.scan_minimum, self.scan_forward_minimum, subgoal_x, subgoal_y,
            *self.map_stats, self.controller_status,
        ))
        self.trace_file.flush()
        rospy.loginfo(
            "RL_FIXED_GOAL_TEST pose=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f) "
            "distance=%.2f local_goal=(%.2f,%.2f) recovery_turn=%s:%s "
            "cmd=(%.2f,%.2f) controller=[%s]%s",
            x, y, yaw, goal_x, goal_y, distance,
            local_x, local_y, recovery_turn, recovery_direction,
            self.command.linear.x, self.command.angular.z,
            self.controller_status,
            " REACHED" if distance <= self.goal_tolerance else "",
        )


def main():
    rospy.init_node("rl_fixed_goal_monitor")
    Monitor()
    rospy.spin()


if __name__ == "__main__":
    main()

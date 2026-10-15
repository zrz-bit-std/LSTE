#!/usr/bin/env python3
"""Publish a fixed test goal, optionally replaced by Gazebo Shift-click goals."""

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("rl_fixed_goal_publisher")
    topic = rospy.get_param("~topic", "/rl_fixed_goal_test/final_goal")
    goal_x = float(rospy.get_param("~goal_x"))
    goal_y = float(rospy.get_param("~goal_y"))
    frame_id = rospy.get_param("~frame_id", "odom")
    allow_click_goal = str(rospy.get_param("~allow_click_goal", False)).lower() in (
        "1", "true", "yes", "on",
    )
    click_topic = rospy.get_param("~click_goal_topic", "/move_base/current_goal")

    publisher = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    active_goal = [goal_x, goal_y]

    def publish_active_goal():
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame_id
        message.pose.position.x = active_goal[0]
        message.pose.position.y = active_goal[1]
        message.pose.orientation.w = 1.0
        publisher.publish(message)

    def on_click_goal(message):
        # The Gazebo click plugin reports world coordinates. In this isolated
        # world the wheel odometry is intentionally initialized in the same
        # world coordinate system, so the coordinate pair is directly usable.
        active_goal[0] = message.pose.position.x
        active_goal[1] = message.pose.position.y
        rospy.logwarn(
            "Gazebo click replaced fixed test goal: (%.2f, %.2f) -> (%.2f, %.2f)",
            goal_x, goal_y, active_goal[0], active_goal[1],
        )
        # Forward the clicked target immediately. The periodic publication
        # below is retained for late subscribers and transient ROS failures.
        publish_active_goal()

    if allow_click_goal:
        rospy.Subscriber(click_topic, PoseStamped, on_click_goal, queue_size=1)
        rospy.loginfo("Gazebo Shift-click goal override enabled on %s", click_topic)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        publish_active_goal()
        rate.sleep()


if __name__ == "__main__":
    main()

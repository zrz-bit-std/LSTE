#!/usr/bin/env python3
"""Render the test-only goal as a moving sphere inside Gazebo Classic."""

import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import PoseStamped


MODEL_NAME = "rl_fixed_goal_marker"
LEGACY_MODEL_NAME = "rl_fixed_goal_sphere"
SPHERE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='rl_fixed_goal_marker'>
    <!-- A non-static, gravity-free model can be moved reliably by Gazebo
         Classic and immediately refreshes in the GUI. -->
    <static>false</static>
    <link name='goal_ball'>
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <visual name='visual'>
        <cast_shadows>false</cast_shadows>
        <geometry><sphere><radius>0.42</radius></sphere></geometry>
        <material>
          <ambient>0.00 1.00 0.05 1.00</ambient>
          <diffuse>0.00 1.00 0.05 1.00</diffuse>
          <emissive>0.00 0.70 0.04 1.00</emissive>
        </material>
      </visual>
    </link>
    <!-- The mast makes the target visible when furniture hides the ball. -->
    <link name='beacon'>
      <pose>0 0 1.15 0 0 0</pose>
      <visual name='visual'>
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>0.06</radius><length>1.40</length></cylinder></geometry>
        <material>
          <ambient>0.00 1.00 0.05 1.00</ambient>
          <diffuse>0.00 1.00 0.05 1.00</diffuse>
          <emissive>0.00 0.70 0.04 1.00</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class GoalSphere:
    def __init__(self):
        self.height = float(rospy.get_param("~height", 0.45))
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_fixed_goal_test/final_goal")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.last_goal = None

        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/delete_model")
        # Do not flash a marker at world origin during startup. The fixed-goal
        # publisher is latched, so this returns the current YAML/click target.
        initial_goal = rospy.wait_for_message(self.goal_topic, PoseStamped)
        self.ensure_model(initial_goal, replace=True)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.on_goal, queue_size=1)
        self.on_goal(initial_goal)
        rospy.loginfo("RL fixed-goal sphere follows %s", self.goal_topic)

    def ensure_model(self, initial_goal, replace=False):
        world_properties = self.get_world_properties()
        if LEGACY_MODEL_NAME in world_properties.model_names:
            response = self.delete_model(LEGACY_MODEL_NAME)
            if not response.success:
                rospy.logwarn("Unable to remove legacy goal sphere: %s", response.status_message)
        if MODEL_NAME in world_properties.model_names:
            if not replace:
                return
            response = self.delete_model(MODEL_NAME)
            if not response.success:
                raise RuntimeError("Failed to replace goal sphere: %s" % response.status_message)
            self.wait_for_model(MODEL_NAME, present=False)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        initial_pose = ModelState().pose
        initial_pose.position.x = initial_goal.pose.position.x
        initial_pose.position.y = initial_goal.pose.position.y
        initial_pose.position.z = self.height
        initial_pose.orientation.w = 1.0
        response = spawn_model(MODEL_NAME, SPHERE_SDF, "", initial_pose, "world")
        if not response.success:
            raise RuntimeError("Failed to spawn goal sphere: %s" % response.status_message)
        self.wait_for_model(MODEL_NAME, present=True)

    def wait_for_model(self, model_name, present):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            exists = model_name in self.get_world_properties().model_names
            if exists == present:
                return
            time.sleep(0.05)
        expected = "appear" if present else "disappear"
        raise RuntimeError("Timed out waiting for %s to %s" % (model_name, expected))

    def on_goal(self, message):
        goal = (message.pose.position.x, message.pose.position.y)
        if self.last_goal == goal:
            return

        state = ModelState()
        state.model_name = MODEL_NAME
        state.reference_frame = "world"
        state.pose.position.x = goal[0]
        state.pose.position.y = goal[1]
        state.pose.position.z = self.height
        state.pose.orientation.w = 1.0
        response = self.set_state(state)
        if not response.success:
            # Recover once if Gazebo removed the visual model unexpectedly.
            rospy.logwarn("Goal marker missing; recreating it: %s", response.status_message)
            self.ensure_model(message)
            response = self.set_state(state)
            if not response.success:
                rospy.logwarn_throttle(5.0, "Unable to move goal sphere: %s", response.status_message)
                return
        self.last_goal = goal


def main():
    rospy.init_node("rl_fixed_goal_sphere")
    GoalSphere()
    rospy.spin()


if __name__ == "__main__":
    main()

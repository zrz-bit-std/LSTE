import time
import rospy
import copy
import tf
import numpy as np
import math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import Int8, Bool
from model.utils import test_init_pose, test_goal_point
from tf.transformations import *



class StageWorld():
    def __init__(self, beam_num, index, num_env):
        self.index = index
        self.num_env = num_env
        node_name = 'StageEnv_' + str(index)
        rospy.init_node(node_name, anonymous=None)

        self.beam_mum = beam_num
        self.laser_cb_num = 0
        self.scan = None

        # used in reset_world
        self.self_speed = [0.0, 0.0]
        self.step_goal = [0., 0.]
        self.step_r_cnt = 0.

        # used in generate goal point
        self.map_size = np.array([8., 8.], dtype=np.float32)  # 20x20m
        # The original training environment used an 0.8 m terminal radius.
        # In the live LSTE pipeline, visual-servo and map waypoints are often
        # only 1--2 m away, so that radius can end a controller episode before
        # the robot has made useful progress. Keep the legacy default while
        # making the live tolerance an explicit controller parameter.
        self.goal_size = max(0.05, float(rospy.get_param('~goal_size', 0.8)))
        # Exploration publishes short odom waypoints.  They are handoff
        # points, not the final visual-task terminal, so use a slightly wider
        # radius to avoid spinning in place when the robot is already beside
        # a waypoint but a stale lidar cell keeps the exact point unreachable.
        self.intermediate_goal_size = max(
            self.goal_size,
            float(rospy.get_param('~intermediate_goal_size', 0.45)),
        )

        self.robot_value = 10.
        self.goal_value = 0.

        # Defaults match sceneb, the pure SA-PPO validation scene. Individual
        # launch commands can override these without editing the environment.
        self.goalx = rospy.get_param('~goal_x', 10.5)
        self.goaly = rospy.get_param('~goal_y', 6.0)
        self.linear_speed_scale = rospy.get_param('~linear_speed_scale', 0.8)
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base/current_goal')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.wait_for_goal = rospy.get_param('~wait_for_goal', False)
        self.goal_update_epsilon = rospy.get_param('~goal_update_epsilon', 0.05)
        self.subscribe_gp_subgoal = rospy.get_param('~subscribe_gp_subgoal', False)
        # Normal LSTE goals are short frontier/visual waypoints.  Reaching one
        # must not terminate the SA-PPO process: Goal Manager will publish the
        # next segment.  Fixed-goal validation leaves this disabled and keeps
        # the legacy terminal-radius behavior.
        self.allow_intermediate_goals = bool(rospy.get_param(
            '~allow_intermediate_goals', False
        ))
        self.task_done = False
        self.task_done_received = False
        # The old implementation appended every odometry sample to three text
        # files forever. Formal experiments use timestamped launcher logs;
        # retain this legacy trace only when someone explicitly asks for it.
        self.legacy_trace_enabled = rospy.get_param('~legacy_trace_enabled', False)
        self.goal_received = not self.wait_for_goal

        # scenea
        # self.goalx = 6.0
        # self.goaly = -8.0

        # sceneb
        # self.goalx = 10.5
        # self.goaly = 6.0

        #self.goalx = 9.5
        #self.goaly = 7.0


        #self.goalx = 17.5
        #self.goaly = 0.0

        # rl_scene4
        #self.goalx = 16.5
        #self.goaly = 12.0

        # rl_scene1
        # self.goalx = -20
        # self.goaly = -0

        # zcy_scene2
        # self.goalx = 7.0
        # self.goaly = 1.6

        #zcyscene1
        # self.final_goalx = 22.0
        # self.final_goaly = 10.0

        # zcyscene3: pass _goal_x:=19.5 _goal_y:=1.0 when launching the node.

        # zcyscene5
        # self.goalx = 0.0
        # self.goaly = -8.5
        #
        # zcyscene5
        # self.final_goalx = 0.0
        # self.final_goaly = -8.5

        # zcy_scene6
        # self.goalx = 19.5
        # self.goaly = -17.5

        # zcy_scene8
        # self.goalx = 21.0
        # self.goaly = 16.5

        # scene7
        # self.final_goalx = 22.0
        # self.final_goaly = -6.0

        # # scene8
        # self.final_goalx = 21.0
        # self.final_goaly = 16.5

        # zcy_dynamic1
        # self.goalx = 9.2
        # self.goaly = 0


        self.terminate = False
        self.statex = 0
        self.statey = 0
        self.statetheta = 0
        self.rl_goalx = 0
        self.rl_goaly = 0
        self.goal_basedx = 0
        self.goal_basedy = 0
        self.goal_basedtheta = 0
        self.goal_point = [0.0, 0.0]
        self.distance = 0.
        self._path = Path()
        self.stop = True
        self.turn = False
        self.right_turn = False

        # -----------Publisher and Subscriber-------------

        # 自动检测并选择激光与里程计话题（兼容 Stage 的常见命名）
        try:
            pubs = [t for t, _ in rospy.get_published_topics()]
        except Exception:
            pubs = []

        def _find_topic(candidates):
            for c in candidates:
                for t in pubs:
                    if t == c or t.endswith('/' + c) or t.endswith(c):
                        return t
            return None

        # 常见激光话题候选
        laser_candidates = ['rlscan', 'base_scan', 'scan', '/rlscan', '/base_scan', '/scan']
        found_laser = _find_topic(laser_candidates)
        if found_laser:
            laser_topic = found_laser
        else:
            laser_topic = 'rlscan'  # fallback to original name

        # 常见里程计候选
        odom_candidates = ['wheel_odom', 'odom', '/wheel_odom', '/odom']
        found_odom = _find_topic(odom_candidates)
        if found_odom:
            odom_topic = found_odom
        else:
            odom_topic = 'wheel_odom'

        # Debug 信息：打印所选话题，便于排查
        try:
            rospy.loginfo('Subscribed laser topic: %s', laser_topic)
            rospy.loginfo('Subscribed odom topic: %s', odom_topic)
        except Exception:
            print('Subscribed laser topic:', laser_topic)
            print('Subscribed odom topic:', odom_topic)

        self.laser_sub = rospy.Subscriber(laser_topic, LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)




        self.sim_clock = rospy.Subscriber('clock', Clock, self.sim_clock_callback)

        self.sub_local_goal = rospy.Subscriber(self.goal_topic, PoseStamped, self.rlgetlocalgoal)
        self.sub_task_done = rospy.Subscriber(
            rospy.get_param('~task_done_topic', '/lste/task_done'),
            Bool,
            self.task_done_callback,
            queue_size=1,
        )
        self.rl_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.loginfo('Subscribed goal topic: %s (wait_for_goal=%s)',
                      self.goal_topic, self.wait_for_goal)

        self.gp_subgl_sub = None
        if self.subscribe_gp_subgoal:
            self.gp_subgl_sub = rospy.Subscriber('/gp_subgoal', PoseStamped, self._gpgl_callback)
            rospy.logwarn('Legacy /gp_subgoal input is enabled and may replace goals from %s',
                          self.goal_topic)


        # -----------Service-------------------
        self.reset_stage = rospy.ServiceProxy('reset_positions', Empty)

        # # Wait until the first callback
        self.speed = None
        self.state = None
        self.speed_GT = None
        self.state_GT = None
        self.is_crashed = None


        rospy.sleep(1.)


    def laser_scan_callback(self, scan):
        self.scan_param = [scan.angle_min, scan.angle_max, scan.angle_increment, scan.time_increment,
                           scan.scan_time, scan.range_min, scan.range_max]
        self.scan = np.array(scan.ranges)
        self.laser_cb_num += 1


    def odometry_callback(self, odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.statex = odometry.pose.pose.position.x
        self.statey = odometry.pose.pose.position.y
        self.statetheta = Euler[2]
        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]
        statex = float(self.statex)
        statey = float(self.statey)
        if self.legacy_trace_enabled:
            with open("./sappo_path.txt", 'a') as path:
                path.write(str(statex))
                path.write(',')
                path.write(str(statey))
                path.write(',')
            with open("./sappo_linear.txt", 'a') as linear:
                linear.write(str(odometry.twist.twist.linear.x))
                linear.write(',')
            with open("./sappo_angular.txt", 'a') as angular:
                angular.write(str(odometry.twist.twist.angular.z))
                angular.write(',')




    def sim_clock_callback(self, clock):
        self.sim_time = clock.clock.secs + clock.clock.nsecs / 1000000000.

    def crash_callback(self, flag):
        self.is_crashed = flag.data

    def get_self_stateGT(self):
        return self.state_GT

    def get_self_speedGT(self):
        return self.speed_GT
	######
    def get_laser_observation(self):
        # 1) 等待直到收到第一帧真实的 laser scan（更安全，防止用“假数据”误导策略）
        while self.scan is None and not rospy.is_shutdown():
            print("Waiting for laser scan...") # 可选：打印等待信息
            rospy.sleep(0.1)

        # 极端情况下仍为 None（比如退出或异常），返回全 0 的特征作为 fallback
        if self.scan is None:
            return np.zeros(self.beam_mum, dtype=np.float32)

        # 2) 确保类型安全，处理 NaN/Inf，并裁剪到合理范围
        scan = np.array(self.scan, dtype=np.float32).copy()
        scan[np.isnan(scan)] = 6.0
        scan[np.isinf(scan)] = 6.0
        # 将下限设为 0.01，避免后续取倒数时出现除以零的问题
        scan = np.clip(scan, 0.01, 6.0)

        # 3) 原有稀疏化逻辑（保持不变）
        raw_beam_num = len(scan)
        sparse_beam_num = self.beam_mum
        # 如果原始点数小于稀疏目标，直接重复/截断以避免索引错误
        if raw_beam_num == 0:
            return np.zeros(sparse_beam_num, dtype=np.float32)

        step = float(raw_beam_num) / sparse_beam_num
        sparse_scan_left = []
        index = 0.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_left.append(scan[int(index)])
            index += step
        sparse_scan_right = []
        index = raw_beam_num - 1.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_right.append(scan[int(index)])
            index -= step
        scan_sparse = np.concatenate((sparse_scan_left, sparse_scan_right[::-1]), axis=0)

        # 4) 返回倒数距离（安全：前面 clip 保证最小为 0.01）
        return 1.0 / scan_sparse
        # return scan_sparse / 6.0 - 0.5


    def get_self_speed(self):
        return self.speed

    def get_self_state(self):
        return self.state

    def get_crash_state(self):
        return self.is_crashed

    def has_goal(self):
        return self.goal_received

    def get_sim_time(self):
        return self.sim_time

#     def get_local_goal(self):
#         #[x, y, theta] = self.get_self_stateGT()
#         #[goal_x, goal_y] = self.goal_point
#         #local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
#         #local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
#         #[rl_goal_basedx,rl_goal_basedy,rl_goal_basedtheta]=self.get_self_state()
#         #rl_global_goalx = self.rl_goalx - rl_goal_basedx
#         #rl_global_goaly = self.rl_goaly - rl_goal_basedy
#         #self.goalx = rl_global_goalx * np.cos(self.goal_basedtheta * math.pi) + rl_global_goaly * np.sin(rl_goal_basedtheta * math.pi)
#         #self.goalx = -rl_global_goalx * np.sin(self.goal_basedtheta) + rl_global_goaly * np.cos(self.goal_basedtheta)
#         #self.goaly = rl_global_goalx * np.sin(-self.goal_basedtheta * math.pi) + rl_global_goaly * np.cos(rl_goal_basedtheta * math.pi)
#         #self.goaly = -rl_global_goalx * np.cos(self.goal_basedtheta) - rl_global_goaly * np.sin(self.goal_basedtheta)
#
#
#         rl_path = self.path_to_numpy(self._path)
#         my_rl_path = np.array(rl_path)
#         if my_rl_path.ndim == 2:
#            self.stop = False
#            i = 0
#            while i < len(rl_path):
#                  if np.sqrt((my_rl_path[i,0] - self.statex) ** 2 + (my_rl_path[i,1] - self.statey) ** 2)>5.0:
#                     break
#                  i +=1
# #        local_x = (self.goalx - self.statex) * np.cos(self.statetheta) + (self.goaly - self.statey) * np.sin(self.statetheta)
# #        local_y = -(self.goalx - self.statex) * np.sin(self.statetheta) + (self.goaly - self.statey) * np.cos(self.statetheta)
#            if i < len(rl_path):
#                goalx = my_rl_path[i-1,0]
#                goaly = my_rl_path[i-1,1]
# #           else:
# #              self.stop = True
# #              goalx = self.statex
# #              goaly = self.statey
#         else:
#            self.stop = True
#            goalx = self.statex
#            goaly = self.statey
#         local_x = (self.goalx - self.statex) * np.cos(self.statetheta) + (self.goaly - self.statey) * np.sin(self.statetheta)
#         local_y = -(self.goalx - self.statex) * np.sin(self.statetheta) + (self.goaly - self.statey) * np.cos(self.statetheta)
#         pub_local_goal = PoseStamped()
#         pub_local_goal.pose.position.x = local_x
#         pub_local_goal.pose.position.y = local_y
#         if local_x < 0.5:
#            self.turn = True
#            if local_y < 0:
#               self.right_turn = True
#            else:
#               self.right_turn = False
#         else:
#            self.turn = False
#         self.pub_local_goal.publish(pub_local_goal)
#         return [local_x, local_y]
# #        return [0, 2]

    def get_local_goal(self):

        local_x = (self.goalx - self.statex) * np.cos(self.statetheta) + (self.goaly - self.statey) * np.sin(
            self.statetheta)
        local_y = -(self.goalx - self.statex) * np.sin(self.statetheta) + (self.goaly - self.statey) * np.cos(
            self.statetheta)

        # Recovery turning is only needed when the target is behind the robot.
        # A target to the side can briefly have a small forward component; forcing
        # a turn in that case makes the direction flip every control cycle.
        if local_x < -0.5:
            self.turn = True
            if local_y < 0:
                self.right_turn = True
            else:
                self.right_turn = False
        else:
            self.turn = False
        # print(f"local_x: {local_x}, local_y: {local_y}, turn: {self.turn},right_turn: {self.right_turn}")
        return [local_x, local_y]

    def reset_world(self):
        self.reset_stage()
        self.self_speed = [0.0, 0.0]
        self.step_goal = [0., 0.]
        self.step_r_cnt = 0.
        self.start_time = time.time()
        rospy.sleep(0.5)


    def generate_goal_point(self):
        self.goal_point = self.get_local_goal()
        self.pre_distance = 0
        self.distance = copy.deepcopy(self.pre_distance)


	#######需要获得全局目标计算distance
    def get_reward_and_terminate(self, t):
        terminate = False
        laser_scan = self.get_laser_observation()
        [x, y, theta] = self.get_self_state()
        [v, w] = self.get_self_speed()
        #[goal_basedx,goal_basedy,goal_basedtheta]=self.get_goal_based_state()
        #goalx_global = self.goal_point[0] * np.cos(-goal_basedtheta) + self.goal_point[1] * np.sin(-goal_basedtheta) + goal_basedx
        #goaly_global = -self.goal_point[0] * np.sin(-goal_basedtheta) + self.goal_point[1] * np.cos(-goal_basedtheta) + goal_basedy
        self.pre_distance = copy.deepcopy(self.distance)
        self.distance = np.sqrt((self.goalx - x) ** 2 + (self.goaly -y) ** 2)
        #self.distance = np.sqrt((self.goalx) ** 2 + (self.goaly) ** 2)
        reward_g = (self.pre_distance - self.distance) * 2.5
        reward_c = 0
        reward_w = 0
        result = 0

        is_crash = self.get_crash_state()

        goal_radius = (
            self.intermediate_goal_size
            if self.allow_intermediate_goals else self.goal_size
        )
        if self.distance < goal_radius:
            terminate = True
            reward_g = 15
            result = 'Reach Goal'
            self.terminate = True

        if is_crash == 1:
            terminate = True
            reward_c = -15.
            result = 'Crashed'

        if np.abs(w) >  0.7:
            reward_w = -0.1 * np.abs(w)

        if t > 10000:
            terminate = True
            result = 'Time out'
        reward = reward_g + reward_c + reward_w

        return reward, terminate, result

    def reset_pose(self):

        reset_pose = test_init_pose(self.index)
        self.control_pose(reset_pose)

    def control_rl_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed_scale * action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = 1.2 * action[1]    # 1.0 # 1.4 #1.5 #1.2
        self.rl_cmd_vel.publish(move_cmd)


    def control_pose(self, pose):
        pose_cmd = Pose()
        assert len(pose)==3
        pose_cmd.position.x = pose[0]
        pose_cmd.position.y = pose[1]
        pose_cmd.position.z = 0

        qtn = tf.transformations.quaternion_from_euler(0, 0, pose[2], 'rxyz')
        pose_cmd.orientation.x = qtn[0]
        pose_cmd.orientation.y = qtn[1]
        pose_cmd.orientation.z = qtn[2]
        pose_cmd.orientation.w = qtn[3]
        self.cmd_pose.publish(pose_cmd)


    def generate_random_pose(self):
        [x_robot, y_robot, theta] = self.get_self_stateGT()
        x = np.random.uniform(9, 19)
        y = np.random.uniform(0, 1)
        if y <= 0.4:
            y = -(y * 10 + 1)
        else:
            y = -(y * 10 + 9)
        dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        while (dis_goal < 7) and not rospy.is_shutdown():
            x = np.random.uniform(9, 19)
            y = np.random.uniform(0, 1)
            if y <= 0.4:
                y = -(y * 10 + 1)
            else:
                y = -(y * 10 + 9)
            dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        theta = np.random.uniform(0, 2*np.pi)
        return [x, y, theta]

    def generate_random_goal(self):
        [x_robot, y_robot, theta] = self.get_self_stateGT()
        x = np.random.uniform(9, 19)
        y = np.random.uniform(0, 1)
        if y <= 0.4:
            y = -(y*10 + 1)
        else:
            y = -(y*10 + 9)
        dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        while (dis_goal < 7) and not rospy.is_shutdown():
            x = np.random.uniform(9, 19)
            y = np.random.uniform(0, 1)
            if y <= 0.4:
                y = -(y * 10 + 1)
            else:
                y = -(y * 10 + 9)
            dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        return [x, y]

    def rlgetlocalgoal(self, goal_msg):
        goalx = goal_msg.pose.position.x
        goaly = goal_msg.pose.position.y
        if not np.isfinite(goalx) or not np.isfinite(goaly):
            rospy.logwarn('Ignoring non-finite goal from %s: (%s, %s)',
                          self.goal_topic, goalx, goaly)
            return

        # Goal Manager republishes its current goal periodically. Once that goal
        # has been reached, an identical message must not re-arm the controller.
        goal_changed = (not self.goal_received or
                        np.hypot(goalx - self.goalx, goaly - self.goaly) > self.goal_update_epsilon)
        if self.terminate and not goal_changed:
            return

        self.goalx = goalx
        self.goaly = goaly
        self.goal_received = True
        if goal_changed:
            self.terminate = False
            rospy.loginfo('Accepted goal from %s: x=%.2f y=%.2f frame=%s',
                          self.goal_topic, goalx, goaly,
                          goal_msg.header.frame_id or '<unspecified>')

    def task_done_callback(self, message):
        """Track final task completion separately from waypoint arrival."""
        self.task_done_received = True
        self.task_done = bool(message.data)

    def get_goal_based_state(self):
        return [self.goal_basedx,self.goal_basedy,self.goal_basedtheta]





    def _gpgl_callback(self,data):
        self.rlgetlocalgoal(data)

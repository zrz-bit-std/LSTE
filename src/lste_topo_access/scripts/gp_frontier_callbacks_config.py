"""One focused responsibility extracted from the legacy GP frontier node."""

import os

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from lste_msgs.msg import LsteState
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from gp_frontier_common import resolve_topo_path, safe_load_yaml

class GpFrontierCallbacksAndConfigurationMixin:
    def lste_state_cb(self, state_msg: LsteState):
        """缓存 /lste/state，当前仅用于日志观察。"""
        try:
            new_state = int(state_msg.state)
        except Exception:
            return
        new_sub = state_msg.subtype or ""
        if new_state != self.lste_state or new_sub != self.lste_subtype:
            rospy.loginfo("gp_subgoal: /lste/state changed state=%s subtype=%s",
                          str(new_state), new_sub)
            self.state_events.append({
                "t": rospy.Time.now().to_sec(),
                "state": new_state,
                "subtype": new_sub,
                "pose": {"x": getattr(self.pose, "x", None), "y": getattr(self.pose, "y", None)},
                "anchor_id": self.anchor_last_id,
                "profile": self.current_profile,
                "active_mode": self.active_mode,
            })
        self.lste_state = new_state
        self.lste_subtype = new_sub
        # 根据 state/subtype 切换 explore 配置：Sus-C 用 fine，其余用 coarse
        desired_profile = "sus_c" if (new_state == 1 and new_sub == "Sus-C") else "pass"
        if desired_profile != getattr(self, "current_profile", None):
            self.apply_access_topo_profile(desired_profile, reason="/lste/state")

    def active_mode_cb(self, msg: String):
        mode = msg.data or "unknown"
        if mode != self.active_mode:
            rospy.loginfo("gp_subgoal: active_mode changed to %s", mode)
        self.active_mode = mode

    def final_goal_cb(self, msg: PoseStamped):
        """Update final goal from /lste/final_goal (odom frame PoseStamped)."""
        self.gl_x = msg.pose.position.x
        self.gl_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.gl_yaw = yaw
        self.gl_wrt_odom = np.array([self.gl_x, self.gl_y, 1], dtype="float32")
        self.final_goal_received = True
        self.goal_events.append({
            "t": rospy.Time.now().to_sec(),
            "goal": {"x": self.gl_x, "y": self.gl_y, "yaw": self.gl_yaw},
            "mode": self.active_mode,
            "profile": self.current_profile,
            "state": self.lste_state,
            "subtype": self.lste_subtype,
            "anchor_id": self.anchor_last_id,
            "pose": {"x": getattr(self.pose, "x", None), "y": getattr(self.pose, "y", None)},
        })

    def vanish_cb(self, msg: Pose):
        """消失点推断的前方距离，单位 m。"""
        try:
            dist = float(msg.position.x)
            if dist > 0:
                self.vanish_distance = dist
        except Exception:
            pass

    def load_access_topo_config(self):
        """加载 Access-Topo 配置并冻结本次运行的快照文件路径。

        ``pass`` 与 ``sus_c`` 可以使用不同的拓扑参数。profile 切换会改变
        分支判定/回退等阈值，但不会更换本次运行的 JSON 文件，确保一份
        access_topo_<run_id>.json 记录完整的一次实验。
        """
        # 两套配置路径：未提供 fine 时回落到 coarse
        default_cfg = resolve_topo_path('topo_tree/cfgs/access_topo.yaml')
        cfg_path_pass = rospy.get_param('~access_topo_config_pass',
                                        rospy.get_param('~access_topo_config', default_cfg))
        cfg_path_sus_c = rospy.get_param('~access_topo_config_sus_c', cfg_path_pass)
        self.cfg_pass = safe_load_yaml(cfg_path_pass) or {}
        self.cfg_sus_c = safe_load_yaml(cfg_path_sus_c) or {}
        # 冻结本次运行的保存路径/文件（不随 profile/state 变化）
        base_root = self.cfg_pass.get('topo_tree_root') or \
                    rospy.get_param('~topo_tree_root', resolve_topo_path(''))
        default_save_dir = os.path.join(base_root, rospy.get_param('~run_name', "run"))
        self.topo_save_dir = rospy.get_param('~topo_save_dir',
                                             self.cfg_pass.get('topo_save_dir', default_save_dir))
        self.topo_save_file = os.path.join(self.topo_save_dir, f"access_topo_{self.run_id}.json")
        self.current_profile = "pass"
        self.apply_access_topo_profile(self.current_profile, reason="init")

    def apply_access_topo_profile(self, profile: str, reason: str = ""):
        """按 profile 应用配置并更新派生参数。profile: 'pass' or 'sus_c'."""
        cfg_file = self.cfg_pass if profile != "sus_c" else self.cfg_sus_c
        if not isinstance(cfg_file, dict):
            cfg_file = {}
        prev = getattr(self, "current_profile", None)
        self.current_profile = profile
        if prev is not None and prev != profile:
            rospy.loginfo("Access-Topo profile switch: %s -> %s (reason=%s)", prev, profile, reason or "")
            self.profile_events.append({
                "t": rospy.Time.now().to_sec(),
                "from": prev,
                "to": profile,
                "reason": reason,
                "pose": {"x": getattr(self.pose, "x", None), "y": getattr(self.pose, "y", None)},
                "anchor_id": self.anchor_last_id,
            })

        # 基础路径与测试名
        self.test_name = cfg_file.get('test_name', rospy.get_param('~test_name', 'default'))
        self.topo_tree_root = cfg_file.get('topo_tree_root') or \
                               rospy.get_param('~topo_tree_root', resolve_topo_path(''))
        # 参数读取（YAML 优先，ROS param 次之）
        self.anchor_step_dist = cfg_file.get('anchor_step_dist', rospy.get_param('~anchor_step_dist', 1.0))
        self.backtrack_arrive_dist = cfg_file.get('backtrack_arrive_dist',
                                                 rospy.get_param('~backtrack_arrive_dist', 1.0))
        self.visited_thresh = cfg_file.get('visited_thresh', rospy.get_param('~visited_thresh', 3.0))
        self.force_window_deg = cfg_file.get('force_window_deg', rospy.get_param('~force_window_deg', 25.0))
        # 前方可行距离过滤开关（默认关闭；避免在走廊/路口远处因短暂预测误差把 frontier 过滤掉）
        self.enable_min_frontier_clearance = bool(
            cfg_file.get('enable_min_frontier_clearance',
                         rospy.get_param('~enable_min_frontier_clearance', False))
        )
        # 前方可行距离过滤阈值（小于该距离的 frontier 视为不可走）
        self.min_frontier_clearance = cfg_file.get('min_frontier_clearance',
                                                   rospy.get_param('~min_frontier_clearance', 0.0))
        self.commit_dist = cfg_file.get('commit_dist', rospy.get_param('~commit_dist', 2.0))
        self.cluster_eps_deg = cfg_file.get('cluster_eps_deg', rospy.get_param('~cluster_eps_deg', 15.0))
        self.cluster_stable_duration = cfg_file.get('cluster_stable_duration',
                                                    rospy.get_param('~cluster_stable_duration', 3.0))
        self.cluster_lost_timeout = cfg_file.get('cluster_lost_timeout',
                                                 rospy.get_param('~cluster_lost_timeout', 1.0))
        # 路口决策：走出一定距离后用实际运动方向确定“被选分支”
        self.junction_decision_dist = cfg_file.get(
            'junction_decision_dist', rospy.get_param('~junction_decision_dist', self.anchor_step_dist))
        self.junction_same_dir_gate_deg = cfg_file.get(
            'junction_same_dir_gate_deg', rospy.get_param('~junction_same_dir_gate_deg', 30.0))
        # 路口窗口：离开路口多远/等待多久后固化分支（距离优先）
        self.junction_finalize_dist = cfg_file.get(
            'junction_finalize_dist', rospy.get_param('~junction_finalize_dist', self.anchor_step_dist * 1.5))
        self.junction_finalize_timeout = cfg_file.get(
            'junction_finalize_timeout', rospy.get_param('~junction_finalize_timeout', 6.0))
        # 路口会话触发：只有“双峰持续存在”才认为进入路口会话（用于过滤伪路口）
        self.junction_enter_confirm_sec = cfg_file.get(
            'junction_enter_confirm_sec', rospy.get_param('~junction_enter_confirm_sec', 0.8))
        # 被认为是“峰”的最小持续时间（秒）：单峰出现太短不计入多峰判断
        self.junction_peak_min_age = cfg_file.get(
            'junction_peak_min_age', rospy.get_param('~junction_peak_min_age', 0.6))
        # 双峰最小角度间隔（度）：避免同一方向被误分成两簇
        self.junction_peak_min_sep_deg = cfg_file.get(
            'junction_peak_min_sep_deg',
            rospy.get_param('~junction_peak_min_sep_deg', max(30.0, float(self.cluster_eps_deg) * 1.2)))
        # 第二峰权重阈值：绝对值 + 相对比例（用于过滤很小的伪峰）
        self.junction_peak_min_weight = cfg_file.get(
            'junction_peak_min_weight', rospy.get_param('~junction_peak_min_weight', 0.0))
        self.junction_second_weight_ratio = cfg_file.get(
            'junction_second_weight_ratio', rospy.get_param('~junction_second_weight_ratio', 0.15))
        # 相邻 anchor 的兴趣方向合并距离（米），用于全局去重
        self.branch_merge_dist = cfg_file.get('branch_merge_dist',
                                             rospy.get_param('~branch_merge_dist', 1.0))
        # 连续无可用 frontier 的触发时间（秒）；同时要求 access_mode==1 才真正回溯
        self.no_frontier_trigger_sec = cfg_file.get(
            'no_frontier_trigger_sec', rospy.get_param('~no_frontier_trigger_sec', 3.0))
        # commit 固定目标距离（沿强制方向一次性前推，不再动态 carrot）
        self.commit_goal_dist = cfg_file.get('commit_goal_dist',
                                             cfg_file.get('commit_carrot_lookahead',
                                                          rospy.get_param('~commit_carrot_lookahead', 5.0)))
        # 路口/分支限制
        self.max_pending_per_node = cfg_file.get('max_pending_per_node',
                                                 rospy.get_param('~max_pending_per_node', 2))
        self.junction_cooldown_anchors = cfg_file.get('junction_cooldown_anchors',
                                                      rospy.get_param('~junction_cooldown_anchors', 4))
        # 路口角度调试日志：开启后记录路口会话内每帧 frontier 角度
        self.junction_angle_log = cfg_file.get('junction_angle_log',
                                               rospy.get_param('~junction_angle_log', False))
        self.junction_angle_dir = cfg_file.get('junction_angle_dir') or os.path.join(self.topo_tree_root, "angle")
        self.junction_angle_counter = 0
        # topo tree 保存周期（文件名/目录已在 load_access_topo_config 冻结）
        self.topo_save_period = cfg_file.get('topo_save_period', rospy.get_param('~topo_save_period', 2.0))

        # 衍生参数（角度转弧度等）
        self.force_window_rad = np.deg2rad(self.force_window_deg)
        self.cluster_eps_rad = np.deg2rad(self.cluster_eps_deg)
        self.junction_peak_min_sep_rad = np.deg2rad(getattr(self, "junction_peak_min_sep_deg", 30.0))
        self.junction_decision_dist = getattr(self, "junction_decision_dist", self.anchor_step_dist)
        self.junction_same_dir_gate_rad = np.deg2rad(getattr(self, "junction_same_dir_gate_deg", 30.0))
        self.junction_finalize_dist = getattr(self, "junction_finalize_dist", self.junction_decision_dist * 1.5)
        self.junction_finalize_timeout = getattr(self, "junction_finalize_timeout", 6.0)

    # -------- Access-Topo: anchors / branches / backtrack --------

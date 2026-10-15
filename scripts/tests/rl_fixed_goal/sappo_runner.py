"""ROS/MPI runtime for the isolated SA-PPO fixed-goal controller test."""

from collections import deque
import os
import sys
from pathlib import Path

from mpi4py import MPI
import numpy as np
import rospy
import torch
from std_msgs.msg import String

from sappo_grid_guard import GridSafetyGuard
from sappo_mppi_guard import MppiSafetyGuard
from sappo_safety_primitives import ActionShaper, DwaSafetyGuard, StabilizedRecovery


OBS_SIZE = 512
LASER_HISTORY = 3
ACTION_BOUND = [[0.0, -0.5], [0.8, 0.5]]

# This test runner lives outside rl_navigation, unlike sappo_pure.py.
RL_NAVIGATION_DIR = Path(__file__).resolve().parents[3] / "rl_navigation"
sys.path.insert(0, str(RL_NAVIGATION_DIR))

from model.ppo import generate_action_no_sampling
from model.sa_pe import MLPPolicy
from rl_env import StageWorld


def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    env = StageWorld(OBS_SIZE, index=rank, num_env=1)
    controller_mode = str(rospy.get_param(
        "~controller_mode", rospy.get_param("~recovery_mode", "policy_only")
    )).strip().lower()
    if controller_mode not in ("policy_only", "stabilized_recovery", "rl_dwa_guard", "rl_mppi_guard", "rl_grid_guard"):
        raise ValueError("controller_mode must be policy_only, stabilized_recovery, rl_dwa_guard, rl_mppi_guard, or rl_grid_guard")
    recovery = StabilizedRecovery() if controller_mode == "stabilized_recovery" else None
    dwa_guard = DwaSafetyGuard() if controller_mode == "rl_dwa_guard" else None
    mppi_guard = MppiSafetyGuard() if controller_mode == "rl_mppi_guard" else None
    grid_guard = GridSafetyGuard() if controller_mode == "rl_grid_guard" else None
    controller_status = rospy.Publisher("/lste/sappo_controller_status", String, queue_size=1)
    action_shaper = ActionShaper()
    last_controller_signature = None
    last_applied_action = np.zeros(2, dtype=np.float32)

    if rank == 0:
        policy = MLPPolicy(obs_space=OBS_SIZE, action_space=2).cuda()
        checkpoint = os.path.join("policy", "sa_peppo_1650.pth")
        policy.load_state_dict(torch.load(checkpoint, map_location="cuda"))
        policy.eval()
        rospy.loginfo("SA-PPO controller mode=%s checkpoint=%s", controller_mode, checkpoint)
    else:
        policy = None

    while not env.has_goal() and not rospy.is_shutdown():
        env.control_rl_vel([0.0, 0.0])
        rospy.sleep(0.1)

    observation = env.get_laser_observation()
    observation_stack = deque([observation, observation, observation])
    state = [observation_stack, np.asarray(env.get_local_goal()), np.asarray(env.get_self_speed())]
    diagnostic_last_time = -float("inf")

    while not rospy.is_shutdown():
        state_list = comm.gather(state, root=0)
        raw_mean, scaled_action = generate_action_no_sampling(
            env=env, state_list=state_list, policy=policy, action_bound=ACTION_BOUND
        )
        action = comm.scatter(scaled_action, root=0)
        local_goal = env.get_local_goal()
        policy_action = np.asarray(action, dtype=np.float32).copy()
        controller_source = "policy"
        controller_reason = "policy_only"
        predicted_clearance = float("nan")

        # ``env.terminate`` is raised by the legacy RL environment whenever a
        # short goal-radius is reached.  In the normal LSTE pipeline that is a
        # frontier/visual handoff, not task completion; hold briefly at the
        # waypoint until Goal Manager publishes the next segment.  A fixed-goal
        # test has ``allow_intermediate_goals`` disabled and retains the old
        # terminal behavior.
        intermediate_handoff = (
            env.allow_intermediate_goals
            and env.terminate
            and not env.task_done
        )
        if not env.has_goal():
            action = [0.0, 0.0]
            controller_source = "stop"
            controller_reason = "goal_unavailable"
        elif env.terminate and not intermediate_handoff:
            action = [0.0, 0.0]
            controller_source = "stop"
            controller_reason = "task_done_or_complete"
        elif intermediate_handoff:
            action = [0.0, 0.0]
            controller_source = "handoff"
            controller_reason = "intermediate_waypoint_reached_wait_next_goal"
        elif mppi_guard is not None:
            action, guarded, controller_reason, predicted_clearance = mppi_guard.choose(
                raw_mean[0], policy_action, local_goal, env.scan,
                getattr(env, "scan_param", None), env.get_self_state(),
            )
            controller_source = "mppi_guard" if guarded else "policy"
        elif grid_guard is not None:
            action, guarded, controller_reason, predicted_clearance = grid_guard.choose(
                raw_mean[0], policy_action, local_goal, env.scan,
                getattr(env, "scan_param", None), env.get_self_state(),
            )
            controller_source = "grid_guard" if guarded else "policy"
        elif dwa_guard is not None:
            action, guarded, controller_reason, predicted_clearance = dwa_guard.choose(
                raw_mean[0], policy_action, local_goal, env.scan,
                getattr(env, "scan_param", None), env.get_self_state(),
            )
            controller_source = "dwa_guard" if guarded else "policy"
        elif recovery is not None:
            recovery_action = recovery.action(local_goal)
            if recovery_action is not None:
                action = recovery_action
                controller_source = "turn_recovery"
                controller_reason = "heading_behind_robot"

        requested_action = np.asarray(action, dtype=np.float32).reshape(-1)
        hard_stop = (
            controller_source == "stop"
            or "emergency_stop" in controller_reason
            or "no_safe" in controller_reason
        )
        # Ordinary goal-facing turns are allowed to decelerate through the
        # action slew limit.  Only a boundary-end turn or a constrained edge
        # clears linear velocity immediately; this prevents the repeated
        # full-stop/full-speed pattern seen at corridor corners while keeping
        # the close-obstacle safety behavior intact.
        immediate_turn_stop = (
            controller_source == "grid_guard"
            and (
                "boundary_blocked" in controller_reason
                or "turn_around_boundary" in controller_reason
                or "turn_before_constrained" in controller_reason
                or "no_safe" in controller_reason
            )
        )
        action = action_shaper.apply(
            requested_action,
            hard_stop=hard_stop,
            immediate_turn_stop=immediate_turn_stop,
        )
        status_reason = controller_reason
        if grid_guard is not None and controller_source == "grid_guard":
            status_reason = "%s %s" % (status_reason, grid_guard.diagnostic())
        signature = (controller_source, controller_reason)
        applied_array = np.asarray(action, dtype=np.float32)
        linear_drop = float(last_applied_action[0] - applied_array[0])
        if signature != last_controller_signature:
            rospy.loginfo(
                "RL_SAFETY_STATE source=%s reason=%s requested=(%.3f,%.3f) "
                "applied=(%.3f,%.3f) linear_drop=%.3f hard_stop=%s",
                controller_source,
                controller_reason,
                requested_action[0],
                requested_action[1],
                applied_array[0],
                applied_array[1],
                linear_drop,
                hard_stop,
            )
            last_controller_signature = signature
        if hard_stop or linear_drop >= 0.08:
            finite_scan = (
                np.asarray(env.scan, dtype=np.float32)
                if env.scan is not None else np.array([], dtype=np.float32)
            )
            finite_scan = finite_scan[np.isfinite(finite_scan)]
            min_scan = float(finite_scan.min()) if finite_scan.size else float("nan")
            rospy.logwarn(
                "RL_BRAKE_EVENT source=%s reason=%s requested_v=%.3f applied_v=%.3f "
                "linear_drop=%.3f min_scan=%.3f hard_stop=%s",
                controller_source,
                controller_reason,
                requested_action[0],
                applied_array[0],
                linear_drop,
                min_scan,
                hard_stop,
            )
        last_applied_action = applied_array
        controller_status.publish(String(
            data=("source=%s reason=%s requested=(%.3f,%.3f) action=(%.3f,%.3f) "
                  "clearance=%.3f" % (
                controller_source, status_reason,
                requested_action[0], requested_action[1], action[0], action[1],
                predicted_clearance
            ))
        ))

        now = rospy.Time.now().to_sec()
        if rank == 0 and now - diagnostic_last_time >= 1.0:
            raw_action = np.asarray(raw_mean[0], dtype=np.float32)
            scan = np.asarray(env.scan, dtype=np.float32) if env.scan is not None else np.array([])
            finite_scan = scan[np.isfinite(scan)]
            min_range = float(finite_scan.min()) if finite_scan.size else float("nan")
            rospy.loginfo(
                "RL_TEST_DIAG local_goal=(%.2f,%.2f) raw_mean=(%.3f,%.3f) "
                "clipped=(%.3f,%.3f) requested=(%.3f,%.3f) applied=(%.3f,%.3f) source=%s reason=%s "
                "predicted_clearance=%.3f min_scan=%.3f terminate=%s",
                local_goal[0], local_goal[1], raw_action[0], raw_action[1],
                policy_action[0], policy_action[1], requested_action[0], requested_action[1],
                action[0], action[1], controller_source, status_reason,
                predicted_clearance, min_range, env.terminate,
            )
            diagnostic_last_time = now

        env.control_rl_vel(action)
        rospy.sleep(0.1)
        env.get_reward_and_terminate(0)

        next_observation = env.get_laser_observation()
        observation_stack.popleft()
        observation_stack.append(next_observation)
        state = [
            observation_stack,
            np.asarray(env.get_local_goal()),
            np.asarray(env.get_self_speed()),
        ]


if __name__ == "__main__":
    main()

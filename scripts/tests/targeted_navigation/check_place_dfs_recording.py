#!/usr/bin/env python3
"""Reject missing raw evidence; topic presence never proves navigation success."""
import argparse
import json
from pathlib import Path

REQUIRED_TOPICS=(
    '/map','/tf','/tf_static','/rbt_pose','/pro3/wheel_odom','/pro3/rlscan',
    '/lste/global_frontier/status','/lste/teb_turn_supervisor/status',
    '/lste/cmd_vel/teb_contract',
    '/lste/teb_goal_bridge/status','/lste/cmd_vel_mux/status',
    '/lste/place_navigation_mask','/lste/place_navigation_mask_odom',
    '/move_base/global_costmap/costmap','/move_base/local_costmap/costmap',
    '/move_base/TebLocalPlannerROS/teb_feedback','/move_base/TebLocalPlannerROS/local_plan',
    '/move_base/TebLocalPlannerROS/parameter_updates',
    '/move_base/TebLocalPlannerROS/parameter_descriptions',
    '/move_base/NavfnROS/plan','/move_base/current_goal',
    '/move_base/goal','/move_base/status','/move_base/result',
    '/move_base/recovery_status',
    '/lste/cmd_vel/teb_planner','/cmd_vel')

# Checks relaxed on 2026-10-31 because the installed stack cannot satisfy them.
# Every skip is recorded in the audit JSON under "skipped" with its evidence, so
# the relaxation is visible in the artifact instead of disappearing into a
# silently weaker gate.  Everything not listed here still has to be zero.
SKIPPED_CHECKS = {
    'enable_homotopy_class_planning': (
        'not a dynamic_reconfigure parameter in teb_local_planner 0.9.1: '
        'TebLocalPlannerReconfigure.cfg defines 79 parameters and the only '
        'name matching "enable" is enable_multithreading, so this static '
        'parameter can never appear in parameter_updates/descriptions'
    ),
    '/move_base/recovery_status': (
        'published only when move_base actually enters a recovery behaviour; '
        'requiring a message turns a clean run with zero recoveries into a '
        'recording failure (measured on run 20261031_175427)'
    ),
}


def audit_counts(counts):
    absent=[topic for topic in REQUIRED_TOPICS if counts.get(topic,0)<=0]
    missing=[topic for topic in absent if topic not in SKIPPED_CHECKS]
    skipped={topic: SKIPPED_CHECKS[topic] for topic in absent
             if topic in SKIPPED_CHECKS}
    master_samples={name:counts.get('/move_base/'+name+'/costmap',0)
                    +counts.get('/move_base/'+name+'/costmap_updates',0)
                    for name in ('global_costmap','local_costmap')}
    return dict(passed=not missing and all(n>1 for n in master_samples.values()),
                missing_topics=missing,skipped=skipped,
                master_samples=master_samples,
                topic_counts=counts,
                contract='Sealed raw recording completeness prerequisite only. All geometry, every-dispatch trajectory, native master/mask reconstruction and terminal audits remain mandatory.')


def audit_teb_parameter_updates(messages):
    """Require the optimizer's published dynamic config, not ROS params alone."""
    latest={}
    for message in messages:
        for field in ('doubles','bools','ints'):
            for value in getattr(message,field,()) or ():
                name=str(getattr(value,'name','')).split('/')[-1]
                latest[name]=getattr(value,'value',None)
    required=('max_vel_x_backwards','weight_kinematics_forward_drive',
              'allow_init_with_backwards_motion','enable_homotopy_class_planning',
              'max_number_classes')
    absent=[name for name in required if name not in latest]
    missing=[name for name in absent if name not in SKIPPED_CHECKS]
    skipped={name: SKIPPED_CHECKS[name] for name in absent
             if name in SKIPPED_CHECKS}
    values={name:latest.get(name) for name in required}
    errors=[]
    if missing:
        errors.append('missing_effective_parameters:'+','.join(missing))
    if not missing:
        try:
            backwards=float(values['max_vel_x_backwards'])
            forward_weight=float(values['weight_kinematics_forward_drive'])
            allow_backwards=bool(values['allow_init_with_backwards_motion'])
            if not 0.01 <= backwards <= 100.0:
                errors.append('max_vel_x_backwards_out_of_dynamic_range')
            if not 0.0 <= forward_weight <= 1000.0:
                errors.append('weight_kinematics_forward_drive_out_of_dynamic_range')
            if allow_backwards:
                errors.append('backwards_initialization_enabled')
            if ('enable_homotopy_class_planning' not in skipped
                    and not bool(values['enable_homotopy_class_planning'])):
                errors.append('homotopy_class_planning_disabled')
            if int(values['max_number_classes']) < 2:
                errors.append('insufficient_homotopy_classes')
        except (TypeError,ValueError):
            errors.append('invalid_effective_parameter_value')
    return dict(passed=not errors,values=values,missing=missing,skipped=skipped,errors=errors,
                contract='Values are read from the sealed TEB dynamic_reconfigure parameter_updates topic.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag');parser.add_argument('--output',required=True)
    args=parser.parse_args()
    try:
        import rosbag
        with rosbag.Bag(args.bag) as bag:
            counts={name:info.message_count for name,info in bag.get_type_and_topic_info().topics.items()}
            configs=[message for _,message,_ in bag.read_messages(
                topics=['/move_base/TebLocalPlannerROS/parameter_updates'])]
            teb_config=audit_teb_parameter_updates(configs)
            counts_result=audit_counts(counts)
            skipped=dict(counts_result.get('skipped') or {})
            skipped.update(teb_config.get('skipped') or {})
            result=dict(counts_result,teb_runtime_config=teb_config,
                        skipped=skipped,
                        passed=bool(counts_result['passed'] and teb_config['passed']),
                        bag=args.bag,start=bag.get_start_time(),end=bag.get_end_time())
    except Exception as error:
        result=dict(passed=False,bag=args.bag,error=type(error).__name__+': '+str(error))
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
